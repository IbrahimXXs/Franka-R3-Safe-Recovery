"""Pure protocol and analysis helpers for the bounded FORGE identification pilot."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from research.contact_response import quat_increment, transport_wrench, table, write_json

AXES = {'x': 0, 'y': 1, 'roll': 3, 'pitch': 4}
WRENCH = ('fx', 'fy', 'fz', 'taux', 'tauy', 'tauz')
POSE = ('dx_m', 'dy_m', 'dz_m', 'droll_rad', 'dpitch_rad', 'dyaw_rad')
LENGTH_M = .01  # Explicit norm metric, not a material parameter.
W_SCALE = np.array([1., 1., 1., 1/LENGTH_M, 1/LENGTH_M, 1/LENGTH_M])
X_SCALE = np.array([1., 1., 1., LENGTH_M, LENGTH_M, LENGTH_M])


def schedule(repeats=2, seed=20260915):
    if type(repeats) is not int or not 2 <= repeats <= 3:
        raise ValueError('This pilot requires two or three repeats')
    rng = np.random.default_rng(seed)
    result = []
    for repeat in range(repeats):
        probes = [('hold', 0)] + [(a, s) for a in AXES for s in (-1, 1)]
        for order, i in enumerate(rng.permutation(len(probes))):
            axis, sign = probes[i]
            amplitude = 0. if axis == 'hold' else (.0001 if axis in ('x', 'y') else np.deg2rad(.2))
            result.append(dict(repeat=repeat, order=order, axis=axis, sign=sign,
                               amplitude_si=amplitude, probe_id=f'r{repeat}_{axis}_{sign:+d}'))
    return result


def waveform(t):
    """Seconds after preparation: before .25, outward .25, dwell .25, return .25, after .5."""
    def smooth(u):
        u = np.clip(u, 0., 1.)
        return float(u*u*(3-2*u))
    if t <= .25: return 'before', 0.
    if t <= .5: return 'outward', smooth((t-.25)/.25)
    if t <= .75: return 'during', 1.
    if t <= 1.: return 'return', 1.-smooth((t-.75)/.25)
    return 'after', 0.


def arrays(rows, anchor):
    pos = np.array([[r[f'peg_world_{a}_m'] for a in 'xyz'] for r in rows])
    q = np.array([[r[k] for k in ('qw', 'qx', 'qy', 'qz')] for r in rows])
    w = np.array([[r[k] for k in WRENCH] for r in rows])
    return pos, q, transport_wrench(pos, w, np.asarray(anchor))


def summarize(rows, anchor):
    """Tail means reduce transient bias; retain drift and within-hold variability separately."""
    pos, q, w = arrays(rows, anchor)
    times = np.array([r['probe_time_s'] for r in rows])
    masks = {}
    for phase in ('before', 'during', 'after'):
        mask = np.array([r['phase'] == phase for r in rows])
        if mask.sum() < 2: raise ValueError(f'Incomplete {phase} dwell')
        masks[phase] = mask & (times >= times[mask].max()-.125+1e-8)
    b, d, a = (masks[k] for k in ('before', 'during', 'after'))
    # All rotations expressed in the same spatial tangent at the first before sample.
    theta = quat_increment(q[np.flatnonzero(b)[0]], q)
    xi = np.column_stack((pos, theta))
    delta = xi[d].mean(0)-xi[b].mean(0)
    dw = w[d].mean(0)-w[b].mean(0)
    dw_after = w[a].mean(0)-w[b].mean(0)
    residual = xi[a].mean(0)-xi[b].mean(0)
    variability = max(float(np.sqrt(np.mean(np.sum(((w[m]-w[m].mean(0))*W_SCALE)**2, axis=1))))
                      for m in (b, d, a))
    out = dict(zip(POSE, delta.tolist()))
    out.update({f'delta_{k}': float(v) for k, v in zip(WRENCH, dw)})
    out.update({f'after_delta_{k}': float(v) for k, v in zip(WRENCH, dw_after)})
    out.update(response_n_equiv=float(np.linalg.norm(dw*W_SCALE)),
        force_response_n=float(np.linalg.norm(dw[:3])), torque_response_nm=float(np.linalg.norm(dw[3:])),
        variability_n_equiv=variability, return_position_error_mm=float(np.linalg.norm(residual[:3])*1000),
        return_orientation_error_deg=float(np.linalg.norm(residual[3:])*180/np.pi),
        before_depth_mm=float(np.mean([rows[i]['depth_mm'] for i in np.flatnonzero(b)])),
        before_normal_load_n=float(np.mean([rows[i]['normal_load_n'] for i in np.flatnonzero(b)])),
        after_normal_load_n=float(np.mean([rows[i]['normal_load_n'] for i in np.flatnonzero(a)])),
        after_response_n_equiv=float(np.linalg.norm(dw_after*W_SCALE)))
    return out


def central_difference(plus, minus, axis):
    """Actual-motion secant, not an independently identified partial derivative.

    A large off-axis span makes attribution to the named axis invalid. Ridge or
    command amplitudes cannot repair absent actual excitation.
    """
    index = AXES[axis]
    xp = np.array([plus[k] for k in POSE]); xm = np.array([minus[k] for k in POSE])
    span = xp-xm
    floor = 1e-6 if index < 3 else np.deg2rad(.002)
    out = dict(actual_span_si=float(span[index]), axis=axis)
    if xp[index] <= floor or xm[index] >= -floor:
        return {**out, 'valid': False, 'reason': 'actual_sign_or_motion_unresolved'}
    scaled = span*X_SCALE
    off_axis = np.linalg.norm(np.delete(scaled, index))/abs(scaled[index])
    out['off_axis_ratio'] = float(off_axis)
    # Still save the coupled secant when directional attribution fails.
    response = np.array([plus[f'delta_{k}']-minus[f'delta_{k}'] for k in WRENCH])
    out.update({f'd_{k}_per_si': float(v) for k, v in zip(WRENCH, response/span[index])})
    out.update(valid=bool(off_axis <= .5), reason='ok' if off_axis <= .5 else 'coupled_motion')
    return out


def cosine(a, b):
    denom = np.linalg.norm(a)*np.linalg.norm(b)
    return float(np.dot(a, b)/denom) if denom > 1e-12 else None


def analyze(directory):
    directory = Path(directory)
    manifest = json.loads((directory/'pilot.json').read_text())
    rows = []
    for case in manifest['cases']:
        for probe in case['probes']:
            row = {k:v for k,v in probe.items() if not isinstance(v, (dict, list))}
            row.update(case_id=case['case_id'], local_success=case['metrics']['insertion_success'],
                       local_stalled=case['metrics']['stalled'])
            path = directory/probe['log']
            with path.open() as f:
                samples = [{k:(v if k == 'phase' else (float(v == 'True') if v in ('True', 'False') else float(v))) for k,v in r.items()}
                           for r in csv.DictReader(f)]
            if probe['completed']:
                row.update(summarize(samples, case['wrench_anchor_world_m']))
            rows.append(row)
    for case in manifest['cases']:
        selected = [r for r in rows if r['case_id'] == case['case_id']]
        holds = [r for r in selected if r['axis'] == 'hold' and r['eligible'] and r['completed']]
        baseline = max([max(r['response_n_equiv'], r['variability_n_equiv']) for r in holds], default=None)
        for r in selected:
            r['baseline_valid_repeats'] = len(holds)
            r['baseline_n_equiv'] = baseline
            r['above_baseline'] = False
            if 'response_n_equiv' in r and baseline is not None:
                r['response_to_baseline'] = r['response_n_equiv']/max(baseline, 1e-4)
                r['above_baseline'] = bool(r['response_to_baseline'] >= 3 and len(holds) >= 2 and r['eligible'])
    central, repeatability, distinctions = [], [], []
    for case in manifest['cases']:
        selected = [r for r in rows if r['case_id'] == case['case_id']]
        for repeat in range(manifest['repeats']):
            for axis in AXES:
                pair = {r['sign']: r for r in selected if r['repeat']==repeat and r['axis']==axis}
                eligible = len(pair)==2 and all(r['eligible'] and r['completed'] for r in pair.values())
                c = central_difference(pair[1], pair[-1], axis) if eligible else dict(valid=False, reason='ineligible_pair', axis=axis)
                central.append(dict(case_id=case['case_id'], repeat=repeat, **c))
        for axis in AXES:
            for sign in (-1, 1):
                items = [r for r in selected if r['axis']==axis and r['sign']==sign and r['eligible'] and r['completed']]
                info = dict(case_id=case['case_id'], axis=axis, sign=sign, valid_repeats=len(items), passed=False)
                if len(items) == manifest['repeats']:
                    vectors = np.array([[r[f'delta_{k}'] for k in WRENCH] for r in items])*W_SCALE
                    mean = vectors.mean(0)
                    error = float(max(np.linalg.norm(v-mean) for v in vectors)/max(np.linalg.norm(mean), 1e-4))
                    cos = min(cosine(a,b) if cosine(a,b) is not None else -1.
                              for i,a in enumerate(vectors) for b in vectors[i+1:])
                    info.update(min_cosine=cos, relative_repeat_error=error,
                                passed=bool(cos >= .8 and error <= .5 and all(r['above_baseline'] for r in items)))
                repeatability.append(info)
        # Compare positive-axis response vectors in physical N-equivalent space.
        # Signs within an axis are not counted as distinct independent directions.
        for i, a in enumerate(AXES):
            for b in list(AXES)[i+1:]:
                groups = [[r for r in selected if r['axis']==axis and r['sign']==1 and r['eligible'] and r['completed']] for axis in (a,b)]
                info = dict(case_id=case['case_id'], axis_a=a, axis_b=b, passed=False)
                if all(len(g)==manifest['repeats'] for g in groups):
                    vs = [np.array([[r[f'delta_{k}'] for k in WRENCH] for r in g])*W_SCALE for g in groups]
                    means = [v.mean(0) for v in vs]
                    distance = float(np.linalg.norm(means[0]-means[1]))
                    radius = sum(max(np.linalg.norm(v-m) for v in group) for group,m in zip(vs,means))
                    noise = max(groups[0][0].get('baseline_n_equiv') or 0., 1e-4)
                    repeated = all(next(r['passed'] for r in repeatability if r['case_id']==case['case_id'] and r['axis']==axis and r['sign']==1) for axis in (a,b))
                    info.update(distance_n_equiv=distance, separation_threshold_n_equiv=float(radius+3*noise),
                                passed=bool(repeated and distance > radius+3*noise))
                distinctions.append(info)
    table(directory/'summary.csv', rows)
    table(directory/'central_differences.csv', central)
    table(directory/'repeatability.csv', repeatability)
    table(directory/'distinguishability.csv', distinctions)
    results = dict(attempts=len(rows), completed=sum(r['completed'] for r in rows), eligible=sum(r['eligible'] for r in rows),
                   baseline_repeats=sum(r['eligible'] and r['axis']=='hold' for r in rows),
                   signed_above_baseline=sum(r['above_baseline'] and r['axis']!='hold' for r in rows),
                   valid_central_pairs=sum(c['valid'] for c in central), repeatable_signed_directions=sum(r['passed'] for r in repeatability),
                   distinguishable_axis_pairs=sum(r['passed'] for r in distinctions))
    write_json(directory/'analysis.json', results)
    plots(directory, rows, repeatability)
    text = ['# Active micro-probing identification pilot', '',
        f"Status: **{manifest['status']}**. Native FORGE physics: {manifest['physics_hz']} Hz. Two selected terminal states; {manifest['repeats']} repeats per signed direction and hold.", '',
        '## Protocol and interpretation', '',
        'Each attempt resets the freely grasped Panda/FORGE scene with the same seed, replays the complete insertion prefix, and holds its final controller target for 0.75 s. The nominal target preserves insertion preload; it is not replaced with the measured pose. Both the prefix and paused state must match a local reference using unchanged FORGE replay tolerances. A mismatch prevents the perturbation. Local references are characterized again; archived outcomes are provenance, not assumed local labels.', '',
        'A probe holds for 0.25 s, ramps outward over 0.25 s, dwells for 0.25 s, ramps back over 0.25 s, and observes the return for 0.5 s. Commands are ±0.1 mm world X/Y or ±0.2 degrees spatial roll/pitch about the nominal peg base. Order is randomized within each repeat. The hold baseline has the identical schedule with zero command change. Every following attempt starts with a fresh replay, so return hysteresis cannot accumulate.', '',
        'All raw per-step contact and wrist wrenches, actual world peg position/quaternion, COM and peg-base velocities, depth, contact load, penetration, grasp slip, commands, and budget flags are saved. Contact torques are transported from the moving peg base to one fixed world anchor per case for analysis. Raw contact/wrist signals remain separate. Operational budgets use the existing raw wrist norms (20 N, 1 Nm), existing overlap screen and grasp-retention limits; no physics/material/controller parameters changed.', '',
        'Δw and actual Δξ use the final 0.125 s of before/plateau dwells. Return errors use the after dwell; a returned command does not establish restoration of contact memory. Return mismatch is reported separately; initial-state replay governs eligibility. Positive/negative differences divide by actual signed pose span, not requested amplitude. Missing opposite actual motion (<1 µm or <0.002 degrees) is rejected. Off-axis motion above 50% in the scaled pose metric prevents directional attribution; saved secants in such cases describe coupled paths only. No full 6D matrix is claimed.', '',
        'Force and torque are reported separately. Combined wrench norms use ||[F, τ/L]|| with L=10 mm (N-equivalent); pose coupling uses [translation, L·rotation]. The baseline is the larger of hold drift and within-dwell RMS variability, maximized across valid holds. A 0.0001 N-equivalent analysis floor prevents division by zero; it is not calibrated sensor noise.', '',
        'Predeclared exploratory criteria: at least two eligible holds; each signed response ≥3× baseline; repeated response vectors have cosine ≥0.8 and maximum deviation from their mean ≤50% of mean magnitude. Two positive-axis vectors are distinguishable when their distance exceeds the sum of repeat radii plus 3× baseline and both pass repeatability. These small-sample criteria are descriptive, not statistical confidence bounds or safety certification.', '',
        '## Local contact states', '', '| Case | Archived success / stalled | Local success / stalled | Max depth (mm) | Max normal load (N) |', '|---|---|---|---:|---:|']
    for c in manifest['cases']:
        m=c['metrics']; src=c['archived_metrics']
        text.append(f"| {c['case_id']} | {src['insertion_success']} / {src['stalled']} | {m['insertion_success']} / {m['stalled']} | {m['max_depth']:.3f} | {m['max_normal_load']:.3f} |")
    text += ['', '## Results', '', *[f'- {k.replace("_", " ")}: {v}' for k,v in results.items()], '',
        'All exclusions and safety stops remain in summary.csv; missing/ineligible pairs do not count as zero response. Check central_differences.csv for actual motion and coupling, and repeatability.csv / distinguishability.csv for the criteria above.', '',
        '**Decision:** '+('Some repeated directional signals exceed the hold baseline. This supports a larger identification/validation study; it does not yet justify constraint-aware control.' if results['repeatable_signed_directions'] else 'The pilot does not establish repeatable directional identification above the hold baseline. Resolve the reported replay, excitation, or variability limitations before constraint-aware control.'), '',
        'Only two contact states and two or three repeats are tested. Timestep convergence, generalization, contact memory, dynamic versus quasi-static response, and finite-amplitude nonlinearity remain unresolved. No recovery label, controller, corrective insertion action, or ML model is implemented.', '',
        '![Directional response and hold variability](direction_response.png)', '', '![Repeated signed responses](repeatability.png)', '', '![Actual return errors](return_errors.png)', '',
        'Re-run analysis without simulation: `python -m research.active_probing outputs/Active-Probing-Pilot-v1`.', '']
    (directory/'report.md').write_text('\n'.join(text))
    return results


def plots(directory, rows, repeatability):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cases = list(dict.fromkeys(r['case_id'] for r in rows))
    fig, axs = plt.subplots(len(cases), 1, figsize=(11, 4*len(cases)), squeeze=False)
    labels = ['hold']+[f'{a}{s:+d}' for a in AXES for s in (-1,1)]
    for ax, case in zip(axs[:,0], cases):
        for r in rows:
            if r['case_id']!=case or 'response_n_equiv' not in r: continue
            label = 'hold' if r['axis']=='hold' else f"{r['axis']}{r['sign']:+d}"
            ax.scatter(labels.index(label)+(r['repeat']-.5)*.12, r['response_n_equiv'],
                       marker='o' if r['eligible'] else 'x', color=f"C{r['repeat']}")
        bs = [r['baseline_n_equiv'] for r in rows if r['case_id']==case and r['baseline_n_equiv'] is not None]
        if bs: ax.axhline(3*max(bs), color='k', linestyle='--', label='3× hold baseline')
        ax.set(xticks=range(9), xticklabels=labels, ylabel='Wrench response (N-equivalent)', title=case)
        if bs: ax.legend()
    fig.tight_layout(); fig.savefig(directory/'direction_response.png', dpi=160); plt.close(fig)
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))
    for r in repeatability:
        if 'min_cosine' not in r: continue
        axs[0].scatter(r['min_cosine'], r['relative_repeat_error'], label=f"{r['case_id']} {r['axis']}{r['sign']:+d}")
    axs[0].axvline(.8,color='gray',linestyle='--'); axs[0].axhline(.5,color='gray',linestyle='--')
    axs[0].set(xlabel='Minimum repeat cosine',ylabel='Relative repeat error',title='Direction and magnitude consistency')
    for case in cases:
        for axis in AXES:
            for sign in (-1,1):
                rr = [r for r in rows if r['case_id']==case and r['axis']==axis and r['sign']==sign and r['eligible'] and 'response_n_equiv' in r]
                if len(rr)>=2: axs[1].scatter(rr[0]['response_n_equiv'],rr[1]['response_n_equiv'])
    axs[1].set(xlabel='Repeat 1 (N-equivalent)',ylabel='Repeat 2 (N-equivalent)',title='Eligible signed responses')
    fig.tight_layout(); fig.savefig(directory/'repeatability.png',dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8,5))
    for case in cases:
        rr = [r for r in rows if r['case_id']==case and 'return_position_error_mm' in r]
        ax.scatter([r['return_position_error_mm'] for r in rr],[r['return_orientation_error_deg'] for r in rr],label=case)
    ax.axvline(.005,color='gray',linestyle='--'); ax.axhline(.02,color='gray',linestyle='--')
    ax.set(xlabel='Actual return position error (mm)',ylabel='Actual return orientation error (deg)',title='Return hysteresis (before / after dwell means)'); ax.legend()
    fig.tight_layout(); fig.savefig(directory/'return_errors.png',dpi=160); plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    print(json.dumps(analyze(parser.parse_args().directory), indent=2))
