"""Plot one completed mechanics case from closed CSV/contact output.

The default right column is the original continuous straight recovery.
--policy realign selects an actually recorded replay recovery; absent motion is
shown as unexecuted. --contacts adds measured XZ contact locations at the
reference sample with maximum total normal load, without inventing a peg mesh.
"""
import argparse
from collections import deque
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path


COLORS = dict(actual='#176a9c', command='#383e48', mean='#ce5a1b', normal='#8265a6',
              friction='#c2612a', stop='#d9dde2', realign='#e7c4a3', retreat='#c6e0ed', clear_hold='#d4e3d1')


def _csv(path):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'Empty completed profile: {path}')
    return rows


def _numbers(rows, key):
    return [float(row[key]) if row.get(key) not in ('', None) else math.nan for row in rows]


def rolling_mean(rows, key, dt):
    """Full 100 ms sample windows, reset at phase changes and clock gaps."""
    count = max(1, math.ceil(.1/dt-1e-9))
    window = deque(maxlen=count)
    previous = phase = None
    result = []
    for row in rows:
        time = float(row.get('recovery_time_s', row['time_s']))
        value = float(row[key]) if row.get(key) not in ('', None) else math.nan
        if (phase != row['phase'] or previous is not None and not math.isclose(time-previous, dt, abs_tol=1e-7)
                or not math.isfinite(value)):
            window.clear()
        if math.isfinite(value) and not ('recovery_time_s' in row and time == 0.):
            window.append(value)
        result.append(math.fsum(window)/count if len(window) == count else math.nan)
        previous, phase = time, row['phase']
    return result


def _phases(ax, rows, *, labels=False):
    times = _numbers(rows, 'recovery_time_s')
    start = 0
    for i in range(1, len(rows)+1):
        if i < len(rows) and rows[i]['phase'] == rows[start]['phase']:
            continue
        phase = rows[start]['phase']
        left = times[start] if start == 0 else (times[start-1]+times[start])/2
        right = times[-1] if i == len(rows) else (times[i-1]+times[i])/2
        ax.axvspan(left, right, color=COLORS.get(phase, '#eeeeee'), alpha=.48, zorder=0)
        if labels and right > left:
            label = {'stop':'STOP', 'realign':'RECENTER + ALIGN', 'retreat':'RETREAT', 'clear_hold':'CLEAR HOLD'}.get(phase, phase.upper())
            ax.text((left+right)/2, .95, label, ha='center', va='top', transform=ax.get_xaxis_transform(), fontsize=8,
                    color='#4c5765')
        start = i


def _markers(ax, metrics):
    for key, style, color in [('depth_gate_time_s', '--', '#8b693a'), ('tilt_ramp_complete_time_s', ':', '#7e527e')]:
        value = metrics.get(key)
        if isinstance(value, (int, float)):
            ax.axvline(value, ls=style, color=color, lw=1.1, zorder=1)


def _contact_sample(path, index):
    with gzip.open(path, 'rt') as stream:
        for i, line in enumerate(stream):
            if i == index:
                return json.loads(line)
    raise ValueError(f'Contact stream has no sample {index}: {path}')


def _contact_panel(ax, record, reference_row, case):
    import numpy as np
    snapshot = record['contacts']
    if not math.isclose(float(record['time_s']), float(reference_row['time_s']), abs_tol=1e-8):
        raise ValueError('Peak normal-load sample does not match contact timestamp')
    radius = case['hole_diameter_mm']/2
    points = snapshot.get('normal_contacts', [])
    ax.plot([-radius, -radius], [0, 24], color='#7e8792', lw=2)
    ax.plot([radius, radius], [0, 24], color='#7e8792', lw=2)
    ax.plot([-radius, radius], [0, 0], color='#7e8792', lw=2)
    ax.axhline(25, color='#647787', lw=1.3, label='Mouth: z = 25 mm')
    ax.axhline(24, color='#9a9fa6', lw=1, ls='--', label='Straight-wall top: 24 mm')
    if points:
        xs = np.array([p['point_socket_m'][0]*1000 for p in points])
        zs = np.array([p['point_socket_m'][2]*1000 for p in points])
        loads = np.array([p['normal_load_n'] for p in points])
        maximum = float(max(loads))
        colors = ['#bb6139' if p.get('wall_side') == 'pos' else '#23719c' if p.get('wall_side') == 'neg'
                  else '#6c7582' for p in points]
        ax.scatter(xs, zs, s=12+85*loads/maximum, c=colors, alpha=.8, edgecolors='white', linewidths=.45, zorder=4)
        # Vectors are a load-scaled direction cue, not displacement/penetration.
        directions = np.array([p['normal_socket'] for p in points])
        lengths = 1.6*loads/maximum
        ax.quiver(xs, zs, directions[:, 0]*lengths, directions[:, 2]*lengths, color=colors,
                  angles='xy', scale_units='xy', scale=1, width=.008, headwidth=4, zorder=5)
        note = f'{len(points)} exported loaded points\nLargest point load: {maximum:.3f} N'
    else:
        note = 'No exported loaded contacts at this sample'
    ax.set_xlim(-radius-2, radius+2)
    ax.set_ylim(-1, 27)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('Socket-local X (mm)')
    ax.set_ylabel('Socket-local Z (mm)')
    ax.set_title('Reference contact locations\n'+f"max ΣN = {float(reference_row['normal_load_n']):.3f} N at {float(reference_row['time_s']):.3f} s",
                 fontsize=10.5, loc='left', pad=12)
    ax.legend(loc='lower center', bbox_to_anchor=(.5, -.17), fontsize=8, frameon=False)
    ax.text(0, -.245, note+'\n\nXZ projection; Y is omitted.\nMarker area and arrow length\nscale with per-point normal load.\nArrows indicate force direction.\n\nBore guide: x = ±'+f'{radius:.3f}'+' mm.\nPolygon facets are not drawn.\nNo peg mesh or deformation inferred.',
            transform=ax.transAxes, ha='left', va='top', fontsize=8.3, linespacing=1.45)


def plot_case(study_dir, case_id, output_dir, *, policy='straight', contacts=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    source = Path(study_dir).resolve()
    destination = Path(output_dir).resolve()
    if destination == source or source in destination.parents:
        raise ValueError('Output directory must be outside the source study')
    manifest_bytes = (source/'study.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    if manifest.get('study') != 'Forge-mechanics-v1':
        raise ValueError('Expected Forge-mechanics-v1')
    attempts = [a for a in manifest['attempts'] if a.get('case_id') == case_id and a.get('status') == 'complete']
    if len(attempts) != 1:
        raise ValueError(f'Need exactly one completed attempt for {case_id}; found {len(attempts)}')
    case = attempts[0]
    folder = source/case['folder']
    metrics = case['metrics']
    reference_path = folder/'insertion.csv'
    reference = _csv(reference_path)
    recovery = case['final_retreat'] if policy == 'straight' else next((p for p in case.get('probes', []) if p.get('policy') == 'realign'), {})
    recovery_path = folder/('final_retreat.csv' if policy == 'straight' else 'terminal_realign_recovery.csv')
    recovered = _csv(recovery_path) if recovery_path.is_file() else []
    retreat_executed = any(row.get('phase') == 'retreat' for row in recovered)
    dt = 1/manifest['physics_hz']
    sources = [reference_path, folder/'trajectory.json'] + ([recovery_path] if recovered else [])
    profile_name = f'{case_id}'+('_realign' if policy == 'realign' else '')+'_mechanics_profile'
    with plt.rc_context({'font.family':'DejaVu Sans', 'font.size':10, 'pdf.fonttype':42,
                         'axes.spines.top':False, 'axes.spines.right':False}):
        fig = plt.figure(figsize=(17.2 if contacts else 13.2, 12.1))
        grid = fig.add_gridspec(4, 3 if contacts else 2, width_ratios=[1, 1, .6] if contacts else [1, 1],
                               left=.065 if contacts else .085, right=.965, bottom=.17, top=.84, hspace=.37, wspace=.33)
        left = [fig.add_subplot(grid[i, 0]) for i in range(4)]
        right = [fig.add_subplot(grid[i, 1]) for i in range(4)]
        time = _numbers(reference, 'time_s')
        left[0].plot(time, _numbers(reference, 'command_depth_mm'), color=COLORS['command'], ls='--', lw=1.1, label='Command')
        left[0].plot(time, _numbers(reference, 'depth_mm'), color=COLORS['actual'], lw=1.5, label='Actual')
        left[0].set_ylabel('Insertion depth (mm)')
        left[0].set_title('Original insertion reference', loc='left', fontsize=12, pad=13)
        left[0].legend(loc='upper left', fontsize=8, ncol=2, frameon=False)
        left[1].plot(time, _numbers(reference, 'command_tilt_deg'), color=COLORS['command'], ls='--', lw=1.1, label='Command')
        left[1].plot(time, _numbers(reference, 'tilt_deg'), color=COLORS['actual'], lw=1.5, label='Actual')
        left[1].set_ylabel('Tilt (deg)')
        left[2].plot(time, _numbers(reference, 'wrist_force_n'), color=COLORS['actual'], lw=.85, alpha=.6, label='Raw')
        left[2].plot(time, rolling_mean(reference, 'wrist_force_n', dt), color=COLORS['mean'], lw=1.6, label='100 ms mean')
        left[2].set_ylabel('Wrist force norm (N)')
        left[2].legend(loc='upper left', fontsize=8, ncol=2, frameon=False)
        left[3].plot(time, _numbers(reference, 'normal_load_n'), color=COLORS['normal'], lw=1.15)
        left[3].set_ylabel('Σ contact normal load (N)')
        for ax in left:
            _markers(ax, metrics)
            ax.set_xlim(time[0], time[-1])
            ax.grid(alpha=.16)
            ax.set_xlabel('Reference time (s)')
        reason = recovery.get('reason', 'recovery_not_recorded')
        right[0].set_title('Original continuous straight recovery' if policy == 'straight' else 'Replayed recenter + align recovery',
                           loc='left', fontsize=12, pad=13)
        if retreat_executed:
            rt = _numbers(recovered, 'recovery_time_s')
            start_depth = float(recovered[0]['depth_mm'])
            actual = [start_depth-float(row['depth_mm']) for row in recovered]
            command = _numbers(recovered, 'command_withdrawal_mm')
            if not any(math.isfinite(x) for x in command):
                command = [0. if i == 0 else start_depth-float(row['command_depth_mm']) for i, row in enumerate(recovered)]
            right[0].plot(rt, command, color=COLORS['command'], ls='--', lw=1.1, label='Command')
            right[0].plot(rt, actual, color=COLORS['actual'], lw=1.5, label='Actual')
            right[0].set_ylabel('Withdrawal from endpoint (mm)')
            right[0].legend(loc='lower right', fontsize=8, ncol=2, frameon=False)
            right[1].plot(rt, _numbers(recovered, 'wrist_force_n'), color=COLORS['actual'], alpha=.6, lw=.9, label='Raw')
            right[1].plot(rt, rolling_mean(recovered, 'wrist_force_n', dt), color=COLORS['mean'], lw=1.6, label='100 ms mean')
            right[1].set_ylabel('Wrist force norm (N)')
            right[1].legend(loc='upper right', fontsize=8, ncol=2, frameon=False)
            for kind in ('normal', 'friction'):
                values = [max(0., -value) if math.isfinite(value) else math.nan for value in _numbers(recovered, f'{kind}_force_world_z_n')]
                right[2].plot(rt, values, color=COLORS[kind], lw=1.1, label=kind.capitalize())
            right[2].set_ylabel('Opposing axial component (N)')
            right[2].legend(loc='upper right', fontsize=8, ncol=2, frameon=False)
            right[3].plot(rt, _numbers(recovered, 'normal_load_n'), color=COLORS['normal'], lw=1.15)
            right[3].set_ylabel('Σ contact normal load (N)')
            for i, ax in enumerate(right):
                _phases(ax, recovered, labels=i == 0)
                ax.set_xlim(0., rt[-1])
                ax.grid(alpha=.16)
                ax.set_xlabel('Recovery time (s)')
        else:
            for i, ax in enumerate(right):
                ax.set_xticks([]); ax.set_yticks([])
                ax.set_facecolor('#f5f6f7')
                message = 'Withdrawal not executed\n'+reason.replace('_', ' ')
                if i == 0 and isinstance(metrics.get('max_grasp_slip_mm'), (int, float)):
                    message += f"\nReference max grasp slip: {metrics['max_grasp_slip_mm']:.4f} mm"
                if i == 1 and recovered:
                    message += f"\nInitial STOP wrist sample: {float(recovered[0]['wrist_force_n']):.3f} N\nThis is not a withdrawal-force measurement."
                if i == 2:
                    message += '\nNo retreat contact-resistance curve.'
                if i == 3:
                    message += '\nNo retreat normal-load curve.'
                ax.text(.5, .5, message, ha='center', va='center', transform=ax.transAxes, fontsize=10,
                        linespacing=1.5, color='#64717d')
        contact_index = None
        if contacts:
            contact_index = max(range(len(reference)), key=lambda index: float(reference[index]['normal_load_n']))
            path = folder/'insertion_contacts.jsonl.gz'
            sources.append(path)
            _contact_panel(fig.add_subplot(grid[:3, 2]), _contact_sample(path, contact_index), reference[contact_index], case)
        ref_reason = metrics.get('termination_reason', metrics.get('reference_termination_reason', 'unknown'))
        numerical = 'valid' if metrics.get('numerically_valid') is True else 'invalid / unknown'
        fig.suptitle(f"Depth {case['target_depth_mm']:g} mm  |  friction (μs, μd) = ({case['pair_static_friction']:g}, {case['pair_dynamic_friction']:g})"
                     f"  |  target tilt {case['tilt_amplitude_deg']:g}°", x=.065 if contacts else .085, ha='left', fontsize=17, y=.976)
        fig.text(.065 if contacts else .085, .925, f"{case_id}  ·  {manifest['physics_hz']:g} Hz  ·  reference: {ref_reason}  ·  numerical screening: {numerical}", fontsize=10)
        handles = [Line2D([0], [0], ls='--', color='#8b693a', label='Actual-depth gate / tilt start'),
                   Line2D([0], [0], ls=':', color='#7e527e', label='Commanded tilt ramp complete')]
        fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(.06 if contacts else .08, .91), ncol=2,
                   fontsize=9, frameon=False)
        other = next((p for p in case.get('probes', []) if p.get('policy') == 'realign'), {})
        realign_note = other.get('reason', 'not recorded')
        prefix_note = 'matched' if other.get('replay_matched') is True and other.get('replay_prefix_matched') is True else 'unmatched / untested'
        lines = [f"Recovery result: {reason}; recenter + align replay: {realign_note} ({prefix_note}). Recenter + align adjusts both XY and orientation.",
                 'Withdrawal zero is the reference endpoint depth; stop drift remains visible. Shading separates recorded recovery phases.',
                 'The 100 ms mean uses complete windows within each phase. Stop peaks are not retreat peaks. Component resistance = max(0, −world-Z force).',
                 'Normal load is the sum of contact-normal magnitudes. Contact-force components and wrist reaction are different measurements; separate component peaks must not be added.',
                 'One simulated run per condition; descriptive evidence only. Missing withdrawal curves are not zero-force results.']
        left_margin = .065 if contacts else .085
        for i, text in enumerate(lines):
            fig.text(left_margin, .12-i*.023, text, fontsize=8.3, color='#596572')
        destination.mkdir(parents=True, exist_ok=True)
        paths = []
        for suffix in ('png', 'pdf'):
            path = destination/f'{profile_name}.{suffix}'
            fig.savefig(path, dpi=180, facecolor='white', bbox_inches='tight', pad_inches=.18)
            paths.append(str(path))
        plt.close(fig)
    source_record = dict(source_study=str(source), source_manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
                         case_id=case_id, trajectory_id=case.get('trajectory_id'), policy=policy, retreat_executed=retreat_executed,
                         reference_contact_peak_sample_index=contact_index, attempt=case,
                         plot_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                         inputs=[dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for path in sources], artifacts=paths)
    (destination/f'{profile_name}.sources.json').write_text(json.dumps(source_record, indent=2, allow_nan=False)+'\n')
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir', type=Path, required=True)
    parser.add_argument('--case-id', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--policy', choices=('straight', 'realign'), default='straight')
    parser.add_argument('--contacts', action='store_true', help='Add observed XZ contact locations at the reference normal-load peak')
    args = parser.parse_args()
    for path in plot_case(args.study_dir, args.case_id, args.output_dir, policy=args.policy, contacts=args.contacts):
        print(path)


if __name__ == '__main__':
    main()
