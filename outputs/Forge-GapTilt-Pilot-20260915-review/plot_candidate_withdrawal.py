"""Plot a completed inserted-stall candidate and its continuous withdrawal.

Optional --comparison-profile accepts another completed attempt directory
containing trajectory.json, insertion.csv and final_retreat.csv. Such a
comparison is an independent numerical diagnostic, not a matched policy pair.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


TEAL, NAVY, RUST, PURPLE = '#087F8C', '#253F63', '#C6532A', '#8661A3'


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def arr(rows, key):
    return np.array([float(row[key]) for row in rows])


def load_profile(folder):
    folder = folder.resolve()
    attempt = json.loads((folder / 'trajectory.json').read_text())
    if attempt['status'] != 'complete':
        raise ValueError(f'Only closed completed attempts are plotted: {folder}')
    all_ins = read_rows(folder / 'insertion.csv')
    ins = [r for r in all_ins if r['phase'] in ('insert', 'hold')]
    rec = read_rows(folder / 'final_retreat.csv')
    rt = arr(rec, 'recovery_time_s')
    dt = float(np.median(np.diff(rt)))
    stop_end = [r for r in rec if r['phase'] == 'stop'][-1]
    retreat = [r for r in rec if r['phase'] == 'retreat']
    peak = max(retreat, key=lambda r: float(r['wrist_force_n']))
    actual = float(stop_end['depth_mm']) - arr(rec, 'depth_mm')
    command = float(stop_end['command_depth_mm']) - arr(rec, 'command_depth_mm')
    peak_i = rec.index(peak)
    info = dict(folder=str(folder), case_id=attempt['case_id'], inferred_sample_hz=round(1/dt),
                initial_peg_position_mm=[float(all_ins[0][key]) for key in ('tip_x_mm', 'tip_y_mm', 'depth_mm')],
                insertion=attempt['metrics'], continuation_straight=attempt['final_retreat'],
                retreat_peak_time_s=float(peak['recovery_time_s']),
                commanded_withdrawal_at_peak_mm=float(command[peak_i]),
                actual_withdrawal_at_peak_mm=float(actual[peak_i]),
                source_sha256={name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
                               for name in ('trajectory.json', 'insertion.csv', 'final_retreat.csv')})
    return dict(attempt=attempt, ins=ins, rec=rec, rt=rt, dt=dt,
                stop_end_s=float(stop_end['recovery_time_s']), actual=actual, command=command,
                peak_time=float(peak['recovery_time_s']), peak_i=peak_i, info=info)


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=here.parent / 'Forge-GapTilt-Pilot-20260915' /
                        'gap_0200' / 'g0200_d30_pitch_pos20_try00')
    parser.add_argument('--comparison-profile', type=Path)
    parser.add_argument('--comparison-label')
    parser.add_argument('--output-dir', type=Path, default=here)
    parser.add_argument('--basename', default='candidate_withdrawal')
    args = parser.parse_args()
    primary = load_profile(args.profile)
    other = load_profile(args.comparison_profile) if args.comparison_profile else None
    a, ins, rec = primary['attempt'], primary['ins'], primary['rec']
    if not (a['metrics']['stalled'] and a['metrics']['numerically_valid'] and a['final_retreat']['safe_recovery']):
        raise ValueError('The candidate title requires a screened insertion stall followed by successful safe withdrawal')
    if other:
        for key in ('radial_clearance_mm', 'tilt_amplitude_deg', 'tilt_onset_mm', 'tilt_sign'):
            if a[key] != other['attempt'][key]:
                raise ValueError(f'Comparison physical case differs: {key}')
    primary_hz = primary['info']['inferred_sample_hz']
    other_hz = other['info']['inferred_sample_hz'] if other else None
    initial_position_difference_mm = (float(np.linalg.norm(np.array(primary['info']['initial_peg_position_mm']) -
                                          np.array(other['info']['initial_peg_position_mm']))) if other else None)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':10, 'axes.labelsize':10,
                         'axes.titlesize':12, 'axes.spines.top':False, 'axes.spines.right':False,
                         'pdf.fonttype':42, 'savefig.facecolor':'white'})
    fig, axes = plt.subplots(2, 2, figsize=(13.2, 9.1))
    fig.subplots_adjust(left=.075, right=.975, bottom=.16, top=.82, hspace=.43, wspace=.2)
    title = (f'Independent {primary_hz} / {other_hz} Hz runs show numerical sensitivity' if other
             else 'An inserted stall resists straight withdrawal, then releases')
    fig.suptitle(title, x=.075, y=.965, ha='left', fontsize=18 if other else 19,
                 weight='bold', color='#18384D')
    fig.text(.075, .919, f"{a['radial_clearance_mm']:g} mm radial clearance · {a['tilt_amplitude_deg']:g}° pitch starts at {a['tilt_onset_mm']:g} mm actual depth · " +
             ('separate completed references and continuous withdrawals' if other else 'original continuous withdrawal'),
             fontsize=11, color='#40586A')
    fig.text(.075, .881, ('The retreat peak rises while its 100 ms mean maximum falls; these runs do not establish timestep convergence.' if other
                         else 'Increasing pull initially produces very little peg motion. Force then drops as withdrawal accelerates; clearance succeeds.'),
             fontsize=10.5, color='#40586A')
    trigger = a['metrics']['tilt_trigger_time_s']
    t = arr(ins, 'time_s') - trigger
    complete = a['metrics']['tilt_completion_time_s'] - trigger
    label = f"{primary['info']['inferred_sample_hz']} Hz actual" if other else 'Actual'
    for ax in axes[0]:
        ax.axvspan(0, complete, color='#E5EFFA')
        ax.axvline(0, color='#90A9C1', lw=.7, ls=':')
        ax.axvline(complete, color='#90A9C1', lw=.7, ls=':')
        ax.set_xlim(-4.3, 4.85)
        ax.set_xticks([-4, -2, 0, 1, 2, 4])
        ax.set_xlabel('Insertion time relative to tilt trigger [s]')
    depth, tilt = axes[0]
    depth.plot(t, arr(ins, 'command_depth_mm'), color=NAVY, ls='--', lw=1.4, label=f'{primary_hz} Hz command' if other else 'Command')
    depth.plot(t, arr(ins, 'depth_mm'), color=TEAL, lw=1.6, label=label)
    depth.set(title='Independent insertion trajectories' if other else 'Insertion stops short of the 20 mm target', ylabel='Depth [mm]', ylim=(-11, 23))
    stall = next(cp for cp in a['checkpoints'] if cp['checkpoint_kind']=='first_stall' and cp['reached'])
    st = stall['state']['time_s'] - trigger; sd = stall['state']['depth_mm']
    depth.scatter([st], [sd], s=25, color=RUST, zorder=5)
    depth.annotate((f'{primary_hz} Hz: ' if other else '') + f"stall detected\nFinal depth: {a['metrics']['max_depth']:.2f} mm", xy=(st, sd), xytext=(-.5, -4.5),
                   fontsize=9, color='#80452C', arrowprops=dict(arrowstyle='->', color=RUST, lw=.8))
    tilt.plot(t, arr(ins, 'command_tilt_deg'), color=NAVY, ls='--', lw=1.4, label='Command (both runs)' if other else 'Command')
    tilt.plot(t, arr(ins, 'tilt_deg'), color=TEAL, lw=1.4, label=label)
    tilt.set(title='Actual tilt under the same nominal command' if other else 'The time-driven tilt ramp completes despite the stall', ylabel='Tilt magnitude [deg]', ylim=(-.08, 2.35))
    tilt.text(.03, .94, 'Blue band: 1 s command ramp', transform=tilt.transAxes, va='top', fontsize=9, color='#557493')
    rt = primary['rt']; force = axes[1, 0]; motion = axes[1, 1]
    peak_time = primary['peak_time']; peak_i = primary['peak_i']; m = a['final_retreat']
    end = max(rt[-1], other['rt'][-1] if other else 0.)
    for ax in axes[1]:
        ax.axvspan(0, primary['stop_end_s'], color='#FFF0D9')
        ax.axvspan(primary['stop_end_s'], end, color='#EDF5EE')
        ax.axvline(primary['stop_end_s'], color='#C5AB7A', ls=':', lw=.8)
        ax.axvline(peak_time, color=TEAL if other else PURPLE, ls=':', lw=.85)
        if other:ax.axvline(other['peak_time'], color=PURPLE, ls=':', lw=.85)
        ax.set_xlim(-.02, end+.1)
        ax.set_xlabel('Continuous recovery time [s]')
    force.plot(rt, arr(rec, 'wrist_force_n'), color=TEAL, lw=1.2,
               label=f"{primary['info']['inferred_sample_hz']} Hz wrist" if other else 'Raw wrist norm')
    if not other:
        force.plot(rt, np.maximum(0., -arr(rec, 'fz')), color=RUST, lw=1., ls='--', alpha=.8,
                   label='Opposing axial contact')
    force.set(title='Raw wrist force: peak and sustained load differ' if other else 'Retreat load rises while motion remains small', ylabel='Force [N]')
    max_force = max(m['max_wrist_force_n'], other['attempt']['final_retreat']['max_wrist_force_n'] if other else 0.)
    force.set_ylim(-.08, max_force*1.2)
    force.scatter([peak_time], [m['retreat_peak_wrist_force_n']], s=25, color=TEAL, zorder=6)
    if other:
        om = other['attempt']['final_retreat']
        force.text(.97, .95, 'Retreat peak / 100 ms mean max\n' +
                   f"{primary_hz} Hz: {m['retreat_peak_wrist_force_n']:.3f} / {m['retreat_peak_wrist_force_mean100ms_n']:.3f} N\n" +
                   f"{other_hz} Hz: {om['retreat_peak_wrist_force_n']:.3f} / {om['retreat_peak_wrist_force_mean100ms_n']:.3f} N",
                   transform=force.transAxes, ha='right', va='top', fontsize=9, color='#40586A',
                   bbox=dict(facecolor='white', edgecolor='none', alpha=.88, pad=3))
    else:
        force.annotate(f"Retreat peak: {m['retreat_peak_wrist_force_n']:.3f} N\n100 ms mean max: {m['retreat_peak_wrist_force_mean100ms_n']:.3f} N",
                       xy=(peak_time, m['retreat_peak_wrist_force_n']), xytext=(.96,.92), textcoords='axes fraction',
                       ha='right', va='top', fontsize=9, color='#26595F',
                       arrowprops=dict(arrowstyle='->', color=TEAL, lw=.8),
                       bbox=dict(facecolor='white', edgecolor='none', alpha=.86, pad=2))
        force.text(.04, .73, f"Stop peak:\n{m['stop_peak_wrist_force_n']:.3f} N", transform=force.transAxes,
                   fontsize=8.5, color='#916539')
    motion.plot(rt, primary['command'], color=NAVY, ls='--', lw=1.4, label=f'{primary_hz} Hz command' if other else 'Commanded withdrawal')
    motion.plot(rt, primary['actual'], color=TEAL, lw=1.6, label=f'{label} withdrawal')
    motion.scatter([peak_time, peak_time], [primary['actual'][peak_i], primary['command'][peak_i]],
                   s=23, color=[TEAL, NAVY], zorder=6)
    motion.set(title='Commanded and actual withdrawal in each run' if other else 'The peg releases and then follows the retreat', ylabel='Withdrawal from stop-end [mm]', ylim=(-.6, 19.2))
    motion.annotate((f'{primary_hz} Hz ' if other else '') + f"at {peak_time:.3f} s peak:\ncommand {primary['command'][peak_i]:.3f} mm\nactual {primary['actual'][peak_i]:.3f} mm",
                    xy=(peak_time, primary['actual'][peak_i]), xytext=(.03, .62), textcoords='axes fraction',
                    ha='left', va='top', fontsize=9, color='#26595F',
                    arrowprops=dict(arrowstyle='->', color=TEAL, lw=.8),
                    bbox=dict(facecolor='white', edgecolor='none', alpha=.8, pad=2))
    if other:
        comp = args.comparison_label or f"{other['info']['inferred_sample_hz']} Hz diagnostic"
        ot = arr(other['ins'], 'time_s') - other['attempt']['metrics']['tilt_trigger_time_s']
        depth.plot(ot, arr(other['ins'], 'command_depth_mm'), color=PURPLE, lw=.9, ls='--', alpha=.65, label=f'{other_hz} Hz command')
        depth.plot(ot, arr(other['ins'], 'depth_mm'), color=PURPLE, lw=1.2, ls='-.', label=comp)
        tilt.plot(ot, arr(other['ins'], 'tilt_deg'), color=PURPLE, lw=1.2, ls='-.', label=comp)
        force.plot(other['rt'], arr(other['rec'], 'wrist_force_n'), color=PURPLE, lw=1.2, ls='-.', label=comp)
        motion.plot(other['rt'], other['command'], color=PURPLE, lw=.9, ls='--', alpha=.65, label=f'{other_hz} Hz command')
        motion.plot(other['rt'], other['actual'], color=PURPLE, lw=1.2, ls='-.', label=comp)
    depth.legend(loc='upper left', fontsize=8, frameon=False)
    tilt.legend(loc='lower right', fontsize=8, frameon=False)
    force.legend(loc='lower right', fontsize=8, frameon=True, edgecolor='none', framealpha=.9)
    motion.legend(loc='upper left', fontsize=8, frameon=False)
    for ax in axes.flat:
        ax.grid(axis='y', color='#CCD5DD', lw=.6, alpha=.55)
        ax.spines['left'].set_color('#93A0AC');ax.spines['bottom'].set_color('#93A0AC')
        ax.tick_params(labelsize=9)
    if other:
        fig.text(.075,.10, f"Amber: stop. Green: withdrawal. Dotted lines mark each retreat peak. Stop peaks: {primary_hz} Hz {m['stop_peak_wrist_force_n']:.3f} N; {other_hz} Hz {om['stop_peak_wrist_force_n']:.3f} N.",
                 fontsize=9, color='#40586A')
        fig.text(.075,.073, f'Initial recorded peg positions differ by {initial_position_difference_mm:.3f} mm. Independent resets and rate-derived controller/filter settings also differ.',
                 fontsize=8.6, color='#536B7C')
        fig.text(.075,.047, 'Wrist forces are raw reactions. This comparison shows configuration sensitivity; it is not a pure time-step test or a matched recovery-policy pair.',
                 fontsize=8.6, color='#536B7C')
    else:
        fig.text(.075,.10, f"Amber: 0.25 s stop. Green: withdrawal. Dotted purple: retreat force peak. Clearance confirmed at {m['clear_time_s']:.3f} s.",
                 fontsize=9, color='#40586A')
        fig.text(.075,.073, 'Wrist norm is an uncalibrated reaction signal; opposing contact = max(0, −world contact Fz). The higher stop peak is separate from the retreat peak.',
                 fontsize=8.6, color='#536B7C')
        fig.text(.075,.047, 'This candidate passes numerical screening, but physical validity and timestep convergence still need confirmation.',
                 fontsize=8.6, color='#536B7C')
    for suffix in ('png', 'pdf'):
        fig.savefig(args.output_dir / f'{args.basename}.{suffix}', dpi=180)
    plt.close(fig)
    provenance = {'primary':primary['info'], 'comparison':other['info'] if other else None,
                  'initial_peg_position_difference_mm':initial_position_difference_mm,
                  'comparison_scope':'independent run configuration sensitivity, not pure timestep convergence' if other else None}
    (args.output_dir / f'{args.basename}_sources.json').write_text(json.dumps(provenance, indent=2, allow_nan=False)+'\n')
    print(args.output_dir / f'{args.basename}.png')


if __name__ == '__main__':
    main()
