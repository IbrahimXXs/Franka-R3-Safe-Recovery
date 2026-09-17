"""Recovery metrics and static plots; usable without starting Isaac Sim."""

import csv
from pathlib import Path
import numpy as np

RECOVERY_PHASES = {'realign', 'retreat', 'clear_hold'}


def summarize(rows, scenario, recovery, force_budget, torque_budget, dt, clearance_mm=.5, hole_entry_z_m=.865):
    insertion = [r for r in rows if r['phase'] == 'insert']
    recover = [r for r in rows if r['phase'] in RECOVERY_PHASES]
    retreat = [r for r in rows if r['phase'] == 'retreat']
    before = [r for r in rows if r['phase'] not in RECOVERY_PHASES]
    start = recover[0]['time_s'] - dt
    clear_time, consecutive = None, 0
    for r in recover:
        clear = r['lowest_peg_z_m'] >= hole_entry_z_m+.002 and r['normal_load_n'] < .1
        consecutive = consecutive + 1 if clear else 0
        if consecutive >= round(.2/dt):
            clear_time = r['time_s']-start
            break
    # Include the complete executed recovery, not just its successful portion.
    peak_force = max(r['force_norm_n'] for r in recover)
    peak_torque = max(r['torque_norm_nm'] for r in recover)
    entered = max(r['depth_mm'] for r in before) >= 1.
    cleared = clear_time is not None
    budget_ok = peak_force <= force_budget and peak_torque <= torque_budget
    status = ('did_not_enter' if not entered else 'not_cleared' if not cleared
              else 'cleared_within_budget' if budget_ok else 'cleared_over_budget')
    resistance = np.array([max(0., -r['fz']) for r in retreat])
    window = max(1, round(.05/dt))
    smoothed = np.convolve(resistance, np.ones(window)/window, mode='valid')
    return {
        'scenario': scenario, 'recovery': recovery, 'status': status,
        'entered': entered, 'cleared': cleared, 'within_budget': budget_ok,
        'max_inserted_depth_mm': max(r['depth_mm'] for r in before),
        'depth_before_recovery_mm': before[-1]['depth_mm'],
        'tilt_before_recovery_deg': before[-1]['tilt_deg'],
        'insertion_peak_resistance_n': max(max(0., r['fz']) for r in insertion),
        'retreat_peak_resistance_n': float(resistance.max()),
        'retreat_peak_50ms_resistance_n': float(smoothed.max()),
        'retreat_resistance_impulse_ns': float(resistance.sum()*dt),
        'recovery_peak_force_n': peak_force, 'recovery_peak_torque_nm': peak_torque,
        'recovery_resistive_work_j': sum(max(0., -r['contact_power_w'])*dt for r in recover),
        'recovery_signed_contact_work_j': sum(r['contact_power_w']*dt for r in recover),
        'time_to_clear_s': clear_time,
        'max_normal_load_n': max(r['normal_load_n'] for r in rows),
        'max_penetration_mm': max(0., -min(r['min_separation_mm'] for r in rows)),
        'baseline_peak_force_n': max(r['force_norm_n'] for r in rows if r['phase'] == 'baseline'),
        'force_budget_n': force_budget, 'torque_budget_nm': torque_budget,
        'penetration_screen_passed': max(0., -min(r['min_separation_mm'] for r in rows)) < .25*clearance_mm,
        'saturated_sample_fraction': sum(bool(r.get('effort_saturated', False)) for r in rows)/len(rows),
    }


def invalid_trial(scenario, recovery, row, reason):
    """Do not invent completed-recovery costs for a truncated invalid trial."""
    return dict(scenario=scenario, recovery=recovery, status='numerically_invalid',
        invalid_reason=reason, aborted_time_s=row['time_s'], aborted_phase=row['phase'],
        aborted_force_n=row['force_norm_n'], aborted_separation_mm=row['min_separation_mm'],
        recovery_resistive_work_j=None, retreat_peak_resistance_n=None, time_to_clear_s=None)


def write_report(directory, results, invalid=()):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory = Path(directory)
    with (directory/'summary.csv').open('w', newline='') as stream:
        all_results = [*results, *invalid]
        fields = list(dict.fromkeys(key for result in all_results for key in result))
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(all_results)
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    depth_fig, depth_axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    for result in results:
        name, policy = result['scenario'], result['recovery']
        with (directory/f'{name}__{policy}.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        values = lambda key: np.array([float(r[key]) for r in rows])
        label = f'{name} / {policy}'
        style = '--' if policy == 'realign' else '-'
        axes[0].plot(values('time_s'), values('fz'), style, lw=1, label=label)
        axes[1].plot(values('time_s'), values('torque_norm_nm'), style, lw=1)
        axes[2].plot(values('time_s'), values('depth_mm'), style, lw=1)
        for ax, phase, sign in zip(depth_axes, ('insert', 'retreat'), (1, -1)):
            selected = [r for r in rows if r['phase'] == phase]
            ax.plot([float(r['depth_mm']) for r in selected],
                    [max(0., sign*float(r['fz'])) for r in selected], style, lw=1, label=label)
    axes[0].set_ylabel('Fixture force Fz [N]\n+upward; −downward')
    axes[1].set_ylabel('Contact torque at peg COM [N m]')
    axes[2].set_ylabel('Actual insertion depth [mm]')
    axes[2].set_xlabel('Experiment time [s]')
    if results:
        axes[0].legend(fontsize=8, ncol=2)
    axes[0].set_title('FR3 simulation: full contact force, torque and actual motion')
    for ax in axes:
        ax.grid(alpha=.25)
    for ax, title in zip(depth_axes, ('Insertion', 'Retreat')):
        ax.set_title(title); ax.set_xlabel('Actual insertion depth [mm]')
        ax.set_ylabel('Axial resisting force [N]'); ax.grid(alpha=.25)
    if results:
        depth_axes[1].legend(fontsize=7)
    fig.savefig(directory/'force_profiles.png', dpi=160)
    fig.savefig(directory/'force_profiles.pdf')
    depth_fig.savefig(directory/'force_vs_depth.png', dpi=160)
    plt.close(fig); plt.close(depth_fig)
    lines = ['# FR3 recovery study', '',
        'Synthetic compliant-contact study. Rankings apply to these parameters and recovery policies.', '',
        f'{len(results)} completed trials; {len(invalid)} numerically invalid trials excluded from recovery costs and plots.', '',
        'Positive Fz is upward fixture-on-peg force. Insertion resistance is max(Fz, 0); '
        'retreat resistance is max(-Fz, 0). All raw samples include normal and friction forces.', '',
        '| Scenario | Recovery | Depth before recovery (mm) | Peak retreat (N) | 50 ms peak (N) | Recovery work proxy (J) | Time to clear (s) | Max overlap (mm) | Contact screen | Outcome |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |']
    for r in results:
        time = 'not cleared' if r['time_to_clear_s'] is None else f"{r['time_to_clear_s']:.3f}"
        lines.append(f"| {r['scenario']} | {r['recovery']} | {r['depth_before_recovery_mm']:.2f} | "
            f"{r['retreat_peak_resistance_n']:.3f} | {r['retreat_peak_50ms_resistance_n']:.3f} | "
            f"{r['recovery_resistive_work_j']:.6f} | {time} | {r['max_penetration_mm']:.4f} | "
            f"{'passed' if r['penetration_screen_passed'] else 'review required'} | {r['status']} |")
    lines += ['', 'The work proxy integrates max(0, −(F·v + τ·ω)) over the full recovery, '
        'using contact wrench and peg COM velocity in the same world frame. It is not motor '
        'energy; a static jam can have zero work while requiring high force. Time and force '
        'limits must be considered separately. Failure to clear means this tested policy '
        'failed within its duration, not that all recovery motions are impossible.', '',
        'Force and torque budgets classify outcomes after execution; the controller does not '
        'enforce those budgets. Different achieved depths are reported and must not be treated '
        'as matched-depth comparisons. See study.json for all run parameters.', '',
        'Contact screen: overlaps above 25% of radial clearance require review. A passed screen '
        'does not establish timestep convergence; compare refinements before trusting a cost ranking.', '',
        '![Force profiles](force_profiles.png)', '', '![Force versus actual depth](force_vs_depth.png)', '']
    if invalid:
        lines += ['## Numerically invalid trials', '',
            'These trials stopped at the numerical guard. Their partial CSVs include the offending sample. '
            'They are excluded from the plots and recovery-cost comparison; missing costs are not zero. '
            'An invalid trial is not evidence of physical jamming or irrecoverability.', '',
            '| Scenario | Recovery | Time (s) | Phase | Force (N) | Separation (mm) | Partial samples |',
            '| --- | --- | ---: | --- | ---: | ---: | --- |']
        for r in invalid:
            filename = f"{r['scenario']}__{r['recovery']}.csv"
            lines.append(f"| {r['scenario']} | {r['recovery']} | {r['aborted_time_s']:.4f} | "
                f"{r['aborted_phase']} | {r['aborted_force_n']:.3f} | {r['aborted_separation_mm']:.4f} | "
                f"[CSV]({filename}) |")
        lines.append('')
    (directory/'report.md').write_text('\n'.join(lines))
