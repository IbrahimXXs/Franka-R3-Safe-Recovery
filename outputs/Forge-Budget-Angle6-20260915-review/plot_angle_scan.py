"""Plot closed straight reference observations; never label missing recovery."""
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulation.summarize_budget_study import summarize_study


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    output = Path(__file__).resolve().parent
    study = output.with_name(output.name.removesuffix('-review'))
    result = summarize_study(study)
    if result['summary']['source_status'] != 'complete':
        raise ValueError('Finalize the study before plotting this completed angle comparison')
    rows = [r for r in result['branches'] if r['policy'] == 'straight']
    if len(rows) != 6 or any(r['status'] != 'complete' for r in rows):
        raise ValueError('Expected the six closed straight branches')
    fields = ('condition_id', 'tilt_amplitude_deg', 'tilt_ramp_duration_s',
              'planned_peak_command_tilt_rate_deg_s', 'terminal_command_tilt_deg',
              'terminal_actual_tilt_deg', 'max_actual_tilt_deg', 'tilt_ramp_completed',
              'terminal_depth_mm', 'reference_complete', 'reference_observed_peak_wrist_force_n',
              'first_budget_crossing_segment', 'first_budget_crossing_phase',
              'first_budget_crossing_time_s', 'outcome', 'recovery_safe')
    with (output/'angle_attainment.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k:r[k] for k in fields} for r in rows)

    x = np.arange(len(rows))
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), layout='constrained')
    names = [('tilt_amplitude_deg', 'Planned target', '#d9d9d9'),
             ('terminal_command_tilt_deg', 'Last executed command', '#dda55b'),
             ('terminal_actual_tilt_deg', 'Actual terminal tilt', '#4687a8')]
    for i, (field, label, color) in enumerate(names):
        bars = axes[0].bar(x+(i-1)*.24, [r[field] for r in rows], width=.23,
                           label=label, color=color)
        axes[0].bar_label(bars, fmt='%.2f', fontsize=9, padding=2)
    axes[0].set(ylabel='Angle (degrees)', ylim=(0, 6.9),
                title='Command targets up to 6 degrees do not imply attained poses')
    axes[0].legend(loc='upper left', framealpha=.95, fontsize=9)

    peaks = [r['reference_observed_peak_wrist_force_n'] for r in rows]
    colors = ['#8abfa0' if r['reference_complete'] else '#e6ad70' for r in rows]
    bars = axes[1].bar(x, peaks, color=colors, width=.65)
    axes[1].bar_label(bars, fmt='%.2f N', fontsize=10, padding=3)
    axes[1].axhline(4., color='#b33232', linestyle='--', label='4 N raw wrist-norm budget')
    axes[1].set(ylabel='Observed reference peak (N)', ylim=(0, max(4., *peaks)*1.22),
                title='Reference only: insertion, tilt and hold; no withdrawal force is shown')
    axes[1].legend(loc='upper left', fontsize=9)
    labels = [f"{r['tilt_amplitude_deg']:g}° target\n" +
              ('Reference complete' if r['reference_complete'] else
               f"Stopped: {r['first_budget_crossing_phase'] or r['reference_reason']}") for r in rows]
    for ax in axes:
        ax.set(xticks=x, xticklabels=labels)
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
    fig.suptitle('18 mm target depth | friction 1/1 | 240 Hz | 1 s commanded tilt ramp\n'
                 'Larger targets also increase planned angular speed; stopped trajectories are incomplete', fontsize=12)
    for suffix in ('png', 'pdf'):
        fig.savefig(output/f'angle_attainment.{suffix}', dpi=180)
    plt.close(fig)
    sources = dict(source_study=result['summary']['source_study'],
                   source_study_sha256=result['summary']['source_study_sha256'],
                   source_files=result['summary']['input_files'],
                   script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   note='Closed observed reference values only. Target/rate values are planned; no unexecuted future is imputed.')
    (output/'angle_plot_sources.json').write_text(json.dumps(sources, indent=2)+'\n')


if __name__ == '__main__':
    main()
