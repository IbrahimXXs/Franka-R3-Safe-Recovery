"""Plot matched terminal policies for the completed 0.2 mm / 2-degree case.

Recenter + align changes BOTH XY centering and orientation. Costs are raw wrist
norms; every phase is included in the left panel and only retreat in the right.
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


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=here.parent / 'Forge-GapTilt-Pilot-20260915' /
                        'gap_0200' / 'g0200_d30_pitch_pos20_try00')
    parser.add_argument('--output-dir', type=Path, default=here)
    args = parser.parse_args()
    attempt = json.loads((args.profile / 'trajectory.json').read_text())
    if attempt['status'] != 'complete':
        raise ValueError('Only a completed attempt can be plotted')
    cp = next(cp for cp in attempt['checkpoints'] if cp['checkpoint_kind'] == 'terminal')
    probes = {p['policy']: p for p in cp['probes']}
    for policy in ('straight', 'realign'):
        p = probes[policy]
        if not (p['label_eligible'] and p['replay_matched'] and p['safe_recovery']):
            raise ValueError('Figure requires matched eligible safe terminal probes for both policies')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':10, 'axes.labelsize':10,
                         'axes.titlesize':12, 'axes.spines.top':False, 'axes.spines.right':False,
                         'pdf.fonttype':42, 'savefig.facecolor':'white'})
    fig, (bars, curves) = plt.subplots(1, 2, figsize=(13.1, 6.1), gridspec_kw={'width_ratios':[1.,1.1]})
    fig.subplots_adjust(left=.07, right=.975, bottom=.255, top=.755, wspace=.22)
    fig.suptitle('Recenter + align reduces the subsequent withdrawal load', x=.07, y=.969,
                 ha='left', fontsize=18, weight='bold', color='#18384D')
    fig.text(.07, .90, '0.2 mm radial clearance · 2° insertion tilt · same terminal checkpoint · both replays matched and recovered safely',
             fontsize=10.5, color='#40586A')
    fig.text(.07, .851, 'The pose adjustment moves the peg toward the hole center in XY and returns its orientation upright.',
             fontsize=10.5, color='#40586A')
    colors = {'straight':'#087F8C', 'realign':'#C76B39'}
    names = {'straight':'Straight', 'realign':'Recenter + align'}
    x = np.arange(3); width = .32
    fields = ('stop_peak_wrist_force_n', 'realign_peak_wrist_force_n', 'retreat_peak_wrist_force_n')
    raw_sources = {}
    for offset, policy in ((-.5, 'straight'), (.5, 'realign')):
        p = probes[policy]
        vals = [p.get(key) for key in fields]
        positions = x + offset * width
        for pos, value in zip(positions, vals):
            if value is None:
                bars.text(pos, .08, 'N/A', ha='center', va='bottom', fontsize=8, color='#75808B')
            else:
                bars.bar(pos, value, width, color=colors[policy], alpha=.91)
                bars.text(pos, value+.07, f'{value:.3f}', ha='center', va='bottom', fontsize=9, color='#28434F')
        bars.plot([], [], color=colors[policy], lw=7, label=names[policy])
        path = args.profile / f'terminal_{policy}_recovery.csv'
        with path.open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        raw_sources[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
        retreat_indices = [i for i, row in enumerate(rows) if row['phase']=='retreat']
        start = float(rows[retreat_indices[0]-1]['recovery_time_s'])
        retreat = [rows[i] for i in retreat_indices]
        t = np.array([float(r['recovery_time_s'])-start for r in retreat])
        values = np.array([float(r['wrist_force_n']) for r in retreat])
        curves.plot(t, values, color=colors[policy], lw=1.25, label=names[policy])
    bars.set_xticks(x, ['Common stop\n0.25 s', 'Recenter + align\n2 s if selected', 'Withdrawal'])
    bars.set(ylabel='Raw wrist norm: phase peak [N]', ylim=(0,3.85),
             title='Full recovery: separate phase peaks')
    bars.legend(loc='upper right', fontsize=8, frameon=False)
    curves.set(xlabel='Time since each policy starts withdrawal [s]', ylabel='Raw wrist force norm [N]',
               ylim=(-.05,2.9), title='Withdrawal phase, aligned at its start')
    curves.legend(loc='upper right', fontsize=9, frameon=True, edgecolor='none', framealpha=.92)
    curves.text(.97,.65, f"Retreat peak: {probes['straight']['retreat_peak_wrist_force_n']:.3f} → {probes['realign']['retreat_peak_wrist_force_n']:.3f} N\n"
                f"100 ms mean max: {probes['straight']['retreat_peak_wrist_force_mean100ms_n']:.3f} → {probes['realign']['retreat_peak_wrist_force_mean100ms_n']:.3f} N",
                transform=curves.transAxes, ha='right', va='top', fontsize=9, color='#40586A',
                bbox=dict(facecolor='white', edgecolor='none', alpha=.85, pad=3))
    for ax in (bars, curves):
        ax.grid(axis='y', color='#CAD4DD', alpha=.6, lw=.6)
        ax.set_axisbelow(True)
        ax.spines['left'].set_color('#93A0AC');ax.spines['bottom'].set_color('#93A0AC')
        ax.tick_params(labelsize=9)
    fig.text(.07, .167, f"Whole-recovery peak remains {probes['straight']['max_wrist_force_n']:.3f} N for BOTH policies, caused by the common stop stage.",
             fontsize=10.5, color='#294E5C', weight='bold')
    fig.text(.07, .122, f"Total recovery duration: straight {probes['straight']['duration_s']:.3f} s; recenter + align {probes['realign']['duration_s']:.3f} s. The stop holds the attained hand pose and can unload insertion forces.",
             fontsize=9, color='#40586A')
    fig.text(.07, .081, 'The reduction belongs to the combined centering/orientation policy; this comparison does not isolate the effect of angle alone.',
             fontsize=9, color='#40586A')
    fig.text(.07, .042, 'These are independent replay branches with observable-state matching. Numerical screening passed; hidden contact memory and timestep convergence are not certified.',
             fontsize=8.5, color='#627888')
    for suffix in ('png', 'pdf'):
        fig.savefig(args.output_dir / f'terminal_policy_comparison.{suffix}', dpi=180)
    plt.close(fig)
    source = dict(attempt_directory=str(args.profile.resolve()), checkpoint_id=cp['checkpoint_id'],
                  checkpoint_state=cp['state'], probes=probes,
                  source_sha256={**raw_sources, 'trajectory.json':hashlib.sha256((args.profile/'trajectory.json').read_bytes()).hexdigest()})
    (args.output_dir/'terminal_policy_comparison_sources.json').write_text(json.dumps(source,indent=2,allow_nan=False)+'\n')
    print(args.output_dir/'terminal_policy_comparison.png')


if __name__=='__main__':main()
