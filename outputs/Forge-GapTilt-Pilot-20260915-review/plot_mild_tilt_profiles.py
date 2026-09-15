"""Reproduce the completed 0.5-degree / 30%-depth case vs aligned controls.

Only reads the six named completed cases. No active run data are changed.
Run with the project's franka-safe-recovery conda Python (matplotlib/numpy).
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


TEAL = '#087F8C'
GRAY = '#838B96'
NAVY = '#233B60'
RUST = '#C7522A'
RAMP = '#E5EFFA'
STOP = '#FFF0D9'
RETREAT = '#EDF5EE'


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def series(rows, key):
    return np.array([float(row[key]) for row in rows])


def load_case(batch, gap_key, tilted):
    case_id = f'g{gap_key}_d30_pitch_pos05' if tilted else f'g{gap_key}_aligned'
    folder = batch / f'gap_{gap_key}' / f'{case_id}_try00'
    attempt = json.loads((folder / 'trajectory.json').read_text())
    if attempt['status'] != 'complete':
        raise ValueError(f'Case is not complete: {case_id}')
    if not attempt['metrics']['numerically_valid']:
        raise ValueError(f'Case failed numerical screening: {case_id}')
    insertion = [row for row in read_rows(folder / 'insertion.csv') if row['phase'] in ('insert', 'hold')]
    recovery = read_rows(folder / 'final_retreat.csv')
    provenance = dict(case_id=case_id, folder=str(folder),
                      radial_clearance_mm=attempt['radial_clearance_mm'],
                      effective_radial_clearance_mm=attempt['effective_radial_clearance_mm'],
                      insertion=attempt['metrics'], continuation_straight=attempt['final_retreat'],
                      source_sha256={name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
                                     for name in ('trajectory.json', 'insertion.csv', 'final_retreat.csv')})
    return attempt, insertion, recovery, provenance


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch-dir', type=Path, default=here.parent / 'Forge-GapTilt-Pilot-20260915')
    parser.add_argument('--output-dir', type=Path, default=here)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.titlesize': 13, 'axes.labelsize': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'pdf.fonttype': 42, 'savefig.facecolor': 'white'})
    fig, axes = plt.subplots(4, 3, figsize=(14.7, 11.8), sharey='row',
                             gridspec_kw={'height_ratios': [1., .95, 1.1, 1.2]})
    fig.subplots_adjust(left=.075, right=.985, bottom=.105, top=.865, hspace=.34, wspace=.13)
    fig.suptitle('Smaller clearance raises loads under a mild insertion tilt',
                 x=.075, y=.974, ha='left', fontsize=19, weight='bold', color='#142E43')
    fig.text(.075, .94, '0.5° pitch starts at 6 mm actual depth (30% of a 20 mm insertion); smooth 1 s ramp.',
             fontsize=11, color='#344A5C')
    fig.text(.075, .915, 'All three tilted insertions and continuous straight withdrawals succeeded; no insertion stall was detected.',
             fontsize=11, color='#344A5C')
    provenance = []
    for col, gap_key in enumerate(('0507', '0200', '0100')):
        attempt, ins, rec, prov = load_case(args.batch_dir, gap_key, True)
        control, base_ins, base_rec, base_prov = load_case(args.batch_dir, gap_key, False)
        if not (attempt['metrics']['insertion_success'] and not attempt['metrics']['stalled']
                and attempt['final_retreat']['safe_recovery']):
            raise ValueError('The figure subtitle requires successful insertion/withdrawal and no detected stall')
        provenance.extend((prov, base_prov))
        trigger = attempt['metrics']['tilt_trigger_time_s']
        complete = attempt['metrics']['tilt_completion_time_s'] - trigger
        t = series(ins, 'time_s') - trigger
        bt = series(base_ins, 'time_s') - trigger
        for ax in axes[:3, col]:
            ax.axvspan(0., complete, color=RAMP, zorder=0)
            ax.axvline(0., color='#7698BC', lw=.75, ls=':', zorder=1)
            ax.axvline(complete, color='#7698BC', lw=.75, ls=':', zorder=1)
            ax.set_xlim(-4.3, 4.85)
            ax.set_xticks([-4, -2, 0, 1, 2, 4])
        ax = axes[0, col]
        ax.set_title(f"Radial clearance {attempt['radial_clearance_mm']:g} mm", pad=13, weight='bold')
        ax.plot(bt, series(base_ins, 'depth_mm'), color=GRAY, lw=1.8, alpha=.75, label='Aligned control')
        ax.plot(t, series(ins, 'depth_mm'), color=TEAL, lw=1.6, label='0.5° case: actual')
        ax.plot(t, series(ins, 'command_depth_mm'), color=NAVY, lw=1., ls='--', label='Command')
        ax.axhline(0., color='#ADB4BC', lw=.6)
        ax.set_ylim(-11, 24)
        ax.set_yticks([-10, 0, 6, 10, 20])
        ax.text(.02, .96, '0 s: 6 mm trigger\n1 s: ramp ends', transform=ax.transAxes,
                ha='left', va='top', fontsize=8.5, color='#466987')
        ax = axes[1, col]
        ax.plot(bt, series(base_ins, 'tilt_deg'), color=GRAY, lw=1.1, alpha=.8)
        ax.plot(t, series(ins, 'tilt_deg'), color=TEAL, lw=1.1)
        ax.plot(t, series(ins, 'command_tilt_deg'), color=NAVY, lw=1.2, ls='--')
        ax.set_ylim(-.025, .64)
        ax.set_yticks([0., .25, .5])
        ax = axes[2, col]
        ax.plot(bt, series(base_ins, 'wrist_force_n'), color=GRAY, lw=1.05, alpha=.9, label='Aligned control')
        ax.plot(t, series(ins, 'wrist_force_n'), color=TEAL, lw=.95, label='0.5° case')
        ax.set_ylim(-.04, 1.24)
        ax.set_yticks([0., .4, .8, 1.2])
        ax.set_xlabel('Insertion time relative to tilt trigger [s]')
        ax.text(.97, .94, f"Recorded peak: {attempt['metrics']['max_wrist_force_n']:.3f} N",
                transform=ax.transAxes, ha='right', va='top', color=TEAL, fontsize=9,
                bbox=dict(facecolor='white', edgecolor='none', alpha=.85, pad=2))
        ax = axes[3, col]
        rt = series(rec, 'recovery_time_s')
        brt = series(base_rec, 'recovery_time_s')
        retreat_start = min(float(row['recovery_time_s']) for row in rec if row['phase'] == 'retreat')
        dt = float(np.median(np.diff(rt)))
        boundary = retreat_start - dt
        ax.axvspan(0., boundary, color=STOP, zorder=0)
        ax.axvspan(boundary, rt[-1], color=RETREAT, zorder=0)
        ax.axvline(boundary, color='#B89B6C', lw=.7, ls=':')
        ax.plot(brt, series(base_rec, 'wrist_force_n'), color=GRAY, lw=1.05, alpha=.9, label='Aligned wrist')
        ax.plot(rt, series(rec, 'wrist_force_n'), color=TEAL, lw=1., label='0.5° wrist')
        ax.plot(rt, np.maximum(0., -series(rec, 'fz')), color=RUST, lw=1., label='Opposing axial contact')
        ax.set_xlim(-.02, 4.65)
        ax.set_ylim(-.018, .53)
        ax.set_yticks([0., .2, .4])
        ax.set_xlabel('Continuous recovery time [s]')
        metrics = attempt['final_retreat']
        ax.text(.98, .96, f"Retreat peak: {metrics['retreat_peak_wrist_force_n']:.3f} N\n"
                f"Stop peak: {metrics['stop_peak_wrist_force_n']:.3f} N",
                transform=ax.transAxes, ha='right', va='top', fontsize=9, color='#244A50',
                bbox=dict(facecolor='white', edgecolor='none', alpha=.88, pad=2))
        for row in range(2):
            axes[row, col].tick_params(labelbottom=False)
        for row in range(4):
            axes[row, col].grid(axis='y', color='#CFD5DB', alpha=.5, lw=.6)
            axes[row, col].tick_params(labelsize=9)
            axes[row, col].spines['left'].set_color('#909BA6')
            axes[row, col].spines['bottom'].set_color('#909BA6')
    axes[0, 0].set_ylabel('Insertion depth [mm]')
    axes[1, 0].set_ylabel('Tilt magnitude [deg]')
    axes[2, 0].set_ylabel('Raw wrist force norm [N]')
    axes[3, 0].set_ylabel('Recovery force [N]')
    axes[0, 1].legend(loc='lower right', fontsize=8, frameon=True, framealpha=.9, edgecolor='none')
    axes[3, 1].legend(loc='upper left', bbox_to_anchor=(.03, .62), fontsize=8,
                      frameon=True, framealpha=.9, edgecolor='none')
    fig.text(.075, .056, 'Blue shading: tilt ramp. Amber: 0.25 s stop. Green: direct withdrawal until clearance. The recovery plots use the original continuous motion, not replay probes.',
             fontsize=9, color='#344A5C')
    fig.text(.075, .034, 'Wrist values are raw reaction-force norms; opposing axial contact = max(0, −world contact Fz). Common force scales are used across gaps. Numerical screening passed; convergence is not established.',
             fontsize=8.5, color='#566774')
    for suffix in ('png', 'pdf'):
        fig.savefig(args.output_dir / f'mild_tilt_profiles.{suffix}', dpi=180)
    plt.close(fig)
    (args.output_dir / 'mild_tilt_profiles_sources.json').write_text(json.dumps(provenance, indent=2, allow_nan=False) + '\n')
    print(args.output_dir / 'mild_tilt_profiles.png')


if __name__ == '__main__':
    main()
