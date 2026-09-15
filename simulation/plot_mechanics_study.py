"""Standalone scientific figures for a mechanics summary, with explicit masks.

All plotted values come from the summary of one immutable manifest snapshot.
Missing, rejected and censored values retain their status instead of becoming
zero. Matplotlib is imported only by write_plots, so table exports remain light.
"""
import math
from pathlib import Path
import textwrap


FRICTION_ORDER = ((.5, .5), (.75, .75), (1., 1.), (1., .5))
REFERENCE_MASKS = {'not_started': '—', 'incomplete': 'RUN', 'excluded': 'EX',
                   'numerically_invalid': 'NV', 'grasp_failed': 'GP',
                   'operational_budget_exceeded': 'BD',
                   'aligned_depth_gate_not_reached': 'DG', 'tilt_sequence_incomplete': 'TI'}


def measurement_cell(row, key, *, recovery=False):
    """Return an eligible value or its explicit exclusion / missing-data mask."""
    if row is None:
        return None, 'N/A'
    state = row.get('reference_state')
    if state != 'valid_protocol_completed':
        return None, REFERENCE_MASKS.get(state, 'NA')
    if recovery:
        if row.get('straight_numerically_valid') is not True:
            return None, 'NV' if row.get('straight_numerically_valid') is False else 'NA'
        if row.get('straight_recovery_censored') is True or row.get('straight_cleared') is False:
            return None, 'RC'
        if row.get('straight_safe_recovery') is not True or row.get('straight_cleared') is not True:
            return None, 'RC' if row.get('straight_safe_recovery') is False else 'NA'
    value = row.get(key)
    if type(value) not in (int, float) or not math.isfinite(value):
        return None, 'NA'
    return value, None


def _ordered_factors(cases):
    depths = sorted({row['target_depth_mm'] for row in cases})
    observed_pairs = {(r['pair_static_friction'], r['pair_dynamic_friction']) for r in cases}
    pairs = [pair for pair in FRICTION_ORDER if pair in observed_pairs]
    pairs += sorted(observed_pairs-set(pairs))
    angles = sorted({row['tilt_amplitude_deg'] for row in cases})
    return depths, pairs, angles


def _grid(cases, depths, pairs, angle, key, recovery):
    selected = {(r['target_depth_mm'], r['pair_static_friction'], r['pair_dynamic_friction']): r
                for r in cases if r['tilt_amplitude_deg'] == angle}
    return [[measurement_cell(selected.get((depth, *pair)), key, recovery=recovery)
             for pair in pairs] for depth in depths]


def _save(fig, directory, basename):
    paths = []
    for extension in ('png', 'pdf'):
        path = directory/f'{basename}.{extension}'
        fig.savefig(path, dpi=180, facecolor='white', bbox_inches='tight', pad_inches=.15)
        paths.append(str(path))
    return paths


def _grid_figure(result, directory, plt, np, *, metrics, basename, heading, recovery=False):
    from matplotlib.colors import Normalize, TwoSlopeNorm
    cases = result['cases']
    depths, pairs, angles = _ordered_factors(cases)
    grids = [[_grid(cases, depths, pairs, angle, key, recovery) for key, *_ in metrics]
             for angle in angles]
    fig, axes = plt.subplots(len(angles), len(metrics), squeeze=False,
                             figsize=(12.8, 3.5*len(angles)+1.45))
    norms = []
    for j, (key, _, _, diverging) in enumerate(metrics):
        values = [value for grid in grids for cells in grid[j] for value, _ in cells if value is not None]
        if diverging:
            extent = max((abs(value) for value in values), default=1.) or .001
            norms.append(TwoSlopeNorm(vmin=-extent, vcenter=0., vmax=extent))
        else:
            # Force panels share the raw-peak scale: a 100 ms mean cannot appear
            # larger merely because its color axis is independently rescaled.
            if recovery:
                values = [value for grid in grids for metric in grid for cells in metric
                          for value, _ in cells if value is not None]
            elif key == 'terminal_actual_tilt_deg':
                values += angles
            norms.append(Normalize(vmin=0., vmax=max(values, default=1.) or .001))
    for i, angle in enumerate(angles):
        for j, (_, title, unit, diverging) in enumerate(metrics):
            ax = axes[i, j]
            cells = grids[i][j]
            values = np.array([[value if value is not None else np.nan for value, _ in line]
                               for line in cells], dtype=float)
            cmap = plt.get_cmap('RdBu_r' if diverging else 'viridis').copy()
            cmap.set_bad('#f1f2f4')
            image = ax.imshow(np.ma.masked_invalid(values), cmap=cmap, norm=norms[j], aspect='auto')
            ax.set_xticks(range(len(pairs)), [f'({s:g}, {d:g})' for s, d in pairs])
            ax.set_yticks(range(len(depths)), [f'{depth:g}' for depth in depths])
            ax.set_xlabel('Effective friction pair (static, dynamic)')
            ax.set_ylabel('Target depth (mm)')
            ax.set_title(f'Target tilt {angle:g}°  |  {title}', loc='left', fontsize=11, pad=10)
            ax.set_xticks(np.arange(-.5, len(pairs), 1), minor=True)
            ax.set_yticks(np.arange(-.5, len(depths), 1), minor=True)
            ax.grid(which='minor', color='white', linewidth=2)
            ax.tick_params(which='minor', bottom=False, left=False)
            for y, line in enumerate(cells):
                for x, (value, mask) in enumerate(line):
                    label = mask if mask else f'{value:.3f}'
                    color = '#56606d'
                    if value is not None:
                        r, g, b, _ = cmap(norms[j](value))
                        color = '#17212b' if .2126*r+.7152*g+.0722*b > .57 else 'white'
                    ax.text(x, y, label, ha='center', va='center', color=color, fontsize=11,
                            weight='semibold' if value is not None else 'normal')
            fig.colorbar(image, ax=ax, fraction=.046, pad=.035, label=unit)
    counts = result['summary']['counts']
    fig.suptitle(heading+f"\n{counts['completed_cases']} completed / {counts['planned_cases']} planned conditions; "
                 f"{counts['valid_protocol_completed']} valid full reference protocols", fontsize=15, x=.07, ha='left')
    note = ('Withdrawal values: valid references + completed, safe continuous straight clearance; stop is excluded. '
            'Mean peak uses full 100 ms windows.' if recovery else
            'Actual tilt is measured at the reference endpoint. Depth loss = depth before tilt window − terminal depth; '
            'negative means deeper. No material-deformation claim.')
    fig.text(.07, .105, textwrap.fill(note, 163), fontsize=8.4, va='bottom')
    masks = ('Masks: — pending; RUN incomplete; EX excluded; NV numerical reject; GP grasp failure; BD budget; '
             'DG depth gate; TI tilt sequence; RC recovery incomplete/censored; NA missing; N/A unplanned.')
    fig.text(.07, .058, textwrap.fill(masks, 165), fontsize=8.1, va='bottom')
    fig.text(.07, .028, 'One run per condition. Differences are descriptive; no statistical significance or physical-world validation is implied.',
             fontsize=8.4, color='#586472')
    fig.subplots_adjust(left=.075, right=.94, bottom=.22, top=.85, hspace=.48, wspace=.34)
    paths = _save(fig, directory, basename)
    plt.close(fig)
    return paths


def _paired_figure(result, directory, plt, np):
    cases = result['cases']
    paired = [row for row in cases if row.get('paired_results_known') is True and
              row.get('paired_costs_complete') is True and row.get('reference_state') == 'valid_protocol_completed']
    paired.sort(key=lambda row: (row['target_depth_mm'],
                                 FRICTION_ORDER.index((row['pair_static_friction'], row['pair_dynamic_friction'])),
                                 row['tilt_amplitude_deg']))
    metrics = [('max_wrist_force_n', 'Whole recovery peak'),
               ('retreat_peak_wrist_force_n', 'Retreat peak'),
               ('retreat_peak_wrist_force_mean100ms_n', 'Retreat 100 ms mean peak')]
    fig, axes = plt.subplots(1, 3, figsize=(14, max(4.9, .47*len(paired)+2.3)), sharey=True)
    for ax, (key, title) in zip(axes, metrics):
        ax.set_title(title, fontsize=11, loc='left')
        ax.set_xlabel('Wrist force norm (N)')
        ax.grid(axis='x', alpha=.18, zorder=0)
        for index, row in enumerate(paired):
            for policy, offset, color, label in [('straight', -.17, '#1f77b4', 'Continuous straight'),
                                                 ('realign', .17, '#dd8452', 'Recenter + align')]:
                value = row.get(f'{policy}_{key}')
                y = index+offset
                if type(value) in (int, float) and math.isfinite(value):
                    ax.barh(y, value, height=.31, color=color, zorder=2, label=label if index == 0 else None)
                    ax.annotate(f'{value:.3f}', (value, y), xytext=(4, 0), textcoords='offset points',
                                va='center', fontsize=8)
                else:
                    ax.annotate('NA', (0, y), xytext=(4, 0), textcoords='offset points', va='center', fontsize=8)
        ax.margins(x=.22)
        ax.spines[['top', 'right']].set_visible(False)
    labels = [f"d={r['target_depth_mm']:g} mm  μ=({r['pair_static_friction']:g}, {r['pair_dynamic_friction']:g})  θ={r['tilt_amplitude_deg']:g}°"
              for r in paired]
    axes[0].set_yticks(range(len(paired)), labels, fontsize=9)
    if paired:
        axes[0].invert_yaxis()
        axes[0].legend(loc='lower left', bbox_to_anchor=(0, 1.10), ncol=2, fontsize=9, frameon=False)
    else:
        for ax in axes:
            ax.set_xlim(0, 1)
            ax.set_xticks([])
            ax.text(.5, .5, 'No eligible completed policy pair', transform=ax.transAxes,
                    ha='center', va='center', fontsize=10, color='#586472')
    fig.suptitle(f"Matched recovery costs: {len(paired)} completed pairs / {len(cases)} planned conditions",
                 x=.06, ha='left', fontsize=15)
    fig.text(.06, .09, 'Pair requires endpoint + full-prefix physical tolerances to match and both eligible recoveries to clear without censoring. '
             'Exact-bit prefix equality is diagnostic only.', fontsize=8.4)
    fig.text(.06, .045, 'Whole recovery includes the attained-pose stop and any alignment. Recenter + align adjusts both XY and orientation. '
             'One run per condition; descriptive costs only.', fontsize=8.4)
    fig.subplots_adjust(left=.285 if paired else .065, right=.965, bottom=.22, top=.74 if len(paired)<4 else .88, wspace=.2)
    paths = _save(fig, directory, 'mechanics_paired_policy_costs')
    plt.close(fig)
    return paths


def _reference_load_figure(result, directory, plt, np):
    """Observed reference loads, preserving the scope of early termination."""
    from matplotlib.colors import Normalize
    from matplotlib.patches import Rectangle
    cases = result['cases']
    depths, pairs, angles = _ordered_factors(cases)
    metrics = [('max_normal_load_n', 'Peak Σ contact-normal magnitudes', 'Normal-load sum ΣN (N)', 'Purples'),
               ('reference_peak_wrist_force_n', 'Peak wrist force norm', 'Wrist force norm (N)', 'Blues')]
    partial = {'grasp_failed', 'operational_budget_exceeded', 'aligned_depth_gate_not_reached', 'tilt_sequence_incomplete'}

    def cell(row, key):
        if row is None:
            return None, 'N/A', None
        state = row.get('reference_state')
        mask = REFERENCE_MASKS.get(state)
        if (row.get('attempt_status') != 'complete' or row.get('numerically_valid') is not True or
                state not in partial | {'valid_protocol_completed'}):
            return None, mask or 'NA', None
        value = row.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            return None, 'NA', None
        return value, mask if state in partial else None, row.get('terminal_actual_tilt_deg')

    maps = [{(r['target_depth_mm'], r['pair_static_friction'], r['pair_dynamic_friction']):r
             for r in cases if r['tilt_amplitude_deg'] == angle} for angle in angles]
    grids = [[[[cell(mapping.get((depth, *pair)), key) for pair in pairs] for depth in depths]
              for key, *_ in metrics] for mapping in maps]
    norms = [Normalize(0, max((v for angle in grids for row in angle[j] for v, _, _ in row if v is not None), default=1.) or .001)
             for j in range(len(metrics))]
    fig, axes = plt.subplots(len(angles), 2, squeeze=False, figsize=(13.2, 3.5*len(angles)+1.65))
    for i, angle in enumerate(angles):
        for j, (_, title, unit, palette) in enumerate(metrics):
            ax = axes[i, j]
            cells = grids[i][j]
            values = np.array([[value if value is not None else np.nan for value, _, _ in row] for row in cells])
            cmap = plt.get_cmap(palette).copy();cmap.set_bad('#ededf0')
            image = ax.imshow(np.ma.masked_invalid(values), cmap=cmap, norm=norms[j], aspect='auto')
            ax.set_title(f'Target tilt {angle:g}°  |  {title}', loc='left', fontsize=10.5, pad=10)
            ax.set_xticks(range(len(pairs)), [f'({s:g}, {d:g})' for s, d in pairs])
            ax.set_yticks(range(len(depths)), [f'{depth:g}' for depth in depths])
            ax.set_xlabel('Effective friction pair (static, dynamic)')
            ax.set_ylabel('Target depth (mm)')
            ax.set_xticks(np.arange(-.5, len(pairs), 1), minor=True)
            ax.set_yticks(np.arange(-.5, len(depths), 1), minor=True)
            ax.grid(which='minor', color='white', linewidth=2)
            ax.tick_params(which='minor', bottom=False, left=False)
            for y, row in enumerate(cells):
                for x, (value, mask, actual_angle) in enumerate(row):
                    if value is None:
                        ax.text(x, y, mask, ha='center', va='center', color='#56606d', fontsize=11)
                        continue
                    rgb = cmap(norms[j](value))[:3]
                    color = '#17212b' if sum(c*w for c, w in zip(rgb, (.2126, .7152, .0722))) > .55 else 'white'
                    if mask:
                        ax.add_patch(Rectangle((x-.49, y-.49), .98, .98, fill=False,
                                               hatch='//', edgecolor='#9a7c46', linewidth=1.2, zorder=2))
                    ax.text(x, y-.13, f'{value:.3f}', ha='center', va='center', color=color,
                            fontsize=11, weight='semibold', zorder=3)
                    tilt = f'θactual {actual_angle:.2f}°' if type(actual_angle) in (int, float) and math.isfinite(actual_angle) else 'θactual NA'
                    ax.text(x, y+.23, (mask+' · ' if mask else '')+tilt, ha='center', va='center', color=color,
                            fontsize=8.2, zorder=3)
            fig.colorbar(image, ax=ax, fraction=.046, pad=.035, label=unit)
    counts = result['summary']['counts']
    fig.suptitle('Reference contact loads and wrist reaction\n'+
                 f"{counts['completed_cases']} closed attempts / {counts['planned_cases']} planned conditions; "
                 'hatched cells are truncated references', x=.07, ha='left', fontsize=15)
    notes = [
        'Independent color scales: ΣN sums contact-normal magnitudes, including opposing sides. ΣN is neither axial pull force nor wrist-force norm.',
        'Cells show the recorded reference peak and actual terminal angle. Hatching + GP/BD/DG/TI retains observed data from a reference stopped before full completion.',
        'GP = grasp-slip threshold reached (not demonstrated dropping); BD = operating budget; DG = depth gate not reached; TI = tilt sequence incomplete.',
        'Grey masks: — pending; RUN active/incomplete file; EX excluded; NV numerical reject; NA missing; N/A unplanned. No missing or rejected peak is replaced by zero.',
        'These are reference-phase loads, not withdrawal costs. One run per condition; descriptive evidence only.'
    ]
    for index, note in enumerate(notes):
        fig.text(.07, .143-index*.025, note, fontsize=8.1, color='#56606d')
    fig.subplots_adjust(left=.075, right=.94, bottom=.24, top=.84, hspace=.48, wspace=.34)
    paths = _save(fig, directory, 'mechanics_reference_load_grid')
    plt.close(fig)
    return paths


def write_plots(result, output_dir):
    """Write PNG/PDF artifacts; source simulation data are never accessed here."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    with plt.rc_context({'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'pdf.fonttype': 42}):
        paths = _grid_figure(result, directory, plt, np, recovery=True,
                            heading='Continuous straight withdrawal: retreat force costs',
                            basename='mechanics_straight_force_grid', metrics=[
                                ('straight_retreat_peak_wrist_force_n', 'Instantaneous peak', 'Wrist force norm (N)', False),
                                ('straight_retreat_peak_wrist_force_mean100ms_n', '100 ms mean peak', 'Wrist force norm (N)', False)])
        paths += _grid_figure(result, directory, plt, np,
                             heading='Fixed-depth tilt response: actual pose and depth loss',
                             basename='mechanics_tilt_depth_grid', metrics=[
                                 ('terminal_actual_tilt_deg', 'Actual terminal tilt', 'Actual tilt (deg)', False),
                                 ('tilt_terminal_depth_loss_mm', 'Terminal depth loss', 'Depth loss (mm)', True)])
        paths += _paired_figure(result, directory, plt, np)
        paths += _reference_load_figure(result, directory, plt, np)
    return paths
