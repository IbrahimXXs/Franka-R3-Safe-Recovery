#!/usr/bin/env python3
"""Build an offline interactive dashboard for one FR3 study (Python stdlib only)."""

import argparse
import base64
import csv
import html
import json
import math
from pathlib import Path
import webbrowser

HERE = Path(__file__).resolve().parent
COMPLETE = {'cleared_within_budget', 'cleared_over_budget', 'not_cleared', 'did_not_enter'}
SERIES = ('time_s', 'phase', 'depth_mm', 'command_depth_mm', 'tilt_deg', 'command_tilt_deg',
          'tip_x_mm', 'tip_y_mm', 'fx', 'fy', 'fz', 'taux', 'tauy', 'tauz',
          'force_norm_n', 'torque_norm_nm', 'normal_load_n', 'min_separation_mm',
          'contact_power_w', 'normal_fx', 'normal_fy', 'normal_fz',
          'friction_fx', 'friction_fy', 'friction_fz')


def clean(value):
    """Keep missing/nonfinite values missing, never silently turn them into zero."""
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, list):
        return [clean(v) for v in value]
    if value is None or value == '':
        return None
    if isinstance(value, str):
        if value.lower() in ('true', 'false'):
            return value.lower() == 'true'
        try:
            number = float(value)
            return number if math.isfinite(number) else None
        except ValueError:
            return value
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def read_csv(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def load_study(directory):
    directory = Path(directory).resolve()
    manifest_path = directory/'study.json'
    if not manifest_path.is_file():
        raise ValueError(f'No study.json in {directory}')
    manifest = clean(json.loads(manifest_path.read_text(encoding='utf-8')))
    warnings, entries = [], {}
    summary = directory/'summary.csv'
    records = read_csv(summary) if summary.is_file() else []
    # Manifest is authoritative for invalid/incomplete status; summary may be older.
    records += manifest.get('completed_trials', []) + manifest.get('invalid_trials', [])
    for source in records:
        r = clean(source)
        if r.get('scenario') and r.get('recovery'):
            key = f"{r['scenario']}__{r['recovery']}"
            entries.setdefault(key, {}).update(r)
    for path in sorted(directory.glob('*__*.csv')):
        scenario, recovery = path.stem.rsplit('__', 1)
        entries.setdefault(path.stem, dict(scenario=scenario, recovery=recovery, status='incomplete'))
    if not summary.is_file():
        warnings.append('summary.csv is missing; available manifest outcomes and partial CSVs are shown.')
    trials, missing_raw = [], []
    for key, result in entries.items():
        # Never use study-controlled names to read outside the selected directory.
        if Path(key).name != key or '/' in key or '\\' in key:
            warnings.append(f'Skipped invalid trial name: {key}')
            continue
        status = result.get('status') or 'incomplete'
        eligible = status in COMPLETE and manifest.get('status') != 'invalid_sensor_configuration'
        series = {}
        path = directory/f'{key}.csv'
        if path.is_file():
            rows = read_csv(path)
            if rows:
                available = [k for k in SERIES if k in rows[0]]
                series = {k: [clean(r.get(k)) for r in rows] for k in available}
                # In-progress files can end with a half-written row. Do not join across it.
                if 'time_s' not in series:
                    warnings.append(f'{path.name} has no time_s column; raw plots unavailable.')
                    series = {}
        if not series:
            missing_raw.append(key)
        trials.append(dict(id=key, summary=result, status=status, eligible=eligible, series=series))
    if missing_raw:
        warnings.append(f'{len(missing_raw)} trial(s) have no raw CSV samples; summary comparisons and original figures remain accessible.')
    if not trials:
        warnings.append('No trial outcomes found. Showing the manifest and available original files.')
    assets = {}
    for name, mime in [('force_profiles.png', 'image/png'), ('force_vs_depth.png', 'image/png'),
                       ('force_profiles.pdf', 'application/pdf'), ('study.json', 'application/json'),
                       ('summary.csv', 'text/csv'), ('report.md', 'text/markdown')]:
        path = directory/name
        if path.is_file():
            assets[name] = f'data:{mime};base64,' + base64.b64encode(path.read_bytes()).decode('ascii')
    return dict(name=directory.name, manifest=manifest, trials=trials, assets=assets,
                report=(directory/'report.md').read_text(encoding='utf-8') if (directory/'report.md').is_file() else 'report.md is missing.',
                warnings=warnings)


def build_dashboard(directory, output=None):
    directory = Path(directory).resolve()
    data = load_study(directory)
    output = Path(output).resolve() if output else directory/'interactive_viewer.html'
    if output.suffix.lower() != '.html':
        raise ValueError('Output must have an .html extension')
    if output in (directory/'study.json', directory/'summary.csv', directory/'report.md'):
        raise ValueError('Output cannot replace study source data')
    bundle = (HERE/'vendor/plotly-basic-3.1.0.min.js').read_text(encoding='utf-8')
    payload = json.dumps(data, allow_nan=False, separators=(',', ':')).replace('<', '\\u003c')
    page = (HERE/'dashboard.html').read_text(encoding='utf-8')
    # One substitution pass: data must not be interpreted as template tokens.
    import re
    substitutions = {'TITLE': html.escape(data['name']), 'PLOTLY': bundle,
                     'DATA': payload, 'APP': (HERE/'dashboard.js').read_text(encoding='utf-8')}
    page = re.sub(r'@@(TITLE|PLOTLY|DATA|APP)@@', lambda match: substitutions[match[1]], page)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(page, encoding='utf-8')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('study', type=Path, help='Study directory, or its study.json file')
    parser.add_argument('--output', type=Path, help='HTML destination (default: STUDY/interactive_viewer.html)')
    parser.add_argument('--no-open', action='store_true', help='Generate without opening a browser')
    args = parser.parse_args()
    directory = args.study.parent if args.study.name == 'study.json' else args.study
    try:
        path = build_dashboard(directory, args.output)
    except (OSError, ValueError) as error:
        parser.exit(1, f'Cannot build viewer: {error}\n')
    print(f'Interactive study viewer: {path}')
    if not args.no_open and not webbrowser.open(path.as_uri()):
        print('Open that HTML file in your browser.')


if __name__ == '__main__':
    main()
