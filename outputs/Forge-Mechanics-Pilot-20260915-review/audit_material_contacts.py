#!/usr/bin/env python3
"""Read closed mechanics CSV/gzip streams; write only the requested audit JSON.

Usage: python3 audit_material_contacts.py --study-dir ../Forge-Mechanics-Pilot-20260915
    --case m_d06_fs075_fd075_a00_try00 --profiles insertion final_retreat
    --output material_contact_audit.json
No simulator, GPU, or third-party Python package is used.
"""
import argparse
import csv
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
from datetime import datetime, timezone


def subtract(a, b):
    return [x-y for x, y in zip(a, b)]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def rotate(q, v, inverse=False):
    length = math.sqrt(sum(x*x for x in q))
    w, x, y, z = [t/length for t in q]
    if inverse:
        x, y, z = -x, -y, -z
    matrix = ((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)),
              (2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)),
              (2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)))
    return [sum(a*b for a, b in zip(row, v)) for row in matrix]


def error(a, b):
    return max(abs(x-y) for x, y in zip(a, b))


def audit_profile(csv_path, raw_path, scene, physics_hz):
    csv_bytes, raw_bytes = csv_path.read_bytes(), raw_path.read_bytes()
    if not csv_bytes.endswith(b'\n'):
        raise ValueError('CSV does not end with a complete record')
    # gzip.decompress verifies EOF and CRC; an unfinished live stream is refused.
    raw_text = gzip.decompress(raw_bytes)
    rows = list(csv.DictReader(io.StringIO(csv_bytes.decode())))
    records = [json.loads(line) for line in raw_text.splitlines()]
    if len(rows) != len(records):
        raise ValueError('CSV and closed gzip have different sample counts')
    if not rows:
        raise ValueError('No profile samples')
    schema = scene['mechanics_contact_schema']
    radius = scene['measured_bore_diameter_mm']/2000
    maxima = {key: 0. for key in (
        'force_additivity_n', 'torque_additivity_nm', 'raw_normal_n', 'raw_friction_n',
        'raw_normal_torque_nm', 'raw_friction_torque_nm', 'raw_load_n', 'point_transform_m',
        'tip_x_mm', 'tip_y_mm', 'depth_mm', 'wall_load_n', 'wall_centroid_mm',
        'contact_axial_span_mm', 'raw_csv_time_s', 'raw_csv_physics_time_s',
        'snapshot_outer_time_s', 'snapshot_outer_physics_time_s', 'recording_time_s',
        'physics_step_error_s', 'physics_vs_state_clock_s')}
    failures = []
    copies = []
    normal_loaded = friction_loaded = both_sides = 0
    point_counts = {'normal_all_max': 0, 'friction_all_max': 0,
                    'normal_loaded_max': 0, 'friction_loaded_max': 0}
    first_loaded = None
    def update(key, value):
        maxima[key] = max(maxima[key], abs(value))
    for index, (row, outer) in enumerate(zip(rows, records)):
        sample = outer['contacts']
        ns, fs = sample['normal_contacts'], sample['friction_contacts']
        normal_loaded += bool(ns)
        friction_loaded += bool(fs)
        both_sides += row['wall_both_sides_loaded'] == 'True'
        for key, value in (('normal_all_max', sample['normal_contact_count_all']),
                           ('friction_all_max', sample['friction_contact_count_all']),
                           ('normal_loaded_max', len(ns)), ('friction_loaded_max', len(fs))):
            point_counts[key] = max(point_counts[key], value)
        update('raw_csv_time_s', outer['time_s']-float(row['time_s']))
        update('raw_csv_physics_time_s', outer['physics_time_s']-float(row['physics_time_s']))
        update('snapshot_outer_time_s', sample['time_s']-outer['time_s'])
        update('snapshot_outer_physics_time_s', sample['physics_time_s']-outer['physics_time_s'])
        update('recording_time_s', outer['recording_time_s']-float(row.get('recovery_time_s') or row['time_s']))
        update('physics_vs_state_clock_s', float(row['physics_time_s'])-float(row['time_s']))
        if index:
            update('physics_step_error_s', float(row['physics_time_s'])-float(rows[index-1]['physics_time_s'])-1/physics_hz)
        if outer['phase'] != row['phase']:
            failures.append([index, 'outer_phase'])
        if sample['phase'] != outer['phase']:
            if index == 0 and row.get('recovery_time_s') == '0.0' and outer['phase'] == 'stop':
                copies.append({'sample_index': 0, 'snapshot_phase': sample['phase'],
                               'csv_phase': 'stop', 'reason': 'same physical state copied from reference endpoint'})
            else:
                failures.append([index, 'snapshot_phase'])
        for axis in 'xyz':
            update('force_additivity_n', float(row[f'normal_force_world_{axis}_n'])
                   + float(row[f'friction_force_world_{axis}_n'])-float(row[f'f{axis}']))
        for axis, total in zip('xyz', ('taux', 'tauy', 'tauz')):
            update('torque_additivity_nm', float(row[f'normal_torque_world_{axis}_nm'])
                   + float(row[f'friction_torque_world_{axis}_nm'])-float(row[total]))
        for kind, stream in (('normal', ns), ('friction', fs)):
            force, torque = [0., 0., 0.], [0., 0., 0.]
            for point in stream:
                f = ([point['normal_load_n']*x for x in point['normal_world']]
                     if kind == 'normal' else point['force_world_n'])
                force = [a+b for a, b in zip(force, f)]
                torque = [a+b for a, b in zip(torque, cross(subtract(point['point_world_m'], sample['origin_world_m']), f))]
                local = rotate(sample['socket_quaternion_wxyz'],
                               subtract(point['point_world_m'], sample['socket_position_world_m']), True)
                update('point_transform_m', error(local, point['point_socket_m']))
            update(f'raw_{kind}_n', error(force, [float(row[f'{kind}_force_world_{a}_n']) for a in 'xyz']))
            update(f'raw_{kind}_torque_nm', error(torque, [float(row[f'{kind}_torque_world_{a}_nm']) for a in 'xyz']))
        update('raw_load_n', sum(point['normal_load_n'] for point in ns)-float(row['normal_load_n']))
        if (int(row['contact_count']) != sample['normal_contact_count_all'] or len(ns) > int(row['contact_count'])
                or int(row['friction_contact_count']) != sample['friction_contact_count_all']
                or len(fs) > int(row['friction_contact_count'])):
            failures.append([index, 'raw_count'])
        tip = rotate(sample['socket_quaternion_wxyz'], subtract(sample['origin_world_m'], sample['socket_position_world_m']), True)
        for key, expected in (('tip_x_mm', tip[0]*1000), ('tip_y_mm', tip[1]*1000), ('depth_mm', 25-tip[2]*1000)):
            update(key, float(row[key])-expected)
        groups, wall = {'pos': [], 'neg': [], 'other': []}, []
        for point in ns:
            x, y, z = point['point_socket_m']
            rho = math.hypot(x, y)
            inside = (schema['wall_axial_margin_m'] < z < .024-schema['wall_axial_margin_m']
                      and abs(rho-radius) <= schema['wall_radial_band_m']
                      and abs(point['normal_socket'][2]) <= schema['wall_max_abs_normal_z'])
            projection = (x*sample['grouping_axis_socket_xy'][0]+y*sample['grouping_axis_socket_xy'][1])/rho if rho else 0.
            side = 'pos' if projection >= schema['side_sector_cosine'] else (
                'neg' if projection <= -schema['side_sector_cosine'] else 'other')
            if inside != point['straight_wall'] or (side if inside else None) != point['wall_side']:
                failures.append([index, 'wall_classification'])
            if inside:
                groups[side].append(point)
                wall.append(point)
        for side, points in groups.items():
            load = sum(p['normal_load_n'] for p in points)
            update('wall_load_n', load-float(row[f'wall_{side}_normal_load_n']))
            if load:
                center = [sum(p['normal_load_n']*p['point_socket_m'][i] for p in points)*1000/load for i in range(3)]
                update('wall_centroid_mm', error(center, [float(row[f'wall_{side}_centroid_{a}_mm']) for a in 'xyz']))
        if row['contact_axial_span_valid'] == 'True':
            span = (max(p['point_socket_m'][2] for p in wall)-min(p['point_socket_m'][2] for p in wall))*1000
            update('contact_axial_span_mm', span-float(row['contact_axial_span_mm']))
        if first_loaded is None and ns:
            first_loaded = {'sample_index': index, 'time_s': float(row['time_s']),
                            'depth_mm': float(row['depth_mm']), 'raw_normal_count': len(ns),
                            'normal_count_all': sample['normal_contact_count_all'],
                            'raw_friction_count': len(fs), 'friction_count_all': sample['friction_contact_count_all'],
                            'wall_both_sides_loaded': row['wall_both_sides_loaded'] == 'True',
                            'contact_axial_span_mm': float(row['contact_axial_span_mm']) if row['contact_axial_span_mm'] else None,
                            'geometry_signed_estimate_mm': float(row['geometry_wall_signed_violation_estimate_mm'])
                            if row['geometry_wall_signed_violation_estimate_mm'] else None}
    # Verify that the closed input bytes did not change during the audit.
    if csv_path.read_bytes() != csv_bytes or raw_path.read_bytes() != raw_bytes:
        raise RuntimeError('Input changed during the audit; no result should be accepted')
    return {'csv_path': str(csv_path.resolve()), 'raw_path': str(raw_path.resolve()),
            'csv_sha256': hashlib.sha256(csv_bytes).hexdigest(),
            'raw_sha256': hashlib.sha256(raw_bytes).hexdigest(),
            'gzip_closed_crc_verified': True, 'input_bytes_stable_during_audit': True,
            'samples': len(rows), 'sample_index_range': [0, len(rows)-1],
            'state_time_range_s': [float(rows[0]['time_s']), float(rows[-1]['time_s'])],
            'normal_loaded_samples': normal_loaded, 'friction_loaded_samples': friction_loaded,
            'both_straight_wall_sides_loaded_samples': both_sides,
            'point_counts': point_counts, 'max_absolute_errors': maxima,
            'structural_mismatches': failures, 'copied_endpoint_snapshots': copies,
            'first_loaded_sample': first_loaded}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir', type=Path, required=True)
    parser.add_argument('--case', required=True)
    parser.add_argument('--profiles', nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    scene_bytes = (args.study_dir/'scene.json').read_bytes()
    scene = json.loads(scene_bytes)
    study = json.loads((args.study_dir/'study.json').read_text())
    attempt = next(a for a in study['attempts'] if a['trajectory_id'] == args.case)
    profiles = [audit_profile(args.study_dir/args.case/f'{p}.csv',
                              args.study_dir/args.case/f'{p}_contacts.jsonl.gz', scene, study['physics_hz'])
                for p in args.profiles]
    result = {'audit_schema': 'mechanics_material_contact_readonly_v1',
              'created_utc': datetime.now(timezone.utc).isoformat(),
              'scope': {'study_path': str(args.study_dir.resolve()), 'case': args.case,
                        'case_status_at_audit': attempt['status'], 'profiles': args.profiles,
                        'all_planned_cases_verified': False},
              'scene_sha256': hashlib.sha256(scene_bytes).hexdigest(),
              'scene_materials': scene['mechanics_materials'],
              'case_material_readback': attempt['material'],
              'socket_mesh_sha256': scene['socket_mesh_sha256'],
              'conservative_radial_clearance_mm': scene['conservative_radial_clearance_mm'],
              'profiles': profiles,
              'limits': [
                  'Only the named closed profiles and recorded material readbacks are checked; this is not a full-batch validation.',
                  'Effective pair friction is calculated from runtime shape coefficients and resolved explicitly bound average modes, not measured by a separate friction experiment.',
                  'Raw files retain only points or anchors above 1e-6 N. All-contact counts include unloaded entries; small reconstruction residuals are expected.',
                  'Normal points and friction anchors have independent counts and positions and are never matched by index.',
                  'Weighted wall centers describe loaded regions, not distinct paired point contacts. Coincident normal points can yield a zero all-point span.',
                  'Geometry estimate coordinates are checked against logged tip pose and hole floor. The analytic cylinder envelope is not an exact mesh penetration, deformation, or physical-accuracy criterion.',
                  'Timestamp consistency does not independently establish when PhysX contact data were computed within a step; no global shift is applied.',
                  'The first recovery snapshot is copied from the insertion endpoint without an extra simulation step; its inner phase retains the original phase.'
              ]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output': str(args.output), 'case': args.case,
                      'profiles': [{'samples': p['samples'], 'structural_mismatches': len(p['structural_mismatches'])}
                                   for p in profiles]}, indent=2))


if __name__ == '__main__':
    main()
