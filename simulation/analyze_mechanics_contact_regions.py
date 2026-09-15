#!/usr/bin/env python3
"""Offline region audit for states selected by analyze_mechanics_states.py.

The frozen strict-wall metrics exclude the bore/entry transition at z=24 mm.
This supplementary export includes that rim without changing a simulation,
source measurement, safety label, or legacy metric. No simulator is imported.
"""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path

try:
    from .analyze_mechanics_states import load_profile, number
except ImportError:
    from analyze_mechanics_states import load_profile, number


POINT_LOAD_THRESHOLD_N = 1e-6
SIDE_LOAD_THRESHOLD_N = .01
SUPPORTED_POINT_LOAD_THRESHOLD_N = .01
RIM_HEIGHT_M = .024
RIM_HALF_WIDTH_M = .00005
MOUTH_HEIGHT_M = .025
RADIAL_BAND_M = .0005
MAX_ABS_NORMAL_Z = .15
SECTOR_COSINE = .5
REGIONS = ('inner_straight_wall', 'upper_rim_band', 'entrance_chamfer_region', 'other')


def _summary(points, prefix):
    """Point extrema use only the explicitly supplied point-load threshold."""
    load = sum(p['normal_load_n'] for p in points)
    z = [p['point_socket_m'][2]*1000 for p in points]
    result = {f'{prefix}_point_count': len(points), f'{prefix}_normal_load_n': load,
              f'{prefix}_max_point_normal_load_n': max((p['normal_load_n'] for p in points), default=None),
              f'{prefix}_z_min_mm': min(z) if z else None,
              f'{prefix}_z_max_mm': max(z) if z else None,
              f'{prefix}_z_span_valid': len(points) >= 2,
              f'{prefix}_z_span_mm': max(z)-min(z) if len(points) >= 2 else None}
    for i, axis in enumerate('xyz'):
        result[f'{prefix}_centroid_{axis}_mm'] = (
            sum(p['normal_load_n']*p['point_socket_m'][i] for p in points)*1000/load
            if load else None)
    return result


def region_measurements(normal_points, bore_radius_m):
    """Classify loaded normal points; friction anchors are deliberately unused.

    Regions form a disjoint partition. The rim label is an axial band, not a
    claim that a particular mesh face generated contact. The additional wall
    plus rim sectors require near-bore points and near-horizontal normals.
    """
    if not math.isfinite(bore_radius_m) or bore_radius_m <= 0:
        raise ValueError('A positive measured bore radius is required')
    loaded = []
    for point in normal_points:
        values = [point['normal_load_n'], *point['point_socket_m'], *point['normal_socket']]
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError('Non-finite raw normal point')
        if point['normal_load_n'] < 0:
            raise ValueError('Negative scalar normal load')
        if point['normal_load_n'] > POINT_LOAD_THRESHOLD_N:
            loaded.append(point)
    groups = {name: [] for name in REGIONS}
    sides = {name: [] for name in ('pos', 'neg', 'other')}
    eligible = []
    for point in loaded:
        x, y, z = point['point_socket_m']
        rho = math.hypot(x, y)
        near_bore = abs(rho-bore_radius_m) <= RADIAL_BAND_M
        horizontal = abs(point['normal_socket'][2]) <= MAX_ABS_NORMAL_Z
        in_rim_band = abs(z-RIM_HEIGHT_M) <= RIM_HALF_WIDTH_M
        inner_wall = (RIM_HALF_WIDTH_M < z < RIM_HEIGHT_M-RIM_HALF_WIDTH_M
                      and near_bore and horizontal)
        expected_chamfer_radius = bore_radius_m+max(0., min(.001, z-RIM_HEIGHT_M))
        entrance = (RIM_HEIGHT_M+RIM_HALF_WIDTH_M < z <= MOUTH_HEIGHT_M+RIM_HALF_WIDTH_M
                    and abs(rho-expected_chamfer_radius) <= RADIAL_BAND_M)
        # Give the entire transition band its own label before other regions.
        region = ('upper_rim_band' if in_rim_band else 'inner_straight_wall' if inner_wall
                  else 'entrance_chamfer_region' if entrance else 'other')
        groups[region].append(point)
        if (inner_wall or in_rim_band) and near_bore and horizontal:
            eligible.append(point)
            cosine = x/rho  # Fixed socket-X grouping, independent of peg tilt.
            side = 'pos' if cosine >= SECTOR_COSINE else ('neg' if cosine <= -SECTOR_COSINE else 'other')
            sides[side].append(point)
    result = _summary(loaded, 'all_loaded_normal')
    total_load = result['all_loaded_normal_normal_load_n']
    # Avoid the redundant word in the canonical overall load field.
    result['all_loaded_normal_load_n'] = result.pop('all_loaded_normal_normal_load_n')
    for region, points in groups.items():
        result.update(_summary(points, region))
        result[f'{region}_normal_load_fraction'] = (
            result[f'{region}_normal_load_n']/total_load if total_load else None)
    supported = [p for p in loaded if p['normal_load_n'] >= SUPPORTED_POINT_LOAD_THRESHOLD_N]
    result.update(_summary(supported, 'force_supported_normal'))
    result['force_supported_normal_load_n'] = result.pop('force_supported_normal_normal_load_n')
    result.update(_summary(eligible, 'wall_rim_eligible'))
    for side, points in sides.items():
        prefix = f'wall_rim_{side}'
        result.update(_summary(points, prefix))
        result[f'{prefix}_loaded'] = result[f'{prefix}_normal_load_n'] >= SIDE_LOAD_THRESHOLD_N
        for i, axis in enumerate('xyz'):
            result[f'{prefix}_normal_force_socket_{axis}_n'] = sum(
                p['normal_load_n']*p['normal_socket'][i] for p in points)
    both = result['wall_rim_pos_loaded'] and result['wall_rim_neg_loaded']
    result['wall_rim_both_sides_loaded'] = both
    result['wall_rim_centroid_axial_span_mm'] = (
        abs(result['wall_rim_pos_centroid_z_mm']-result['wall_rim_neg_centroid_z_mm']) if both else None)
    result['wall_rim_centroid_distance_mm'] = (math.sqrt(sum(
        (result[f'wall_rim_pos_centroid_{axis}_mm']-result[f'wall_rim_neg_centroid_{axis}_mm'])**2
        for axis in 'xyz')) if both else None)
    result['region_load_partition_error_n'] = sum(result[f'{name}_normal_load_n'] for name in REGIONS)-total_load
    return result


COPY_FIELDS = (
    'case_id', 'trajectory_id', 'folder', 'target_depth_mm', 'tilt_amplitude_deg',
    'pair_static_friction', 'pair_dynamic_friction', 'effective_radial_clearance_mm',
    'policy', 'state', 'state_observed', 'missing_reason', 'source_csv', 'sample_index',
    'phase_executed', 'phase_executed_sample_count', 'phase_observed_duration_s',
    'sample_is_copied_recovery_start', 'sample_is_reference_initialization',
    'phase', 'time_s', 'physics_time_s', 'recovery_time_s', 'depth_mm', 'tilt_deg',
    'tip_x_mm', 'tip_y_mm', 'wrist_force_n', 'normal_load_n', 'min_separation_mm',
    'grasp_slip_mm', 'grasp_slip_deg', 'geometric_guide_length_Lg_mm',
    'reference_numerically_valid', 'reference_grasp_retained', 'policy_label_eligible',
    'policy_replay_matched', 'policy_safe_recovery',
)
LEGACY_FIELDS = {
    'wall_both_sides_loaded': 'legacy_strict_wall_both_sides_loaded',
    'wall_centroid_axial_span_mm': 'legacy_strict_wall_centroid_axial_span_mm',
    'contact_axial_span_valid': 'legacy_strict_wall_point_z_span_valid',
    'contact_axial_span_mm': 'legacy_strict_wall_point_z_span_mm',
    'wall_excluded_normal_load_n': 'legacy_strict_wall_excluded_normal_load_n',
}


def export_regions(study_dir, output_dir, states_path=None):
    study_dir, output_dir = Path(study_dir).resolve(), Path(output_dir).resolve()
    if output_dir == study_dir or study_dir in output_dir.parents:
        raise ValueError('Region audit must be written outside the simulation study')
    states_path = Path(states_path).resolve() if states_path else output_dir/'mechanics_states.csv'
    states, states_hash = load_profile(states_path)
    scene_path = study_dir/'scene.json'
    scene_bytes = scene_path.read_bytes()
    scene = json.loads(scene_bytes)
    radius = float(scene['measured_bore_diameter_mm'])/2000
    schema = scene.get('mechanics_contact_schema', {})
    if scene.get('stage_meters_per_unit') != 1. or scene.get('stage_up_axis') != 'Z':
        raise ValueError('Region positions require the audited metre-scale Z-up scene')
    if not math.isclose(float(schema.get('point_load_threshold_n', POINT_LOAD_THRESHOLD_N)),
                        POINT_LOAD_THRESHOLD_N, rel_tol=0, abs_tol=1e-15):
        raise ValueError('Raw file point threshold differs from the declared region audit threshold')
    requests = {}
    for state in states:
        if state.get('state_observed') is not True:
            continue
        index = number(state, 'sample_index')
        if index is None or index < 0 or index != int(index):
            raise ValueError('Observed state has no valid source sample index')
        source = study_dir/str(state['folder'])/Path(state['source_csv']).name
        raw_path = source.with_name(source.stem+'_contacts.jsonl.gz').resolve()
        if study_dir not in raw_path.parents:
            raise ValueError('State refers to a profile outside the specified study')
        requests.setdefault(raw_path, set()).add(int(index))
    snapshots, raw_hashes, raw_counts = {}, {}, {}
    for path, indices in requests.items():
        data = path.read_bytes()
        # EOF/CRC verification refuses unfinished streams, even if their prefix
        # happens to contain a selected state. Only named completed states count.
        decoded = gzip.decompress(data)
        count = 0
        for index, line in enumerate(decoded.splitlines()):
            count += 1
            if index in indices:
                snapshots[(path, index)] = json.loads(line)
        if any((path, index) not in snapshots for index in indices):
            raise ValueError(f'Selected sample is absent from closed raw file: {path}')
        raw_hashes[str(path)] = hashlib.sha256(data).hexdigest()
        raw_counts[str(path)] = count
    template = region_measurements([], radius)
    output = []
    for state in states:
        row = {key: state.get(key) for key in COPY_FIELDS}
        row.update({new: state.get(old) for old, new in LEGACY_FIELDS.items()})
        row.update({key: None for key in template})
        row.update(region_state_observed=False, source_raw=None, raw_normal_contact_count_all=None,
                   raw_loaded_normal_count=None, source_csv_minus_raw_normal_load_n=None)
        if state.get('state_observed') is not True:
            output.append(row)
            continue
        source = study_dir/str(state['folder'])/Path(state['source_csv']).name
        raw_path = source.with_name(source.stem+'_contacts.jsonl.gz').resolve()
        index = int(state['sample_index'])
        outer = snapshots[(raw_path, index)]
        snapshot = outer['contacts']
        for key in ('time_s', 'physics_time_s'):
            expected = number(state, key)
            if expected is None or not math.isclose(float(outer[key]), expected, rel_tol=0, abs_tol=1e-8):
                raise ValueError(f'State/raw timestamp mismatch: {raw_path}, sample {index}, {key}')
        if outer['phase'] != state['phase']:
            raise ValueError(f'State/raw phase mismatch: {raw_path}, sample {index}')
        values = region_measurements(snapshot['normal_contacts'], radius)
        count_all = int(snapshot['normal_contact_count_all'])
        if len(snapshot['normal_contacts']) > count_all:
            raise ValueError('Loaded normal count exceeds all reported normal contacts')
        difference = number(state, 'normal_load_n')-values['all_loaded_normal_load_n'] \
            if number(state, 'normal_load_n') is not None else None
        if difference is not None and abs(difference) > count_all*POINT_LOAD_THRESHOLD_N+1e-5:
            raise ValueError(f'State/raw normal load mismatch: {raw_path}, sample {index}')
        row.update(values)
        row.update(region_state_observed=True, source_raw=str(raw_path),
                   raw_normal_contact_count_all=count_all,
                   raw_loaded_normal_count=len(snapshot['normal_contacts']),
                   source_csv_minus_raw_normal_load_n=difference)
        output.append(row)
    # Catch accidental use of a writer that changed any input while we read it.
    if hashlib.sha256(states_path.read_bytes()).hexdigest() != states_hash:
        raise RuntimeError('Selected states changed during region extraction')
    for path, digest in raw_hashes.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'Raw profile changed during region extraction: {path}')
    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir/'mechanics_contact_regions.csv'
    fields = COPY_FIELDS+tuple(LEGACY_FIELDS.values())+tuple(template)+(
        'region_state_observed', 'source_raw', 'raw_normal_contact_count_all',
        'raw_loaded_normal_count', 'source_csv_minus_raw_normal_load_n')
    with destination.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in output:
            writer.writerow({key: ('true' if value else 'false') if type(value) is bool else value
                             for key, value in row.items()})
    metadata = {
        'schema': 'mechanics_contact_regions_v1', 'study_dir': str(study_dir),
        'states_path': str(states_path), 'states_sha256': states_hash,
        'scene_path': str(scene_path), 'scene_sha256': hashlib.sha256(scene_bytes).hexdigest(),
        'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'state_parser_sha256': hashlib.sha256(Path(__file__).with_name('analyze_mechanics_states.py').read_bytes()).hexdigest(),
        'socket_mesh_sha256': scene.get('socket_mesh_sha256'),
        'measured_bore_radius_mm': radius*1000,
        'states_exported': len(output), 'states_observed': sum(row['region_state_observed'] for row in output),
        'case_ids': sorted({str(row['case_id']) for row in output}),
        'scope': 'Only selected states in the supplied mechanics_states.csv; not all frames or all planned cases.',
        'output_csv': str(destination), 'raw_sha256': raw_hashes,
        'raw_file_sample_counts': raw_counts, 'raw_gzip_closed_crc_verified': True,
        'coordinates': 'socket-local metres in raw data; exported positions and spans in mm; z=0 floor, z=24mm transition, z=25mm mouth',
        'thresholds': {'retained_point_normal_load_gt_n': POINT_LOAD_THRESHOLD_N,
                       'wall_rim_side_total_load_ge_n': SIDE_LOAD_THRESHOLD_N,
                       'force_supported_span_point_load_ge_n': SUPPORTED_POINT_LOAD_THRESHOLD_N,
                       'upper_rim_center_mm': RIM_HEIGHT_M*1000,
                       'upper_rim_half_width_mm': RIM_HALF_WIDTH_M*1000,
                       'near_bore_radial_band_mm': RADIAL_BAND_M*1000,
                       'near_horizontal_max_abs_normal_z': MAX_ABS_NORMAL_Z,
                       'fixed_x_sector_cosine': SECTOR_COSINE},
        'region_partition': {
            'upper_rim_band': 'all retained normal points in z=24mm±0.05mm, irrespective of normal direction/radius; an axial region label, not a mesh-face identification',
            'inner_straight_wall': 'outside rim band: 0.05<z<23.95mm, near-bore and near-horizontal normal',
            'entrance_chamfer_region': 'outside rim band: 24.05<z<=25.05mm and within radial band of the nominal 1:1 entry chamfer',
            'other': 'all remaining retained normal points, including floor or other non-wall contacts',
        },
        'span_definitions': {
            'all_loaded_normal_z_span_mm': 'max(z)-min(z) of all >1e-6N normal points; valid when at least two retained records, including possibly coincident points',
            'force_supported_normal_z_span_mm': 'same extrema only for individual points >=0.01N; valid with at least two such records; may omit diffuse load distributed over many weak points',
            'wall_rim_centroid_axial_span_mm': 'normal-load weighted positive/negative socket-X sector centers from near-bore, near-horizontal straight-wall OR upper-rim points; only if each side total >=0.01N',
            'legacy_strict_wall_point_z_span_mm': 'unchanged original contact_axial_span_mm field: strict-wall points only, despite the older generic name',
        },
        'limitations': [
            'Loaded region centers are estimates, not uniquely paired physical contacts or proof of a two-point locking mechanism.',
            'A false legacy_strict_wall_both_sides_loaded value cannot rule out rim/edge or other contacts.',
            'A false wall_rim_both_sides_loaded value only describes this selected sample and these sectors/thresholds; it is not a global no-jam label.',
            'The rim band intentionally catches the straight-wall/entrance transition excluded by the frozen 0.05mm wall margin; no frozen field, physical parameter, or label was changed.',
            'Friction anchors are not normal contacts and are neither classified here nor paired by index.',
            'Null means unobserved/undefined; zero load in an observed sample is an actual empty region. Tiny points can expand the unfiltered span; compare the separately thresholded span.',
            'Contact/pose synchronization within the solver step is not independently established; no global time shift is applied.',
        ],
    }
    (output_dir/'mechanics_contact_regions.schema.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--states-csv', type=Path, help='Default: OUTPUT_DIR/mechanics_states.csv')
    args = parser.parse_args()
    result = export_regions(args.study_dir, args.output_dir, args.states_csv)
    print(json.dumps({key: result[key] for key in ('states_exported', 'states_observed', 'case_ids', 'output_csv')}, indent=2))


if __name__ == '__main__':
    main()
