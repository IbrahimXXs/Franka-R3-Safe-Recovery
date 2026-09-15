"""Simulator-independent measurements for the mechanics experiment.

All input positions are in metres, quaternions are wxyz, and forces are N.
Normal and friction records are independent streams; their indices are never
paired.  Wall centroids are load-weighted regions, not inferred contact pairs.
"""
import math


POINT_LOAD_THRESHOLD_N = 1e-6
SIDE_LOAD_THRESHOLD_N = .01
WALL_AXIAL_MARGIN_M = .00005
WALL_RADIAL_BAND_M = .0005
WALL_MAX_ABS_NORMAL_Z = .15
SIDE_SECTOR_COSINE = .5


def hole_friction_for_average(pair_static, pair_dynamic, peg_friction=.75):
    """Invert the explicitly verified arithmetic-average combination rule."""
    values = tuple(float(v) for v in (pair_static, pair_dynamic, peg_friction))
    if any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError('Friction coefficients must be finite and nonnegative')
    ps, pd, peg = values
    if pd > ps:
        raise ValueError('Dynamic friction must not exceed static friction')
    result = (2*ps-peg, 2*pd-peg)
    if min(result) < 0:
        raise ValueError('Requested pair friction requires negative hole friction')
    return result


def valid_contact_ranges(counts, starts, capacity):
    """Validate each stream independently, refusing a possibly full buffer."""
    if len(counts) != len(starts):
        raise ValueError('Contact counts and starts have different lengths')
    ranges = []
    for count, start in zip(counts, starts):
        if int(count) != count or count < 0:
            raise ValueError('Invalid contact count')
        if count == 0:
            continue  # Unused start entries need not be initialized.
        if int(start) != start or start < 0 or start+count >= capacity:
            raise RuntimeError('Contact buffer at capacity or invalid range')
        span = (int(start), int(start+count))
        if any(span[0] < old[1] and old[0] < span[1] for old in ranges):
            raise RuntimeError('Overlapping contact ranges would duplicate force')
        ranges.append(span)
    return ranges


def unit_axis_xy(axis):
    if len(axis) != 2 or any(not math.isfinite(float(v)) for v in axis):
        raise ValueError('Contact grouping axis must be a finite XY pair')
    length = math.hypot(*axis)
    if length <= 0:
        raise ValueError('Contact grouping axis must be nonzero')
    return tuple(float(v)/length for v in axis)


def rotate_vector(quaternion, vector, inverse=False):
    norm = math.sqrt(sum(float(v)**2 for v in quaternion))
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError('Invalid quaternion')
    w, x, y, z = [float(v)/norm for v in quaternion]
    if inverse:
        x, y, z = -x, -y, -z
    a, b, c = vector
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return [a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx]


def socket_point(point_world, socket_position, socket_quaternion):
    return rotate_vector(socket_quaternion,
                         [p-o for p, o in zip(point_world, socket_position)], inverse=True)


def contact_statistics(normal_records, friction_records, socket_position,
                       socket_quaternion, bore_radius_m, axis_xy=(1., 0.)):
    """Classify loaded normal points and decorate each independent raw stream.

    Strict straight-wall membership excludes the floor, entry chamfer, normals
    with an appreciable axial component, and points away from the inner bore.
    Fixed positive/negative 120-degree sectors avoid relabelling sides as the
    peg rotates. Other wall points remain available in the raw data.
    """
    axis = unit_axis_xy(axis_xy)
    normal_output, friction_output, wall = [], [], []
    groups = {'pos': [], 'neg': [], 'other': []}
    excluded_load = 0.
    for record in normal_records:
        load = float(record['normal_load_n'])
        if not math.isfinite(load) or load < 0:
            raise ValueError('Normal loads must be finite and nonnegative')
        if load <= POINT_LOAD_THRESHOLD_N:
            continue
        if any(not math.isfinite(float(v)) for v in
               (*record['point_world_m'], *record['normal_world'], record['separation_m'])):
            raise ValueError('Non-finite loaded normal contact')
        p = socket_point(record['point_world_m'], socket_position, socket_quaternion)
        n = rotate_vector(socket_quaternion, record['normal_world'], inverse=True)
        radius = math.hypot(p[0], p[1])
        straight = (WALL_AXIAL_MARGIN_M < p[2] < .024-WALL_AXIAL_MARGIN_M
                    and abs(radius-bore_radius_m) <= WALL_RADIAL_BAND_M
                    and abs(n[2]) <= WALL_MAX_ABS_NORMAL_Z)
        side = None
        if straight:
            projected = (p[0]*axis[0]+p[1]*axis[1])/radius
            side = 'pos' if projected >= SIDE_SECTOR_COSINE else (
                'neg' if projected <= -SIDE_SECTOR_COSINE else 'other')
        entry = dict(record, point_socket_m=p, normal_socket=n,
                     straight_wall=straight, wall_side=side)
        normal_output.append(entry)
        if straight:
            groups[side].append(entry)
            wall.append(entry)
        else:
            excluded_load += load
    for record in friction_records:
        if any(not math.isfinite(float(v)) for v in
               (*record['point_world_m'], *record['force_world_n'])):
            raise ValueError('Non-finite friction contact')
        magnitude = math.sqrt(sum(float(v)**2 for v in record['force_world_n']))
        if not math.isfinite(magnitude):
            raise ValueError('Friction force must be finite')
        if magnitude <= POINT_LOAD_THRESHOLD_N:
            continue
        friction_output.append(dict(record,
            point_socket_m=socket_point(record['point_world_m'], socket_position, socket_quaternion),
            force_socket_n=rotate_vector(socket_quaternion, record['force_world_n'], inverse=True)))

    row = {'wall_contact_count': len(wall), 'wall_excluded_normal_load_n': excluded_load,
           'wall_axis_x': axis[0], 'wall_axis_y': axis[1]}
    for side, records in groups.items():
        load = sum(r['normal_load_n'] for r in records)
        row[f'wall_{side}_normal_load_n'] = load
        row[f'wall_{side}_contact_count'] = len(records)
        row[f'wall_{side}_loaded'] = load >= SIDE_LOAD_THRESHOLD_N
        for i, name in enumerate('xyz'):
            row[f'wall_{side}_centroid_{name}_mm'] = (
                sum(r['normal_load_n']*r['point_socket_m'][i] for r in records)/load*1000
                if load else None)
            row[f'wall_{side}_normal_force_socket_{name}_n'] = sum(
                r['normal_load_n']*r['normal_socket'][i] for r in records)
    wall_load = sum(r['normal_load_n'] for r in wall)
    span_valid = len(wall) >= 2 and wall_load >= SIDE_LOAD_THRESHOLD_N
    both = row['wall_pos_loaded'] and row['wall_neg_loaded']
    row.update(wall_normal_load_n=wall_load,
               contact_axial_span_valid=span_valid,
               contact_axial_span_mm=(max(r['point_socket_m'][2] for r in wall)
                                      - min(r['point_socket_m'][2] for r in wall))*1000
                                     if span_valid else None,
               wall_both_sides_loaded=both,
               wall_centroid_axial_span_mm=abs(row['wall_pos_centroid_z_mm']
                                               - row['wall_neg_centroid_z_mm']) if both else None)
    return row, normal_output, friction_output


def polygon_wall_planes(ring_xy):
    """Return actual polygon wall outward XY unit normals and offsets (m)."""
    ring = sorted(([float(p[0]), float(p[1])] for p in ring_xy),
                  key=lambda p: math.atan2(p[1], p[0]))
    if len(ring) < 3:
        raise ValueError('At least three bore vertices are required')
    result = []
    for a, b in zip(ring, ring[1:]+ring[:1]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if length <= 0:
            raise ValueError('Duplicate bore vertices')
        n = ((b[1]-a[1])/length, (a[0]-b[0])/length)
        offset = n[0]*a[0]+n[1]*a[1]
        if offset <= 0:
            raise ValueError('Bore polygon must contain its origin')
        result.append((n[0], n[1], offset))
    return result


def cylinder_wall_overlap_estimate(tip_socket_m, axis_socket, wall_planes,
                                   peg_radius_m=.003993):
    """Fast *approximate envelope* violation; never a numerical-validity label.

    The full-radius Factory shaft spans local z=.5..49.5 mm. Its axis segment
    is clipped to the straight wall z=0..24 mm; infinite-cylinder elliptical
    cross-section support is evaluated at the clipped segment endpoints.
    This omits finite end-cap clipping and the .5 mm peg chamfers. It is neither
    an exact mesh intersection nor total penetration depth, and is evaluated
    from post-step poses without shifting the separate PhysX contact stream.
    """
    result = {'geometry_overlap_estimate_valid': False,
              'geometry_overlap_estimate_mm': None,
              'geometry_wall_signed_violation_estimate_mm': None,
              'effective_guide_span_estimate_mm': None}
    length = math.sqrt(sum(float(v)**2 for v in axis_socket))
    if not math.isfinite(length) or length <= 0:
        return result
    ux, uy, uz = [float(v)/length for v in axis_socket]
    if uz <= .5 or not wall_planes:
        return result
    lower = max(0., tip_socket_m[2]+.0005*uz)
    upper = min(.024, tip_socket_m[2]+.0495*uz)
    if upper <= lower:
        return result
    maximum = -math.inf
    for nx, ny, offset in wall_planes:
        tilt_projection = (nx*ux+ny*uy)/uz
        support = peg_radius_m*math.sqrt(1+tilt_projection**2)
        for z in (lower, upper):
            x = tip_socket_m[0]+(z-tip_socket_m[2])*ux/uz
            y = tip_socket_m[1]+(z-tip_socket_m[2])*uy/uz
            maximum = max(maximum, nx*x+ny*y+support-offset)
    result.update(geometry_overlap_estimate_valid=True,
                  geometry_overlap_estimate_mm=max(0., maximum)*1000,
                  geometry_wall_signed_violation_estimate_mm=maximum*1000,
                  effective_guide_span_estimate_mm=(upper-lower)*1000)
    return result
