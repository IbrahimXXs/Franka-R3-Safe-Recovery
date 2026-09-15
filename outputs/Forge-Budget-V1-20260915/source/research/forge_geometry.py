"""Pure geometry checks for the audited Factory peg/socket (all points in metres).

The requested clearance is a *radial vertex* clearance. The actual conservative
clearance is slightly smaller because the circular hole uses polygonal walls.
No USD or simulator imports are required by this module.
"""
import hashlib
import json
import math


PEG_RADIUS_M = .003993
BORE_RADIUS_M = .0045
BASELINE_RADIAL_CLEARANCE_MM = .507
SOCKET_RING_VERTICES = 144
GEOMETRY_TOLERANCE_M = 1e-6


def validate_radial_clearance(clearance_mm):
    value = float(clearance_mm)
    if not math.isfinite(value) or not 0. < value <= BASELINE_RADIAL_CLEARANCE_MM:
        raise ValueError('Radial clearance must be finite and in (0, 0.507] mm')
    return value


def mesh_sha256(points, face_counts=(), face_indices=()):
    """Hash exact numeric geometry, independent of USD paths and asset names."""
    data = {'points': [[float(v) for v in p] for p in points],
            'face_counts': [int(v) for v in face_counts],
            'face_indices': [int(v) for v in face_indices]}
    return hashlib.sha256(json.dumps(data, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _ring(points, z_m, radius_m):
    result = [(i, p) for i, p in enumerate(points)
              if abs(p[2] - z_m) <= GEOMETRY_TOLERANCE_M
              and abs(math.hypot(p[0], p[1]) - radius_m) <= GEOMETRY_TOLERANCE_M]
    if len(result) != SOCKET_RING_VERTICES:
        raise ValueError(f'Expected {SOCKET_RING_VERTICES} socket ring vertices at '
                         f'z={z_m}, r={radius_m}; found {len(result)}')
    # A repeated/missing sector must not pass merely by matching the point count.
    angles = sorted(math.atan2(p[1], p[0]) for _, p in result)
    gaps = [(angles[(i + 1) % len(angles)] - a) % (2 * math.pi)
            for i, a in enumerate(angles)]
    expected = 2 * math.pi / SOCKET_RING_VERTICES
    if any(abs(gap - expected) > 1e-4 for gap in gaps):
        raise ValueError('Socket ring has duplicate, missing, or nonuniform angular sectors')
    return result


def _inradius(ring):
    points = sorted((p for _, p in ring), key=lambda p: math.atan2(p[1], p[0]))
    distances = []
    for i, a in enumerate(points):
        b = points[(i + 1) % len(points)]
        edge_length = math.hypot(b[0] - a[0], b[1] - a[1])
        distances.append(abs(a[0] * b[1] - a[1] * b[0]) / edge_length)
    return min(distances)


def audit_socket_points(points, peg_radius_m=PEG_RADIUS_M,
                        expected_bore_radius_m=BORE_RADIUS_M):
    """Read back bore dimensions and the most restrictive polygonal wall gap."""
    bottom = _ring(points, 0., expected_bore_radius_m)
    top = _ring(points, .024, expected_bore_radius_m)
    chamfer = _ring(points, .025, expected_bore_radius_m + .001)
    radii = [math.hypot(p[0], p[1]) for _, p in bottom + top]
    inradius = min(_inradius(bottom), _inradius(top))
    clearance = (inradius - peg_radius_m) * 1000
    if not math.isfinite(peg_radius_m) or peg_radius_m <= 0 or clearance <= 0:
        raise ValueError('Measured peg and socket must have a positive conservative clearance')
    return {'measured_bore_diameter_mm': 2 * sum(radii) / len(radii) * 1000,
            'measured_peg_diameter_mm': 2 * peg_radius_m * 1000,
            'measured_radial_vertex_clearance_mm': (min(radii) - peg_radius_m) * 1000,
            'measured_bore_inradius_mm': inradius * 1000,
            'conservative_radial_clearance_mm': clearance,
            'bore_wall_sides': len(top),
            'socket_chamfer_axial_mm': 1.,
            'socket_chamfer_radial_mm':
                (sum(math.hypot(p[0], p[1]) for _, p in chamfer) / len(chamfer)
                 - sum(math.hypot(p[0], p[1]) for _, p in top) / len(top)) * 1000}


def resize_socket_points(points, radial_clearance_mm):
    """Shrink only the two bore rings and chamfer rim of the audited mesh.

    The same radial delta on the bore and rim preserves the 1 mm chamfer.
    Outer vertices, z coordinates, topology, peg dimensions and baseline point
    values are untouched. Refuse unrecognized upstream geometry rather than
    changing arbitrary nearby mesh vertices.
    """
    clearance = validate_radial_clearance(radial_clearance_mm)
    original = [[float(v) for v in p] for p in points]
    audit_socket_points(original)
    target_radius = PEG_RADIUS_M + clearance / 1000
    delta = target_radius - BORE_RADIUS_M
    if math.isclose(clearance, BASELINE_RADIAL_CLEARANCE_MM, abs_tol=1e-12):
        return original
    edited = [p[:] for p in original]
    for z, radius in ((0., BORE_RADIUS_M), (.024, BORE_RADIUS_M),
                      (.025, BORE_RADIUS_M + .001)):
        for i, p in _ring(original, z, radius):
            r = math.hypot(p[0], p[1])
            scale = (r + delta) / r
            edited[i] = [p[0] * scale, p[1] * scale, p[2]]
    audit_socket_points(edited, expected_bore_radius_m=target_radius)
    return edited
