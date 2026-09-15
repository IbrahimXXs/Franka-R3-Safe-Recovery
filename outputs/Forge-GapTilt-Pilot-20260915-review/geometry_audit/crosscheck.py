#!/usr/bin/env python3
"""Offline mesh-vs-contact cross-check; does not run physics or change inputs.

Requirements: pxr (Usd/UsdGeom) plus the adjacent archived forge_geometry.py helper.
Use --phase retreat when selecting withdrawal peaks; otherwise stop-phase
residual insertion load could be selected as the whole-record maximum.
Assumes the current experiments' identity-oriented fixed hole. Logged tip_x/y
and depth reconstruct peg translation in the fixed-hole frame; logged peg
quaternion is relative because the fixed-hole orientation is identity.

Reported wall violations are positive when the transformed peg surface enters
one of the cavity wall halfspaces. They are not an overall mesh penetration
metric, a contact-solver error estimate, a material deformation, or a force.
Clipping triangles at the axial region boundaries includes shaft crossings
between the peg's endpoint rings. Comparing contact distances to post-step poses
also cannot establish when within a solver step each quantity was computed.
"""
import argparse
import hashlib
import csv
import json
import math
from pathlib import Path
import sys

from pxr import Usd, UsdGeom, Gf, Vt


def ring(points, z, radius):
    values = [tuple(p) for p in points if abs(p[2] - z) < 1e-6
              and abs(math.hypot(p[0], p[1]) - radius) < 1e-6]
    if len(values) != 144:
        raise ValueError(f'Expected 144 audited bore ring vertices; got {len(values)}')
    return sorted(values, key=lambda p: math.atan2(p[1], p[0]))


def planes_between(lower, upper):
    result = []
    for i, a in enumerate(lower):
        b = lower[(i + 1) % len(lower)]
        c = upper[i]
        u = [b[j] - a[j] for j in range(3)]
        v = [c[j] - a[j] for j in range(3)]
        normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        length = math.sqrt(sum(x*x for x in normal))
        normal = [x/length for x in normal]
        if normal[0]*a[0] + normal[1]*a[1] < 0:
            normal = [-x for x in normal]
        result.append((normal, sum(normal[j]*a[j] for j in range(3))))
    return result


def clip(poly, z, sign):
    result = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        ia, ib = sign*(a[2]-z) >= 0, sign*(b[2]-z) >= 0
        if ia:
            result.append(a)
        if ia != ib:
            t = (z-a[2])/(b[2]-a[2])
            result.append([a[j] + t*(b[j]-a[j]) for j in range(3)])
    return result


def transform(point, row):
    w, x, y, z = [float(row[k]) for k in ('qw','qx','qy','qz')]
    norm = math.sqrt(w*w+x*x+y*y+z*z)
    w, x, y, z = [v/norm for v in (w,x,y,z)]
    a, b, c = point
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return [a+w*tx+y*tz-z*ty+float(row['tip_x_mm'])/1000,
            b+w*ty+z*tx-x*tz+float(row['tip_y_mm'])/1000,
            c+w*tz+x*ty-y*tx+.025-float(row['depth_mm'])/1000]


def maximum_violation(points, faces, lower, upper, planes):
    maximum = None
    for face in faces:
        poly = clip([points[i] for i in face], lower, 1)
        if not poly:
            continue
        poly = clip(poly, upper, -1)
        for point in poly:
            violation = max(sum(n[j]*point[j] for j in range(3))-offset for n, offset in planes)
            maximum = violation if maximum is None else max(maximum, violation)
    return None if maximum is None else maximum*1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    archive = Path(__file__).resolve().parent
    parser.add_argument('--hole-usd', type=Path, default=archive/'assets/factory_hole_8mm.usd')
    parser.add_argument('--peg-usd', type=Path, default=archive/'assets/factory_peg_8mm.usd')
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--radial-clearance-mm', type=float, default=.507)
    parser.add_argument('--neighbors', type=int, default=0)
    parser.add_argument('--phase', choices=('all','approach','insert','hold','stop','realign','retreat','clear_hold'), default='all', help='Select extrema only within this recorded phase; neighboring frames keep their actual phases')
    parser.add_argument('--scene-json', type=Path, help='Require reconstructed hole hash to match this saved runtime scene')
    args = parser.parse_args()
    if args.neighbors < 0:
        parser.error('--neighbors must be nonnegative')
    # Use the archived helper, independent of future edits to the working repo.
    from forge_geometry import resize_socket_points, mesh_sha256
    # Keep stages alive; USD mesh wrappers do not retain the stage themselves.
    stages = [Usd.Stage.Open(str(path.resolve())) for path in (args.hole_usd, args.peg_usd)]
    meshes = [next(UsdGeom.Mesh(p) for p in s.Traverse() if p.IsA(UsdGeom.Mesh)) for s in stages]
    hole, peg = meshes
    hole_points = Vt.Vec3fArray([Gf.Vec3f(*p) for p in
        resize_socket_points(hole.GetPointsAttr().Get(), args.radial_clearance_mm)])
    radius = .003993 + args.radial_clearance_mm/1000
    bottom, bore_top, rim = [ring(hole_points,z,r) for z,r in ((0.,radius),(.024,radius),(.025,radius+.001))]
    regions = [('straight_wall',0.,.024,planes_between(bottom,bore_top)),
               ('chamfer_wall',.024,.025,planes_between(bore_top,rim))]
    peg_points = peg.GetPointsAttr().Get()
    indices = peg.GetFaceVertexIndicesAttr().Get()
    faces = []; offset = 0
    for count in peg.GetFaceVertexCountsAttr().Get():
        faces.append(indices[offset:offset+count]); offset += count
    with args.profile.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('Profile has no samples')
    selected = [i for i,r in enumerate(rows) if args.phase == 'all' or r['phase'] == args.phase]
    if not selected:
        raise ValueError(f'No samples in selected phase: {args.phase}')
    selections = {'max_reported_overlap': min(selected,key=lambda i:float(rows[i]['min_separation_mm'])),
                  'max_wrist_force': max(selected,key=lambda i:float(rows[i]['wrist_force_n']))}
    reconstructed_hash = mesh_sha256(hole_points,hole.GetFaceVertexCountsAttr().Get(),hole.GetFaceVertexIndicesAttr().Get())
    runtime_hash = None
    if args.scene_json is not None:
        runtime_hash = json.loads(args.scene_json.read_text())['socket_mesh_sha256']
        if runtime_hash != reconstructed_hash:
            raise ValueError('Reconstructed hole mesh hash differs from saved runtime scene')
    output = {'profile':str(args.profile.resolve()),'radial_clearance_mm':args.radial_clearance_mm,
              'hole_mesh_sha256':reconstructed_hash,
              'runtime_hole_mesh_sha256':runtime_hash,
              'peak_selection_phase':args.phase,
              'profile_sha256':hashlib.sha256(args.profile.read_bytes()).hexdigest(),
              'timing_caveat':'Adjacent-frame comparisons can suggest timing offsets but do not establish PhysX API timing; no global sample shift is applied',
              'assumption':'fixed hole identity orientation; metre-scale audited Factory geometry',
              'metric':'signed wall-halfspace violation in mm; positive enters socket wall; not full mesh penetration',
              'samples':[]}
    for label,index in selections.items():
        for i in range(max(0,index-args.neighbors),min(len(rows),index+args.neighbors+1)):
            row = rows[i]
            points = [transform(p,row) for p in peg_points]
            result = {'selection':label,'sample_index':i,'relative_sample_index':i-index,
                      'time_s':float(row['time_s']),'phase':row['phase'],
                      'recovery_time_s':float(row['recovery_time_s']) if row.get('recovery_time_s') else None,
                      'depth_mm':float(row['depth_mm']),
                      'command_depth_mm':float(row['command_depth_mm']),
                      'tilt_deg':float(row['tilt_deg']),
                      'contact_force_z_n':float(row['fz']),
                      'wrist_world_z_n':float(row['wrist_force_world_2']),
                      'solver_reported_overlap_mm':max(0.,-float(row['min_separation_mm'])),
                      'wrist_force_n':float(row['wrist_force_n']),
                      'contact_count':int(float(row['contact_count']))}
            for name,lower,upper,planes in regions:
                result[name+'_signed_violation_mm'] = maximum_violation(points,faces,lower,upper,planes)
            output['samples'].append(result)
    print(json.dumps(output,indent=2,allow_nan=False))


if __name__ == '__main__':
    main()
