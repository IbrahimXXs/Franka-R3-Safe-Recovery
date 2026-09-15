"""Mechanics-only FORGE adapter; legacy experiments and sensors are unchanged.

Construct this module only after AppLauncher. MechanicsBench construction is
single-threaded: the legacy constructor's environment factory is temporarily
replaced, then restored in finally. The subclass binds independent materials
in _setup_scene, before PhysX creates the shapes. No existing USD is edited.
"""
import copy
import math

import torch
from pxr import PhysxSchema, UsdGeom, UsdPhysics, UsdShade

import forge_backend as legacy
from contact import ContactWrench
from research.forge_geometry import PEG_RADIUS_M
from research.forge_mechanics_contacts import (
    POINT_LOAD_THRESHOLD_N, SIDE_LOAD_THRESHOLD_N, SIDE_SECTOR_COSINE,
    WALL_AXIAL_MARGIN_M, WALL_MAX_ABS_NORMAL_Z, WALL_RADIAL_BAND_M,
    contact_statistics, cylinder_wall_overlap_estimate, hole_friction_for_average,
    polygon_wall_planes, rotate_vector, socket_point, unit_axis_xy,
    valid_contact_ranges,
)


def _bound_material(prim):
    material, relationship = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
    return material, relationship


class _MechanicsForgeEnv(legacy.ForgeEnv):
    def _setup_scene(self):
        super()._setup_scene()
        if self.scene.num_envs != 1:
            raise RuntimeError('Mechanics material audit requires one environment')
        self.mechanics_material_bindings = {}
        stage = self.sim.stage
        for name, fragment in (('hole', '/FixedAsset/'), ('peg', '/HeldAsset/')):
            colliders = [p for p in stage.Traverse()
                         if str(p.GetPath()).startswith('/World/envs/env_0/')
                         and fragment in str(p.GetPath())
                         and p.HasAPI(UsdPhysics.CollisionAPI)]
            if len(colliders) != 1:
                raise RuntimeError(f'Expected one {name} collider, found {len(colliders)}')
            collider = colliders[0]
            if collider.IsInstanceProxy():
                raise RuntimeError('Cannot safely override an instance-proxy material')
            for child in collider.GetChildren():
                if child.IsA(UsdGeom.Subset) and _bound_material(child)[0]:
                    raise RuntimeError('Per-face physics materials need a separate audit')
            previous, _ = _bound_material(collider)
            previous_path = str(previous.GetPath()) if previous else None
            path = f'/World/MechanicsMaterials/{name}'
            material = UsdShade.Material.Define(stage, path)
            physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            physics.CreateStaticFrictionAttr(.75)
            physics.CreateDynamicFrictionAttr(.75)
            physics.CreateRestitutionAttr(0.)
            physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
            physx.CreateFrictionCombineModeAttr('average')
            physx.CreateRestitutionCombineModeAttr('average')
            physx.CreateCompliantContactStiffnessAttr(0.)
            physx.CreateCompliantContactDampingAttr(0.)
            UsdShade.MaterialBindingAPI.Apply(collider).Bind(
                material, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose='physics')
            actual, _ = _bound_material(collider)
            if not actual or str(actual.GetPath()) != path:
                raise RuntimeError(f'{name} material binding did not resolve to its independent material')
            self.mechanics_material_bindings[name] = {
                'collider_path': str(collider.GetPath()), 'material_path': path,
                'previous_bound_material_path': previous_path,
                'bound_before_physics_initialization': True,
            }


class MechanicsContactWrench(ContactWrench):
    """One normal/friction read, with detached valid-point copies for auditing."""

    def _pack(self, buffers, counts, starts):
        # The second PhysX API call may reuse the first call's point/count/start
        # buffers. Copy every valid normal field before calling friction_data.
        ranges = valid_contact_ranges(counts.flatten().tolist(), starts.flatten().tolist(), self.capacity)
        packed = []
        for buffer in buffers:
            if any(end > len(buffer) for _, end in ranges):
                raise RuntimeError('Contact range exceeds returned buffer')
            packed.append(torch.cat([buffer[start:end] for start, end in ranges], dim=0).clone()
                          if ranges else buffer[:0].clone())
        return packed

    @staticmethod
    def _sum_wrench(forces, points, origin):
        force = forces.sum(0)
        torque = torch.cross(points-origin, forces, dim=-1).sum(0)
        if not torch.isfinite(force).all() or not torch.isfinite(torque).all():
            raise RuntimeError('Non-finite mechanics contact wrench')
        return force, torque

    def read(self, origin):
        magnitudes, points, normals, separations, counts, starts = self.view.get_contact_data(self.dt)
        magnitude, point, normal, separation = self._pack(
            (magnitudes, points, normals, separations), counts, starts)
        magnitude = magnitude.reshape(-1)
        separation = separation.reshape(-1)
        if not torch.isfinite(magnitude).all() or (magnitude < 0).any():
            raise RuntimeError('Invalid scalar normal loads')
        normal_vectors = magnitude[:, None]*normal
        normal_force, normal_torque = self._sum_wrench(normal_vectors, point, origin)

        forces, points, counts, starts = self.view.get_friction_data(self.dt)
        friction_vectors, friction_points = self._pack((forces, points), counts, starts)
        friction_force, friction_torque = self._sum_wrench(friction_vectors, friction_points, origin)

        selected = magnitude > POINT_LOAD_THRESHOLD_N
        normal_rows = torch.cat((magnitude[selected, None], point[selected], normal[selected],
                                 separation[selected, None]), dim=1).cpu().tolist()
        selected = friction_vectors.norm(dim=-1) > POINT_LOAD_THRESHOLD_N
        friction_rows = torch.cat((friction_points[selected], friction_vectors[selected]), dim=1).cpu().tolist()
        raw_normal = [{'normal_load_n': r[0], 'point_world_m': r[1:4],
                       'normal_world': r[4:7], 'separation_m': r[7]} for r in normal_rows]
        raw_friction = [{'point_world_m': r[:3], 'force_world_n': r[3:6]} for r in friction_rows]
        value = {
            'force': normal_force+friction_force, 'torque': normal_torque+friction_torque,
            'normal_force': normal_force, 'friction_force': friction_force,
            'normal_torque': normal_torque, 'friction_torque': friction_torque,
            'contact_count': len(magnitude), 'friction_count': len(friction_vectors),
            'normal_load_n': magnitude.sum().item(),
            'min_separation_m': min(0., separation.min().item()) if len(separation) else 0.,
            'raw_normal_contacts': raw_normal, 'raw_friction_contacts': raw_friction,
        }
        self.last_value = value
        return value


class MechanicsBench(legacy.Bench):
    """Bench API plus per-case material readback and per-observation mechanics.

    set_case_material(pair_static_friction, pair_dynamic_friction) must precede
    prepare(seed). set_contact_axis((x,y)) chooses a fixed socket-frame grouping
    direction (default X for pitch). contact_snapshot() returns a detached JSON
    record for the most recent observe, ready for the runner's JSONL writer.
    """

    def __init__(self, args, app):
        self._contact_axis = (1., 0.)
        self._last_contact_snapshot = None
        original_environment = legacy.ForgeEnv
        if original_environment is not _MechanicsForgeEnv.__bases__[0]:
            raise RuntimeError('Unexpected concurrent FORGE environment factory override')
        legacy.ForgeEnv = _MechanicsForgeEnv
        try:
            super().__init__(args, app)
        finally:
            legacy.ForgeEnv = original_environment
        stage = self.sim.stage
        held = [str(p.GetPath()) for p in stage.Traverse()
                if str(p.GetPath()).startswith('/World/envs/env_0/')
                and '/HeldAsset/' in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        fixed = [str(p.GetPath()) for p in stage.Traverse()
                 if str(p.GetPath()).startswith('/World/envs/env_0/')
                 and '/FixedAsset/' in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(held) != 1 or len(fixed) != 1:
            raise RuntimeError('Unexpected mechanics rigid-body structure')
        self.contacts = MechanicsContactWrench(self.sim, held[0], fixed)
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath(self.env.mechanics_material_bindings['hole']['collider_path']))
        self._bore_radius = self.scene_info['measured_bore_diameter_mm']/2000
        ring = [p for p in mesh.GetPointsAttr().Get()
                if abs(p[2]) < 1e-6 and abs(math.hypot(p[0], p[1])-self._bore_radius) < 1e-6]
        if len(ring) != self.scene_info['bore_wall_sides']:
            raise RuntimeError('Could not read the audited straight-wall polygon')
        self._wall_planes = polygon_wall_planes(ring)
        self._peg_material_reference = self.env._held_asset.root_physx_view.get_material_properties().clone()
        self._robot_material_reference = self.env._robot.root_physx_view.get_material_properties().clone()
        for name, values in (('peg', self._peg_material_reference), ('robot', self._robot_material_reference)):
            if not torch.allclose(values[..., :2], torch.full_like(values[..., :2], .75), atol=1e-6, rtol=0):
                raise RuntimeError(f'{name} runtime friction is not the expected .75')
        self._requested_pair = (.75, .75)
        self._requested_hole = (.75, .75)
        self.material_state = self._read_material_state()
        self.scene_info['mechanics_materials'] = copy.deepcopy(self.material_state)
        self.scene_info['mechanics_contact_schema'] = {
            'version': 1, 'world_force_sign': 'fixture_on_peg; world +Z points upward',
            'torque_origin': 'peg rigid-body root, identical to legacy contact torque',
            'socket_origin': 'hole floor; straight wall z=0..24mm, mouth z=25mm',
            'point_load_threshold_n': POINT_LOAD_THRESHOLD_N,
            'side_load_threshold_n': SIDE_LOAD_THRESHOLD_N,
            'wall_axial_margin_m': WALL_AXIAL_MARGIN_M,
            'wall_radial_band_m': WALL_RADIAL_BAND_M,
            'wall_max_abs_normal_z': WALL_MAX_ABS_NORMAL_Z,
            'side_sector_cosine': SIDE_SECTOR_COSINE,
            'centroid_definition': 'normal-load weighted straight-wall regions; not two paired contacts',
            'geometry_estimate': 'full-radius analytic cylinder envelope against actual polygon walls; '
                'omits finite end caps and peg chamfers; not exact mesh penetration or material deformation',
            'timing': 'contact and post-step pose sampled without a global shift; API synchronization '
                'within the solver step is not independently established',
        }

    def _read_material_state(self):
        bindings = copy.deepcopy(self.env.mechanics_material_bindings)
        for name, expected in (('hole', self._requested_hole), ('peg', (.75, .75))):
            item = bindings[name]
            collider = self.sim.stage.GetPrimAtPath(item['collider_path'])
            material, _ = _bound_material(collider)
            if not material or str(material.GetPath()) != item['material_path']:
                raise RuntimeError(f'{name} no longer uses its independently bound material')
            physx = PhysxSchema.PhysxMaterialAPI(material.GetPrim())
            mode = str(physx.GetFrictionCombineModeAttr().Get())
            if mode != 'average':
                raise RuntimeError(f'{name} friction combine mode changed: {mode}')
            physics = UsdPhysics.MaterialAPI(material.GetPrim())
            usd_values = [physics.GetStaticFrictionAttr().Get(), physics.GetDynamicFrictionAttr().Get()]
            if any(not math.isclose(v, e, rel_tol=0, abs_tol=1e-6) for v, e in zip(usd_values, expected)):
                raise RuntimeError(f'{name} USD friction differs from requested material')
            item.update(usd_static_friction=usd_values[0], usd_dynamic_friction=usd_values[1],
                        resolved_friction_combine_mode=mode)
        runtime = {name: asset.root_physx_view.get_material_properties().clone()
                   for name, asset in (('hole', self.env._fixed_asset), ('peg', self.env._held_asset),
                                       ('robot', self.env._robot))}
        for name, reference in (('peg', self._peg_material_reference), ('robot', self._robot_material_reference)):
            if not torch.allclose(runtime[name], reference, atol=1e-6, rtol=0):
                raise RuntimeError(f'{name} material changed while varying only hole friction')
        hole = runtime['hole']
        if hole.shape[-2:] != (1, 3):
            raise RuntimeError('Expected exactly one hole shape for friction readback')
        expected = torch.tensor(self._requested_hole, dtype=hole.dtype, device=hole.device)
        if not torch.allclose(hole[..., :2], expected.expand_as(hole[..., :2]), atol=1e-6, rtol=0):
            raise RuntimeError('Hole runtime friction did not retain the requested values')
        if not torch.allclose(hole[..., 2], torch.zeros_like(hole[..., 2]), atol=1e-6, rtol=0):
            raise RuntimeError('Hole restitution unexpectedly changed')
        peg = runtime['peg']
        if peg.shape[-2:] != (1, 3):
            raise RuntimeError('Expected exactly one peg shape for friction readback')
        pair = ((hole[0, 0, :2]+peg[0, 0, :2])/2).tolist()
        if any(not math.isclose(v, e, rel_tol=0, abs_tol=1e-6) for v, e in zip(pair, self._requested_pair)):
            raise RuntimeError('Runtime pair friction differs from the requested average')
        return {'requested_pair_static_friction': self._requested_pair[0],
                'requested_pair_dynamic_friction': self._requested_pair[1],
                'effective_pair_static_friction': pair[0], 'effective_pair_dynamic_friction': pair[1],
                'effective_pair_basis': 'runtime per-shape coefficients + resolved explicitly bound average '
                    'material modes; not a separately measured contact-pair coefficient',
                'bindings': bindings,
                'runtime_materials': {name: values.tolist() for name, values in runtime.items()}}

    def set_case_material(self, pair_static_friction, pair_dynamic_friction):
        hole = hole_friction_for_average(pair_static_friction, pair_dynamic_friction)
        material_prim = self.sim.stage.GetPrimAtPath(
            self.env.mechanics_material_bindings['hole']['material_path'])
        physics = UsdPhysics.MaterialAPI(material_prim)
        physics.GetStaticFrictionAttr().Set(hole[0])
        physics.GetDynamicFrictionAttr().Set(hole[1])
        view = self.env._fixed_asset.root_physx_view
        values = view.get_material_properties().clone()
        values[..., 0] = hole[0]
        values[..., 1] = hole[1]
        view.set_material_properties(values, torch.arange(self.env.scene.num_envs, device='cpu'))
        self._requested_pair = (float(pair_static_friction), float(pair_dynamic_friction))
        self._requested_hole = hole
        self.material_state = self._read_material_state()
        self.scene_info['mechanics_materials'] = copy.deepcopy(self.material_state)
        self.scene_info['effective_assets']['fixed_asset']['materials'] = copy.deepcopy(
            self.material_state['runtime_materials']['hole'])
        return copy.deepcopy(self.material_state)

    def set_contact_axis(self, axis_xy):
        self._contact_axis = unit_axis_xy(axis_xy)

    def prepare(self, seed):
        self._read_material_state()
        initial = super().prepare(seed)
        self.material_state = self._read_material_state()
        return initial

    def observe(self, phase, command_depth):
        row = super().observe(phase, command_depth)
        contact = self.contacts.last_value
        socket_position = self.env._fixed_asset.data.root_pos_w[0].tolist()
        socket_quaternion = self.env._fixed_asset.data.root_quat_w[0].tolist()
        stats, raw_normal, raw_friction = contact_statistics(
            contact['raw_normal_contacts'], contact['raw_friction_contacts'],
            socket_position, socket_quaternion, self._bore_radius, self._contact_axis)
        row.update(stats)
        for kind in ('normal', 'friction'):
            for measure, unit in (('force', 'n'), ('torque', 'nm')):
                vector = contact[f'{kind}_{measure}'].tolist()
                row.update({f'{kind}_{measure}_world_{axis}_{unit}': value
                            for axis, value in zip('xyz', vector)})
                row[f'{kind}_{measure}_norm_{unit}'] = math.sqrt(sum(v*v for v in vector))
        row['friction_contact_count'] = contact['friction_count']
        row['effective_pair_static_friction'] = self.material_state['effective_pair_static_friction']
        row['effective_pair_dynamic_friction'] = self.material_state['effective_pair_dynamic_friction']
        peg_position = self.env._held_asset.data.root_pos_w[0].tolist()
        peg_quaternion = self.env._held_asset.data.root_quat_w[0].tolist()
        tip = socket_point(peg_position, socket_position, socket_quaternion)
        peg_axis = rotate_vector(socket_quaternion, rotate_vector(peg_quaternion, (0., 0., 1.)), inverse=True)
        row.update(cylinder_wall_overlap_estimate(tip, peg_axis, self._wall_planes, PEG_RADIUS_M))
        if not all(math.isfinite(v) for v in row.values() if isinstance(v, (int, float))):
            raise RuntimeError('Non-finite mechanics measurement')
        self._last_contact_snapshot = {
            'schema_version': 1, 'time_s': row['time_s'], 'physics_time_s': row['physics_time_s'],
            'phase': phase, 'command_depth_mm': command_depth,
            'origin_world_m': peg_position, 'socket_position_world_m': socket_position,
            'socket_quaternion_wxyz': socket_quaternion,
            'grouping_axis_socket_xy': list(self._contact_axis),
            'normal_contacts': raw_normal, 'friction_contacts': raw_friction,
            'normal_contact_count_all': contact['contact_count'],
            'friction_contact_count_all': contact['friction_count'],
        }
        return row

    def contact_snapshot(self):
        if self._last_contact_snapshot is None:
            raise RuntimeError('No mechanics observation has been sampled')
        return copy.deepcopy(self._last_contact_snapshot)
