import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from research.forge_mechanics_contacts import (
    contact_statistics, cylinder_wall_overlap_estimate, hole_friction_for_average,
    polygon_wall_planes, rotate_vector, valid_contact_ranges,
)


def normal(load, point, direction=(-1., 0., 0.)):
    return {'normal_load_n': load, 'point_world_m': list(point),
            'normal_world': list(direction), 'separation_m': -.00001}


class MechanicsContactStatisticsTests(unittest.TestCase):
    def test_two_regions_weight_by_load_and_exclude_chamfer_floor(self):
        records = [normal(2., (.0042, 0., .010)), normal(1., (.0042, 0., .016)),
                   normal(3., (-.0042, 0., .020), (1., 0., 0.)),
                   normal(40., (.0042, 0., .0245)),
                   normal(30., (0., 0., 0.), (0., 0., 1.))]
        friction = [{'point_world_m': [0., 0., .011], 'force_world_n': [0., 0., -5.]}]
        row, raw_normal, raw_friction = contact_statistics(records, friction, (0., 0., 0.),
                                                           (1., 0., 0., 0.), .0042)
        self.assertEqual(row['wall_contact_count'], 3)
        self.assertEqual(row['wall_pos_normal_load_n'], 3.)
        self.assertEqual(row['wall_neg_normal_load_n'], 3.)
        self.assertEqual(row['wall_excluded_normal_load_n'], 70.)
        self.assertAlmostEqual(row['wall_pos_centroid_z_mm'], 12.)
        self.assertAlmostEqual(row['wall_centroid_axial_span_mm'], 8.)
        self.assertAlmostEqual(row['contact_axial_span_mm'], 10.)
        self.assertTrue(row['wall_both_sides_loaded'])
        self.assertEqual(len(raw_normal), 5)
        self.assertEqual(len(raw_friction), 1)
        self.assertNotIn('normal_load_n', raw_friction[0])
        json.dumps((row, raw_normal, raw_friction), allow_nan=False)

    def test_no_contact_schema_and_one_side_do_not_invent_two_point_span(self):
        empty, _, _ = contact_statistics([], [], (0., 0., 0.), (1., 0., 0., 0.), .0042)
        self.assertIsNone(empty['contact_axial_span_mm'])
        self.assertIsNone(empty['wall_centroid_axial_span_mm'])
        self.assertFalse(empty['wall_both_sides_loaded'])
        records = [normal(.05, (.0042, 0., .01)), normal(.05, (.0042, 0., .02))]
        one, _, _ = contact_statistics(records, [], (0., 0., 0.), (1., 0., 0., 0.), .0042)
        self.assertEqual(set(one), set(empty))
        self.assertTrue(one['contact_axial_span_valid'])
        self.assertFalse(one['wall_both_sides_loaded'])
        self.assertIsNone(one['wall_centroid_axial_span_mm'])

    def test_grouping_uses_hole_frame_and_explicit_axis(self):
        q = (math.sqrt(.5), 0., 0., math.sqrt(.5))
        position = (1., 2., 3.)
        local = (.0042, 0., .012)
        rotated = rotate_vector(q, local)
        record = normal(1., [v+p for v, p in zip(rotated, position)], rotate_vector(q, (-1., 0., 0.)))
        row, raw, _ = contact_statistics([record], [], position, q, .0042)
        self.assertTrue(row['wall_pos_loaded'])
        self.assertAlmostEqual(raw[0]['point_socket_m'][0], .0042)
        other, _, _ = contact_statistics([record], [], position, q, .0042, axis_xy=(0., 1.))
        self.assertFalse(other['wall_pos_loaded'])
        self.assertEqual(other['wall_other_normal_load_n'], 1.)

    def test_weak_contacts_remain_raw_but_do_not_mark_both_sides(self):
        records = [normal(1., (.0042, 0., .01)), normal(.001, (-.0042, 0., .02), (1., 0., 0.))]
        row, raw, _ = contact_statistics(records, [], (0., 0., 0.), (1., 0., 0., 0.), .0042)
        self.assertEqual(len(raw), 2)
        self.assertFalse(row['wall_both_sides_loaded'])

    def test_friction_targets_keep_peg_at_point75(self):
        for target, hole in ((.5, .25), (.75, .75), (1., 1.25)):
            self.assertEqual(hole_friction_for_average(target, target), (hole, hole))
        self.assertEqual(hole_friction_for_average(1., .5), (1.25, .25))
        for values in ((.2, .2), (.5, .75), (float('nan'), .5), (-1., -1.)):
            with self.assertRaises(ValueError):
                hole_friction_for_average(*values)

    def test_independent_ranges_refuse_capacity_and_overlap(self):
        self.assertEqual(valid_contact_ranges([2, 0], [1, -999], 8), [(1, 3)])
        self.assertEqual(valid_contact_ranges([1], [5], 8), [(5, 6)])
        for counts, starts in (([3], [5]), ([2, 2], [1, 2]), ([-1], [0])):
            with self.assertRaises((ValueError, RuntimeError)):
                valid_contact_ranges(counts, starts, 8)

    def test_geometry_is_signed_envelope_and_missing_when_outside(self):
        radius = .0042
        ring = [(radius*math.cos(i*math.tau/144), radius*math.sin(i*math.tau/144)) for i in range(144)]
        planes = polygon_wall_planes(ring)
        aligned = cylinder_wall_overlap_estimate((0., 0., .013), (0., 0., 1.), planes)
        self.assertTrue(aligned['geometry_overlap_estimate_valid'])
        self.assertEqual(aligned['geometry_overlap_estimate_mm'], 0.)
        self.assertAlmostEqual(aligned['effective_guide_span_estimate_mm'], 10.5)
        self.assertAlmostEqual(aligned['geometry_wall_signed_violation_estimate_mm'],
                               (.003993-radius*math.cos(math.pi/144))*1000)
        shifted = cylinder_wall_overlap_estimate((.0005, 0., .013), (0., 0., 1.), planes)
        self.assertGreater(shifted['geometry_overlap_estimate_mm'], .29)
        clear = cylinder_wall_overlap_estimate((0., 0., .030), (0., 0., 1.), planes)
        self.assertFalse(clear['geometry_overlap_estimate_valid'])
        self.assertIsNone(clear['geometry_overlap_estimate_mm'])


try:
    import torch
except ImportError:
    torch = None


@unittest.skipIf(torch is None, 'CPU torch needed for a mocked PhysX buffer test')
class MechanicsBufferTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Import the real new implementation with only the simulator imports
        # stubbed. No Isaac app, PhysX scene, CUDA context, or GPU is started.
        legacy = types.ModuleType('forge_backend')
        legacy.Bench = type('Bench', (), {})
        legacy.ForgeEnv = type('ForgeEnv', (), {})
        contact = types.ModuleType('contact')
        contact.ContactWrench = type('ContactWrench', (), {})
        pxr = types.ModuleType('pxr')
        for name in ('PhysxSchema', 'UsdGeom', 'UsdPhysics', 'UsdShade'):
            setattr(pxr, name, types.SimpleNamespace())
        path = Path(__file__).resolve().parents[1]/'simulation/forge_mechanics_backend.py'
        spec = importlib.util.spec_from_file_location('_mechanics_backend_cpu_test', path)
        cls.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'forge_backend': legacy, 'contact': contact, 'pxr': pxr}):
            spec.loader.exec_module(cls.module)

    def test_second_stream_reuses_every_buffer_without_corrupting_normal_snapshot(self):
        class View:
            def __init__(self):
                self.points = torch.zeros((8, 3))
                self.vectors = torch.zeros((8, 3))
                self.counts = torch.tensor([[2]], dtype=torch.int32)
                self.starts = torch.tensor([[1]], dtype=torch.int32)
                self.magnitude = torch.zeros((8, 1))
                self.separation = torch.zeros((8, 1))

            def get_contact_data(self, dt):
                self.points[1] = torch.tensor([1., 0., 0.])
                self.points[2] = torch.tensor([0., 2., 0.])
                self.vectors[1] = torch.tensor([-1., 0., 0.])
                self.vectors[2] = torch.tensor([0., -1., 0.])
                self.magnitude[1, 0] = 2.
                self.magnitude[2, 0] = 3.
                self.separation[1, 0] = -.0001
                return self.magnitude, self.points, self.vectors, self.separation, self.counts, self.starts

            def get_friction_data(self, dt):
                self.points.fill_(999.)
                self.vectors.fill_(999.)
                self.magnitude.fill_(999.)
                self.separation.fill_(999.)
                self.counts[0, 0] = 1
                self.starts[0, 0] = 5
                self.points[5] = torch.tensor([1., 0., 0.])
                self.vectors[5] = torch.tensor([0., 0., -4.])
                return self.vectors, self.points, self.counts, self.starts

        sensor = self.module.MechanicsContactWrench.__new__(self.module.MechanicsContactWrench)
        sensor.capacity = 8
        sensor.dt = 1/240
        sensor.view = View()
        value = sensor.read(torch.zeros(3))
        torch.testing.assert_close(value['normal_force'], torch.tensor([-2., -3., 0.]))
        torch.testing.assert_close(value['friction_force'], torch.tensor([0., 0., -4.]))
        torch.testing.assert_close(value['torque'], torch.tensor([0., 4., 0.]))
        self.assertEqual(value['normal_load_n'], 5.)
        self.assertEqual(value['contact_count'], 2)
        self.assertEqual(value['friction_count'], 1)
        self.assertAlmostEqual(value['min_separation_m'], -.0001)
        self.assertEqual(value['raw_normal_contacts'][0]['point_world_m'], [1., 0., 0.])
        self.assertEqual(value['raw_normal_contacts'][1]['normal_load_n'], 3.)
        self.assertEqual(len(value['raw_friction_contacts']), 1)


if __name__ == '__main__':
    unittest.main()
