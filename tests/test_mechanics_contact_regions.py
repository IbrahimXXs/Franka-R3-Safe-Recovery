import unittest

from simulation.analyze_mechanics_contact_regions import region_measurements


def point(load, x, z, normal=(-1., 0., 0.), y=0.):
    return {'normal_load_n': load, 'point_socket_m': [x, y, z], 'normal_socket': list(normal)}


class MechanicsContactRegionTests(unittest.TestCase):
    def test_upper_transition_load_is_retained_outside_strict_wall(self):
        result = region_measurements([point(2.5784, .004193, .024000019),
                                      point(.0111, .004193, .024000019)], .004193)
        self.assertAlmostEqual(result['upper_rim_band_normal_load_n'], 2.5895)
        self.assertEqual(result['inner_straight_wall_normal_load_n'], 0.)
        self.assertAlmostEqual(result['wall_rim_pos_normal_load_n'], 2.5895)
        self.assertFalse(result['wall_rim_both_sides_loaded'])
        self.assertIsNone(result['wall_rim_centroid_axial_span_mm'])
        self.assertEqual(result['all_loaded_normal_z_span_mm'], 0.)

    def test_opposite_inner_wall_and_rim_have_a_loaded_region_span(self):
        result = region_measurements([point(2., .004193, .024),
                                      point(.5, -.004193, .012, normal=(1., 0., 0.))], .004193)
        self.assertTrue(result['wall_rim_both_sides_loaded'])
        self.assertAlmostEqual(result['wall_rim_centroid_axial_span_mm'], 12.)
        self.assertEqual(result['upper_rim_band_normal_load_n'], 2.)
        self.assertEqual(result['inner_straight_wall_normal_load_n'], .5)
        self.assertEqual(result['region_load_partition_error_n'], 0.)

    def test_weak_remote_point_only_expands_the_low_threshold_span(self):
        result = region_measurements([point(1.3, .004193, .024), point(1.2, .004193, .024),
                                      point(.0001, -.004193, .002, normal=(1., 0., 0.))], .004193)
        self.assertAlmostEqual(result['all_loaded_normal_z_span_mm'], 22.)
        self.assertEqual(result['force_supported_normal_z_span_mm'], 0.)
        self.assertFalse(result['wall_rim_both_sides_loaded'])
        self.assertFalse(result['wall_rim_neg_loaded'])

    def test_far_rim_or_steep_normal_does_not_enter_wall_rim_sectors(self):
        result = region_measurements([point(3., .02, .024),
                                      point(4., -.004193, .024, normal=(.707, 0., .707)),
                                      point(2., .004693, .0245, normal=(-.707, 0., .707)),
                                      point(1., 0., 0., normal=(0., 0., 1.))], .004193)
        self.assertEqual(result['upper_rim_band_normal_load_n'], 7.)
        self.assertEqual(result['entrance_chamfer_region_normal_load_n'], 2.)
        self.assertEqual(result['other_normal_load_n'], 1.)
        self.assertEqual(result['wall_rim_eligible_normal_load_n'], 0.)
        self.assertFalse(result['wall_rim_both_sides_loaded'])

    def test_no_contacts_and_single_supported_point_do_not_invent_span(self):
        empty = region_measurements([], .004193)
        self.assertEqual(empty['all_loaded_normal_load_n'], 0.)
        self.assertIsNone(empty['all_loaded_normal_z_span_mm'])
        self.assertFalse(empty['all_loaded_normal_z_span_valid'])
        single = region_measurements([point(1., .004193, .024)], .004193)
        self.assertIsNone(single['force_supported_normal_z_span_mm'])
        self.assertFalse(single['force_supported_normal_z_span_valid'])
        self.assertEqual(set(empty), set(single))
        with self.assertRaises(ValueError):
            region_measurements([point(float('nan'), .004193, .024)], .004193)


if __name__ == '__main__':
    unittest.main()
