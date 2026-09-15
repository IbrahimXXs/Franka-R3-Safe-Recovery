import math
import unittest

from research.forge_mechanics_motion import WithdrawalProfile


class WithdrawalProfileTests(unittest.TestCase):
    def test_distances_share_actual_speed_ramp_and_only_cruise_duration_changes(self):
        profiles = [WithdrawalProfile(distance) for distance in (16., 22., 28.)]
        for p, expected_duration in zip(profiles, (3.7, 4.9, 6.1)):
            self.assertAlmostEqual(p.duration_s, expected_duration)
            self.assertEqual(p.max_speed_mm_s, 5.)
            self.assertEqual(p.effective_ramp_s, .5)
            self.assertEqual(p.metadata['max_acceleration_mm_s2'], 15.)
        for time in (0., .125, .25, .5, 1., 2.):
            self.assertEqual(len({p.position_mm(time) for p in profiles}), 1)
            self.assertEqual(len({p.speed_mm_s(time) for p in profiles}), 1)
        self.assertAlmostEqual(profiles[0].position_mm(.5), 1.25)

    def test_position_and_velocity_are_continuous_at_all_phase_boundaries(self):
        p = WithdrawalProfile(16.)
        for time in (0., p.ramp_s, p.duration_s - p.ramp_s, p.duration_s):
            epsilon = 1e-7
            self.assertAlmostEqual(p.position_mm(time - epsilon), p.position_mm(time + epsilon), delta=2e-6)
            self.assertAlmostEqual(p.speed_mm_s(time - epsilon), p.speed_mm_s(time + epsilon), delta=2e-6)
        self.assertEqual(p.speed_mm_s(0.), 0.)
        self.assertEqual(p.speed_mm_s(p.duration_s), 0.)
        self.assertEqual(p.position_mm(p.duration_s), 16.)

    def test_speed_integral_and_position_derivative_match_displacement(self):
        for distance in (0.25, 2.5, 16., 28.):
            p = WithdrawalProfile(distance)
            steps = 10000
            dt = p.duration_s / steps
            integral = sum((p.speed_mm_s(i * dt) + p.speed_mm_s((i + 1) * dt)) * dt / 2
                           for i in range(steps))
            self.assertAlmostEqual(integral, distance, delta=1e-6)
            positions = [p.position_mm(i * dt) for i in range(steps + 1)]
            self.assertTrue(all(a <= b + 1e-12 for a, b in zip(positions, positions[1:])))
            for fraction in (.1, .25, .5, .75, .9):
                time = fraction * p.duration_s
                epsilon = 1e-6
                derivative = (p.position_mm(time + epsilon) - p.position_mm(time - epsilon)) / (2 * epsilon)
                self.assertAlmostEqual(derivative, p.speed_mm_s(time), delta=1e-6)

    def test_short_move_preserves_acceleration_scale_and_lowers_peak(self):
        p = WithdrawalProfile(.25)
        self.assertEqual(p.cruise_duration_s, 0.)
        self.assertLess(p.max_speed_mm_s, 5.)
        self.assertLess(p.ramp_s, .5)
        self.assertAlmostEqual(p.max_speed_mm_s / p.ramp_s, 5. / .5)
        self.assertAlmostEqual(p.max_speed_mm_s * p.ramp_s, .25)
        self.assertAlmostEqual(p.position_mm(p.ramp_s), .125)
        self.assertAlmostEqual(p.speed_mm_s(p.ramp_s), p.max_speed_mm_s)
        self.assertTrue(p.metadata['short_move'])
        boundary = WithdrawalProfile(2.5)
        self.assertEqual(boundary.cruise_duration_s, 0.)
        self.assertEqual(boundary.ramp_s, .5)
        self.assertEqual(boundary.max_speed_mm_s, 5.)
        self.assertFalse(boundary.metadata['short_move'])

    def test_zero_distance_is_complete_and_times_outside_profile_are_clamped(self):
        zero = WithdrawalProfile(0.)
        self.assertEqual(zero.duration_s, 0.)
        for time in (-1., 0., 1.):
            self.assertEqual(zero.position_mm(time), 0.)
            self.assertEqual(zero.speed_mm_s(time), 0.)
        p = WithdrawalProfile(5.)
        self.assertEqual(p.position_mm(-1.), 0.)
        self.assertEqual(p.position_mm(100.), 5.)
        self.assertEqual(p.speed_mm_s(100.), 0.)

    def test_invalid_parameters_and_nonfinite_times_are_rejected(self):
        for args in ((-1.,), (math.nan,), (1., 0.), (1., -1.), (1., 5., 0.), (1., 5., math.inf)):
            with self.assertRaises(ValueError):
                WithdrawalProfile(*args)
        p = WithdrawalProfile(1.)
        for time in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                p.position_mm(time)
            with self.assertRaises(ValueError):
                p.speed_mm_s(time)


if __name__ == '__main__':
    unittest.main()
