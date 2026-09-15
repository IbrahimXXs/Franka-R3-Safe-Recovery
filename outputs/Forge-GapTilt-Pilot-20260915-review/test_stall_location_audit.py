"""Tests for the independent offline classification; no simulator imports."""
import unittest
from stall_location_audit import derived_events


def row(index, depth):
    return dict(time_s=index / 10, reference_step=index, phase='insert', depth_mm=depth,
                command_depth_mm=index / 5, normal_load_n=0., contact_count=0.,
                lowest_peg_z_above_mouth_mm=-depth)


class OfflineStallAuditTests(unittest.TestCase):
    def test_continuous_generic_stall_crossing_entry_gets_later_in_hole_confirmation(self):
        rows = [row(i, (i - 5) / 100) for i in range(20)]
        protocol = dict(stall_window_s=.5, stall_command_progress_mm=.5, stall_progress_mm=.1)
        generic, in_hole = derived_events(rows, protocol, 10)
        self.assertEqual([e['index'] for e in generic], [5])
        self.assertEqual(generic[0]['window_location'], 'entry_transition')
        self.assertEqual([e['index'] for e in in_hole], [11])
        self.assertGreater(in_hole[0]['window_depth_min_mm'], 0.)

    def test_free_space_stall_keeps_tracking_label_but_has_no_in_hole_event(self):
        rows = [row(i, -10. + i / 100) for i in range(12)]
        protocol = dict(stall_window_s=.5, stall_command_progress_mm=.5, stall_progress_mm=.1)
        generic, in_hole = derived_events(rows, protocol, 10)
        self.assertEqual(len(generic), 1)
        self.assertEqual(generic[0]['classification'], 'pre_entry_no_contact_tracking_stall')
        self.assertEqual(in_hole, [])


if __name__ == '__main__':
    unittest.main()
