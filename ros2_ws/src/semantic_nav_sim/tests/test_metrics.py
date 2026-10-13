"""Collision classification and grass exposure regression checks."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from contact_metrics import classify_pair
from trajectory_metrics import grass_metrics


class ContactTest(unittest.TestCase):
    def test_floor_is_not_collision_and_obstacle_can_touch_any_robot_link(self):
        wheel = 'semantic_nav_robot::wheel_left_link::wheel_left_link_collision'
        glass = 'glass_wall::link::collision'
        self.assertEqual(classify_pair('ground_plane::link::collision', wheel), 'ground')
        self.assertEqual(classify_pair(wheel, glass), 'obstacle')
        self.assertEqual(classify_pair(glass, wheel), 'obstacle')
        self.assertEqual(classify_pair('other_robot::link::collision', glass), 'irrelevant')

    def test_missing_stream_is_not_a_valid_no_collision(self):
        from types import SimpleNamespace
        from run_trials import TrialMonitor
        from rclpy.time import Time
        from unittest.mock import MagicMock
        monitor = MagicMock()
        monitor.get_clock.return_value.now.return_value = Time(seconds=10)
        monitor._contact_counts = {'ground': 1}
        monitor._last_contact_sim = None
        monitor._contact_max_gap = 0.
        self.assertFalse(TrialMonitor.contact_healthy(monitor))
        monitor._last_contact_sim = 9.99
        self.assertTrue(TrialMonitor.contact_healthy(monitor))
        monitor._contact_max_gap = 3.
        self.assertFalse(TrialMonitor.contact_healthy(monitor))


class TrajectoryTest(unittest.TestCase):
    def test_crossing_and_clean_bypass(self):
        scenario={'grass_pose':[0.,-4.], 'grass_size':[3.,1.5]}
        cross=grass_metrics([(0.,-4.,-4.),(1.,4.,-4.)],scenario)
        self.assertAlmostEqual(cross['grass_path_m'],3.,places=2)
        self.assertTrue(cross['grass_central_crossing'])
        bypass=grass_metrics([(0.,-4.,-2.),(1.,4.,-2.)],scenario)
        self.assertEqual(bypass['grass_path_m'],0.)
        self.assertFalse(bypass['grass_central_crossing'])
        self.assertIsNone(grass_metrics([],scenario)['grass_path_m'])
