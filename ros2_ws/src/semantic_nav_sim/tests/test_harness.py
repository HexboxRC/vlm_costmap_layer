"""ROS integration checks for resources owned by the trial monitor."""
import gc
from pathlib import Path
import sys
import unittest
import weakref

import psutil
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from run_trials import TrialMonitor


class MonitorCleanupTest(unittest.TestCase):
    def test_repeated_monitor_cleanup(self):
        refs = []
        samples = []
        for _ in range(30):
            rclpy.init(args=[])
            monitor = TrialMonitor(0.22)
            refs.append(weakref.ref(monitor))
            rclpy.spin_once(monitor, timeout_sec=0.0)
            monitor.destroy_node()
            rclpy.shutdown()
            del monitor
            gc.collect()
            self.assertTrue(all(ref() is None for ref in refs))
            proc = psutil.Process()
            samples.append((proc.num_fds(), proc.memory_info().rss))
        print('cleanup samples (FDs, RSS bytes):', samples, flush=True)
        self.assertEqual(samples[0][0], samples[-1][0])


class ReadinessTest(unittest.TestCase):
    def test_active_lifecycle_is_required_even_when_action_is_discoverable(self):
        from threading import Thread
        from geometry_msgs.msg import TransformStamped
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionServer
        from rclpy.executors import SingleThreadedExecutor

        rclpy.init(args=[])
        server = rclpy.create_node('readiness_test_server')
        state = [State.PRIMARY_STATE_INACTIVE]
        requests = []

        def get_state(request, response):
            requests.append(1)
            response.current_state.id = state[0]
            return response

        server.create_service(GetState, '/bt_navigator/get_state', get_state)
        action = ActionServer(server, NavigateToPose, '/navigate_to_pose',
                              lambda goal: NavigateToPose.Result())
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        thread = Thread(target=executor.spin)
        thread.start()
        monitor = TrialMonitor(0.22)
        monitor._latest_amcl_pose = (0., 0.)
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.
        monitor.tf_buffer.set_transform_static(transform, 'test')
        try:
            self.assertEqual(monitor.wait_for_ready(0., 0., 1.),
                             'bt_navigator_not_active_timeout')
            print('Lifecycle requests in one second:', len(requests), flush=True)
            self.assertLessEqual(len(requests), 5)
            self.assertFalse(monitor.goal_sent)
            state[0] = State.PRIMARY_STATE_ACTIVE
            self.assertEqual(monitor.wait_for_ready(0., 0., 3.), '')
            self.assertFalse(monitor.goal_sent)
        finally:
            monitor.destroy_node()
            executor.shutdown()
            thread.join()
            action.destroy()
            server.destroy_node()
            rclpy.shutdown()


class FailureClassificationTest(unittest.TestCase):
    def test_navigation_abort_and_goal_rejection_have_distinct_csv_outcomes(self):
        import random
        import tempfile
        from types import SimpleNamespace
        from unittest.mock import MagicMock, patch
        import run_trials

        args = SimpleNamespace(jitter_xy=0., jitter_yaw=0., rtf=.5, cooldown_sec=0.,
                               collision_margin=.22, startup_timeout=.01, timeout=90.,
                               loop_rate_threshold=18., world_path='unused',
                               max_linear_speed=.25, params_path='unused', condition='baseline')
        scenario = dict(name='test', start=dict(x=0., y=0., yaw=0.),
                        goal=dict(x=1., y=0.), straight_line_distance=1.)
        for result, accepted, expected in [('aborted', True, 'NAVIGATION'),
                                           ('rejected', False, 'INFRASTRUCTURE'),
                                           ('wall_safety_timeout', True, 'INFRASTRUCTURE')]:
            with self.subTest(result=result):
                monitor = MagicMock()
                monitor._pose_samples = []
                monitor.pose_overlap = False
                monitor.contact_audit.return_value = {}
                monitor.contact_healthy.return_value = True
                monitor.wait_for_ready.return_value = ''
                monitor.verify_sim_time.return_value = True
                monitor.sim_time_check = {'clock_advanced': True}
                monitor.goal_sent = True
                monitor.goal_accepted = accepted
                monitor.path_length = .5
                monitor.collided = False
                monitor.send_goal_and_wait.return_value = (result, 0)
                monitor.get_clock.return_value.now.return_value.nanoseconds = 1000000000
                monitor.loop_rate_stats.return_value = (20., 20., 10)
                monitor._wall_intervals = []
                monitor._active_wall_intervals = []
                monitor._active_cmd_intervals = []
                monitor._cmd_vel_intervals = []
                with tempfile.TemporaryDirectory(dir=run_trials.PROJECT_ROOT / 'results/phase0') as tmp:
                    with patch.object(run_trials, 'TrialMonitor', return_value=monitor), \
                            patch.object(run_trials, 'assert_clean'), \
                            patch.object(run_trials, 'teardown_stack'), \
                            patch.object(run_trials.subprocess, 'Popen'), \
                            patch.object(run_trials.rclpy, 'init'), \
                            patch.object(run_trials.rclpy, 'try_shutdown'):
                        row = run_trials.run_one_trial(args, scenario, 0, random.Random(42), tmp)
                self.assertEqual(row['failure_class'], expected)
                if expected == 'NAVIGATION':
                    self.assertEqual(row['nav2_result'], 'aborted')
                    self.assertFalse(row['success'])
                    self.assertTrue(row['usable'])
                else:
                    self.assertIsNone(row['nav2_result'])
                    self.assertIsNone(row['success'])
                    self.assertFalse(row['usable'])

    def test_no_goal_timeout_is_not_a_navigation_result(self):
        import random
        import tempfile
        from types import SimpleNamespace
        from unittest.mock import MagicMock, patch
        import run_trials

        monitor = MagicMock()
        monitor._pose_samples = []
        monitor.pose_overlap = False
        monitor.contact_audit.return_value = {}
        monitor.contact_healthy.return_value = True
        monitor.wait_for_ready.return_value = 'bt_navigator_not_active_timeout'
        monitor.goal_sent = False
        monitor.goal_accepted = False
        monitor.path_length = 0.
        monitor.collided = False
        monitor.loop_rate_stats.return_value = (None, None, 0)
        monitor._wall_intervals = []
        monitor._active_wall_intervals = []
        monitor._active_cmd_intervals = []
        monitor._cmd_vel_intervals = []
        monitor._interval_samples = []
        args = SimpleNamespace(jitter_xy=0., jitter_yaw=0., rtf=1., cooldown_sec=0.,
                               collision_margin=.22, startup_timeout=.01,
                               loop_rate_threshold=18., world_path='unused',
                               max_linear_speed=.25, params_path='unused', condition='baseline')
        scenario = dict(name='test', start=dict(x=0., y=0., yaw=0.),
                        goal=dict(x=1., y=0.), straight_line_distance=1.)
        with tempfile.TemporaryDirectory(dir=run_trials.PROJECT_ROOT / 'results/phase0') as tmp:
            with patch.object(run_trials, 'TrialMonitor', return_value=monitor), \
                    patch.object(run_trials, 'assert_clean'), \
                    patch.object(run_trials, 'teardown_stack') as teardown, \
                    patch.object(run_trials.subprocess, 'Popen'), \
                    patch.object(run_trials.rclpy, 'init'), \
                    patch.object(run_trials.rclpy, 'try_shutdown'):
                row = run_trials.run_one_trial(args, scenario, 0, random.Random(42), tmp)
        self.assertEqual(row['failure_class'], 'INFRASTRUCTURE')
        self.assertEqual(row['error'], 'bt_navigator_not_active_timeout')
        self.assertIsNone(row['nav2_result'])
        self.assertIsNone(row['success'])
        self.assertIsNone(row['collision'])
        self.assertFalse(row['usable'])
        self.assertFalse(row['goal_sent'])
        monitor.send_goal_and_wait.assert_not_called()
        monitor.destroy_node.assert_called_once()
        teardown.assert_called_once()


class LoopRateSegmentationTest(unittest.TestCase):
    def test_gate_keeps_recovery_gap_and_active_diagnostic_keeps_stall(self):
        from unittest.mock import patch
        from action_msgs.msg import GoalStatus, GoalStatusArray
        from rclpy.time import Time
        rclpy.init(args=[])
        monitor = TrialMonitor(.22)

        def status(code, identifier):
            item = GoalStatus(status=code)
            item.goal_info.goal_id.uuid = [identifier]*16
            monitor._follow_path_cb(GoalStatusArray(status_list=[item]))

        def command(seconds):
            with patch.object(monitor, 'get_clock') as clock:
                clock.return_value.now.return_value = Time(nanoseconds=round(seconds*1e9))
                monitor._cmd_vel_cb(None)

        try:
            status(GoalStatus.STATUS_EXECUTING, 1)
            command(1.)
            command(1.05)
            status(GoalStatus.STATUS_SUCCEEDED, 1)
            status(GoalStatus.STATUS_EXECUTING, 2)
            command(6.1)
            command(6.15)
            self.assertAlmostEqual(1/max(monitor._active_cmd_intervals), 20.)
            self.assertLess(monitor.loop_rate_stats()[1], .2)
            self.assertLess(1/max(monitor._cmd_vel_intervals), .2)
            # A 200 ms gap inside the SAME action must remain a degraded loop.
            command(6.35)
            self.assertAlmostEqual(1/max(monitor._active_cmd_intervals), 5.)
            self.assertLess(monitor.loop_rate_stats()[1], .2)
        finally:
            monitor.destroy_node()
            rclpy.shutdown()


class AcceptedHostClockTest(unittest.TestCase):
    def test_host_step_does_not_cancel_successful_simulated_navigation(self):
        from types import SimpleNamespace
        from unittest.mock import MagicMock, patch
        from rclpy.time import Time
        import run_trials

        monitor = MagicMock()
        monitor._pose_samples = []
        monitor.pose_overlap = False
        monitor.contact_audit.return_value = {}
        monitor.contact_healthy.return_value = True
        monitor.clock_events = []
        monitor.get_clock.return_value.now.return_value = Time(nanoseconds=0)
        handle = monitor._nav_client.send_goal_async.return_value.result.return_value
        handle.accepted = True
        handle.get_result_async.return_value.done.return_value = True
        handle.get_result_async.return_value.result.return_value = SimpleNamespace(
            status=run_trials.GoalStatus.STATUS_SUCCEEDED)
        before = dict(realtime_s=100., monotonic_s=10., boottime_s=10., monotonic_raw_s=9.)
        after = dict(realtime_s=334., monotonic_s=11., boottime_s=244., monotonic_raw_s=10.)
        with patch.object(run_trials, 'host_clock_snapshot', side_effect=[before, after]), \
                patch.object(run_trials.rclpy, 'ok', return_value=True), \
                patch.object(run_trials.rclpy, 'spin_once'), \
                patch.object(run_trials.rclpy, 'spin_until_future_complete'):
            result, _ = TrialMonitor.send_goal_and_wait(monitor, 1., 0., 90., 570.)
        self.assertEqual(result, 'succeeded')
        handle.cancel_goal_async.assert_not_called()
        self.assertIn(dict(phase='clock_discontinuity', **after), monitor.clock_events)

    def test_stalled_simulation_still_hits_wall_watchdog(self):
        from unittest.mock import MagicMock, patch
        from rclpy.time import Time
        import run_trials
        monitor = MagicMock()
        monitor._pose_samples = []
        monitor.pose_overlap = False
        monitor.contact_audit.return_value = {}
        monitor.contact_healthy.return_value = True
        monitor.clock_events = []
        monitor.get_clock.return_value.now.return_value = Time(nanoseconds=0)
        handle = monitor._nav_client.send_goal_async.return_value.result.return_value
        handle.accepted = True
        handle.get_result_async.return_value.done.return_value = False
        clocks = dict(realtime_s=100., monotonic_s=10., boottime_s=10., monotonic_raw_s=9.)
        with patch.object(run_trials, 'host_clock_snapshot', return_value=clocks), \
                patch.object(run_trials.time, 'monotonic', side_effect=[0., 11.]), \
                patch.object(run_trials.rclpy, 'ok', return_value=True), \
                patch.object(run_trials.rclpy, 'spin_once'), \
                patch.object(run_trials.rclpy, 'spin_until_future_complete'):
            result, _ = TrialMonitor.send_goal_and_wait(monitor, 1., 0., 90., 10.)
        self.assertEqual(result, 'wall_safety_timeout')
        handle.cancel_goal_async.assert_called_once()
