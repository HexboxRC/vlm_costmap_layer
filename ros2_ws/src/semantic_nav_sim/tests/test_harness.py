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

        def get_state(request, response):
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
