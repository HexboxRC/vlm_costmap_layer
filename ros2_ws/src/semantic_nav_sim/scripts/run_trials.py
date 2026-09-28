#!/usr/bin/env python3
"""Milestone 6 trial runner: N headless NavigateToPose trials per scenario
in config/scenarios.yaml, logged to CSV.

Each trial launches a completely fresh bringup_all.launch.py (Gazebo +
robot + stock Nav2), sends one NavigateToPose goal, and tears the whole
stack down before the next trial starts. Two things in here exist
specifically because of a real failure diagnosed in Milestone 5, not as
generic caution:

1. Process teardown never shells out to `pkill -f <pattern>`. During
   Milestone 5's manual debugging, a pkill pattern using escaped
   alternation ("a\\|b\\|c") silently failed to match anything, so a
   `kill -9` on what was believed to be the whole stack left every
   child process running. Three full Gazebo+Nav2 stacks ended up
   running concurrently on the same ROS_DOMAIN_ID, and the resulting
   CPU contention + crossed-wires /clock and /tf traffic between them
   is what caused that session's "robot drives into the wrong wall"
   symptom — not a bug in the robot/world/Nav2 config. See that
   commit's message for the full diagnostic trail. Teardown here kills
   by tracked PID/process-group instead of by shell pattern, and the
   *verification* that nothing survived uses Python-side substring
   matching against psutil's process table (no shell quoting to get
   wrong). If anything survives teardown anyway, the batch aborts
   loudly rather than silently starting the next trial on top of
   contaminated state.

2. The controller's actual loop rate is measured directly (by timing
   consecutive /cmd_vel_nav messages — one is published per
   controller_server control-loop iteration) and logged per trial as
   loop_rate_mean_hz / loop_rate_min_hz / loop_rate_degraded. MPPI
   assumes a steady 20 Hz loop (model_dt: 0.05 in nav2_params.yaml);
   Milestone 5 measured it dropping as low as 6.29 Hz under CPU
   contention on this WSL2 host, which breaks MPPI's internal rollout
   timing and produces erratic motion that has nothing to do with the
   semantic condition being tested. Once the semantic costmap layer
   adds CPU load in later work, some trials may legitimately run
   compute-contaminated — this column is what lets those trials be
   identified and excluded rather than silently counted as a semantic
   navigation result.

Collision detection is by pose overlap against worlds/arena.sdf's known
obstacle geometry (hand-copied below, same approach and same caveat as
scripts/generate_static_map.py: no shared source of truth with the SDF,
keep in sync by hand), sampled continuously from the map->base_footprint
TF at ~20 Hz — not from any camera/visual inspection. This directly
catches the glass_wall scenario's expected failure mode (stock Nav2
cannot see the glass wall and is expected to collide with it) as well
as any incidental wall clipping in the grass_patch (control) scenario.
"""

import argparse
import csv
import gc
import json
import hashlib
import xml.etree.ElementTree as ET
import math
import os
import random
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import psutil
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener

# ================= Known obstacle geometry (worlds/arena.sdf) =================
# (x_min, x_max, y_min, y_max), hand-copied from worlds/arena.sdf's wall_*
# and glass_wall models (pose +/- half of size). Kept separate from
# scripts/generate_static_map.py's WALLS_WORLD because this list also
# includes glass_wall — the static map deliberately excludes it (see that
# script's docstring), but collision detection here must not: driving
# into the glass wall despite it being invisible to lidar/the static map
# is the experimental failure mode this whole project measures.
OBSTACLE_BOXES = [
    (-6.15, 6.15, 5.925, 6.075),    # wall_north
    (-6.15, 6.15, -6.075, -5.925),  # wall_south
    (5.925, 6.075, -6.15, 6.15),    # wall_east
    (-6.075, -5.925, -6.15, 6.15),  # wall_west
    (-0.075, 0.075, 2.75, 5.25),    # glass_wall
]

# Chassis is 0.30 x 0.30 (half-diagonal 0.212 m); a flat per-axis margin
# is a conservative (Minkowski-square, not Minkowski-circle) over
# approximation of "robot footprint overlaps obstacle box" — slightly
# more sensitive at box corners than a true circular footprint would be,
# which is the safe direction to be wrong in for collision detection.
DEFAULT_COLLISION_MARGIN = 0.22

# Substrings matched against each running process's full cmdline to
# identify anything belonging to this project's simulation/Nav2 stack.
# Matched in Python against psutil data, never via a shell pattern —
# see module docstring point 1 for why that distinction matters here.
STACK_MARKERS = [
    "semantic_nav_sim",       # ros2 launch invocation, bridge/RSP param files
    "gz sim",                 # Gazebo server (reported comm is "ruby"; argv is not)
    "nav2_amcl", "nav2_map_server", "nav2_controller", "nav2_planner",
    "nav2_behaviors", "nav2_smoother", "nav2_route", "nav2_bt_navigator",
    "nav2_waypoint_follower", "nav2_velocity_smoother",
    "nav2_collision_monitor", "opennav_docking", "nav2_lifecycle_manager",
    "ros_gz_bridge",
]

PROJECT_ROOT = Path(__file__).resolve().parents[4]


def resource_snapshot(boundary, trial_id):
    """Record aggregate usage; never inspect shared-memory or file-descriptor contents."""
    shm = psutil.disk_usage('/dev/shm')
    memory = psutil.virtual_memory()
    fd_total = 0
    fd_unreadable = 0
    process_count = 0
    for proc in psutil.process_iter():
        process_count += 1
        try:
            fd_total += proc.num_fds()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            fd_unreadable += 1
    with os.scandir('/dev/shm') as entries:
        segment_count = sum(1 for _ in entries)
    runner = psutil.Process()
    return {
        'timestamp': datetime.now().isoformat(),
        'boundary': boundary, 'trial_id': trial_id,
        'shm_used_bytes': shm.used, 'shm_segment_count': segment_count,
        'process_count': process_count, 'open_fds_total_visible': fd_total,
        'fd_unreadable_processes': fd_unreadable,
        'runner_open_fds': runner.num_fds(),
        'runner_rss_bytes': runner.memory_info().rss,
        'memory_free_bytes': memory.free,
        'memory_available_bytes': memory.available,
    }


class OrphanProcessError(RuntimeError):
    """Raised when processes from this project's stack survive teardown.
    Always fatal to the whole batch — see module docstring point 1."""


def _self_and_ancestor_pids():
    """This process's PID plus every ancestor up to init. Needed because
    `ros2 run semantic_nav_sim run_trials.py` itself has "semantic_nav_sim"
    in its own argv (as does the `ros2 run` wrapper and any `timeout`/shell
    invoking it) — without excluding the whole chain, the very first
    assert_clean() call matches its own invocation and false-positives.
    Real orphans from a *previous* run are never this process's ancestors,
    so this exclusion cannot hide genuine contamination."""
    pids = {os.getpid()}
    try:
        proc = psutil.Process(os.getpid())
        while True:
            parent = proc.parent()
            if parent is None or parent.pid in pids:
                break
            pids.add(parent.pid)
            proc = parent
    except psutil.NoSuchProcess:
        pass
    return pids


def find_stack_processes(exclude_pids=None):
    """System-wide scan (not just descendants of a tracked PID) for any
    process matching STACK_MARKERS. Returns [(pid, cmdline_str), ...]."""
    exclude_pids = set(exclude_pids or ()) | _self_and_ancestor_pids()
    found = []
    for proc in psutil.process_iter(['pid', 'cmdline']):
        if proc.info['pid'] in exclude_pids:
            continue
        try:
            cmdline = ' '.join(proc.info['cmdline'] or [])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
        if any(marker in cmdline for marker in STACK_MARKERS):
            # Binary names alone also match the user's unrelated ROS workspace.
            # Only processes launched with a cwd inside this project are ours.
            try:
                if Path(proc.cwd()).is_relative_to(PROJECT_ROOT):
                    found.append((proc.info['pid'], cmdline))
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    return found


def assert_clean(context):
    """Abort the whole batch loudly if any stack process is found. Called
    before every trial and after every teardown."""
    survivors = find_stack_processes()
    if survivors:
        lines = [f'  pid={pid}: {cmdline[:160]}' for pid, cmdline in survivors]
        raise OrphanProcessError(
            f'{context}: {len(survivors)} orphaned stack process(es) found:\n'
            + '\n'.join(lines)
        )


def teardown_stack(launch_proc, log):
    """Complete cleanup even if the user interrupts during teardown."""
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        _teardown_stack(launch_proc, log)
    finally:
        signal.signal(signal.SIGINT, previous)


def _teardown_stack(launch_proc, log):
    """Kill the launched stack by tracked PID/process-group, then verify
    system-wide that nothing survived. Never uses a shell pkill pattern."""
    if launch_proc.poll() is None:
        try:
            pgid = os.getpgid(launch_proc.pid)
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            launch_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass

    # Belt-and-suspenders: walk the actual process tree by PID lineage
    # (psutil), independent of whether everything really shared one
    # process group, and SIGKILL any survivor plus the launch PID itself.
    try:
        parent = psutil.Process(launch_proc.pid)
        descendants = parent.children(recursive=True)
    except psutil.NoSuchProcess:
        descendants = []
    for d in descendants:
        try:
            d.kill()
        except psutil.NoSuchProcess:
            pass
    try:
        if launch_proc.poll() is None:
            launch_proc.kill()
            launch_proc.wait(timeout=5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    time.sleep(1.0)

    # Final safety net: system-wide scan by cmdline content (see
    # find_stack_processes) in case anything detached from both the
    # process group and the PID tree (e.g. via its own setsid).
    survivors = find_stack_processes()
    for pid, _ in survivors:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if survivors:
        time.sleep(1.0)

    survivors = find_stack_processes()
    if survivors:
        lines = [f'  pid={pid}: {cmdline[:160]}' for pid, cmdline in survivors]
        raise OrphanProcessError(
            f'teardown: {len(survivors)} process(es) survived cleanup:\n'
            + '\n'.join(lines)
        )
    log.write(f'[{ts()}] teardown verified clean\n')
    log.flush()


def ts():
    return datetime.now().strftime('%H:%M:%S')


class TrialMonitor(Node):
    """One instance per trial. Owns the NavigateToPose action client, the
    /cmd_vel_nav loop-rate timer, and the map->base_footprint TF sampler
    used for both path length and collision detection."""

    def __init__(self, collision_margin):
        super().__init__(
            'trial_monitor',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.goal_sent = False
        self.goal_accepted = False
        self._collision_margin = collision_margin
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_sample = None  # (x, y, t_sec)
        self.path_length = 0.0
        self.collided = False
        self._pose_timer = self.create_timer(0.05, self._sample_pose)  # ~20 Hz

        self._cmd_vel_intervals = []
        self._last_cmd_vel_time = None
        self._last_cmd_wall_time = None
        self._wall_intervals = []
        self._interval_samples = []
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_vel_cb, 20)

        self._latest_amcl_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

    def destroy_node(self):
        # rclpy.Node.destroy_node() does not destroy action waitables.
        self._nav_client.destroy()
        self.tf_listener.unregister()
        self.tf_buffer.clear()
        return super().destroy_node()

    def _amcl_cb(self, msg):
        self._latest_amcl_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _cmd_vel_cb(self, _msg):
        now = self.get_clock().now()
        wall_now = time.monotonic()
        if self._last_cmd_vel_time is not None:
            dt = (now - self._last_cmd_vel_time).nanoseconds * 1e-9
            wall_dt = wall_now - self._last_cmd_wall_time
            self._wall_intervals.append(wall_dt)
            self._interval_samples.append((now.nanoseconds*1e-9, dt, wall_dt))
            if dt > 0.0:
                self._cmd_vel_intervals.append(dt)
        self._last_cmd_vel_time = now
        self._last_cmd_wall_time = wall_now

    def _sample_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if self._last_sample is not None:
            lx, ly, lt = self._last_sample
            if t > lt:
                self.path_length += math.hypot(x - lx, y - ly)
        self._last_sample = (x, y, t)

        m = self._collision_margin
        for (xmin, xmax, ymin, ymax) in OBSTACLE_BOXES:
            if (xmin - m) <= x <= (xmax + m) and (ymin - m) <= y <= (ymax + m):
                self.collided = True
                break

    def loop_rate_stats(self):
        if not self._cmd_vel_intervals:
            return None, None, 0
        rates = [1.0 / dt for dt in self._cmd_vel_intervals]
        return statistics.mean(rates), min(rates), len(self._cmd_vel_intervals) + 1

    def wait_for_ready(self, expected_x, expected_y, timeout_sec):
        """Use one wall-clock deadline; require active lifecycle and action discovery."""
        client = self.create_client(GetState, '/bt_navigator/get_state')
        deadline = time.monotonic() + timeout_sec
        future = None
        active = False
        reason = 'bt_navigator_not_active_timeout'
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if client.service_is_ready() and future is None:
                    future = client.call_async(GetState.Request())
                rclpy.spin_once(self, timeout_sec=min(0.1, max(0., deadline-time.monotonic())))
                if future is not None and future.done():
                    response = future.result()
                    active = (response is not None and
                              response.current_state.id == State.PRIMARY_STATE_ACTIVE)
                    future = None
                reason = 'bt_navigator_not_active_timeout'
                if not active:
                    continue
                reason = 'navigate_to_pose_server_timeout'
                if not self._nav_client.server_is_ready():
                    continue
                reason = 'localization_timeout'
                if self._latest_amcl_pose is None:
                    continue
                x, y = self._latest_amcl_pose
                if math.hypot(x-expected_x, y-expected_y) > 1.0:
                    continue
                if not self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                    reason = 'localization_tf_timeout'
                    continue
                return ''
            return reason
        finally:
            if future is not None and not future.done():
                future.cancel()
            self.destroy_client(client)

    def verify_sim_time(self):
        """Read live parameters, rather than inferring them only from launch files."""
        for name in ('controller_server', 'bt_navigator'):
            client = self.create_client(GetParameters, f'/{name}/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=5.):
                    return False
                request = GetParameters.Request(names=['use_sim_time'])
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
                if not future.done() or not future.result().values[0].bool_value:
                    return False
            finally:
                self.destroy_client(client)
        return True

    def send_goal_and_wait(self, goal_x, goal_y, timeout_sec, wall_timeout):
        """Returns (nav2_result: str, number_of_recoveries: int)."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        recoveries = {'n': 0}

        def feedback_cb(feedback_msg):
            recoveries['n'] = feedback_msg.feedback.number_of_recoveries

        self.goal_sent = True
        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.)
        if not send_future.done():
            return 'goal_ack_timeout', recoveries['n']
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return 'rejected', recoveries['n']

        self.goal_accepted = True
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + wall_timeout
        clock_offset = time.time() - time.monotonic()
        sim_start = self.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_future.done():
                break
            sim_elapsed = self.get_clock().now().nanoseconds * 1e-9 - sim_start
            clock_jump = abs(time.time() - time.monotonic() - clock_offset) > 2.
            if clock_jump or sim_elapsed >= timeout_sec or time.monotonic() > deadline:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                result = ('clock_discontinuity' if clock_jump else
                          'timeout' if sim_elapsed >= timeout_sec else 'wall_safety_timeout')
                return result, recoveries['n']

        status = result_future.result().status
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_ABORTED: 'aborted',
            GoalStatus.STATUS_CANCELED: 'canceled',
        }
        return status_map.get(status, f'unknown_status_{status}'), recoveries['n']


def load_scenarios(config_path, names_filter):
    with open(config_path) as f:
        data = yaml.safe_load(f)
    scenarios = data['scenarios']
    if names_filter:
        scenarios = [s for s in scenarios if s['name'] in names_filter]
    return scenarios


def run_one_trial(args, scenario, trial_idx, rng, log_dir):
    trial_wall_start = time.monotonic()
    trial_calendar_start = time.time()
    start = scenario['start']
    jx = start['x'] + rng.uniform(-args.jitter_xy, args.jitter_xy)
    jy = start['y'] + rng.uniform(-args.jitter_xy, args.jitter_xy)
    jyaw = start['yaw'] + rng.uniform(-args.jitter_yaw, args.jitter_yaw)
    goal = scenario['goal']

    trial_name = f"{scenario['name']}_{trial_idx:03d}"
    log_path = os.path.join(log_dir, f'{trial_name}.log')

    row = {
        'trial_id': trial_name,
        'scenario': scenario['name'],
        'start_x': round(jx, 4), 'start_y': round(jy, 4), 'start_yaw': round(jyaw, 4),
        'goal_x': goal['x'], 'goal_y': goal['y'],
        'straight_line_distance': scenario['straight_line_distance'],
        'nav2_result': None, 'collision': None, 'success': None,
        'path_length_m': None, 'wall_clock_s': None, 'number_of_recoveries': None,
        'loop_rate_mean_hz': None, 'loop_rate_min_hz': None,
        'loop_rate_degraded': None, 'num_cmd_vel_msgs': 0,
        'goal_sent': False, 'goal_accepted': False,
        'failure_class': 'INFRASTRUCTURE', 'usable': False,
        'rtf_target': args.rtf, 'rtf_observed': None,
        'nav2_use_sim_time': None, 'navigation_sim_s': None,
        'total_wall_s': None, 'cooldown_sec': args.cooldown_sec,
        'host_clock_gap_s': None,
        'loop_rate_wall_min_hz': None,
        'error': '',
    }

    assert_clean(f'trial {trial_name}: pre-launch check')

    with open(log_path, 'w') as log:
        log.write(f'[{ts()}] launching bringup_all.launch.py '
                  f'x={jx:.3f} y={jy:.3f} yaw={jyaw:.3f}\n')
        log.flush()
        launch_proc = subprocess.Popen(
            ['ros2', 'launch', 'semantic_nav_sim', 'bringup_all.launch.py',
             f'x:={jx}', f'y:={jy}', f'yaw:={jyaw}', f'world:={args.world_path}'],
            stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        monitor = None
        try:
            rclpy.init(args=[])
            monitor = TrialMonitor(args.collision_margin)
            wall_start = time.monotonic()

            readiness_error = monitor.wait_for_ready(jx, jy, args.startup_timeout)
            if not readiness_error:
                row['nav2_use_sim_time'] = monitor.verify_sim_time()
                if not row['nav2_use_sim_time']:
                    readiness_error = 'use_sim_time_verification_failed'
            if readiness_error:
                row['error'] = readiness_error
            else:
                log.write(f'[{ts()}] READY: bt_navigator active, action available, localized\n')
                log.flush()
                nav_wall_start = time.monotonic()
                nav_sim_start = monitor.get_clock().now().nanoseconds * 1e-9
                result, recoveries = monitor.send_goal_and_wait(
                    goal['x'], goal['y'], args.timeout, args.timeout / args.rtf * 3 + 30)
                nav_sim_s = monitor.get_clock().now().nanoseconds * 1e-9 - nav_sim_start
                row['navigation_sim_s'] = round(nav_sim_s, 4)
                row['rtf_observed'] = round(nav_sim_s / (time.monotonic()-nav_wall_start), 4)
                if result in ('wall_safety_timeout', 'clock_discontinuity'):
                    row['error'] = result
                elif monitor.goal_accepted:
                    row['nav2_result'] = result
                    row['number_of_recoveries'] = recoveries
                    row['failure_class'] = 'NONE' if result == 'succeeded' else 'NAVIGATION'
                else:
                    row['error'] = result
            row['goal_sent'] = monitor.goal_sent
            row['goal_accepted'] = monitor.goal_accepted

            row['wall_clock_s'] = round(time.monotonic() - wall_start, 2)
            row['path_length_m'] = round(monitor.path_length, 4)
            row['collision'] = monitor.collided
            mean_hz, min_hz, n_msgs = monitor.loop_rate_stats()
            row['loop_rate_mean_hz'] = round(mean_hz, 2) if mean_hz else None
            row['loop_rate_min_hz'] = round(min_hz, 2) if min_hz else None
            row['num_cmd_vel_msgs'] = n_msgs
            if monitor._wall_intervals:
                row['loop_rate_wall_min_hz'] = round(1/max(monitor._wall_intervals), 4)
            with open(os.path.join(log_dir, trial_name + '_intervals.csv'), 'w') as samples:
                writer = csv.writer(samples)
                writer.writerow(['sim_time_s', 'sim_dt_s', 'wall_dt_s'])
                writer.writerows(monitor._interval_samples)
            row['loop_rate_degraded'] = (
                min_hz is None or min_hz < args.loop_rate_threshold
            )
            if monitor.goal_accepted:
                row['success'] = row['nav2_result'] == 'succeeded' and not row['collision']
                if row['collision']:
                    row['failure_class'] = 'NAVIGATION'
                row['usable'] = not row['loop_rate_degraded'] and n_msgs > 1
                if row['loop_rate_degraded']:
                    row['failure_class'] = 'COMPUTE_DEGRADED'
            if row['error']:
                row['failure_class'] = 'INFRASTRUCTURE'
                row['usable'] = False
                row['nav2_result'] = None
                row['success'] = None
            if not monitor.goal_accepted:
                for key in ('collision', 'success', 'path_length_m', 'loop_rate_degraded'):
                    row[key] = None

        except (Exception, KeyboardInterrupt) as e:  # Always reach teardown and save the attempt.
            row['error'] = str(e) or type(e).__name__
            row['failure_class'] = 'INFRASTRUCTURE'
            row['usable'] = False
            row['nav2_result'] = None
            row['success'] = None
        finally:
            try:
                if monitor is not None:
                    row['goal_sent'] = monitor.goal_sent
                    row['goal_accepted'] = monitor.goal_accepted
                    monitor.destroy_node()
            finally:
                rclpy.try_shutdown()
                monitor = None
                gc.collect()
                teardown_stack(launch_proc, log)

        row['total_wall_s'] = round(time.monotonic()-trial_wall_start, 3)
        gap = time.time()-trial_calendar_start-row['total_wall_s']
        row['host_clock_gap_s'] = round(gap, 3)
        if abs(gap) > 2.:
            row.update(error='clock_discontinuity', failure_class='INFRASTRUCTURE',
                       usable=False, nav2_result=None, success=None)
        log.write(f'[{ts()}] trial result: {row}\n')
        log.flush()

    assert_clean(f'trial {trial_name}: post-teardown check')
    return row


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    default_config = os.path.join(
        get_package_share_directory('semantic_nav_sim'), 'config', 'scenarios.yaml')
    parser.add_argument('--config', default=default_config)
    parser.add_argument('--scenario', action='append', default=None,
                         help='Restrict to named scenario(s); repeatable. '
                              'Default: all scenarios in the config.')
    parser.add_argument('--trials', type=int, default=10,
                         help='Trials per scenario (default: 10).')
    parser.add_argument('--jitter-xy', type=float, default=0.10,
                         help='Uniform +/- meters applied to start x,y (default: 0.10).')
    parser.add_argument('--jitter-yaw', type=float, default=0.10,
                         help='Uniform +/- radians applied to start yaw (default: 0.10).')
    parser.add_argument('--timeout', type=float, default=90.0,
                         help='Navigation timeout in simulated seconds (default: 90).')
    parser.add_argument('--rtf', type=float, default=1.0,
                         help='Gazebo target real-time factor; physics step stays 1 ms.')
    parser.add_argument('--startup-timeout', type=float, default=45.0,
                         help='Max wait for action server + AMCL localization '
                              '(default: 45).')
    parser.add_argument('--loop-rate-threshold', type=float, default=18.0,
                         help='Minimum acceptable controller loop rate, Hz. '
                              'Trials with loop_rate_min_hz below this are '
                              'flagged loop_rate_degraded=True (default: 18.0).')
    parser.add_argument('--collision-margin', type=float, default=DEFAULT_COLLISION_MARGIN,
                         help=f'Robot footprint margin for pose-overlap collision '
                              f'detection, meters (default: {DEFAULT_COLLISION_MARGIN}).')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--cooldown-sec', type=float, default=0.0,
                         help='Idle wall seconds between trials (default: 0). '
                              'Record and use the same setting for every condition.')
    parser.add_argument('--out', default=None,
                         help='CSV output path (default: results/trials_<timestamp>.csv)')
    args = parser.parse_args()

    if args.rtf <= 0 or args.trials < 1 or args.timeout <= 0 or args.cooldown_sec < 0:
        parser.error('rtf, trials and timeout must be positive; cooldown must be nonnegative')
    rng = random.Random(args.seed)

    scenarios = load_scenarios(args.config, args.scenario)
    if not scenarios:
        print('No matching scenarios found.', file=sys.stderr)
        sys.exit(1)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out_path = args.out or os.path.join('results', f'trials_{stamp}.csv')
    log_dir = os.path.join(os.path.dirname(out_path) or '.', f'logs_{stamp}')
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    world = ET.parse(os.path.join(get_package_share_directory('semantic_nav_sim'),
                                  'worlds', 'arena.sdf'))
    world.find('./world/physics/real_time_factor').text = str(args.rtf)
    args.world_path = str(Path(log_dir, 'arena.sdf').resolve())
    world.write(args.world_path, encoding='utf-8', xml_declaration=True)
    with open(os.path.splitext(out_path)[0] + '_manifest.json', 'w') as manifest:
        metadata = dict(vars(args))
        metadata['git_commit'] = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=PROJECT_ROOT, text=True).strip()
        metadata['source_sha256'] = {}
        for relative in ('scripts/run_trials.py', 'scripts/seed_initial_pose.py',
                         'config/nav2_params.yaml', 'worlds/arena.sdf',
                         'urdf/semantic_nav_robot.urdf.xacro'):
            source = PROJECT_ROOT / 'ros2_ws/src/semantic_nav_sim' / relative
            metadata['source_sha256'][relative] = hashlib.sha256(source.read_bytes()).hexdigest()
        json.dump(metadata, manifest, indent=2)

    fieldnames = [
        'trial_id', 'scenario', 'start_x', 'start_y', 'start_yaw',
        'goal_x', 'goal_y', 'straight_line_distance',
        'nav2_result', 'collision', 'success', 'path_length_m',
        'wall_clock_s', 'number_of_recoveries',
        'loop_rate_mean_hz', 'loop_rate_min_hz', 'loop_rate_degraded',
        'rtf_target', 'rtf_observed', 'nav2_use_sim_time', 'navigation_sim_s',
        'total_wall_s', 'cooldown_sec', 'loop_rate_wall_min_hz',
        'host_clock_gap_s',
        'num_cmd_vel_msgs', 'goal_sent', 'goal_accepted', 'failure_class', 'usable', 'error',
    ]

    print(f'Writing results to {out_path}')
    print(f'Per-trial launch logs in {log_dir}/')
    resource_path = os.path.splitext(out_path)[0] + '_resources.csv'
    print(f'Resource snapshots in {resource_path}')

    try:
        assert_clean('startup')
    except OrphanProcessError as e:
        print(f'\nABORTING BEFORE FIRST TRIAL — stack not clean:\n{e}\n',
              file=sys.stderr)
        sys.exit(1)

    with open(out_path, 'w', newline='') as csvfile, open(
            resource_path, 'w', newline='') as resource_file:
        snapshot = resource_snapshot('batch_start', '')
        resource_writer = csv.DictWriter(resource_file, fieldnames=list(snapshot))
        resource_writer.writeheader()
        resource_writer.writerow(snapshot)
        resource_file.flush()
        print(f'RESOURCES {snapshot}', flush=True)
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        csvfile.flush()

        total = len(scenarios) * args.trials
        done = 0
        for scenario in scenarios:
            for trial_idx in range(args.trials):
                done += 1
                print(f'[{ts()}] trial {done}/{total}: '
                      f"{scenario['name']} #{trial_idx:03d} ...", flush=True)
                try:
                    row = run_one_trial(args, scenario, trial_idx, rng, log_dir)
                except OrphanProcessError as e:
                    print(f'\n{"=" * 70}\n'
                          f'ABORTING BATCH — orphaned process(es) survived teardown.\n'
                          f'{e}\n'
                          f'Results so far are saved in {out_path}.\n'
                          f'{"=" * 70}\n', file=sys.stderr)
                    sys.exit(1)
                writer.writerow(row)
                csvfile.flush()
                snapshot = resource_snapshot('after_teardown', row['trial_id'])
                resource_writer.writerow(snapshot)
                resource_file.flush()
                print(f'RESOURCES {snapshot}', flush=True)
                flag = ' [DEGRADED LOOP RATE]' if row['loop_rate_degraded'] else ''
                print(f"    -> {row['nav2_result']} "
                      f"success={row['success']} collision={row['collision']} "
                      f"loop_rate(mean/min)={row['loop_rate_mean_hz']}/"
                      f"{row['loop_rate_min_hz']} Hz{flag}", flush=True)
                if row['error']:
                    print(f"ABORTING BATCH — infrastructure exception: {row['error']}",
                          file=sys.stderr)
                    sys.exit(1)
                if args.cooldown_sec > 0 and done < total:
                    time.sleep(args.cooldown_sec)

    print(f'\nDone. {total} trials written to {out_path}')


if __name__ == '__main__':
    main()
