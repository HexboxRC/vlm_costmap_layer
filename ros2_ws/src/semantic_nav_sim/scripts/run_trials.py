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

2. Command-arrival rate is a conservative timing proxy, not controller
   compute duration. The gate retains every positive simulation-clock gap,
   including recovery waits. FollowPath-active intervals are additional
   diagnostics only and never replace the gate metric. Both simulation and
   wall clocks and all raw intervals remain logged. The 18 Hz threshold and
   runtime settings must be identical across experimental conditions.

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
import multiprocessing
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
from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from ros_gz_interfaces.msg import Contacts
from rclpy.qos import qos_profile_sensor_data
from contact_metrics import classify_pair
from trajectory_metrics import grass_metrics
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener

def host_clock_snapshot():
    """Read independent host clocks even when command callbacks have stopped."""
    return {
        'realtime_s': time.time(),
        'monotonic_s': time.monotonic(),
        'boottime_s': time.clock_gettime(time.CLOCK_BOOTTIME),
        'monotonic_raw_s': time.clock_gettime(time.CLOCK_MONOTONIC_RAW),
    }


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
        self._pose_samples = []
        self.path_length = 0.0
        self.collided = False
        self.pose_overlap = False
        self._contact_events = []
        self._contact_counts = {'ground': 0, 'obstacle': 0, 'irrelevant': 0}
        self._last_contact_sim = None
        self._contact_max_gap = 0.
        self.create_subscription(Contacts, '/benchmark/contacts', self._contacts_cb, 1000)
        self.create_subscription(Contacts, '/benchmark/ground_contacts', self._contacts_cb,
                                 qos_profile_sensor_data)
        self._pose_timer = self.create_timer(0.05, self._sample_pose)  # ~20 Hz

        self._cmd_vel_intervals = []
        self._last_cmd_vel_time = None
        self._last_cmd_wall_time = None
        self._wall_intervals = []
        self._interval_samples = []
        self.clock_events = []
        self._active_follow_path_ids = frozenset()
        self._active_segment = 0
        self._last_active_cmd = None
        self._active_cmd_intervals = []
        self._active_wall_intervals = []
        self._num_cmd_msgs = 0
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_vel_cb, 20)
        self.create_subscription(
            GoalStatusArray, '/follow_path/_action/status', self._follow_path_cb,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

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

    def _follow_path_cb(self, msg):
        active = frozenset(bytes(item.goal_info.goal_id.uuid) for item in msg.status_list
                           if item.status == GoalStatus.STATUS_EXECUTING)
        if active != self._active_follow_path_ids:
            self._active_follow_path_ids = active
            self._active_segment += 1
            self._last_active_cmd = None

    def _cmd_vel_cb(self, _msg):
        now = self.get_clock().now()
        wall_now = time.monotonic()
        self._num_cmd_msgs += 1
        active_dt = active_wall_dt = ''
        if self._active_follow_path_ids:
            if self._last_active_cmd is not None:
                previous, previous_wall = self._last_active_cmd
                active_dt = (now-previous).nanoseconds * 1e-9
                active_wall_dt = wall_now-previous_wall
                if active_dt > 0:
                    self._active_cmd_intervals.append(active_dt)
                    self._active_wall_intervals.append(active_wall_dt)
            self._last_active_cmd = (now, wall_now)
        if self._last_cmd_vel_time is not None:
            dt = (now - self._last_cmd_vel_time).nanoseconds * 1e-9
            wall_dt = wall_now - self._last_cmd_wall_time
            self._wall_intervals.append(wall_dt)
            self._interval_samples.append((now.nanoseconds*1e-9, dt, wall_dt,
                                           self._active_segment, active_dt, active_wall_dt,
                                           bool(self._active_follow_path_ids), time.time(),
                                           wall_now, time.clock_gettime(time.CLOCK_BOOTTIME),
                                           time.clock_gettime(time.CLOCK_MONOTONIC_RAW)))
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
        self._pose_samples.append((t, x, y))

        m = self._collision_margin
        for (xmin, xmax, ymin, ymax) in OBSTACLE_BOXES:
            if (xmin - m) <= x <= (xmax + m) and (ymin - m) <= y <= (ymax + m):
                self.pose_overlap = True
                break

    def _contacts_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_contact_sim is not None:
            self._contact_max_gap = max(self._contact_max_gap, now-self._last_contact_sim)
        self._last_contact_sim = now
        for contact in msg.contacts:
            kind = classify_pair(contact.collision1.name, contact.collision2.name)
            self._contact_counts[kind] += 1
            if kind == 'obstacle':
                self._contact_events.append(dict(
                    sim_time_s=now, source_stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                    navigation=self.goal_sent,
                    collision1=contact.collision1.name, collision2=contact.collision2.name,
                    depths=list(contact.depths),
                    positions=[dict(x=p.x, y=p.y, z=p.z) for p in contact.positions]))
                if self.goal_sent:
                    self.collided = True

    def contact_healthy(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        return (self._contact_counts['ground'] > 0 and self._last_contact_sim is not None
                and 0 <= now-self._last_contact_sim <= 2. and self._contact_max_gap <= 2.)

    def verify_contacts(self):
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if self.contact_healthy():
                return True
        return False

    def contact_audit(self):
        return dict(metric='gazebo_obstacle_contacts_v1', counts=self._contact_counts,
                    max_gap_sim_s=self._contact_max_gap, events=self._contact_events,
                    healthy=self.contact_healthy())

    def loop_rate_stats(self):
        if not self._cmd_vel_intervals:
            return None, None, self._num_cmd_msgs
        rates = [1.0 / dt for dt in self._cmd_vel_intervals]
        return statistics.mean(rates), min(rates), self._num_cmd_msgs

    def wait_for_ready(self, expected_x, expected_y, timeout_sec):
        """Use one wall-clock deadline; require active lifecycle and action discovery."""
        client = self.create_client(GetState, '/bt_navigator/get_state')
        deadline = time.monotonic() + timeout_sec
        future = None
        next_request = 0.
        request_deadline = 0.
        active = False
        reason = 'bt_navigator_not_active_timeout'
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                now = time.monotonic()
                if future is not None and not future.done() and now >= request_deadline:
                    client.remove_pending_request(future)
                    future.cancel()
                    future = None
                if client.service_is_ready() and future is None and now >= next_request:
                    future = client.call_async(GetState.Request())
                    next_request = now + 0.25
                    request_deadline = now + 1.
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
        """One bounded live parameter/clock graph check before navigation."""
        names = {'controller_server', 'bt_navigator', 'planner_server', 'amcl',
                 'map_server', 'robot_state_publisher', 'parameter_bridge'}
        # Include every live Nav2 server/costmap, not only the two action nodes.
        for name, namespace in self.get_node_names_and_namespaces():
            if (name.endswith('_server') or name.endswith('_costmap') or
                    name in ('velocity_smoother', 'collision_monitor', 'waypoint_follower',
                             'semantic_ground_truth') or
                    name.startswith('lifecycle_manager')):
                names.add((namespace.rstrip('/') + '/' + name).lstrip('/'))
        subscribers = {(info.node_namespace.rstrip('/') + '/' + info.node_name).lstrip('/')
                       for info in self.get_subscriptions_info_by_topic('/clock')}
        self.sim_time_check = {'nodes': {}, 'clock_advanced': False}
        valid = True
        for name in sorted(names):
            client = self.create_client(GetParameters, f'/{name}/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=5.):
                    self.sim_time_check['nodes'][name] = {'error': 'parameter_service_missing'}
                    valid = False
                    continue
                request = GetParameters.Request(names=['use_sim_time'])
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
                response = future.result() if future.done() else None
                enabled = bool(response and response.values and response.values[0].bool_value)
                connected = name in subscribers
                self.sim_time_check['nodes'][name] = {
                    'use_sim_time': enabled, 'clock_subscription': connected}
                valid = valid and enabled and connected
            finally:
                self.destroy_client(client)
        start = self.get_clock().now().nanoseconds
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > start:
                self.sim_time_check['clock_advanced'] = True
                break
        return valid and self.sim_time_check['clock_advanced']

    def verify_ground_truth(self, grass_cost=40):
        """Require actual source delivery and glass cost in the global master map."""
        received = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [self.create_subscription(
            Costmap, topic, lambda msg, key=key: received.update({key: msg}), qos)
            for key, topic in [('source', '/semantic_costmap'),
                               ('master', '/global_costmap/costmap_raw')]]
        self.semantic_check = {'ready': False}
        deadline = time.monotonic() + 10.
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.1)
                if not all(key in received for key in ('source', 'master')):
                    continue
                source, master = received['source'], received['master']
                if master.header.frame_id != 'map' or master.metadata.resolution <= 0:
                    continue

                def cost_at(x, y):
                    m = master.metadata
                    ix = math.floor((x-m.origin.position.x)/m.resolution)
                    iy = math.floor((y-m.origin.position.y)/m.resolution)
                    return (master.data[iy*m.size_x+ix]
                            if 0 <= ix < m.size_x and 0 <= iy < m.size_y else None)

                stamp = source.header.stamp.sec + source.header.stamp.nanosec*1e-9
                age = self.get_clock().now().nanoseconds*1e-9 - stamp
                self.semantic_check = dict(ready=False, source=source.metadata.layer,
                                           age_sim_s=age, glass_cost=cost_at(0., 4.),
                                           grass_cost=cost_at(0., -4.))
                if (source.metadata.layer == 'ground_truth' and 0 <= age <= 5. and
                        self.semantic_check['glass_cost'] == 254 and
                        self.semantic_check['grass_cost'] == grass_cost):
                    self.semantic_check['ready'] = True
                    return True
            return False
        finally:
            for subscription in subscriptions:
                self.destroy_subscription(subscription)

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

        # Measurement interval starts at goal submission; retain command-gap history.
        self._pose_samples = []
        self._last_sample = None
        self.path_length = 0.
        self.pose_overlap = False
        self.collided = False
        self._contact_max_gap = 0.
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
        previous_clocks = host_clock_snapshot()
        clock_offset = previous_clocks['realtime_s'] - previous_clocks['monotonic_s']
        self.clock_events.append(dict(phase='navigation_start', **previous_clocks))
        sim_start = self.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            clocks = host_clock_snapshot()
            clock_jump = abs(clocks['realtime_s'] - clocks['monotonic_s'] - clock_offset) > 2.
            if clock_jump:
                self.clock_events.extend([
                    dict(phase='before_clock_discontinuity', **previous_clocks),
                    dict(phase='clock_discontinuity', **clocks)])
                # Accepted limitation (October 6): observe once per step;
                # host realtime changes do not invalidate simulated navigation.
                clock_offset = clocks['realtime_s'] - clocks['monotonic_s']
            previous_clocks = clocks
            if result_future.done():
                self.clock_events.append(dict(phase='navigation_result', **clocks))
                break
            sim_elapsed = self.get_clock().now().nanoseconds * 1e-9 - sim_start
            if sim_elapsed >= timeout_sec or time.monotonic() > deadline:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                result = 'timeout' if sim_elapsed >= timeout_sec else 'wall_safety_timeout'
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
    clock_events = [dict(phase='trial_start', **host_clock_snapshot())]
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
        'condition': args.condition,
        'semantic_ready': None,
        'collision_metric': 'gazebo_obstacle_contacts_v1',
        'pose_overlap': None, 'grass_path_m': None, 'grass_central_crossing': None,
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
        'clock_policy': 'sim_time_host_steps_accepted_v1',
        'max_linear_speed': args.max_linear_speed,
        'runner_restart_policy': 'per_trial', 'worker_pid': os.getpid(),
        'worker_rss_bytes': None, 'worker_open_fds': None,
        'loop_rate_wall_min_hz': None,
        'raw_command_gap_min_hz': None, 'raw_command_wall_min_hz': None,
        'active_command_min_hz': None, 'active_command_wall_min_hz': None,
        'num_active_cmd_intervals': 0, 'loop_rate_metric': 'all_command_gaps_v1',
        'error': '',
    }

    assert_clean(f'trial {trial_name}: pre-launch check')

    with open(log_path, 'w') as log:
        log.write(f'[{ts()}] launching bringup_all.launch.py '
                  f'x={jx:.3f} y={jy:.3f} yaw={jyaw:.3f}\n')
        log.flush()
        launch_proc = subprocess.Popen(
            ['ros2', 'launch', 'semantic_nav_sim', 'bringup_all.launch.py',
             f'x:={jx}', f'y:={jy}', f'yaw:={jyaw}', f'world:={args.world_path}',
             f'params_file:={args.params_path}', f'condition:={args.condition}',
             f'grass_cost:={254 if args.condition == "binary" else 40}'],
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
                with open(os.path.join(log_dir, trial_name + '_sim_time.json'), 'w') as stream:
                    json.dump(monitor.sim_time_check, stream, indent=2)
                if not row['nav2_use_sim_time']:
                    readiness_error = 'use_sim_time_verification_failed'
            if not readiness_error and not monitor.verify_contacts():
                readiness_error = 'contact_sensor_not_ready'
            if not readiness_error and args.condition != 'baseline':
                row['semantic_ready'] = monitor.verify_ground_truth(254 if args.condition == 'binary' else 40)
                with open(os.path.join(log_dir, trial_name + '_semantic.json'), 'w') as stream:
                    json.dump(monitor.semantic_check, stream, indent=2)
                if not row['semantic_ready']:
                    readiness_error = 'semantic_costmap_not_ready'
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
                if result == 'wall_safety_timeout':
                    row['error'] = result
                elif monitor.goal_accepted:
                    row['nav2_result'] = result
                    row['number_of_recoveries'] = recoveries
                    row['failure_class'] = 'NONE' if result == 'succeeded' else 'NAVIGATION'
                else:
                    row['error'] = result
            if monitor.goal_accepted and not monitor.contact_healthy():
                row['error'] = 'contact_stream_unhealthy'
            row['pose_overlap'] = monitor.pose_overlap if monitor.goal_accepted else None
            row.update(grass_metrics(monitor._pose_samples, scenario))
            with open(os.path.join(log_dir, trial_name + '_contacts.json'), 'w') as stream:
                json.dump(monitor.contact_audit(), stream, indent=2)
            row['goal_sent'] = monitor.goal_sent
            row['goal_accepted'] = monitor.goal_accepted

            row['wall_clock_s'] = round(time.monotonic() - wall_start, 2)
            row['path_length_m'] = round(monitor.path_length, 4)
            row['collision'] = monitor.collided
            mean_hz, min_hz, n_msgs = monitor.loop_rate_stats()
            row['loop_rate_mean_hz'] = round(mean_hz, 2) if mean_hz else None
            row['loop_rate_min_hz'] = round(min_hz, 2) if min_hz else None
            row['num_cmd_vel_msgs'] = n_msgs
            row['num_active_cmd_intervals'] = len(monitor._active_cmd_intervals)
            if monitor._active_wall_intervals:
                row['active_command_wall_min_hz'] = round(1/max(monitor._active_wall_intervals), 4)
            if monitor._active_cmd_intervals:
                row['active_command_min_hz'] = round(1/max(monitor._active_cmd_intervals), 4)
            if monitor._cmd_vel_intervals:
                row['raw_command_gap_min_hz'] = round(1/max(monitor._cmd_vel_intervals), 4)
            if monitor._wall_intervals:
                row['raw_command_wall_min_hz'] = round(1/max(monitor._wall_intervals), 4)
                row['loop_rate_wall_min_hz'] = row['raw_command_wall_min_hz']
            with open(os.path.join(log_dir, trial_name + '_intervals.csv'), 'w') as samples:
                writer = csv.writer(samples)
                writer.writerow(['sim_time_s', 'sim_dt_s', 'wall_dt_s', 'active_segment',
                                 'active_sim_dt_s', 'active_wall_dt_s', 'follow_path_active',
                                 'realtime_s', 'monotonic_s', 'boottime_s', 'monotonic_raw_s'])
                writer.writerows(monitor._interval_samples)
            with open(os.path.join(log_dir, trial_name + '_poses.csv'), 'w') as samples:
                writer = csv.writer(samples)
                writer.writerow(['sim_time_s', 'x', 'y'])
                writer.writerows(monitor._pose_samples)
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
            if monitor is None or not monitor.goal_accepted:
                row['collision'] = None
                row['path_length_m'] = None
                row['number_of_recoveries'] = None
                row['loop_rate_degraded'] = None
        finally:
            try:
                if monitor is not None:
                    row['goal_sent'] = monitor.goal_sent
                    row['goal_accepted'] = monitor.goal_accepted
                    clock_events.extend(monitor.clock_events)
                    monitor.destroy_node()
            finally:
                rclpy.try_shutdown()
                monitor = None
                gc.collect()
                clock_events.append(dict(phase='before_teardown', **host_clock_snapshot()))
                try:
                    teardown_stack(launch_proc, log)
                finally:
                    clock_events.append(dict(phase='after_teardown', **host_clock_snapshot()))
                    with open(os.path.join(log_dir, trial_name + '_clocks.csv'), 'w') as stream:
                        writer = csv.DictWriter(stream, fieldnames=list(clock_events[0]))
                        writer.writeheader()
                        writer.writerows(clock_events)

        row['total_wall_s'] = round(time.monotonic()-trial_wall_start, 3)
        gap = time.time()-trial_calendar_start-row['total_wall_s']
        row['host_clock_gap_s'] = round(gap, 3)
        row['worker_rss_bytes'] = psutil.Process().memory_info().rss
        row['worker_open_fds'] = psutil.Process().num_fds()
        log.write(f'[{ts()}] trial result: {row}\n')
        log.flush()

    assert_clean(f'trial {trial_name}: post-teardown check')
    return row


def trial_worker(connection, args, scenario, trial_idx, rng, log_dir):
    """Fresh interpreter for each attempt; no ROS objects survive into the next."""
    try:
        row = run_one_trial(args, scenario, trial_idx, rng, log_dir)
        connection.send((row, rng.getstate(), None))
    except BaseException as exc:
        connection.send((None, None, f'{type(exc).__name__}: {exc}'))
    finally:
        connection.close()


def run_isolated_trial(args, scenario, trial_idx, rng, log_dir):
    context = multiprocessing.get_context('spawn')
    receive, send = context.Pipe(duplex=False)
    worker = context.Process(target=trial_worker,
                             args=(send, args, scenario, trial_idx, rng, log_dir))
    worker.start()
    send.close()
    try:
        try:
            row, state, error = receive.recv()
        except EOFError as exc:
            raise OrphanProcessError('Trial worker exited without a saved result; stop batch') from exc
        worker.join()
        if error or worker.exitcode:
            raise OrphanProcessError(error or f'Trial worker exit code {worker.exitcode}')
        rng.setstate(state)
        return row
    finally:
        receive.close()
        if not worker.is_alive():
            worker.close()


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
    parser.add_argument('--condition', choices=['baseline', 'ground_truth', 'binary'], default='baseline',
                        help='Baseline excludes the oracle and semantic layer entirely.')
    parser.add_argument('--jitter-xy', type=float, default=0.10,
                         help='Uniform +/- meters applied to start x,y (default: 0.10).')
    parser.add_argument('--jitter-yaw', type=float, default=0.10,
                         help='Uniform +/- radians applied to start yaw (default: 0.10).')
    parser.add_argument('--timeout', type=float, default=90.0,
                         help='Navigation timeout in simulated seconds (default: 90).')
    parser.add_argument('--rtf', type=float, default=0.5,
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
    parser.add_argument('--max-linear-speed', type=float, default=0.25,
                        help='Common forward/reverse speed cap in m/s (default: 0.25).')
    parser.add_argument('--cooldown-sec', type=float, default=0.0,
                         help='Idle wall seconds between trials (default: 0). '
                              'Record and use the same setting for every condition.')
    parser.add_argument('--out', default=None,
                         help='CSV output path (default: results/trials_<timestamp>.csv)')
    args = parser.parse_args()

    if (args.rtf <= 0 or args.trials < 1 or args.timeout <= 0 or args.cooldown_sec < 0
            or not 0 < args.max_linear_speed <= 0.5):
        parser.error('rtf, trials and timeout must be positive; cooldown must be nonnegative')
    rng = random.Random(args.seed)

    scenarios = load_scenarios(args.config, args.scenario)
    if not scenarios:
        print('No matching scenarios found.', file=sys.stderr)
        sys.exit(1)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out_path = args.out or os.path.join('results', f'trials_{stamp}.csv')
    if Path(out_path).exists():
        parser.error('Output already exists; use a fresh path to preserve every attempt')
    log_dir = os.path.join(os.path.dirname(out_path) or '.', f'logs_{stamp}')
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    args.world_paths = {}
    for scenario in scenarios:
        world = ET.parse(os.path.join(get_package_share_directory('semantic_nav_sim'),
                                      'worlds', 'arena.sdf'))
        world.find('./world/physics/real_time_factor').text = str(args.rtf)
        if 'grass_pose' in scenario:
            grass = world.find('./world/model[@name="grass_patch"]')
            x, y = scenario['grass_pose']
            sx, sy = scenario['grass_size']
            grass.find('pose').text = f'{x} {y} 0.001 0 0 0'
            grass.find('link/visual/geometry/plane/size').text = f'{sx} {sy}'
        path = str(Path(log_dir, scenario['name'] + '.sdf').resolve())
        world.write(path, encoding='utf-8', xml_declaration=True)
        args.world_paths[scenario['name']] = path
    params = yaml.safe_load((PROJECT_ROOT / 'ros2_ws/src/semantic_nav_sim/config/nav2_params.yaml').read_text())
    controller = params['controller_server']['ros__parameters']['FollowPath']
    controller['vx_max'] = args.max_linear_speed
    controller['vx_min'] = -args.max_linear_speed
    smoother = params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][0] = args.max_linear_speed
    smoother['min_velocity'][0] = -args.max_linear_speed
    if args.condition != 'baseline':
        for name in ('local_costmap', 'global_costmap'):
            costmap = params[name][name]['ros__parameters']
            costmap['plugins'].insert(costmap['plugins'].index('inflation_layer'), 'semantic_layer')
            costmap['semantic_layer'] = {
                'plugin': 'semantic_nav_costmap_layer::SemanticLayer',
                'enabled': True, 'topic': '/semantic_costmap', 'observation_timeout': 5.0}
    args.params_path = str(Path(log_dir, 'nav2_trial_params.yaml').resolve())
    Path(args.params_path).write_text(yaml.safe_dump(params, sort_keys=False))
    with open(os.path.splitext(out_path)[0] + '_manifest.json', 'w') as manifest:
        metadata = dict(vars(args))
        metadata['scenario_definitions'] = scenarios
        metadata['world_sha256'] = {name: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                                    for name, path in args.world_paths.items()}
        metadata['runner_restart_policy'] = 'per_trial'
        metadata['clock_policy'] = 'sim_time_host_steps_accepted_v1'
        metadata['params_sha256'] = hashlib.sha256(Path(args.params_path).read_bytes()).hexdigest()
        metadata['git_commit'] = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=PROJECT_ROOT, text=True).strip()
        metadata['source_sha256'] = {}
        for relative in ('scripts/run_trials.py', 'scripts/seed_initial_pose.py',
                         'scripts/publish_ground_truth.py', 'scripts/contact_metrics.py',
                         'scripts/trajectory_metrics.py', 'config/robot_bridge.yaml',
                         'config/nav2_params.yaml', 'config/scenarios.yaml',
                         'launch/nav2_bringup.launch.py', 'launch/robot_bringup.launch.py',
                         'launch/bringup_all.launch.py', 'worlds/arena.sdf',
                         'urdf/semantic_nav_robot.urdf.xacro'):
            source = PROJECT_ROOT / 'ros2_ws/src/semantic_nav_sim' / relative
            metadata['source_sha256'][relative] = hashlib.sha256(source.read_bytes()).hexdigest()
        if args.condition != 'baseline':
            for source in sorted((PROJECT_ROOT / 'ros2_ws/src/semantic_nav_costmap_layer').rglob('*')):
                if source.is_file():
                    metadata['source_sha256'][str(source.relative_to(PROJECT_ROOT))] = (
                        hashlib.sha256(source.read_bytes()).hexdigest())
        json.dump(metadata, manifest, indent=2)

    fieldnames = [
        'trial_id', 'scenario', 'condition', 'semantic_ready',
        'collision_metric', 'pose_overlap', 'grass_path_m', 'grass_central_crossing', 'start_x', 'start_y', 'start_yaw',
        'goal_x', 'goal_y', 'straight_line_distance',
        'nav2_result', 'collision', 'success', 'path_length_m',
        'wall_clock_s', 'number_of_recoveries',
        'loop_rate_mean_hz', 'loop_rate_min_hz', 'loop_rate_degraded',
        'rtf_target', 'rtf_observed', 'nav2_use_sim_time', 'navigation_sim_s',
        'total_wall_s', 'cooldown_sec', 'loop_rate_wall_min_hz',
        'host_clock_gap_s',
        'clock_policy', 'max_linear_speed', 'runner_restart_policy',
        'worker_pid', 'worker_rss_bytes', 'worker_open_fds',
        'raw_command_gap_min_hz', 'raw_command_wall_min_hz',
        'active_command_min_hz', 'active_command_wall_min_hz',
        'num_active_cmd_intervals', 'loop_rate_metric',
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
            args.world_path = args.world_paths[scenario['name']]
            for trial_idx in range(args.trials):
                done += 1
                print(f'[{ts()}] trial {done}/{total}: '
                      f"{scenario['name']} #{trial_idx:03d} ...", flush=True)
                try:
                    row = run_isolated_trial(args, scenario, trial_idx, rng, log_dir)
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
                    print(f"INFRASTRUCTURE failure retained: {row['error']}; teardown clean",
                          file=sys.stderr, flush=True)
                if args.cooldown_sec > 0 and done < total:
                    time.sleep(args.cooldown_sec)

    print(f'\nDone. {total} trials written to {out_path}')


if __name__ == '__main__':
    main()
