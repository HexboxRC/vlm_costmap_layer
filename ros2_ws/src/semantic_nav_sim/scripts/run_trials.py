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
import math
import os
import random
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime

import psutil
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
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
            found.append((proc.info['pid'], cmdline))
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
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_vel_cb, 20)

        self._latest_amcl_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

    def _amcl_cb(self, msg):
        self._latest_amcl_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _cmd_vel_cb(self, _msg):
        now = self.get_clock().now()
        if self._last_cmd_vel_time is not None:
            dt = (now - self._last_cmd_vel_time).nanoseconds * 1e-9
            if dt > 0.0:
                self._cmd_vel_intervals.append(dt)
        self._last_cmd_vel_time = now

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

    def wait_for_server(self, timeout_sec):
        return self._nav_client.wait_for_server(timeout_sec=timeout_sec)

    def wait_for_bt_navigator_active(self, timeout_sec):
        """wait_for_server() only confirms the action name is discoverable
        — bt_navigator advertises it before finishing lifecycle activation,
        and sending a goal in that window gets "Action server is inactive.
        Rejecting the goal." (hit for real during this milestone's own
        smoke test). Poll the lifecycle GetState service instead, which
        reflects the actual state the autostart lifecycle_manager drives
        it through."""
        client = self.create_client(GetState, '/bt_navigator/get_state')
        deadline = time.time() + timeout_sec
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False
        while time.time() < deadline:
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                if future.result().current_state.label == 'active':
                    return True
            time.sleep(0.3)
        return False

    def wait_for_localization(self, expected_x, expected_y, timeout_sec, tol=1.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self._latest_amcl_pose is not None:
                dx = self._latest_amcl_pose[0] - expected_x
                dy = self._latest_amcl_pose[1] - expected_y
                if math.hypot(dx, dy) <= tol:
                    return True
        return False

    def send_goal_and_wait(self, goal_x, goal_y, timeout_sec):
        """Returns (nav2_result: str, number_of_recoveries: int)."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        recoveries = {'n': 0}

        def feedback_cb(feedback_msg):
            recoveries['n'] = feedback_msg.feedback.number_of_recoveries

        send_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return 'rejected', recoveries['n']

        result_future = goal_handle.get_result_async()
        deadline = time.time() + timeout_sec
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_future.done():
                break
            if time.time() > deadline:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                return 'timeout', recoveries['n']

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
        'error': '',
    }

    assert_clean(f'trial {trial_name}: pre-launch check')

    with open(log_path, 'w') as log:
        log.write(f'[{ts()}] launching bringup_all.launch.py '
                  f'x={jx:.3f} y={jy:.3f} yaw={jyaw:.3f}\n')
        log.flush()
        launch_proc = subprocess.Popen(
            ['ros2', 'launch', 'semantic_nav_sim', 'bringup_all.launch.py',
             f'x:={jx}', f'y:={jy}', f'yaw:={jyaw}'],
            stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        try:
            rclpy.init(args=[])
            monitor = TrialMonitor(args.collision_margin)
            wall_start = time.time()

            if not monitor.wait_for_server(args.startup_timeout):
                row['nav2_result'] = 'server_timeout'
            elif not monitor.wait_for_bt_navigator_active(args.startup_timeout):
                row['nav2_result'] = 'bt_navigator_not_active_timeout'
            elif not monitor.wait_for_localization(jx, jy, args.startup_timeout):
                row['nav2_result'] = 'localization_timeout'
            else:
                result, recoveries = monitor.send_goal_and_wait(
                    goal['x'], goal['y'], args.timeout)
                row['nav2_result'] = result
                row['number_of_recoveries'] = recoveries

            row['wall_clock_s'] = round(time.time() - wall_start, 2)
            row['path_length_m'] = round(monitor.path_length, 4)
            row['collision'] = monitor.collided
            mean_hz, min_hz, n_msgs = monitor.loop_rate_stats()
            row['loop_rate_mean_hz'] = round(mean_hz, 2) if mean_hz else None
            row['loop_rate_min_hz'] = round(min_hz, 2) if min_hz else None
            row['num_cmd_vel_msgs'] = n_msgs
            row['loop_rate_degraded'] = (
                min_hz is None or min_hz < args.loop_rate_threshold
            )
            row['success'] = (
                row['nav2_result'] == 'succeeded' and not row['collision']
            )

            monitor.destroy_node()
        except Exception as e:  # noqa: BLE001 — must still reach teardown
            row['error'] = str(e)
            row['success'] = False
        finally:
            rclpy.shutdown()

        log.write(f'[{ts()}] trial result: {row}\n')
        log.flush()

        teardown_stack(launch_proc, log)

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
                         help='Per-trial navigation timeout, seconds (default: 90).')
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
    parser.add_argument('--out', default=None,
                         help='CSV output path (default: results/trials_<timestamp>.csv)')
    args = parser.parse_args()

    rng = random.Random(args.seed)

    scenarios = load_scenarios(args.config, args.scenario)
    if not scenarios:
        print('No matching scenarios found.', file=sys.stderr)
        sys.exit(1)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = args.out or os.path.join('results', f'trials_{stamp}.csv')
    log_dir = os.path.join(os.path.dirname(out_path) or '.', f'logs_{stamp}')
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    fieldnames = [
        'trial_id', 'scenario', 'start_x', 'start_y', 'start_yaw',
        'goal_x', 'goal_y', 'straight_line_distance',
        'nav2_result', 'collision', 'success', 'path_length_m',
        'wall_clock_s', 'number_of_recoveries',
        'loop_rate_mean_hz', 'loop_rate_min_hz', 'loop_rate_degraded',
        'num_cmd_vel_msgs', 'error',
    ]

    print(f'Writing results to {out_path}')
    print(f'Per-trial launch logs in {log_dir}/')

    try:
        assert_clean('startup')
    except OrphanProcessError as e:
        print(f'\nABORTING BEFORE FIRST TRIAL — stack not clean:\n{e}\n',
              file=sys.stderr)
        sys.exit(1)

    with open(out_path, 'w', newline='') as csvfile:
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
                flag = ' [DEGRADED LOOP RATE]' if row['loop_rate_degraded'] else ''
                print(f"    -> {row['nav2_result']} "
                      f"success={row['success']} collision={row['collision']} "
                      f"loop_rate(mean/min)={row['loop_rate_mean_hz']}/"
                      f"{row['loop_rate_min_hz']} Hz{flag}", flush=True)

    print(f'\nDone. {total} trials written to {out_path}')


if __name__ == '__main__':
    main()
