# semantic-nav-ros

## Isolation rule (hard constraint)

This machine also holds unrelated research work elsewhere in the user's
home directory. **Everything created for this project must live under
this project directory** (`/home/ketan/semantic-nav-ros`). Do not read,
modify, move, or reference anything outside it. The one exception is
apt-installed system packages, which are necessarily global.

If something outside this directory genuinely needs to change, stop and
ask the user first — do not do it silently.

`.claude/settings.json` enforces part of this technically
(`blockReadsOutsideWorkingDirectories`, deny rules on home-directory
dotfiles/SSH/AWS creds), but it is not exhaustive — treat it as backup,
not a substitute for actually following this rule. Note: a deny rule
using a bare `~/*` glob was tried and found to match paths at any depth
(including inside this project), which self-locked file edits — keep
deny patterns for outside-project paths fully qualified (specific
dotfiles, `~/.ssh/**`, etc.), never a bare top-level wildcard.

## ROS environment rule (hard constraint)

**Never add ROS setup to `~/.bashrc` or any other global shell config.**
The user has another ROS workspace on this machine, and sourcing two
workspaces in the same shell causes package shadowing.

All ROS/Gazebo environment setup for this project lives in
`./env.sh` at the project root, which the user sources manually per
terminal (`source env.sh`). Never source it automatically from a global
file.

## Project goal

Research paper targeting IEEE RA-L, hard deadline **2026-10-27**
(updated by the user on 2026-09-27). Prefer apt packages over source builds
throughout — no time to debug source builds.

Eventual goal: a custom `nav2_costmap_2d::Layer` plugin that writes
graduated semantic cost values (0-254) into the costmap instead of binary
occupancy, so a planner can treat a puddle differently from a wall and
can avoid glass that LiDAR passes through but a camera sees. **That
plugin is future work** — the current build phase is simulation +
baseline (stock Nav2, no tuning) only.

Core experimental mechanism: a glass wall with collision geometry (robot
physically hits it) but zero LiDAR returns (`visibility_mask` on the
`gpu_lidar` sensor vs. `visibility_flags` on the glass visual — verified
against the installed sdformat14 spec, see project history). A camera
must still see it clearly, since 3D LiDAR points get projected into
camera pixels in later work — so TF and camera_info must be correct now.

## Environment

- Ubuntu 24.04, WSL2, ROS 2 Jazzy, Gazebo Harmonic (`gz-sim` 8.x).
- **`ROS_DOMAIN_ID=47`** (chosen to avoid collision with the user's other
  ROS workspace on this machine). Set only in `env.sh`.
- Headless Gazebo (`gz sim -s`) is the primary target — batch trials must
  run unattended. GUI (`gz sim` without `-s`) is a bonus, not a
  requirement, and may hit OpenGL/WSLg issues.
- Diff-drive robot uses gz-sim's native `DiffDrive` + `JointStatePublisher`
  system plugins bridged via `ros_gz_bridge` — deliberately not
  `ros2_control`/`gz_ros2_control` (not installed, adds a control-manager
  layer not worth the debugging risk in a 3-week window).
- Nav2 bringup is stock/default config (voxel/obstacle + inflation
  layers, MPPI controller) — this is the experimental control condition
  and must not be tuned.

## Package layout

`ros2_ws/src/semantic_nav_sim` (ament_cmake) holds the simulation,
robot description, and trial-runner package. Structure is meant to leave
room for a sibling package (e.g. `semantic_nav_costmap_layer`) holding the
custom costmap plugin later, without restructuring.

## Working style

- Run every step and show real terminal output — don't assert something
  should work without evidence.
- Work in order; stop at the first failure rather than building on top of
  it.
- Commit to git at each working milestone.


## Phase 0: harness reliability (current task; gate NOT MET)

October 1: the single-trial boundary-capture diagnostic aborted on a
backward realtime step (-2.297378 s), while monotonic/boot/raw advanced
about 2 ms. Cause remains unidentified. Five regression tests passed and
live event logging worked. No new soak started; read-only host clock
investigation awaits permission under isolation. See the latest status.

Latest continuation: the independent-seed 50-trial attempt stopped at
trial 25, with 21/25 usable (three recovery-gap exclusions and one
infrastructure failure). The final attempt logged a 233.457 s clock
discrepancy and DDS/network errors; teardown was clean. Runner FDs stayed
25; RSS rose 82.10 -> 83.34 MB. No protocol is validated. Four-clock guard
and boundary snapshots have since been added and regression-tested; no
new batch followed the failure. See the latest status before resuming.

Do not begin plugin work or the data campaign until a 50-trial soak has
at least 48 usable trials and no observed loop-rate trend. The current
checkout has only `semantic_nav_sim`; a second package and its one-way
dependency do not exist yet.

Use `bash scripts/phase0.sh ...` from the project root for batch runs.
It manually sources `env.sh` and confines ROS/Gazebo logs, caches, and
scratch files to `results/phase0/runtime`. ROS/Gazebo cannot run inside
the Codex network/PID sandbox; the runner needs normal host execution.
Do not install anything or change global shell configuration.

Resource snapshots are written at batch start and after every teardown
in `<output>_resources.csv`. FD totals are limited to accessible processes;
the inaccessible count is explicit. The 15-trial diagnostic found stable
shared memory, stable runner FDs after initialization, fluctuating process
counts, and monotonically increasing runner RSS (81.13 -> 96.55 MB).
Explicit action-client/node cleanup and collection were followed by a
five-trial run with RSS 81.55–81.74 MB and constant 25 runner FDs. No DDS
segments were deleted and no daemon workaround was applied. Grouped
runner restarts have not been needed/tested at this point.

Initial pose is published only after AMCL activation and acknowledged
localization. Goals require an active bt_navigator, a discoverable
NavigateToPose action server, localization, and TF. Infrastructure errors
abort the batch, have `usable=False`, and leave navigation results empty.
Never count a no-goal attempt as a navigation outcome. Clock discontinuities
above two seconds also invalidate the attempt as infrastructure failure.

**Provisional soak candidate: RTF 0.5 / zero cooldown / no runner restarts**,
seed 20260927, using the original all-command-gap gate metric. A fresh
five-trial check passed 5/5; later recovered comparison evidence gave
0/5 usable at RTF 1.0 / 10 s cooldown and 3/5 at RTF 0.5 / no cooldown.
**Final RTF selection is still pending soak validation.** RTF 0.5 / no cooldown passed a five-trial
pilot; its controlled comparison against RTF 1.0 / 10-second cooldown was
interrupted by host clock jumps and DDS network errors. Default RTF remains
1.0 until comparison and soak validation finish. Whatever RTF, cooldown,
and restart policy is ultimately selected MUST apply identically to every
baseline and semantic condition. Never compare conditions using different
RTFs. Preserve the 1 ms physics step and stock Nav2 parameters.

Live controller_server and bt_navigator parameters both report
`use_sim_time=true`. At RTF 0.5, measured command publication remains about
20 Hz in wall time and becomes about 40 Hz in simulation time. Therefore,
reduced RTF must NOT be described as preserving controller timing or
trajectories. `loop_rate_min_hz` retains the original simulated-clock
command-arrival definition, including conservative penalties for recovery
gaps; it is not a direct measurement of controller compute duration.
`loop_rate_wall_min_hz` and raw interval CSVs retain the other clock.
Navigation timeouts now use simulated seconds; a separate wall-time
watchdog protects stalled runs. Host must remain awake during experiments.

Evidence, limitations, and exact resume commands: `docs/phase0/status.md`.
