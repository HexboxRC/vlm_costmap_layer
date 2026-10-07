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
can avoid glass that LiDAR passes through but a camera sees. Plugin implementation is now on the critical path and proceeds alongside
harness validation (October 6 decision below).

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
  with only the explicitly authorized common speed cap below.

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


## October 6 decision — current instructions (supersedes historical gates)

Submission is October 27; harness target is due October 9. Speed takes
priority over exhaustive diagnosis. Time-box work off the critical path
to one hour; notify the user before exceeding it and use a documented
workaround rather than extending an investigation.

Paper scope is simulation-only: the graduated semantic costmap layer,
the glass-wall benchmark, and a ground-truth versus cloud-VLM ablation.
DROPPED: local lightweight backend on Pi and hardware validation. Both
belong in Future Work; do not implement them for this submission.

STOP time-synchronization investigation. Do not build synchronization
infrastructure, trace clock writers, or modify host time services. Check
once that Nav2, the bridge, and robot_state_publisher use simulated time
and subscribe to an advancing /clock. Remaining host-clock disagreement
is accepted as a known paper limitation, not a reason to block trials.
Host time and throughput are not reliable scientific timing measures;
report simulated navigation duration. Existing host-clock observations
may be logged but must not abort a valid simulated-time trial. A stalled
simulation still needs a bounded wall watchdog.

Accepted mitigations: low approach speed (0.25 m/s maximum linear speed,
forward and reverse) in every condition; latest-available TF for project
lookups and existing Nav2 transform tolerances. This is an explicitly
authorized common speed cap on stock Nav2; do not tune other controller
parameters to improve yield. Preserve the 1 ms physics step.

Harness acceptance replaces the former 48/50 soak/no-trend gate:
**at least 27 usable attempts in a completed 30-trial batch (90%)**.
Retain every attempt, original all-command-gap metric and 18 Hz cutoff;
report navigation outcomes separately from command-gap exclusions.
Infrastructure failures (including no goal, inactive lifecycle, rejected
or unacknowledged goals, stalled simulation) MUST remain distinct from
navigation failures in the CSV, with empty navigation-result/success
fields. This is permanent methodology. Retain raw intervals and resources.

Common protocol selected for this validation: **RTF 0.5, zero cooldown,
a fresh runner process per trial, 0.25 m/s speed cap, 90 simulated seconds
navigation timeout**. These settings MUST be identical across baseline,
ground-truth semantic, and cloud-VLM conditions. Record any change before
rerunning validation; do not compare arms with different settings.
Reduced RTF does not preserve controller timing relative to physics.
Use the per-process restart workaround for potential accumulated resources
rather than diagnosing a leak. Stop on unsafe teardown/orphan failures;
record ordinary infrastructure failures and proceed only after clean teardown.

Start plugin development while unattended batches run. Do not wait for a
perfect harness or reinstate the old 50-trial gate. Prior latest long-batch
yield was 21/25 (84%); the prior five-trial pilot was 5/5. Neither validates
the new low-speed/restart protocol. See docs/phase0/status.md for new evidence.

Use `bash scripts/phase0.sh ...` from the project root. It manually sources
`env.sh` and confines ROS/Gazebo logs, caches, and scratch files to
`results/phase0/runtime`. ROS/Gazebo requires normal host execution,
not the Codex network/PID sandbox. Do not install software or change global
shell configuration. Keep all project artifacts inside this repository.
