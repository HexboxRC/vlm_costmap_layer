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

Science competition project, research paper due **2026-09-27** (~3 weeks
from project start on 2026-09-06). Prefer apt packages over source builds
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
