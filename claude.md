# Semantic Costmap Layer — Project Context

## What this is
A custom ROS 2 Nav2 costmap plugin that replaces binary LiDAR occupancy
(free=0 / lethal=254) with graduated semantic cost values, so a planner can
distinguish traversable grass from standing water from a glass wall that
LiDAR passes through entirely.

## Target and deadline
- IEEE RA-L submission: Oct 27–28, 2026 (hard)
- arXiv preprint: same date
- Open-source package v0.1.0: by Oct 15, 2026
- Everything is scoped to fit that. When something must give, cut scope,
  not rigor.

## Paper framing (affects architecture — read before designing)
The contribution is the semantic costmap layer and the glass-wall
benchmark, with a PLUGGABLE INFERENCE BACKEND. Two backends ship in this
paper:
- Cloud VLM: open-vocabulary, high accuracy, slow and high-variance latency
- Local lightweight model (Pi-class): fixed vocabulary, lower accuracy,
  fast and deterministic
The accuracy-vs-latency comparison between them IS a result. Quantized VLM
on Jetson is future work, not part of this paper. Do NOT write claims about
edge deployment latency — we have no edge hardware results.

## Architecture (strict)
Two packages in ros2_ws:
1. `semantic_costmap_layer` (C++, ament_cmake) — the reusable library.
   Becomes the public release. No knowledge of our robot, world, or
   experiment. No hardcoded topics, frames, or cost values — all via ROS
   parameters and YAML. Backends sit behind one interface so a new one can
   be added without touching the layer. Must still build if
   semantic_nav_sim is deleted. Apache-2.0.
2. `semantic_nav_sim` — the experiment harness. Worlds, robot, launch,
   trial runner, analysis, our specific cost values and scenarios.
   May depend on the library. The reverse dependency is forbidden.

## Environment rules
- Unrelated UNT research shares this WSL2 machine. Stay inside the project
  directory. Never edit ~/.bashrc or global shell config.
- ROS setup lives in env.sh, sourced manually per terminal. env.sh RESETS
  the ROS environment rather than layering onto it — a stale
  CYCLONEDDS_URI in .bashrc previously aborted every ROS 2 node
  system-wide.
- ROS 2 Jazzy, Gazebo Harmonic, WSL2. Prefer apt over source builds.
- Gazebo runs headless (gz sim -s) for all batch work.

## Built and verified
- Colcon workspace, both packages scaffolded, building clean
- Glass wall mechanism: visibility_mask (lidar/camera sensor) x
  visibility_flags (visual). Verified — opaque wall returns finite ranges
  (~3.2m), glass returns inf across its full bearing window, camera renders
  both. This is the core experimental mechanism.
- 12x12m arena with glass wall and grass patch; scenario poses in YAML
- Diff-drive robot via native gz DiffDrive + JointStatePublisher (not
  ros2_control — deliberate choice), gpu_lidar + RGB camera bridged via
  ros_gz_bridge, static TF for lidar_link/camera_link, real camera_info
- Stock Nav2 bringup: voxel/obstacle + inflation layers, MPPI controller.
  UNTUNED BY DESIGN — this is the experimental control condition. Never
  tune it.
- Trial runner with CSV logging and per-trial loop-rate columns
  (loop_rate_mean_hz, loop_rate_min_hz, loop_rate_degraded)

## Known defects
- Harness yield ~33%. Three failure modes:
  1. Progressive resource exhaustion across a batch (loop rate declined
     monotonically 18.52 -> 18.18 -> 17.54 -> 17.86)
  2. bt_navigator_not_active_timeout — goal sent before Nav2 lifecycle
     nodes finish activating
  3. Loop-rate degradation under WSL2 CPU contention; MPPI emits
     wrong-direction commands below ~18 Hz
- Fixing this is the critical path. A campaign of several hundred trials
  per condition is impossible at 33% yield.

## Timeline and gates
No phase starts until the previous gate is met WITH EVIDENCE.

| Phase | Dates | Gate |
|---|---|---|
| 0. Harness reliability | Sep 17–21 | 50-trial soak, >=95% yield, no loop-rate trend |
| 1. Plugin + ground-truth labels | Sep 22–29 | Planner routes around glass; gradient values in costmap |
| 2. Fusion + cloud VLM backend | Sep 30–Oct 6 | Live labels writing cost in active costmap |
| 3. Local backend on Pi | Oct 7–10 | Second backend behind same interface |
| 4. Simulation campaign | Oct 10–18 | Full runs, ablations, 3 baselines, both backends |
| 5. Writing | Oct 14–26 | Overlaps campaign |
| 6. Package v0.1.0 | by Oct 15 | Docs, install, license, tests, CI |
| 7. Hardware validation | Oct 18–24 | CONDITIONAL — drops if anything upstream slips |
| 8. Submit + arXiv | Oct 27–28 | — |

## Experimental design (build for this from the start)
Baselines — beating only stock Nav2 is beating a strawman:
1. Stock Nav2 (geometric only)
2. Camera detections injected as binary obstacles
3. Stock Nav2 with tuned inflation radius

Ablations:
- Ground-truth labels vs cloud VLM vs local model (isolates plugin design
  from perception error)
- Graduated cost vs binary
- With and without the temporal cache
- Sensitivity to cost value assignment

Metrics: collision rate, path length vs geodesic optimum, minimum
clearance, per-cycle added latency (full distribution, not mean),
controller loop rate.

Statistics: confidence intervals and effect sizes, not bare means. Fix
trial counts and seeds in advance. Log every excluded trial and why.

Cost control: record VLM labels once per scenario and REPLAY them for the
bulk campaign. Live inference only for a subset and hardware trials. This
bounds API spend and makes results reproducible.

## Working rules
- Show real terminal output or CSV for every claim. Never "this should
  work."
- Fail loud: any invariant violation (no goal sent, lifecycle not active,
  loop rate below threshold, orphan process) aborts the trial and is
  recorded DISTINCTLY. Infrastructure failures must never be counted as
  navigation results.
- Soak-test before trusting. 9 passing trials means nothing; 50 does.
- Commit at each working milestone.
- Append a status note here at session end: what works, what doesn't,
  what's next.
