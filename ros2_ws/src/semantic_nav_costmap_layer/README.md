# Semantic costmap layer

`semantic_nav_costmap_layer::SemanticLayer` is a Nav2 layer independent of
the simulator and perception backend. It follows the
[Nav2 Jazzy layer interface](https://docs.nav2.org/jazzy/tutorials/plugin_tutorials/writing_new_costmap2d_plugin/writing_new_costmap2d_plugin/).
It does not depend on `semantic_nav_sim`.

Input: complete `nav2_msgs/msg/Costmap` snapshots on `/semantic_costmap`,
reliable/transient-local QoS, depth 1. Grid resolution, dimensions, planar
origin pose and frame come from the message. Stamp with the **observation's
simulated time**, preserving cloud inference latency rather than restamping
an old observation as fresh. Publish refreshed snapshots as new evidence
arrives; this is not an incremental-update protocol.

Costs retain their numeric meaning: 0 adds no penalty, 1–252 are graduated
penalties, 253 is Nav2's inscribed cost, 254 is lethal, and 255 is no semantic
observation. Merge takes the maximum with known master costs, so semantic
free space cannot erase a lidar obstacle. Positive observations may replace
unknown master cells; zero observations leave unknown master cells unknown.
This is a cost field, not a calibrated occupancy probability.

Place this layer after static/obstacle/voxel layers and **before inflation**
in both global and local costmaps. Existing lethal geometry is preserved;
semantic lethal cells feed stock inflation. Intermediate penalties are
preserved but are not separately inflated as lethal obstacles.

```yaml
semantic_layer:
  plugin: semantic_nav_costmap_layer::SemanticLayer
  enabled: true
  topic: /semantic_costmap
  observation_timeout: 5.0
```

`observation_timeout` is positive simulated seconds. Old or future-stamped
snapshots are ignored. Invalid dimensions, nonfinite values and nonplanar
origins are rejected. Missing input/TF falls back to the other costmap
layers; it does not stop Nav2. A future campaign must record semantic input
availability separately from navigation success, especially for cloud input.

Transforms use latest available TF, as authorized October 6. Low-speed
operation limits (but does not quantify or eliminate) residual spatial error.
No exact sensor-time matching or clock synchronization infrastructure is
introduced. Source timestamps are used for age checks only.

For the small benchmark arena, the plugin requests a full map redraw each
cycle. That deliberately simple implementation handles replacement, expiry,
resizing and rolling-window movement without ghost costs. It is not yet
optimized for large maps. Expiry removes semantic obstacles; this is a
simulation research behavior, not a hardware safety guarantee.

The simulation package supplies an omniscient SDF ground-truth producer.
Run it through the harness so the semantic layer is inserted before inflation
in both costmaps and input readiness is checked before sending the goal:

```bash
bash scripts/phase0.sh --scenario glass_wall --condition ground_truth \
  --trials 1 --seed 20261012 --out results/phase0/glass_ground_truth.csv
```

Use a new output filename for every run. `--condition baseline` excludes
the producer and layer. Both conditions retain the validated RTF 0.5,
zero cooldown, fresh worker per trial, 0.25 m/s speed cap and 90 simulated
seconds timeout. The oracle assigns glass 254 and grass 40. `--condition binary` instead
assigns both classes 254. The route_choice and forced_crossing scenarios
exercise short versus long clean detours; use the matched pilot for
experimental evidence. The cloud-VLM adapter remains unfinished.

Tests exercise pluginlib loading and ROS delivery, graded costs, preserving
obstacles/unknown cells, expiry/replacement, frame transforms, rotated maps,
update bounds, malformed input, and stock inflation. Layer tests alone do
not establish benchmark efficacy.
