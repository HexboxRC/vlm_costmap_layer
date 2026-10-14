# Cloud perception foundation — not yet a deployed node

Pure camera projection and latency-summary utilities, prepared alongside the
fixed pilot. They do not publish ROS data, read world/SDF geometry,
call a cloud provider or run on the Pi yet. No dependency was installed.

`GroundProjector` requires rectified pinhole intrinsics, a camera-to-map
rigid transform cached with the image, a known floor height, and explicit
range/pixel-sensitivity limits. Glass uses two visible base endpoints;
grass uses a ground polygon. It rejects missing bases, horizon rays,
invalid geometry, image-boundary uncertainty and excessive sensitivity.
Coordinates are pixel-center coordinates, not normalized fractions.
Pixel error is a caller-supplied bound to study sensitivity, not a claim
that VLM errors are calibrated or that the assumed bound holds.

Eight pure tests cover independent known-point forward/inverse projection,
yaw/translation, grass polygons, rejection, far-range sensitivity, latency
tails and censored timeout records. Run:

```bash
python3 -m unittest discover -s ros2_ws/src/semantic_nav_perception/tests -v
```

For this robot's .15 m camera height and horizontal field of view 1.4 rad,
a hypothetical +/-2 pixel error gives worst corner sensitivity roughly
.037 m at 1 m, .151 m at 2 m, .353 m at 3 m, .654 m at 4 m, and 1.065 m at
5 m. These are analytic sensitivities, not measured cloud accuracy. A
.25 m acceptance limit would reject the distant observations. Validate
actual rendered boundaries and model endpoint errors before selecting
bounds or claiming availability. Never fill missing geometry from the oracle.
Future rasterization should retain only observable, accepted portions of a
ground region rather than extrapolate an uncertain polygon beyond view.

Latency summaries retain status counts, missing measurements, raw sorted
samples and median/p90/p95/p99/max. Completed-roundtrip percentiles exclude
censored timeout/transport attempts, whose counts and attempt-elapsed
measurements remain explicit. No cross-machine timestamp subtraction.
Negative local durations become missing measurements, never a navigation veto.

Stationary rendered-image calibration is recorded in
`docs/experiments/cloud_localization_design.md`: four captures, three manually
annotated bases and one clipped-base rejection. At 1.775 m the maximum
endpoint error is 0.009 m, while assumed +/-2 pixel sensitivity is 0.151 m;
farther views are much more sensitive. This does not validate VLM accuracy.

Remaining: provider/model adapter,
Pi connection and credential configuration, network transport, ROS publisher,
application acknowledgements and live end-to-end latency evidence. The
proposed architecture is in docs/experiments/cloud_localization_design.md.
