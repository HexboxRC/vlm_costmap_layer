# Proposed Pi-hosted cloud adapter — explained before implementation

Status: pure projection and latency utilities plus stationary rendered-camera
validation are implemented; the network/ROS/cloud node is not deployed. Pi
access, OS/ROS compatibility, provider/model and credential arrangement are
pending user input, which the user explicitly deferred. No credentials belong in committed files. Local Pi model comparison
and physical robot validation remain out of scope.

## Spatial method for lidar-transparent glass

Do not project lidar returns behind glass onto its pixels. Use the visible
base of an upright floor-supported panel. The VLM response must identify
the two endpoints of the glass/floor boundary in normalized image pixels,
plus semantic class and visibility status. Grass uses a ground-surface
polygon. This explicitly restricts the benchmark to a calibrated camera,
known locally planar floor and visible ground attachment; it is not general
monocular depth recovery or arbitrary transparent-object reconstruction.

For a pixel (u,v), form the optical ray K^-1[u,v,1]. Cache the latest
available camera-to-map transform at image receipt, as authorized by the
common TF policy; do not look up the robot's later pose after the API call.
For map-frame camera position c and direction d, intersect with z=0 using
lambda=-c_z/d_z, then p=c+lambda*d. Reject lambda<=0, rays near the horizon,
out-of-range results, nonfinite inputs, invalid endpoint ordering, missing
base visibility and unacceptably large propagated pixel uncertainty.
Rasterize a visible panel's reconstructed base segment as lethal cost;
rasterize grass polygons as cost 40. Use the same input grid contract and
original simulated image timestamp as the oracle. No hidden SDF lookup,
nominal glass-location lookup or fallback to ground truth.

Test exact ray/plane algebra and known projections first, then rendered
camera boundaries against withheld world geometry, camera TF and intrinsics.
Report footprint location/orientation error and pixel-perturbation sensitivity
by range. Also test background lidar returns, occluded base, image boundary,
horizon and delayed responses. This low-mounted camera makes distant ground
intersections sensitive to a few pixels of error; determine a justified
acceptance range from calibration evidence before declaring availability.
If this assumption fails on the rendered glass, report that and revise the
method explicitly before cloud navigation, rather than borrowing oracle geometry.

## Network and latency design

Laptop runs Gazebo, camera capture, frame IDs, cached transforms, geometric
projection and cost-grid publication. Pi runs the cloud API client over the
network; remote cloud service performs VLM inference. Return observation ID,
image-space labels, errors, model/version and API metadata; never restamp an
old observation as new. Bound in-flight requests and timeouts; do not silently
apply a late result to the current camera pose.

Retain one record per attempted request, including timeout, API failure,
geometry rejection, stale result and eventual application. Primary freshness
is simulation image time to first costmap application, requiring an explicit
application acknowledgement from the layer. Separately measure laptop
round-trip elapsed on one laptop monotonic clock, Pi request duration on one
Pi monotonic clock, and processing stages locally. Do not subtract laptop
and Pi absolute timestamps. Host timing remains subject to the accepted WSL
clock limitation; keep anomaly indicators and do not present host durations
as a validated scientific clock without qualification. No time-service work.

Before cloud trials, declare pipeline-readiness criteria independently of
label correctness. Do not require the model to find the known glass before
admitting an attempt: that would select out detection failures. A connected,
calibrated pipeline can return an empty, incorrect, rejected or late label;
retain that perception result and its navigation consequences. Distinguish
pipeline startup unavailability from provider/perception failures during an
otherwise valid navigation attempt. The oracle's known-cell readiness probe
must not be reused as a correctness gate for cloud observations.

Report raw samples, n attempted/returned/applied, timeout and rejection rates,
ECDF/histogram, median, p90, p95, p99 and max. Do not compute percentiles only
on successful navigation trials or hide timeouts. Simulated age, network/API
elapsed and navigation duration are different measures and must stay distinct.

Do not issue paid API requests until provider/model and intended credentials
are available; the campaign's fixed trial budget does not imply unlimited
API spend. Establish an explicit per-trial request cap in a dated amendment
before the first cloud run, with identical cloud settings across scenarios.

## October 13 rendered-camera check

Four stationary captures at robot x=-4,-3,-2,-1.5, y=4 completed with clean
teardown. Each retains the image, CameraInfo, static camera TF composed with
Gazebo pose for evaluation only, scan, world hash and launch log. Deployment
must use the estimated map transform cached with the image; this diagnostic
does not validate localization error under motion or delayed application.
Manually inspected endpoint annotations were projected without fitting their
positions to the SDF. This is not a blinded annotation study or VLM evaluation.
Only the scoring script reads the withheld world geometry.

| Camera-to-front x distance | Maximum endpoint error | Midpoint error | Length error | Maximum sensitivity, assumed +/-2 px | Example 0.25 m sensitivity gate |
|---|---|---|---|---|---|
| 3.775 m | 0.026 m | 0.025 m | 0.010 m | 0.623 m | reject |
| 2.775 m | 0.081 m | 0.074 m | 0.057 m | 0.354 m | reject |
| 1.775 m | 0.009 m | 0.006 m | 0.008 m | 0.151 m | accept |
| 1.275 m | unavailable | unavailable | unavailable | unavailable | reject: endpoints clipped |

Orientation error was approximately zero in these axis-aligned views, with
manually chosen equal-row endpoints; it is not evidence of general orientation
accuracy. Forward lidar ranges were 9.926, 8.926, 7.925 and 7.425 m, showing
background returns through the nearer glass. Lidar is not an input to projection.
The practical window for two visible, sufficiently precise endpoints is narrow.
The assumed two-pixel bound is unvalidated for the future model; select actual
acceptance limits only after model endpoint-error measurements. Do not interpret
the gate or the small manual reconstruction error as cloud readiness.

[Raw captures, annotations and evaluation](evidence/oct13_projection.tar.gz)
retain all four views, including the clipped rejection. Eight pure projection
and latency tests pass. Remaining validation includes actual VLM outputs,
motion/cached-transform behavior, delayed responses and costmap application
acknowledgements before any claim of end-to-end label latency.
