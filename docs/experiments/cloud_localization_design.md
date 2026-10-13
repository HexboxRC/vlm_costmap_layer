# Proposed Pi-hosted cloud adapter — explained before implementation

Status: design only; implement after the matched pilot. Pi access, OS/ROS
compatibility, provider/model and credential arrangement are pending user
input. No credentials belong in committed files. Local Pi model comparison
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

Report raw samples, n attempted/returned/applied, timeout and rejection rates,
ECDF/histogram, median, p90, p95, p99 and max. Do not compute percentiles only
on successful navigation trials or hide timeouts. Simulated age, network/API
elapsed and navigation duration are different measures and must stay distinct.

Do not issue paid API requests until provider/model and intended credentials
are available; the campaign's fixed trial budget does not imply unlimited
API spend. Establish an explicit per-trial request cap in a dated amendment
before the first cloud run, with identical cloud settings across scenarios.
