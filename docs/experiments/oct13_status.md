# October 13 implementation and pilot status

Scenario design and contact measurement are implemented. Pilot implementation
is being frozen after the development evidence below; the 90-attempt pilot
has its own output directory and denominator. Cloud deployment is pending
Pi connection/provider details, requested from the user.

## Design

[Scenario geometry and planner evidence](scenario_design.md): the existing
12 m arena suffices; only grass dimensions/placement vary. Cost 40 produces
a short clean bypass but crosses when the binary bypass would be 16.89 m.
Stock and binary controls, generated-world hashes and matched jitter are wired.

## Measurement

[Contact metric](contact_metric.md) replaced the pose/geometry proxy within
about 13 minutes, below the two-hour limit. Twelve Python tests pass; the
seven previously verified layer tests are unchanged. Raw contacts, timing,
resources, poses and manifests are retained. Physics/controller settings are
unchanged. An evaluation-only 20 Hz PosePublisher now measures actual grass
exposure, true path length and final goal distance without feeding Nav2.
It was verified at spawn x=-1.000 and x=-.2319 immediately before a known
glass contact near x=-.225. No software installation or global configuration
change was made.

## Development smokes — not the pilot

[All nine initial smokes](evidence/oct13_smoke.tar.gz),
[summary](evidence/oct13_smoke_summary.json),
[pose and additional route diagnostics](evidence/oct13_pose_and_route_diagnostics.tar.gz).

| Scenario | Stock | Graduated | Binary |
|---|---|---|---|
| Short bypass | Crossed; usable success | Abort; excluded | Bypassed; usable success |
| Long bypass | Crossed; usable success | Crossed; usable success | Abort; excluded |
| Glass | Physical contact; usable failure | No contact; usable success | No contact; usable success |

Seven of nine were usable, six collision-free goal successes, one contact
failure, two aborts excluded by original command gaps; zero infrastructure
failures and nine clean teardowns. These initial exposure/path observations
used estimated map poses and are not silently upgraded to simulator truth.
An additional jittered short-bypass run with truth logging also aborted:
actual path 2.1594 m away from the goal, final distance 9.6736 m. It is retained.

A predeclared nominal-start diagnostic then succeeded: zero true grass
centerline exposure, true path 8.0419 m, minimum command rate 33.33 Hz.
The recorded first NavFn path bypasses above the patch near y=-3.209 at x=0.
Nav2 reported success at true final distance .2647 m; the separate physical
<=.25 m flag is false. Do not conflate estimated goal success with exact
physical goal arrival. Command-value logging was empty in this diagnostic
because the new hook initially targeted the wrong callback argument name;
fixed before pilot. The two jittered graduated failures remain unexplained;
no tuning or geometry changes were made to suppress them. This is evidence
of capability and execution instability, not a repeatability claim.

## Fixed next stages

[Preregistration and dated amendments](oct13_preregistration.md) declare 10
attempts per scenario/condition (90 total), matched seeds, condition-order
rotation and all exclusions before any pilot attempt. Run
`bash scripts/phase0.sh pilot`; progress and raw data are under
`results/oct13_pilot`. Resume skips only completed attempts and refuses to
replace an incomplete one. Analyze with
`python3 scripts/analyze_matched_pilot.py results/oct13_pilot`.

[Cloud localization/network design](cloud_localization_design.md) was
explained before implementation: visible glass base endpoints projected onto
the known floor, uncertainty/visibility rejection, cached camera transform,
no oracle fallback, and distinct Pi/laptop/simulated-age latency distributions.
The Pi hosts the cloud client, not a local inference-model comparison.
Campaign: October 18, 30 attempts per 3 scenarios x 4 conditions (360 planned),
separate seeds, writing in parallel from the same date. Cloud unavailability
must not delay ready campaign arms or be hidden behind oracle substitution.
