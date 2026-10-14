# October 13 implementation and pilot status

Scenario design and contact measurement are implemented. The fixed pilot
completed all 90 attempts from commit `4765300`, with 90 clean teardowns.
Cloud deployment is pending Pi/provider details; the user explicitly said
those can be provided later. No Pi connection or paid API call has occurred.

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

## Completed matched pilot

[Full raw evidence](evidence/oct13_pilot.tar.gz),
[summary and paired comparisons](evidence/oct13_pilot_summary.json),
[raw-evidence audit](evidence/oct13_pilot_audit.json),
[trajectory figure](evidence/oct13_pilot_trajectories.png),
[PDF figure](evidence/oct13_pilot_trajectories.pdf).

Every row below represents ten scheduled attempts, with no replacements.
Success means the preregistered Nav2 goal success without obstacle contact.
Infrastructure attempts have empty navigation-result/success fields.

| Scenario | Arm | Contact-free successes | Contacts | Nav aborts | Infrastructure | Usable | Physical goal <=.25 m |
|---|---|---:|---:|---:|---:|---:|---:|
| Glass | Stock | 0 | 10 | 0 | 0 | 9 | 0 |
| Glass | Graduated | 9 | 0 | 0 | 1 | 8 | 9 |
| Glass | Binary | 10 | 0 | 0 | 0 | 10 | 10 |
| Short bypass | Stock | 9 | 0 | 0 | 1 | 8 | 8 |
| Short bypass | Graduated | 6 | 0 | 3 | 1 | 6 | 5 |
| Short bypass | Binary | 9 | 0 | 0 | 1 | 9 | 9 |
| Forced crossing | Stock | 10 | 0 | 0 | 0 | 9 | 9 |
| Forced crossing | Graduated | 10 | 0 | 0 | 0 | 10 | 8 |
| Forced crossing | Binary | 0 | 0 | 9 | 1 | 0 | 0 |

All successful stock grass runs crossed; all successful graduated short-route
runs bypassed by centerline; all ten graduated forced-crossing runs crossed.
Binary short-route successes bypassed. Binary forced-crossing produced nine
navigation aborts and one startup failure, not successful long detours.
Its first recorded global plans in the first three seeds were 16.85–16.96 m;
the controller failure mechanism remains unresolved.

The glass contact rates among valid navigated attempts are stock 10/10,
graduated 0/9, binary 0/10. Their 95% Wilson intervals are approximately
72.2–100%, 0–29.9%, and 0–27.8%. There are nine valid stock/graduated matched
pairs, all discordant in the expected direction; seven pairs have both
attempts usable. This supports semantic lethal insertion on this benchmark,
not a distinct graduated-cost advantage on glass or a general safety guarantee.

69/90 attempts were usable (76.7%). Five were infrastructure failures
(three unhealthy evaluation-pose streams, two lifecycle startup timeouts),
and sixteen additional attempts were excluded only by command gaps. Two
infrastructure failures also had degraded command gaps: these categories
must not be added as if disjoint. The aggregate usability is below 90%.
All twelve valid navigation aborts were excluded by command gaps, so a
usable-only outcome table would conceal failures. Preserve both analyses.

All 90 attempts used distinct worker PIDs and 29 open worker FDs. The audit
found unchanged source hashes, per-scenario worlds, per-condition parameters,
matched start poses, retained raw intervals/resources and agreement between
raw contacts/trajectories and CSV metrics. Two startup failures never reached
the simulated-time check; the audit records that stage explicitly rather
than fabricating a check result. Physics/controller/protocol settings were
not tuned during the pilot. The frozen core was compared to `4765300` before
post-pilot tooling edits. The guarded resume was subsequently verified to
skip all 90 completed attempts without launching another trial.

Graduated short-route execution is not reliably solved: six successes,
three aborts and one infrastructure failure. Retain that limitation, the
centerline-versus-footprint distinction, and the independent physical-goal
flags. Nav2 reported goal success in all ten stock glass runs even though
the robot physically stopped at the glass; contact and truth telemetry
prevent those from being presented as successful physical navigation.

## Fixed next stages

[Preregistration and dated amendments](oct13_preregistration.md) declare 10
attempts per scenario/condition (90 total), matched seeds, condition-order
rotation and all exclusions before any pilot attempt. The completed raw
data are under `results/oct13_pilot`. Resume skips completed attempts and
refuses inconsistent or incomplete evidence, including startup logs without
a CSV. Analyze with
`python3 scripts/analyze_matched_pilot.py results/oct13_pilot`.

[Cloud localization/network design](cloud_localization_design.md) was
explained before implementation: visible glass base endpoints projected onto
the known floor, uncertainty/visibility rejection, cached camera transform,
no oracle fallback, and distinct Pi/laptop/simulated-age latency distributions.
The Pi hosts the cloud client, not a local inference-model comparison.
Campaign: October 18, 30 attempts per 3 scenarios x 4 conditions (360 planned),
separate seeds, writing in parallel from the same date. The
[campaign runner and writing runbook](oct18_execution.md) are ready for the
three implemented arms; four schedule/resume checks pass. No launch timer
was installed. Cloud unavailability
must not delay ready campaign arms or be hidden behind oracle substitution.

## Local perception follow-up

The pilot parent exited successfully; tool session 36828 is finished.
Provider-independent projection and latency utilities are prepared under
`ros2_ws/src/semantic_nav_perception` (eight pure tests pass). They are not
installed, connected to ROS, used by this pilot, or deployed on the Pi.
The stationary camera-capture diagnostic is next. Do not guess a Pi
endpoint, provider or credential while the user has deferred those details.
