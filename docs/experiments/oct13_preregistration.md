# October 13 design diagnostics and matched pilot — declared before new runs

The October 12 smoke is development evidence, not pilot/campaign data.
The old grass=0 condition is superseded by grass=40, explicitly authorized
October 13. Existing October 6 harness validation is historical; contact
instrumentation changes require a fresh common-protocol smoke before pilot.

## Scenario design diagnostics

Keep arena walls/static map, glass geometry, start/goal and all stock
planner/controller parameters. Grass is visual-only and traversable.
Nominal grass runs go from (-4,-4,0) to (4,-4); glass from (-4,4,0) to (4,4).
Route choice: grass rectangle center (0,-4), size 3 x 1.5 m.
Forced crossing: grass center (0,-2), size 3 x 7 m; its lower edge at -5.5
leaves only a narrow, inflated boundary corridor, while the upper detour is long.
Baseline excludes semantic input; oracle assigns glass=254, grass=40;
binary uses identical oracle geometry with both classes=254.

Before navigation, use installed NavFn Dijkstra with stock inflation
(radius .7, scaling 3, robot radius .22, resolution .05) to check all six
grass scenario/condition combinations. These are design diagnostics, not
trials or evidence of controller behavior. Preserve all variants if design
changes are needed. Do not tune controller/planner to force divergence.
Then perform one development navigation smoke per scenario/condition,
seed 20261013. Preserve failures; no selective retries presented as success.

## Pilot (after contact decision and design validation)

Scenarios: glass_wall, route_choice, forced_crossing.
Conditions: baseline, ground_truth (grass=40), binary (grass=254).
Ten attempts per scenario/condition: 90 attempts total. No replacement
of excluded or unsuccessful attempts. Matched independent seeds:
2026101300 through 2026101309, each with one attempt per cell. Same start
jitter +/- .10 m and yaw +/- .10 rad in all arms. Paired seed indexes are
run in ascending order. Scenario order glass, route, forced. Within a
scenario rotate the condition order by seed index modulo 3 to distribute
time/order effects. New interpreter and clean teardown between all runs.

Shared protocol: RTF .5, zero cooldown, 1 ms physics, speed cap +/- .25 m/s,
90 simulated seconds navigation timeout, startup timeout 45 wall seconds.
Bounded wall watchdog remains. Host clock steps do not invalidate valid
simulated navigation. No controller tuning or time-service investigation.

Retain every attempted run, manifest/source hashes, contact events,
trajectory, command intervals and resources. Report navigation outcome and
exclusion independently. Original all-command-gap minimum must be >=18 Hz
and have >1 command message for usable classification. Missing goal,
inactive lifecycle, goal rejection/ack timeout, missing required sensor or
semantic readiness, stalled simulation and exceptions are infrastructure
failures, with empty navigation result/success. Navigation abort/timeout
and physical contact are navigation failures, not infrastructure failures.
Stop the batch on unsafe teardown or surviving project processes.

Primary glass metric: any obstacle contact during navigation; success
requires goal success with no obstacle contact. Grass metrics: length
inside grass, crossing vs bypass through the central x band [-1,1], total
path length, simulated duration, goal completion and contacts. Record
trajectory-derived metrics on all navigated attempts, including exclusions.
Clean vs crossing is not judged solely by success. Hypothesis: baseline
crosses both grass variants; binary bypasses both (or fails); graduated
bypasses route_choice and crosses forced_crossing. A contrary result is
reported rather than changing the pilot after inspection.

## October 18 campaign declaration

Starts October 18 regardless; writing begins concurrently. Separate from
pilot. Thirty attempts per scenario/condition, seeds 2026101800..2026101829,
three scenarios above, baseline / binary / ground_truth / cloud_vlm:
360 planned attempts. Same protocol, order rotation and exclusion rules.
Cloud data run live for this initial fixed campaign; record API usage and
full latency samples. No replay presented as live inference. If cloud or
any condition is unavailable October 18, start ready arms and mark that
arm pending/unavailable, never silently substitute oracle data or shift
the start date. Amendments require a dated committed record before the
first affected run. This is a fixed sample budget, not a power claim.
Report per-cell denominators, 95% Wilson intervals for rates, paired seed
comparisons, full outcome/exclusion tables and latency distributions
(median/p90/p95/p99/max plus raw observations and timeout counts).
