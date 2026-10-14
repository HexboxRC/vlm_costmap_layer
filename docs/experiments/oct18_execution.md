# October 18 execution and writing checklist

The binding budget and exclusions are in
[the preregistration](oct13_preregistration.md). This runbook does not change
them. October 13 pilot evidence remains separate from campaign evidence.

## Fixed campaign

30 start-jitter seeds, 2026101800–2026101829; three scenarios; four declared
conditions; 360 planned attempts. The condition order rotates by seed index
over baseline, binary, ground_truth, cloud_vlm. An unavailable arm remains
pending; it does not become an exclusion, an oracle run, or a replacement.
Internal AMCL/MPPI random streams are not controlled by these seeds.

`scripts/run_campaign.py plan` materializes the exact schedule under
`results/oct18_campaign`. Its pure checks verify cell counts, matched seed
coverage, order rotation and refusal to overwrite incomplete evidence.
It does not start ROS or consume trial counts. Actual runs are date-gated
to October 18 or later and require an explicit list of ready conditions.
The cloud arm is deliberately unavailable in the runner until deployment,
localization validation and latency instrumentation are complete.

Run through `scripts/phase0.sh`. Start ready arms on October 18 with:

```bash
bash scripts/phase0.sh campaign run --available baseline binary ground_truth
```

Use `bash scripts/phase0.sh campaign plan` for a read-only trial plan (it
writes only the schedule and derived availability file). The common
navigation protocol stays unchanged. Record any amendment before its first affected run. Do not
run a camera diagnostic or another batch alongside a campaign. Stop and
review any nonzero child exit or incomplete prior attempt; do not delete
its files to make resume succeed. Normal infrastructure failures with clean
teardown remain recorded in their original scheduled cells.
The invocation and execution-order ledgers retain readiness decisions and
actual ordering. If a pending arm runs later, report the resulting temporal
imbalance; do not imply that all four arms ran interleaved from the start.

## Before enabling cloud

- Obtain Pi connection details, provider/model and project-local credential
  arrangement; keep secrets out of git and logs.
- Test the visible-glass-base localization method against held-out rendered
  geometry. Report pixel and world-position error by range, including failed
  or rejected observations. Analytic ray tests alone are insufficient.
- Freeze image resolution/cadence, model/version and prompt, temperature if
  supported, timeouts, uncertainty/range limits and per-trial request cap in
  a dated amendment. Keep these identical across the cloud scenarios.
- Verify original image timestamps, cached transforms, rejected/stale results
  and actual costmap-application acknowledgements. No ground-truth fallback.
- Retain end-to-end latency samples and failure/censoring counts. Report
  distributions, including median/p90/p95/p99/max, rather than just a mean.
  Do not subtract absolute timestamps from different machines.

## Writing starts October 18 alongside the campaign

Prepare the methods and system diagram from the frozen implementation while
runs execute. Fill result tables only from the saved data and regenerated
analysis. Every figure must identify its stage, number of attempted runs,
exclusions and observed denominator; do not mix development diagnostics,
pilot and campaign to enlarge a sample after looking at outcomes.

The paper's graduated-cost claim rests on the two grass scenarios. Glass
is evidence for camera-derived obstacle insertion; binary lethal input can
also solve it. Present grass=40 as an explicit benchmark cost, not a learned
probability or a calibrated measure of physical risk. A failed binary detour
is a navigation failure; the planner's longer route and the controller's
failure mechanism are different observations.

Grass exposure measures the robot centerline, not the full swept footprint.
The graduated bypass can skim the patch with a wheel or chassis edge; do not
describe it as zero whole-robot contact with grass. No inflation/padding
change is implied by reporting this limitation.

Report contact rates on all navigated attempts and the preregistered usable
subset, with denominators and confidence intervals. Command-gap exclusions
can coincide with navigation aborts, so excluded runs must remain visible;
the usable subset is not an unbiased substitute for all attempted outcomes.
Infrastructure failures remain separate, with empty navigation outcomes.

Keep Nav2-reported goal success and simulator-truth goal distance separate.
The contact-based success definition must not hide localization error or
imply physical arrival when the independent <=.25 m flag is false. Plot
actual world trajectories; label estimated trajectories as diagnostics.

Limitations to retain: simulation-only evidence; RTF .5 does not preserve
controller timing relative to physics; accepted host-clock uncertainty;
known planar floor and visible ground attachment for monocular glass
localization; endpoint/pixel uncertainty; API availability and label age;
and small per-cell pilot sample sizes. Local Pi model comparison and
physical robot validation remain Future Work.

The campaign script is an executable run plan, not an installed timer or an
automatic October 18 launch. A live cloud deployment and API request budget
are still required before that arm can run.
