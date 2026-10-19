# October 18 campaign status

Resumed from implementation commit `52a33f0`. The four pure campaign
schedule/resume tests passed, and planning materialized the declared 360
cells without changing the protocol. Started on the host with:

```bash
bash scripts/phase0.sh campaign run --available baseline binary ground_truth
```

The active tool session is `94495`. At this note's creation the first attempt,
`2026101800_glass_wall_baseline`, is running; no completion marker exists yet.
Do not launch another simulation, camera diagnostic or batch concurrently.
The runner stops on nonzero child exit and retains incomplete evidence for
review. Ordinary infrastructure failures proceed only through clean teardown.

There are 270 ready-arm attempts and 90 pending cloud attempts. The pending
arm is not an exclusion or oracle substitution. Pi/provider details remain
deferred; no cloud deployment or paid API request was performed. Actual order
and readiness decisions are retained under `results/oct18_campaign` in
`execution_order.jsonl` and `invocations.jsonl`; individual completion markers
contain each retained CSV row. This status note is a launch record, not a
claim that the campaign completed or met the usability target.

Writing began with [the methods draft and system diagram](../paper/methods_draft.md),
based on frozen source and preregistration. Campaign results are intentionally
pending. Literature, submission formatting and measured cloud results remain
unfinished. The earlier pilot and manual projection diagnostics retain their
separate evidence boundaries.

Next: monitor the running campaign; stop and inspect any teardown failure;
analyze saved campaign evidence separately from the pilot after completion.
Finish cloud integration only after the required connection/provider inputs
and the declared readiness/validation steps are available.
