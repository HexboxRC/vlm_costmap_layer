# Clock discrepancy investigation

The user confirmed the Windows host stayed awake during the observed
clock discrepancies. Do not describe these as established Windows sleep
or suspend events.

The runner measured a 2265.176 s divergence between CLOCK_REALTIME and
CLOCK_MONOTONIC during a trial taking only 40.322 monotonic seconds. Earlier
attempts also exhibited discrepancies. This proves disagreement between
clocks, not its cause. DDS UDP errors accompanied some earlier events.

Read-only diagnostics on 2026-09-27:

- Kernel: `6.18.33.2-microsoft-standard-WSL2`.
- Current clocksource: `tsc`. Available: `tsc`,
  `hyperv_clocksource_tsc_page`, `hyperv_clocksource_msr`, `acpi_pm`.
- Python realtime is CLOCK_REALTIME; monotonic/perf_counter are
  CLOCK_MONOTONIC, with reported 1 ns resolution.
- `timedatectl`: NTP enabled and synchronized, timezone America/Chicago,
  RTC in UTC. At 21:06 CDT, NTP offset was -308.146 ms, frequency correction
  +4.676 ppm; this snapshot does not explain the prior large discrepancy.
- The timesyncd journal reported server contact changes at 12:55, 15:12,
  18:09, and 19:51, but no message identifying a large step. The queried
  kernel journal returned no clocksource/timekeeping/suspend/resume entries.
  Absence of such messages does not exclude a clock correction or VM pause.
- A 120-second probe at 20:49–20:51 found realtime/monotonic drift of only
  a few microseconds. Boot time tracked monotonic within microseconds.
  Raw probe: `results/phase0/clock_probe_20260927_204940.csv`.

Historical cause remains **unconfirmed**. The earlier trial files did not
record enough independent clocks to distinguish a realtime correction
from a guest elapsed-clock interruption. No Windows power policy, kernel
clocksource, time-sync service, or global shell configuration was changed.

New command-interval CSVs include realtime, monotonic, boot, and raw
monotonic timestamps. All raw command gaps are retained. A clock discrepancy
above two seconds is conservatively classified as infrastructure failure
and stops the batch; this is data-quality protection, not a claim about
which clock or host component failed. If it recurs, compare the four clock
deltas before changing the guard or proposing system-level mitigation.

## Recurrence during independent-seed soak

`resumed_raw_soak50.csv` stopped on attempt 25 with a 233.457 s clock
discrepancy and DDS UDP / Gazebo network-unreachable errors. Teardown was
verified clean. The 729 command intervals cover only 36.552 s before the
interruption; the largest realtime-minus-monotonic adjacent delta in that
window was about 0.000251 s. Last command realtime: 1790569913.138. Those
samples do not straddle the discrepancy, so boot/raw clocks still cannot
identify its cause. The earlier awake-host statement must not be replaced
with an unsupported claim of sleep.

The runner now also writes per-trial `_clocks.csv` event snapshots,
including both sides of navigation guard detection and trial/teardown
boundaries. These do not depend on command messages arriving. A regression
test verifies event capture with no commands and a simultaneously ready
navigation result. No additional batch has been run since this failure.

## October 1: captured backward realtime step

Single-trial diagnostic `clock_boundary_smoke_20261001.csv` stopped at the
guard. Adjacent boundary deltas: realtime -2.297378 s, monotonic +0.002189 s,
boot +0.002190 s, raw monotonic +0.002034 s. This is now direct evidence
of a backward realtime step relative to all three elapsed clocks for this
event. It does not identify who changed realtime or retrospectively explain
older events. Startup monotonic/raw spans were 13.990/12.965 s, also requiring
investigation. Read-only host time-sync/clocksource diagnostics await user
permission under project isolation. No further batch has been started.
