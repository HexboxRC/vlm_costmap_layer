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
