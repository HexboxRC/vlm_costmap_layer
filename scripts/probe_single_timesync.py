#!/usr/bin/env python3
"""Approved diagnostic only: temporarily stop timesyncd, probe, then restore it.

Affects the distribution's system-wide time service. Requires separate explicit
approval and root. Never disables the service or changes clock parameters.
"""
import argparse
import csv
import os
import signal
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
SERVICE = 'systemd-timesyncd.service'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    out = args.out.resolve()
    if not out.is_relative_to(ROOT) or out.exists():
        parser.error('Use a new output directory inside the project')
    if os.geteuid() != 0:
        parser.error('Requires root and explicit approval for the temporary service stop')
    if subprocess.run(['systemctl', 'is-active', '--quiet', SERVICE]).returncode:
        parser.error('Service must initially be active; no changes made')
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}; restoring service if stopped')

    signal.signal(signal.SIGTERM, interrupted)
    out.mkdir(parents=True)
    reader = ROOT / 'results/phase0/host_clock_20261001/read_timex'
    if not reader.is_file():
        parser.error('Read-only timex probe missing; no service changes made')
    with (out / 'clocks.csv').open('x') as stream, (out / 'actions.txt').open('x') as log:
        writer = csv.writer(stream)
        writer.writerow(['phase', 'realtime_s', 'monotonic_s', 'boottime_s', 'raw_s', 'timex'])

        def command(verb):
            result = subprocess.run(['systemctl', verb, SERVICE], capture_output=True, text=True)
            log.write(f'{verb}: exit={result.returncode}\n{result.stdout}{result.stderr}')
            log.flush()
            result.check_returncode()

        def sample(phase, seconds):
            print(f'{phase}: {seconds} seconds', flush=True)
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                clocks = [time.time(), time.monotonic(), time.clock_gettime(time.CLOCK_BOOTTIME),
                          time.clock_gettime(time.CLOCK_MONOTONIC_RAW)]
                timex = subprocess.check_output([str(reader)], text=True).strip()
                writer.writerow([phase, *clocks, timex]); stream.flush()
                time.sleep(0.25)

        sample('before', 20)
        try:
            command('stop')
            sample('timesyncd_stopped', 96)
        finally:
            command('start')
            print('systemd-timesyncd restored', flush=True)
        sample('restored', 20)
    print(f'Evidence saved in {out}', flush=True)


if __name__ == '__main__':
    main()
