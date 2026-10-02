#!/usr/bin/env python3
"""After explicit approval, briefly trace clock-setting syscall callers.

Creates an isolated tracefs instance; never changes clocks or services.
Requires root and an already mounted tracefs. No packages are installed.
"""
import argparse
import os
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=40)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    output = args.out.resolve()
    if not output.is_relative_to(ROOT) or not 1 <= args.seconds <= 60:
        parser.error('Output must be inside this project; duration must be 1–60 seconds')
    if output.exists():
        parser.error('Refusing to overwrite an existing trace')
    if os.geteuid() != 0:
        parser.error('Root required; obtain explicit user approval before running with sudo')
    base = Path('/sys/kernel/tracing')
    events = ('sys_enter_adjtimex', 'sys_enter_clock_adjtime', 'sys_enter_settimeofday',
              'sys_enter_clock_settime')
    for event in events:
        if not (base / 'events/syscalls' / event / 'enable').exists():
            raise SystemExit(f'Required trace event unavailable: {event}; no changes made')
    instance = base / 'instances' / f'semantic_nav_clock_{os.getpid()}'
    instance.mkdir()
    try:
        (instance / 'tracing_on').write_text('0')
        for event in events:
            (instance / 'events/syscalls' / event / 'enable').write_text('1')
        (instance / 'tracing_on').write_text('1')
        print(f'Capturing only clock-setting syscall callers for {args.seconds} seconds', flush=True)
        try:
            time.sleep(args.seconds)
        finally:
            (instance / 'tracing_on').write_text('0')
            with output.open('x') as stream:
                stream.write((instance / 'trace').read_text())
    finally:
        (instance / 'tracing_on').write_text('0')
        for event in events:
            (instance / 'events/syscalls' / event / 'enable').write_text('0')
        instance.rmdir()
    print(f'Saved {output}; isolated trace instance removed', flush=True)


if __name__ == '__main__':
    main()
