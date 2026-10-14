#!/usr/bin/env python3
"""Execute the declared Oct 18 schedule; preserve pending arms and every attempt.

The cloud arm remains unavailable until the actual cloud runner is implemented.
Planning is safe now; running is date-gated and requires an explicit arm list.
"""
import argparse
import csv
from datetime import date, datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import psutil

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ('glass_wall', 'route_choice', 'forced_crossing')
CONDITIONS = ('baseline', 'binary', 'ground_truth', 'cloud_vlm')
IMPLEMENTED = ('baseline', 'binary', 'ground_truth')


def schedule():
    return [dict(seed=2026101800+i, scenario=scenario, condition=condition)
            for i in range(30) for scenario in SCENARIOS
            for condition in CONDITIONS[i % 4:] + CONDITIONS[:i % 4]]


def name(cell):
    return f"{cell['seed']}_{cell['scenario']}_{cell['condition']}"


def completion_state(directory, cell):
    stem = name(cell)
    csvfile = directory/(stem+'.csv')
    marker = directory/(stem+'.complete.json')
    if marker.exists():
        if not csvfile.exists():
            raise RuntimeError(f'Completion marker lacks CSV: {stem}')
        with csvfile.open() as stream:
            rows = list(csv.DictReader(stream))
        recorded = json.loads(marker.read_text())
        if len(rows) != 1 or recorded.get('returncode') != 0 or recorded.get('row') != rows[0]:
            raise RuntimeError(f'Completion evidence disagrees: {stem}')
        return 'completed'
    if any(directory.glob(stem+'*')):
        raise RuntimeError(f'Incomplete attempt retained; review before continuing: {stem}')
    return 'unstarted'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('plan', 'run'))
    parser.add_argument('--available', nargs='+', choices=CONDITIONS,
                        help='Explicit ready conditions; omitted conditions remain pending, not excluded')
    args = parser.parse_args()
    if args.action == 'run':
        if date.today() < date(2026, 10, 18):
            raise SystemExit('Campaign is declared for October 18; use plan before then')
        if not args.available:
            raise SystemExit('Run requires an explicit --available condition list')
        if any(arm not in IMPLEMENTED for arm in args.available):
            raise SystemExit('cloud_vlm is not implemented/deployed yet; it cannot be run or replaced by oracle')
        for process in psutil.process_iter():
            try:
                if process.pid == os.getpid() or Path(process.cwd()).resolve() != ROOT:
                    continue
                if any(Path(arg).name in ('run_matched_pilot.py', 'run_campaign.py',
                                          'capture_projection_evidence.py') for arg in process.cmdline()):
                    raise SystemExit('Another project batch/camera diagnostic is active; wait for clean completion')
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    directory = ROOT/'results/oct18_campaign'
    directory.mkdir(parents=True, exist_ok=True)
    with (directory/'.batch.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Another campaign command owns this batch')
        cells = schedule()
        schedule_file = directory/'schedule.json'
        if schedule_file.exists():
            if json.loads(schedule_file.read_text()) != cells:
                raise SystemExit('Existing campaign schedule differs; do not overwrite')
        else:
            schedule_file.write_text(json.dumps(cells, indent=2)+'\n')
        ready = set(args.available or ())
        states = [dict(cell, status=completion_state(directory, cell),
                       available=cell['condition'] in ready) for cell in cells]
        (directory/'availability.generated.json').write_text(json.dumps(states, indent=2)+'\n')
        print(f'{len(cells)} planned attempts; seeds 2026101800..2026101829; no replacement', flush=True)
        if args.action == 'plan':
            return
        with (directory/'invocations.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(metadata_utc=datetime.now(timezone.utc).isoformat(),
                                         available=list(args.available)))+'\n')
        for i, cell in enumerate(cells):
            if cell['condition'] not in ready or completion_state(directory, cell) == 'completed':
                continue
            stem = name(cell)
            csvfile = directory/(stem+'.csv')
            command = ['bash', 'scripts/phase0.sh', '--scenario', cell['scenario'],
                       '--condition', cell['condition'], '--seed', str(cell['seed']),
                       '--trials', '1', '--rtf', '.5', '--cooldown-sec', '0',
                       '--max-linear-speed', '.25', '--timeout', '90',
                       '--startup-timeout', '45', '--loop-rate-threshold', '18',
                       '--jitter-xy', '.10', '--jitter-yaw', '.10', '--out', str(csvfile)]
            print(f'START {i+1}/{len(cells)} {stem}', flush=True)
            with (directory/(stem+'.console.log')).open('x') as log:
                with (directory/'execution_order.jsonl').open('a') as ledger:
                    ledger.write(json.dumps(dict(cell, scheduled_index=i+1))+'\n')
                result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            if result.returncode:
                raise SystemExit(f'Runner exited {result.returncode}; stopped for teardown review: {stem}')
            with csvfile.open() as stream:
                rows = list(csv.DictReader(stream))
            if len(rows) != 1:
                raise SystemExit(f'Expected one retained row: {stem}')
            (directory/(stem+'.complete.json')).write_text(
                json.dumps(dict(returncode=0, row=rows[0]), indent=2)+'\n')
            print('RESULT '+json.dumps(dict(cell, **{k: rows[0][k] for k in
                  ('nav2_result', 'collision', 'success', 'usable', 'failure_class')})), flush=True)
        print('Ready arms completed; other declared arms remain pending', flush=True)


if __name__ == '__main__':
    main()
