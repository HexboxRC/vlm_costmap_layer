#!/usr/bin/env python3
"""Execute the preregistered schedule sequentially; never replace attempts."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
from run_campaign import completion_state

root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('stage',choices=['smoke','pilot'])
args=p.parse_args()
out=root/'results'/('oct13_'+args.stage)
out.mkdir(exist_ok=True)
conditions=['baseline','ground_truth','binary']
scenarios=['glass_wall','route_choice','forced_crossing'] if args.stage=='pilot' else ['route_choice','forced_crossing','glass_wall']
schedule=[]
for index in range(10 if args.stage=='pilot' else 1):
    seed=2026101300+index if args.stage=='pilot' else 20261013
    for scenario in scenarios:
        for condition in conditions[index%3:]+conditions[:index%3]:
            schedule.append(dict(seed=seed,scenario=scenario,condition=condition))
schedule_file=out/'schedule.json'
if schedule_file.exists():
    if json.loads(schedule_file.read_text())!=schedule:
        raise SystemExit('Existing schedule differs; stop and reconcile.')
else:
    schedule_file.write_text(json.dumps(schedule,indent=2)+'\n')
for i, cell in enumerate(schedule):
    name=f"{cell['seed']}_{cell['scenario']}_{cell['condition']}"
    csvfile=out/(name+'.csv')
    completion=out/(name+'.complete.json')
    if completion_state(out, cell) == 'completed':
        print(f'SKIP recorded attempt {i+1}/{len(schedule)} {name}',flush=True)
        continue
    cmd=['bash','scripts/phase0.sh','--scenario',cell['scenario'],'--condition',cell['condition'],
         '--seed',str(cell['seed']),'--trials','1','--out',str(csvfile)]
    print(f'START {i+1}/{len(schedule)} {name}',flush=True)
    with (out/(name+'.console.log')).open('w') as log:
        result=subprocess.run(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT)
    if result.returncode:
        raise SystemExit(f'Runner exited {result.returncode}; stopped for teardown/infrastructure review: {name}')
    rows=list(csv.DictReader(csvfile.open()))
    if len(rows)!=1:
        raise SystemExit(f'Expected exactly one retained attempt: {name}')
    completion.write_text(json.dumps(dict(returncode=result.returncode,row=rows[0]),indent=2)+'\n')
    row=rows[0]
    print('RESULT '+json.dumps(dict(cell,**{k:row[k] for k in ['nav2_result','collision','success','usable','loop_rate_min_hz','grass_path_m','grass_central_crossing','error']})),flush=True)
    # Development smoke failure is investigated before using the pilot budget.
    if args.stage=='smoke' and row['failure_class']=='INFRASTRUCTURE':
        raise SystemExit('Development infrastructure failure; stop before the next smoke.')
print(f'COMPLETE {len(schedule)} attempts, stage={args.stage}',flush=True)
