#!/usr/bin/env python3
"""Summarize every scheduled attempt with explicit missing/excluded denominators."""
import argparse
import csv
import json
import math
from pathlib import Path
from collections import Counter, defaultdict


def wilson(k,n):
    if not n:return None
    z=1.959963984540054
    mid=(k/n+z*z/(2*n))/(1+z*z/n)
    half=z*math.sqrt((k/n*(1-k/n)+z*z/(4*n))/n)/(1+z*z/n)
    return [mid-half,mid+half]


def paired_results(pairs):
    results=[]
    for scenario in ('glass_wall','route_choice','forced_crossing'):
        for left,right in (('baseline','ground_truth'),('binary','ground_truth'),('baseline','binary')):
            matched=[]
            for (seed,name),group in sorted(pairs.items()):
                if name!=scenario:continue
                arms={row['condition']:row for row in group}
                if left in arms and right in arms:matched.append((seed,arms[left],arms[right]))
            navigated=[p for p in matched if p[1]['nav2_result'] and p[2]['nav2_result']]
            usable=[p for p in navigated if p[1]['usable']=='True' and p[2]['usable']=='True']
            record=dict(scenario=scenario,left=left,right=right,completed_pairs=len(matched),
                        both_navigated=len(navigated),both_usable=len(usable),metrics={})
            for metric in ('collision','success','physical_goal_reached','grass_central_crossing'):
                sets={}
                for label,subset in [('all_navigated',navigated),('both_usable',usable)]:
                    observed=[p for p in subset if p[1].get(metric) and p[2].get(metric)]
                    counts={'both_true':0,'left_only':0,'right_only':0,'neither':0}
                    for _,a,b in observed:
                        x,y=a[metric]=='True',b[metric]=='True'
                        key='both_true' if x and y else 'left_only' if x else 'right_only' if y else 'neither'
                        counts[key]+=1
                    sets[label]=dict(n=len(observed),**counts)
                record['metrics'][metric]=sets
            results.append(record)
    return results


def analyze(directory):
    schedule=json.loads((directory/'schedule.json').read_text())
    cells=defaultdict(list);missing=[];rows=[];pairs=defaultdict(list)
    for cell in schedule:
        name=f"{cell['seed']}_{cell['scenario']}_{cell['condition']}"
        path=directory/(name+'.csv')
        data=list(csv.DictReader(path.open())) if path.exists() else []
        if len(data)!=1 or not (directory/(name+'.complete.json')).exists():
            missing.append(dict(cell,status='incomplete' if path.exists() else 'unstarted'))
            continue
        row=dict(data[0],seed=cell['seed'],source_file=path.name)
        rows.append(row);cells[(cell['scenario'],cell['condition'])].append(row)
        pairs[(cell['seed'],cell['scenario'])].append(row)
    result=[]
    for key,group in sorted(cells.items()):
        usable=[r for r in group if r['usable']=='True']
        navigated=[r for r in group if r['nav2_result']]
        collisions=sum(r['collision']=='True' for r in navigated)
        result.append(dict(scenario=key[0],condition=key[1],attempts=len(group),
            usable=len(usable),unusable=len(group)-len(usable),
            infrastructure=sum(r['failure_class']=='INFRASTRUCTURE' for r in group),
            command_gap_exclusions=sum(r['loop_rate_degraded']=='True' for r in group),
            command_gap_only_exclusions=sum(r['loop_rate_degraded']=='True' and r['failure_class']!='INFRASTRUCTURE' for r in group),
            infrastructure_with_command_gaps=sum(r['loop_rate_degraded']=='True' and r['failure_class']=='INFRASTRUCTURE' for r in group),
            navigated=len(navigated),goal_successes=sum(r['nav2_result']=='succeeded' for r in navigated),
            navigation_results=dict(Counter(r['nav2_result'] for r in navigated)),
            infrastructure_reasons=dict(Counter(r['error'] for r in group if r['failure_class']=='INFRASTRUCTURE')),
            collision_free_successes=sum(r['success']=='True' for r in navigated),
            contact_collisions=collisions,contact_rate_wilson95=wilson(collisions,len(navigated)),
            usable_contact_collisions=sum(r['collision']=='True' for r in usable),
            usable_contact_rate_wilson95=wilson(sum(r['collision']=='True' for r in usable),len(usable)),
            usable_successes=sum(r['success']=='True' for r in usable),
            grass_crossings_observed=sum(r['grass_central_crossing']=='True' for r in navigated),
            successful_crossings=sum(r['success']=='True' and r['grass_central_crossing']=='True' for r in navigated),
            successful_bypasses=sum(r['success']=='True' and r['grass_central_crossing']=='False' for r in navigated),
            grass_path_m=[float(r['grass_path_m']) for r in navigated if r['grass_path_m']],
            path_length_m=[float(r['path_length_m']) for r in navigated],
            navigation_sim_s=[float(r['navigation_sim_s']) for r in navigated],
            minimum_command_hz=[float(r['loop_rate_min_hz']) for r in group if r['loop_rate_min_hz']],
            true_path_length_m=[float(r['true_path_length_m']) for r in navigated if r.get('true_path_length_m')],
            physical_goal_reached=sum(r.get('physical_goal_reached')=='True' for r in navigated),
            physical_goal_measured=sum(bool(r.get('physical_goal_reached')) for r in navigated)))
    mismatches=[]
    for key, group in pairs.items():
        if len({(r['start_x'],r['start_y'],r['start_yaw']) for r in group})>1:
            mismatches.append(key)
    summary=dict(planned=len(schedule),completed=len(rows),missing=missing,seed_pose_mismatches=mismatches,
                 cells=result,paired_comparisons=paired_results(pairs))
    (directory/'summary.generated.json').write_text(json.dumps(summary,indent=2)+'\n')
    if rows:
        with (directory/'attempts.generated.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(summary,indent=2))
    if mismatches:raise SystemExit('Matched seeds did not produce matched poses')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    analyze(parser.parse_args().directory)
