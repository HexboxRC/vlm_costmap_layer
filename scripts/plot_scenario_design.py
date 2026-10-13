#!/usr/bin/env python3
"""Plot saved design-diagnostic paths; does not rerun the planner."""
import csv
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[1]
pkg=root/'ros2_ws/src/semantic_nav_sim'
out=root/'results/scenario_design'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
fig, axes = plt.subplots(1, 2, figsize=(11, 5), constrained_layout=True)
for ax, name in zip(axes, ['route_choice','forced_crossing']):
    scenario=next(s for s in yaml.safe_load((pkg/'config/scenarios.yaml').read_text())['scenarios'] if s['name']==name)
    cx,cy=scenario['grass_pose']; sx,sy=scenario['grass_size']
    ax.add_patch(Rectangle((cx-sx/2,cy-sy/2),sx,sy,color='green',alpha=.18,label='Grass'))
    ax.add_patch(Rectangle((-.075,2.75),.15,2.5,color='deepskyblue',label='Glass'))
    for condition in ['baseline','ground_truth','binary']:
        points=list(csv.DictReader((out/(name+'_'+condition+'.csv')).open()))
        ax.plot([float(p['x']) for p in points],[float(p['y']) for p in points],label=condition)
    ax.set(xlim=(-6,6),ylim=(-6,6),aspect='equal',title=name.replace('_',' '),xlabel='x (m)',ylabel='y (m)')
    ax.legend(fontsize=8)
fig.savefig(out/'routes.png',dpi=160)
