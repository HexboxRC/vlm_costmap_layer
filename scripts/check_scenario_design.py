#!/usr/bin/env python3
"""Run installed NavFn/inflation on exact arena map and semantic rectangles."""
import csv
import json
import math
from pathlib import Path
import subprocess
import yaml
import numpy as np
from PIL import Image

root = Path(__file__).resolve().parents[1]
pkg = root / 'ros2_ws/src/semantic_nav_sim'
out = root / 'results/scenario_design'
out.mkdir(parents=True, exist_ok=True)
base = np.flipud(np.array(Image.open(pkg / 'maps/arena_lidar_view.pgm')))
base = np.where(base < 128, 254, 0).astype(np.uint8)
coords = -6.5 + (np.arange(260) + .5)*.05
x,y = np.meshgrid(coords,coords)
summary=[]
for scenario in yaml.safe_load((pkg/'config/scenarios.yaml').read_text())['scenarios']:
    if 'grass_pose' not in scenario:
        continue
    cx,cy=scenario['grass_pose']; sx,sy=scenario['grass_size']
    mask=(abs(x-cx)<=sx/2)&(abs(y-cy)<=sy/2)
    for condition, cost in [('baseline',0),('ground_truth',40),('binary',254)]:
        name=scenario['name']+'_'+condition
        grid=base.copy()
        if cost:
            grid[mask]=cost
            grid[(abs(x)<=.075+1e-9)&(abs(y-4)<=1.25)]=254
        binary=out/(name+'.bin'); path=out/(name+'.csv')
        if path.exists():
            raise SystemExit(f'Refusing overwrite: {path}')
        grid.tofile(binary)
        subprocess.run([str(root/'ros2_ws/install/semantic_nav_sim/lib/semantic_nav_sim/probe_routes'),str(binary),str(path)],check=True)
        points=[(float(r['x']),float(r['y'])) for r in csv.DictReader(path.open())]
        length=exposure=0.
        for a,b in zip(points,points[1:]):
            dist=math.dist(a,b); length+=dist
            mx,my=(a[0]+b[0])/2,(a[1]+b[1])/2
            if abs(mx-cx)<sx/2 and abs(my-cy)<sy/2:
                exposure+=dist
        center=[p for p in points if abs(p[0])<1.]
        crossing=any(abs(p[1]-cy)<sy/2 for p in center)
        expected=(condition=='baseline' or (condition=='ground_truth' and scenario['name']=='forced_crossing'))
        summary.append(dict(scenario=scenario['name'],condition=condition,path_length_m=length,
                            grass_length_m=exposure,central_crossing=crossing,expected_crossing=expected,
                            hypothesis_met=crossing==expected))
print(json.dumps(summary,indent=2))
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
if not all(r['hypothesis_met'] for r in summary):
    raise SystemExit('Design hypothesis failed; retain output and revise explicitly before navigation.')
