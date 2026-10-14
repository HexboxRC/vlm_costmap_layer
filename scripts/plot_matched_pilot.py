#!/usr/bin/env python3
"""Plot every completed simulator-truth trajectory, including excluded attempts."""
import argparse
import csv
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

runtime = Path(__file__).resolve().parents[1] / 'results/phase0/runtime'
for name in ('matplotlib', 'cache', 'tmp'):
    (runtime / name).mkdir(parents=True, exist_ok=True)
os.environ['MPLCONFIGDIR'] = str(runtime / 'matplotlib')
os.environ['XDG_CACHE_HOME'] = str(runtime / 'cache')
os.environ['TMPDIR'] = str(runtime / 'tmp')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    schedule = json.loads((directory / 'schedule.json').read_text())
    scenarios = ['glass_wall', 'route_choice', 'forced_crossing']
    conditions = ['baseline', 'ground_truth', 'binary']
    names = {'baseline': 'Stock', 'ground_truth': 'Graduated (grass 40)',
             'binary': 'Binary (grass 254)'}
    fig, axes = plt.subplots(3, 3, figsize=(12, 11), constrained_layout=True)
    complete = 0
    for row_index, scenario in enumerate(scenarios):
        for col_index, condition in enumerate(conditions):
            ax = axes[row_index, col_index]
            count = usable = contacts = successes = infra = 0
            background = False
            for cell in schedule:
                if cell['scenario'] != scenario or cell['condition'] != condition:
                    continue
                stem = f"{cell['seed']}_{scenario}_{condition}"
                marker = directory / (stem + '.complete.json')
                if not marker.exists():
                    continue
                result = json.loads(marker.read_text())['row']
                manifest = json.loads((directory / (stem + '_manifest.json')).read_text())
                # Resolve relative to the evidence directory, so an extracted archive works.
                saved_world = Path(manifest['world_paths'][scenario])
                logdir = directory / saved_world.parent.name
                if not background:
                    world = ET.parse(logdir / saved_world.name).getroot().find('world')
                    for model_name, color, alpha in [('grass_patch', 'green', .16),
                                                     ('glass_wall', 'deepskyblue', .8)]:
                        model = world.find(f"model[@name='{model_name}']")
                        pose = [float(v) for v in model.findtext('pose').split()]
                        geometry = model.find('.//visual/geometry')
                        size = [float(v) for v in next(iter(geometry)).findtext('size').split()]
                        if abs(pose[5]) > 1e-8:
                            raise ValueError('Plot assumes axis-aligned benchmark rectangles')
                        ax.add_patch(Rectangle((pose[0]-size[0]/2, pose[1]-size[1]/2),
                                               size[0], size[1], color=color, alpha=alpha))
                    ax.plot(float(result['goal_x']), float(result['goal_y']),
                            marker='*', color='black', markersize=10)
                    background = True
                count += 1
                usable += result['usable'] == 'True'
                contacts += bool(result['nav2_result']) and result['collision'] == 'True'
                successes += result['success'] == 'True'
                infra += result['failure_class'] == 'INFRASTRUCTURE'
                path = logdir / (result['trial_id'] + '_truth_poses.csv')
                points = list(csv.DictReader(path.open())) if path.exists() else []
                if not points:
                    continue
                color = ('#707070' if result['failure_class'] == 'INFRASTRUCTURE' else
                         '#c43c39' if result['collision'] == 'True' else
                         '#2266a6' if result['success'] == 'True' else '#c87f00')
                xs, ys, previous = [], [], None
                for point in points:
                    stamp = float(point['sim_time_s'])
                    if previous is not None and (stamp-previous > 2. or stamp <= previous):
                        xs.append(float('nan')); ys.append(float('nan'))
                    xs.append(float(point['x'])); ys.append(float(point['y'])); previous = stamp
                ax.plot(xs, ys,
                        color=color, linestyle='-' if result['usable'] == 'True' else '--',
                        linewidth=1.1, alpha=.7)
                ax.plot(float(points[-1]['x']), float(points[-1]['y']), '.', color=color, markersize=4)
            complete += count
            ax.set(xlim=(-6, 6), ylim=(-6, 6), aspect='equal',
                   title=f"{scenario.replace('_', ' ')} — {names[condition]}\n"
                         f"n={count}; usable={usable}; infra={infra}\ncontact={contacts}; success={successes}",
                   xlabel='World x (m)', ylabel='World y (m)')
            ax.grid(alpha=.15)
    handles = [Line2D([], [], color=c, label=t) for c, t in
               [('#2266a6', 'Nav2 success, no contact'), ('#c43c39', 'Obstacle contact'),
                ('#c87f00', 'Navigation abort/timeout'), ('#707070', 'Infrastructure failure')]]
    handles += [Line2D([], [], color='black', linestyle='--', label='Excluded attempt'),
                Line2D([], [], color='black', marker='*', linestyle='', label='Goal')]
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, -.045), ncol=3, fontsize=9)
    fig.suptitle(f'Simulator-truth trajectories: {complete}/{len(schedule)} completed attempts\n'
                 'All recorded outcomes; success uses preregistered Nav2/contact definition', fontsize=12)
    for suffix in ('png', 'pdf'):
        path = directory / ('trajectories.generated.' + suffix)
        fig.savefig(path, dpi=170, bbox_inches='tight')
        print(path)


if __name__ == '__main__':
    main()
