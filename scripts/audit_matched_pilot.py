#!/usr/bin/env python3
"""Read-only raw-evidence audit; writes only its own derived JSON report."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def clipped_length(a, b, center, size):
    """Exact line/axis-aligned-rectangle clipping, independent of runner sampling."""
    lo, hi = 0., 1.
    for axis in range(2):
        delta = b[axis] - a[axis]
        left, right = center[axis]-size[axis]/2, center[axis]+size[axis]/2
        if abs(delta) < 1e-15:
            if not left < a[axis] < right:
                return 0.
            continue
        t1, t2 = (left-a[axis])/delta, (right-a[axis])/delta
        lo, hi = max(lo, min(t1, t2)), min(hi, max(t1, t2))
        if hi <= lo:
            return 0.
    return (hi-lo)*math.dist(a, b)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    schedule = json.loads((directory/'schedule.json').read_text())
    errors, audited, poses, workers, not_reached = [], [], {}, [], []
    hashes, commits, worlds, parameters = {}, set(), {}, {}

    def check(condition, name, message):
        if not condition:
            errors.append(dict(attempt=name, issue=message))

    for cell in schedule:
        name = f"{cell['seed']}_{cell['scenario']}_{cell['condition']}"
        marker = directory/(name+'.complete.json')
        if not marker.exists():
            continue
        completion = json.loads(marker.read_text())
        rows = read_csv(directory/(name+'.csv'))
        check(completion['returncode'] == 0 and len(rows) == 1, name, 'Completion marker/row count')
        if len(rows) != 1:
            continue
        row = rows[0]
        check(row == completion['row'], name, 'Completion marker disagrees with CSV')
        m = json.loads((directory/(name+'_manifest.json')).read_text())
        source_world = Path(m['world_paths'][cell['scenario']])
        logdir = directory/source_world.parent.name
        prefix = logdir/row['trial_id']
        for suffix in ('_contacts.json', '_sim_time.json', '_truth_poses.csv',
                       '_commands.csv', '_intervals.csv', '_poses.csv', '.log'):
            exists = Path(str(prefix)+suffix).exists()
            if (suffix == '_sim_time.json' and not exists and row['failure_class'] == 'INFRASTRUCTURE'
                    and row['goal_sent'] == 'False' and row['nav2_use_sim_time'] == ''):
                # The runner performs this check only after lifecycle/localization readiness.
                not_reached.append(dict(attempt=name, check='sim_time', readiness_failure=row['error']))
            else:
                check(exists, name, f'Missing raw artifact {suffix}')
        resources = read_csv(directory/(name+'_resources.csv'))
        check(any(r['boundary'] == 'after_teardown' for r in resources), name, 'Missing teardown resource snapshot')
        check(m['rtf'] == .5 and m['timeout'] == 90. and m['cooldown_sec'] == 0.
              and m['max_linear_speed'] == .25 and m['loop_rate_threshold'] == 18.,
              name, 'Common protocol differs')
        commits.add(m['git_commit'])
        for path, digest in m['source_sha256'].items():
            hashes.setdefault(path, set()).add(digest)
        actual_world = hashlib.sha256((logdir/source_world.name).read_bytes()).hexdigest()
        check(actual_world == m['world_sha256'][cell['scenario']], name, 'Saved world hash mismatch')
        worlds.setdefault(cell['scenario'], set()).add(actual_world)
        paramfile = logdir/Path(m['params_path']).name
        actual_params = hashlib.sha256(paramfile.read_bytes()).hexdigest()
        check(actual_params == m['params_sha256'], name, 'Saved parameter hash mismatch')
        parameters.setdefault(cell['condition'], set()).add(actual_params)
        poses.setdefault((cell['seed'], cell['scenario']), set()).add(
            (row['start_x'], row['start_y'], row['start_yaw']))
        workers.append(row['worker_pid'])
        infra = row['failure_class'] == 'INFRASTRUCTURE'
        if infra:
            check(row['nav2_result'] == '' and row['success'] == '', name, 'Infrastructure has navigation outcome')
        else:
            contact = json.loads(Path(str(prefix)+'_contacts.json').read_text())
            collision = any(e['navigation'] for e in contact['events'])
            check((row['collision'] == 'True') == collision, name, 'Contact flag disagrees with events')
            check(contact['counts']['ground'] > 0, name, 'No contact-sensor heartbeat')
            truth = read_csv(Path(str(prefix)+'_truth_poses.csv'))
            points = [(float(p['x']), float(p['y'])) for p in truth]
            check(bool(points), name, 'No evaluation trajectory')
            if points:
                length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
                check(abs(length-float(row['true_path_length_m'])) <= .00011, name, 'True path length mismatch')
                goal_distance = math.dist(points[-1], (float(row['goal_x']), float(row['goal_y'])))
                check(abs(goal_distance-float(row['true_goal_distance_m'])) <= .00011, name, 'True goal distance mismatch')
                check((row['physical_goal_reached'] == 'True') == (goal_distance <= .25), name, 'Physical goal flag mismatch')
                scenario = m['scenario_definitions'][0]
                center, size = scenario.get('grass_pose', [0., -4.]), scenario.get('grass_size', [3., 3.])
                exposure = sum(clipped_length(a, b, center, size) for a, b in zip(points, points[1:]))
                # The runner uses <=1 cm midpoint segments; up to 1 cm error at each boundary crossing.
                check(abs(exposure-float(row['grass_path_m'])) <= .03, name, 'Grass exposure differs from exact clipping by >3 cm')
            intervals = read_csv(Path(str(prefix)+'_intervals.csv'))
            positive = [float(r['sim_dt_s']) for r in intervals if float(r['sim_dt_s']) > 0]
            hz = 1/max(positive) if positive else 0.
            check(abs(hz-float(row['raw_command_gap_min_hz'])) <= .00011, name, 'Original command-gap metric mismatch')
            expected_usable = len(positive) > 0 and int(row['num_cmd_vel_msgs']) > 1 and hz >= 18.
            check((row['usable'] == 'True') == expected_usable, name, 'Usability disagrees with original cutoff')
        audited.append(name)
    check(all(len(v) == 1 for v in hashes.values()), '*', 'Source hashes changed between attempts')
    check(len(commits) <= 1, '*', 'Recorded implementation commit changed')
    check(all(len(v) == 1 for v in worlds.values()), '*', 'World differs within a scenario')
    check(all(len(v) == 1 for v in parameters.values()), '*', 'Parameters differ within a condition')
    check(all(len(v) == 1 for v in poses.values()), '*', 'Matched start poses differ')
    report = dict(planned=len(schedule), audited=len(audited), commits=sorted(commits),
                  unique_worker_pids=len(set(workers)), source_hashes_stable=all(len(v) == 1 for v in hashes.values()),
                  errors=errors, checks_not_reached=not_reached, audited_attempts=audited)
    (directory/'audit.generated.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'audited_attempts'}, indent=2))
    if errors:
        raise SystemExit('Raw evidence audit failed; inspect before drawing conclusions')


if __name__ == '__main__':
    main()
