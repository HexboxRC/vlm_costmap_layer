#!/usr/bin/env python3
"""Summarize Phase 0 CSVs and plot every attempted trial, including failures."""
import argparse
import csv
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ['MPLCONFIGDIR'] = str(ROOT / 'results/phase0/runtime/matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import linregress, t


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    args = parser.parse_args()
    with args.csv.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit('No completed trial rows')
    x = np.arange(1, len(rows)+1)
    y = np.array([float(row['loop_rate_min_hz']) if row['loop_rate_min_hz'] else np.nan
                  for row in rows])
    usable = np.array([row.get('usable', str(row['loop_rate_degraded'] == 'False')) == 'True'
                       for row in rows])
    report = {'trials': len(rows), 'usable': int(usable.sum()),
              'usable_fraction': float(usable.mean()),
              'navigation_successes': sum(row['success'] == 'True' for row in rows),
              'infrastructure_failures': sum(row.get('failure_class') == 'INFRASTRUCTURE'
                                              for row in rows)}
    resources_path = args.csv.with_name(args.csv.stem + '_resources.csv')
    if resources_path.exists():
        from datetime import datetime
        with resources_path.open() as stream:
            resources = list(csv.DictReader(stream))
        elapsed = (datetime.fromisoformat(resources[-1]['timestamp']) -
                   datetime.fromisoformat(resources[0]['timestamp'])).total_seconds()
        timing_valid = elapsed > 0 and not any(row.get('error') == 'clock_discontinuity'
                                               for row in rows)
        report['batch_timing_valid'] = timing_valid
        report['batch_calendar_elapsed_s'] = elapsed
        report['batch_wall_s_including_cooldown'] = elapsed if timing_valid else None
        report['usable_trials_per_hour'] = (float(usable.sum()) / elapsed * 3600
                                           if timing_valid else None)
        report['resources_after_teardown'] = {}
        for key in list(resources[0])[3:]:
            values = [int(row[key]) for row in resources[1:]]
            report['resources_after_teardown'][key] = {
                'first': values[0], 'last': values[-1], 'min': min(values), 'max': max(values),
                'nondecreasing': all(b >= a for a, b in zip(values, values[1:]))}
    mask = np.isfinite(y)
    if mask.sum() >= 3:
        fit = linregress(x[mask], y[mask])
        delta = t.ppf(.975, mask.sum()-2) * fit.stderr
        report['slope_hz_per_trial'] = fit.slope
        report['slope_95_ci'] = [fit.slope-delta, fit.slope+delta]
        report['linear_trend_p'] = fit.pvalue
        report['linear_trend_detected_005'] = bool(fit.pvalue < .05)
    if len(rows) >= 20 and np.isfinite(y[:10]).any() and np.isfinite(y[-10:]).any():
        first = float(np.nanmean(y[:10]))
        last = float(np.nanmean(y[-10:]))
        report['first_10_mean_hz'] = first
        report['last_10_mean_hz'] = last
        report['first_last_relative_change'] = (last-first)/first if first else None
        report['practical_trend_within_10_percent'] = bool(first and abs(last-first)/first <= .10)
    gate_metric_valid = all(row.get('loop_rate_metric', 'all_command_gaps_v1')
                            == 'all_command_gaps_v1' for row in rows)
    report['gate_metric_valid'] = gate_metric_valid
    report['gate_pass'] = bool(gate_metric_valid and len(rows) == 50 and usable.sum() >= 48 and
                              mask.sum() >= 48 and
                              not report.get('linear_trend_detected_005', True) and
                              report.get('practical_trend_within_10_percent', False))
    fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
    metric_label = ('Active FollowPath minima (simulation clock)'
                    if rows[0].get('loop_rate_metric') == 'follow_path_active_v2'
                    else 'All command-gap minima (simulation clock)')
    ax.plot(x, y, 'o-', lw=1, ms=4, label=metric_label)
    if 'raw_command_gap_min_hz' in rows[0]:
        raw = [float(row['raw_command_gap_min_hz']) if row['raw_command_gap_min_hz']
               else np.nan for row in rows]
        ax.plot(x, raw, '.', color='silver', label='Raw minima, including between-action gaps')
    ax.scatter(x[~usable], np.nan_to_num(y[~usable]), marker='x', s=65,
               color='crimson', label='Unusable / missing measurement', zorder=3)
    ax.axhline(18, color='crimson', ls='--', label='18 Hz threshold')
    if mask.sum() >= 3:
        ax.plot(x, fit.intercept+fit.slope*x, '--', color='gray', label='Linear fit')
    ax.set(xlabel='Trial index (1-based; no failures omitted)',
           ylabel='loop_rate_min_hz', title=f'{args.csv.stem}: {usable.sum()}/{len(rows)} usable')
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
    fig.savefig(args.csv.with_suffix('.png'), dpi=180)
    fig.savefig(args.csv.with_suffix('.pdf'))
    args.csv.with_suffix('.summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
