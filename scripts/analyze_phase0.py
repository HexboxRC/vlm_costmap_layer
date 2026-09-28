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
        report['batch_wall_s_including_cooldown'] = elapsed
        report['usable_trials_per_hour'] = float(usable.sum()) / elapsed * 3600
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
    report['gate_pass'] = bool(len(rows) == 50 and usable.mean() >= .95 and
                              mask.sum() >= 48 and
                              not report.get('linear_trend_detected_005', True))
    fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
    ax.plot(x, y, 'o-', lw=1, ms=4, label='All trial minima (simulation clock)')
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
