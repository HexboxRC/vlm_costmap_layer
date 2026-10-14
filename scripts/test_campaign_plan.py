#!/usr/bin/env python3
"""Pure schedule/resume checks: no ROS processes or trial-budget consumption."""
from collections import Counter
import csv
import json
from pathlib import Path
import tempfile
import unittest

from run_campaign import CONDITIONS, ROOT, SCENARIOS, completion_state, name, schedule


class CampaignPlanTests(unittest.TestCase):
    def setUp(self):
        scratch = ROOT/'results/phase0/runtime/tmp'
        scratch.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix='campaign_checks_', dir=scratch)
        self.directory = Path(self.temp.name)
        self.cell = schedule()[0]

    def tearDown(self):
        self.temp.cleanup()

    def test_full_budget_and_rotated_matches(self):
        cells = schedule()
        self.assertEqual(len(cells), 360)
        self.assertEqual(len({name(c) for c in cells}), 360)
        self.assertEqual(Counter((c['scenario'], c['condition']) for c in cells),
                         Counter({(s, a): 30 for s in SCENARIOS for a in CONDITIONS}))
        for i in range(30):
            for scenario in SCENARIOS:
                group = [c['condition'] for c in cells if c['seed'] == 2026101800+i
                         and c['scenario'] == scenario]
                self.assertEqual(tuple(group), CONDITIONS[i % 4:] + CONDITIONS[:i % 4])

    def test_incomplete_artifact_cannot_be_replaced(self):
        self.assertEqual(completion_state(self.directory, self.cell), 'unstarted')
        (self.directory/(name(self.cell)+'.console.log')).write_text('retained failed startup')
        with self.assertRaisesRegex(RuntimeError, 'Incomplete attempt'):
            completion_state(self.directory, self.cell)

    def test_complete_requires_consistent_evidence(self):
        row = dict(success='False', usable='False')
        path = self.directory/(name(self.cell)+'.csv')
        with path.open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row)); writer.writeheader(); writer.writerow(row)
        marker = self.directory/(name(self.cell)+'.complete.json')
        marker.write_text(json.dumps(dict(returncode=0, row=row)))
        self.assertEqual(completion_state(self.directory, self.cell), 'completed')
        marker.write_text(json.dumps(dict(returncode=0, row=dict(row, success='True'))))
        with self.assertRaisesRegex(RuntimeError, 'disagrees'):
            completion_state(self.directory, self.cell)

    def test_missing_csv_does_not_repeat_attempt(self):
        (self.directory/(name(self.cell)+'.complete.json')).write_text('{}')
        with self.assertRaisesRegex(RuntimeError, 'lacks CSV'):
            completion_state(self.directory, self.cell)


if __name__ == '__main__':
    unittest.main(verbosity=2)
