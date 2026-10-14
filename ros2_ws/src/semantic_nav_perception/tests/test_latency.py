"""Timeout and clock-anomaly records remain visible in latency summaries."""
import unittest
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from semantic_nav_perception.latency import elapsed_seconds, summarize


class LatencyTest(unittest.TestCase):
    def test_tail_and_timeouts_not_hidden_behind_success_mean(self):
        rows=[dict(status='applied',laptop_roundtrip_s=v) for v in [1.,1.,1.,20.]]
        rows.append(dict(status='timeout',attempt_elapsed_s=30.,laptop_roundtrip_s=None))
        rows.append(dict(status='transport_error',laptop_roundtrip_s=None))
        summary=summarize(rows)
        self.assertEqual(summary['attempted'],6)
        self.assertEqual(summary['status_counts']['timeout'],1)
        d=summary['durations']['laptop_roundtrip_s']
        self.assertEqual(d['n'],4)
        self.assertEqual(d['missing_or_invalid'],2)
        self.assertEqual(d['median'],1.)
        self.assertGreater(d['p95'],15.)
        self.assertEqual(d['max'],20.)
        self.assertEqual(summary['durations']['attempt_elapsed_s']['max'],30.)
        self.assertEqual(summary['censored_roundtrips'],2)
        self.assertEqual(summary['durations']['sim_age_at_apply_s']['n'],0)

    def test_negative_duration_is_flaggable_not_zero_or_navigation_veto(self):
        self.assertIsNone(elapsed_seconds(5,4))
        self.assertAlmostEqual(elapsed_seconds(1000000000,2000000000),1.)
