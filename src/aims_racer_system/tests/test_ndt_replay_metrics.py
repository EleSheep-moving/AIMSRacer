"""Acceptance must measure source freshness, not repeated TF timestamps."""
import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).parents[1] / 'scripts' / 'ndt_replay_metrics.py'
spec = importlib.util.spec_from_file_location('ndt_metrics', SCRIPT)
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


class ReplayMetricsTest(unittest.TestCase):
    def samples(self):
        return [dict(stream='ekf', now=i * .005, stamp=i * .005,
                     input_age=.02, mono=i * .005) for i in range(1000)]

    def test_two_hundred_hertz_fresh_state(self):
        result = metrics.summarize(self.samples(), ndt_enabled=False, trim_seconds=0)
        self.assertTrue(result['gates']['ekf_timing'])
        self.assertAlmostEqual(result['ekf']['rate_hz'], 200)

    def test_retimestamping_cannot_hide_missing_inputs(self):
        rows = self.samples()
        for row in rows[500:]:
            row['input_age'] = 1.
        self.assertFalse(metrics.summarize(rows, ndt_enabled=False, trim_seconds=0)['gates']['ekf_timing'])

    def test_fresh_tf_cannot_hide_stale_accepted_match(self):
        rows = self.samples() + [dict(stream='ndt', now=.1, stamp=.1, accepted=True)]
        rows += [dict(stream='tf', now=i*.02, stamp=i*.02) for i in range(250)]
        result = metrics.summarize(rows, ndt_enabled=True, trim_seconds=0)
        self.assertFalse(result['gates']['map_freshness'])
        self.assertGreater(result['ndt']['max_accepted_age_s'], 4.)

    def test_late_anchor_is_not_available_in_the_past(self):
        rows = [dict(stream='ekf', now=i*.005, stamp=i*.005, input_age=.02,
                     mono=i*.005) for i in range(12000)]
        rows += [dict(stream='anchor', now=i*.1+1., stamp=i*.1, accepted=True)
                 for i in range(600)]
        result = metrics.summarize(rows, ndt_enabled=True, trim_seconds=0)
        self.assertFalse(result['gates']['map_freshness'])
        self.assertGreater(result['ndt']['accepted_age_s']['p95'], 1.)

    def test_fault_recovery_waits_for_receipt_not_source_stamp(self):
        rows = self.samples() + [dict(stream='anchor', now=13.5, stamp=10.1, accepted=True),
                                 dict(stream='anchor', now=10.1, stamp=9.9, accepted=True)]
        result = metrics.summarize(rows, injection=dict(kind='pause',end_source_sec=10.))
        self.assertFalse(result['gates']['fault_recovery'])
        self.assertAlmostEqual(result['fault']['accepted_recovery_s'], 3.5)

    def test_future_input_or_state_is_not_fresh(self):
        rows = self.samples()
        for row in rows:
            row['input_age']=-100.
            row['stamp']+=100.
        self.assertFalse(metrics.summarize(rows, ndt_enabled=False, trim_seconds=0)['gates']['ekf_timing'])

    def test_ekf_alone_cannot_pass_complete_local_chain(self):
        result = metrics.summarize(self.samples(), ndt_enabled=False, trim_seconds=0)
        self.assertFalse(result['timing_pass'])

    def test_bounded_deskew_outputs_required(self):
        rows = self.samples()
        rows += [dict(stream='cloud', now=i*.1, stamp=i*.1, points=100) for i in range(50)]
        rows += [dict(stream='deskew_status',now=4.9,stamp=4.9,values=dict(received='50',published='50',pending='0',
                  queue_age_ms='1',source_age_sec='.1',oldest_pending_age_ms='0',active_stage_age_ms='2'))]
        self.assertTrue(metrics.summarize(rows,ndt_enabled=False,trim_seconds=0)['timing_pass'])
        rows[-1]['values']['pending']='3'
        self.assertFalse(metrics.summarize(rows,ndt_enabled=False,trim_seconds=0)['timing_pass'])

    def test_one_future_sensor_cannot_hide_behind_normal_other_sensor(self):
        self.assertIsNone(metrics.input_freshness(10., dict(imu=110.,wheel=9.98)))
        self.assertAlmostEqual(metrics.input_freshness(10., dict(imu=9.99,wheel=9.98)),.02)

    def test_seed_composition_includes_rotation(self):
        # old map/odom rotates +90deg, old odom/base is (1,0,+0): result (10,1,+90)
        seed = metrics.compose_planar((10., 0., 1.5707963267948966), (1., 0., 0.))
        self.assertAlmostEqual(seed[0], 10.)
        self.assertAlmostEqual(seed[1], 1.)
        self.assertAlmostEqual(seed[2], 1.5707963267948966)

    def test_quantile_small_sample(self):
        self.assertEqual(metrics.percentile([], 95), None)
        self.assertAlmostEqual(metrics.percentile([0, 10], 95), 9.5)


if __name__ == '__main__':
    unittest.main()
