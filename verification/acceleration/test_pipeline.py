"""Whole-pipeline causality and evaluation contracts with synthetic truth."""
import unittest
from types import SimpleNamespace
import numpy as np
from observer import ObserverConfig
from replay import replay
from evaluate import received_hold, gap_interp, evaluate_run, onset_lags


class PipelineTests(unittest.TestCase):
    def test_hold_cannot_use_future_received_output(self):
        x = received_hold(np.array([.02, .05]), np.array([2., 5.]),
                          np.array([.01, .02, .04, .05, .09]), .03)
        np.testing.assert_allclose(x, [np.nan, 2., 2., 5., np.nan], equal_nan=True)

    def test_reference_interpolation_excludes_large_gaps(self):
        x = gap_interp(np.array([0., .02, .20]), np.array([0., 2., 20.]),
                       np.array([.01, .02, .10, .20]), .06)
        np.testing.assert_allclose(x, [1., 2., np.nan, 20.], equal_nan=True)

    def test_delayed_lio_does_not_change_earlier_emission(self):
        origin = 1_770_000_000_000_000_000
        events = []
        for k in range(100):
            stamp = origin + k*10_000_000
            events.extend([(stamp+1_000_000, stamp, 'speed', 0., True),
                           (stamp+2_000_000, stamp, 'force', .5, True)])
        delayed = events + [(origin+705_000_000, origin+400_000_000, 'gravity', .5, True)]
        base = SimpleNamespace(origin_ns=origin, events=sorted(events))
        extra = SimpleNamespace(origin_ns=origin, events=sorted(delayed))
        config = ObserverConfig(use_imu=True, use_gravity=True)
        a, _, _ = replay(base, config)
        b, counters, _ = replay(extra, config)
        np.testing.assert_array_equal(a[a[:,0]<.705], b[b[:,0]<.705])
        self.assertGreater(counters['delayed'], 0)
        self.assertAlmostEqual(b[0,0], .002, places=14)
        self.assertTrue(np.any(a[a[:,0]>.705,2] != b[b[:,0]>.705,2]))

    def test_forecast_metrics_constant_acceleration(self):
        origin = 1_770_000_000_000_000_000
        ts = np.arange(0., 10., .005)
        speed = np.clip(ts-3., 0., 3.)
        accel = ((ts>=3.) & (ts<6.)).astype(float)
        ns = origin+np.rint(ts*1e9).astype(np.int64)
        run = SimpleNamespace(origin_ns=origin, start_s=3., end_s=7.,
                              imu_receipt_ns=ns, imu_source_ns=ns, gyro=np.zeros((len(ts),3)),
                              wheel_source_ns=ns, wheel_speed=speed,
                              lio_source_ns=ns, lio_speed=speed)
        estimates = np.column_stack([ts, speed, accel, np.full(len(ts),.01),
                                     np.full(len(ts),np.nan)])
        metrics = evaluate_run(run, estimates)
        self.assertLess(metrics['forecast_100ms_rmse_mps'], .02)
        self.assertLess(metrics['stationary_sigma_mps2'], 1e-10)
        self.assertGreater(metrics['coverage_fraction'], .99)

    def test_onset_does_not_invent_crossing_at_window_boundary(self):
        grid=np.arange(0.,3.,.005)
        proxy=np.zeros(len(grid));proxy[(grid>=1)&(grid<1.4)]=1.
        measured=np.ones(len(grid))
        m=onset_lags(grid,proxy,measured,np.ones(len(grid),dtype=bool))
        self.assertEqual(m['acceleration_onset_count'],1)
        self.assertEqual(m['acceleration_onset_missed'],1)

    def test_onset_does_not_treat_coverage_start_as_crossing(self):
        grid=np.arange(0.,3.,.005)
        proxy=np.zeros(len(grid));proxy[(grid>=1)&(grid<1.4)]=1.
        measured=np.full(len(grid),np.nan);measured[grid>=.9]=1.
        m=onset_lags(grid,proxy,measured,np.ones(len(grid),dtype=bool))
        self.assertEqual(m['acceleration_onset_missed'],1)

    def test_onset_reports_signed_lag_of_actual_sustained_crossing(self):
        grid=np.arange(0.,3.,.005)
        proxy=np.zeros(len(grid));proxy[(grid>=1)&(grid<1.4)]=1.
        measured=np.zeros(len(grid));measured[(grid>=1.1)&(grid<1.6)]=1.
        m=onset_lags(grid,proxy,measured,np.ones(len(grid),dtype=bool))
        self.assertEqual(m['acceleration_onset_missed'],0)
        self.assertAlmostEqual(m['acceleration_onset_lag_median_ms'],100.)

    def test_active_window_does_not_invent_reference_onset(self):
        grid=np.arange(0.,3.,.005)
        proxy=np.zeros(len(grid));proxy[(grid>=.8)&(grid<1.4)]=1.
        m=onset_lags(grid,proxy,proxy,grid>=1.)
        self.assertEqual(m['acceleration_onset_count'],0)

    def test_one_measured_onset_cannot_match_two_reference_events(self):
        grid=np.arange(0.,3.,.005)
        proxy=np.zeros(len(grid))
        proxy[(grid>=1)&(grid<1.3)]=1.;proxy[(grid>=1.8)&(grid<2.1)]=1.
        measured=np.zeros(len(grid));measured[(grid>=1.4)&(grid<1.6)]=1.
        m=onset_lags(grid,proxy,measured,np.ones(len(grid),dtype=bool))
        self.assertEqual(m['acceleration_onset_count'],2)
        self.assertEqual(m['acceleration_onset_missed'],1)


if __name__ == '__main__':
    unittest.main()
