"""Synthetic contract tests: numerical consistency does not prove vehicle accuracy."""

from dataclasses import FrozenInstanceError
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    from observer import LongitudinalObserver, ObserverConfig
except ModuleNotFoundError as exc:
    if exc.name != "observer":
        raise
    LongitudinalObserver = ObserverConfig = None


class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(LongitudinalObserver, "observer implementation is not yet available")

    def test_configuration_is_immutable_and_rejects_invalid_models(self):
        cfg = ObserverConfig()
        self.assertEqual(cfg.speed_variance, 0.0004)
        with self.assertRaises(FrozenInstanceError):
            cfg.jerk_psd = 2.0
        for kw in ({"history_seconds": 0}, {"jerk_psd": -1},
                   {"speed_variance": 0}, {"force_variance": np.nan},
                   {"use_gravity": True}, {"jerk_psd": 10 ** 1000}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                ObserverConfig(**kw)

    def test_wheel_only_recovers_constant_acceleration_at_large_epoch(self):
        obs = LongitudinalObserver(ObserverConfig(jerk_psd=0.01))
        epoch = 1_800_000_000_000_000_000
        for i in range(201):
            self.assertTrue(obs.receive("speed", epoch + i * 20_000_000, 1 + 2 * i * 0.02))
        out = obs.snapshot(epoch + 4_000_000_000)
        self.assertAlmostEqual(out.velocity, 9.0, delta=0.002)
        self.assertAlmostEqual(out.acceleration, 2.0, delta=0.005)
        self.assertIsNone(out.disturbance)
        self.assertGreater(out.acceleration_variance, 0)

    def test_force_offset_is_separated_from_stationary_acceleration(self):
        obs = LongitudinalObserver(ObserverConfig(use_imu=True, jerk_psd=0.01,
                                                  disturbance_psd=0.001))
        for i in range(1001):
            stamp = i * 5_000_000
            self.assertTrue(obs.receive("force", stamp, 1.7))
            if i % 4 == 0:
                self.assertTrue(obs.receive("speed", stamp, 0.0))
        out = obs.snapshot(5_000_000_000)
        self.assertAlmostEqual(out.acceleration, 0.0, delta=0.015)
        self.assertAlmostEqual(out.disturbance, 1.7, delta=0.02)

    def test_gravity_updates_effective_disturbance(self):
        obs = LongitudinalObserver(ObserverConfig(use_imu=True, use_gravity=True,
                                                  jerk_psd=0.01, disturbance_psd=0.0))
        for i in range(101):
            stamp = i * 20_000_000
            obs.receive("speed", stamp, i * 0.02)
            obs.receive("force", stamp, 3.0, variance=0.001)
            obs.receive("gravity", stamp, 2.0, variance=0.001)
        out = obs.snapshot(2_000_000_000)
        self.assertAlmostEqual(out.acceleration, 1.0, delta=0.01)
        self.assertAlmostEqual(out.disturbance, 2.0, delta=0.01)

    def test_delayed_observations_match_source_order_after_same_inputs(self):
        cfg = ObserverConfig(use_imu=True, use_gravity=True, history_seconds=10)
        events = [(i * 10_000_000, "force", 0.6 + 0.1 * np.sin(i / 10)) for i in range(101)]
        events += [(i * 20_000_000, "speed", 0.4 * i * 0.02) for i in range(51)]
        events += [(i * 100_000_000, "gravity", 0.2) for i in range(11)]
        events.sort(key=lambda e: (e[0], e[1]))
        ordered, delayed = LongitudinalObserver(cfg), LongitudinalObserver(cfg)
        for stamp, kind, value in events:
            ordered.receive(kind, stamp, value)
        # Shuffle only between distinct stamps, preserving equal-stamp arrival order.
        groups = {}
        for e in events:
            groups.setdefault(e[0], []).append(e)
        stamps = list(groups)
        np.random.default_rng(10).shuffle(stamps)
        for stamp in stamps:
            for _, kind, value in groups[stamp]:
                delayed.receive(kind, stamp, value)
        a, b = ordered.snapshot(1_200_000_000), delayed.snapshot(1_200_000_000)
        np.testing.assert_allclose([a.velocity, a.acceleration, a.disturbance, a.acceleration_variance],
                                   [b.velocity, b.acceleration, b.disturbance, b.acceleration_variance],
                                   rtol=1e-11, atol=1e-11)
        self.assertGreater(delayed.counters["delayed"], 0)

    def test_previous_output_is_frozen_and_only_received_inputs_change_current_state(self):
        cfg = ObserverConfig(use_imu=True, use_gravity=True)
        obs, control = LongitudinalObserver(cfg), LongitudinalObserver(cfg)
        for o in (obs, control):
            o.receive("speed", 0, 0)
            o.receive("force", 100_000_000, 3)
        before = obs.snapshot(200_000_000)
        self.assertEqual(before, control.snapshot(200_000_000))
        old_values = (before.velocity, before.acceleration, before.disturbance)
        obs.receive("gravity", 50_000_000, 2, 0.001)
        self.assertEqual(old_values, (before.velocity, before.acceleration, before.disturbance))
        with self.assertRaises(FrozenInstanceError):
            before.acceleration = 100
        self.assertNotEqual(obs.snapshot(200_000_000), control.snapshot(200_000_000))

    def test_snapshot_prediction_does_not_change_measurement_history(self):
        obs, control = LongitudinalObserver(ObserverConfig()), LongitudinalObserver(ObserverConfig())
        for o in (obs, control):
            o.receive("speed", 0, 0)
            o.receive("speed", 100_000_000, 0.1)
        obs.snapshot(5_000_000_000)
        for o in (obs, control):
            o.receive("speed", 200_000_000, 0.2)
        self.assertEqual(obs.snapshot(300_000_000), control.snapshot(300_000_000))

    def test_variance_grows_during_measurement_gap(self):
        obs = LongitudinalObserver(ObserverConfig(jerk_psd=2))
        obs.receive("speed", 0, 1)
        initial = obs.snapshot(0)
        later = obs.snapshot(2_000_000_000)
        self.assertAlmostEqual(later.acceleration_variance - initial.acceleration_variance, 4)
        self.assertGreater(later.acceleration_variance, initial.acceleration_variance)

    def test_snapshot_cannot_use_a_time_before_newest_admitted_source(self):
        obs = LongitudinalObserver(ObserverConfig())
        with self.assertRaises(RuntimeError):
            obs.snapshot(0)
        obs.receive("speed", 1_000_000_000, 1)
        with self.assertRaises(ValueError):
            obs.snapshot(999_999_999)

    def test_invalid_observations_do_not_change_estimate(self):
        obs = LongitudinalObserver(ObserverConfig())
        obs.receive("speed", 100, 1)
        original = obs.snapshot(100)
        cases = [("speed", 100, np.nan, None), ("speed", 100, np.inf, None),
                 ("speed", 100, 1, 0), ("speed", 100, 1, -1),
                 ("speed", 100, 1, np.nan), ("speed", -1, 1, None),
                 ("speed", 100.0, 1, None), ("unknown", 100, 1, None),
                 ("speed", 100, 10 ** 1000, None), ("speed", True, 1, None)]
        for args in cases:
            self.assertFalse(obs.receive(*args))
            self.assertEqual(obs.snapshot(100), original)
        self.assertEqual(obs.counters["invalid"], len(cases))
        self.assertFalse(obs.receive("force", 100, 1))
        self.assertFalse(obs.receive("gravity", 100, 1))
        self.assertEqual(obs.counters["disabled"], 2)

    def test_old_history_rejection_does_not_stop_live_output(self):
        obs = LongitudinalObserver(ObserverConfig(history_seconds=0.2))
        for i in range(501):
            obs.receive("speed", i * 20_000_000, i * 0.02)
        self.assertFalse(obs.receive("speed", 0, 100))
        self.assertEqual(obs.counters["too_old"], 1)
        out = obs.snapshot(11_000_000_000)
        self.assertAlmostEqual(out.velocity, 11, delta=0.01)
        self.assertAlmostEqual(out.acceleration, 1, delta=0.01)
        self.assertLessEqual(obs.history_size, 11)

    def test_trimmed_anchor_replay_matches_full_history(self):
        short = LongitudinalObserver(ObserverConfig(history_seconds=0.5))
        full = LongitudinalObserver(ObserverConfig(history_seconds=20))
        for i in range(201):
            for o in (short, full):
                o.receive("speed", i * 20_000_000, i * 0.02 + 0.01 * np.sin(i))
        for o in (short, full):
            self.assertTrue(o.receive("speed", 3_710_000_000, 3.73))
        a, b = short.snapshot(4_100_000_000), full.snapshot(4_100_000_000)
        np.testing.assert_allclose([a.velocity, a.acceleration, a.acceleration_variance],
                                   [b.velocity, b.acceleration, b.acceleration_variance], atol=1e-12)

    def test_no_implicit_innovation_gate(self):
        obs = LongitudinalObserver(ObserverConfig())
        for i in range(20):
            obs.receive("speed", i * 20_000_000, 0)
        previous = obs.snapshot(380_000_000)
        self.assertTrue(obs.receive("speed", 400_000_000, 20))
        self.assertGreater(obs.snapshot(400_000_000).velocity, previous.velocity + 1)

    def test_joseph_updates_remain_finite_under_small_variance(self):
        obs = LongitudinalObserver(ObserverConfig(use_imu=True, use_gravity=True,
                                                  jerk_psd=0, disturbance_psd=0))
        for i in range(300):
            stamp = i * 5_000_000
            for kind, value in (("speed", 0), ("force", 2), ("gravity", 2)):
                self.assertTrue(obs.receive(kind, stamp, value, 1e-12))
        out = obs.snapshot(2_000_000_000)
        self.assertTrue(np.isfinite([out.velocity, out.acceleration, out.disturbance,
                                     out.acceleration_variance]).all())
        self.assertGreaterEqual(out.acceleration_variance, 0)

    def test_history_count_configuration_requires_an_integer_at_least_two(self):
        self.assertTrue(hasattr(ObserverConfig(), "max_history_measurements"),
                        "hard history count cap is not yet implemented")
        self.assertEqual(ObserverConfig().max_history_measurements, 4096)
        for cap in (0, 1, -1, 2.0, True, np.nan, "2"):
            with self.subTest(cap=cap), self.assertRaises(ValueError):
                ObserverConfig(max_history_measurements=cap)
        self.assertEqual(ObserverConfig(max_history_measurements=np.int64(2)).max_history_measurements, 2)

    def test_repeated_source_stamp_history_is_bounded_without_losing_latest_state(self):
        self.assertTrue(hasattr(ObserverConfig(), "max_history_measurements"),
                        "hard history count cap is not yet implemented")
        limited = LongitudinalObserver(ObserverConfig(history_seconds=0.001,
                                                       max_history_measurements=32))
        reference = LongitudinalObserver(ObserverConfig(history_seconds=0.001))
        for i in range(2000):
            value = 1 + 0.01 * np.sin(i)
            self.assertTrue(limited.receive("speed", 100, value))
            reference.receive("speed", 100, value)
            self.assertLessEqual(limited.history_size, 32)
        self.assertEqual(limited.history_size, 32)
        self.assertEqual(limited.counters["count_trimmed"], 1968)
        self.assertEqual(limited.counters["max_history"], 32)
        self.assertEqual(limited.snapshot(100), reference.snapshot(100))
        for o in (limited, reference):
            self.assertTrue(o.receive("speed", 1_000_000_100, 2))
        self.assertEqual(limited.snapshot(2_000_000_100), reference.snapshot(2_000_000_100))

    def test_count_trimmed_anchor_supports_late_replay_and_rejects_earlier_sources(self):
        self.assertTrue(hasattr(ObserverConfig(), "max_history_measurements"),
                        "hard history count cap is not yet implemented")
        limited = LongitudinalObserver(ObserverConfig(history_seconds=2, max_history_measurements=2))
        reference = LongitudinalObserver(ObserverConfig(history_seconds=2))
        for stamp, value in ((0, 0), (100_000_000, 0.1),
                             (200_000_000, 0.2), (300_000_000, 0.3)):
            for o in (limited, reference):
                self.assertTrue(o.receive("speed", stamp, value))
        before = limited.snapshot(400_000_000)
        self.assertFalse(limited.receive("speed", 50_000_000, 10))
        self.assertEqual(limited.snapshot(400_000_000), before)
        self.assertEqual(limited.counters["too_old"], 1)
        # Equal to the anchor stamp is valid: receipt order places it after the
        # measurements represented by that checkpoint.
        for o in (limited, reference):
            self.assertTrue(o.receive("speed", 100_000_000, 0.15))
        self.assertEqual(limited.snapshot(400_000_000), reference.snapshot(400_000_000))
        self.assertEqual(limited.counters["count_trimmed"], 3)
        self.assertLessEqual(limited.history_size, 2)

    def test_stamps_outside_signed_int64_are_rejected_without_poisoning_state(self):
        obs = LongitudinalObserver(ObserverConfig())
        for stamp in (10 ** 1000, 2 ** 63):
            self.assertFalse(obs.receive("speed", stamp, 1))
        self.assertEqual(obs.history_size, 0)
        self.assertTrue(obs.receive("speed", 100, 1))
        before = obs.snapshot(100)
        self.assertFalse(obs.receive("speed", 10 ** 1000, 100))
        self.assertEqual(obs.snapshot(100), before)
        self.assertTrue(obs.receive("speed", 200, 1))
        for stamp in (10 ** 1000, 2 ** 63):
            with self.assertRaises(ValueError):
                obs.snapshot(stamp)
        boundary = LongitudinalObserver(ObserverConfig())
        self.assertTrue(boundary.receive("speed", np.int64(2 ** 63 - 1), 1))
        self.assertEqual(boundary.snapshot(2 ** 63 - 1).stamp_ns, 2 ** 63 - 1)


if __name__ == "__main__":
    unittest.main()
