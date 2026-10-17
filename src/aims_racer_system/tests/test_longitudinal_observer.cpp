#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "aims_racer_system/longitudinal_observer.hpp"
using namespace aims_racer_system;
TEST(LongitudinalObserver, RequiresDataAndChecksSnapshotTime) {
  ObserverConfig c;
  c.use_imu = true;
  LongitudinalObserver o(c);
  EXPECT_THROW(o.snapshot(0), std::runtime_error);
  ASSERT_TRUE(o.receive("speed", 100, 2.0));
  EXPECT_THROW(o.snapshot(99), std::invalid_argument);
  EXPECT_NEAR(o.snapshot(100).velocity, 2.0 / 1.0004, 1e-13);
  EXPECT_EQ(o.counters().at("admitted"), 1u);
}
TEST(LongitudinalObserver, DelayedReplayMatchesStableSourceOrderAtUnixEpoch) {
  ObserverConfig c;
  c.use_imu = true;
  LongitudinalObserver delayed(c), sorted(c);
  constexpr int64_t t = 1780000000000000000LL;
  ASSERT_TRUE(delayed.receive("speed", t, 1.0));
  ASSERT_TRUE(delayed.receive("force", t + 20000000, .7));
  ASSERT_TRUE(delayed.receive("speed", t + 10000000, 1.01));
  ASSERT_TRUE(delayed.receive("force", t + 20000000, .9));
  ASSERT_TRUE(sorted.receive("speed", t, 1.0));
  ASSERT_TRUE(sorted.receive("speed", t + 10000000, 1.01));
  ASSERT_TRUE(sorted.receive("force", t + 20000000, .7));
  ASSERT_TRUE(sorted.receive("force", t + 20000000, .9));
  const auto a = delayed.snapshot(t + 30000000), b = sorted.snapshot(t + 30000000);
  EXPECT_DOUBLE_EQ(a.velocity, b.velocity);
  EXPECT_DOUBLE_EQ(a.acceleration, b.acceleration);
  EXPECT_DOUBLE_EQ(a.disturbance.value(), b.disturbance.value());
  EXPECT_DOUBLE_EQ(a.acceleration_variance, b.acceleration_variance);
  EXPECT_EQ(delayed.counters().at("delayed"), 1u);
  EXPECT_EQ(delayed.counters().at("replayed"), 2u);
}
TEST(LongitudinalObserver, CountAnchorPreservesPosteriorAndRejectsOlderInput) {
  ObserverConfig c;
  c.use_imu = true;
  c.max_history_measurements = 2;
  LongitudinalObserver bounded(c);
  c.max_history_measurements = 100;
  LongitudinalObserver full(c);
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(bounded.receive(i % 2 ? "force" : "speed", 1000000000LL * i, .1 * i));
    ASSERT_TRUE(full.receive(i % 2 ? "force" : "speed", 1000000000LL * i, .1 * i));
  }
  EXPECT_EQ(bounded.history_size(), 2u);
  EXPECT_DOUBLE_EQ(bounded.snapshot(20000000000LL).acceleration,
                   full.snapshot(20000000000LL).acceleration);
  EXPECT_FALSE(bounded.receive("speed", 1, .2));
  EXPECT_EQ(bounded.counters().at("too_old"), 1u);
}
TEST(LongitudinalObserver, RejectsInvalidAndNumericFailureTransactionally) {
  ObserverConfig c;
  c.use_imu = true;
  LongitudinalObserver o(c);
  ASSERT_TRUE(o.receive("speed", 0, -1e308));
  const auto before = o.snapshot(100);
  EXPECT_FALSE(o.receive("speed", 50, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(o.receive("speed", -1, 1.));
  EXPECT_FALSE(o.receive("force", 50, 1., 0.));
  EXPECT_FALSE(o.receive("speed", 50, 1e308, 1e-300));
  EXPECT_DOUBLE_EQ(before.acceleration, o.snapshot(100).acceleration);
  EXPECT_EQ(o.history_size(), 1u);
  EXPECT_EQ(o.counters().at("numerical_failure"), 1u);
}
TEST(LongitudinalObserver, SnapshotsDoNotMutateHistoryAndRemainAvailableAfterGaps) {
  ObserverConfig c;
  c.use_imu = true;
  LongitudinalObserver o(c);
  ASSERT_TRUE(o.receive("speed", 0, 2.));
  ASSERT_TRUE(o.receive("force", 10000000, 1.));
  auto a = o.snapshot(10000000);
  auto gap = o.snapshot(100000000000LL);
  EXPECT_TRUE(std::isfinite(gap.acceleration_variance));
  EXPECT_GE(gap.acceleration_variance, 0.);
  EXPECT_EQ(o.history_size(), 2u);
  EXPECT_DOUBLE_EQ(a.acceleration, o.snapshot(10000000).acceleration);
}
TEST(LongitudinalObserver, ConfigurationValidationAndDisabledInput) {
  ObserverConfig c;
  c.history_seconds = 0.;
  EXPECT_THROW(LongitudinalObserver o(c), std::invalid_argument);
  c = ObserverConfig{};
  c.max_history_measurements = 1;
  EXPECT_THROW(LongitudinalObserver o(c), std::invalid_argument);
  c = ObserverConfig{};
  LongitudinalObserver o(c);
  EXPECT_FALSE(o.receive("force", 0, 1.));
  EXPECT_EQ(o.counters().at("disabled"), 1u);
}

TEST(LongitudinalObserver, NumericFailureInLateSuffixDoesNotCommitInsertedMeasurement) {
  ObserverConfig config;
  config.use_imu = true;
  LongitudinalObserver observer(config);
  ASSERT_TRUE(observer.receive("speed", 100, 1e308));
  const auto before = observer.snapshot(100);
  // The inserted speed update itself is finite; replaying the existing positive
  // observation then overflows its innovation. Neither posterior may commit.
  EXPECT_FALSE(observer.receive("speed", 50, -1e308));
  EXPECT_EQ(observer.history_size(), 1u);
  EXPECT_DOUBLE_EQ(before.velocity, observer.snapshot(100).velocity);
  EXPECT_DOUBLE_EQ(before.acceleration_variance, observer.snapshot(100).acceleration_variance);
  EXPECT_EQ(observer.counters().at("numerical_failure"), 1u);
  EXPECT_EQ(observer.counters().at("admitted"), 1u);
  EXPECT_EQ(observer.counters().at("delayed"), 0u);
  EXPECT_EQ(observer.counters().at("replayed"), 0u);
}

TEST(LongitudinalObserver, RepeatedTinyVarianceUpdatesRetainFiniteNonnegativeCovariance) {
  ObserverConfig config;
  config.use_imu = true;
  config.max_history_measurements = 8;
  LongitudinalObserver observer(config);
  constexpr int64_t epoch = 1800000000000000003LL;
  for (int64_t i = 0; i < 10000; ++i) {
    ASSERT_TRUE(observer.receive(i % 2 ? "force" : "speed", epoch + i * 1000, .1, 1e-15));
    const auto estimate = observer.snapshot(epoch + i * 1000);
    ASSERT_TRUE(std::isfinite(estimate.acceleration_variance));
    ASSERT_GE(estimate.acceleration_variance, 0.);
  }
  EXPECT_EQ(observer.history_size(), 8u);
  EXPECT_EQ(observer.counters().at("numerical_failure"), 0u);
}
