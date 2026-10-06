// Copyright 2026 AIMSRacer.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "ackermann_mux/joystick_selector.hpp"
#include "gtest/gtest.h"

namespace ackermann_mux
{
namespace
{
SteadyTime at(double seconds)
{
  return SteadyTime{} + std::chrono::duration_cast<SteadyTime::duration>(
    std::chrono::duration<double>(seconds));
}

Channels manual(const JoystickConfig & c)
{
  Channels channels;
  channels.fill(992);
  channels[c.lock_channel - 1] = 1810;
  channels[c.control_source_channel - 1] = 172;
  channels[c.esc_mode_channel - 1] = 172;
  channels[c.calib_mode_channel - 1] = 172;
  channels[c.limit_channel - 1] = 1810;
  return channels;
}

void expect_stop(const Selection & output)
{
  EXPECT_DOUBLE_EQ(output.command.speed, 0.0);
  EXPECT_DOUBLE_EQ(output.command.steering, 0.0);
  EXPECT_DOUBLE_EQ(output.command.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(output.command.jerk, 0.0);
}

TEST(JoystickSelector, RejectsInvalidConfiguration)
{
  auto c = JoystickConfig{};
  EXPECT_THROW(JoystickConfig::profile("typo"), std::invalid_argument);
  c.speed_channel = 17;
  EXPECT_THROW(JoystickSelector{c}, std::invalid_argument);
  c = JoystickConfig{};
  c.speed_channel = c.lock_channel;
  EXPECT_THROW(JoystickSelector{c}, std::invalid_argument);
  c = JoystickConfig{};
  c.channel_mid = c.channel_min;
  EXPECT_THROW(JoystickSelector{c}, std::invalid_argument);
  c = JoystickConfig{};
  c.limit_max = c.limit_min;
  EXPECT_THROW(JoystickSelector{c}, std::invalid_argument);
  c = JoystickConfig{};
  c.calib_timeout = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(JoystickSelector{c}, std::invalid_argument);
  c = JoystickConfig{};
  c.command_duty_limit = 1.1;
  EXPECT_THROW(JoystickSelector{c}, std::invalid_argument);
}

TEST(JoystickSelector, BothProfilesPreserveManualModesAndLimits)
{
  for (const auto * profile : {"sequential_ch1_ch2", "steering_ch1_throttle_ch3_aux_ch5_to_ch10"}) {
    auto c = JoystickConfig::profile(profile);
    JoystickSelector selector(c);
    auto rc = manual(c);
    rc[c.speed_channel - 1] = 1810;
    rc[c.steering_channel - 1] = 172;
    selector.receive_rc(rc, at(1.0));
    auto out = selector.select(at(1.0));
    EXPECT_DOUBLE_EQ(out.command.speed, 12.0);
    EXPECT_DOUBLE_EQ(out.command.steering, 0.4751);
    EXPECT_FALSE(out.autonomy_speed_enabled);
    rc[c.limit_channel - 1] = 172;
    selector.receive_rc(rc, at(1.1));
    EXPECT_DOUBLE_EQ(selector.select(at(1.1)).command.speed, 2.0);
    rc[c.esc_mode_channel - 1] = 992;
    selector.receive_rc(rc, at(1.2));
    out = selector.select(at(1.2));
    EXPECT_DOUBLE_EQ(out.command.acceleration, 3.0);
    EXPECT_DOUBLE_EQ(out.command.jerk, 2.0);
    rc[c.limit_channel - 1] = 1810;
    selector.receive_rc(rc, at(1.3));
    EXPECT_DOUBLE_EQ(selector.select(at(1.3)).command.acceleration, 20.0);
    rc[c.esc_mode_channel - 1] = 1810;
    selector.receive_rc(rc, at(1.4));
    out = selector.select(at(1.4));
    EXPECT_DOUBLE_EQ(out.command.acceleration, 0.8);
    EXPECT_DOUBLE_EQ(out.command.jerk, 3.0);
    rc[c.limit_channel - 1] = 172;
    selector.receive_rc(rc, at(1.5));
    EXPECT_DOUBLE_EQ(selector.select(at(1.5)).command.acceleration, 0.0);
  }
}

TEST(JoystickSelector, MatchesRecordedPythonManualOutputs)
{
  std::ifstream input(JOYSTICK_REFERENCE_PATH);
  ASSERT_TRUE(input.good());
  std::string line;
  std::getline(input, line);
  size_t count = 0;
  while (std::getline(input, line)) {
    std::stringstream row(line);
    std::string field;
    std::getline(row, field, ',');
    const auto c = JoystickConfig::profile(field);
    Channels rc{};
    for (auto & raw : rc) {
      std::getline(row, field, ',');
      raw = std::stoi(field);
    }
    JoystickSelector selector(c);
    selector.receive_rc(rc, at(1.0));
    const auto command = selector.select(at(1.0)).command;
    for (const double actual :
      {command.steering, command.speed, command.acceleration, command.jerk})
    {
      std::getline(row, field, ',');
      EXPECT_NEAR(actual, std::stod(field), 1e-6) << "reference row " << count;
    }
    ++count;
  }
  EXPECT_EQ(count, 120u);
}

TEST(JoystickSelector, DeadzoneReversalAndSteeringClamp)
{
  auto c = JoystickConfig{};
  c.direction_reverse = true;
  c.steering_reverse = false;
  JoystickSelector selector(c);
  auto rc = manual(c);
  rc[c.speed_channel - 1] = c.channel_mid + c.deadzone;
  selector.receive_rc(rc, at(1.0));
  EXPECT_DOUBLE_EQ(selector.select(at(1.0)).command.speed, 0.0);
  rc[c.speed_channel - 1] = 172;
  rc[c.steering_channel - 1] = 2047;
  selector.receive_rc(rc, at(1.1));
  EXPECT_DOUBLE_EQ(selector.select(at(1.1)).command.speed, 12.0);
  EXPECT_DOUBLE_EQ(selector.select(at(1.1)).command.steering, c.steering_limit);
}

TEST(JoystickSelector, NoRCInvalidRCLockAndTimeoutAlwaysStop)
{
  auto c = JoystickConfig{};
  JoystickSelector selector(c);
  expect_stop(selector.select(at(1.0)));
  auto rc = manual(c);
  rc[c.speed_channel - 1] = -1;
  selector.receive_rc(rc, at(1.0));
  EXPECT_EQ(selector.select(at(1.0)).state, SelectionState::InvalidRC);
  expect_stop(selector.select(at(1.0)));
  rc = manual(c);
  rc[c.lock_channel - 1] = 172;
  selector.receive_rc(rc, at(1.0));
  EXPECT_EQ(selector.select(at(1.0)).state, SelectionState::Locked);
  expect_stop(selector.select(at(1.0)));
  rc[c.lock_channel - 1] = 1810;
  rc[c.speed_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.0));
  EXPECT_DOUBLE_EQ(selector.select(at(1.2)).command.speed, 12.0);
  expect_stop(selector.select(at(1.200001)));
  expect_stop(selector.select(at(0.9)));  // Defensive handling of invalid test/caller time.
  expect_stop(selector.select(at(10.0)));
  selector.receive_rc(rc, at(10.0));
  EXPECT_DOUBLE_EQ(selector.select(at(10.0)).command.speed, 12.0);
}

TEST(JoystickSelector, NavigationSelectionHasNoProducerEnableCycle)
{
  auto c = JoystickConfig{};
  JoystickSelector selector(c);
  auto rc = manual(c);
  rc[c.control_source_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.0));
  auto out = selector.select(at(1.0));
  EXPECT_TRUE(out.autonomy_speed_enabled);
  expect_stop(out);
  selector.receive_navigation({0.1, 2.0, 0.0, 0.0}, at(1.0));
  EXPECT_DOUBLE_EQ(selector.select(at(1.1)).command.speed, 2.0);
  selector.receive_rc(rc, at(1.2));
  out = selector.select(at(1.21));
  EXPECT_TRUE(out.autonomy_speed_enabled);
  EXPECT_EQ(out.state, SelectionState::NavigationExpired);
  expect_stop(out);
  for (const int mode : {992, 1810}) {
    rc[c.esc_mode_channel - 1] = mode;
    selector.receive_rc(rc, at(1.22));
    out = selector.select(at(1.22));
    EXPECT_FALSE(out.autonomy_speed_enabled);
    EXPECT_EQ(out.state, SelectionState::UnsupportedNavigationMode);
    expect_stop(out);
  }
}

TEST(JoystickSelector, CalibrationRequiresNewCommandAfterArming)
{
  auto c = JoystickConfig{};
  JoystickSelector selector(c);
  auto rc = manual(c);
  selector.receive_rc(rc, at(1.0));
  selector.receive_calibration({0.1, 0.0, 8.0, 2.0}, at(1.01), 0.0);
  rc[c.calib_mode_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.02));
  expect_stop(selector.select(at(1.02)));
  selector.receive_calibration({0.1, 0.0, 8.0, 2.0}, at(1.025), 0.01);
  expect_stop(selector.select(at(1.025)));  // Stamped before arming, delivered afterwards.
  selector.receive_calibration({0.1, 0.0, 8.0, 2.0}, at(1.03), 0.0);
  EXPECT_DOUBLE_EQ(selector.select(at(1.04)).command.acceleration, 8.0);
  rc[c.lock_channel - 1] = 172;
  selector.receive_rc(rc, at(1.05));
  expect_stop(selector.select(at(1.05)));
  selector.receive_calibration({0.1, 0.0, 8.0, 2.0}, at(1.06), 0.0);
  rc[c.lock_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.07));
  expect_stop(selector.select(at(1.07)));
  selector.receive_calibration({0.1, 0.0, 8.0, 2.0}, at(1.08), 0.0);
  EXPECT_DOUBLE_EQ(selector.select(at(1.08)).command.acceleration, 8.0);
  rc[c.calib_mode_channel - 1] = 172;
  selector.receive_rc(rc, at(1.09));
  rc[c.calib_mode_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.1));
  expect_stop(selector.select(at(1.1)));
}

TEST(JoystickSelector, CalibrationTimeoutCannotRenewItself)
{
  auto c = JoystickConfig{};
  JoystickSelector selector(c);
  auto rc = manual(c);
  rc[c.calib_mode_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.0));
  selector.receive_calibration({0.1, 0.0, 8.0, 2.0}, at(1.0), 0.0);
  for (int tick = 1; tick <= 19; ++tick) {
    selector.receive_rc(rc, at(1.0 + tick * 0.01));
    EXPECT_DOUBLE_EQ(selector.select(at(1.0 + tick * 0.01)).command.acceleration, 8.0);
  }
  selector.receive_rc(rc, at(1.21));
  EXPECT_EQ(selector.select(at(1.21)).state, SelectionState::CalibrationExpired);
  expect_stop(selector.select(at(1.21)));
  // Replaying an old stamped message cannot extend its lifetime.
  selector.receive_calibration({0.1, 0.0, 8.0, 2.0}, at(1.22), 0.22);
  expect_stop(selector.select(at(1.22)));
  selector.receive_calibration({0.1, 0.0, 8.0, 2.0}, at(1.23), 0.15);
  EXPECT_DOUBLE_EQ(selector.select(at(1.24)).command.acceleration, 8.0);
  selector.receive_rc(rc, at(1.3));
  expect_stop(selector.select(at(1.3)));
}

TEST(JoystickSelector, CalibrationRecoveryAfterRCLossNeedsNewCommand)
{
  auto c = JoystickConfig{};
  c.calib_timeout = 2.0;
  auto rc = manual(c);
  rc[c.calib_mode_channel - 1] = 1810;
  JoystickSelector selector(c);
  selector.receive_rc(rc, at(1.0));
  selector.receive_calibration({0.0, 0.0, 8.0, 2.0}, at(1.0), 0.0);
  expect_stop(selector.select(at(1.21)));
  selector.receive_rc(rc, at(1.22));
  expect_stop(selector.select(at(1.22)));
  selector.receive_calibration({0.0, 0.0, 8.0, 2.0}, at(1.23), 0.0);
  EXPECT_DOUBLE_EQ(selector.select(at(1.23)).command.acceleration, 8.0);
  // Recovery must also work if no timer happened to observe the outage.
  selector.receive_rc(rc, at(1.5));
  expect_stop(selector.select(at(1.5)));
}

TEST(JoystickSelector, CalibrationUsesProducerModeAndIndependentCurrentBound)
{
  auto c = JoystickConfig{};
  JoystickSelector selector(c);
  auto rc = manual(c);
  rc[c.calib_mode_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.0));
  for (const auto & command : {DriveCommand{0.2, 3.0, 0.0, 0.0},
      DriveCommand{0.2, 0.0, 80.0, 2.0}, DriveCommand{0.2, 0.0, 0.3, 3.0}})
  {
    selector.receive_calibration(command, at(1.01), 0.0);
    const auto out = selector.select(at(1.01));
    EXPECT_EQ(out.state, SelectionState::Calibration);
    EXPECT_DOUBLE_EQ(out.command.jerk, command.jerk);
    EXPECT_DOUBLE_EQ(out.command.speed, command.speed);
    EXPECT_DOUBLE_EQ(out.command.acceleration, command.acceleration);
    EXPECT_FALSE(out.autonomy_speed_enabled);
  }
}

TEST(JoystickSelector, InvalidCommandsReplacePreviouslyValidCommandsWithStops)
{
  auto c = JoystickConfig{};
  JoystickSelector selector(c);
  auto rc = manual(c);
  rc[c.calib_mode_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.0));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto & invalid : {DriveCommand{nan, 1.0, 0.0, 0.0},
      DriveCommand{0.0, nan, 0.0, 0.0}, DriveCommand{0.0, 1.0, nan, 0.0},
      DriveCommand{0.0, 1.0, 0.0, nan}, DriveCommand{0.0, 1.0, 0.0, 1.0},
      DriveCommand{0.0, 1.0, 0.0, 2.5}, DriveCommand{0.0, 13.0, 0.0, 0.0},
      DriveCommand{0.0, 0.0, 101.0, 2.0}, DriveCommand{0.0, 0.0, 0.81, 3.0}})
  {
    selector.receive_calibration({0.0, 1.0, 0.0, 0.0}, at(1.0), 0.0);
    selector.receive_calibration(invalid, at(1.01), 0.0);
    EXPECT_EQ(selector.select(at(1.01)).state, SelectionState::InvalidCalibration);
    expect_stop(selector.select(at(1.01)));
  }
  for (double age : {nan, -0.1, 0.21}) {
    selector.receive_calibration({0.0, 1.0, 0.0, 0.0}, at(1.01), age);
    expect_stop(selector.select(at(1.01)));
  }
  rc[c.calib_mode_channel - 1] = 172;
  rc[c.control_source_channel - 1] = 1810;
  selector.receive_rc(rc, at(1.02));
  selector.receive_navigation({0.0, 0.0, 8.0, 2.0}, at(1.02));
  expect_stop(selector.select(at(1.02)));
  selector.receive_navigation({nan, 1.0, 0.0, 0.0}, at(1.02));
  expect_stop(selector.select(at(1.02)));
}

TEST(JoystickSelector, SourceAgeAndReceiveAgeBothLimitNavigationLifetime)
{
  auto c = JoystickConfig{};
  auto rc = manual(c);
  rc[c.control_source_channel - 1] = 1810;
  JoystickSelector selector(c);
  selector.receive_rc(rc, at(1.0));
  selector.receive_navigation({1.0, 2.0, 0.0, 0.0}, at(1.0), 0.15);
  EXPECT_DOUBLE_EQ(selector.select(at(1.01)).command.steering, c.steering_limit);
  expect_stop(selector.select(at(1.06)));
}
}  // namespace
}  // namespace ackermann_mux
