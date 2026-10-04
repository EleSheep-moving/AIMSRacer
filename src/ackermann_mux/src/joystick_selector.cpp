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

#include "ackermann_mux/joystick_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ackermann_mux
{
JoystickConfig JoystickConfig::profile(const std::string & name)
{
  JoystickConfig config;
  if (name == "steering_ch1_throttle_ch3_aux_ch5_to_ch10") {
    config.speed_channel = 3;
    config.steering_channel = 1;
    config.lock_channel = 5;
    config.esc_mode_channel = 6;
    config.control_source_channel = 7;
    config.calib_mode_channel = 8;
    config.limit_channel = 10;
  } else if (name != "sequential_ch1_ch2") {
    throw std::invalid_argument("Unknown channel_profile: " + name);
  }
  return config;
}

void JoystickConfig::validate() const
{
  const std::array<int, 7> indices{{speed_channel, steering_channel, lock_channel,
    esc_mode_channel, control_source_channel, limit_channel, calib_mode_channel}};
  auto sorted = indices;
  std::sort(sorted.begin(), sorted.end());
  if (sorted.front() < 1 || sorted.back() > 16 ||
    std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
  {
    throw std::invalid_argument("Channel mappings must be distinct and within [1, 16]");
  }
  if (!(0 < channel_min && channel_min < channel_mid && channel_mid < channel_max &&
    channel_max <= 2047 && limit_min >= 0 && limit_min < limit_max && limit_max <= 2047 &&
    deadzone >= 0 && deadzone < std::min(channel_mid - channel_min, channel_max - channel_mid)))
  {
    throw std::invalid_argument("Invalid RC ranges, midpoint, limit range or deadzone");
  }
  if (!(std::isfinite(switch_low) && std::isfinite(switch_high) &&
    channel_min < switch_low && switch_low < switch_mid && switch_mid < switch_high &&
    switch_high < channel_max))
  {
    throw std::invalid_argument("Invalid switch thresholds");
  }
  const std::array<double, 10> positive{{steering_limit, command_speed_limit,
    command_current_limit, command_duty_limit, rc_timeout, nav_timeout, calib_timeout,
    speed_max, current_max, switch_high}};
  for (const double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Limits and timeouts must be finite and positive");
    }
  }
  if (!(std::isfinite(speed_min) && 0.0 <= speed_min && speed_min <= speed_max &&
    std::isfinite(current_min) && 0.0 <= current_min && current_min <= current_max &&
    command_duty_limit <= 1.0 && speed_max <= command_speed_limit &&
    current_max <= command_current_limit))
  {
    throw std::invalid_argument("Invalid speed/current/duty bounds");
  }
}

JoystickSelector::JoystickSelector(const JoystickConfig & config)
: config_(config)
{
  config_.validate();
}

bool JoystickSelector::fresh(SteadyTime now, SteadyTime received, double timeout)
{
  const double age = std::chrono::duration<double>(now - received).count();
  return age >= 0.0 && age <= timeout;
}

void JoystickSelector::receive_rc(const Channels & channels, SteadyTime now)
{
  const bool previously_armed = rc_present_ && rc_valid_ &&
    fresh(now, rc_received_, config_.rc_timeout) && !locked() && calibrating();
  channels_ = channels;
  rc_received_ = now;
  rc_present_ = true;
  rc_valid_ = true;
  // CRSF channels are 11-bit values. Accept transmitter endpoint variation,
  // but reject corrupt values on every channel actually used by this profile.
  for (const int index : {config_.speed_channel, config_.steering_channel, config_.lock_channel,
      config_.esc_mode_channel, config_.control_source_channel, config_.limit_channel,
      config_.calib_mode_channel})
  {
    rc_valid_ = rc_valid_ && channel(index) > 0 && channel(index) <= 2047;
  }
  const bool armed = rc_valid_ && !locked() && calibrating();
  if (!armed || !previously_armed) {
    calibration_ = CachedCommand{};  // Re-arm requires a command received after this RC frame.
    if (armed) {
      calibration_armed_at_ = now;
    }
  }
}

bool JoystickSelector::valid_command(const DriveCommand & c, double source_age) const
{
  if (!std::isfinite(c.steering) || !std::isfinite(c.speed) ||
    !std::isfinite(c.acceleration) || !std::isfinite(c.jerk) ||
    !std::isfinite(source_age) || source_age < 0.0)
  {
    return false;
  }
  if (c.jerk == 0.0) {
    return std::abs(c.speed) <= config_.command_speed_limit;
  }
  if (c.jerk == 2.0) {
    return std::abs(c.acceleration) <= config_.command_current_limit;
  }
  if (c.jerk == 3.0) {
    return std::abs(c.acceleration) <= config_.command_duty_limit;
  }
  // jerk=1 is not implemented by this vehicle's ackermann_to_vesc converter.
  return false;
}

JoystickSelector::CachedCommand JoystickSelector::cache(
  const DriveCommand & command, SteadyTime now, double source_age) const
{
  return CachedCommand{command, now, source_age, true, valid_command(command, source_age)};
}

void JoystickSelector::receive_navigation(
  const DriveCommand & command, SteadyTime now, double source_age)
{
  navigation_ = cache(command, now, source_age);
}

void JoystickSelector::receive_calibration(
  const DriveCommand & command, SteadyTime now, double source_age)
{
  if (rc_present_ && rc_valid_ && fresh(now, rc_received_, config_.rc_timeout) &&
    !locked() && calibrating())
  {
    calibration_ = cache(command, now, source_age);
    // A delayed/replayed command generated before arming must not resume actuation.
    calibration_.valid = calibration_.valid && source_age <=
      std::chrono::duration<double>(now - calibration_armed_at_).count();
  }
}

bool JoystickSelector::fresh_command(
  const CachedCommand & cached, SteadyTime now, double timeout) const
{
  return fresh(now, cached.received, timeout) &&
         cached.source_age + std::chrono::duration<double>(now - cached.received).count() <=
         timeout;
}

int JoystickSelector::esc_mode() const
{
  const int value = channel(config_.esc_mode_channel);
  return value < config_.switch_low ? 0 : (value < config_.switch_high ? 2 : 3);
}

double JoystickSelector::normalized(int raw) const
{
  const double range = raw > config_.channel_mid ?
    config_.channel_max - config_.channel_mid : config_.channel_mid - config_.channel_min;
  return std::clamp((raw - config_.channel_mid) / range, -1.0, 1.0);
}

double JoystickSelector::manual_throttle(double minimum, double maximum) const
{
  const int raw = channel(config_.speed_channel);
  if (std::abs(raw - config_.channel_mid) <= config_.deadzone) {
    return 0.0;
  }
  const int limit = std::clamp(
    channel(config_.limit_channel), config_.limit_min, config_.limit_max);
  const double ratio = static_cast<double>(limit - config_.limit_min) /
    (config_.limit_max - config_.limit_min);
  return normalized(raw) * (minimum + ratio * (maximum - minimum)) *
         (config_.direction_reverse ? -1.0 : 1.0);
}

DriveCommand JoystickSelector::sanitized(const DriveCommand & command) const
{
  DriveCommand result = command;
  result.steering = std::clamp(
    result.steering, -config_.steering_limit, config_.steering_limit);
  if (result.jerk == 0.0) {
    result.acceleration = 0.0;
  } else {
    result.speed = 0.0;
  }
  return result;
}

Selection JoystickSelector::select(SteadyTime now)
{
  Selection output;
  if (!rc_present_) {return output;}
  if (!rc_valid_) {
    output.state = SelectionState::InvalidRC;
    return output;
  }
  if (!fresh(now, rc_received_, config_.rc_timeout)) {
    calibration_ = CachedCommand{};
    output.state = SelectionState::RCExpired;
    return output;
  }
  if (locked()) {
    output.state = SelectionState::Locked;
    return output;
  }
  // This flag denotes authority selection, independent of producer availability.
  output.autonomy_speed_enabled = navigating() && esc_mode() == 0 && !calibrating();
  if (calibrating()) {
    if (!calibration_.present) {
      output.state = SelectionState::WaitingCalibration;
    } else if (!calibration_.valid) {
      output.state = SelectionState::InvalidCalibration;
    } else if (!fresh_command(calibration_, now, config_.calib_timeout)) {
      output.state = SelectionState::CalibrationExpired;
    } else {
      output.state = SelectionState::Calibration;
      output.command = sanitized(calibration_.command);
    }
    return output;
  }
  if (navigating()) {
    if (esc_mode() != 0) {
      output.state = SelectionState::UnsupportedNavigationMode;
    } else if (!navigation_.present) {
      output.state = SelectionState::WaitingNavigation;
    } else if (!navigation_.valid || navigation_.command.jerk != 0.0) {
      output.state = SelectionState::InvalidNavigation;
    } else if (!fresh_command(navigation_, now, config_.nav_timeout)) {
      output.state = SelectionState::NavigationExpired;
    } else {
      output.state = SelectionState::Navigation;
      output.command = sanitized(navigation_.command);
    }
    return output;
  }
  output.command.steering = normalized(channel(config_.steering_channel)) *
    config_.steering_limit * (config_.steering_reverse ? -1.0 : 1.0);
  output.command.jerk = esc_mode();
  if (esc_mode() == 0) {
    output.state = SelectionState::ManualSpeed;
    output.command.speed = manual_throttle(config_.speed_min, config_.speed_max);
  } else if (esc_mode() == 2) {
    output.state = SelectionState::ManualCurrent;
    output.command.acceleration = manual_throttle(config_.current_min, config_.current_max);
  } else {
    output.state = SelectionState::ManualDuty;
    output.command.acceleration = manual_throttle(0.0, config_.command_duty_limit);
  }
  return output;
}

const char * state_name(SelectionState state)
{
  switch (state) {
    case SelectionState::WaitingRC: return "waiting for RC; stopped";
    case SelectionState::InvalidRC: return "invalid RC; stopped";
    case SelectionState::RCExpired: return "RC timeout; stopped";
    case SelectionState::Locked: return "locked; stopped";
    case SelectionState::ManualSpeed: return "manual speed";
    case SelectionState::ManualCurrent: return "manual current";
    case SelectionState::ManualDuty: return "manual duty";
    case SelectionState::Navigation: return "navigation speed";
    case SelectionState::Calibration: return "calibration";
    case SelectionState::WaitingNavigation: return "waiting for navigation; stopped";
    case SelectionState::NavigationExpired: return "navigation timeout; stopped";
    case SelectionState::InvalidNavigation: return "invalid navigation command; stopped";
    case SelectionState::WaitingCalibration: return "waiting for new calibration command; stopped";
    case SelectionState::CalibrationExpired: return "calibration timeout; stopped";
    case SelectionState::InvalidCalibration: return "invalid calibration command; stopped";
    case SelectionState::UnsupportedNavigationMode:
      return "unsupported navigation ESC mode; stopped";
  }
  return "unknown; stopped";
}
}  // namespace ackermann_mux
