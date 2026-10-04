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

#ifndef ACKERMANN_MUX__JOYSTICK_SELECTOR_HPP_
#define ACKERMANN_MUX__JOYSTICK_SELECTOR_HPP_

#include <array>
#include <chrono>
#include <string>

namespace ackermann_mux
{
using SteadyTime = std::chrono::steady_clock::time_point;
using Channels = std::array<int, 16>;

struct JoystickConfig
{
  int speed_channel{1}, steering_channel{2}, lock_channel{3}, esc_mode_channel{4};
  int control_source_channel{5}, limit_channel{6}, calib_mode_channel{7};
  int channel_min{172}, channel_mid{992}, channel_max{1810}, deadzone{100};
  int limit_min{172}, limit_max{1810}, switch_mid{992};
  double switch_low{582.0}, switch_high{1401.0};
  double speed_min{2.0}, speed_max{12.0}, current_min{3.0}, current_max{20.0};
  double steering_limit{0.4751};
  // External commands have separate bounds; the RC knob only limits manual driving.
  double command_speed_limit{12.0}, command_current_limit{100.0}, command_duty_limit{0.8};
  double rc_timeout{0.2}, nav_timeout{0.2}, calib_timeout{0.2};
  bool direction_reverse{false}, steering_reverse{true};

  static JoystickConfig profile(const std::string & name);
  void validate() const;
};

struct DriveCommand
{
  double steering{0.0}, speed{0.0}, acceleration{0.0}, jerk{0.0};
};

enum class SelectionState
{
  WaitingRC, InvalidRC, RCExpired, Locked, ManualSpeed, ManualCurrent, ManualDuty,
  Navigation, Calibration, WaitingNavigation, NavigationExpired, InvalidNavigation,
  WaitingCalibration, CalibrationExpired, InvalidCalibration, UnsupportedNavigationMode
};
const char * state_name(SelectionState state);

struct Selection
{
  DriveCommand command{};  // Every branch returns an explicit command, including stops.
  bool autonomy_speed_enabled{false};
  SelectionState state{SelectionState::WaitingRC};
};

class JoystickSelector
{
public:
  explicit JoystickSelector(const JoystickConfig & config);
  void receive_rc(const Channels & channels, SteadyTime now);
  // source_age is measured once using the producer's ROS timestamp. All subsequent
  // ageing uses steady time, so pausing/resetting ROS time cannot renew a command.
  void receive_navigation(const DriveCommand & command, SteadyTime now, double source_age = 0.0);
  void receive_calibration(const DriveCommand & command, SteadyTime now, double source_age);
  Selection select(SteadyTime now);

private:
  struct CachedCommand
  {
    DriveCommand command{};
    SteadyTime received{};
    double source_age{0.0};
    bool present{false}, valid{false};
  };
  static bool fresh(SteadyTime now, SteadyTime received, double timeout);
  bool fresh_command(const CachedCommand & cached, SteadyTime now, double timeout) const;
  bool valid_command(const DriveCommand & command, double source_age) const;
  CachedCommand cache(const DriveCommand & command, SteadyTime now, double source_age) const;
  int channel(int index) const {return channels_[static_cast<size_t>(index - 1)];}
  bool locked() const {return channel(config_.lock_channel) < config_.switch_mid;}
  bool calibrating() const {return channel(config_.calib_mode_channel) > config_.switch_mid;}
  bool navigating() const {return channel(config_.control_source_channel) >= config_.switch_mid;}
  int esc_mode() const;
  double normalized(int raw) const;
  double manual_throttle(double minimum, double maximum) const;
  DriveCommand sanitized(const DriveCommand & command) const;

  JoystickConfig config_;
  Channels channels_{};
  SteadyTime rc_received_{};
  SteadyTime calibration_armed_at_{};
  bool rc_present_{false}, rc_valid_{false};
  CachedCommand navigation_{}, calibration_{};
};
}  // namespace ackermann_mux
#endif  // ACKERMANN_MUX__JOYSTICK_SELECTOR_HPP_
