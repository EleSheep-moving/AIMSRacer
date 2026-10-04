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

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "crsf_receiver_msg/msg/crsf_channels16.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace ackermann_mux
{
class JoystickControl : public rclcpp::Node
{
public:
  JoystickControl()
  : Node("joystick_control")
  {
    const auto profile = parameter<std::string>("channel_profile", "sequential_ch1_ch2");
    auto c = JoystickConfig::profile(profile);
    c.speed_channel = parameter("speed_channel", c.speed_channel);
    c.steering_channel = parameter("steering_channel", c.steering_channel);
    c.lock_channel = parameter("lock_channel", c.lock_channel);
    c.esc_mode_channel = parameter("esc_mode_channel", c.esc_mode_channel);
    c.control_source_channel = parameter("control_source_channel", c.control_source_channel);
    c.limit_channel = parameter("limit_channel", c.limit_channel);
    c.calib_mode_channel = parameter("calib_mode_channel", c.calib_mode_channel);
    // Canonical parameters override legacy aliases, matching the Python implementation.
    c.limit_min = alias("limit_min_value", "channel8_min_value", c.limit_min);
    c.limit_max = alias("limit_max_value", "channel8_max_value", c.limit_max);
    c.speed_min = alias("speed_limit_min_speed", "speed_channel8_min_speed", c.speed_min);
    c.speed_max = alias("speed_limit_max_speed", "speed_channel8_max_speed", c.speed_max);
    c.current_min = alias(
      "current_limit_min_current", "current_channel8_min_current", c.current_min);
    c.current_max = alias(
      "current_limit_max_current", "current_channel8_max_current", c.current_max);
    c.channel_mid = alias("channel_mid", "steering_channel_mid", c.channel_mid);
    c.channel_min = parameter("channel_min_range", c.channel_min);
    c.channel_max = parameter("channel_max_range", c.channel_max);
    c.deadzone = parameter("channel_deadzone", c.deadzone);
    c.switch_mid = parameter("switch_mid_value", c.channel_mid);
    c.switch_low = parameter(
      "switch_low_threshold", (static_cast<double>(c.channel_min) + c.switch_mid) / 2.0);
    c.switch_high = parameter(
      "switch_high_threshold", (static_cast<double>(c.switch_mid) + c.channel_max) / 2.0);
    c.direction_reverse = parameter("direction_reverse", c.direction_reverse);
    c.steering_reverse = parameter("steering_reverse", c.steering_reverse);
    c.steering_limit = parameter("steering_limit", c.steering_limit);
    c.command_speed_limit = parameter("command_speed_limit", c.speed_max);
    c.command_current_limit = parameter("command_current_limit", c.command_current_limit);
    c.command_duty_limit = parameter("command_duty_limit", c.command_duty_limit);
    c.rc_timeout = parameter("rc_timeout_sec", c.rc_timeout);
    c.nav_timeout = parameter("nav_timeout_sec", c.nav_timeout);
    c.calib_timeout = parameter("calib_timeout_sec", c.calib_timeout);
    require_calib_stamp_ = parameter("require_calibration_stamp", true);
    const double publish_rate = parameter("publish_rate_hz", 200.0);
    if (!std::isfinite(publish_rate) || publish_rate < 1.0 || publish_rate > 1000.0) {
      throw std::invalid_argument("publish_rate_hz must be within [1, 1000]");
    }
    selector_ = std::make_unique<JoystickSelector>(c);

    // Latest samples only: accumulating RC/command history increases actuation latency.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    rc_sub_ = create_subscription<crsf_receiver_msg::msg::CRSFChannels16>(
      "/rc/channels", qos, [this](const crsf_receiver_msg::msg::CRSFChannels16 & msg) {
        selector_->receive_rc(
          Channels{{msg.ch1, msg.ch2, msg.ch3, msg.ch4, msg.ch5, msg.ch6,
            msg.ch7, msg.ch8, msg.ch9, msg.ch10, msg.ch11, msg.ch12, msg.ch13, msg.ch14,
            msg.ch15, msg.ch16}}, std::chrono::steady_clock::now());
      });
    nav_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", qos, [this](const ackermann_msgs::msg::AckermannDriveStamped & msg) {
        selector_->receive_navigation(
          command(msg), std::chrono::steady_clock::now(), source_age(msg, false));
      });
    calib_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/calib/ackermann_cmd", qos, [this](const ackermann_msgs::msg::AckermannDriveStamped & msg) {
        selector_->receive_calibration(
          command(msg), std::chrono::steady_clock::now(), source_age(msg, require_calib_stamp_));
      });
    command_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/ackermann_cmd", 10);
    status_pub_ = create_publisher<std_msgs::msg::Bool>("/control/autonomy_speed_enabled", 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate)),
      [this]() {publish();});
    RCLCPP_INFO(
      get_logger(), "C++ joystick profile=%s: throttle CH%d, steering CH%d, lock CH%d, "
      "ESC CH%d, source CH%d, calibration CH%d, limit CH%d; %.1f Hz; RC/nav/calibration "
      "timeouts %.3f/%.3f/%.3f s", profile.c_str(), c.speed_channel, c.steering_channel,
      c.lock_channel, c.esc_mode_channel, c.control_source_channel, c.calib_mode_channel,
      c.limit_channel, publish_rate, c.rc_timeout, c.nav_timeout, c.calib_timeout);
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & fallback)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;  // Configuration is validated once, before subscriptions start.
    return declare_parameter<T>(name, fallback, descriptor);
  }

  template<typename T>
  T alias(const std::string & name, const std::string & legacy, const T & fallback)
  {
    return parameter(name, parameter(legacy, fallback));
  }

  static DriveCommand command(const ackermann_msgs::msg::AckermannDriveStamped & msg)
  {
    return {msg.drive.steering_angle, msg.drive.speed, msg.drive.acceleration, msg.drive.jerk};
  }

  double source_age(const ackermann_msgs::msg::AckermannDriveStamped & msg, bool required)
  {
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      // Nav2/legacy /drive publishers may omit their header. Calibration publishers
      // in this workspace stamp commands; require it to reject replayed stale commands.
      return required ? std::numeric_limits<double>::quiet_NaN() : 0.0;
    }
    if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    return (get_clock()->now() - stamp).seconds();
  }

  void publish()
  {
    const auto selected = selector_->select(std::chrono::steady_clock::now());
    std_msgs::msg::Bool status;
    status.data = selected.autonomy_speed_enabled;
    status_pub_->publish(status);
    ackermann_msgs::msg::AckermannDriveStamped out;
    out.header.stamp = get_clock()->now();
    out.drive.steering_angle = selected.command.steering;
    out.drive.speed = selected.command.speed;
    out.drive.acceleration = selected.command.acceleration;
    out.drive.jerk = selected.command.jerk;
    command_pub_->publish(out);
    // Log transitions only, including timeouts. A persistent fault cannot emit 200 warnings/s.
    if (!state_reported_ || selected.state != last_state_) {
      RCLCPP_INFO(get_logger(), "Control state: %s", state_name(selected.state));
      last_state_ = selected.state;
      state_reported_ = true;
    }
  }

  std::unique_ptr<JoystickSelector> selector_;
  bool require_calib_stamp_{true}, state_reported_{false};
  SelectionState last_state_{SelectionState::WaitingRC};
  rclcpp::Subscription<crsf_receiver_msg::msg::CRSFChannels16>::SharedPtr rc_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr nav_sub_, calib_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace ackermann_mux

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ackermann_mux::JoystickControl>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("joystick_control"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
