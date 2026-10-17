#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <array>
#include <cstdint>
#include <deque>
#include <optional>

namespace aims_racer_system
{
struct GyroSample
{
  std::int64_t stamp_ns;
  Eigen::Vector3d omega;
};
using GyroHistory = std::deque<GyroSample>;

// Exact measured knot or interpolation in source time; never extrapolate.
std::optional<Eigen::Vector3d> source_time_angular_velocity(
  const GyroHistory & history, std::int64_t stamp_ns,
  std::int64_t max_interval_ns = 30000000LL);

struct OdometryConfig
{
  // T_base_livox. Quaternion input follows Eigen's w,x,y,z constructor order.
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
  std::array<double, 6> pose_variance{{.01, .01, .01, .01, .01, .01}};
  std::array<double, 6> twist_variance{{.04, .04, .04, .01, .01, .01}};
};

// Pose covariance: world-fixed small-angle errors. Twist: body-expressed.
// Floors are conservative assumptions, not calibrated uncertainty estimates.
nav_msgs::msg::Odometry convert_odometry(
  const nav_msgs::msg::Odometry & msg, const Eigen::Vector3d & angular_velocity,
  const OdometryConfig & config);
}  // namespace aims_racer_system
