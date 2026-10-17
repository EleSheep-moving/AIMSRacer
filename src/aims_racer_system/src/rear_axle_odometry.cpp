#include "aims_racer_system/rear_axle_odometry.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aims_racer_system
{
namespace
{
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using RowMatrix6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

Eigen::Matrix3d skew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d matrix;
  matrix << 0., -vector.z(), vector.y(), vector.z(), 0., -vector.x(),
    -vector.y(), vector.x(), 0.;
  return matrix;
}

Matrix6 covariance(const std::array<double, 36> & values, const std::array<double, 6> & floors)
{
  const Matrix6 input = Eigen::Map<const RowMatrix6>(values.data());
  if (!input.allFinite()) {
    throw std::invalid_argument("Nonfinite odometry covariance");
  }
  Matrix6 result = .5 * (input + input.transpose());
  const Eigen::SelfAdjointEigenSolver<Matrix6> eigen(result, Eigen::EigenvaluesOnly);
  if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() < -1e-9) {
    throw std::invalid_argument("Invalid odometry covariance");
  }
  for (int i = 0; i < 6; ++i) {
    result(i, i) += std::max(0., floors[i] - input(i, i));
  }
  return result;
}

void copy_covariance(const Matrix6 & input, std::array<double, 36> & output)
{
  Eigen::Map<RowMatrix6> mapped(output.data());
  mapped = input;
}
}  // namespace

std::optional<Eigen::Vector3d> source_time_angular_velocity(
  const GyroHistory & history, std::int64_t stamp_ns, std::int64_t max_interval_ns)
{
  const auto right = std::lower_bound(
    history.begin(), history.end(), stamp_ns,
    [](const GyroSample & sample, std::int64_t stamp) {return sample.stamp_ns < stamp;});
  if (right != history.end() && right->stamp_ns == stamp_ns) {
    return right->omega;
  }
  if (right == history.begin() || right == history.end()) {
    return std::nullopt;
  }
  const auto & left = *std::prev(right);
  const auto interval = right->stamp_ns - left.stamp_ns;
  if (interval <= 0 || interval > max_interval_ns) {
    return std::nullopt;
  }
  const double fraction = static_cast<double>(stamp_ns - left.stamp_ns) / interval;
  return (1. - fraction) * left.omega + fraction * right->omega;
}

nav_msgs::msg::Odometry convert_odometry(
  const nav_msgs::msg::Odometry & msg, const Eigen::Vector3d & angular_velocity,
  const OdometryConfig & config)
{
  if (msg.header.frame_id != "odom" || msg.child_frame_id != "livox_frame") {
    throw std::invalid_argument("Expected odom / livox_frame odometry");
  }
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  const auto & v = msg.twist.twist.linear;
  const Eigen::Vector3d position(p.x, p.y, p.z);
  const Eigen::Vector3d velocity(v.x, v.y, v.z);
  const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (!position.allFinite() || !velocity.allFinite() || !orientation.coeffs().allFinite() ||
    !angular_velocity.allFinite() || !config.translation.allFinite() ||
    !config.quaternion.coeffs().allFinite())
  {
    throw std::invalid_argument("Nonfinite state or extrinsic");
  }
  if (std::abs(orientation.norm() - 1.) > .01 ||
    std::abs(config.quaternion.norm() - 1.) > .01)
  {
    throw std::invalid_argument("Invalid orientation quaternion");
  }
  const Eigen::Matrix3d r_wi = orientation.normalized().toRotationMatrix();
  const Eigen::Matrix3d r_bi = config.quaternion.normalized().toRotationMatrix();
  const Eigen::Matrix3d r_wb = r_wi * r_bi.transpose();
  const Eigen::Vector3d lever_world = r_wb * config.translation;
  const Eigen::Vector3d base_position = position - lever_world;
  const Eigen::Vector3d omega = r_bi * angular_velocity;
  const Eigen::Vector3d base_velocity = r_bi * velocity - omega.cross(config.translation);

  auto out = msg;
  out.child_frame_id = "base_link";
  out.pose.pose.position.x = base_position.x();
  out.pose.pose.position.y = base_position.y();
  out.pose.pose.position.z = base_position.z();
  const Eigen::Quaterniond result = Eigen::Quaterniond(r_wb).normalized();
  out.pose.pose.orientation.x = result.x();
  out.pose.pose.orientation.y = result.y();
  out.pose.pose.orientation.z = result.z();
  out.pose.pose.orientation.w = result.w();
  out.twist.twist.linear.x = base_velocity.x();
  out.twist.twist.linear.y = base_velocity.y();
  out.twist.twist.linear.z = base_velocity.z();
  out.twist.twist.angular.x = omega.x();
  out.twist.twist.angular.y = omega.y();
  out.twist.twist.angular.z = omega.z();

  Matrix6 jp = Matrix6::Identity();
  jp.block<3, 3>(0, 3) = skew(lever_world);
  Matrix6 jt = Matrix6::Zero();
  jt.block<3, 3>(0, 0) = r_bi;
  jt.block<3, 3>(3, 3) = r_bi;
  jt.block<3, 3>(0, 3) = skew(config.translation) * r_bi;
  copy_covariance(jp * covariance(msg.pose.covariance, config.pose_variance) * jp.transpose(),
    out.pose.covariance);
  copy_covariance(jt * covariance(msg.twist.covariance, config.twist_variance) * jt.transpose(),
    out.twist.covariance);
  return out;
}
}  // namespace aims_racer_system
