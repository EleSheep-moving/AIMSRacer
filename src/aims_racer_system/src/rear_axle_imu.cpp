#include "aims_racer_system/rear_axle_imu.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aims_racer_system
{
namespace
{
double maximum_eigenvalue(const Matrix3 & covariance)
{
  Eigen::SelfAdjointEigenSolver<Matrix3> solver(covariance, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success) {
    throw std::invalid_argument("Covariance eigendecomposition failed");
  }
  return solver.eigenvalues().maxCoeff();
}

Quaternion integrate(const Quaternion & rotation, const Vector3 & increment)
{
  const double angle = increment.norm();
  Quaternion delta = Quaternion::Identity();
  if (angle > 0.0) {
    delta = Quaternion(Eigen::AngleAxisd(angle, increment / angle));
  }
  return (rotation * delta).normalized();
}
}  // namespace

Matrix3 validate_covariance(const Matrix3 & covariance, double fallback)
{
  if (!covariance.allFinite()) {
    throw std::invalid_argument("Nonfinite covariance");
  }
  if (covariance(0, 0) == -1.0) {
    throw std::invalid_argument("Sensor marks measurement unavailable");
  }
  const Matrix3 symmetric = (covariance + covariance.transpose()) * 0.5;
  // Zero covariance is common on raw Livox messages; avoid an eigensolve for it.
  if (symmetric.isZero(0.0)) {
    return Matrix3::Identity() * fallback;
  }
  Eigen::SelfAdjointEigenSolver<Matrix3> solver(symmetric, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-9) {
    throw std::invalid_argument("Invalid covariance");
  }
  return symmetric;
}

ForceEstimate compensate_force(
  const Vector3 & force, const Vector3 & omega, const Vector3 & alpha,
  const Vector3 & lever, const Matrix3 & force_covariance,
  const Matrix3 & omega_covariance, const Matrix3 & alpha_covariance)
{
  const Vector3 corrected = force - alpha.cross(lever) - omega.cross(omega.cross(lever));
  const Matrix3 jw = 2.0 * lever * omega.transpose() - omega * lever.transpose() -
    omega.dot(lever) * Matrix3::Identity();
  Matrix3 ja;
  ja << 0.0, -lever.z(), lever.y(), lever.z(), 0.0, -lever.x(),
    -lever.y(), lever.x(), 0.0;
  const Matrix3 covariance = force_covariance + 2.0 * (
    jw * omega_covariance * jw.transpose() + ja * alpha_covariance * ja.transpose());
  return {corrected, (covariance + covariance.transpose()) * 0.5};
}

AngularHistory::AngularHistory(double window, double max_gap)
: window_(window), max_gap_(max_gap)
{
  if (!std::isfinite(window) || !std::isfinite(max_gap) ||
    window <= 0.0 || window > 0.2 || max_gap <= 0.0 || max_gap > 0.2)
  {
    throw std::invalid_argument("Invalid gyro window/gap");
  }
}

bool AngularHistory::add(double stamp, const Vector3 & omega, const Matrix3 & covariance)
{
  if (!std::isfinite(stamp) || !omega.allFinite() || !covariance.allFinite()) {
    throw std::invalid_argument("Nonfinite gyro input");
  }
  const bool reset = !samples_.empty() &&
    (stamp <= samples_.back().stamp || stamp - samples_.back().stamp > max_gap_);
  if (reset) {
    samples_.clear();
  }
  samples_.push_back({stamp, omega, covariance});
  if (samples_.size() > 400) {
    samples_.pop_front();
  }
  return reset;
}

std::optional<AngularHistory::Derivative> AngularHistory::derivative() const
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  const double now = samples_.back().stamp;
  auto first = samples_.end();
  while (first != samples_.begin() && now - std::prev(first)->stamp <= window_ + 1e-9) {
    --first;
  }
  const auto count = std::distance(first, samples_.end());
  if (count < 3) {
    return std::nullopt;
  }
  double mean = 0.0;
  for (auto it = first; it != samples_.end(); ++it) {
    mean += it->stamp - now;
  }
  mean /= static_cast<double>(count);
  double denominator = 0.0;
  for (auto it = first; it != samples_.end(); ++it) {
    const double time = it->stamp - now - mean;
    denominator += time * time;
  }
  if (denominator < 1e-10) {
    return std::nullopt;
  }
  Derivative result{Vector3::Zero(), Matrix3::Identity() * 0.25};
  for (auto it = first; it != samples_.end(); ++it) {
    const double weight = (it->stamp - now - mean) / denominator;
    result.alpha += weight * it->omega;
    result.covariance += weight * weight * it->covariance;
  }
  return result;
}

std::optional<Quaternion> AngularHistory::orientation(
  double stamp, const Quaternion & quaternion, double target) const
{
  if (target < stamp || samples_.empty() || samples_.front().stamp > stamp ||
    samples_.back().stamp < target)
  {
    return std::nullopt;
  }
  auto next = std::upper_bound(
    samples_.begin(), samples_.end(), stamp,
    [](double value, const Sample & sample) {return value < sample.stamp;});
  const auto & left = *std::prev(next);
  Vector3 omega = left.omega;
  if (next != samples_.end()) {
    const double fraction = (stamp - left.stamp) / (next->stamp - left.stamp);
    omega += fraction * (next->omega - left.omega);
  }
  Quaternion rotation = quaternion.normalized();
  double previous = stamp;
  // At recorded knots the rates are already known: no arrays or repeated interpolation.
  for (; next != samples_.end() && next->stamp < target; ++next) {
    rotation = integrate(rotation, 0.5 * (omega + next->omega) * (next->stamp - previous));
    previous = next->stamp;
    omega = next->omega;
  }
  if (target > previous) {
    Vector3 final_omega = omega;
    if (next != samples_.end()) {
      const double fraction = (target - previous) / (next->stamp - previous);
      final_omega += fraction * (next->omega - omega);
    }
    rotation = integrate(rotation, 0.5 * (omega + final_omega) * (target - previous));
  }
  return rotation;
}

RearAxleImu::RearAxleImu(const ImuConfig & config)
: config_(config), history_(config.angular_acceleration_window, config.max_gyro_gap)
{
  if (!config.lever.allFinite() || !config.mounting.coeffs().allFinite() ||
    std::abs(config.mounting.norm() - 1.0) > 1e-3)
  {
    throw std::invalid_argument("Invalid IMU mounting transform");
  }
  if (!std::isfinite(config.accel_scale) || config.accel_scale <= 0.0 ||
    !std::isfinite(config.max_attitude_age) || config.max_attitude_age <= 0.0 ||
    config.max_attitude_age > 1.0)
  {
    throw std::invalid_argument("Invalid scale or attitude age");
  }
  config_.mounting.normalize();
  rotation_ = config_.mounting.toRotationMatrix();
}

bool RearAxleImu::add_attitude(
  double stamp, const Quaternion & world_imu, const Matrix3 & covariance)
{
  if (!std::isfinite(stamp) || !world_imu.coeffs().allFinite() ||
    std::abs(world_imu.norm() - 1.0) > 1e-2 ||
    (!attitudes_.empty() && stamp <= attitudes_.back().stamp))
  {
    return false;
  }
  try {
    const Matrix3 valid = validate_covariance(covariance, 0.01);
    attitudes_.push_back({stamp, (world_imu.normalized() * config_.mounting.conjugate()).normalized(),
        maximum_eigenvalue(valid)});
    if (attitudes_.size() > 100) {
      attitudes_.pop_front();
    }
  } catch (const std::invalid_argument &) {
    return false;
  }
  return true;
}

std::optional<ImuOutput> RearAxleImu::process(const ImuInput & input)
{
  if (!input.omega.allFinite() || (!config_.gyro_only && !input.specific_force.allFinite())) {
    return std::nullopt;
  }
  ImuOutput out;
  Matrix3 force_covariance;
  try {
    out.omega = rotation_ * input.omega;
    const Matrix3 cw = validate_covariance(input.omega_covariance, 0.01);
    out.omega_covariance = rotation_ * cw * rotation_.transpose();
    if (config_.gyro_only) {
      return out;
    }
    const double scale_squared = config_.accel_scale * config_.accel_scale;
    const Matrix3 cf = validate_covariance(input.force_covariance, 0.05 / scale_squared);
    force_covariance = rotation_ * cf * rotation_.transpose() * scale_squared;
    if (history_.add(input.stamp, out.omega, out.omega_covariance)) {
      attitudes_.clear();
    }
  } catch (const std::invalid_argument &) {
    return std::nullopt;
  }
  const auto derivative = history_.derivative();
  const Attitude * anchor = nullptr;
  for (auto it = attitudes_.rbegin(); it != attitudes_.rend(); ++it) {
    const double age = input.stamp - it->stamp;
    if (age > config_.max_attitude_age) {
      break;
    }
    if (age >= 0.0) {
      anchor = &*it;
      break;
    }
  }
  if (derivative && anchor) {
    const auto propagated = history_.orientation(anchor->stamp, anchor->quaternion, input.stamp);
    if (propagated) {
      const Vector3 force = rotation_ * input.specific_force * config_.accel_scale;
      const auto corrected = compensate_force(
        force, out.omega, derivative->alpha, config_.lever,
        force_covariance, out.omega_covariance, derivative->covariance);
      const double age = input.stamp - anchor->stamp;
      const double variance = std::max(anchor->maximum_variance, 1e-6) +
        age * age * maximum_eigenvalue(out.omega_covariance);
      out.compensated = true;
      out.orientation = *propagated;
      out.orientation_covariance = Matrix3::Identity() * variance;
      out.specific_force = corrected.force;
      out.force_covariance = corrected.covariance + Matrix3::Identity() * 9.80665 * 9.80665 * variance;
    }
  }
  return out;
}
}  // namespace aims_racer_system
