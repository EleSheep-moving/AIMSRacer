#include "aims_racer_system/rear_axle_imu.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aims_racer_system
{
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

RearAxleImu::RearAxleImu(const ImuConfig & config)
: config_(config), history_(config.angular_acceleration_window, config.max_gyro_gap)
{
  if (!config.lever.allFinite() || !config.mounting.coeffs().allFinite() ||
    std::abs(config.mounting.norm() - 1.0) > 1e-3)
  {
    throw std::invalid_argument("Invalid IMU mounting transform");
  }
  if (!std::isfinite(config.accel_scale) || config.accel_scale <= 0.0)
  {
    throw std::invalid_argument("Invalid acceleration scale");
  }
  config_.mounting.normalize();
  rotation_ = config_.mounting.toRotationMatrix();
}

std::optional<ImuOutput> RearAxleImu::process(const ImuInput & input)
{
  if (!input.omega.allFinite() || !input.specific_force.allFinite()) {
    return std::nullopt;
  }
  ImuOutput out;
  Matrix3 force_covariance;
  try {
    out.omega = rotation_ * input.omega;
    const Matrix3 cw = validate_covariance(input.omega_covariance, 0.01);
    out.omega_covariance = rotation_ * cw * rotation_.transpose();
    const double scale_squared = config_.accel_scale * config_.accel_scale;
    const Matrix3 cf = validate_covariance(input.force_covariance, 0.05 / scale_squared);
    force_covariance = rotation_ * cf * rotation_.transpose() * scale_squared;
    history_.add(input.stamp, out.omega, out.omega_covariance);
  } catch (const std::invalid_argument &) {
    return std::nullopt;
  }
  const auto derivative = history_.derivative();
  if (derivative) {
    const Vector3 force = rotation_ * input.specific_force * config_.accel_scale;
    const auto corrected = compensate_force(
      force, out.omega, derivative->alpha, config_.lever,
      force_covariance, out.omega_covariance, derivative->covariance);
    out.compensated = true;
    out.specific_force = corrected.force;
    out.force_covariance = corrected.covariance;
  }
  return out;
}

}  // namespace aims_racer_system
