#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <optional>

namespace aims_racer_system
{
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;

Matrix3 validate_covariance(const Matrix3 & covariance, double fallback);

struct ForceEstimate
{
  Vector3 force;
  Matrix3 covariance;
};

ForceEstimate compensate_force(
  const Vector3 & force, const Vector3 & omega, const Vector3 & alpha,
  const Vector3 & lever, const Matrix3 & force_covariance,
  const Matrix3 & omega_covariance, const Matrix3 & alpha_covariance);

class AngularHistory
{
public:
  struct Derivative
  {
    Vector3 alpha;
    Matrix3 covariance;
  };

  explicit AngularHistory(double window = 0.025, double max_gap = 0.03);
  bool add(double stamp, const Vector3 & omega, const Matrix3 & covariance);
  std::optional<Derivative> derivative() const;

private:
  struct Sample
  {
    double stamp;
    Vector3 omega;
    Matrix3 covariance;
  };
  std::deque<Sample> samples_;
  double window_;
  double max_gap_;
};

struct ImuConfig
{
  Vector3 lever = Vector3::Zero();
  Quaternion mounting = Quaternion::Identity();
  double accel_scale = 9.80665;
  double angular_acceleration_window = 0.025;
  double max_gyro_gap = 0.03;
};

struct ImuInput
{
  double stamp;
  Vector3 omega;
  Vector3 specific_force;
  Matrix3 omega_covariance;
  Matrix3 force_covariance;
};

struct ImuOutput
{
  Vector3 omega;
  Matrix3 omega_covariance;
  bool compensated = false;
  Vector3 specific_force = Vector3::Zero();
  Matrix3 force_covariance = Matrix3::Zero();
};

// No ROS dependencies: source timestamps and unavailable-data semantics are explicit.
class RearAxleImu
{
public:
  explicit RearAxleImu(const ImuConfig & config);
  std::optional<ImuOutput> process(const ImuInput & input);

private:
  ImuConfig config_;
  Matrix3 rotation_;
  AngularHistory history_;
};
}  // namespace aims_racer_system
