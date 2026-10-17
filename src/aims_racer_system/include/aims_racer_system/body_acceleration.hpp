#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <cstddef>
#include <memory>

namespace aims_racer_system::body_acceleration
{
struct Config
{
  Eigen::Quaterniond mounting = Eigen::Quaterniond::Identity(); // IMU -> body
  Eigen::Vector3d lever = Eigen::Vector3d(0.30, 0.0, 0.03); // rear -> IMU, body metres
  Eigen::Vector3d accel_bias_body = Eigen::Vector3d::Zero(); // m/s^2, fixed calibration
  double gravity = 9.80665; // also raw g -> m/s^2 scale; changing this changes both
  double alpha_window_s = 0.025;
  double min_alpha_span_s = 0.0; // zero preserves legacy derivative admission
  double max_gyro_gap_s = 0.03;
  double max_lio_age_s = 0.5;
  bool continue_on_lio_timeout = false; // status warns while gyro-propagated attitude remains usable
  double history_s = 2.0;
  std::size_t max_events = 4096; // combined retained IMU + LIO events
  bool filter_enabled = true;
  bool lateral_enabled = false; // opt-in matched XY filtering; legacy output stays x-only
  double jerk_psd = 1e4; // continuous white noise driving acceleration derivative
  double force_variance = 0.01; // per axis m^2/s^4, MODEL assumptions, not calibration
  double gyro_variance = 1e-4; // per axis rad^2/s^2
  double tilt_std_rad = 0.008726646259971648; // 0.5 degree, MODEL assumption
};
struct ImuSample
{
  int64_t stamp_ns = 0;
  Eigen::Vector3d specific_force_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_rad_s = Eigen::Vector3d::Zero();
};
struct LioSample
{
  int64_t stamp_ns = 0;
  Eigen::Quaterniond q_world_body = Eigen::Quaterniond::Identity();
};
enum class Status { no_imu, no_orientation, gyro_warmup, gyro_time_support, gyro_gap, lio_stale, numerical_error, valid };
const char * status_name(Status status);
struct Estimate
{
  int64_t stamp_ns = 0;
  int64_t anchor_stamp_ns = 0;
  bool valid = false;
  Status status = Status::no_imu;
  Eigen::Quaterniond q_world_body = Eigen::Quaterniond::Identity();
  Eigen::Vector3d force_body = Eigen::Vector3d::Zero(); // mounted and converted to m/s^2
  Eigen::Vector3d corrected_force_body = Eigen::Vector3d::Zero(); // minus fixed bias
  Eigen::Vector3d gravity_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_body = Eigen::Vector3d::Zero(); // unsmoothed mounted gyro
  Eigen::Vector3d alpha_body = Eigen::Vector3d::Zero(); // causal regression
  Eigen::Vector3d tangential = Eigen::Vector3d::Zero(); // alpha x lever, subtracted
  Eigen::Vector3d centripetal = Eigen::Vector3d::Zero(); // omega x (omega x lever), subtracted
  Eigen::Vector3d acceleration_body = Eigen::Vector3d::Zero(); // inertial at rear, body basis
  double alpha_window_span_s = 0.0; // retained causal gyro sample support
  double alpha_sample_center_age_s = 0.0; // sample centre, not an exact effective timestamp
  double ax = 0.0; // optional filtered longitudinal acceleration
  double jerk = 0.0;
  double measurement_variance = 0.0;
  double acceleration_variance = 0.0; // conservative model floor; not calibrated precision
  double ay = 0.0;
  double lateral_jerk = 0.0;
  double lateral_acceleration_variance = 0.0;
  bool lateral_valid = false;
};
struct Counters
{
  uint64_t accepted_imu = 0, accepted_lio = 0, invalid = 0, duplicate = 0, too_old = 0;
  uint64_t replays = 0, replayed_imu = 0, gyro_gaps = 0, evicted = 0;
  uint64_t numerical_error = 0; // subset of invalid: transactional arithmetic rejections
};
// Source-time processing only. The caller controls publication: add_lio may revise latest()
// at the same IMU stamp, but never emits a historical estimate. Values returned earlier
// are independent copies. No wheel, odometry velocity or navigation-EKF input exists.
class Observer
{
public:
  explicit Observer(const Config & config = Config{});
  ~Observer();
  Observer(Observer &&) noexcept;
  Observer & operator=(Observer &&) noexcept;
  Observer(const Observer &) = delete;
  Observer & operator=(const Observer &) = delete;
  bool add_imu(const ImuSample & sample);
  bool add_lio(const LioSample & sample);
  Estimate latest() const;
  Counters counters() const;
  std::size_t history_size() const;
  void reset();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace aims_racer_system::body_acceleration
