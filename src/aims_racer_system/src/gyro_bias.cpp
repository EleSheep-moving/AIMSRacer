#include "aims_racer_system/gyro_bias.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aims_racer_system {
GyroBiasCalibrator::GyroBiasCalibrator(GyroBiasParameters parameters):parameters_(parameters)
{
  const auto & p=parameters_;
  const double values[]={p.duration,p.max_observation_age,p.max_imu_gap,p.max_wheel_speed,
    p.max_lio_speed,p.max_position_change,p.max_orientation_change,p.max_gyro_norm,
    p.max_gyro_std,p.max_accel_std,p.accel_norm_min,p.accel_norm_max};
  if (p.min_samples<3 || p.accel_norm_min>=p.accel_norm_max ||
    std::any_of(std::begin(values),std::end(values),[](double v){return !std::isfinite(v) || v<=0.;}))
  {
    throw std::invalid_argument("Calibration parameters must be finite positive values");
  }
}

void GyroBiasCalibrator::clear_window()
{
  samples_.clear();anchor_.reset();sample_count_=0;
}

std::optional<Eigen::Vector3d> GyroBiasCalibrator::reject(const std::string & reason)
{
  clear_window();reason_=reason;return std::nullopt;
}

void GyroBiasCalibrator::add_wheel(double stamp,double speed)
{
  if (!std::isfinite(stamp) || !std::isfinite(speed)) {
    throw std::invalid_argument("Wheel input must be finite");
  }
  if (wheels_.empty() || stamp>wheels_.back().stamp) {
    wheels_.push_back({stamp,speed});
    if (wheels_.size()>200) {wheels_.pop_front();}
  }
}

void GyroBiasCalibrator::add_lio(double stamp,const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation,const Eigen::Vector3d & velocity)
{
  const double norm=orientation.norm();
  if (!std::isfinite(stamp) || !position.allFinite() || !orientation.coeffs().allFinite() ||
    !velocity.allFinite() || std::abs(norm-1.)>.01)
  {
    throw std::invalid_argument("LIO input must have a finite pose and normalized quaternion");
  }
  if (lios_.empty() || stamp>lios_.back().stamp) {
    lios_.push_back({stamp,position,orientation.normalized(),velocity});
    if (lios_.size()>200) {lios_.pop_front();}
  }
}

std::optional<Eigen::Vector3d> GyroBiasCalibrator::process(double stamp,
  const Eigen::Vector3d & gyro,const Eigen::Vector3d & accel)
{
  if (!std::isfinite(stamp) || !gyro.allFinite() || !accel.allFinite()) {
    return reject("invalid IMU input");
  }
  if (last_stamp_ && stamp<=*last_stamp_) {
    ready_=false;bias_.setZero();wheels_.clear();lios_.clear();last_stamp_=stamp;
    return reject("source clock reset; new calibration required");
  }
  const bool gap=last_stamp_ && stamp-*last_stamp_>parameters_.max_imu_gap;
  last_stamp_=stamp;
  if (ready_) {return Eigen::Vector3d(gyro-bias_);}
  if (gap) {reject("IMU gap; restarting stationary window");}
  // Select the most recent observation at or before the IMU source timestamp.
  const Wheel * wheel=nullptr;
  const Lio * lio=nullptr;
  for (auto i=wheels_.rbegin();i!=wheels_.rend();++i) {
    if (i->stamp<=stamp) {
      if (stamp-i->stamp<=parameters_.max_observation_age) {wheel=&*i;}
      break;
    }
  }
  for (auto i=lios_.rbegin();i!=lios_.rend();++i) {
    if (i->stamp<=stamp) {
      if (stamp-i->stamp<=parameters_.max_observation_age) {lio=&*i;}
      break;
    }
  }
  if (!wheel || !lio) {return reject("missing or stale source-time wheel/LIO observations");}
  const auto & p=parameters_;
  if (std::abs(wheel->speed)>p.max_wheel_speed || lio->velocity.norm()>p.max_lio_speed) {
    return reject("wheel/LIO motion; stationary calibration pending");
  }
  const double accel_norm=accel.norm();
  if (gyro.norm()>p.max_gyro_norm || accel_norm<p.accel_norm_min || accel_norm>p.accel_norm_max) {
    return reject("IMU motion or invalid raw acceleration units");
  }
  if (!anchor_) {anchor_=*lio;} else {
    const double angle=2.*std::acos(std::clamp(std::abs(anchor_->orientation.dot(lio->orientation)),0.,1.));
    if ((lio->position-anchor_->position).norm()>p.max_position_change || angle>p.max_orientation_change) {
      return reject("LIO pose changed during calibration");
    }
  }
  // Saturating multiplication avoids signed overflow for invalidly large ROS integer settings.
  const auto min_samples=static_cast<std::uint64_t>(p.min_samples);
  const auto doubled=min_samples>std::numeric_limits<std::uint64_t>::max()/2 ?
    std::numeric_limits<std::uint64_t>::max() : min_samples*2;
  if (samples_.size()>=std::max<std::uint64_t>(10000,doubled)) {
    return reject("calibration sample budget exceeded");
  }
  samples_.push_back({stamp,gyro,accel});sample_count_=samples_.size();
  reason_="collecting stationary samples";
  if (stamp-samples_.front().stamp<p.duration || sample_count_<min_samples) {return std::nullopt;}
  Eigen::Vector3d gyro_mean=Eigen::Vector3d::Zero(),accel_mean=Eigen::Vector3d::Zero();
  for (const auto & sample:samples_) {gyro_mean+=sample.gyro;accel_mean+=sample.accel;}
  gyro_mean/=static_cast<double>(sample_count_);accel_mean/=static_cast<double>(sample_count_);
  Eigen::Vector3d gyro_variance=Eigen::Vector3d::Zero(),accel_variance=Eigen::Vector3d::Zero();
  for (const auto & sample:samples_) {
    gyro_variance+=(sample.gyro-gyro_mean).array().square().matrix();
    accel_variance+=(sample.accel-accel_mean).array().square().matrix();
  }
  const double denominator=static_cast<double>(sample_count_-1);
  gyro_std_=(gyro_variance/denominator).array().sqrt().matrix();
  if (gyro_std_.maxCoeff()>p.max_gyro_std ||
    (accel_variance/denominator).array().sqrt().maxCoeff()>p.max_accel_std)
  {
    return reject("sensor variation too high; restarting stationary window");
  }
  bias_=gyro_mean;ready_=true;
  reason_="calibrated; bias frozen until restart or source-clock reset";
  samples_.clear();anchor_.reset();
  return Eigen::Vector3d(gyro-bias_);
}

sensor_msgs::msg::Imu correct_imu(const sensor_msgs::msg::Imu & message,const Eigen::Vector3d & bias)
{
  if (!bias.allFinite() || message.angular_velocity_covariance[0]==-1.) {
    throw std::invalid_argument("Finite gyro bias and available angular velocity required");
  }
  auto output=message;
  output.angular_velocity.x-=bias.x();output.angular_velocity.y-=bias.y();output.angular_velocity.z-=bias.z();
  if (std::any_of(output.angular_velocity_covariance.begin(),output.angular_velocity_covariance.end(),
    [](double v){return v!=0.;}))
  {
    for (const auto i:{0,4,8}) {output.angular_velocity_covariance[i]+=1e-6;}
  }
  return output;
}
}  // namespace aims_racer_system
