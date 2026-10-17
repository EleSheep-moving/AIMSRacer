#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/imu.hpp>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace aims_racer_system {
struct GyroBiasParameters {
  double duration=10.;
  std::int64_t min_samples=1000;
  double max_observation_age=.25, max_imu_gap=.05, max_wheel_speed=.02,
    max_lio_speed=.025, max_position_change=.02, max_orientation_change=.01,
    max_gyro_norm=.1, max_gyro_std=.005, max_accel_std=.02,
    accel_norm_min=.75, accel_norm_max=1.25;
};

class GyroBiasCalibrator {
public:
  explicit GyroBiasCalibrator(GyroBiasParameters parameters={});
  void add_wheel(double stamp, double speed);
  void add_lio(double stamp, const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation, const Eigen::Vector3d & velocity);
  std::optional<Eigen::Vector3d> process(double stamp, const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & accel);
  bool ready() const {return ready_;}
  const Eigen::Vector3d & bias() const {return bias_;}
  const Eigen::Vector3d & gyro_std() const {return gyro_std_;}
  const std::string & reason() const {return reason_;}
  std::size_t sample_count() const {return sample_count_;}
  const GyroBiasParameters & parameters() const {return parameters_;}
private:
  struct Wheel {double stamp; double speed;};
  struct Lio {double stamp; Eigen::Vector3d position; Eigen::Quaterniond orientation; Eigen::Vector3d velocity;};
  struct Sample {double stamp; Eigen::Vector3d gyro; Eigen::Vector3d accel;};
  void clear_window();
  std::optional<Eigen::Vector3d> reject(const std::string & reason);
  GyroBiasParameters parameters_;
  std::deque<Wheel> wheels_;
  std::deque<Lio> lios_;
  std::optional<double> last_stamp_;
  bool ready_=false;
  Eigen::Vector3d bias_=Eigen::Vector3d::Zero(),gyro_std_=Eigen::Vector3d::Zero();
  std::string reason_="waiting for stationary wheel and LIO observations";
  std::size_t sample_count_=0;
  std::vector<Sample> samples_;
  std::optional<Lio> anchor_;
};
sensor_msgs::msg::Imu correct_imu(const sensor_msgs::msg::Imu & message,
  const Eigen::Vector3d & bias);
}  // namespace aims_racer_system
