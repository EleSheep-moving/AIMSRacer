#include "aims_racer_system/rear_axle_imu.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <array>
#include <memory>
#include <stdexcept>
#include <vector>

namespace aims_racer_system
{
namespace
{
double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

Matrix3 matrix(const std::array<double, 9> & values)
{
  return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(values.data());
}

void copy_matrix(const Matrix3 & matrix, std::array<double, 9> & values)
{
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> output(values.data());
  output = matrix;
}

void copy_vector(const Vector3 & vector, geometry_msgs::msg::Vector3 & output)
{
  output.x = vector.x();
  output.y = vector.y();
  output.z = vector.z();
}
}  // namespace

class ImuToRearAxle : public rclcpp::Node
{
public:
  ImuToRearAxle()
  : Node("imu_to_rear_axle")
  {
    ImuConfig config;
    const auto lever = declare_parameter<std::vector<double>>("livox_translation", {0., 0., 0.});
    const auto quaternion = declare_parameter<std::vector<double>>("livox_quaternion", {0., 0., 0., 1.});
    if (lever.size() != 3 || quaternion.size() != 4) {
      throw std::invalid_argument("Mounting translation/quaternion require 3/4 values");
    }
    config.lever = Vector3(lever[0], lever[1], lever[2]);
    config.mounting = Quaternion(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    config.accel_scale = declare_parameter("accel_scale", 9.80665);
    config.max_attitude_age = declare_parameter("max_attitude_age", 0.25);
    config.angular_acceleration_window = declare_parameter("angular_acceleration_window", 0.025);
    config.max_gyro_gap = declare_parameter("max_gyro_gap", 0.03);
    processor_ = std::make_unique<RearAxleImu>(config);
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("/rear_axle/imu", rclcpp::QoS(50));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {imu(*msg);});
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fastlio2/lio_odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odometry(*msg);});
  }

private:
  void odometry(const nav_msgs::msg::Odometry & msg)
  {
    if (msg.header.frame_id != "odom" || msg.child_frame_id != "livox_frame") {
      return;
    }
    const auto & q = msg.pose.pose.orientation;
    Matrix3 covariance;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance(row, column) = msg.pose.covariance[(row + 3) * 6 + column + 3];
      }
    }
    processor_->add_attitude(seconds(msg.header.stamp), Quaternion(q.w, q.x, q.y, q.z), covariance);
  }

  void imu(const sensor_msgs::msg::Imu & msg)
  {
    if (msg.header.frame_id != "livox_frame") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Expected raw IMU in livox_frame");
      return;
    }
    const auto & w = msg.angular_velocity;
    const auto & f = msg.linear_acceleration;
    const auto result = processor_->process({seconds(msg.header.stamp),
        Vector3(w.x, w.y, w.z), Vector3(f.x, f.y, f.z),
        matrix(msg.angular_velocity_covariance), matrix(msg.linear_acceleration_covariance)});
    if (!result) {
      return;
    }
    sensor_msgs::msg::Imu out;
    out.header = msg.header;
    out.header.frame_id = "base_link";
    out.orientation.w = 1.0;
    out.orientation_covariance[0] = -1.0;
    out.linear_acceleration_covariance[0] = -1.0;
    copy_vector(result->omega, out.angular_velocity);
    copy_matrix(result->omega_covariance, out.angular_velocity_covariance);
    if (result->compensated) {
      out.orientation.x = result->orientation.x();
      out.orientation.y = result->orientation.y();
      out.orientation.z = result->orientation.z();
      out.orientation.w = result->orientation.w();
      copy_matrix(result->orientation_covariance, out.orientation_covariance);
      copy_vector(result->specific_force, out.linear_acceleration);
      copy_matrix(result->force_covariance, out.linear_acceleration_covariance);
    }
    publisher_->publish(out);
  }

  std::unique_ptr<RearAxleImu> processor_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};
}  // namespace aims_racer_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<aims_racer_system::ImuToRearAxle>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_to_rear_axle"), "%s", exception.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
