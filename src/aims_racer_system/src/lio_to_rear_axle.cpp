#include "aims_racer_system/rear_axle_odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace aims_racer_system
{
namespace
{
std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

class LioToRearAxle : public rclcpp::Node
{
public:
  LioToRearAxle()
  : Node("lio_to_rear_axle")
  {
    const auto translation = declare_parameter<std::vector<double>>("livox_translation", {0., 0., 0.});
    const auto quaternion = declare_parameter<std::vector<double>>("livox_quaternion", {0., 0., 0., 1.});
    const auto pose_variance = declare_parameter<std::vector<double>>("pose_variance", {.01, .01, .01, .01, .01, .01});
    const auto twist_variance = declare_parameter<std::vector<double>>("twist_variance", {.04, .04, .04, .01, .01, .01});
    if (translation.size() != 3 || quaternion.size() != 4 ||
      pose_variance.size() != 6 || twist_variance.size() != 6)
    {
      throw std::invalid_argument("Translation/quaternion/variance require 3/4/6 values");
    }
    config_.translation = Eigen::Vector3d(translation.data());
    config_.quaternion = Eigen::Quaterniond(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    std::copy(pose_variance.begin(), pose_variance.end(), config_.pose_variance.begin());
    std::copy(twist_variance.begin(), twist_variance.end(), config_.twist_variance.begin());
    const double interval = declare_parameter("max_gyro_interval", .03);
    if (!std::isfinite(interval) || interval <= 0. || interval >= 9e9) {
      throw std::invalid_argument("max_gyro_interval must be positive and representable");
    }
    max_interval_ns_ = static_cast<std::int64_t>(interval * 1e9);
    if (max_interval_ns_ <= 0) {
      throw std::invalid_argument("max_gyro_interval must be positive");
    }
    publisher_ = create_publisher<nav_msgs::msg::Odometry>("/rear_axle/lio_odom", 10);
    if (declare_parameter("publish_tf", false)) {
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {imu(*msg);});
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fastlio2/lio_odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odometry(msg);});
  }

private:
  void imu(const sensor_msgs::msg::Imu & msg)
  {
    if (msg.header.frame_id != "livox_frame") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Expected raw IMU in livox_frame");
      return;
    }
    const auto stamp = stamp_nanoseconds(msg.header.stamp);
    const auto & w = msg.angular_velocity;
    const Eigen::Vector3d omega(w.x, w.y, w.z);
    if (!omega.allFinite()) {return;}
    if (!history_.empty() && stamp < history_.back().stamp_ns) {
      history_.clear();
      pending_.clear();
      last_odom_.reset();
    }
    if (!history_.empty() && stamp == history_.back().stamp_ns) {return;}
    if (history_.size() == 2000) {history_.pop_front();}
    history_.push_back({stamp, omega});
    publish_pending();
  }

  void odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    const auto stamp = stamp_nanoseconds(msg->header.stamp);
    if ((last_odom_ && stamp <= *last_odom_) ||
      (!pending_.empty() && stamp <= stamp_nanoseconds(pending_.back()->header.stamp)))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping nonmonotonic LIO stamp");
      return;
    }
    if (pending_.size() == 20) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping oldest pending LIO: IMU coverage unavailable");
      pending_.pop_front();
    }
    pending_.push_back(msg);
    publish_pending();
  }

  void publish_pending()
  {
    while (!pending_.empty() && !history_.empty()) {
      const auto msg = pending_.front();
      const auto stamp = stamp_nanoseconds(msg->header.stamp);
      if (stamp > history_.back().stamp_ns) {return;}
      const auto omega = source_time_angular_velocity(history_, stamp, max_interval_ns_);
      pending_.pop_front();
      if (!omega) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Dropping LIO: missing source-time IMU coverage");
        continue;
      }
      try {
        const auto out = convert_odometry(*msg, *omega, config_);
        last_odom_ = stamp;
        publisher_->publish(out);
        if (tf_) {
          geometry_msgs::msg::TransformStamped transform;
          transform.header = out.header;
          transform.child_frame_id = out.child_frame_id;
          transform.transform.translation.x = out.pose.pose.position.x;
          transform.transform.translation.y = out.pose.pose.position.y;
          transform.transform.translation.z = out.pose.pose.position.z;
          transform.transform.rotation = out.pose.pose.orientation;
          tf_->sendTransform(transform);
        }
      } catch (const std::invalid_argument & error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
      }
    }
  }

  OdometryConfig config_;
  std::int64_t max_interval_ns_;
  GyroHistory history_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> pending_;
  std::optional<std::int64_t> last_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};
}  // namespace aims_racer_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aims_racer_system::LioToRearAxle>());
  rclcpp::shutdown();
  return 0;
}
