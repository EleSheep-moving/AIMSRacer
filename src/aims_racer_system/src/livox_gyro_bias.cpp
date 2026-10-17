// Calibrate once in the raw Livox frame, then share a frozen corrected stream.
#include "aims_racer_system/gyro_bias.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace aims_racer_system {
namespace {
double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec+stamp.nanosec*1e-9;
}
std::string json_vector(const Eigen::Vector3d & value)
{
  std::ostringstream out;
  out<<std::setprecision(17)<<'['<<value.x()<<", "<<value.y()<<", "<<value.z()<<']';
  return out.str();
}
}
class LivoxGyroBias : public rclcpp::Node {
public:
  LivoxGyroBias():Node("livox_gyro_bias"),calibrator_(declare_calibration_parameters())
  {
    output_=create_publisher<sensor_msgs::msg::Imu>("/livox/imu_bias_corrected",50);
    status_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/imu/gyro_bias/status",10);
    imu_=create_subscription<sensor_msgs::msg::Imu>("/livox/imu",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message){on_imu(*message);});
    wheel_=create_subscription<nav_msgs::msg::Odometry>("/rear_axle/wheel_odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message){on_wheel(*message);});
    lio_=create_subscription<nav_msgs::msg::Odometry>("/fastlio2/lio_odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message){on_lio(*message);});
    timer_=rclcpp::create_timer(this,get_clock(),std::chrono::seconds(1),[this](){diagnostics();});
    RCLCPP_INFO(get_logger(),"Gyro calibration requires a stationary car for %.1f s; FAST-LIO keeps raw IMU input.",
      calibrator_.parameters().duration);
  }
private:
  GyroBiasParameters declare_calibration_parameters()
  {
    GyroBiasParameters p;
    p.duration=declare_parameter<double>("duration",p.duration);
    p.min_samples=declare_parameter<std::int64_t>("min_samples",p.min_samples);
    p.max_observation_age=declare_parameter<double>("max_observation_age",p.max_observation_age);
    p.max_imu_gap=declare_parameter<double>("max_imu_gap",p.max_imu_gap);
    p.max_wheel_speed=declare_parameter<double>("max_wheel_speed",p.max_wheel_speed);
    p.max_lio_speed=declare_parameter<double>("max_lio_speed",p.max_lio_speed);
    p.max_position_change=declare_parameter<double>("max_position_change",p.max_position_change);
    p.max_orientation_change=declare_parameter<double>("max_orientation_change",p.max_orientation_change);
    p.max_gyro_norm=declare_parameter<double>("max_gyro_norm",p.max_gyro_norm);
    p.max_gyro_std=declare_parameter<double>("max_gyro_std",p.max_gyro_std);
    p.max_accel_std=declare_parameter<double>("max_accel_std",p.max_accel_std);
    p.accel_norm_min=declare_parameter<double>("accel_norm_min",p.accel_norm_min);
    p.accel_norm_max=declare_parameter<double>("accel_norm_max",p.accel_norm_max);
    return p;
  }
  void on_wheel(const nav_msgs::msg::Odometry & message)
  {
    if (message.header.frame_id!="odom" || message.child_frame_id!="base_link") {return;}
    try {
      calibrator_.add_wheel(seconds(message.header.stamp),message.twist.twist.linear.x);
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"%s",error.what());
    }
  }
  void on_lio(const nav_msgs::msg::Odometry & message)
  {
    if (message.header.frame_id!="odom" || message.child_frame_id!="livox_frame") {return;}
    const auto & p=message.pose.pose.position;
    const auto & q=message.pose.pose.orientation;
    const auto & v=message.twist.twist.linear;
    try {
      calibrator_.add_lio(seconds(message.header.stamp),{p.x,p.y,p.z},
        Eigen::Quaterniond(q.w,q.x,q.y,q.z),{v.x,v.y,v.z});
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"%s",error.what());
    }
  }
  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    if (message.header.frame_id!="livox_frame" || message.angular_velocity_covariance[0]==-1.) {return;}
    const auto & w=message.angular_velocity;
    const auto & a=message.linear_acceleration;
    const bool was_ready=calibrator_.ready();
    const auto corrected=calibrator_.process(seconds(message.header.stamp),{w.x,w.y,w.z},{a.x,a.y,a.z});
    if (corrected) {output_->publish(correct_imu(message,calibrator_.bias()));}
    if (calibrator_.ready() && !was_ready) {
      RCLCPP_INFO(get_logger(),"Gyro bias calibrated in livox_frame [rad/s]: %s",json_vector(calibrator_.bias()).c_str());
      diagnostics();
    }
  }
  void diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticStatus item;
    item.name="livox_gyro_bias";item.hardware_id="livox_frame";
    item.level=calibrator_.ready() ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    item.message=calibrator_.reason();
    auto add=[&item](const std::string & key,const std::string & value) {
      diagnostic_msgs::msg::KeyValue field;field.key=key;field.value=value;
      item.values.push_back(std::move(field));
    };
    add("ready",calibrator_.ready() ? "true" : "false");
    add("gyro_bias_radps",json_vector(calibrator_.bias()));
    add("samples",std::to_string(calibrator_.sample_count()));
    add("gyro_std_radps",json_vector(calibrator_.gyro_std()));
    add("frame_id","\"livox_frame\"");
    std::ostringstream duration;duration<<std::setprecision(17)<<calibrator_.parameters().duration;
    add("duration_s",duration.str());
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp=get_clock()->now();message.status.push_back(std::move(item));
    status_->publish(message);
  }
  GyroBiasCalibrator calibrator_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_,lio_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace aims_racer_system

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<aims_racer_system::LivoxGyroBias>());
  rclcpp::shutdown();
  return 0;
}
