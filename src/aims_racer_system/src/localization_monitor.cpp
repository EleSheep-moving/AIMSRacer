#include "aims_racer_system/localization_health.hpp"
#include "aims_racer_system/localization_alignment.hpp"
#include "aims_racer_system/localization_quality.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace aims_racer_system
{
namespace
{
using namespace std::chrono_literals;
double monotonic_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
double age_seconds(std::int64_t now, std::int64_t stamp)
{
  return static_cast<double>((static_cast<long double>(now) - stamp) * 1e-9L);
}
template<class Position, class Quaternion>
Pose pose(const Position & p, const Quaternion & q)
{
  return checked_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w);
}
}  // namespace

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor()
  : Node("localization_monitor"),
    map_(declare_parameter<std::string>("map_file", "")),
    ekf_max_age_(declare_parameter<double>("ekf_max_age_sec", .1)),
    cloud_max_age_(declare_parameter<double>("cloud_max_age_sec", .5)),
    health_(declare_parameter<double>("anchor_max_age_sec", 1.),
      declare_parameter<int>("recovery_commits", 3))
  {
    if (!std::isfinite(ekf_max_age_) || ekf_max_age_ <= 0 ||
      !std::isfinite(cloud_max_age_) || cloud_max_age_ <= 0)
    {
      throw std::invalid_argument("positive finite input age limits required");
    }
    auto latched = rclcpp::QoS(1).transient_local();
    sha_pub_ = create_publisher<std_msgs::msg::String>("/localization/map_sha256", latched);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/localization/map_valid", latched);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/localization/status", 10);
    std_msgs::msg::String sha;
    sha.data = map_.sha256();
    sha_pub_->publish(sha);
    // Health needs the latest state. Pose history is retained separately for scan diagnostics.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",
      rclcpp::QoS(2), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odometry(*msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/fastlio2/body_cloud",
      rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (msg->header.frame_id == "livox_frame" && remember_input("body_cloud", msg->header.stamp)) {
          cloud_ = msg;
        }
      });
    anchor_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/localization/anchor_status", rclcpp::QoS(100).transient_local(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {anchor(*msg);});
    static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
      rclcpp::QoS(100).transient_local(), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & t : msg->transforms) {
          if (t.header.frame_id == "base_link" && t.child_frame_id == "livox_frame") {
            try {mount_ = pose(t.transform.translation, t.transform.rotation);}
            catch (const std::exception &) {mount_.reset();}
          }
        }
      });
    timer_ = create_wall_timer(100ms, [this]() {tick();});
    quality_timer_ = create_wall_timer(500ms, [this]() {launch_quality();});
    RCLCPP_INFO(get_logger(), "C++ localization monitor: EKF queue=2, cloud queue=1; map=%s",
      map_.sha256().c_str());
  }

  ~LocalizationMonitor() override
  {
    // The worker references map_; join before member destruction, including on shutdown.
    if (future_.valid()) {future_.wait();}
  }

private:
  struct Input {std::int64_t stamp; double received;};

  std::int64_t synchronize_clock()
  {
    const auto now = get_clock()->now().nanoseconds();
    if (last_clock_ && now < *last_clock_) {
      health_.rewind();
      history_.clear();
      inputs_.clear();
      packets_.clear();
      alignment_.clear();
      cloud_.reset();
      ++generation_;
      quality_.reset();
      last_quality_stamp_.reset();
      quality_error_ = "clock_rewind";
    }
    last_clock_ = now;
    return now;
  }

  bool remember_input(const std::string & key, const builtin_interfaces::msg::Time & stamp)
  {
    const auto now = synchronize_clock();
    const auto source = stamp_ns(stamp);
    auto previous = inputs_.find(key);
    if (stamp.nanosec >= 1000000000U || source < 0 || source > now ||
      (previous != inputs_.end() && source < previous->second.stamp))
    {
      inputs_.erase(key);
      health_.invalidate("invalid_" + key + "_stamp");
      return false;
    }
    if (previous != inputs_.end() && source == previous->second.stamp) {return false;}
    inputs_[key] = {source, monotonic_now()};
    if (key == "ekf") {
      ekf_callback_age_ = age_seconds(now, source);
      ekf_max_callback_age_ = std::max(ekf_max_callback_age_, ekf_callback_age_);
      ++ekf_callbacks_;
      if (ekf_callback_age_ > ekf_max_age_) {++ekf_stale_callbacks_;}
    }
    return true;
  }

  void odometry(const nav_msgs::msg::Odometry & message)
  {
    if (message.header.frame_id != "odom" || message.child_frame_id != "base_link") {return;}
    try {
      const auto source_pose = pose(message.pose.pose.position, message.pose.pose.orientation);
      if (remember_input("ekf", message.header.stamp)) {
        const auto source = stamp_ns(message.header.stamp);
        history_.emplace_back(source, source_pose);
        while (!history_.empty() && source - history_.front().first > 5000000000LL) {
          history_.pop_front();
        }
      }
    } catch (const std::invalid_argument &) {return;}
  }

  void anchor(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    const auto now = synchronize_clock();
    for (const auto & status : message.status) {
      if (status.name != "lidar_localization/anchor") {continue;}
      HealthFields values;
      for (const auto & item : status.values) {values[item.key] = item.value;}
      const auto previous_epoch = health_.epoch();
      const bool accepted = health_.observe(values, stamp_ns(message.header.stamp), now, monotonic_now());
      if (health_.epoch() != previous_epoch) {
        packets_.clear();
        alignment_.clear();
        ++generation_;
        quality_.reset();
        last_quality_stamp_.reset();
        quality_error_ = "waiting_for_scan_and_mount";
      }
      if (!alignment_.observe(values, accepted, health_.epoch(), health_.stamp_ns())) {
        health_.invalidate("invalid_alignment_snapshot");
        quality_error_ = "missing_or_invalid_commit_snapshot";
      }
      if (accepted && values.at("anchor_committed") == "true") {
        try {
          auto component = [&values](const char * key) {
              const auto & text = values.at(std::string("map_odom_") + key);
              std::size_t consumed = 0;
              const double result = std::stod(text, &consumed);
              if (consumed != text.size()) {throw std::invalid_argument("invalid pose component");}
              return result;
            };
          packets_.add(static_cast<std::int64_t>(health_.stamp_ns()),
            checked_pose(component("x"), component("y"), component("z"), component("qx"),
            component("qy"), component("qz"), component("qw")));
        } catch (const std::exception &) {
          quality_error_ = "missing_or_invalid_commit_snapshot";
        }
      }
      publish_status(now);
    }
  }

  void launch_quality()
  {
    if (future_.valid() || !cloud_) {return;}
    const auto source = stamp_ns(cloud_->header.stamp);
    if (last_quality_stamp_ && source == *last_quality_stamp_) {return;}
    const auto source_pose = interpolate_pose(history_, source);
    const auto correction = packets_.held(source);
    if (!mount_) {quality_error_ = "quality_mount_unavailable"; return;}
    if (!source_pose) {quality_error_ = "quality_source_pose_unavailable"; return;}
    if (!correction) {quality_error_ = "quality_correction_unavailable"; return;}
    last_quality_stamp_ = source;
    worker_generation_ = generation_;
    const Pose transform = correction->second * *source_pose * *mount_;
    // Immutable snapshots: no health state or executor locks are shared with this worker.
    future_ = std::async(std::launch::async,
      [this, cloud = cloud_, transform, stamp = correction->first, generation = generation_]() {
        return map_.check(*cloud, transform, stamp, generation);
      });
  }

  void tick()
  {
    const auto now = synchronize_clock();
    if (future_.valid() && future_.wait_for(0ms) == std::future_status::ready) {
      try {
        auto result = future_.get();
        if (result.generation == generation_) {
          quality_ = result;
          quality_error_.clear();
        }
      } catch (const std::exception & error) {
        if (worker_generation_ == generation_) {quality_error_ = error.what();}
      }
    }
    publish_status(now);
  }

  void publish_status(std::int64_t now)
  {
    const auto mono = monotonic_now();
    bool present = true;
    HealthFields ages;
    for (const auto & item : {std::make_pair("ekf", ekf_max_age_),
      std::make_pair("body_cloud", cloud_max_age_)})
    {
      const auto input = inputs_.find(item.first);
      const auto age = input == inputs_.end() ? std::numeric_limits<double>::infinity() :
        age_seconds(now, input->second.stamp);
      const auto wall_age = input == inputs_.end() ? std::numeric_limits<double>::infinity() :
        mono - input->second.received;
      ages[std::string(item.first) + "_age_sec"] = number(age);
      ages[std::string(item.first) + "_receive_age_sec"] = number(wall_age);
      present = present && age >= 0 && age <= item.second && wall_age >= 0 && wall_age <= item.second;
    }
    auto values = health_.evaluate(now, mono, present);
    const auto alignment_fields = alignment_.fields(values);
    values.insert(alignment_fields.begin(), alignment_fields.end());
    if (values.at("alignment_valid") != "true") {
      values["ready"] = "false";
      values["state"] = "lost";
    }
    const bool valid = values.at("ready") == "true";
    if (health_.epoch() != publish_epoch_) {
      publish_epoch_ = health_.epoch();
      publish_sequence_ = 0;
    }
    values["health_sequence"] = std::to_string(++publish_sequence_);
    values["map_sha256"] = map_.sha256();
    values["map_valid"] = valid ? "true" : "false";
    values["quality_error"] = quality_error_;
    values["implementation"] = "cpp";
    values["ekf_callback_age_sec"] = number(ekf_callback_age_);
    values["ekf_max_callback_age_sec"] = number(ekf_max_callback_age_);
    values["ekf_callback_count"] = std::to_string(ekf_callbacks_);
    values["ekf_stale_callback_count"] = std::to_string(ekf_stale_callbacks_);
    values["quality_worker_busy"] = future_.valid() ? "true" : "false";
    values.insert(ages.begin(), ages.end());
    if (quality_) {
      values["quality_points"] = std::to_string(quality_->points);
      values["inlier_fraction"] = number(quality_->inlier_fraction);
      values["inlier_rmse_m"] = number(quality_->rmse);
      values["quality_time_ms"] = number(quality_->time_ms);
      values["quality_stamp_ns"] = std::to_string(quality_->stamp);
      values["quality_correction_stamp_ns"] = std::to_string(quality_->correction_stamp);
    }
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "aims_racer_system/localization";
    status.message = values.at("state");
    status.level = valid ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    for (const auto & field : values) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = field.first;
      item.value = field.second;
      status.values.push_back(std::move(item));
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = get_clock()->now();
    array.status.push_back(std::move(status));
    status_pub_->publish(array);
    std_msgs::msg::Bool validity;
    validity.data = valid;
    valid_pub_->publish(validity);
  }

  MapQuality map_;
  double ekf_max_age_, cloud_max_age_;
  AnchorHealth health_;
  CommittedAlignment alignment_;
  std::map<std::string, Input> inputs_;
  PoseHistory history_;
  MapOdomPackets packets_;
  std::optional<Pose> mount_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
  std::optional<std::int64_t> last_clock_, last_quality_stamp_;
  std::uint64_t generation_{0}, worker_generation_{0}, publish_sequence_{0};
  std::string publish_epoch_, quality_error_{"waiting_for_scan_and_mount"};
  std::optional<QualityResult> quality_;
  std::future<QualityResult> future_;
  double ekf_callback_age_{std::numeric_limits<double>::infinity()}, ekf_max_callback_age_{0};
  std::uint64_t ekf_callbacks_{0}, ekf_stale_callbacks_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sha_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr anchor_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  rclcpp::TimerBase::SharedPtr timer_, quality_timer_;
};
}  // namespace aims_racer_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<aims_racer_system::LocalizationMonitor>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_monitor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
