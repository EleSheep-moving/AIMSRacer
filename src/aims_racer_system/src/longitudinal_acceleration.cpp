// Independent causal scalar acceleration observer. Inputs are body wheel speed
// and body specific force only; no orientation, gravity, LIO or command input.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <iomanip>
#include <map>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "aims_racer_system/longitudinal_observer.hpp"
namespace aims_racer_system {
class LongitudinalAccelerationNode : public rclcpp::Node {
 public:
  LongitudinalAccelerationNode() : Node("longitudinal_acceleration") {
    const auto wheel_topic = declare_parameter<std::string>("wheel_topic", "/rear_axle/wheel_odom");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/rear_axle/imu");
    const auto output_topic = declare_parameter<std::string>(
        "output_topic", "/state_estimation/longitudinal_acceleration");
    const auto status_topic = declare_parameter<std::string>(
        "status_topic", "/state_estimation/longitudinal_acceleration/status");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    config_.use_imu = true;
    config_.use_gravity = false;
    config_.jerk_psd = declare_parameter<double>("jerk_psd", 30.);
    config_.disturbance_psd = declare_parameter<double>("disturbance_psd", .1);
    config_.speed_variance = declare_parameter<double>("speed_variance", .01);
    config_.force_variance = declare_parameter<double>("force_variance", 16.);
    config_.history_seconds = declare_parameter<double>("history_seconds", 2.);
    config_.max_history_measurements = declare_parameter<int64_t>("max_history_measurements", 4096);
    warning_age_["wheel"] = declare_parameter<double>("wheel_warning_age_s", .2);
    warning_age_["imu"] = declare_parameter<double>("imu_warning_age_s", .05);
    for (const auto& entry : warning_age_)
      if (!std::isfinite(entry.second) || entry.second < 0.) {
        throw std::invalid_argument("source warning ages must be finite and nonnegative");
      }
    for (const auto* name :
         {"wheel_callbacks", "imu_callbacks", "wheel_wrong_frame", "imu_wrong_frame",
          "wheel_nonfinite", "imu_nonfinite", "force_unavailable", "invalid_stamp", "future_stamp",
          "clock_resets", "snapshot_failure", "receive_failure"})
      input_counters_[name] = 0;
    reset_state();
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    acceleration_publisher_ =
        create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(output_topic, 10);
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic, 10);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        wheel_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_wheel(*message); }, options);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { on_imu(*message); }, options);
    status_timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::seconds(1), [this]() { publish_status(); },
        callback_group_);
  }

 private:
  // RAII records the full callback duration on all early-return paths. Steady
  // time measures execution cost while source/receipt stamps use the ROS clock.
  struct CallbackTiming {
    explicit CallbackTiming(std::deque<double>& destination)
        : destination(destination), started(std::chrono::steady_clock::now()) {}
    ~CallbackTiming() {
      const auto elapsed = std::chrono::steady_clock::now() - started;
      if (destination.size() == 512)
        destination.pop_front();
      destination.push_back(std::chrono::duration<double, std::milli>(elapsed).count());
    }
    std::deque<double>& destination;
    std::chrono::steady_clock::time_point started;
  };
  bool initialized() const {
    return have_wheel_ && have_force_;
  }
  void reset_state() {
    observer_ = std::make_unique<LongitudinalObserver>(config_);
    have_wheel_ = have_force_ = force_valid_ = false;
    source_stamps_["wheel"] = std::nullopt;
    source_stamps_["imu"] = std::nullopt;
    last_force_source_ns_.reset();
    last_estimate_.reset();
  }
  void observe_clock(int64_t now_ns) {
    if (last_clock_ns_ && now_ns < *last_clock_ns_) {
      reset_state();
      ++input_counters_["clock_resets"];
      RCLCPP_WARN(get_logger(), "ROS clock reversed; longitudinal observer reset");
    }
    last_clock_ns_ = now_ns;
  }
  std::optional<int64_t> source_ns(const builtin_interfaces::msg::Time& stamp, int64_t receipt_ns) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {
      ++input_counters_["invalid_stamp"];
      return std::nullopt;
    }
    const int64_t value = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    if (value > receipt_ns) {
      ++input_counters_["future_stamp"];
      return std::nullopt;
    }
    return value;
  }
  void remember_source(const std::string& kind, int64_t stamp) {
    auto& previous = source_stamps_.at(kind);
    if (!previous || stamp > *previous)
      previous = stamp;
  }
  bool admit(const std::string& kind, int64_t stamp, double value) {
    try {
      return observer_->receive(kind, stamp, value);
    } catch (const std::exception&) {
      ++input_counters_["receive_failure"];
      return false;
    }
  }
  void on_wheel(const nav_msgs::msg::Odometry& message) {
    CallbackTiming timing(wheel_timing_ms_);
    ++input_counters_["wheel_callbacks"];
    const auto receipt_ns = get_clock()->now().nanoseconds();
    observe_clock(receipt_ns);
    if (message.child_frame_id != base_frame_id_) {
      ++input_counters_["wheel_wrong_frame"];
      return;
    }
    const auto stamp = source_ns(message.header.stamp, receipt_ns);
    if (!stamp)
      return;
    const double value = message.twist.twist.linear.x;
    if (!std::isfinite(value)) {
      ++input_counters_["wheel_nonfinite"];
      return;
    }
    if (admit("speed", *stamp, value)) {
      have_wheel_ = true;
      remember_source("wheel", *stamp);
    }
  }
  void on_imu(const sensor_msgs::msg::Imu& message) {
    CallbackTiming timing(imu_timing_ms_);
    ++input_counters_["imu_callbacks"];
    const auto receipt_ns = get_clock()->now().nanoseconds();
    observe_clock(receipt_ns);
    force_valid_ = false;
    if (message.header.frame_id != base_frame_id_) {
      ++input_counters_["imu_wrong_frame"];
      return;
    }
    const auto stamp = source_ns(message.header.stamp, receipt_ns);
    if (!stamp)
      return;
    remember_source("imu", *stamp);
    if (message.linear_acceleration_covariance[0] < 0.) {
      ++input_counters_["force_unavailable"];
      publish_estimate();
      return;
    }
    const double value = message.linear_acceleration.x;
    if (!std::isfinite(value) || !std::isfinite(message.linear_acceleration_covariance[0])) {
      ++input_counters_["imu_nonfinite"];
      return;
    }
    if (admit("force", *stamp, value)) {
      have_force_ = force_valid_ = true;
      if (!last_force_source_ns_ || *stamp > *last_force_source_ns_)
        last_force_source_ns_ = *stamp;
    }
    publish_estimate();
  }
  void publish_estimate() {
    if (!initialized())
      return;
    const auto now = get_clock()->now();
    const auto now_ns = now.nanoseconds();
    observe_clock(now_ns);
    if (!initialized())
      return;
    std::optional<Estimate> estimate;
    try {
      estimate = observer_->snapshot(now_ns);
    } catch (const std::exception&) {
      ++input_counters_["snapshot_failure"];
      return;
    }
    geometry_msgs::msg::AccelWithCovarianceStamped message;
    message.header.stamp = now;
    message.header.frame_id = base_frame_id_;
    message.accel.accel.linear.x = estimate->acceleration;
    for (const size_t index : {7u, 14u, 21u, 28u, 35u})
      message.accel.covariance[index] = 1e6;
    message.accel.covariance[0] = estimate->acceleration_variance;
    last_estimate_ = estimate;
    acceleration_publisher_->publish(message);
  }
  template <typename T>
  static std::string as_string(T value) {
    std::ostringstream output;
    output << std::setprecision(17) << value;
    return output.str();
  }
  static double percentile(std::vector<double> sorted, double fraction) {
    if (sorted.empty())
      return 0.;
    std::sort(sorted.begin(), sorted.end());
    const double index = (sorted.size() - 1) * fraction;
    const size_t lower = static_cast<size_t>(index), upper = std::min(lower + 1, sorted.size() - 1);
    return sorted[lower] + (sorted[upper] - sorted[lower]) * (index - lower);
  }
  void publish_status() {
    const auto now = get_clock()->now();
    const auto now_ns = now.nanoseconds();
    observe_clock(now_ns);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "longitudinal_acceleration";
    status.hardware_id = "scalar_observer";
    std::vector<std::string> warnings;
    if (!initialized())
      warnings.push_back("waiting for wheel and valid force");
    if (!force_valid_)
      warnings.push_back("force unavailable or invalid");
    auto add = [&status](const std::string& key, const auto& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = as_string(value);
      status.values.push_back(std::move(item));
    };
    add("initialized", initialized() ? "true" : "false");
    add("force_valid", force_valid_ ? "true" : "false");
    add("history_size", observer_->history_size());
    for (const auto* kind : {"wheel", "imu"}) {
      const auto source = source_stamps_.at(kind);
      const std::string key(kind);
      add(key + "_source_stamp_ns", source ? as_string(*source) : "none");
      add(key + "_age_s",
          source ? as_string(static_cast<double>(now_ns - *source) * 1e-9) : "none");
      if (source && static_cast<double>(now_ns - *source) * 1e-9 > warning_age_.at(kind))
        warnings.push_back(key + " source stale");
    }
    add("force_source_stamp_ns",
        last_force_source_ns_ ? as_string(*last_force_source_ns_) : "none");
    add("force_age_s", last_force_source_ns_
                           ? as_string(static_cast<double>(now_ns - *last_force_source_ns_) * 1e-9)
                           : "none");
    if (last_force_source_ns_ &&
        static_cast<double>(now_ns - *last_force_source_ns_) * 1e-9 > warning_age_.at("imu")) {
      warnings.push_back("valid force source stale");
    }
    if (last_estimate_) {
      add("estimate.velocity", last_estimate_->velocity);
      add("estimate.acceleration", last_estimate_->acceleration);
      add("estimate.disturbance", *last_estimate_->disturbance);
      add("estimate.acceleration_variance", last_estimate_->acceleration_variance);
      add("estimate.stamp_ns", last_estimate_->stamp_ns);
    }
    for (const auto& entry : observer_->counters())
      add("observer." + entry.first, entry.second);
    for (const auto& entry : input_counters_)
      add("input." + entry.first, entry.second);
    if (observer_->counters().at("numerical_failure") || input_counters_.at("snapshot_failure") ||
        input_counters_.at("receive_failure")) {
      warnings.push_back("numerical failure recorded");
    }
    auto add_timing = [&add](const std::string& kind, const std::deque<double>& samples) {
      const std::vector<double> values(samples.begin(), samples.end());
      add("timing." + kind + ".p50_ms", percentile(values, .5));
      add("timing." + kind + ".p95_ms", percentile(values, .95));
      add("timing." + kind + ".max_ms",
          values.empty() ? 0. : *std::max_element(values.begin(), values.end()));
      add("timing." + kind + ".samples", samples.size());
    };
    add_timing("wheel", wheel_timing_ms_);
    add_timing("imu", imu_timing_ms_);
    status.level = warnings.empty() ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                    : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    if (warnings.empty())
      status.message = "observer initialized";
    else
      for (const auto& warning : warnings) {
        if (!status.message.empty())
          status.message += "; ";
        status.message += warning;
      }
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now;
    message.header.frame_id = base_frame_id_;
    message.status.push_back(std::move(status));
    status_publisher_->publish(message);
  }
  ObserverConfig config_;
  std::unique_ptr<LongitudinalObserver> observer_;
  std::string base_frame_id_;
  bool have_wheel_{false}, have_force_{false}, force_valid_{false};
  std::optional<int64_t> last_clock_ns_, last_force_source_ns_;
  std::optional<Estimate> last_estimate_;
  std::map<std::string, std::optional<int64_t>> source_stamps_;
  std::map<std::string, double> warning_age_;
  std::map<std::string, uint64_t> input_counters_;
  std::deque<double> wheel_timing_ms_, imu_timing_ms_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
      acceleration_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace aims_racer_system
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aims_racer_system::LongitudinalAccelerationNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
