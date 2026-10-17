// Independent source-time body acceleration: corrected raw IMU + LIO orientation.
#include "aims_racer_system/body_acceleration.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace ba = aims_racer_system::body_acceleration;
class BodyAccelerationNode final : public rclcpp::Node {
 public:
  BodyAccelerationNode() : Node("body_acceleration") {
    config_.gravity = declare_parameter("gravity", config_.gravity);
    config_.alpha_window_s = declare_parameter("alpha_window_s", config_.alpha_window_s);
    config_.max_gyro_gap_s = declare_parameter("max_gyro_gap_s", config_.max_gyro_gap_s);
    config_.max_lio_age_s = declare_parameter("max_lio_age_s", config_.max_lio_age_s);
    config_.history_s = declare_parameter("history_s", config_.history_s);
    const auto max_events = declare_parameter<int64_t>("max_events", config_.max_events);
    if (max_events < 8) throw std::invalid_argument("max_events must be at least 8");
    config_.max_events = static_cast<std::size_t>(max_events);
    config_.filter_enabled = declare_parameter("filter_enabled", config_.filter_enabled);
    config_.lateral_enabled = declare_parameter("lateral_enabled", false);
    config_.jerk_psd = declare_parameter("jerk_psd", config_.jerk_psd);
    config_.force_variance = declare_parameter("force_variance", config_.force_variance);
    config_.gyro_variance = declare_parameter("gyro_variance", config_.gyro_variance);
    config_.tilt_std_rad = declare_parameter("tilt_std_rad", config_.tilt_std_rad);
    config_.lever = vector_parameter("livox_translation", {.30, 0., .03});
    config_.accel_bias_body = vector_parameter("accel_bias_body", {0., 0., 0.});
    const auto q = declare_parameter<std::vector<double>>("livox_quaternion", {0., 0., 0., 1.});
    if (q.size() != 4) throw std::invalid_argument("livox_quaternion requires xyzw");
    config_.mounting = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
    if (!config_.mounting.coeffs().allFinite() || std::abs(config_.mounting.norm() - 1.) > 1e-3)
      throw std::invalid_argument("livox_quaternion must be finite and unit length");
    config_.mounting.normalize();
    imu_frame_ = declare_parameter<std::string>("imu_frame", "livox_frame");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    imu_max_age_s_ = declare_parameter("imu_max_age_s", .05);
    if (!std::isfinite(imu_max_age_s_) || imu_max_age_s_ <= 0.)
      throw std::invalid_argument("imu_max_age_s must be positive and finite");
    extrinsics_calibrated_ = declare_parameter("extrinsics_calibrated", false);
    accelerometer_calibrated_ = declare_parameter("accelerometer_calibrated", false);
    const auto output = declare_parameter<std::string>("output_topic", "/state_estimation/body_acceleration");
    observer_ = std::make_unique<ba::Observer>(config_);
    for (const auto * key : {"imu_callbacks", "lio_callbacks", "imu_wrong_frame", "lio_wrong_frame",
        "imu_unavailable", "imu_nonfinite", "lio_nonfinite", "invalid_stamp", "future_stamp", "clock_resets",
        "numerical_output_invalid"})
      inputs_[key] = 0;
    publisher_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(output, 10);
    if(config_.lateral_enabled)
      g_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(output + "/g", 10);
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(output + "/status", 10);
    if (declare_parameter("publish_debug", true))
      debug_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(output + "/debug", 10);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        declare_parameter<std::string>("imu_topic", "/livox/imu_bias_corrected"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(*msg); });
    lio_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("lio_topic", "/rear_axle/lio_odom"), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_lio(*msg); });
    // Wall timer preserves dropout diagnostics even when /clock pauses.
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { publish_status(); });
  }
 private:
  struct Timing {
    explicit Timing(std::deque<double> & values) : values(values), start(std::chrono::steady_clock::now()) {}
    ~Timing() {
      if (values.size() == 512) values.pop_front();
      values.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
    }
    std::deque<double> & values;
    std::chrono::steady_clock::time_point start;
  };
  Eigen::Vector3d vector_parameter(const std::string & name, const std::vector<double> & defaults) {
    const auto v = declare_parameter<std::vector<double>>(name, defaults);
    if (v.size() != 3) throw std::invalid_argument(name + " requires three values");
    Eigen::Vector3d result(v[0], v[1], v[2]);
    if (!result.allFinite()) throw std::invalid_argument(name + " must be finite");
    return result;
  }
  int64_t observe_clock() {
    const int64_t stamp = now().nanoseconds();
    if (last_clock_ && stamp < *last_clock_) {
      observer_->reset();
      imu_source_.reset(); lio_source_.reset(); imu_receipt_.reset(); lio_receipt_.reset();
      published_stamp_.reset(); numerical_rejected_ = false;
      ++inputs_["clock_resets"];
    }
    last_clock_ = stamp;
    return stamp;
  }
  std::optional<int64_t> source_stamp(const builtin_interfaces::msg::Time & stamp, int64_t receipt) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {
      ++inputs_["invalid_stamp"]; return std::nullopt;
    }
    const int64_t result = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    if (result <= 0) { ++inputs_["invalid_stamp"]; return std::nullopt; }
    if (result > receipt) { ++inputs_["future_stamp"]; return std::nullopt; }
    return result;
  }
  static void remember(std::optional<int64_t> & previous, int64_t stamp) {
    if (!previous || stamp > *previous) previous = stamp;
  }
  static double age(int64_t current, std::optional<int64_t> stamp) {
    return stamp ? static_cast<double>(current - *stamp) * 1e-9 : -1.;
  }
  static bool finite_estimate(const ba::Estimate & e) {
    return std::isfinite(e.ax) && std::isfinite(e.jerk) &&
        std::isfinite(e.acceleration_variance) && e.acceleration_variance >= 0. &&
        std::isfinite(e.measurement_variance) && e.measurement_variance >= 0. &&
        e.q_world_body.coeffs().allFinite() && e.force_body.allFinite() &&
        e.corrected_force_body.allFinite() && e.gravity_body.allFinite() &&
        e.omega_body.allFinite() && e.alpha_body.allFinite() && e.tangential.allFinite() &&
        e.centripetal.allFinite() && e.acceleration_body.allFinite() &&
        std::isfinite(e.ay) && std::isfinite(e.lateral_jerk) &&
        std::isfinite(e.lateral_acceleration_variance);
  }
  bool valid(const ba::Estimate & estimate, int64_t current) const {
    return !numerical_rejected_ && estimate.valid && finite_estimate(estimate) && imu_source_ && lio_source_ &&
        age(current, imu_source_) >= 0. && age(current, imu_source_) <= imu_max_age_s_ &&
        age(current, lio_source_) >= 0. && age(current, lio_source_) <= config_.max_lio_age_s;
  }
  std::string reason(const ba::Estimate & estimate, int64_t current) const {
    if (numerical_rejected_ || !finite_estimate(estimate)) return "numerical_error";
    if (!estimate.valid) return ba::status_name(estimate.status);
    if (age(current, imu_source_) > imu_max_age_s_) return "imu_stale";
    if (age(current, lio_source_) > config_.max_lio_age_s) return "lio_stale";
    return valid(estimate, current) ? "valid" : "missing_input";
  }
  void on_imu(const sensor_msgs::msg::Imu & msg) {
    Timing timing(imu_cost_);
    ++inputs_["imu_callbacks"];
    const auto receipt = observe_clock();
    auto stamp = source_stamp(msg.header.stamp, receipt);
    if (!stamp) return;
    if (msg.header.frame_id != imu_frame_) { ++inputs_["imu_wrong_frame"]; return; }
    if (msg.linear_acceleration_covariance[0] < 0. || msg.angular_velocity_covariance[0] < 0.) {
      ++inputs_["imu_unavailable"]; return;
    }
    const Eigen::Vector3d force(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    const Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    const auto finite = [](double value) { return std::isfinite(value); };
    if (!force.allFinite() || !gyro.allFinite() ||
        !std::all_of(msg.linear_acceleration_covariance.begin(), msg.linear_acceleration_covariance.end(), finite) ||
        !std::all_of(msg.angular_velocity_covariance.begin(), msg.angular_velocity_covariance.end(), finite)) {
      ++inputs_["imu_nonfinite"]; return;
    }
    const auto invalid_before = observer_->counters().invalid;
    if (!observer_->add_imu({*stamp, force, gyro})) {
      if (observer_->counters().invalid > invalid_before) {
        numerical_rejected_ = true; ++inputs_["numerical_output_invalid"];
      }
      return;
    }
    remember(imu_source_, *stamp); imu_receipt_ = receipt;
    const auto estimate = observer_->latest();
    if (!finite_estimate(estimate)) { numerical_rejected_ = true; ++inputs_["numerical_output_invalid"]; return; }
    if (estimate.valid && estimate.stamp_ns == *stamp) numerical_rejected_ = false;
    // Late IMU admission and LIO replay revise history without retroactive publication.
    if (estimate.stamp_ns != *stamp || (published_stamp_ && *stamp <= *published_stamp_) ||
        !valid(estimate, receipt)) return;
    geometry_msgs::msg::AccelWithCovarianceStamped output;
    output.header.stamp = msg.header.stamp;
    output.header.frame_id = body_frame_;
    output.accel.accel.linear.x = estimate.ax;
    output.accel.covariance[0] = estimate.acceleration_variance;
    for (std::size_t i = 1; i < 6; ++i) output.accel.covariance[i * 7] = 1e6;
    if(config_.lateral_enabled) {
      output.accel.accel.linear.y = estimate.ay;
      output.accel.covariance[7] = estimate.lateral_acceleration_variance;
    }
    publisher_->publish(output);
    if(g_publisher_) {
      geometry_msgs::msg::Vector3Stamped g;g.header=output.header;
      g.vector.x=estimate.ax/9.80665;g.vector.y=estimate.ay/9.80665;
      g_publisher_->publish(g);
    }
    published_stamp_ = *stamp;
  }
  void on_lio(const nav_msgs::msg::Odometry & msg) {
    Timing timing(lio_cost_);
    ++inputs_["lio_callbacks"];
    const auto receipt = observe_clock();
    auto stamp = source_stamp(msg.header.stamp, receipt);
    if (!stamp) return;
    if (msg.header.frame_id != world_frame_ || msg.child_frame_id != body_frame_) {
      ++inputs_["lio_wrong_frame"]; return;
    }
    const auto & o = msg.pose.pose.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.) > 1e-3 ||
        !std::all_of(msg.pose.covariance.begin(), msg.pose.covariance.end(), [](double v) { return std::isfinite(v); })) {
      ++inputs_["lio_nonfinite"]; return;
    }
    q.normalize();
    const auto invalid_before = observer_->counters().invalid;
    if (observer_->add_lio({*stamp, q})) { remember(lio_source_, *stamp); lio_receipt_ = receipt; }
    else if (observer_->counters().invalid > invalid_before) {
      numerical_rejected_ = true; ++inputs_["numerical_output_invalid"];
    }
  }
  template<class T> static void field(diagnostic_msgs::msg::DiagnosticStatus & out, const std::string & key, const T & value) {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    std::ostringstream stream; stream << std::setprecision(17) << value;
    entry.value = stream.str(); out.values.push_back(entry);
  }
  static void vector_fields(diagnostic_msgs::msg::DiagnosticStatus & out, const std::string & key, const Eigen::Vector3d & value) {
    for (int i = 0; i < 3; ++i) field(out, key + "." + std::string(1, "xyz"[i]), value[i]);
  }
  static void costs(diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & name, const std::deque<double> & values) {
    std::vector<double> sorted(values.begin(), values.end());
    std::sort(sorted.begin(), sorted.end());
    for (const auto & p : std::vector<std::pair<std::string, double>>{{"p50_ms", .5}, {"p95_ms", .95}, {"p99_ms", .99}})
      field(status, "callback." + name + "." + p.first, sorted.empty() ? 0. : sorted[static_cast<std::size_t>((sorted.size() - 1) * p.second)]);
    field(status, "callback." + name + ".samples", sorted.size());
  }
  void publish_status() {
    const auto current = observe_clock();
    const auto estimate = observer_->latest();
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now(); message.header.frame_id = body_frame_;
    diagnostic_msgs::msg::DiagnosticStatus status;
    const bool good = valid(estimate, current);
    status.name = get_name(); status.hardware_id = "independent_imu_lio";
    status.level = good ? status.OK : status.WARN;
    status.message = reason(estimate, current);
    field(status, "reason", status.message);
    field(status, "x_valid", good ? "true" : "false");
    field(status,"y_valid",good && config_.lateral_enabled ? "true" : "false");
    for (const auto * key : {"z_valid", "angular_valid"}) field(status, key, "false");
    field(status,"lateral_enabled",config_.lateral_enabled ? "true" : "false");
    field(status,"xy_cross_covariance", "unidentified_diagonal_effective_model");
    field(status,"g_reference", "rear_axle_base_link_g0_9.80665");
    field(status, "extrinsics_calibrated", extrinsics_calibrated_ ? "true" : "false");
    field(status, "accelerometer_calibrated", accelerometer_calibrated_ ? "true" : "false");
    field(status, "covariance", "model_uncalibrated");
    field(status, "imu_age_s", age(current, imu_source_));
    field(status, "lio_age_s", age(current, lio_source_));
    field(status, "latest_received_lio_source_age_s", age(current, lio_source_));
    const double anchor_age = estimate.anchor_stamp_ns > 0 ?
        static_cast<double>(estimate.stamp_ns - estimate.anchor_stamp_ns) * 1e-9 : -1.;
    field(status, "estimate_lio_anchor_age_s", anchor_age);
    field(status, "alpha_window_span_s", estimate.alpha_window_span_s);
    field(status, "alpha_sample_center_age_s", estimate.alpha_sample_center_age_s);
    field(status, "imu_receipt_age_s", age(current, imu_receipt_));
    field(status, "lio_receipt_age_s", age(current, lio_receipt_));
    field(status, "source_stamp_ns", estimate.stamp_ns);
    field(status, "lio_anchor_stamp_ns", estimate.anchor_stamp_ns);
    field(status, "gyro_continuity", imu_source_ && age(current, imu_source_) <= config_.max_gyro_gap_s &&
        estimate.status != ba::Status::gyro_gap && estimate.status != ba::Status::gyro_warmup &&
        estimate.status != ba::Status::no_imu ? "true" : "false");
    for (const auto & entry : inputs_) field(status, "input." + entry.first, entry.second);
    const auto counts = observer_->counters();
#define COUNT(name) field(status, "observer." #name, counts.name)
    COUNT(accepted_imu); COUNT(accepted_lio); COUNT(invalid); COUNT(numerical_error); COUNT(duplicate); COUNT(too_old);
    COUNT(replays); COUNT(replayed_imu); COUNT(gyro_gaps); COUNT(evicted);
#undef COUNT
    field(status, "observer.history_size", observer_->history_size());
    costs(status, "imu", imu_cost_); costs(status, "lio", lio_cost_);
    message.status.push_back(status); status_publisher_->publish(message);
    if (debug_publisher_ && finite_estimate(estimate)) {
      diagnostic_msgs::msg::DiagnosticStatus debug;
      debug.name = std::string(get_name()) + "/debug"; debug.hardware_id = status.hardware_id;
      debug.level = status.level; debug.message = status.message;
      field(debug, "source_stamp_ns", estimate.stamp_ns);
      field(debug, "latest_received_lio_source_age_s", age(current, lio_source_));
      field(debug, "estimate_lio_anchor_age_s", anchor_age);
      field(debug, "alpha_window_span_s", estimate.alpha_window_span_s);
      field(debug, "alpha_sample_center_age_s", estimate.alpha_sample_center_age_s);
      vector_fields(debug, "force_si", estimate.force_body);
      vector_fields(debug, "force_bias_corrected", estimate.corrected_force_body);
      vector_fields(debug, "gravity_body", estimate.gravity_body);
      vector_fields(debug, "omega_body", estimate.omega_body);
      vector_fields(debug, "alpha_body", estimate.alpha_body);
      vector_fields(debug, "tangential", estimate.tangential);
      vector_fields(debug, "centripetal", estimate.centripetal);
      field(debug, "reconstructed_ax", estimate.acceleration_body.x());
      field(debug, "filtered_ax", estimate.ax);
      field(debug, "jerk", estimate.jerk);
      field(debug, "measurement_variance", estimate.measurement_variance);
      field(debug, "acceleration_variance", estimate.acceleration_variance);
      if(config_.lateral_enabled) {
        field(debug,"reconstructed_ay",estimate.acceleration_body.y());
        field(debug,"filtered_ay",estimate.ay);field(debug,"lateral_jerk",estimate.lateral_jerk);
        field(debug,"longitudinal_g",estimate.ax/9.80665);field(debug,"lateral_g",estimate.ay/9.80665);
        field(debug,"lateral_acceleration_variance",estimate.lateral_acceleration_variance);
      }
      message.status = {debug}; debug_publisher_->publish(message);
    }
  }
  ba::Config config_;
  std::unique_ptr<ba::Observer> observer_;
  std::string imu_frame_, body_frame_, world_frame_;
  double imu_max_age_s_;
  bool extrinsics_calibrated_, accelerometer_calibrated_;
  bool numerical_rejected_ = false;
  std::optional<int64_t> imu_source_, lio_source_, imu_receipt_, lio_receipt_, last_clock_, published_stamp_;
  std::map<std::string, uint64_t> inputs_;
  std::deque<double> imu_cost_, lio_cost_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr g_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_, debug_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<BodyAccelerationNode>()); }
  catch (const std::exception & error) { RCLCPP_FATAL(rclcpp::get_logger("body_acceleration"), "%s", error.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
