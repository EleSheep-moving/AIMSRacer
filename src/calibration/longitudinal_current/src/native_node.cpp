#include "aims_calibration/native_engine.hpp"
#include "aims_calibration/native_runtime.hpp"
#include "aims_pp/runtime.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <crsf_receiver_msg/msg/crsf_channels16.hpp>
#include <cstring>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <iomanip>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <openssl/evp.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sys/wait.h>
#include <unistd.h>
#include <vesc_msgs/msg/vesc_command.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
namespace aims_calibration {
using VescCommand = vesc_msgs::msg::VescCommand;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using GetParameters = rcl_interfaces::srv::GetParameters;
static double monotonic() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
static double
number(const json &j, const std::string &k,
       double fallback = std::numeric_limits<double>::quiet_NaN()) {
  auto it = j.find(k);
  return it != j.end() && it->is_number() ? it->get<double>() : fallback;
}
static double seconds(const builtin_interfaces::msg::Time &s) {
  return s.sec + s.nanosec * 1e-9;
}
static int64_t nanoseconds(const builtin_interfaces::msg::Time &s) {
  return int64_t(s.sec) * 1000000000LL + s.nanosec;
}
static json finite(double x) { return std::isfinite(x) ? json(x) : json(); }
static std::string text(const json &v) {
  return v.is_string() ? v.get<std::string>() : v.dump();
}
static std::string sha256_file(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("file unavailable: " + path);
  auto context = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>(
      EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256 initialization failed");
  char buffer[65536];
  while (in) {
    in.read(buffer, sizeof(buffer));
    if (in.gcount() &&
        EVP_DigestUpdate(context.get(), buffer, in.gcount()) != 1)
      throw std::runtime_error("SHA256 update failed");
  }
  if (!in.eof())
    throw std::runtime_error("file read failed: " + path);
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned length = 0;
  if (EVP_DigestFinal_ex(context.get(), digest, &length) != 1)
    throw std::runtime_error("SHA256 completion failed");
  std::ostringstream out;
  for (unsigned i = 0; i < length; ++i)
    out << std::hex << std::setfill('0') << std::setw(2) << int(digest[i]);
  return out.str();
}
static json prepare(int argc, char **argv) {
  char path[] = "/tmp/aims-calibration-prepared-XXXXXX";
  int fd = mkstemp(path);
  if (fd < 0)
    throw std::runtime_error("startup descriptor creation failed");
  close(fd);
  pid_t pid = fork();
  if (pid < 0) {
    unlink(path);
    throw std::runtime_error("startup helper fork failed");
  }
  if (pid == 0) {
    std::vector<std::string> args = {"python3", "-m",
                                     "aims_calibration.native_prepare", path};
    for (int i = 1; i < argc; ++i)
      args.emplace_back(argv[i]);
    std::vector<char *> raw;
    for (auto &a : args)
      raw.push_back(a.data());
    raw.push_back(nullptr);
    execvp("python3", raw.data());
    _exit(127);
  }
  int status = 0;
  while (waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) {
      unlink(path);
      throw std::runtime_error("startup helper wait failed");
    }
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    unlink(path);
    throw std::runtime_error("one-shot calibration preparation failed");
  }
  try {
    std::ifstream in(path);
    json descriptor;
    in >> descriptor;
    unlink(path);
    descriptor["startup_helper_exited"] = true;
    return descriptor;
  } catch (...) {
    unlink(path);
    throw;
  }
}
static rclcpp::NodeOptions native_options(const json &descriptor) {
  std::vector<rclcpp::Parameter> overrides;
  for (auto it = descriptor.at("parameters").begin();
       it != descriptor.at("parameters").end(); ++it) {
    const auto &v = it.value();
    if (v.is_boolean())
      overrides.emplace_back(it.key(), v.get<bool>());
    else if (v.is_number_integer())
      overrides.emplace_back(it.key(), v.get<int64_t>());
    else if (v.is_number())
      overrides.emplace_back(it.key(), v.get<double>());
    else if (v.is_string())
      overrides.emplace_back(it.key(), v.get<std::string>());
    else if (v.is_array())
      overrides.emplace_back(it.key(), v.get<std::vector<double>>());
    else if (v.is_null() && (it.key() == "currents" || it.key() == "speeds"))
      overrides.emplace_back(it.key(), std::vector<double>{});
  }
  return rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .parameter_overrides(overrides);
}
static std::vector<char *> native_arguments(int argc, char **argv) {
  std::vector<char *> result{argv[0]};
  bool ros = false;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--ros-args")
      ros = true;
    else if (arg == "--")
      ros = false;
    if (ros && (arg == "--params-file" || arg == "-p" || arg == "--param")) {
      if (i + 1 >= argc)
        throw std::runtime_error("missing ROS parameter argument");
      ++i;
      continue;
    }
    if (ros &&
        (arg.rfind("--params-file=", 0) == 0 || arg.rfind("--param=", 0) == 0))
      continue;
    result.push_back(argv[i]);
  }
  return result;
}
class CalibrationNode final : public rclcpp::Node {
public:
  explicit CalibrationNode(json descriptor)
      : Node("longitudinal_calibration", native_options(descriptor)),
        d_(std::move(descriptor)), engine_(d_["prepared"]),
        config_(d_["prepared"]["config"]), obs_(d_["observation"]),
        writer_(d_["output_directory"].get<std::string>(), config_,
                d_["metadata"], d_["manifest"]) {
    for (auto it = d_["parameters"].begin(); it != d_["parameters"].end();
         ++it) {
      if (has_parameter(it.key()))
        continue;
      auto v = it.value();
      if (v.is_boolean())
        declare_parameter(it.key(), v.get<bool>());
      else if (v.is_number_integer())
        declare_parameter(it.key(), v.get<int64_t>());
      else if (v.is_number())
        declare_parameter(it.key(), v.get<double>());
      else if (v.is_string())
        declare_parameter(it.key(), v.get<std::string>());
      else if (v.is_array())
        declare_parameter(it.key(), v.get<std::vector<double>>());
    }
    if (has_parameter("use_sim_time") &&
        get_parameter("use_sim_time").as_bool())
      set_parameter(get_parameter("use_sim_time"));
    if (get_parameter("auto_start").as_bool())
      throw std::runtime_error(
          "auto_start must remain false; use explicit /calib/enable service");
    if (config_["steering_source"] == "pp")
      tracking_ = std::make_unique<aims_pp::Runtime>(
          d_["parameters"]["reference_directory"].get<std::string>(),
          d_["pp_configuration"], config_["stage"] == "decel_current",
          config_["sweep_protocol"] == "full_range_step");
    auto latest =
        rclcpp::QoS(1).reliable().lifespan(std::chrono::milliseconds(100));
    auto sensor = rclcpp::SensorDataQoS();
    auto sensor_latest = rclcpp::QoS(1).best_effort();
    auto durable = rclcpp::QoS(1).reliable().transient_local();
    commands_ = create_publisher<VescCommand>("/calib/vesc_command", latest);
    statuses_ = create_publisher<DiagnosticArray>("/calib/status", 10);
    subscribe<nav_msgs::msg::Odometry>(
        "/rear_axle/wheel_odom", sensor, [this](const auto &m) {
          receipt("wheel");
          obs_["wheel_stamp"] = seconds(m.header.stamp);
          obs_["velocity"] = m.child_frame_id == "base_link"
                                 ? finite(m.twist.twist.linear.x)
                                 : json();
        });
    subscribe<vesc_msgs::msg::VescStateStamped>(
        "/sensors/core", sensor, [this](const auto &m) {
          receipt("telemetry");
          obs_["telemetry_stamp"] = seconds(m.header.stamp);
          auto &s = m.state;
          bool valid = std::isfinite(s.current_motor) &&
                       std::isfinite(s.voltage_input) && std::isfinite(s.speed);
          obs_["current_motor"] = valid ? finite(s.current_motor) : json();
          obs_["voltage_input"] = valid ? finite(s.voltage_input) : json();
          obs_["erpm"] = valid ? finite(s.speed) : json();
          obs_["avg_iq"] = finite(s.avg_iq);
          obs_["fault_code"] = s.fault_code;
        });
    subscribe<crsf_receiver_msg::msg::CRSFChannels16>(
        "/rc/channels", sensor_latest, [this](const auto &m) {
          receipt("rc");
          std::array<int, 16> v = {
              m.ch1, m.ch2,  m.ch3,  m.ch4,  m.ch5,  m.ch6,  m.ch7,  m.ch8,
              m.ch9, m.ch10, m.ch11, m.ch12, m.ch13, m.ch14, m.ch15, m.ch16};
          bool standard = d_["parameters"]["rc_profile"] ==
                          "steering_ch1_throttle_ch3_aux_ch5_to_ch10";
          std::vector<int> used = standard
                                      ? std::vector<int>{1, 3, 5, 6, 7, 8, 10}
                                      : std::vector<int>{1, 2, 3, 4, 5, 6, 7};
          bool valid = true;
          for (auto i : used)
            valid = valid && v[i - 1] > 0 && v[i - 1] <= 2047;
          int raw = v[standard ? 0 : 1], delta = raw - 992;
          valid =
              valid && v[standard ? 4 : 2] >= 992 && v[standard ? 7 : 6] > 992;
          obs_["rc_stamp"] = now().seconds();
          obs_["rc_valid"] = valid;
          if (!tracking_) {
            obs_["steering"] =
                -std::clamp(delta / double(delta > 0 ? 818 : 820), -1., 1.) *
                .4751;
            obs_["straight"] = std::abs(delta) <= 50;
          }
          if (!valid)
            selected_fault("rc_invalid_or_locked");
        });
    subscribe<std_msgs::msg::Bool>("/control/calibration_enabled", latest,
                                   [this](const auto &m) {
                                     receipt("authority");
                                     obs_["authority"] = m.data;
                                     obs_["authority_stamp"] = now().seconds();
                                     if (!m.data)
                                       selected_fault("authority_withdrawn");
                                   });
    subscribe<VescCommand>("/vesc/command", latest, [this](const auto &m) {
      receipt("final");
      std::array<double, 5> targets = {m.target_speed_mps, m.target_current,
                                       m.target_duty, m.target_brake_current,
                                       m.target_acceleration_mps2};
      int mode = m.control_mode;
      bool valid = m.header.frame_id == "base_link" &&
                   (mode == 0 || mode == 1) && std::isfinite(m.steering_angle);
      for (int i = 0; i < 5; ++i)
        valid = valid && std::isfinite(targets[i]) &&
                (i == mode || targets[i] == 0.);
      obs_["final_request"] = valid ? json({{"stamp", seconds(m.header.stamp)},
                                            {"mode", mode},
                                            {"steering", m.steering_angle},
                                            {"speed", targets[0]},
                                            {"current", targets[1]}})
                                    : json();
      obs_["final_received"] = now().seconds();
      if (!valid)
        selected_fault("invalid_final_command");
    });
    subscribe<geometry_msgs::msg::AccelWithCovarianceStamped>(
        "/state_estimation/longitudinal_acceleration", sensor,
        [this](const auto &m) {
          receipt("acceleration");
          obs_["acceleration"] = finite(m.accel.accel.linear.x);
          obs_["acceleration_stamp"] = seconds(m.header.stamp);
        });
    subscribe<DiagnosticArray>(
        "/state_estimation/longitudinal_acceleration/status", 10,
        [this](const auto &m) {
          receipt("acceleration_quality");
          acceleration_status_ = json::object();
          std::string quality;
          for (auto &s : m.status) {
            if (!quality.empty())
              quality += "; ";
            quality += std::to_string(s.level) + ":" + s.message;
            for (auto &v : s.values)
              acceleration_status_[v.key] = v.value;
          }
          obs_["acceleration_quality"] =
              quality.empty() ? "unavailable" : quality;
        });
    if (tracking_) {
      subscribe<nav_msgs::msg::Odometry>(
          "/odometry/filtered", sensor, [this](const auto &m) {
            receipt("odometry");
            auto &p = m.pose.pose.position;
            auto &q = m.pose.pose.orientation;
            double norm =
                std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!std::isfinite(norm) || std::abs(norm - 1.) > .01)
              throw std::runtime_error("invalid odometry quaternion");
            double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
                                    1 - 2 * (q.y * q.y + q.z * q.z));
            tracking_->observe_odometry(
                {{"x", p.x},
                 {"y", p.y},
                 {"yaw", yaw},
                 {"velocity", m.twist.twist.linear.x},
                 {"stamp_ns", nanoseconds(m.header.stamp)},
                 {"frame_id", m.header.frame_id},
                 {"child_frame_id", m.child_frame_id}},
                now().nanoseconds(), monotonic());
            tracking_fault();
          });
      subscribe<DiagnosticArray>(
          "/localization/status", 10, [this](const auto &m) {
            json values = json::object();
            size_t count = 0;
            const diagnostic_msgs::msg::DiagnosticStatus *record = nullptr;
            for (auto &s : m.status)
              if (s.name == "aims_racer_system/localization") {
                ++count;
                record = &s;
              }
            double age = now().seconds() - seconds(m.header.stamp);
            if (count == 1 && nanoseconds(m.header.stamp) > 0 && age >= 0 &&
                age <= .3) {
              for (auto &v : record->values) {
                if (values.contains(v.key)) {
                  values = json::object();
                  break;
                }
                values[v.key] = v.value;
              }
            }
            tracking_->observe_health(values, now().nanoseconds(), monotonic());
            tracking_fault();
          });
      subscribe<std_msgs::msg::String>(
          "/localization/map_sha256", durable, [this](const auto &m) {
            tracking_->observe_map_identity(m.data);
            if (m.data != d_["metadata"]["map_sha256"])
              selected_fault("map_identity_mismatch");
          });
      subscribe<std_msgs::msg::Bool>(
          "/localization/map_valid", durable, [this](const auto &m) {
            tracking_->observe_map_valid(m.data);
            if (!m.data)
              selected_fault("map_validity_withdrawn");
          });
      reference_ =
          create_publisher<nav_msgs::msg::Path>("/calib/reference", durable);
      publish_reference();
    }
    driver_ = create_client<GetParameters>("/vesc_driver_node/get_parameters");
    enable_ = create_service<std_srvs::srv::SetBool>(
        "/calib/enable",
        [this](std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          enable(req->data, *res);
        });
    parameter_guard_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = std::all_of(
              parameters.begin(), parameters.end(),
              [](const auto &p) { return p.get_name() == "use_sim_time"; });
          result.reason = result.successful
                              ? ""
                              : "Calibration configuration is frozen; restart "
                                "with matching artifacts";
          return result;
        });
    timer_ =
        create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    status_timer_ =
        create_wall_timer(std::chrono::milliseconds(200), [this] { status(); });
    RCLCPP_INFO(get_logger(), "Native calibration disabled; evidence: %s",
                d_["output_directory"].get<std::string>().c_str());
  }
  ~CalibrationNode() override {
    try {
      engine_.enable(false);
      checkpoint();
      writer_.close();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Evidence shutdown: %s", e.what());
    }
  }

private:
  json d_;
  NativeEngine engine_;
  json config_, obs_, acceleration_status_ = json::object();
  AsyncEvidenceWriter writer_;
  std::unique_ptr<aims_pp::Runtime> tracking_;
  std::map<std::string, double> receipts_;
  SteadyGuard guard_;
  std::string evidence_error_;
  json last_request_ = {{"stamp", 0.},
                        {"mode", 0},
                        {"steering", 0.},
                        {"speed", 0.},
                        {"current", 0.}};
  size_t enable_count_ = 0, tracking_generation_ = 0;
  double last_remaining_ = std::numeric_limits<double>::quiet_NaN(),
         driver_received_ = -std::numeric_limits<double>::infinity();
  bool driver_disabled_ = false, driver_pending_ = false;
  size_t saved_revision_ = 0, saved_trials_ = 0, saved_skips_ = 0;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<VescCommand>::SharedPtr commands_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr statuses_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_;
  rclcpp::Client<GetParameters>::SharedPtr driver_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_guard_;
  template <class M, class Q, class F>
  void subscribe(const std::string &t, const Q &q, F f) {
    subscriptions_.push_back(
        create_subscription<M>(t, q, [this, f](typename M::ConstSharedPtr m) {
          try {
            f(*m);
          } catch (const std::exception &e) {
            if (tracking_)
              tracking_->invalidate(e.what());
            selected_fault(e.what());
          }
        }));
  }
  void receipt(const std::string &s) { receipts_[s] = monotonic(); }
  double age(const std::string &s, double steady) const {
    auto it = receipts_.find(s);
    return it == receipts_.end() ? std::numeric_limits<double>::infinity()
                                 : steady - it->second;
  }
  void selected_fault(const std::string &s) {
    auto state = engine_.status();
    if (state.value("enabled", false) && state.value("selected", false))
      engine_.fault(s);
  }
  void tracking_fault() {
    auto state = tracking_->snapshot(now().nanoseconds(), monotonic());
    if (state.value("fault_generation", size_t(0)) != tracking_generation_)
      selected_fault(state.value(
          "fault_reason", std::string("localization_validity_withdrawn")));
  }
  json tracking_observation(json o, double ros, double steady) {
    if (!tracking_)
      return o;
    tracking_->observe_clock(int64_t(ros * 1e9));
    auto result = tracking_->evaluate(0., int64_t(ros * 1e9), steady);
    std::string reason = result.value("reason", std::string());
    auto state = engine_.status();
    if (state.value("enabled", false) && state.value("selected", false) &&
        result.value("fault_generation", size_t(0)) != tracking_generation_)
      reason = result.value("fault_reason",
                            std::string("localization_validity_withdrawn"));
    if (!reason.empty()) {
      o["straight"] = false;
      o["tracking_reason"] = reason;
      o["straight_remaining_m"] = nullptr;
      return o;
    }
    double s = result.at("progress");
    o["steering"] = result.at("steering");
    o["straight"] = false;
    o["region"] = "";
    o["straight_remaining_m"] = nullptr;
    o["tracking_reason"] = "";
    double length = d_["reference_length"];
    s = std::fmod(s, length);
    if (s < 0)
      s += length;
    for (auto &r : d_["regions"]) {
      double a = r["start_s"], b = r["end_s"];
      if (a < b ? (s >= a && s < b) : (s >= a || s < b)) {
        o["straight"] = true;
        o["region"] = r["id"];
        double remaining = std::fmod(b - s + length, length);
        o["straight_remaining_m"] = remaining;
        break;
      }
    }
    return o;
  }
  void poll_driver() {
    if (driver_pending_ || monotonic() - driver_received_ < .5 ||
        !driver_->service_is_ready())
      return;
    driver_pending_ = true;
    auto req = std::make_shared<GetParameters::Request>();
    req->names = {"current_startup_enabled"};
    driver_->async_send_request(
        req, [this](rclcpp::Client<GetParameters>::SharedFuture f) {
          driver_pending_ = false;
          try {
            auto v = f.get()->values;
            driver_disabled_ =
                v.size() == 1 && v[0].type == 1 && !v[0].bool_value;
            driver_received_ = monotonic();
          } catch (...) {
            driver_disabled_ = false;
          }
        });
  }
  json preflight(bool verify = false) {
    double ros = now().seconds(), steady = monotonic();
    auto state = engine_.status();
    json ages = json::object();
    for (auto pair : std::vector<std::pair<std::string, std::string>>{
             {"wheel", "wheel_stamp"},
             {"telemetry", "telemetry_stamp"},
             {"rc", "rc_stamp"},
             {"authority", "authority_stamp"}})
      ages[pair.first + "_source"] = finite(ros - number(obs_, pair.second));
    ages["final_source"] =
        finite(obs_["final_request"].is_null()
                   ? std::numeric_limits<double>::infinity()
                   : ros - number(obs_["final_request"], "stamp"));
    for (auto name : {"wheel", "telemetry", "final", "rc", "authority"})
      ages[std::string(name) + "_receipt"] = finite(age(name, steady));
    auto final = obs_["final_request"];
    json snapshot = {
        {"state", state["state"]},
        {"enabled", state["enabled"]},
        {"enable_count", enable_count_},
        {"logging_error",
         writer_.error().empty() ? evidence_error_ : writer_.error()},
        {"logging_dropped", writer_.dropped()},
        {"current_signal", "avg_iq"},
        {"freshness", config_["freshness"]},
        {"ages", ages},
        {"wheel_speed", obs_["velocity"]},
        {"telemetry_valid",
         std::isfinite(number(obs_, "avg_iq")) &&
             std::isfinite(number(obs_, "current_motor")) &&
             std::isfinite(number(obs_, "voltage_input")) &&
             std::isfinite(number(obs_, "erpm")) &&
             number(obs_, "voltage_input") > 0 &&
             (config_["stage"] == "speed_hold" || obs_["fault_code"] == 0)},
        {"authority", obs_["authority"]},
        {"rc_valid", obs_["rc_valid"]},
        {"final_zero", !final.is_null() && final["mode"] == 0 &&
                           final["speed"] == 0 && final["current"] == 0},
        {"publishers",
         {{"calibration", count_publishers("/calib/vesc_command")},
          {"final", count_publishers("/vesc/command")},
          {"drive", count_publishers("/drive")}}},
        {"driver_startup_disabled", driver_disabled_},
        {"driver_parameter_age", finite(steady - driver_received_)},
        {"pp", bool(tracking_)},
        {"localization_reason", ""},
        {"cross_track", nullptr},
        {"heading_error", nullptr},
        {"require_acceleration", d_["parameters"]["require_acceleration"]},
        {"artifact_error", ""}};
    bool acc = acceleration_status_.value("initialized", "") == "true" &&
               acceleration_status_.value("force_valid", "") == "true";
    for (auto key : {"wheel_age_s", "imu_age_s", "force_age_s"})
      try {
        double a = std::stod(acceleration_status_.at(key).get<std::string>());
        acc = acc && std::isfinite(a) && a >= 0 && a <= .1;
      } catch (...) {
        acc = false;
      }
    double a = ros - number(obs_, "acceleration_stamp");
    snapshot["acceleration_valid"] =
        acc && std::isfinite(number(obs_, "acceleration")) && a >= 0 &&
        a <= .1 && age("acceleration", steady) >= 0 &&
        age("acceleration", steady) <= .1 &&
        age("acceleration_quality", steady) >= 0 &&
        age("acceleration_quality", steady) <= 2.;
    if (tracking_) {
      tracking_->observe_clock(now().nanoseconds());
      auto p = tracking_->evaluate(0., now().nanoseconds(), steady);
      snapshot["localization_reason"] = p.value("reason", "");
      if (p.value("reason", "").empty()) {
        snapshot["cross_track"] = p["lateral_error"];
        snapshot["heading_error"] = p["heading_error"];
        snapshot["progress"] = p["progress"];
      }
    }
    if (verify) {
      auto metadata = d_["metadata"];
      auto check = [&](std::string path, json expected, std::string label) {
        try {
          if (!expected.is_string() ||
              sha256_file(path) != expected.get<std::string>())
            snapshot["artifact_error"] = label + " changed after launch";
        } catch (const std::exception &e) {
          snapshot["artifact_error"] = label + " unavailable: " + e.what();
        }
      };
      for (auto pair : std::vector<std::pair<std::string, std::string>>{
               {"vehicle_config", "vehicle_sha256"},
               {"calibration_config", "calibration_config_sha256"},
               {"pp_config", "pp_config_sha256"},
               {"base_current_file", "base_current_file_sha256"},
               {"response_limits_file", "response_limits_sha256"}})
        if (metadata.contains(pair.first) && metadata[pair.first].is_string() &&
            !metadata[pair.first].get<std::string>().empty())
          check(metadata[pair.first], metadata.value(pair.second, json()),
                pair.first);
      if (tracking_)
        for (auto pair : std::vector<std::pair<std::string, std::string>>{
                 {"reference.json", "reference_sha256"},
                 {"calibration_regions.json", "calibration_regions_sha256"}})
          check(metadata["reference_directory"].get<std::string>() + "/" +
                    pair.first,
                metadata[pair.second], pair.first);
    }
    return snapshot;
  }
  void checkpoint() {
    auto state = engine_.snapshot();
    writer_.trials(state["trials"], evidence_error_, state["skipped"]);
    saved_revision_ = state.value("results_revision", size_t(0));
    saved_trials_ = state["trials"].size();
    saved_skips_ = state["skipped"].size();
  }
  void enable(bool value, std_srvs::srv::SetBool::Response &response) {
    try {
      if (value) {
        auto snapshot = preflight(true);
        auto reasons = check_start_native(snapshot);
        if (!reasons.empty()) {
          response.success = false;
          response.message = "Start refused: ";
          for (size_t i = 0; i < reasons.size(); ++i) {
            if (i)
              response.message += ", ";
            response.message += reasons[i];
          }
          return;
        }
        std::string marker =
            d_["output_directory"].get<std::string>() + "/enable-attempt.json";
        int fd = open(marker.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
        if (fd < 0)
          throw std::runtime_error("enable marker unavailable: " +
                                   std::string(strerror(errno)));
        auto payload =
            json({{"requested_at_ns",
                   std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count()},
                  {"preflight", snapshot}})
                .dump();
        size_t offset = 0;
        while (offset < payload.size()) {
          ssize_t count =
              write(fd, payload.data() + offset, payload.size() - offset);
          if (count <= 0) {
            close(fd);
            throw std::runtime_error("enable marker write failed");
          }
          offset += count;
        }
        if (close(fd) != 0)
          throw std::runtime_error("enable marker close failed");
        ++enable_count_;
        engine_.enable(true, now().seconds());
        obs_ = d_["observation"];
        receipts_.clear();
        guard_.reset();
        writer_.reset_observations();
        if (tracking_)
          tracking_generation_ =
              tracking_->snapshot(now().nanoseconds(), monotonic())
                  .value("fault_generation", size_t(0));
        checkpoint();
      } else {
        engine_.enable(false);
        checkpoint();
      }
      response.success = true;
      response.message =
          value ? "PREPARE; awaiting fresh selected calibration authority"
                : "Stopped; incomplete trial invalidated";
    } catch (const std::exception &e) {
      engine_.fault(e.what());
      response.success = false;
      response.message = e.what();
    }
  }
  void publish(json request, const rclcpp::Time &ros) {
    VescCommand command;
    command.header.stamp = ros;
    command.header.frame_id = "base_link";
    int mode = request.at("mode");
    if (mode != 0 && mode != 1)
      throw std::runtime_error("calibration emits only speed/current modes");
    for (auto key : {"steering", "speed", "current"})
      if (!std::isfinite(number(request, key)))
        throw std::runtime_error("nonfinite calibration request");
    command.control_mode = mode;
    command.steering_angle = number(request, "steering");
    if (mode == 0)
      command.target_speed_mps = number(request, "speed");
    else
      command.target_current = number(request, "current");
    commands_->publish(command);
    engine_.sent(request);
    last_request_ = std::move(request);
  }
  void tick() {
    if (has_parameter("use_sim_time") &&
        get_parameter("use_sim_time").as_bool() &&
        (!get_clock()->ros_time_is_active() || now().nanoseconds() <= 0) &&
        engine_.status().value("state", "") == "IDLE") {
      auto ros = now();
      publish({{"stamp", ros.seconds()},
               {"mode", 0},
               {"steering", 0.},
               {"speed", 0.},
               {"current", 0.}},
              ros);
      return;
    }
    double started = monotonic();
    auto ros = now();
    double steady = monotonic();
    auto state = engine_.status();
    size_t first = state["trials_count"];
    double interval = guard_.last_timer >= 0 ? steady - guard_.last_timer : 0.;
    try {
      auto fault = guard_.check(
          ros.seconds(), steady,
          state.value("enabled", false) && state.value("selected", false),
          number(config_, "freshness"), !d_["prepared"]["protection"].is_null(),
          count_publishers("/calib/vesc_command"), receipts_);
      if (!fault.empty())
        engine_.fault(fault);
      auto obs = tracking_observation(obs_, ros.seconds(), steady);
      last_remaining_ = number(obs, "straight_remaining_m");
      if (age("acceleration", steady) > .2 ||
          ros.seconds() - number(obs, "acceleration_stamp") < 0 ||
          ros.seconds() - number(obs, "acceleration_stamp") > .2) {
        obs["acceleration"] = nullptr;
        obs["acceleration_quality"] = "estimate_stale_or_unavailable";
      } else if (age("acceleration_quality", steady) > 2.)
        obs["acceleration_quality"] = "quality_stale_or_unavailable";
      auto error = writer_.error();
      if (!error.empty()) {
        evidence_error_ = "logging_writer_failure: " + error;
        engine_.fault(evidence_error_);
        engine_.invalidate_evidence(
            std::min(first, writer_.failed_after_trial_count()),
            evidence_error_);
      }
      auto request = engine_.tick(ros.seconds(), obs);
      publish(request, ros);
      state = engine_.status();
      try {
        writer_.observation(obs, request, state["state"]);
        if (state.value("results_revision", size_t(0)) != saved_revision_ ||
            state["trials_count"] != saved_trials_ ||
            state["skipped_count"] != saved_skips_ || !evidence_error_.empty())
          checkpoint();
        writer_.runtime({{"ros_stamp", ros.seconds()},
                         {"callback_ms", (monotonic() - started) * 1000.},
                         {"timer_interval_ms", interval * 1000.},
                         {"pending", writer_.pending()},
                         {"dropped", writer_.dropped()},
                         {"state", state["state"]}});
      } catch (const std::exception &e) {
        engine_.fault(e.what());
        engine_.invalidate_evidence(first, e.what());
        evidence_error_ = e.what();
        try {
          checkpoint();
        } catch (...) {
        }
        publish(engine_.tick(ros.seconds(), obs), ros);
      }
    } catch (const std::exception &e) {
      engine_.fault(e.what());
      publish({{"stamp", ros.seconds()},
               {"mode", 1},
               {"steering", 0.},
               {"speed", 0.},
               {"current", 0.}},
              ros);
    }
  }
  void status() {
    try {
      poll_driver();
      auto state = engine_.status();
      auto snapshot = preflight();
      auto reasons = check_start_native(snapshot);
      json values = {
          {"state", state["state"]},
          {"enabled", state["enabled"]},
          {"enable_count", enable_count_},
          {"start_ready", reasons.empty()},
          {"start_refusals", json(reasons).dump()},
          {"preflight_json", snapshot.dump()},
          {"reason", state["reason"]},
          {"vehicle_config", d_["parameters"]["vehicle_config"]},
          {"reference_directory", d_["parameters"]["reference_directory"]},
          {"calibration_config", d_["parameters"]["config_file"]},
          {"stage", config_["stage"]},
          {"steering_source", config_["steering_source"]},
          {"trial_index", state["index"]},
          {"trials", state["trials_count"]},
          {"planned_trials", d_["runtime_plan"].size()},
          {"hold_sampling", config_["hold_sampling"]},
          {"sampled_seconds", state.value("hold_elapsed", json(0.))},
          {"measurement_seconds", config_["measurement_seconds"]},
          {"segment_count", state.value("hold_segments", json::array()).size()},
          {"current_signal", "avg_iq"},
          {"avg_iq", obs_["avg_iq"]},
          {"output_directory", d_["output_directory"]},
          {"current_motor", obs_["current_motor"]},
          {"voltage_input", obs_["voltage_input"]},
          {"erpm", obs_["erpm"]},
          {"velocity", obs_["velocity"]},
          {"request_mode", last_request_["mode"]},
          {"request_current", last_request_["current"]},
          {"request_speed", last_request_["speed"]},
          {"authority", obs_["authority"]},
          {"logging_pending", writer_.pending()},
          {"logging_dropped", writer_.dropped()},
          {"logging_committed_trials", writer_.committed_trials()},
          {"logging_committed_skips", writer_.committed_skips()},
          {"logging_error", writer_.error()},
          {"runtime_implementation", "native_cpp17"},
          {"startup_helper_exited", true}};
      size_t index = state.value("index", size_t(0));
      if (config_["stage"] == "speed_hold")
        values["target_speed"] = index < d_["runtime_plan"].size()
                                     ? d_["runtime_plan"][index]
                                     : json("");
      else {
        json item = index < d_["prepared"]["plan"].size()
                        ? d_["prepared"]["plan"][index]
                        : json();
        values.update(
            {{"predictive_exit", config_["predictive_exit"]},
             {"measured_current_handoff", config_["measured_current_handoff"]},
             {"sweep_protocol", config_["sweep_protocol"]},
             {"straight_remaining_m", finite(last_remaining_)},
             {"steady_evidence", state.value("steady_evidence", json()).dump()},
             {"exit_prediction", state.value("exit_prediction", json()).dump()},
             {"handoff", state.value("handoff", json()).dump()},
             {"plan_mode", d_["parameters"]["plan_mode"]},
             {"trial_kind", item.is_null() ? json("") : item["kind"]},
             {"target_speed", item.is_null() ? json("") : item["start_speed"]},
             {"target_end_speed",
              item.is_null() ? json("") : item["end_speed"]},
             {"target_current", item.is_null() ? json("") : item["current"]},
             {"skipped_trials", state["skipped_count"]},
             {"current_tracking_status",
              state.value("tracking_status", json("pending"))},
             {"current_tracking_evidence",
              state.value("tracking_evidence", json::object()).dump()},
             {"base_current_file",
              d_["metadata"].value("base_current_file", json(""))}});
      }
      DiagnosticArray message;
      message.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus s;
      s.name = "longitudinal_calibration";
      s.hardware_id = "vesc_calibration";
      s.level = state["state"] == "FAULT" ? 2 : 0;
      s.message = state.value("reason", std::string());
      for (auto it = values.begin(); it != values.end(); ++it) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = it.key();
        kv.value = text(it.value());
        s.values.push_back(kv);
      }
      message.status.push_back(s);
      statuses_->publish(message);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "status failed: %s", e.what());
    }
  }
  void publish_reference() {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    auto points = tracking_->reference_points();
    size_t step = std::max(size_t(1), points.size() / 2000);
    for (size_t i = 0; i < points.size(); i += step) {
      auto p = points[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p["x"];
      pose.pose.position.y = p["y"];
      double yaw = p["yaw"];
      pose.pose.orientation.z = std::sin(yaw / 2);
      pose.pose.orientation.w = std::cos(yaw / 2);
      path.poses.push_back(pose);
    }
    reference_->publish(path);
  }
};
} // namespace aims_calibration
int main(int argc, char **argv) {
  try {
    auto descriptor = aims_calibration::prepare(argc, argv);
    auto arguments = aims_calibration::native_arguments(argc, argv);
    rclcpp::init(static_cast<int>(arguments.size()), arguments.data());
    auto node = std::make_shared<aims_calibration::CalibrationNode>(
        std::move(descriptor));
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "calibration native startup/runtime error: " << e.what()
              << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
