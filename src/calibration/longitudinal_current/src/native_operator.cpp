// Operator service client and sustained monitor. Never publishes motor commands.
#include "aims_calibration/native_runtime.hpp"
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
extern char **environ;

namespace {
using json = nlohmann::json;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void interrupt_handler(int) { interrupted = 1; }
double monotonic() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
double positive(const std::string &value) {
  size_t used = 0;
  const double number = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(number) || number <= 0.)
    throw std::runtime_error("timeouts must be positive and finite");
  return number;
}
struct Options {
  std::string command;
  double timeout = 5.;
  std::optional<double> max_seconds;
  bool help = false;
};
Options options(int argc, char **argv) {
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  Options result;
  result.command = std::filesystem::path(args.at(0)).filename().string();
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--help" || args[i] == "-h") result.help = true;
    else if ((args[i] == "--timeout" || args[i] == "--max-seconds" || args[i] == "--command") && i + 1 < args.size()) {
      const std::string key = args[i++], value = args[i];
      if (key == "--timeout") result.timeout = positive(value);
      else if (key == "--max-seconds") result.max_seconds = positive(value);
      else result.command = value;
    } else throw std::runtime_error("Unknown or incomplete argument: " + args[i]);
  }
  if (result.command != "inspect" && result.command != "start" && result.command != "stop")
    throw std::runtime_error("Operator command must be inspect, start or stop");
  if (result.max_seconds && result.command != "start") throw std::runtime_error("--max-seconds applies to start");
  return result;
}

class Monitor {
 public:
  Monitor() : node_(std::make_shared<rclcpp::Node>("calibration_operator")) {
    status_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/calib/status", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        for (const auto &status : msg->status) if (status.name == "longitudinal_calibration") {
          values_.clear();
          for (const auto &entry : status.values) values_[entry.key] = entry.value;
          status_received_ = monotonic();
        }
      });
    wheel_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/rear_axle/wheel_odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (msg->child_frame_id != "base_link") return;
        wheel_ = msg->twist.twist.linear.x;
        wheel_stamp_ = static_cast<double>(msg->header.stamp.sec) + msg->header.stamp.nanosec * 1e-9;
        wheel_received_ = monotonic();
      });
    client_ = node_->create_client<std_srvs::srv::SetBool>("/calib/enable");
    executor_.add_node(node_);
  }
  ~Monitor() { executor_.remove_node(node_); }
  std::string value(const std::string &key, const std::string &fallback = "") const {
    auto it = values_.find(key); return it == values_.end() ? fallback : it->second;
  }
  const std::map<std::string, std::string> &values() const { return values_; }
  void spin(bool check_interrupt = true) {
    if (check_interrupt && interrupted) throw std::runtime_error("Operator interrupted");
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (check_interrupt && interrupted) throw std::runtime_error("Operator interrupted");
  }
  void wait_status(double timeout) {
    const double deadline = monotonic() + timeout;
    while (monotonic() < deadline) {
      spin();
      if (!values_.empty() && monotonic() - status_received_ <= .4) return;
    }
    throw std::runtime_error("No fresh /calib/status; launch calibration first");
  }
  void request(bool enabled, double timeout) {
    const double discovery_deadline = monotonic() + timeout;
    while (!client_->service_is_ready() && monotonic() < discovery_deadline) spin(enabled);
    if (!client_->service_is_ready()) throw std::runtime_error("/calib/enable unavailable");
    if (enabled && interrupted) throw std::runtime_error("Operator interrupted");
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>(); req->data = enabled;
    auto future = client_->async_send_request(req);
    const double deadline = monotonic() + timeout;
    while (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready && monotonic() < deadline) spin(enabled);
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      client_->remove_pending_request(future);
      throw std::runtime_error("Enable response timed out; outcome is uncertain");
    }
    const auto response = future.get();
    if (!response || !response->success) throw std::runtime_error(response ? response->message : "No service response");
    std::cout << response->message << std::endl;
  }
  bool stationary() const {
    const double age = node_->now().seconds() - wheel_stamp_;
    return std::isfinite(wheel_) && monotonic() - wheel_received_ <= .2 && age >= 0. && age <= .2 && std::abs(wheel_) <= .05;
  }
  bool completion() const {
    if (monotonic() - status_received_ > 1.) throw std::runtime_error("Completion status lost");
    if (value("state") != "COMPLETE") throw std::runtime_error("Acquisition changed after completion: " + value("state", "missing"));
    if (!value("logging_error").empty() || std::stoll(value("logging_dropped", "0")) != 0)
      throw std::runtime_error("Logging failed after completion: " + value("logging_error", "dropped evidence"));
    return std::stoll(value("logging_committed_trials", "-1")) == std::stoll(value("trials", "-2")) &&
           std::stoll(value("logging_committed_skips", "0")) == std::stoll(value("skipped_trials", "0"));
  }
  bool fresh() const { return monotonic() - status_received_ <= 1.; }
 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  std::map<std::string, std::string> values_;
  double status_received_ = -INFINITY, wheel_ = NAN, wheel_stamp_ = -INFINITY, wheel_received_ = -INFINITY;
};

// Reuse the offline acceptance checker only after runtime completion. The
// operator monitor and all service/clock handling stay in this native process.
std::pair<int, std::string> accept_results(const std::string &directory) {
  int pipes[2]; if (pipe(pipes) != 0) throw std::runtime_error("Result checker pipe failed");
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, pipes[1], STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, pipes[1], STDERR_FILENO);
  posix_spawn_file_actions_addclose(&actions, pipes[0]);
  posix_spawn_file_actions_addclose(&actions, pipes[1]);
  const std::string code = "from aims_calibration.cli import results_main; raise SystemExit(results_main())";
  std::vector<std::string> args = {"python3", "-c", code, (std::filesystem::path(directory) / "trials.json").string()};
  std::vector<char *> argv; for (auto &a : args) argv.push_back(a.data()); argv.push_back(nullptr);
  pid_t child;
  int result = posix_spawnp(&child, "python3", &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions); close(pipes[1]);
  if (result != 0) { close(pipes[0]); throw std::runtime_error("Cannot start offline result checker"); }
  std::string output; char buffer[4096]; ssize_t count;
  while ((count = read(pipes[0], buffer, sizeof(buffer))) > 0) output.append(buffer, static_cast<size_t>(count));
  close(pipes[0]); int status = 0;
  while (waitpid(child, &status, 0) < 0) { if (errno != EINTR) throw std::runtime_error("Result checker wait failed"); }
  return {WIFEXITED(status) ? WEXITSTATUS(status) : 3, output};
}

int run(const Options &o) {
  Monitor monitor;
  if (o.command == "inspect") {
    try {
      monitor.wait_status(o.timeout);
      const auto preflight = json::parse(monitor.value("preflight_json", "{}"));
      std::cout << json({{"status", monitor.values()}, {"preflight", preflight},
                        {"start_refusals", aims_calibration::check_start_native(preflight)}}).dump(2) << std::endl;
      return 0;
    } catch (const std::exception &e) { std::cout << e.what() << std::endl; return 2; }
  }
  if (o.command == "stop") {
    try { monitor.request(false, o.timeout); std::cout << "Stop requested. Confirm physical standstill; zero current alone does not guarantee it." << std::endl; return 0; }
    catch (const std::exception &e) { std::cout << e.what() << std::endl; return 3; }
  }
  bool attempted = false;
  try {
    monitor.wait_status(o.timeout);
    const double max_seconds = o.max_seconds.value_or(monitor.value("stage", "speed_hold") == "speed_hold" ? 900. : 7200.);
    const auto snapshot = json::parse(monitor.value("preflight_json", "{}"));
    const auto reasons = aims_calibration::check_start_native(snapshot);
    if (!reasons.empty()) { std::cout << json({{"started", false}, {"reasons", reasons}}).dump(2) << std::endl; return 2; }
    attempted = true; monitor.request(true, o.timeout); // Never retry uncertain enable.
    const double began = monotonic(); double report_at = began;
    std::string terminal;
    while (monotonic() - began < max_seconds) {
      monitor.spin(); const double now = monotonic();
      if (!monitor.fresh()) throw std::runtime_error("Calibration monitor input lost");
      if (now >= report_at) {
        json report = json::object();
        for (const auto *key : {"state", "stage", "trial_kind", "target_speed", "target_end_speed", "target_current", "trials", "planned_trials", "skipped_trials", "sampled_seconds", "measurement_seconds", "segment_count", "velocity", "reason", "output_directory"}) report[key] = monitor.value(key);
        std::cout << report.dump() << std::endl; report_at = now + 2.;
      }
      const auto state = monitor.value("state");
      if (std::stoll(monitor.value("enable_count", "0")) >= 1 && (state == "COMPLETE" || state == "FAULT" || state == "STOPPING")) { terminal = state; break; }
    }
    if (terminal.empty()) throw std::runtime_error("Acquisition monitor timeout");
    if (terminal != "COMPLETE") monitor.request(false, o.timeout);
    const double stop_deadline = monotonic() + 20.; std::optional<double> stationary_since;
    while (monotonic() < stop_deadline) {
      monitor.spin(); if (terminal == "COMPLETE") monitor.completion();
      if (monitor.stationary()) { if (!stationary_since) stationary_since = monotonic(); if (monotonic() - *stationary_since >= 3.) break; }
      else stationary_since.reset();
    }
    const bool stopped = stationary_since && monotonic() - *stationary_since >= 3.;
    std::cout << json({{"terminal", terminal}, {"stationary_verified", stopped}, {"output_directory", monitor.value("output_directory")}}).dump(2) << std::endl;
    if (!stopped) { std::cout << "Physical standstill unverified: operator stop required. Keep the vehicle launch running." << std::endl; return 3; }
    if (terminal != "COMPLETE") return 3;
    const double checkpoint_deadline = monotonic() + o.timeout; std::string summary;
    while (monotonic() < checkpoint_deadline) {
      monitor.spin(); if (!monitor.completion()) continue;
      const auto result = accept_results(monitor.value("output_directory")); summary = result.second;
      if (result.first == 0) { monitor.completion(); std::cout << summary << std::flush; return 0; }
    }
    std::cout << summary << std::flush; return 3;
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
    if (attempted) try { monitor.request(false, o.timeout); }
      catch (const std::exception &stop_error) { std::cout << "Stop request failed: " << stop_error.what() << "; lock the vehicle with RC" << std::endl; }
    return 3;
  }
}
} // namespace

int main(int argc, char **argv) {
  try {
    const auto o = options(argc, argv);
    if (o.help) { std::cout << o.command << " [--timeout SECONDS]" << (o.command == "start" ? " [--max-seconds SECONDS]" : "") << " [ROS arguments]\n"; return 0; }
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, interrupt_handler); std::signal(SIGTERM, interrupt_handler);
    const int result = run(o); rclcpp::shutdown(); return result;
  } catch (const std::exception &e) { std::cerr << e.what() << std::endl; if (rclcpp::ok()) rclcpp::shutdown(); return 2; }
}
