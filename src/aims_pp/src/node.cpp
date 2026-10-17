#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <aims_pp/runtime.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
#include <unistd.h>
#include <vesc_msgs/msg/vesc_command.hpp>
using aims_pp::json;
namespace {
double steady() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
int64_t stamp(const builtin_interfaces::msg::Time &t) {
  if (t.sec < 0 || t.nanosec >= 1000000000)
    throw std::invalid_argument("invalid ROS header stamp");
  return int64_t(t.sec) * 1000000000 + t.nanosec;
}
std::string string_value(const json &v) {
  return v.is_string() ? v.get<std::string>() : v.dump();
}
class RuntimeLog {
  std::ofstream file_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<json> queue_;
  bool stop_ = false;
  uint64_t dropped_ = 0;
  std::string error_;
  const std::vector<std::string> columns_ = {
      "event",         "ros_ns",       "steady_s", "callback_s",
      "publish_gap_s", "source_age_s", "phase",    "speed",
      "steering",      "progress",     "traveled", "reason"};
  static std::string csv(const json &v) {
    auto s = string_value(v);
    std::string out = "\"";
    for (char c : s) {
      out += c;
      if (c == '\"')
        out += c;
    }
    return out + '\"';
  }
  void run() {
    try {
      for (;;) {
        json row;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cv_.wait_for(lock, std::chrono::milliseconds(50),
                       [&] { return stop_ || !queue_.empty(); });
          if (queue_.empty()) {
            file_.flush();
            if (stop_)
              break;
            continue;
          }
          row = std::move(queue_.front());
          queue_.pop_front();
        }
        for (size_t i = 0; i < columns_.size(); i++) {
          if (i)
            file_ << ',';
          if (row.contains(columns_[i]))
            file_ << csv(row[columns_[i]]);
        }
        file_ << '\n';
        if (!file_)
          throw std::runtime_error("runtime CSV write failed");
      }
      file_.flush();
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = e.what();
    }
  }

public:
  explicit RuntimeLog(const std::string &directory) {
    if (directory.empty())
      return;
    std::filesystem::create_directories(directory);
    auto path = (std::filesystem::path(directory) / "runtime.csv").string();
    int fd = ::open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd < 0)
      throw std::runtime_error(
          "runtime.csv already exists or cannot be created");
    ::close(fd);
    file_.open(path);
    for (size_t i = 0; i < columns_.size(); i++)
      file_ << (i ? "," : "") << columns_[i];
    file_ << '\n';
    file_.flush();
    thread_ = std::thread([this] { run(); });
  }
  ~RuntimeLog() { close(); }
  void close() {
    if (thread_.joinable()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
      }
      cv_.notify_all();
      thread_.join();
      file_.close();
    }
  }
  void submit(json row) {
    if (!thread_.joinable())
      return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= 1024) {
        dropped_++;
        return;
      }
      queue_.push_back(std::move(row));
    }
    cv_.notify_one();
  }
  json state() {
    std::lock_guard<std::mutex> lock(mutex_);
    return {{"log_dropped", dropped_}, {"log_error", error_}};
  }
};
class PPNode : public rclcpp::Node {
  std::unique_ptr<aims_pp::Runtime> runtime_;
  std::unique_ptr<RuntimeLog> log_;
  uint64_t sequence_ = 0, callback_count_ = 0;
  double callback_max_ = 0, last_publish_ = NAN, previous_publish_ = NAN;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      drive_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::TimerBase::SharedPtr tick_timer_, status_timer_;
  OnSetParametersCallbackHandle::SharedPtr guard_;
  std::pair<int64_t, double> clocks() {
    auto n = now().nanoseconds();
    double t = steady();
    runtime_->observe_clock(n);
    return {n, t};
  }
  void publish(const json &out, int64_t n, double t) {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = rclcpp::Time(n);
    cmd.header.frame_id = "base_link";
    cmd.drive.speed = out.at("speed").get<double>();
    cmd.drive.steering_angle = out.at("steering").get<double>();
    drive_->publish(cmd);
    previous_publish_ = last_publish_;
    last_publish_ = t;
    sequence_++;
  }
  template <class F> void accept(const std::string &event, F action) {
    double started = steady();
    auto [n, t] = clocks();
    auto before = runtime_->status(n, t);
    try {
      action(n, t);
    } catch (const std::exception &e) {
      runtime_->invalidate(event + ": " + e.what());
    }
    auto after = runtime_->status(n, t);
    if (before["enabled"] == true &&
        after["fault_generation"] != after["controller_generation"]) {
      auto zero = runtime_->tick(n, t, count_publishers("/drive"));
      publish(zero, n, t);
    }
    double elapsed = steady() - started;
    callback_count_++;
    callback_max_ = std::max(callback_max_, elapsed);
    log_->submit({{"event", event},
                  {"ros_ns", n},
                  {"steady_s", t},
                  {"callback_s", elapsed},
                  {"phase", runtime_->status(n, t)["phase"]},
                  {"reason", runtime_->snapshot(n, t)["fault_reason"]}});
  }
  void tick() {
    double started = steady();
    auto [n, t] = clocks();
    auto out = runtime_->tick(n, t, count_publishers("/drive"));
    publish(out, n, t);
    if (out.contains("lookahead")) {
      auto target = out["lookahead"];
      geometry_msgs::msg::PointStamped point;
      point.header.stamp = now();
      point.header.frame_id = "map";
      point.point.x = target["x"];
      point.point.y = target["y"];
      lookahead_->publish(point);
    }
    auto state = runtime_->status(n, t);
    json row = {{"event", "command"},
                {"ros_ns", n},
                {"steady_s", t},
                {"callback_s", steady() - started},
                {"source_age_s", state["source_age_s"]},
                {"phase", state["phase"]},
                {"speed", out["speed"]},
                {"steering", out["steering"]},
                {"progress", out["progress"]},
                {"traveled", out["traveled"]},
                {"reason", state["reason"]}};
    if (std::isfinite(previous_publish_))
      row["publish_gap_s"] = t - previous_publish_;
    log_->submit(row);
  }
  void status() {
    auto [n, t] = clocks();
    auto values = runtime_->status(n, t, count_publishers("/drive"));
    values.erase("pose");
    values.erase("source_age_s");
    values.erase("fault_generation");
    values.erase("fault_reason");
    values["sequence"] = sequence_;
    values["callback_count"] = callback_count_;
    values["callback_max_s"] = callback_max_;
    values.update(log_->state());
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "aims_pp/controller";
    status.hardware_id = "AIMSRacer";
    status.level = values["phase"] == "FAULT" ? 2 : 0;
    status.message = string_value(values["reason"]);
    for (auto it = values.begin(); it != values.end(); ++it) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = it.key();
      kv.value = string_value(it.value());
      status.values.push_back(kv);
    }
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    a.status.push_back(status);
    status_->publish(a);
  }

public:
  PPNode() : Node("aims_pp") {
    auto reference = declare_parameter<std::string>("reference_directory", "");
    auto vehicle = declare_parameter<std::string>("vehicle_config", "");
    auto tuning = declare_parameter<std::string>("pp_config", "");
    auto log = declare_parameter<std::string>("log_directory", "");
    auto speed = declare_parameter<double>("speed_limit", 1.);
    auto repeat = declare_parameter<bool>("repeat_laps", false);
    if (declare_parameter<bool>("auto_start", false))
      throw std::invalid_argument(
          "PP requires explicit /pp/enable; auto_start must be false");
    if (reference.empty() || vehicle.empty())
      throw std::invalid_argument(
          "reference_directory and vehicle_config are required");
    runtime_ = std::make_unique<aims_pp::Runtime>(
        reference, aims_pp::load_config(vehicle, tuning));
    runtime_->configure_tracking(speed, repeat);
    log_ = std::make_unique<RuntimeLog>(log);
    auto reliable = rclcpp::QoS(1).reliable();
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    drive_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", reliable);
    status_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/pp/status", 10);
    reference_ =
        create_publisher<nav_msgs::msg::Path>("/pp/reference", latched);
    lookahead_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/pp/lookahead_point", reliable);
    subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", rclcpp::QoS(1).best_effort(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          accept("odometry", [&](int64_t n, double t) {
            auto &p = m->pose.pose.position;
            auto &q = m->pose.pose.orientation;
            runtime_->observe_odometry(
                {{"x", p.x},
                 {"y", p.y},
                 {"quaternion", json::array({q.x, q.y, q.z, q.w})},
                 {"velocity", m->twist.twist.linear.x},
                 {"stamp_ns", stamp(m->header.stamp)},
                 {"frame_id", m->header.frame_id},
                 {"child_frame_id", m->child_frame_id}},
                n, t);
          });
        }));
    subscriptions_.push_back(
        create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/localization/status", 10,
            [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
              accept("health", [&](int64_t n, double t) {
                json values = json::object();
                double age = (n - stamp(m->header.stamp)) * 1e-9;
                size_t count = 0;
                const diagnostic_msgs::msg::DiagnosticStatus *record = nullptr;
                for (auto &r : m->status)
                  if (r.name == "aims_racer_system/localization") {
                    count++;
                    record = &r;
                  }
                if (count == 1 && age >= 0 && age <= .3) {
                  for (auto &kv : record->values) {
                    if (values.contains(kv.key)) {
                      values = json::object();
                      break;
                    }
                    values[kv.key] = kv.value;
                  }
                }
                runtime_->observe_health(values, n, t);
              });
            }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        "/localization/map_sha256", latched,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          accept("map", [&](int64_t, double) {
            runtime_->observe_map_identity(m->data);
          });
        }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
        "/localization/map_valid", latched,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          accept("map_valid", [&](int64_t, double) {
            runtime_->observe_map_valid(m->data);
          });
        }));
    subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(
        "/control/autonomy_speed_enabled", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          accept("authority", [&](int64_t, double t) {
            runtime_->observe_authority(m->data, t);
          });
        }));
    subscriptions_.push_back(create_subscription<vesc_msgs::msg::VescCommand>(
        "/vesc/command", reliable,
        [this](vesc_msgs::msg::VescCommand::ConstSharedPtr m,
               const rclcpp::MessageInfo &info) {
          accept("final_command", [&](int64_t n, double t) {
            auto source = info.get_rmw_message_info().source_timestamp;
            if (source > 0) {
              double age =
                  (std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count() -
                   source) *
                  1e-9;
              if (!(age >= 0 && age <= .1)) {
                runtime_->observe_command(json::object(), n, t);
                throw std::invalid_argument("stale final VESC DDS source");
              }
            }
            runtime_->observe_command(
                {{"control_mode", m->control_mode},
                 {"steering", m->steering_angle},
                 {"speed", m->target_speed_mps},
                 {"stamp_ns", stamp(m->header.stamp)},
                 {"frame_id", m->header.frame_id},
                 {"targets",
                  json::array({m->target_speed_mps, m->target_current,
                               m->target_duty, m->target_brake_current,
                               m->target_acceleration_mps2})}},
                n, t);
          });
        }));
    enable_ = create_service<std_srvs::srv::SetBool>(
        "/pp/enable", [this](std_srvs::srv::SetBool::Request::SharedPtr req,
                             std_srvs::srv::SetBool::Response::SharedPtr res) {
          auto [n, t] = clocks();
          if (req->data) {
            auto out = runtime_->enable(n, t, count_publishers("/drive"));
            res->success = out["success"];
            res->message = out["message"];
          } else {
            runtime_->disable();
            res->success = true;
            res->message = runtime_->status(n, t)["reason"];
          }
          if (runtime_->status(n, t)["enabled"] == false)
            publish({{"speed", 0.}, {"steering", 0.}}, n, t);
          log_->submit({{"event", req->data ? "enable" : "disable"},
                        {"ros_ns", n},
                        {"steady_s", t},
                        {"phase", runtime_->status(n, t)["phase"]},
                        {"reason", res->message}});
        });
    tick_timer_ =
        create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
    status_timer_ =
        create_wall_timer(std::chrono::milliseconds(100), [this] { status(); });
    guard_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &params) {
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = true;
          for (auto &p : params)
            if (p.get_name() != "use_sim_time") {
              r.successful = false;
              r.reason = "PP configuration is fixed; restart with matching "
                         "reference and vehicle";
            }
          return r;
        });
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (auto at : runtime_->reference_points()) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = at["x"];
      p.pose.position.y = at["y"];
      double yaw = at["yaw"];
      p.pose.orientation.z = std::sin(yaw / 2);
      p.pose.orientation.w = std::cos(yaw / 2);
      path.poses.push_back(p);
    }
    reference_->publish(path);
    RCLCPP_INFO(get_logger(), "Enhanced PP ready: native C++17, 50 Hz, "
                              "disabled, explicit enable required");
  }
  ~PPNode() {
    if (runtime_)
      runtime_->disable();
    if (drive_ && rclcpp::ok()) {
      auto [n, t] = clocks();
      publish({{"speed", 0.}, {"steering", 0.}}, n, t);
    }
    if (log_)
      log_->close();
  }
};
} // namespace
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<PPNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception &e) {
    std::cerr << "PP startup/runtime failure: " << e.what() << '\n';
    result = 1;
  }
  if (rclcpp::ok())
    rclcpp::shutdown();
  return result;
}
