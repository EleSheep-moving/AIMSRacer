// Record publisher identity for the two dynamic localization TF edges.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
std::int64_t steady_nanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string gid_hex(const rmw_gid_t & gid)
{
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (const auto byte : gid.data) {
    result << std::setw(2) << static_cast<unsigned>(byte);
  }
  return result.str();
}

class TfAuthorityAudit : public rclcpp::Node
{
public:
  explicit TfAuthorityAudit(const std::string & positional_path)
  : Node("tf_authority_audit")
  {
    const auto parameter_path = declare_parameter<std::string>("audit_path", "");
    if (!parameter_path.empty() && !positional_path.empty() && parameter_path != positional_path) {
      throw std::invalid_argument("positional path and audit_path parameter differ");
    }
    const auto path = positional_path.empty() ? parameter_path : positional_path;
    if (path.empty()) {
      throw std::invalid_argument("provide an audit JSONL path or -p audit_path:=PATH");
    }
    if (std::filesystem::exists(path)) {
      throw std::invalid_argument("audit_path already exists; select a new output file");
    }
    output_.open(path, std::ios::out);
    if (!output_) {
      throw std::runtime_error("cannot open audit_path: " + path);
    }
    output_ << "{\"event\":\"startup\",\"node\":\"tf_authority_audit\","
            << "\"receive_steady_ns\":" << steady_nanoseconds() << "}\n";
    output_.flush();
    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100).reliable().durability_volatile(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        const auto gid = gid_hex(info.get_rmw_message_info().publisher_gid);
        for (const auto & transform : message->transforms) {
          const auto & parent = transform.header.frame_id;
          const auto & child = transform.child_frame_id;
          if (!((parent == "map" && child == "odom") ||
            (parent == "odom" && child == "base_link")))
          {
            continue;
          }
          const auto & stamp = transform.header.stamp;
          const auto stamp_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
          output_ << "{\"event\":\"tf\",\"publisher_gid\":\"" << gid
                  << "\",\"frame_id\":\"" << parent << "\",\"child_frame_id\":\"" << child
                  << "\",\"stamp_ns\":" << stamp_ns
                  << ",\"stamp_sec\":" << stamp.sec << ",\"stamp_nanosec\":" << stamp.nanosec
                  << ",\"receive_ros_ns\":" << now().nanoseconds()
                  << ",\"receive_steady_ns\":" << steady_nanoseconds() << "}\n";
        }
        output_.flush();
        if (!output_) {
          throw std::runtime_error("TF audit write failed");
        }
      });
    RCLCPP_INFO(get_logger(), "Recording localization TF publisher GIDs in %s", path.c_str());
  }

private:
  std::ofstream output_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
};
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    if (arguments.size() > 2) {
      throw std::invalid_argument("usage: tf_authority_audit [new-output.jsonl] [--ros-args ...]");
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfAuthorityAudit>(arguments.size() == 2 ? arguments[1] : "");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "TF authority audit failed: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
