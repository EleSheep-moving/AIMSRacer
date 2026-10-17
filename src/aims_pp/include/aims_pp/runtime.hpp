#pragma once
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
namespace aims_pp {
using json = nlohmann::json;
// Effective config is dataclasses.asdict(PPConfig), with its defaults accepted.
json load_config(const std::string &vehicle_yaml,
                 const std::string &tuning_yaml = "");
class Runtime {
public:
  Runtime(const std::string &reference_directory,
          const json &effective_pp_config, bool allow_reverse_velocity = false,
          bool allow_speed_overshoot = false);
  ~Runtime();
  Runtime(Runtime &&) noexcept;
  Runtime &operator=(Runtime &&) noexcept;
  Runtime(const Runtime &) = delete;
  Runtime &operator=(const Runtime &) = delete;
  void observe_clock(int64_t now);
  // Values: x,y,yaw,velocity,stamp_ns; optional
  // frame_id=odom,child_frame_id=base_link. Alternatively quaternion
  // [qx,qy,qz,qw] instead of yaw (unit checked).
  bool observe_odometry(const json &values, int64_t now, double steady);
  bool observe_health(const json &protocol_v1_string_values, int64_t now,
                      double steady);
  void observe_map_identity(const std::string &hash);
  void observe_map_valid(bool valid);
  void observe_authority(bool enabled, double steady);
  // control_mode,steering,speed,stamp_ns; frame_id=base_link; targets optional
  // 5 values.
  void observe_command(const json &values, int64_t now, double steady);
  void invalidate(const std::string &reason);
  // snapshot: reason,pose=[map rear x,y,yaw,forward v] or null,
  // fault_generation,
  // fault_reason,map_sha256,anchor_epoch,anchor_sequence,source_age_s.
  // require_anchor=false permits qualified alignment without anchor freshness
  // only.
  json snapshot(int64_t now, double steady, bool require_anchor = true) const;
  // association/evaluate: pose,progress,lateral_error,heading_error,reference,
  // evaluate additionally
  // steering,lookahead,debug,speed,reason,fault_generation,fault_reason.
  json associate(const json &pose, std::optional<double> expected_s = {},
                 std::optional<double> max_progress_delta = {}) const;
  json evaluate(double target_speed, int64_t now, double steady,
                bool require_anchor = true) const;
  json reference_at(double s) const;
  json project(const json &xy) const;
  json steering(const json &values) const;
  const json &reference_metadata() const;
  const json &effective_config() const;
  double total_length() const;
  std::string reference_checksum() const;
  json reference_points() const;
  void configure_tracking(double speed_limit, bool repeat_laps);
  void reset_controller();
  json enable(int64_t now, double steady, int publisher_count = 1);
  void disable();
  json tick(int64_t now, double steady, int publisher_count = 1);
  json status(int64_t now, double steady, int publisher_count = 1) const;

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace aims_pp
