#pragma once
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <map>

namespace aims_mpcc_rt {
using State = std::array<double, 6>;
using Control = std::array<double, 3>;
using Applied = std::array<double, 3>;
using Alignment = std::array<double, 3>;  // odom -> map [tx,ty,yaw]
using InternalState = std::array<double, 9>;
using Parameters = std::array<double, 10>;

struct Config {
  std::string profile;
  std::string command_profile{"legacy_bounded_v1"};
  double wheelbase{}, rear_offset{}, half_width{}, front_extent{}, rear_extent{};
  double cruise_speed{}, max_speed{}, minimum_drive_speed{}, steer_limit{}, steer_rate{};
  double steering_acceleration_scale{};
  double steer_acceleration{}, accel_limit{}, brake_limit{}, jerk_limit{}, steering_tau{};
  double understeer_coefficient{}, envelope_accel{}, envelope_brake{}, lateral_accel_limit{};
  double recovery_jerk_limit{}, envelope_recovery_time{}, envelope_slack_limit{};
  bool enforce_corridor{}, envelope_soft_enabled{}, recovery_jerk_enabled{};
  int horizon{};
  int acados_rti_steps{1};
  double dt{};
};

struct ReferencePoint {double x{}, y{}, yaw{}, curvature{}, tangent_norm{};};
class Reference {
 public:
  ReferencePoint at(double theta) const;
  double project(double x, double y) const;
  double speed_at(double theta) const;
  std::vector<double> speed_refs(double theta, int horizon, double dt) const;
  double length() const {return length_;}
  const std::string &frame_id() const {return frame_;}
  const std::string &map_sha256() const {return map_hash_;}
  double left_width() const {return left_width_;}
  double right_width() const {return right_width_;}
  bool closed_lap() const {return closed_lap_;}
  bool recording_verified() const {return recording_verified_;}
 private:
  friend class Bundle;
  double length_{}, left_width_{}, right_width_{};
  std::string frame_, map_hash_;
  bool closed_lap_{}, recording_verified_{};
  std::vector<double> knots_;
  std::vector<double> speed_positions_, speeds_;
  std::vector<std::array<std::array<double, 2>, 6>> coefficients_;
};

class Bundle {
 public:
  static Bundle load(const std::string &directory, const std::string &expected_config_path="",
                     const std::string &expected_reference_dir="", const std::string &source_root="");
  const Config &config() const;
  const Reference &reference() const;
  const std::string &fingerprint() const;
  // Generated functions for deterministic model/cost/constraint parity checks.
  std::vector<double> evaluate(int kind, const InternalState &, const std::vector<double> &,
                               const Parameters &) const;
 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  friend class Core;
};

struct Plan {
  bool success{};
  int status{-1}, native_passes{}, geometry_refreshes{};
  double max_geometry_progress_shift{}, geometry_refresh_time_s{};
  std::map<std::string,double> constraint_violations;
  std::vector<State> states;
  std::vector<Control> controls;
  double dt{}, source_epoch{}, forecast_epoch{}, solve_time_s{}, native_time_s{};
  double preparation_time_s{}, validation_time_s{};
  double max_violation{}, cost{};
  double native_cost{}, raw_optimizer_cost{};
  double reanchor_time_s{};
  bool prefix_transported{};
  Alignment map_alignment{};
  Applied initial_applied{};
  std::vector<double> speed_targets;
  std::string artifact_fingerprint;
  std::string reason;
};

class Core {
 public:
  explicit Core(Bundle);
  ~Core();
  Core(const Core &) = delete;
  Core &operator=(const Core &) = delete;
  const Config &config() const {return bundle_.config();}
  const Reference &reference() const {return bundle_.reference();}
  Plan solve(State initial, const Applied &, const Alignment &, double elapsed_s,
             double budget_s=.05, bool allow_second_rti=false,
             double source_epoch=0., double forecast_epoch=0.,
             const std::vector<double> &speed_targets={}, bool refresh_second_geometry=true);
  Plan reanchor(const Plan &, State actual, const Applied &actual_prefix, double actual_epoch,
                const std::optional<Alignment> &current_alignment=std::nullopt) const;
  void reset();
  State transition(State, const Control &, double previous_endpoint, double duration) const;
  // Control samples interpolate by elapsed time, including fractional stages.
  static Control interpolate(const std::vector<Control> &, double dt, double elapsed);
 private:
  Plan validate_candidate(const State &, const Applied &, const Alignment &,
                          const std::vector<std::vector<double>> &,
                          const std::vector<Parameters> &, Plan) const;
  Bundle bundle_;
  void *capsule_{};
  Plan previous_;
  double previous_elapsed_{};
  bool have_progress_{};
  double previous_theta_{}, previous_yaw_{};
};
}  // namespace aims_mpcc_rt
