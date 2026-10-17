#include "aims_calibration/native_engine.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

namespace aims_calibration {
namespace {
using J = nlohmann::json;
const double NaN = std::numeric_limits<double>::quiet_NaN();
double n(const J &j, const char *k, double fallback = NaN) {
  auto it = j.find(k);
  return it != j.end() && it->is_number() ? it->get<double>() : fallback;
}
double n(const J &j) { return j.is_number() ? j.get<double>() : NaN; }
bool present(const J &j, const char *k) {
  return j.contains(k) && !j[k].is_null();
}
bool yes(const J &j, const char *k) {
  return j.contains(k) && j[k].is_boolean() && j[k].get<bool>();
}
std::string str(const J &j, const char *k, std::string fallback = "") {
  return j.contains(k) && j[k].is_string() ? j[k].get<std::string>() : fallback;
}
J number(double v) { return std::isfinite(v) ? J(v) : J(nullptr); }
bool close(double a, double b, double atol = 1e-5) {
  return std::abs(a - b) <=
         std::max(atol, 1e-9 * std::max(std::abs(a), std::abs(b)));
}
double ulp(double a) {
  return std::abs(std::nextafter(a, std::numeric_limits<double>::infinity()) -
                  a);
}
double eps(std::initializer_list<double> v) {
  double out = 0;
  for (double x : v)
    out = std::max(out, ulp(x));
  return 4 * out;
}
double mean(const std::vector<double> &v) {
  double s = 0;
  for (double a : v)
    s += a;
  return v.empty() ? NaN : s / v.size();
}
double sd(const std::vector<double> &v) {
  double m = mean(v), s = 0;
  for (double a : v)
    s += (a - m) * (a - m);
  return v.empty() ? NaN : std::sqrt(s / v.size());
}
bool in(std::string v, std::initializer_list<const char *> a) {
  for (auto x : a)
    if (v == x)
      return true;
  return false;
}
bool corresponds(const J &a, const J &b) {
  return n(a, "mode") == n(b, "mode") &&
         close(n(a, "steering"), n(b, "steering")) &&
         close(n(a, "speed"), n(b, "speed")) &&
         close(n(a, "current"), n(b, "current"));
}
} // namespace
struct NativeEngine::Impl {
  J c, tr, p, b, limits, tp, s, wheel = J::array(), steady_wheel = J::array(),
                                steady_iq = J::array();
  J acceleration = nullptr, instant = nullptr, steady_last_wheel = nullptr,
    steady_last_current = nullptr, steady_context = nullptr,
    confirmation = nullptr;
  bool full = false, protected_flow = false;
  std::string stage() const { return str(c, "stage"); }
  std::string state() const { return str(s, "state"); }
  const J &item() const { return tp.at(static_cast<size_t>(n(s, "index"))); }
  size_t index() const { return static_cast<size_t>(n(s, "index")); }
  void state(const char *v) { s["state"] = v; }
  bool fresh(double now, double stamp) const {
    return std::isfinite(stamp) && stamp > 0 && 0 <= now - stamp &&
           now - stamp <= n(c, "freshness");
  }
  Impl(const J &prepared)
      : c(prepared.at("config")), tr(prepared.at("tracking")),
        p(prepared.value("protection", J(nullptr))),
        b(prepared.value("baseline", J(nullptr))),
        limits(prepared.value("response_limits", J(nullptr))),
        tp(prepared.value("plan", J::array())) {
    full = str(c, "sweep_protocol") == "full_range_step";
    protected_flow =
        yes(c, "predictive_exit") || yes(c, "measured_current_handoff");
    if (protected_flow &&
        (p.is_null() || b.is_null() || stage() == "speed_hold"))
      throw std::invalid_argument("current handoff/exit protection requires a "
                                  "B/C baseline and protection object");
    if (!protected_flow)
      p = nullptr;
    s = {{"config", c},
         {"tracking_config", tr},
         {"protection_settings", p},
         {"state", "IDLE"},
         {"reason", "disabled"},
         {"enabled", false},
         {"selected", false},
         {"index", 0},
         {"history", J::array()},
         {"samples", J::array()},
         {"trials", J::array()},
         {"skipped", J::array()},
         {"results_revision", 0},
         {"last_now", nullptr},
         {"armed_at", nullptr},
         {"attempt_counts", J::object()},
         {"attempt_id", nullptr},
         {"attempt_number", 0},
         {"decel_owner_index", nullptr},
         {"group_recovery_started", nullptr},
         {"group_recovery_request_stamp", nullptr},
         {"group_recovery_ack_stamp", nullptr},
         {"group_forward_since", nullptr},
         {"group_iq_samples", 0},
         {"group_iq_stamp", nullptr},
         {"handoff", nullptr},
         {"exit_prediction", nullptr},
         {"retry_region", nullptr},
         {"retry_left", false},
         {"full_step", full}};
    J plan = J::array();
    if (stage() == "speed_hold") {
      tp = J::array();
      for (auto v : c.at("speeds"))
        for (int r = 0; r < n(c, "repeats"); ++r)
          plan.push_back(v);
    } else {
      if (tp.empty())
        for (auto current : c.at("currents"))
          for (int r = 1; r <= n(c, "repeats"); ++r)
            tp.push_back({{"stage", stage()},
                          {"start_speed", c.at("start_speed")},
                          {"end_speed", c.at("end_speed")},
                          {"current", current},
                          {"repeat", r},
                          {"kind", stage() == "accel_interval" ? "acceleration"
                                                               : "braking"},
                          {"baseline_current", nullptr}});
      for (auto &v : tp) {
        if (!v.contains("repeat"))
          v["repeat"] = 1;
        if (!v.contains("baseline_current"))
          v["baseline_current"] = nullptr;
        plan.push_back(v.at("current"));
      }
    }
    s["plan"] = plan;
    s["trial_plan"] = tp;
    s["serializable_plan"] = tp;
    for (size_t i = 0; i < tp.size(); ++i)
      s["serializable_plan"][i]["plan_index"] = i;
    reset();
  }
  void steady_reset() {
    steady_wheel = J::array();
    steady_iq = J::array();
    steady_last_wheel = nullptr;
    steady_last_current = nullptr;
    steady_context = nullptr;
    confirmation = nullptr;
  }
  void protect_reset() {
    wheel = J::array();
    acceleration = nullptr;
    instant = nullptr;
  }
  void reset() {
    steady_reset();
    for (auto k :
         {"last_prepare_wheel", "exit_prediction", "stable_since", "started",
          "start_velocity", "last_pair", "region", "last_wheel",
          "last_telemetry", "phase_started", "phase_ack_stamp", "hold_previous",
          "hold_segment", "first_current_request_stamp",
          "starting_wheel_observation", "start_trigger_observation",
          "last_tracking_stamp", "tracking_started_stamp"})
      s[k] = nullptr;
    for (auto k : {"steady_evidence", "tracking_evidence"})
      s[k] = J::object();
    for (auto k : {"active_samples", "hold_segments", "tracking_window",
                   "tracking_observations"})
      s[k] = J::array();
    s["hold_elapsed"] = 0.;
    s["tracking_status"] = "unverified";
  }
  void revision() {
    s["results_revision"] = s["results_revision"].get<std::size_t>() + 1;
  }
  J request(double now, const J &o, int mode = 0, double speed = 0,
            double current = 0) const {
    return {{"stamp", now},
            {"mode", mode},
            {"steering", in(state(), {"IDLE", "STOPPING", "FAULT", "COMPLETE"})
                             ? 0.
                             : n(o, "steering", 0.)},
            {"speed", speed},
            {"current", current}};
  }
  void pause_hold(const std::string &reason) {
    if (present(s, "hold_segment")) {
      s["hold_segment"]["end_reason"] = reason;
      s["hold_segments"].back() = s["hold_segment"];
    }
    s["hold_previous"] = nullptr;
    s["hold_segment"] = nullptr;
    s["stable_since"] = nullptr;
    s["phase_started"] = nullptr;
    s["region"] = nullptr;
    s["reason"] = reason;
  }
  void sample_hold(const J &o, J &row) {
    if (!present(s, "hold_segment")) {
      s["hold_segment"] = {{"segment_id", s["hold_segments"].size() + 1},
                           {"region", o.at("region")},
                           {"start_stamp", o.at("wheel_stamp")},
                           {"end_stamp", o.at("wheel_stamp")},
                           {"start_telemetry_stamp", o.at("telemetry_stamp")},
                           {"end_telemetry_stamp", o.at("telemetry_stamp")},
                           {"duration_s", 0.},
                           {"sample_offset", s["active_samples"].size()},
                           {"samples", 0},
                           {"end_reason", "sampling"}};
      s["hold_segments"].push_back(s["hold_segment"]);
    }
    auto &h = s["hold_segment"];
    if (present(s, "hold_previous")) {
      double duration = n(o, "wheel_stamp") - n(s["hold_previous"][0]);
      s["hold_elapsed"] = n(s, "hold_elapsed") + duration;
      h["duration_s"] = n(h, "duration_s") + duration;
    }
    s["hold_previous"] =
        J::array({o.at("wheel_stamp"), o.at("telemetry_stamp")});
    h["end_stamp"] = o.at("wheel_stamp");
    h["end_telemetry_stamp"] = o.at("telemetry_stamp");
    h["samples"] = h["samples"].get<std::size_t>() + 1;
    row["segment_id"] = h["segment_id"];
    s["hold_segments"].back() = h;
  }
  void begin_attempt() {
    auto key = std::to_string(index());
    s["attempt_counts"][key] =
        static_cast<std::size_t>(n(s["attempt_counts"], key.c_str(), 0)) + 1;
    s["attempt_number"] = s["attempt_counts"][key];
    s["attempt_id"] = s["trials"].size() + 1;
    s["tracking_status"] =
        str(item(), "kind") == "coast" ? "not_applicable" : "unverified";
  }
  void advance(const std::string &reason) {
    s["retry_region"] = nullptr;
    s["retry_left"] = false;
    s["index"] = index() + 1;
    reset();
    state(index() == s["plan"].size() ? "COMPLETE" : "PREPARE");
    s["reason"] = reason;
  }
  void finish(bool success, const std::string &reason, J end = J(nullptr)) {
    if (!present(s, "started") && s["active_samples"].empty() &&
        !in(state(), {"MEASURE", "APPLY"}))
      return;
    J last = !end.is_null() && !end.empty()
                 ? end
                 : (!s["active_samples"].empty() ? s["active_samples"].back()
                                                 : J::object());
    double ended = n(last, "wheel_stamp"), velocity = n(last, "velocity"),
           duration = ended - n(s, "started");
    std::vector<double> currents, iq;
    for (auto &row : s["active_samples"]) {
      currents.push_back(n(row, "current_motor"));
      iq.push_back(n(row, "avg_iq"));
    }
    bool cumulative =
        stage() == "speed_hold" && str(c, "hold_sampling") == "cumulative";
    J segments = J::array();
    if (cumulative)
      for (auto seg : s["hold_segments"]) {
        size_t offset = n(seg, "sample_offset"), count = n(seg, "samples");
        seg.erase("sample_offset");
        std::vector<double> cv(currents.begin() + offset,
                               currents.begin() + offset + count),
            qv(iq.begin() + offset, iq.begin() + offset + count);
        seg.update({{"current_signal", "avg_iq"},
                    {"mean_current_motor", number(mean(cv))},
                    {"std_current_motor", number(sd(cv))},
                    {"mean_iq", number(mean(qv))},
                    {"std_iq", number(sd(qv))}});
        if (str(seg, "end_reason") == "sampling")
          seg["end_reason"] = reason;
        segments.push_back(seg);
      }
    J result = {
        {"current_signal", "avg_iq"},
        {"trial_id", s["trials"].size() + 1},
        {"plan_index", index()},
        {"stage", stage()},
        {"requested_target", s["plan"].at(index())},
        {"region", s["region"]},
        {"success", success},
        {"reason", reason},
        {"first_current_ack_stamp", s["phase_ack_stamp"]},
        {"evidence_valid", true},
        {"start_stamp", s["started"]},
        {"end_stamp", number(ended)},
        {"start_speed", s["start_velocity"]},
        {"end_speed", number(velocity)},
        {"duration_s", cumulative ? s["hold_elapsed"] : number(duration)},
        {"samples", currents.size()},
        {"mean_current_motor", number(mean(currents))},
        {"std_current_motor", number(sd(currents))},
        {"mean_iq", number(mean(iq))},
        {"std_iq", number(sd(iq))},
        {"mean_acceleration",
         number(success && stage() != "speed_hold" && duration > 0
                    ? (velocity - n(s, "start_velocity")) / duration
                    : NaN)}};
    if (stage() == "speed_hold")
      result.update({{"hold_sampling", c.at("hold_sampling")},
                     {"wall_duration_s", number(duration)},
                     {"segment_count", segments.size()},
                     {"segments", segments}});
    else {
      auto &it = item();
      result.update(
          {{"attempt_id", s["attempt_id"]},
           {"attempt_number", s["attempt_number"]},
           {"repeat", it.at("repeat")},
           {"kind", it.at("kind")},
           {"current", it.at("current")},
           {"start_target_speed", it.at("start_speed")},
           {"end_target_speed", it.at("end_speed")},
           {"baseline_current", it.at("baseline_current")},
           {"first_current_request_stamp", s["first_current_request_stamp"]},
           {"starting_wheel_observation", s["starting_wheel_observation"]},
           {"response_duration_s",
            number(ended - n(s, "first_current_request_stamp"))},
           {"tracking_status", s["tracking_status"]},
           {"tracking_evidence", s["tracking_evidence"]},
           {"tracking_observations", s["tracking_observations"]},
           {"uncovered", false}});
    }
    if (protected_flow && stage() != "speed_hold")
      result.update(
          {{"exit_prediction", s["exit_prediction"]},
           {"pulse_mean_acceleration",
            number(duration > 0 ? (velocity - n(s, "start_velocity")) / duration
                                : NaN)},
           {"interval_completed", success},
           {"protective_flow", true},
           {"pulse_valid", in(reason, {"completed", "predictive_exit"}) &&
                               duration > 0 && currents.size() >= 2}});
    if (full) {
      if (stage() == "decel_current")
        result["start_trigger_observation"] = s["start_trigger_observation"];
      bool normal = in(reason, {"speed_boundary", "steady_state"}),
           observed = currents.size() >= 2 && present(s, "phase_ack_stamp");
      std::vector<double> velocities;
      for (auto &row : s["active_samples"])
        velocities.push_back(n(row, "velocity"));
      if (std::isfinite(velocity))
        velocities.push_back(velocity);
      result.update(
          {{"stop_reason", reason},
           {"data_status",
            success && normal && observed
                ? "complete"
                : (!currents.empty() ? "partial" : "uncollected")},
           {"endpoint_reached",
            success && reason == "speed_boundary" && observed},
           {"interval_completed",
            success && reason == "speed_boundary" && observed},
           {"pulse_valid", success && normal && observed},
           {"steady_evidence", s["steady_evidence"]},
           {"attempts_exhausted", false},
           {"wheel_speed_min",
            velocities.empty()
                ? J(nullptr)
                : J(*std::min_element(velocities.begin(), velocities.end()))},
           {"wheel_speed_max",
            velocities.empty()
                ? J(nullptr)
                : J(*std::max_element(velocities.begin(), velocities.end()))}});
    }
    s["trials"].push_back(result);
  }
  void fault(const std::string &reason) {
    if (present(s, "handoff") && in(state(), {"UNLOAD", "HOLD"})) {
      s["handoff"].update({{"failure", reason}, {"hold_confirmed", false}});
      s["trials"].back()["handoff"] = s["handoff"];
      revision();
    }
    if (state() != "FAULT")
      finish(false, reason);
    state("FAULT");
    s["reason"] = reason;
    s["enabled"] = false;
    reset();
  }
  void enable(bool value, double now) {
    if (present(s, "handoff") && in(state(), {"UNLOAD", "HOLD"})) {
      s["handoff"].update(
          {{"failure", value ? "operator_reenabled" : "operator_disabled"},
           {"hold_confirmed", false}});
      s["trials"].back()["handoff"] = s["handoff"];
      revision();
    }
    if (present(s, "started") || !s["active_samples"].empty() ||
        (stage() != "speed_hold" && state() == "APPLY"))
      finish(false, value ? "operator_reenabled" : "operator_disabled");
    s["enabled"] = value;
    s["selected"] = false;
    reset();
    s["history"] = J::array();
    s["handoff"] = nullptr;
    s["exit_prediction"] = nullptr;
    s["retry_region"] = nullptr;
    s["retry_left"] = false;
    if (protected_flow)
      protect_reset();
    s["decel_owner_index"] = nullptr;
    for (auto k : {"group_recovery_started", "group_recovery_request_stamp",
                   "group_recovery_ack_stamp", "group_forward_since"})
      s[k] = nullptr;
    state(value ? "PREPARE" : "STOPPING");
    s["reason"] = value ? "waiting_for_selection" : "operator_disabled";
    if (value) {
      s["index"] = 0;
      s["last_now"] = nullptr;
      s["armed_at"] = number(now);
      s["attempt_counts"] = J::object();
    }
  }
  void sent(const J &r) {
    s["history"].push_back(r);
    if (s["history"].size() > 24)
      s["history"].erase(s["history"].begin());
    if (full && present(s, "handoff")) {
      auto field = state() == "UNLOAD" ? "withdraw_request_stamp"
                                       : "recovery_request_stamp";
      if (!s["handoff"].contains(field)) {
        s["handoff"][field] = r.at("stamp");
        s["trials"].back()["handoff"] = s["handoff"];
        revision();
      }
    }
    if (state() == "GROUP_RECOVER" && n(r, "mode") == 1 &&
        n(r, "current") == 0 && !present(s, "group_recovery_request_stamp"))
      s["group_recovery_request_stamp"] = r.at("stamp");
    if (stage() != "speed_hold" && in(state(), {"APPLY", "MEASURE"}) &&
        n(r, "mode") == 1 && close(n(r, "current"), n(item(), "current")) &&
        !present(s, "first_current_request_stamp")) {
      s["first_current_request_stamp"] = r.at("stamp");
      if (stage() == "decel_current")
        s["decel_owner_index"] = index();
    }
  }
  void observe(double stamp, double velocity) {
    if (!wheel.empty() && stamp <= n(wheel.back()[0]))
      return;
    if (!wheel.empty() && stamp - n(wheel.back()[0]) > n(p, "source_gap"))
      protect_reset();
    if (!wheel.empty())
      instant = (velocity - n(wheel.back()[1])) / (stamp - n(wheel.back()[0]));
    wheel.push_back(J::array({stamp, velocity}));
    while (wheel.size() > 2 &&
           n(wheel[1][0]) < stamp - n(p, "derivative_window"))
      wheel.erase(wheel.begin());
    if (wheel.size() > 1)
      acceleration = (velocity - n(wheel[0][1])) / (stamp - n(wheel[0][0]));
  }
  double holding(double speed) const {
    if (b.contains("planning_model") && !b["planning_model"].is_null()) {
      const auto &m = b.at("planning_model");
      if (!(n(m.at("speed_domain_mps")[0]) <= speed &&
            speed <= n(m.at("speed_domain_mps")[1])))
        throw std::invalid_argument("speed outside polynomial baseline domain");
      double cur = 0;
      for (auto it = m.at("coefficients").rbegin();
           it != m.at("coefficients").rend(); ++it)
        cur = cur * speed + n(*it);
      return cur;
    }
    double left = -std::numeric_limits<double>::infinity(),
           right = std::numeric_limits<double>::infinity(), lv = NaN, rv = NaN;
    for (auto &pt : b.at("points")) {
      double v = n(pt, "speed"), iq = n(pt, "mean_iq");
      if (v == speed)
        return iq;
      if (v < speed && v > left) {
        left = v;
        lv = iq;
      }
      if (v > speed && v < right) {
        right = v;
        rv = iq;
      }
    }
    if (!full || !std::isfinite(left) || !std::isfinite(right))
      throw std::invalid_argument("holding speed not covered by Stage A");
    return lv + (rv - lv) * (speed - left) / (right - left);
  }
  J prediction(double now, const J &o) {
    double bound = NaN;
    for (auto &row : limits.at("limits"))
      if (str(row, "stage") == stage() &&
          close(n(row, "start_speed"), n(item(), "start_speed"), 1e-6) &&
          close(n(row, "current"), n(item(), "current"), 1e-4))
        bound = std::isfinite(bound)
                    ? std::max(bound, n(row, "acceleration_bound"))
                    : n(row, "acceleration_bound");
    if (!std::isfinite(bound) || bound <= 0)
      throw std::invalid_argument("response_limit_unavailable");
    double sign = stage() == "accel_interval" ? 1 : -1, rate = bound;
    for (auto a : {acceleration, instant})
      if (a.is_number())
        rate = std::max(rate, sign * n(a));
    double horizon =
               n(p, "exit_horizon") + std::max(0., now - n(o, "wheel_stamp")),
           predicted = n(o, "velocity") + sign * rate * horizon;
    return {{"wheel_stamp", o.at("wheel_stamp")},
            {"velocity", o.at("velocity")},
            {"wheel_acceleration", acceleration},
            {"instantaneous_acceleration", instant},
            {"response_limit", bound},
            {"directional_rate", rate},
            {"horizon_s", horizon},
            {"predicted_velocity", predicted},
            {"target_speed", item().at("end_speed")},
            {"observer_acceleration", o.at("acceleration")},
            {"observer_quality", o.at("acceleration_quality")},
            {"exits", sign * (predicted - n(item(), "end_speed")) >= 0}};
  }
  J empty_steady(const std::string &reason, J ws = nullptr, J ts = nullptr) {
    return {{"steady", false},
            {"reason", reason},
            {"slope", nullptr},
            {"speed_span", nullptr},
            {"sample_count", steady_wheel.size()},
            {"current_sample_count", steady_iq.size()},
            {"current_good_fraction", nullptr},
            {"wheel_stamp", ws},
            {"telemetry_stamp", ts},
            {"window_start_stamp", nullptr},
            {"current_window_start_stamp", nullptr},
            {"confirmation_start_stamp", nullptr},
            {"confirmed_seconds", 0.}};
  }
  static void trim(J &ring, double duration) {
    double cutoff = n(ring.back()[0]) - duration;
    while (ring.size() > 1 &&
           n(ring[1][0]) <= cutoff + eps({cutoff, n(ring[1][0])}))
      ring.erase(ring.begin());
  }
  J steady_observe(const J &o, double target, bool mode) {
    double ws = n(o, "wheel_stamp"), v = n(o, "velocity"),
           ts = n(o, "telemetry_stamp"), iq = n(o, "avg_iq"),
           ack = n(s, "phase_ack_stamp");
    if (!std::isfinite(ws) || !std::isfinite(v) || !std::isfinite(ts) ||
        !std::isfinite(iq) || !std::isfinite(target) || !std::isfinite(ack) ||
        !mode) {
      steady_reset();
      return empty_steady("invalid_sample_or_mode");
    }
    J context = J::array({ack, target});
    if (context != steady_context) {
      steady_reset();
      steady_context = context;
    }
    if ((!steady_last_wheel.is_null() && ws < n(steady_last_wheel)) ||
        (!steady_last_current.is_null() && ts < n(steady_last_current))) {
      steady_reset();
      return empty_steady("backwards_source", ws, ts);
    }
    // SteadySettings.source_gap is independent of
    // ProtectionSettings.source_gap. steady_settings(config, tracking)
    // preserves its fixed default.
    double gap = .08;
    bool discontinuity = (!steady_last_wheel.is_null() &&
                          ws - n(steady_last_wheel) >
                              gap + eps({ws, n(steady_last_wheel)})) ||
                         (!steady_last_current.is_null() &&
                          ts - n(steady_last_current) >
                              gap + eps({ts, n(steady_last_current)})),
         stale = std::abs(ws - ts) > gap + eps({ws, ts});
    if (discontinuity || stale) {
      steady_wheel = J::array();
      steady_iq = J::array();
      confirmation = nullptr;
    }
    bool nw = steady_last_wheel.is_null() || ws != n(steady_last_wheel),
         nc = steady_last_current.is_null() || ts != n(steady_last_current);
    steady_last_wheel = ws;
    steady_last_current = ts;
    double settled = ack + n(tr, "settle_seconds");
    if (stale || std::min(ws, ts) <= ack ||
        std::min(ws, ts) < settled - eps({settled, ws, ts})) {
      confirmation = nullptr;
      return empty_steady(stale ? "stale_source" : "current_settling", ws, ts);
    }
    if (nw) {
      steady_wheel.push_back(J::array({ws, v}));
      trim(steady_wheel, n(c, "steady_window_seconds"));
    }
    if (nc) {
      double tolerance = std::max(n(tr, "abs_tolerance"),
                                  n(tr, "rel_tolerance") * std::abs(target));
      steady_iq.push_back(J::array({ts, std::abs(iq - target) <= tolerance}));
      trim(steady_iq, n(tr, "window_seconds"));
    }
    J ev = empty_steady("insufficient_window", ws, ts);
    if (steady_wheel.empty() || steady_iq.empty()) {
      confirmation = nullptr;
      return ev;
    }
    double first = n(steady_wheel[0][0]), cf = n(steady_iq[0][0]), fraction = 0;
    for (auto row : steady_iq)
      fraction += row[1].get<bool>();
    fraction /= steady_iq.size();
    std::vector<double> times, speeds;
    for (auto row : steady_wheel) {
      times.push_back(n(row[0]) - first);
      speeds.push_back(n(row[1]));
    }
    double mt = mean(times), mv = mean(speeds), denom = 0, numer = 0;
    for (size_t i = 0; i < times.size(); ++i) {
      denom += (times[i] - mt) * (times[i] - mt);
      numer += (times[i] - mt) * (speeds[i] - mv);
    }
    double slope = denom ? numer / denom : NaN,
           span = *std::max_element(speeds.begin(), speeds.end()) -
                  *std::min_element(speeds.begin(), speeds.end());
    ev.update({{"slope", number(slope)},
               {"speed_span", span},
               {"current_good_fraction", fraction},
               {"window_start_stamp", first},
               {"current_window_start_stamp", cf}});
    bool covered =
        steady_wheel.size() >= n(c, "steady_min_samples") &&
        ws - first + eps({ws, first}) >= n(c, "steady_window_seconds") &&
        steady_iq.size() >= n(tr, "min_samples") &&
        ts - cf + eps({ts, cf}) >= n(tr, "window_seconds");
    bool eligible = covered && std::isfinite(slope) &&
                    std::abs(slope) <= n(c, "steady_max_abs_acceleration") &&
                    span <= n(c, "steady_max_speed_span") && fraction >= .9;
    if (!eligible) {
      confirmation = nullptr;
      ev["reason"] = covered ? "outside_steady_limits" : "insufficient_window";
      return ev;
    }
    if (confirmation.is_null())
      confirmation = ws;
    double confirmed = ws - n(confirmation);
    bool ready = confirmed + eps({ws, n(confirmation)}) >=
                 n(c, "steady_confirm_seconds");
    ev.update({{"confirmation_start_stamp", confirmation},
               {"confirmed_seconds", confirmed},
               {"steady", ready},
               {"reason", ready ? "confirmed" : "confirming"}});
    return ev;
  }
  bool track_current(const J &o) {
    if (str(item(), "kind") == "coast")
      return false;
    double stamp = n(o, "telemetry_stamp"),
           time_tolerance = std::max(1e-9, 2 * ulp(stamp));
    if (!present(s, "phase_ack_stamp") ||
        stamp + time_tolerance <
            n(s, "phase_ack_stamp") + n(tr, "settle_seconds") ||
        (present(s, "last_tracking_stamp") &&
         stamp <= n(s, "last_tracking_stamp")))
      return false;
    s["last_tracking_stamp"] = stamp;
    if (!present(s, "tracking_started_stamp"))
      s["tracking_started_stamp"] = stamp;
    J row = {{"telemetry_stamp", stamp},
             {"avg_iq", o.at("avg_iq")},
             {"current_motor", o.at("current_motor")},
             {"wheel_stamp", o.at("wheel_stamp")},
             {"velocity", o.at("velocity")},
             {"region", o.at("region")}};
    s["tracking_window"].push_back(row);
    s["tracking_observations"].push_back(row);
    auto &ring = s["tracking_window"];
    while (!ring.empty() &&
           n(ring[0], "telemetry_stamp") <
               stamp - n(tr, "window_seconds") - time_tolerance)
      ring.erase(ring.begin());
    size_t count = ring.size();
    double start = std::max(n(s, "tracking_started_stamp"),
                            stamp - n(tr, "window_seconds")),
           span = stamp - start,
           tolerance = std::max(n(tr, "abs_tolerance"),
                                n(tr, "rel_tolerance") *
                                    std::abs(n(item(), "current")));
    J ev = {{"current_signal", "avg_iq"},
            {"target_current", item().at("current")},
            {"tolerance", tolerance},
            {"sample_count", count},
            {"window_start_stamp", start},
            {"window_end_stamp", stamp},
            {"window_duration_s", span},
            {"settle_seconds", tr.at("settle_seconds")},
            {"window_seconds", tr.at("window_seconds")},
            {"min_samples", tr.at("min_samples")},
            {"failure_fraction", tr.at("failure_fraction")}};
    if (count < n(tr, "min_samples") ||
        span + time_tolerance < n(tr, "window_seconds")) {
      if (str(s, "tracking_status") == "unverified")
        s["tracking_evidence"] = ev;
      return false;
    }
    int bad = 0, above = 0, below = 0;
    std::vector<double> iq, motor;
    for (auto r : ring) {
      double value = n(r, "avg_iq");
      iq.push_back(value);
      motor.push_back(n(r, "current_motor"));
      if (str(item(), "kind") == "positive_deceleration") {
        above += value - n(item(), "current") > tolerance;
        below += n(item(), "current") - value > tolerance;
      } else
        bad += (n(item(), "current") > 0 ? 1 : -1) *
                   (n(item(), "current") - value) >
               tolerance;
    }
    if (str(item(), "kind") == "positive_deceleration") {
      bad = std::max(above, below);
      ev.update({{"error_side", above >= below ? "above" : "below"},
                 {"above_count", above},
                 {"below_count", below}});
    } else
      ev["under_magnitude_count"] = bad;
    ev.update({{"failure_count", bad},
               {"failure_ratio", double(bad) / count},
               {"mean_iq", mean(iq)},
               {"mean_current_motor", mean(motor)}});
    bool unreachable = double(bad) / count + 1e-12 >= n(tr, "failure_fraction");
    s["tracking_status"] = unreachable ? "outside_tolerance" : "verified";
    s["tracking_evidence"] = ev;
    return unreachable;
  }
  void straight_exit(const J &o) {
    if (in(state(), {"MEASURE", "APPLY"})) {
      finish(false, "straight_region_exit",
             {{"wheel_stamp", o.at("wheel_stamp")},
              {"velocity", o.at("velocity")}});
      if (n(s, "attempt_number") >= n(tr, "max_attempts")) {
        s["trials"].back()["uncovered"] = true;
        advance("straight_region_exit");
      } else {
        reset();
        state("RECOVER");
        s["reason"] = "straight_region_exit";
      }
    } else {
      reset();
      state("RECOVER");
      s["reason"] = "straight_region_exit";
    }
  }
  J next(double now, const J &o) const {
    if (state() == "COMPLETE")
      return request(now, o);
    double speed = !yes(o, "straight") && str(c, "steering_source") == "pp"
                       ? n(c, "recovery_speed")
                       : n(item(), "start_speed");
    return request(now, o, 0, speed);
  }
  J skip_group(double now, const J &o, const std::string &reason) {
    if (in(state(), {"APPLY", "MEASURE"}))
      finish(false, reason,
             {{"wheel_stamp", o.at("wheel_stamp")},
              {"velocity", o.at("velocity")}});
    if (!present(s, "decel_owner_index")) {
      fault("decel_group_trigger_unavailable");
      return request(now, o, 1);
    }
    size_t origin = n(s, "decel_owner_index");
    J *trigger = nullptr;
    for (auto it = s["trials"].rbegin(); it != s["trials"].rend(); ++it)
      if (n(*it, "plan_index") == origin) {
        trigger = &*it;
        break;
      }
    if (!trigger) {
      fault("decel_group_trigger_unavailable");
      return request(now, o, 1);
    }
    J evidence;
    if (reason == "reverse_motion") {
      (*trigger).update(
          {{"response_completed", (*trigger).at("success")},
           {"success", false},
           {"reason", reason},
           {"mean_acceleration", nullptr},
           {"decision_evidence", {{"observed_at", now}, {"observation", o}}}});
      if (protected_flow)
        (*trigger).update(
            {{"pulse_valid", false}, {"interval_completed", false}});
      evidence = (*trigger)["decision_evidence"];
    } else
      evidence = (*trigger)["tracking_evidence"];
    double speed = n(tp.at(origin), "start_speed");
    size_t end = origin + 1;
    while (end < s["plan"].size() && n(tp.at(end), "start_speed") == speed)
      ++end;
    for (size_t i = std::max(index(), origin + 1); i < end; ++i) {
      J skipped = tp.at(i);
      skipped.erase("plan_index");
      skipped.update({{"plan_index", i},
                      {"success", false},
                      {"reason", "skipped_speed_group"},
                      {"trigger_reason", reason},
                      {"triggering_attempt_id", (*trigger).at("attempt_id")},
                      {"evidence", evidence}});
      s["skipped"].push_back(skipped);
    }
    s["index"] = std::max(index(), end);
    reset();
    revision();
    state("GROUP_RECOVER");
    s["reason"] = reason;
    s["group_recovery_started"] = now;
    s["group_recovery_request_stamp"] = nullptr;
    s["group_recovery_ack_stamp"] = nullptr;
    s["group_forward_since"] = nullptr;
    s["group_iq_samples"] = 0;
    s["group_iq_stamp"] = nullptr;
    s["handoff"] = nullptr;
    return request(now, o, 1);
  }
  J begin_handoff(double now, const J &o, const std::string &reason,
                  bool advance_ = true, bool turn = false) {
    double target = turn ? n(c, "recovery_speed") : n(item(), "end_speed"),
           current = turn ? 0 : holding(target);
    if (!std::isfinite(current) || std::abs(current) > n(c, "max_current")) {
      fault("invalid_holding_current");
      return request(now, o, 1);
    }
    make_handoff(now, target, current, advance_, turn, reason);
    return request(now, o, 1, 0, current);
  }
  void make_handoff(double now, double target, double current, bool advance_,
                    bool turn, const std::string &reason) {
    s["handoff"] = {{"started", now},
                    {"target_speed", target},
                    {"target_current", current},
                    {"advance", advance_},
                    {"turn", turn},
                    {"ack_stamp", nullptr},
                    {"last_iq_stamp", nullptr},
                    {"iq_samples", 0},
                    {"quiet_since", nullptr},
                    {"last_hold_wheel", nullptr},
                    {"hold_confirmed", false},
                    {"reason", reason}};
    s["trials"].back()["handoff"] = s["handoff"];
    s["started"] = nullptr;
    s["active_samples"] = J::array();
    state("UNLOAD");
    s["reason"] = reason;
  }
  J handoff_tick(double now, const J &o, const J &echo) {
    auto &h = s["handoff"];
    if (!yes(o, "straight") && !yes(h, "turn")) {
      h.update({{"turn", true},
                {"target_speed", c.at("recovery_speed")},
                {"target_current", 0.},
                {"started", now},
                {"ack_stamp", nullptr},
                {"last_iq_stamp", nullptr},
                {"iq_samples", 0},
                {"quiet_since", nullptr}});
      state("UNLOAD");
      revision();
    }
    if (state() == "UNLOAD") {
      if (full &&
          now - n(h, "started") > n(p, "unload_timeout") + 2 * ulp(now)) {
        fault("iq_unload_timeout");
        return request(now, o, 1);
      }
      bool matching = n(echo, "mode") == 1 &&
                      close(n(echo, "current"), n(h, "target_current")) &&
                      n(echo, "speed") == 0 &&
                      n(echo, "stamp") >= n(h, "started");
      if (matching && !present(h, "ack_stamp"))
        h["ack_stamp"] = echo.at("stamp");
      if (!matching)
        h["iq_samples"] = 0;
      bool after = present(h, "ack_stamp") &&
                   (full ? n(o, "telemetry_stamp") > n(h, "ack_stamp")
                         : n(o, "telemetry_stamp") >= n(h, "ack_stamp"));
      if (after && (!present(h, "last_iq_stamp") ||
                    n(o, "telemetry_stamp") > n(h, "last_iq_stamp"))) {
        if (present(h, "last_iq_stamp") &&
            n(o, "telemetry_stamp") - n(h, "last_iq_stamp") >
                n(p, "source_gap"))
          h["iq_samples"] = 0;
        h["last_iq_stamp"] = o.at("telemetry_stamp");
        h["iq_samples"] =
            matching && std::abs(n(o, "avg_iq") - n(h, "target_current")) <=
                            n(p, "iq_tolerance")
                ? h["iq_samples"].get<std::size_t>() + 1
                : 0;
        if (n(h, "iq_samples") >= n(p, "iq_confirm_samples")) {
          h["actual_iq_confirm_stamp"] = o.at("telemetry_stamp");
          state("HOLD");
          h["hold_started"] = now;
          h["quiet_since"] = nullptr;
          revision();
        }
      }
      if (state() == "UNLOAD" &&
          now - n(h, "started") >= n(p, "unload_timeout")) {
        fault("iq_unload_timeout");
        return request(now, o, 1);
      }
      s["trials"].back()["handoff"] = h;
      if (state() == "UNLOAD")
        return request(now, o, 1, 0, n(h, "target_current"));
    }
    if (full &&
        now - n(h, "hold_started") > n(p, "hold_timeout") + 2 * ulp(now)) {
      fault("post_pulse_hold_timeout");
      return request(now, o, 1);
    }
    bool matching = n(echo, "mode") == 0 &&
                    close(n(echo, "speed"), n(h, "target_speed")) &&
                    n(echo, "stamp") >= n(h, "hold_started");
    if (matching && !full && acceleration.is_number() && instant.is_number() &&
        n(acceleration) * n(instant) > 0 &&
        std::abs(n(instant)) > n(p, "quiet_acceleration")) {
      double predicted =
          n(o, "velocity") +
          n(acceleration) *
              (n(p, "exit_horizon") + std::max(0., now - n(o, "wheel_stamp")));
      if ((n(acceleration) > 0 &&
           predicted > n(h, "target_speed") + n(c, "stable_tolerance")) ||
          (n(acceleration) < 0 &&
           predicted < n(h, "target_speed") - n(c, "stable_tolerance"))) {
        h["hold_prediction"] = {{"predicted_velocity", predicted},
                                {"wheel_acceleration", acceleration},
                                {"wheel_stamp", o.at("wheel_stamp")}};
        fault("post_pulse_predicted_excursion");
        return request(now, o, 1);
      }
    }
    if (!matching)
      h["quiet_since"] = nullptr;
    if (present(h, "last_hold_wheel") &&
        n(o, "wheel_stamp") - n(h, "last_hold_wheel") > n(p, "source_gap"))
      h["quiet_since"] = nullptr;
    bool fresh_hold = (!present(h, "last_hold_wheel") ||
                       n(o, "wheel_stamp") > n(h, "last_hold_wheel")) &&
                      n(o, "wheel_stamp") >= n(h, "hold_started");
    if (fresh_hold) {
      h["last_hold_wheel"] = o.at("wheel_stamp");
      bool quiet = matching &&
                   std::abs(n(o, "velocity") - n(h, "target_speed")) <=
                       n(c, "stable_tolerance") &&
                   acceleration.is_number() &&
                   std::abs(n(acceleration)) <= n(p, "quiet_acceleration");
      if (quiet) {
        if (!present(h, "quiet_since"))
          h["quiet_since"] = o.at("wheel_stamp");
        if (n(o, "wheel_stamp") - n(h, "quiet_since") >=
            n(c, "stable_seconds")) {
          h.update({{"hold_confirmed", true},
                    {"hold_confirmed_stamp", o.at("wheel_stamp")}});
          s["trials"].back()["handoff"] = h;
          revision();
          s["decel_owner_index"] = nullptr;
          bool adv = yes(h, "advance");
          auto reason = str(h, "reason");
          if (adv)
            advance(reason);
          else {
            reset();
            state("RECOVER");
            s["reason"] = reason;
          }
          s["handoff"] = nullptr;
          return next(now, o);
        }
      } else
        h["quiet_since"] = nullptr;
    }
    if (now - n(h, "hold_started") >= n(p, "hold_timeout")) {
      fault("post_pulse_hold_timeout");
      return request(now, o, 1);
    }
    s["trials"].back()["handoff"] = h;
    return request(now, o, 0, n(h, "target_speed"));
  }
  J full_stop(double now, const J &o, const std::string &reason,
              bool verified = false) {
    bool normal = in(reason, {"speed_boundary", "steady_state"}),
         observed = verified && s["active_samples"].size() >= 2 &&
                    present(s, "phase_ack_stamp");
    double target =
        reason == "steady_state" ? n(o, "velocity") : n(item(), "end_speed");
    if (!normal)
      target = n(c, "recovery_speed");
    double current = 0;
    std::string error;
    try {
      current = normal ? holding(target) : 0.;
    } catch (const std::invalid_argument &e) {
      error = e.what();
      target = n(c, "recovery_speed");
      current = 0.;
    }
    if (!std::isfinite(current) || std::abs(current) > n(c, "max_current")) {
      fault("invalid_holding_current");
      return request(now, o, 1);
    }
    finish(
        normal && observed, reason,
        {{"wheel_stamp", o.at("wheel_stamp")}, {"velocity", o.at("velocity")}});
    bool retry = in(reason, {"trial_timeout", "straight_headroom",
                             "straight_region_exit"}),
         exhausted = !retry || n(s, "attempt_number") >= n(tr, "max_attempts");
    s["trials"].back()["attempts_exhausted"] = exhausted;
    if (retry && !exhausted) {
      s["retry_region"] = s["region"];
      s["retry_left"] = false;
    }
    make_handoff(now, target, current, exhausted, !normal || !error.empty(),
                 reason);
    if (!error.empty()) {
      s["handoff"].update({{"holding_current_unavailable", true},
                           {"holding_current_error", error}});
      s["trials"].back()["handoff"] = s["handoff"];
    }
    return request(now, o, 1, 0, current);
  }
  bool full_distance(const J &o) const {
    return std::isfinite(n(o, "straight_remaining_m")) &&
           n(o, "straight_remaining_m") > n(c, "straight_exit_margin_m");
  }
  void add_sample(const J &o, const J &req) {
    J row = o;
    row.erase("final_request");
    if (stage() == "speed_hold")
      row.erase("fault_code");
    row.update({{"trial_id", s["trials"].size() + 1},
                {"plan_index", index()},
                {"stage", stage()},
                {"requested_mode", req.at("mode")},
                {"requested_speed", req.at("speed")},
                {"requested_current", req.at("current")}});
    if (stage() == "speed_hold" && str(c, "hold_sampling") == "cumulative")
      sample_hold(o, row);
    if (stage() != "speed_hold") {
      row.update({{"attempt_id", s["attempt_id"]},
                  {"attempt_number", s["attempt_number"]},
                  {"repeat", item().at("repeat")},
                  {"kind", item().at("kind")}});
      if (!present(s, "starting_wheel_observation"))
        s["starting_wheel_observation"] = {
            {"wheel_stamp", o.at("wheel_stamp")},
            {"telemetry_stamp", o.at("telemetry_stamp")},
            {"velocity", o.at("velocity")}};
    }
    s["samples"].push_back(row);
    s["active_samples"].push_back(row);
    if (!present(s, "started")) {
      s["started"] = o.at("wheel_stamp");
      s["start_velocity"] = o.at("velocity");
    }
    state("MEASURE");
  }
  J full_tick(double now, const J &o, const J &echo, const J &pair,
              bool unique) {
    if (in(state(), {"UNLOAD", "HOLD"})) {
      auto &row = s["trials"].back();
      J previous = J::array({row["wheel_speed_min"], row["wheel_speed_max"]});
      if (!present(row, "wheel_speed_min")) {
        row["wheel_speed_min"] = o.at("velocity");
        row["wheel_speed_max"] = o.at("velocity");
      } else {
        row["wheel_speed_min"] =
            std::min(n(row, "wheel_speed_min"), n(o, "velocity"));
        row["wheel_speed_max"] =
            std::max(n(row, "wheel_speed_max"), n(o, "velocity"));
      }
      if (previous !=
          J::array({row["wheel_speed_min"], row["wheel_speed_max"]}))
        revision();
    }
    if (stage() == "decel_current" && yes(p, "reverse_guard_enabled") &&
        in(state(), {"APPLY", "MEASURE", "UNLOAD", "HOLD"}) &&
        n(o, "velocity") <= n(p, "reverse_guard")) {
      fault("near_reverse");
      return request(now, o, 1);
    }
    if (in(state(), {"UNLOAD", "HOLD"}))
      return handoff_tick(now, o, echo);
    if (state() == "RECOVER") {
      if (!yes(o, "straight") || o.at("region") != s["retry_region"])
        s["retry_left"] = true;
      if (present(s, "retry_region") &&
          !(yes(s, "retry_left") && yes(o, "straight")))
        return request(now, o, 0, n(c, "recovery_speed"));
      s["retry_region"] = nullptr;
      s["retry_left"] = false;
      state("PREPARE");
    }
    if (state() == "PREPARE") {
      if (!yes(o, "straight")) {
        s["stable_since"] = nullptr;
        s["last_prepare_wheel"] = nullptr;
        return request(now, o, 0, n(c, "recovery_speed"));
      }
      s["region"] = o.at("region");
      bool ready = false;
      if (stage() == "decel_current") {
        bool fresh_wheel = !present(s, "last_prepare_wheel") ||
                           n(o, "wheel_stamp") > n(s, "last_prepare_wheel");
        if (fresh_wheel)
          s["last_prepare_wheel"] = o.at("wheel_stamp");
        ready = fresh_wheel && n(o, "velocity") >= n(item(), "start_speed") &&
                n(echo, "mode") == 0 &&
                close(n(echo, "speed"), n(item(), "start_speed"));
      } else if (unique) {
        s["last_pair"] = pair;
        if (present(s, "last_prepare_wheel") &&
            n(o, "wheel_stamp") - n(s, "last_prepare_wheel") >
                n(p, "source_gap"))
          s["stable_since"] = nullptr;
        s["last_prepare_wheel"] = o.at("wheel_stamp");
        bool stable = std::abs(n(o, "velocity") - n(item(), "start_speed")) <=
                          n(c, "stable_tolerance") &&
                      acceleration.is_number() &&
                      std::abs(n(acceleration)) <= n(p, "quiet_acceleration") &&
                      n(echo, "mode") == 0 &&
                      close(n(echo, "speed"), n(item(), "start_speed"));
        if (stable) {
          if (!present(s, "stable_since"))
            s["stable_since"] = o.at("wheel_stamp");
        } else
          s["stable_since"] = nullptr;
        ready = present(s, "stable_since") &&
                n(o, "wheel_stamp") - n(s, "stable_since") >=
                    n(c, "stable_seconds") &&
                std::min(n(item(), "start_speed"), n(item(), "end_speed")) <=
                    n(o, "velocity") &&
                n(o, "velocity") <=
                    std::max(n(item(), "start_speed"), n(item(), "end_speed"));
      }
      if (ready) {
        state("APPLY");
        s["phase_started"] = o.at("wheel_stamp");
        begin_attempt();
        if (!std::isfinite(n(o, "straight_remaining_m"))) {
          finish(false, "straight_distance_unavailable",
                 {{"wheel_stamp", o.at("wheel_stamp")},
                  {"velocity", o.at("velocity")}});
          s["trials"].back().update(
              {{"attempts_exhausted", true}, {"preflight_rejected", true}});
          advance("straight_distance_unavailable");
          return next(now, o);
        }
        if (!full_distance(o)) {
          auto key = std::to_string(index());
          s["attempt_counts"][key] =
              s["attempt_counts"][key].get<std::size_t>() - 1;
          reset();
          s["retry_region"] = o.at("region");
          s["retry_left"] = false;
          state("RECOVER");
          return request(now, o, 0, n(c, "recovery_speed"));
        }
        if (stage() == "decel_current")
          s["start_trigger_observation"] = {
              {"wheel_stamp", o.at("wheel_stamp")},
              {"telemetry_stamp", o.at("telemetry_stamp")},
              {"velocity", o.at("velocity")},
              {"request_stamp", now}};
        return request(now, o, 1, 0, n(item(), "current"));
      }
      return request(now, o, 0, n(item(), "start_speed"));
    }
    if (!in(state(), {"APPLY", "MEASURE"})) {
      fault("invalid_full_step_state");
      return request(now, o, 1);
    }
    bool matching = n(echo, "mode") == 1 &&
                    close(n(echo, "current"), n(item(), "current")) &&
                    n(echo, "speed") == 0 &&
                    present(s, "first_current_request_stamp") &&
                    n(echo, "stamp") >= n(s, "first_current_request_stamp");
    if (matching && !present(s, "phase_ack_stamp"))
      s["phase_ack_stamp"] = echo.at("stamp");
    bool ack = present(s, "phase_ack_stamp") &&
               std::min(n(pair[0]), n(pair[1])) >= n(s, "phase_ack_stamp"),
         verified = unique && matching && ack;
    if (verified) {
      s["last_pair"] = pair;
      add_sample(o, request(now, o, 1, 0, n(item(), "current")));
    }
    s["steady_evidence"] =
        steady_observe(o, n(item(), "current"), matching && ack);
    bool boundary = stage() == "accel_interval"
                        ? n(o, "velocity") >= n(item(), "end_speed")
                        : n(o, "velocity") <= n(item(), "end_speed");
    if (!yes(o, "straight") || o.at("region") != s["region"])
      return full_stop(now, o, "straight_region_exit");
    if (boundary)
      return full_stop(now, o, "speed_boundary", verified);
    if (!full_distance(o))
      return full_stop(now, o, "straight_headroom");
    if (present(s, "first_current_request_stamp") &&
        now - n(s, "first_current_request_stamp") >= n(c, "trial_timeout"))
      return full_stop(now, o, "trial_timeout");
    const auto &q = s["steady_evidence"];
    if (present(q, "current_window_start_stamp") &&
        n(q, "current_sample_count") >= n(tr, "min_samples") &&
        n(o, "telemetry_stamp") - n(q, "current_window_start_stamp") +
                2 * ulp(n(o, "telemetry_stamp")) >=
            n(tr, "window_seconds")) {
      double fraction = n(q, "current_good_fraction");
      s["tracking_evidence"] = q;
      s["tracking_status"] = fraction >= .9 ? "verified" : "unverified";
      if (1 - fraction + 1e-12 >= n(tr, "failure_fraction")) {
        s["tracking_status"] = "outside_tolerance";
        return full_stop(now, o, "current_tracking_not_achieved");
      }
    }
    if (yes(q, "steady"))
      return full_stop(now, o, "steady_state", verified);
    return request(now, o, 1, 0, n(item(), "current"));
  }
  J tick(double now, J o) {
    static const J defaults = {{"wheel_stamp", 0.},
                               {"telemetry_stamp", 0.},
                               {"velocity", nullptr},
                               {"current_motor", nullptr},
                               {"avg_iq", nullptr},
                               {"voltage_input", nullptr},
                               {"erpm", nullptr},
                               {"authority", false},
                               {"authority_stamp", 0.},
                               {"final_request", nullptr},
                               {"final_received", 0.},
                               {"rc_stamp", 0.},
                               {"rc_valid", false},
                               {"straight", false},
                               {"region", ""},
                               {"steering", 0.},
                               {"tracking_reason", ""},
                               {"acceleration", nullptr},
                               {"acceleration_stamp", 0.},
                               {"acceleration_quality", "unavailable"},
                               {"fault_code", 0},
                               {"straight_remaining_m", nullptr}};
    J merged = defaults;
    merged.update(o);
    o = std::move(merged);
    if (!std::isfinite(now) || now <= 0 ||
        (present(s, "last_now") && now < n(s, "last_now"))) {
      fault("clock_backwards");
      return request(now, o, 1);
    }
    s["last_now"] = now;
    if (in(state(), {"FAULT", "STOPPING"}))
      return request(now, o, 1);
    if (!yes(s, "enabled") || in(state(), {"IDLE", "COMPLETE"}))
      return request(now, o);
    if (stage() != "speed_hold" && n(o, "fault_code") != 0) {
      fault("vesc_fault_" +
            std::to_string(static_cast<int>(n(o, "fault_code"))));
      return request(now, o, 1);
    }
    if (!present(s, "armed_at"))
      s["armed_at"] = now;
    bool authority = yes(o, "authority") &&
                     fresh(now, n(o, "authority_stamp")) &&
                     n(o, "authority_stamp") > n(s, "armed_at");
    const J &echo = o.at("final_request");
    bool echo_ok = !echo.is_null() && fresh(now, n(o, "final_received")) &&
                   fresh(now, n(echo, "stamp"));
    if (echo_ok) {
      echo_ok = false;
      for (auto &r : s["history"])
        if (fresh(now, n(r, "stamp")) && n(r, "stamp") <= n(echo, "stamp") &&
            corresponds(r, echo)) {
          echo_ok = true;
          break;
        }
    }
    if (!yes(s, "selected")) {
      bool ready = authority && yes(o, "rc_valid");
      for (auto k : {"rc_stamp", "wheel_stamp", "telemetry_stamp"})
        ready = ready && fresh(now, n(o, k)) && n(o, k) > n(s, "armed_at");
      for (auto k : {"velocity", "avg_iq", "current_motor", "voltage_input",
                     "erpm", "steering"})
        ready = ready && std::isfinite(n(o, k));
      if (!ready || !echo_ok || n(echo, "stamp") <= n(s, "armed_at"))
        return request(now, o);
      s["selected"] = true;
    }
    if (!authority) {
      fault("authority_lost_or_stale");
      return request(now, o, 1);
    }
    std::string reason;
    if (!fresh(now, n(o, "rc_stamp")) || !yes(o, "rc_valid"))
      reason = "rc_invalid_or_stale";
    else if (!fresh(now, n(o, "wheel_stamp")) ||
             !std::isfinite(n(o, "velocity")))
      reason = "wheel_invalid_or_stale";
    else if (!fresh(now, n(o, "telemetry_stamp")) ||
             !std::isfinite(n(o, "avg_iq")) ||
             !std::isfinite(n(o, "current_motor")) ||
             !std::isfinite(n(o, "voltage_input")) ||
             !std::isfinite(n(o, "erpm")))
      reason = "telemetry_invalid_or_stale";
    else if (str(c, "steering_source") == "pp" &&
             !str(o, "tracking_reason").empty())
      reason = str(o, "tracking_reason");
    else if (!std::isfinite(n(o, "steering")))
      reason = "steering_invalid";
    else if (n(o, "velocity") < 0 &&
             (stage() != "decel_current" ||
              (!present(s, "decel_owner_index") && state() != "GROUP_RECOVER")))
      reason = "reverse_motion";
    if (!reason.empty()) {
      fault(reason);
      return request(now, o, 1);
    }
    if (!echo_ok) {
      if (s["history"].empty())
        return request(now, o);
      fault("final_command_mismatch_or_stale");
      return request(now, o, 1);
    }
    J pair = J::array({o.at("wheel_stamp"), o.at("telemetry_stamp")});
    if ((present(s, "last_wheel") &&
         n(o, "wheel_stamp") < n(s, "last_wheel")) ||
        (present(s, "last_telemetry") &&
         n(o, "telemetry_stamp") < n(s, "last_telemetry"))) {
      fault("source_clock_backwards");
      return request(now, o, 1);
    }
    s["last_wheel"] = o.at("wheel_stamp");
    s["last_telemetry"] = o.at("telemetry_stamp");
    bool unique =
        !present(s, "last_pair") || (n(pair[0]) > n(s["last_pair"][0]) &&
                                     n(pair[1]) > n(s["last_pair"][1]));
    if (protected_flow) {
      observe(n(o, "wheel_stamp"), n(o, "velocity"));
      if (!full &&
          n(o, "velocity") > n(c, "max_speed") + n(p, "overspeed_margin")) {
        fault("wheel_overspeed");
        return request(now, o, 1);
      }
    }
    if (full)
      return full_tick(now, o, echo, pair, unique);
    if (state() == "GROUP_RECOVER") {
      bool neutral = n(echo, "mode") == 1 && n(echo, "current") == 0 &&
                     n(echo, "speed") == 0;
      if (neutral && present(s, "group_recovery_request_stamp") &&
          n(echo, "stamp") >= n(s, "group_recovery_request_stamp") &&
          !present(s, "group_recovery_ack_stamp"))
        s["group_recovery_ack_stamp"] = echo.at("stamp");
      bool iq_ready = true;
      if (protected_flow) {
        if (!neutral)
          s["group_iq_samples"] = 0;
        if (neutral && present(s, "group_recovery_ack_stamp") &&
            n(o, "telemetry_stamp") >= n(s, "group_recovery_ack_stamp") &&
            (!present(s, "group_iq_stamp") ||
             n(o, "telemetry_stamp") > n(s, "group_iq_stamp"))) {
          s["group_iq_stamp"] = o.at("telemetry_stamp");
          s["group_iq_samples"] =
              std::abs(n(o, "avg_iq")) <= n(p, "iq_tolerance")
                  ? s["group_iq_samples"].get<std::size_t>() + 1
                  : 0;
        }
        iq_ready =
            neutral && n(s, "group_iq_samples") >= n(p, "iq_confirm_samples");
        if (!iq_ready &&
            now - n(s, "group_recovery_started") >= n(p, "unload_timeout")) {
          fault("iq_unload_timeout");
          return request(now, o, 1);
        }
      }
      if (iq_ready && n(o, "velocity") >= 0 &&
          present(s, "group_recovery_ack_stamp") &&
          std::min(n(pair[0]), n(pair[1])) >=
              n(s, "group_recovery_ack_stamp")) {
        if (!present(s, "group_forward_since"))
          s["group_forward_since"] = o.at("wheel_stamp");
        if (n(o, "wheel_stamp") - n(s, "group_forward_since") >=
            n(c, "stable_seconds")) {
          s["decel_owner_index"] = nullptr;
          state(index() == s["plan"].size() ? "COMPLETE" : "PREPARE");
          return next(now, o);
        }
      } else
        s["group_forward_since"] = nullptr;
      if (now - n(s, "group_recovery_started") >= n(c, "trial_timeout"))
        fault("decel_group_recovery_timeout");
      return request(now, o, 1);
    }
    if (stage() == "decel_current" && n(o, "velocity") < 0)
      return skip_group(now, o, "reverse_motion");
    if (protected_flow) {
      if (stage() == "decel_current" && yes(p, "reverse_guard_enabled") &&
          n(o, "velocity") <= n(p, "reverse_guard") &&
          in(state(), {"APPLY", "MEASURE", "UNLOAD", "HOLD"})) {
        fault("near_reverse");
        return request(now, o, 1);
      }
      if (in(state(), {"UNLOAD", "HOLD"}))
        return handoff_tick(now, o, echo);
    }
    double target = stage() == "speed_hold" ? n(s["plan"].at(index()))
                                            : n(item(), "start_speed"),
           end_speed = stage() == "speed_hold" ? n(c, "end_speed")
                                               : n(item(), "end_speed");
    J requested = stage() == "speed_hold"
                      ? request(now, o, 0, target)
                      : request(now, o, 1, 0, n(s["plan"].at(index())));
    bool phase_echo = n(echo, "mode") == n(requested, "mode") &&
                      close(n(echo, "speed"), n(requested, "speed")) &&
                      close(n(echo, "current"), n(requested, "current")),
         current_phase =
             stage() != "speed_hold" && in(state(), {"APPLY", "MEASURE"});
    if (current_phase && phase_echo && !present(s, "phase_ack_stamp"))
      s["phase_ack_stamp"] = echo.at("stamp");
    bool ack = present(s, "phase_ack_stamp") &&
               std::min(n(pair[0]), n(pair[1])) >= n(s, "phase_ack_stamp"),
         boundary = current_phase && (stage() == "accel_interval"
                                          ? n(o, "velocity") >= end_speed
                                          : n(o, "velocity") <= end_speed),
         same_region = yes(o, "straight") &&
                       (!present(s, "region") || s["region"] == o.at("region"));
    if (current_phase && !same_region) {
      if (protected_flow) {
        finish(false, "straight_region_exit",
               {{"wheel_stamp", o.at("wheel_stamp")},
                {"velocity", o.at("velocity")}});
        return begin_handoff(now, o, "straight_region_exit",
                             n(s, "attempt_number") >= n(tr, "max_attempts"),
                             true);
      }
      straight_exit(o);
      if (state() == "COMPLETE")
        return request(now, o);
      return request(now, o, 0,
                     str(c, "steering_source") == "pp"
                         ? n(c, "recovery_speed")
                         : n(item(), "start_speed"));
    }
    if (boundary && !(unique && phase_echo && ack && same_region)) {
      if (!phase_echo)
        reason = "boundary_before_current_echo";
      else if (!unique)
        reason = n(pair[1]) <= n(s["last_pair"][1])
                     ? "boundary_without_new_telemetry"
                     : "boundary_without_new_wheel";
      else if (!ack)
        reason = "boundary_before_selected_current_observation";
      else
        reason = "straight_region_exit";
      finish(false, reason,
             {{"wheel_stamp", o.at("wheel_stamp")},
              {"velocity", o.at("velocity")}});
      if (protected_flow)
        return begin_handoff(now, o, reason);
      advance(reason);
      return next(now, o);
    }
    if (!yes(o, "straight") ||
        (present(s, "region") && s["region"] != o.at("region"))) {
      if (stage() == "speed_hold" && str(c, "hold_sampling") == "cumulative") {
        pause_hold("straight_region_exit");
        state("RECOVER");
        return request(now, o, 0,
                       str(c, "steering_source") == "pp"
                           ? n(c, "recovery_speed")
                           : target);
      }
      if (in(state(), {"MEASURE", "APPLY"})) {
        finish(false, "straight_region_exit");
        s["reason"] = "straight_region_exit";
      }
      reset();
      state("RECOVER");
      return request(now, o, 0,
                     str(c, "steering_source") == "pp" ? n(c, "recovery_speed")
                                                       : target);
    }
    if (state() == "RECOVER")
      state("PREPARE");
    s["region"] = o.at("region");
    if (state() == "PREPARE") {
      if (unique) {
        s["last_pair"] = pair;
        bool stable =
            std::abs(n(o, "velocity") - target) <= n(c, "stable_tolerance") &&
            (!protected_flow ||
             (acceleration.is_number() &&
              std::abs(n(acceleration)) <= n(p, "quiet_acceleration"))) &&
            n(echo, "mode") == 0 && close(n(echo, "speed"), target);
        if (stable) {
          if (!present(s, "stable_since"))
            s["stable_since"] = o.at("wheel_stamp");
          if (n(o, "wheel_stamp") - n(s, "stable_since") >=
              n(c, "stable_seconds")) {
            state(stage() == "speed_hold" ? "MEASURE" : "APPLY");
            s["phase_started"] = o.at("wheel_stamp");
            if (stage() != "speed_hold")
              begin_attempt();
            if (protected_flow) {
              try {
                s["exit_prediction"] = prediction(now, o);
                reason = yes(s["exit_prediction"], "exits")
                             ? "insufficient_exit_headroom"
                             : "";
              } catch (const std::invalid_argument &e) {
                reason = e.what();
              }
              if (!reason.empty()) {
                finish(false, reason,
                       {{"wheel_stamp", o.at("wheel_stamp")},
                        {"velocity", o.at("velocity")}});
                advance(reason);
                return next(now, o);
              }
            }
            if (stage() != "speed_hold" &&
                (stage() == "accel_interval" ? n(o, "velocity") >= end_speed
                                             : n(o, "velocity") <= end_speed)) {
              reason = "boundary_before_current_request";
              finish(false, reason,
                     {{"wheel_stamp", o.at("wheel_stamp")},
                      {"velocity", o.at("velocity")}});
              advance(reason);
              return next(now, o);
            }
            if (stage() == "speed_hold" &&
                str(c, "hold_sampling") == "continuous") {
              s["started"] = o.at("wheel_stamp");
              s["start_velocity"] = o.at("velocity");
            }
          }
        } else
          s["stable_since"] = nullptr;
      }
      return stage() == "speed_hold" || state() == "PREPARE"
                 ? request(now, o, 0, target)
                 : request(now, o, 1, 0, n(s["plan"].at(index())));
    }
    if (stage() == "speed_hold" &&
        std::abs(n(o, "velocity") - target) > n(c, "stable_tolerance")) {
      if (str(c, "hold_sampling") == "cumulative") {
        pause_hold("speed_unstable");
        state("PREPARE");
        return request(now, o, 0, target);
      }
      finish(false, "speed_unstable");
      reset();
      state("PREPARE");
      s["reason"] = "speed_unstable";
      return request(now, o, 0, target);
    }
    if (stage() == "speed_hold" && str(c, "hold_sampling") == "cumulative") {
      if (!phase_echo) {
        pause_hold("waiting_for_target_speed_echo");
        state("PREPARE");
        return requested;
      }
      if (unique && present(s, "hold_previous") &&
          (n(pair[0]) - n(s["hold_previous"][0]) > n(c, "freshness") ||
           n(pair[1]) - n(s["hold_previous"][1]) > n(c, "freshness"))) {
        pause_hold("sample_source_gap");
        state("PREPARE");
        return requested;
      }
    }
    bool predictive = false;
    if (protected_flow && current_phase) {
      try {
        s["exit_prediction"] = prediction(now, o);
      } catch (const std::invalid_argument &e) {
        fault(e.what());
        return request(now, o, 1);
      }
      predictive = yes(s["exit_prediction"], "exits");
    }
    if (current_phase && !boundary && phase_echo && ack) {
      if (track_current(o) && stage() == "decel_current")
        return skip_group(now, o, "current_tracking_not_achieved");
    }
    if (unique && phase_echo && (stage() == "speed_hold" || ack)) {
      s["last_pair"] = pair;
      add_sample(o, requested);
      double elapsed = n(o, "wheel_stamp") - n(s, "started"),
             hold_duration = str(c, "hold_sampling") == "cumulative"
                                 ? n(s, "hold_elapsed")
                                 : elapsed;
      bool reached = stage() == "speed_hold"
                         ? hold_duration + 1e-9 >= n(c, "measurement_seconds")
                         : (stage() == "accel_interval"
                                ? n(o, "velocity") >= end_speed
                                : n(o, "velocity") <= end_speed),
           timeout = stage() != "speed_hold" &&
                     n(o, "wheel_stamp") - n(s, "phase_started") >=
                         n(c, "trial_timeout");
      if (reached || timeout || predictive) {
        bool valid = reached && (stage() == "speed_hold" || elapsed > 0);
        reason = valid ? "completed"
                       : (reached ? "interval_without_positive_duration"
                                  : (predictive ? "predictive_exit"
                                                : "trial_timeout"));
        finish(valid, reason, s["active_samples"].back());
        if (protected_flow)
          return begin_handoff(now, o, reason);
        advance(reason);
        if (stage() == "speed_hold")
          return state() == "PREPARE"
                     ? request(now, o, 0, n(s["plan"].at(index())))
                     : request(now, o);
        return next(now, o);
      }
    }
    if (protected_flow && current_phase && predictive) {
      finish(false, "predictive_exit_unverified",
             {{"wheel_stamp", o.at("wheel_stamp")},
              {"velocity", o.at("velocity")}});
      return begin_handoff(now, o, "predictive_exit_unverified");
    }
    if (current_phase && !boundary &&
        present(s, "first_current_request_stamp") &&
        now - n(s, "first_current_request_stamp") >= n(c, "trial_timeout")) {
      finish(false, "trial_timeout",
             {{"wheel_stamp", o.at("wheel_stamp")},
              {"velocity", o.at("velocity")}});
      if (protected_flow)
        return begin_handoff(now, o, "trial_timeout");
      advance("trial_timeout");
      return next(now, o);
    }
    return requested;
  }
};
NativeEngine::NativeEngine(const json &prepared)
    : impl_(std::make_unique<Impl>(prepared)) {}
NativeEngine::~NativeEngine() = default;
NativeEngine::NativeEngine(NativeEngine &&) noexcept = default;
NativeEngine &NativeEngine::operator=(NativeEngine &&) noexcept = default;
NativeEngine::json NativeEngine::tick(double now, const json &observation) {
  return impl_->tick(now, observation);
}
void NativeEngine::sent(const json &request) { impl_->sent(request); }
void NativeEngine::enable(bool value, double now) { impl_->enable(value, now); }
void NativeEngine::fault(const std::string &reason) { impl_->fault(reason); }
void NativeEngine::invalidate_evidence(std::size_t first,
                                       const std::string &reason) {
  auto &trials = impl_->s["trials"];
  for (size_t i = first; i < trials.size(); ++i) {
    auto &r = trials[i];
    J completed = r.value("response_completed", r.at("success"));
    r.update({{"success", false},
              {"reason", reason},
              {"response_completed", completed},
              {"evidence_valid", false},
              {"evidence_failure", reason},
              {"mean_acceleration", nullptr}});
  }
}
NativeEngine::json NativeEngine::snapshot(bool include_raw_samples) const {
  J out = J::object();
  for (auto it = impl_->s.begin(); it != impl_->s.end(); ++it) {
    if (!include_raw_samples &&
        in(it.key(), {"samples", "active_samples", "tracking_observations",
                      "tracking_window", "history"}))
      continue;
    out[it.key()] = it.value();
  }
  out["wheel_acceleration"] = impl_->acceleration;
  out["instantaneous_acceleration"] = impl_->instant;
  return out;
}
NativeEngine::json NativeEngine::status() const {
  J out = J::object();
  for (const auto key :
       {"config", "tracking_config", "state", "selected", "enabled", "index",
        "results_revision", "reason", "hold_elapsed", "hold_segments",
        "steady_evidence", "exit_prediction", "handoff", "tracking_status",
        "tracking_evidence"})
    out[key] = impl_->s.at(key);
  out["trials_count"] = impl_->s.at("trials").size();
  out["skipped_count"] = impl_->s.at("skipped").size();
  return out;
}
} // namespace aims_calibration
