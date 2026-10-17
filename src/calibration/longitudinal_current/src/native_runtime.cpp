#include "aims_calibration/native_runtime.hpp"
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
namespace aims_calibration {
namespace {
bool bounded(const json &v, double a, double b) {
  return v.is_number() && std::isfinite(v.get<double>()) &&
         v.get<double>() >= a && v.get<double>() <= b;
}
std::string cell(const json &v) {
  if (v.is_null())
    return "";
  std::string s = v.is_string() ? v.get<std::string>() : v.dump();
  if (s.find_first_of(",\"\n\r") != std::string::npos) {
    std::string out = "\"";
    for (char c : s) {
      if (c == '\"')
        out += '\"';
      out += c;
    }
    return out + '\"';
  }
  return s;
}
} // namespace
std::vector<std::string> check_start_native(const json &d) {
  std::vector<std::string> r;
  auto require = [&](bool ok, const std::string &s) {
    if (!ok)
      r.push_back(s);
  };
  require(d.value("state", "") == "IDLE" && d.value("enabled", true) == false &&
              d.value("enable_count", -1) == 0,
          "acquisition_already_started");
  require(d.value("logging_error", "missing").empty() &&
              d.value("logging_dropped", -1) == 0,
          "logging_unavailable");
  auto f = d.value("freshness", 0.);
  if (!std::isfinite(f) || f < .001 || f > .2)
    f = 0.;
  auto ages = d.value("ages", json::object());
  for (auto suffix : {"source", "receipt"})
    for (auto name : {"wheel", "telemetry", "final", "rc", "authority"}) {
      auto k = std::string(name) + "_" + suffix;
      require(bounded(ages.value(k, json()), 0., f), k + "_stale_or_future");
    }
  require(bounded(d.value("wheel_speed", json()), -.1, .1),
          "vehicle_must_be_stationary");
  require(d.value("telemetry_valid", false), "invalid_motor_telemetry");
  require(d.value("current_signal", "") == "avg_iq", "q_axis_current_required");
  require(d.value("authority", false), "calibration_authority_required");
  require(d.value("rc_valid", false), "rc_locked_or_wrong_mode");
  require(d.value("final_zero", false), "final_zero_speed_required");
  require(d.value("publishers", json()) ==
              json({{"calibration", 1}, {"final", 1}, {"drive", 0}}),
          "exclusive_command_producers_required");
  require(d.value("driver_startup_disabled", false),
          "driver_startup_must_be_disabled");
  require(bounded(d.value("driver_parameter_age", json()), 0., 2.),
          "driver_parameter_unverified");
  require(d.value("artifact_error", "missing").empty(), "artifact_changed");
  if (d.value("pp", false)) {
    require(d.value("localization_reason", "missing").empty(),
            "localization_unavailable");
    require(bounded(d.value("cross_track", json()), -.25, .25),
            "reference_distance_exceeded");
    require(bounded(d.value("heading_error", json()), -.35, .35),
            "reference_heading_exceeded");
  }
  if (d.value("require_acceleration", false))
    require(d.value("acceleration_valid", false), "acceleration_unavailable");
  return r;
}
std::string SteadyGuard::check(double ros, double steady, bool selected,
                               double freshness, bool protected_flow,
                               size_t producers,
                               const std::map<std::string, double> &receipts) {
  std::string reason;
  if (selected) {
    if (last_timer >= 0. && steady - last_timer > (protected_flow ? .06 : .2))
      reason = "publication_timer_stale";
    if (producers != 1)
      reason = "multiple_calibration_publishers";
    for (auto name : {"wheel", "telemetry", "rc", "authority", "final"}) {
      auto it = receipts.find(name);
      if (it == receipts.end() || steady - it->second < 0 ||
          steady - it->second > freshness) {
        if (reason.empty())
          reason = std::string(name) + "_receipt_stale";
        break;
      }
    }
  }
  last_timer = steady;
  if (ros != last_ros)
    last_changed = steady;
  else if (selected && steady - last_changed > .2 && reason.empty())
    reason = "ros_clock_stalled";
  last_ros = ros;
  return reason;
}
AsyncEvidenceWriter::AsyncEvidenceWriter(std::string directory, json config,
                                         json metadata, json manifest,
                                         size_t capacity)
    : directory_(std::move(directory)), config_(std::move(config)),
      metadata_(std::move(metadata)), manifest_(std::move(manifest)),
      capacity_(capacity) {
  observations_.exceptions(std::ios::failbit | std::ios::badbit);
  runtime_.exceptions(std::ios::failbit | std::ios::badbit);
  observations_.open(directory_ + "/observations.csv", std::ios::app);
  runtime_.open(directory_ + "/runtime.csv", std::ios::app);
  worker_ = std::thread([this] { run(); });
}
AsyncEvidenceWriter::~AsyncEvidenceWriter() { close(); }
size_t AsyncEvidenceWriter::pending() const {
  std::lock_guard<std::mutex> l(mutex_);
  return queue_.size();
}
std::string AsyncEvidenceWriter::error() const {
  std::lock_guard<std::mutex> l(mutex_);
  return error_;
}
void AsyncEvidenceWriter::latch(const std::string &e) {
  std::lock_guard<std::mutex> l(mutex_);
  if (error_.empty()) {
    error_ = e;
    failed_after_ = committed_.load();
  }
  cv_.notify_one();
}
void AsyncEvidenceWriter::submit(Job job) {
  std::lock_guard<std::mutex> l(mutex_);
  if (!error_.empty())
    throw std::runtime_error("logging writer failure: " + error_);
  if (stopping_)
    throw std::runtime_error("logging writer closed");
  if (queue_.size() >= capacity_) {
    ++dropped_;
    failed_after_ = committed_.load();
    error_ = "logging queue overflow; acquisition invalidated";
    cv_.notify_one();
    throw std::runtime_error(error_);
  }
  queue_.push_back(std::move(job));
  cv_.notify_one();
}
void AsyncEvidenceWriter::observation(const json &o, const json &r,
                                      const std::string &s) {
  submit({"observation", {{"obs", o}, {"request", r}, {"state", s}}});
}
void AsyncEvidenceWriter::runtime(const json &r) { submit({"runtime", r}); }
void AsyncEvidenceWriter::reset_observations() { submit({"reset", nullptr}); }
void AsyncEvidenceWriter::trials(const json &t, const std::string &e,
                                 const json &s) {
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (t != latest_trials_ || s != latest_skipped_) {
      latest_trials_ = t;
      latest_skipped_ = s;
      ++version_;
    }
    cv_.notify_one();
  }
  submit({"trials", {{"trials", t}, {"error", e}, {"skipped", s}}});
}
void AsyncEvidenceWriter::csv(std::ofstream &out,
                              std::vector<std::string> &columns,
                              const json &row) {
  if (columns.empty()) {
    for (auto it = row.begin(); it != row.end(); ++it)
      columns.push_back(it.key());
    for (size_t i = 0; i < columns.size(); ++i) {
      if (i)
        out << ',';
      out << cell(columns[i]);
    }
    out << '\n';
  }
  for (size_t i = 0; i < columns.size(); ++i) {
    if (i)
      out << ',';
    out << cell(row.value(columns[i], json()));
  }
  out << '\n';
  out.flush();
}
void AsyncEvidenceWriter::checkpoint(json t, json skipped, std::string e,
                                     bool failure) {
  metadata_["evidence_valid"] =
      metadata_.value("evidence_valid", true) && e.empty();
  if (!e.empty()) {
    auto errors = metadata_.value("evidence_errors", json::array());
    if (std::find(errors.begin(), errors.end(), e) == errors.end())
      errors.push_back(e);
    metadata_["evidence_errors"] = errors;
  }
  if (failure)
    for (size_t i = failed_after_; i < t.size(); ++i) {
      auto &r = t[i];
      r["response_completed"] =
          r.value("response_completed", r.value("success", false));
      r["success"] = false;
      r["reason"] = "logging_writer_failure";
      r["evidence_valid"] = false;
      r["evidence_failure"] = e;
      r["mean_acceleration"] = nullptr;
    }
  if (!failure && !t.empty()) {
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(directory_ + "/trials.csv.tmp");
    std::vector<std::string> cols;
    for (auto &row : t)
      for (auto it = row.begin(); it != row.end(); ++it)
        if (std::find(cols.begin(), cols.end(), it.key()) == cols.end())
          cols.push_back(it.key());
    for (size_t i = 0; i < cols.size(); ++i) {
      if (i)
        out << ',';
      out << cell(cols[i]);
    }
    out << '\n';
    for (auto &row : t)
      csv(out, cols, row);
    out.close();
    std::filesystem::rename(directory_ + "/trials.csv.tmp",
                            directory_ + "/trials.csv");
  }
  json payload = {
      {"configuration", config_}, {"metadata", metadata_}, {"trials", t}};
  if (!manifest_.is_null()) {
    payload.update(manifest_);
    payload["skipped"] = skipped;
    payload["schema_version"] =
        config_.value("sweep_protocol", "") == "full_range_step" ? 3 : 2;
    payload["current_signal"] = "avg_iq";
  }
  std::ofstream output;
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.open(directory_ + "/trials.json.tmp");
  output << payload.dump(2) << '\n';
  output.close();
  std::filesystem::rename(directory_ + "/trials.json.tmp",
                          directory_ + "/trials.json");
  if (!failure) {
    committed_ = t.size();
    committed_skips_ = skipped.size();
  }
}
void AsyncEvidenceWriter::failure_checkpoint() {
  json t, s;
  std::string e;
  size_t version;
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (error_.empty() || failure_version_ == version_)
      return;
    t = latest_trials_;
    s = latest_skipped_;
    e = error_;
    version = version_;
  }
  try {
    checkpoint(t, s, "logging_writer_failure: " + e, true);
    std::lock_guard<std::mutex> l(mutex_);
    failure_version_ = version;
  } catch (const std::exception &ex) {
    std::lock_guard<std::mutex> l(mutex_);
    if (error_.find("failure checkpoint unavailable") == std::string::npos)
      error_ += "; failure checkpoint unavailable: " + std::string(ex.what());
    failure_version_ = version;
  }
}
void AsyncEvidenceWriter::run() {
  for (;;) {
    failure_checkpoint();
    Job job;
    {
      std::unique_lock<std::mutex> l(mutex_);
      cv_.wait_for(l, std::chrono::milliseconds(50), [this] {
        return stopping_ || !queue_.empty() ||
               (!error_.empty() && failure_version_ != version_);
      });
      if (queue_.empty()) {
        if (stopping_)
          break;
        continue;
      }
      job = std::move(queue_.front());
      queue_.pop_front();
    }
    try {
      if (job.kind == "trials") {
        if (error().empty())
          checkpoint(job.value["trials"], job.value["skipped"],
                     job.value["error"]);
      } else if (job.kind == "runtime")
        csv(runtime_, runtime_columns_, job.value);
      else if (job.kind == "reset")
        last_wheel_ = last_telemetry_ = 0.;
      else if (job.kind == "observation") {
        auto row = job.value["obs"];
        double w = row.value("wheel_stamp", 0.),
               t = row.value("telemetry_stamp", 0.);
        if (w <= last_wheel_ || t <= last_telemetry_ || std::min(w, t) <= 0)
          continue;
        last_wheel_ = w;
        last_telemetry_ = t;
        auto echo = row["final_request"];
        row.erase("final_request");
        if (config_.value("stage", "") == "speed_hold")
          row.erase("fault_code");
        auto r = job.value["request"];
        row.update({{"state", job.value["state"]},
                    {"request_stamp", r["stamp"]},
                    {"requested_mode", r["mode"]},
                    {"requested_speed", r["speed"]},
                    {"requested_current", r["current"]},
                    {"requested_steering", r["steering"]}});
        for (auto name : {"mode", "speed", "current", "steering", "stamp"})
          row[std::string("final_") + name] =
              echo.is_null() ? json("") : echo[name];
        csv(observations_, observation_columns_, row);
      }
    } catch (const std::exception &e) {
      latch(e.what());
    }
    failure_checkpoint();
  }
  failure_checkpoint();
  try {
    observations_.close();
    runtime_.close();
  } catch (const std::exception &e) {
    latch(e.what());
    failure_checkpoint();
  }
}
void AsyncEvidenceWriter::close() {
  {
    std::lock_guard<std::mutex> l(mutex_);
    stopping_ = true;
    cv_.notify_one();
  }
  if (worker_.joinable())
    worker_.join();
}
} // namespace aims_calibration
