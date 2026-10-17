#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
#include <vector>
namespace aims_calibration {
using json = nlohmann::json;
std::vector<std::string> check_start_native(const json &);
struct SteadyGuard {
  double last_timer = -1., last_ros = -1., last_changed = 0.;
  void reset() {
    last_timer = -1.;
    last_ros = -1.;
    last_changed = 0.;
  }
  std::string check(double ros, double steady, bool selected, double freshness,
                    bool protected_flow, size_t producers,
                    const std::map<std::string, double> &receipts);
};
class AsyncEvidenceWriter {
public:
  AsyncEvidenceWriter(std::string directory, json config, json metadata,
                      json manifest, size_t capacity = 128);
  ~AsyncEvidenceWriter();
  void observation(const json &, const json &, const std::string &);
  void runtime(const json &);
  void trials(const json &, const std::string &, const json &);
  void reset_observations();
  void close();
  size_t pending() const;
  size_t dropped() const { return dropped_; }
  size_t committed_trials() const { return committed_; }
  size_t committed_skips() const { return committed_skips_; }
  size_t failed_after_trial_count() const { return failed_after_; }
  std::string error() const;

private:
  struct Job {
    std::string kind;
    json value;
  };
  std::string directory_;
  json config_, metadata_, manifest_, latest_trials_ = json::array(),
                                      latest_skipped_ = json::array();
  size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Job> queue_;
  std::thread worker_;
  bool stopping_ = false;
  std::string error_;
  std::atomic<size_t> dropped_{0}, committed_{0}, committed_skips_{0},
      failed_after_{0};
  size_t version_ = 0, failure_version_ = size_t(-1);
  std::ofstream observations_, runtime_;
  std::vector<std::string> observation_columns_, runtime_columns_;
  double last_wheel_ = 0., last_telemetry_ = 0.;
  void submit(Job);
  void run();
  void latch(const std::string &);
  void checkpoint(json, json, std::string, bool failure = false);
  void failure_checkpoint();
  void csv(std::ofstream &, std::vector<std::string> &, const json &);
};
} // namespace aims_calibration
