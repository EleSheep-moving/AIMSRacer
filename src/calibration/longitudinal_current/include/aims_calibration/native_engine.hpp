#pragma once
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

namespace aims_calibration {
class NativeEngine {
public:
  using json = nlohmann::json;
  explicit NativeEngine(const json &prepared);
  ~NativeEngine();
  NativeEngine(NativeEngine &&) noexcept;
  NativeEngine &operator=(NativeEngine &&) noexcept;
  NativeEngine(const NativeEngine &) = delete;
  NativeEngine &operator=(const NativeEngine &) = delete;
  json tick(double now, const json &observation);
  void sent(const json &request);
  void enable(bool value,
              double now = std::numeric_limits<double>::quiet_NaN());
  void fault(const std::string &reason);
  void invalidate_evidence(std::size_t first_trial, const std::string &reason);
  // Exclude acquisition buffers before copying for operational checkpoints.
  json snapshot(bool include_raw_samples = false) const;
  // Bounded current-state copy; results are represented by counts.
  json status() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace aims_calibration
