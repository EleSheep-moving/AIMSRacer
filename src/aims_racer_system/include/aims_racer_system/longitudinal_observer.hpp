#ifndef AIMS_RACER_SYSTEM__LONGITUDINAL_OBSERVER_HPP_
#define AIMS_RACER_SYSTEM__LONGITUDINAL_OBSERVER_HPP_
#include <Eigen/Core>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>
namespace aims_racer_system {
struct ObserverConfig {
  bool use_imu{false};
  bool use_gravity{false};
  double jerk_psd{1.0}, disturbance_psd{0.01}, speed_variance{0.0004};
  double force_variance{0.49}, gravity_variance{0.0225}, history_seconds{2.0};
  int64_t max_history_measurements{4096};
};
struct Estimate {
  int64_t stamp_ns;
  double velocity, acceleration;
  std::optional<double> disturbance;
  double acceleration_variance;
};
// Stable source-time replay with a single retained prefix posterior. Snapshots do
// not enter history. Admission is transactional over the entire replay suffix.
class LongitudinalObserver {
 public:
  explicit LongitudinalObserver(const ObserverConfig& config = ObserverConfig{});
  bool receive(const std::string& kind, int64_t stamp_ns, double value,
               std::optional<double> variance = std::nullopt);
  Estimate snapshot(int64_t stamp_ns) const;
  size_t history_size() const {
    return measurements_.size();
  }
  const std::map<std::string, uint64_t>& counters() const {
    return counters_;
  }

 private:
  using Vector = Eigen::VectorXd;
  using Matrix = Eigen::MatrixXd;
  struct Measurement {
    int64_t stamp_ns;
    std::string kind;
    double value, variance;
  };
  struct Checkpoint {
    int64_t stamp_ns;
    Vector state;
    Matrix covariance;
  };
  Checkpoint prior(int64_t stamp_ns) const;
  Checkpoint predict(const Checkpoint&, int64_t stamp_ns) const;
  Checkpoint update(const Checkpoint&, const Measurement&) const;
  void checked(Vector&, Matrix&) const;
  ObserverConfig config_;
  int dimension_;
  int64_t history_ns_;
  Matrix identity_;
  std::map<std::string, Vector> models_;
  std::vector<Measurement> measurements_;
  std::vector<Checkpoint> checkpoints_;
  std::optional<Checkpoint> anchor_;
  std::map<std::string, uint64_t> counters_;
};
}  // namespace aims_racer_system
#endif
