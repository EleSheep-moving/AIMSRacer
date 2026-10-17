#include "aims_racer_system/longitudinal_observer.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace aims_racer_system {
LongitudinalObserver::LongitudinalObserver(const ObserverConfig& config)
    : config_(config),
      dimension_(config.use_imu ? 3 : 2),
      history_ns_(0),
      identity_(Matrix::Identity(dimension_, dimension_)) {
  if (config.use_gravity && !config.use_imu) {
    throw std::invalid_argument("gravity constraints require IMU disturbance state");
  }
  if (config.max_history_measurements < 2) {
    throw std::invalid_argument("max_history_measurements must be at least two");
  }
  for (double value : {config.jerk_psd, config.disturbance_psd}) {
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("invalid process noise");
  }
  for (double value : {config.speed_variance, config.force_variance, config.gravity_variance,
                       config.history_seconds}) {
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("invalid variance/history");
  }
  const double history = config.history_seconds * 1e9;
  if (!std::isfinite(history) || history < 1.)
    throw std::invalid_argument("invalid history nanoseconds");
  // Larger finite windows cover the entire valid int64 timestamp domain.
  history_ns_ = history >= static_cast<double>(std::numeric_limits<int64_t>::max())
                    ? std::numeric_limits<int64_t>::max()
                    : static_cast<int64_t>(std::nearbyint(history));
  Vector h = Vector::Zero(dimension_);
  h[0] = 1.;
  models_["speed"] = h;
  if (config.use_imu) {
    h.setZero();
    h[1] = h[2] = 1.;
    models_["force"] = h;
  }
  if (config.use_gravity) {
    h.setZero();
    h[2] = 1.;
    models_["gravity"] = h;
  }
  for (const auto* name :
       {"received", "admitted", "delayed", "too_old", "invalid", "disabled", "numerical_failure",
        "replayed", "trimmed", "count_trimmed", "max_history"})
    counters_[name] = 0;
}
LongitudinalObserver::Checkpoint LongitudinalObserver::prior(int64_t stamp) const {
  Matrix covariance = Matrix::Identity(dimension_, dimension_) * 4.;
  covariance(0, 0) = 1.;
  return {stamp, Vector::Zero(dimension_), covariance};
}
void LongitudinalObserver::checked(Vector& state, Matrix& covariance) const {
  covariance = ((covariance + covariance.transpose()) * .5).eval();
  if (!state.allFinite() || !covariance.allFinite())
    throw std::runtime_error("nonfinite posterior");
  Eigen::SelfAdjointEigenSolver<Matrix> solver(covariance, Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success)
    throw std::runtime_error("covariance eigensolver failed");
  const double minimum = solver.eigenvalues()[0];
  const double tolerance = 1e-12 * std::max(1., covariance.cwiseAbs().maxCoeff());
  if (minimum < -tolerance)
    throw std::runtime_error("covariance not positive semidefinite");
  if (minimum < 0.)
    covariance += identity_ * (-minimum);
}
LongitudinalObserver::Checkpoint LongitudinalObserver::predict(const Checkpoint& point,
                                                               int64_t stamp) const {
  if (stamp < point.stamp_ns)
    throw std::invalid_argument("cannot predict backwards");
  const double dt = static_cast<double>(stamp - point.stamp_ns) * 1e-9;
  Matrix transition = identity_;
  transition(0, 1) = dt;
  Matrix process = Matrix::Zero(dimension_, dimension_);
  process(0, 0) = config_.jerk_psd * (dt * dt * dt / 3.);
  process(0, 1) = process(1, 0) = config_.jerk_psd * (dt * dt / 2.);
  process(1, 1) = config_.jerk_psd * dt;
  if (config_.use_imu)
    process(2, 2) = config_.disturbance_psd * dt;
  Vector state = transition * point.state;
  Matrix covariance = transition * point.covariance * transition.transpose() + process;
  checked(state, covariance);
  return {stamp, state, covariance};
}
LongitudinalObserver::Checkpoint LongitudinalObserver::update(const Checkpoint& point,
                                                              const Measurement& m) const {
  Checkpoint result = predict(point, m.stamp_ns);
  const Vector& h = models_.at(m.kind);
  const Vector projected = result.covariance * h;
  const double innovation_variance = h.dot(projected) + m.variance;
  if (!std::isfinite(innovation_variance) || innovation_variance <= 0.)
    throw std::runtime_error("invalid innovation variance");
  const Vector gain = projected / innovation_variance;
  const double innovation = m.value - h.dot(result.state);
  if (!std::isfinite(innovation))
    throw std::runtime_error("nonfinite innovation");
  result.state += gain * innovation;
  const Matrix residual = identity_ - gain * h.transpose();
  result.covariance =
      (residual * result.covariance * residual.transpose() + m.variance * (gain * gain.transpose()))
          .eval();
  checked(result.state, result.covariance);
  return result;
}
bool LongitudinalObserver::receive(const std::string& kind, int64_t stamp, double value,
                                   std::optional<double> variance) {
  ++counters_["received"];
  if ((kind != "speed" && kind != "force" && kind != "gravity") || stamp < 0) {
    ++counters_["invalid"];
    return false;
  }
  if (!models_.count(kind)) {
    ++counters_["disabled"];
    return false;
  }
  const double noise = variance.value_or(kind == "speed"   ? config_.speed_variance
                                         : kind == "force" ? config_.force_variance
                                                           : config_.gravity_variance);
  if (!std::isfinite(value) || !std::isfinite(noise) || noise <= 0.) {
    ++counters_["invalid"];
    return false;
  }
  const int64_t newest = measurements_.empty() ? stamp : measurements_.back().stamp_ns;
  if (stamp < newest - history_ns_ || (anchor_ && stamp < anchor_->stamp_ns)) {
    ++counters_["too_old"];
    return false;
  }
  const auto where =
      std::upper_bound(measurements_.begin(), measurements_.end(), stamp,
                       [](int64_t s, const Measurement& m) { return s < m.stamp_ns; });
  const size_t index = static_cast<size_t>(where - measurements_.begin());
  const bool delayed = index < measurements_.size();
  const Measurement measurement{stamp, kind, value, noise};
  Checkpoint point = index ? checkpoints_[index - 1] : anchor_.value_or(prior(stamp));
  std::vector<Checkpoint> suffix;
  suffix.reserve(measurements_.size() - index + 1);
  try {
    point = update(point, measurement);
    suffix.push_back(point);
    if (delayed)
      for (size_t i = index; i < measurements_.size(); ++i) {
        point = update(point, measurements_[i]);
        suffix.push_back(point);
      }
  } catch (const std::runtime_error&) {
    ++counters_["numerical_failure"];
    return false;
  }
  measurements_.insert(measurements_.begin() + index, measurement);
  checkpoints_.erase(checkpoints_.begin() + index, checkpoints_.end());
  checkpoints_.insert(checkpoints_.end(), suffix.begin(), suffix.end());
  ++counters_["admitted"];
  if (delayed) {
    ++counters_["delayed"];
    counters_["replayed"] += suffix.size();
  }
  const int64_t cutoff = measurements_.back().stamp_ns - history_ns_;
  const size_t time_trim = static_cast<size_t>(
      std::lower_bound(measurements_.begin(), measurements_.end(), cutoff,
                       [](const Measurement& m, int64_t s) { return m.stamp_ns < s; }) -
      measurements_.begin());
  const size_t cap = static_cast<size_t>(config_.max_history_measurements);
  const size_t count_trim = measurements_.size() > cap ? measurements_.size() - cap : 0;
  const size_t trim = std::max(time_trim, count_trim);
  if (trim) {
    anchor_ = checkpoints_[trim - 1];
    measurements_.erase(measurements_.begin(), measurements_.begin() + trim);
    checkpoints_.erase(checkpoints_.begin(), checkpoints_.begin() + trim);
    counters_["trimmed"] += trim;
    counters_["count_trimmed"] += count_trim > time_trim ? count_trim - time_trim : 0;
  }
  counters_["max_history"] =
      std::max(counters_["max_history"], static_cast<uint64_t>(history_size()));
  return true;
}
Estimate LongitudinalObserver::snapshot(int64_t stamp) const {
  if (stamp < 0)
    throw std::invalid_argument("negative snapshot stamp");
  if (checkpoints_.empty())
    throw std::runtime_error("no observation admitted");
  const auto point = predict(checkpoints_.back(), stamp);
  return {stamp, point.state[0], point.state[1],
          config_.use_imu ? std::optional<double>(point.state[2]) : std::nullopt,
          point.covariance(1, 1)};
}
}  // namespace aims_racer_system
