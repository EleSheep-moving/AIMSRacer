#pragma once
#include <array>
#include <cstdint>
namespace aims_racer_system::yaw_rate {
struct Config {
  std::array<double,4> mount_xyzw{0.,0.,0.,1.};
  double process_psd=4.; // alpha random-walk PSD, rad^2/s^5; effective model tuning
  double measurement_variance=1e-4; // rad^2/s^2; not calibrated precision
  double initial_alpha_variance=100.; // rad^2/s^4
  double max_gap_s=.03;
};
struct Sample {int64_t stamp_ns=0;std::array<double,3> gyro_rad_s{};};
struct Estimate {
  int64_t stamp_ns=0;bool valid=false;
  double measured_rate=0.,rate=0.,alpha=0.,rate_variance=0.,alpha_variance=0.,cross_covariance=0.;
  unsigned consecutive_samples=0;
};
struct Counters {uint64_t accepted=0,invalid=0,duplicate=0,out_of_order=0,gaps=0,numerical_error=0;};
// Single source stream; source-order rejection, no receive-time prediction or bias learning.
class Observer {
 public:
  explicit Observer(const Config & config=Config{});
  bool add(const Sample & sample);
  Estimate latest() const {return estimate_;}
  Counters counters() const {return counters_;}
  void reset(); // Clears state; cumulative diagnostics are retained.
 private:
  Config config_;std::array<double,3> project_{};Estimate estimate_;Counters counters_;
};
}
