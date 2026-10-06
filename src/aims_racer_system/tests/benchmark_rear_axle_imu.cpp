#include "aims_racer_system/rear_axle_imu.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace aims_racer_system;

template<typename Function>
void measure(const char * name, Function function)
{
  constexpr int repetitions = 10000;
  std::vector<double> times;
  times.reserve(repetitions);
  volatile double checksum = 0.0;
  for (int i = 0; i < repetitions; ++i) {
    const auto started = std::chrono::steady_clock::now();
    checksum += function();
    times.push_back(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count());
  }
  std::sort(times.begin(), times.end());
  std::cout << name << " median_ms=" << times[repetitions / 2]
            << " p95_ms=" << times[repetitions * 95 / 100] << '\n';
  (void)checksum;
}

int main()
{
  std::cout << std::fixed << std::setprecision(6);
  AngularHistory history;
  const Vector3 omega(0.01, 0.02, 0.3);
  const Matrix3 covariance = Matrix3::Identity() * 0.01;
  for (int i = 0; i < 400; ++i) {history.add(i * 0.005, omega, covariance);}
  const double target = 399 * 0.005;
  for (double age : {0.025, 0.05, 0.1, 0.15, 0.25}) {
    std::cout << "orientation_age_s=" << age << ' ';
    measure("orientation", [&]() {
        return history.orientation(target - age, Quaternion::Identity(), target)->w();
      });
  }
  measure("derivative", [&]() {return history.derivative()->alpha.norm();});
  measure("force_compensation", [&]() {
      return compensate_force(Vector3(0., 0., 9.80665), omega, Vector3::Zero(),
        Vector3(0.3, 0., 0.), covariance, covariance, covariance).force.norm();
    });
}
