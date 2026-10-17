// Native/Python parity probe. Commands: M kind stamp value [variance], S stamp.
// Optional args: history_seconds max_history_measurements jerk_psd disturbance_psd
//                speed_variance force_variance. Uses the three-state IMU model.
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "aims_racer_system/longitudinal_observer.hpp"
int main(int argc, char** argv) {
  using namespace aims_racer_system;
  ObserverConfig config;
  config.use_imu = true;
  if (argc > 1)
    config.history_seconds = std::stod(argv[1]);
  if (argc > 2)
    config.max_history_measurements = std::stoll(argv[2]);
  if (argc > 3)
    config.jerk_psd = std::stod(argv[3]);
  if (argc > 4)
    config.disturbance_psd = std::stod(argv[4]);
  if (argc > 5)
    config.speed_variance = std::stod(argv[5]);
  if (argc > 6)
    config.force_variance = std::stod(argv[6]);
  LongitudinalObserver observer(config);
  std::string line;
  std::cout << std::setprecision(17);
  while (std::getline(std::cin, line)) {
    std::istringstream input(line);
    std::string op;
    input >> op;
    if (op.empty() || op[0] == '#')
      continue;
    try {
      if (op == "M") {
        std::string kind, value_text, variance_text;
        int64_t stamp;
        if (!(input >> kind >> stamp >> value_text))
          throw std::invalid_argument("invalid measurement command");
        const double value = std::stod(value_text);
        std::optional<double> variance;
        if (input >> variance_text)
          variance = std::stod(variance_text);
        const bool admitted = observer.receive(kind, stamp, value, variance);
        std::cout << "M " << admitted << ' ' << observer.history_size();
      } else if (op == "S") {
        int64_t stamp;
        if (!(input >> stamp))
          throw std::invalid_argument("invalid snapshot command");
        const auto e = observer.snapshot(stamp);
        std::cout << "S " << e.stamp_ns << ' ' << e.velocity << ' ' << e.acceleration << ' '
                  << *e.disturbance << ' ' << e.acceleration_variance;
      } else if (op == "C") {
        std::cout << "C";
        for (const auto& item : observer.counters())
          std::cout << ' ' << item.first << '=' << item.second;
      } else
        throw std::invalid_argument("unknown command");
      std::cout << '\n';
    } catch (const std::exception&) {
      std::cout << "E\n";
    }
  }
}
