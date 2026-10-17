#include <aims_pp/runtime.hpp>
#include <iostream>
int main(int, char **argv) {
  try {
    aims_pp::json init;
    std::string line;
    std::getline(std::cin, line);
    init = aims_pp::json::parse(line);
    aims_pp::Runtime r(argv[1], init.at("config"),
                       init.value("allow_reverse_velocity", false),
                       init.value("allow_speed_overshoot", false));
    aims_pp::json q;
    while (std::getline(std::cin, line)) {
      q = aims_pp::json::parse(line);
      auto n = q.value("now", int64_t(10000000000));
      auto t = q.value("steady", 10.);
      try {
        const auto op = q.at("op").get<std::string>();
        aims_pp::json out;
        if (op == "load_config")
          out = aims_pp::load_config(q.at("vehicle"),
                                     q.value("tuning", std::string()));
        else if (op == "at")
          out = r.reference_at(q.at("s"));
        else if (op == "project")
          out = r.project(q.at("pose"));
        else if (op == "steering")
          out = r.steering(q.at("values"));
        else if (op == "clock")
          r.observe_clock(n);
        else if (op == "map")
          r.observe_map_identity(q.at("value"));
        else if (op == "valid")
          r.observe_map_valid(q.at("value"));
        else if (op == "health")
          out = r.observe_health(q.at("values"), n, t);
        else if (op == "odom")
          out = r.observe_odometry(q.at("values"), n, t);
        else if (op == "authority")
          r.observe_authority(q.at("value"), t);
        else if (op == "command")
          r.observe_command(q.at("values"), n, t);
        else if (op == "snapshot")
          out = r.snapshot(n, t, q.value("require_anchor", true));
        else if (op == "enable")
          out = r.enable(n, t, q.value("publisher_count", 1));
        else if (op == "disable")
          r.disable();
        else if (op == "tick")
          out = r.tick(n, t, q.value("publisher_count", 1));
        else
          throw std::runtime_error("unknown op");
        std::cout << aims_pp::json({{"ok", true},
                                    {"result", out},
                                    {"status", r.status(n, t)}})
                         .dump()
                  << std::endl;
      } catch (const std::exception &e) {
        std::cout << aims_pp::json({{"ok", false},
                                    {"error", e.what()},
                                    {"status", r.status(n, t)}})
                         .dump()
                  << std::endl;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
