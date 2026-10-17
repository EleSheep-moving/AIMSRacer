#include "aims_calibration/native_engine.hpp"
#include <chrono>
#include <iostream>
int main() {
  using json = nlohmann::json;
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      auto trace = json::parse(line);
      aims_calibration::NativeEngine engine(trace.at("prepared"));
      if (trace.contains("benchmark_status")) {
        engine.enable(true, 1.);
        json previous = nullptr;
        auto step = [&](int i) {
          const double now = 1. + i * .01;
          json observation = {
              {"wheel_stamp", now},     {"telemetry_stamp", now},
              {"velocity", 1.},         {"current_motor", 99.},
              {"avg_iq", 5.},           {"voltage_input", 16.},
              {"erpm", 100.},           {"authority", true},
              {"authority_stamp", now}, {"rc_stamp", now},
              {"rc_valid", true},       {"straight", true},
              {"region", "benchmark"}};
          if (!previous.is_null()) {
            previous["stamp"] = now;
            observation["final_request"] = previous;
            observation["final_received"] = now;
          }
          previous = engine.tick(now, observation);
          engine.sent(previous);
        };
        auto measure = [&]() {
          const auto started = std::chrono::steady_clock::now();
          for (int i = 0; i < 1000; ++i) {
            const auto live = engine.status();
            for (const auto key :
                 {"samples", "active_samples", "trials", "skipped", "history",
                  "tracking_observations"})
              if (live.contains(key))
                throw std::runtime_error("status copied acquisition history");
          }
          return std::chrono::duration<double, std::micro>(
                     std::chrono::steady_clock::now() - started)
                     .count() /
                 1000.;
        };
        for (int i = 1; i <= 100; ++i)
          step(i);
        const auto small = engine.snapshot(true).at("samples").size();
        const auto small_us = measure();
        for (int i = 101; i <= trace.at("benchmark_status").get<int>(); ++i)
          step(i);
        const auto large = engine.snapshot(true).at("samples").size();
        const auto large_us = measure();
        const auto checkpoint = engine.snapshot();
        for (const auto key : {"samples", "active_samples", "history",
                               "tracking_observations", "tracking_window"})
          if (checkpoint.contains(key))
            throw std::runtime_error("checkpoint copied acquisition history");
        std::cout << json{{"benchmark",
                           {{"small_samples", small},
                            {"large_samples", large},
                            {"small_status_us", small_us},
                            {"large_status_us", large_us},
                            {"large_enabled", engine.status().at("enabled")}}}}
                         .dump()
                  << '\n';
        continue;
      }
      json output = json::array();
      for (const auto &event : trace.at("events")) {
        json request = nullptr;
        const auto op = event.at("op").get<std::string>();
        if (op == "tick")
          request = engine.tick(event.at("now"), event.at("observation"));
        else if (op == "sent")
          engine.sent(event.at("request"));
        else if (op == "enable")
          engine.enable(
              event.at("value"),
              event.value("now", std::numeric_limits<double>::quiet_NaN()));
        else if (op == "fault")
          engine.fault(event.at("reason"));
        else if (op == "invalidate_evidence")
          engine.invalidate_evidence(event.at("first_trial"),
                                     event.at("reason"));
        else
          throw std::runtime_error("unknown trace event");
        auto snapshot = engine.snapshot(true);
        for (const auto key : {"samples", "active_samples",
                               "tracking_observations", "tracking_window"}) {
          const auto value = snapshot.at(key);
          snapshot[key] = {
              {"size", value.size()},
              {"last", value.empty() ? json(nullptr) : value.back()}};
        }
        output.push_back({{"request", request}, {"snapshot", snapshot}});
      }
      std::cout << json{{"output", output}}.dump() << '\n';
    } catch (const std::exception &e) {
      std::cout << json{{"error", e.what()}}.dump() << '\n';
    }
  }
}
