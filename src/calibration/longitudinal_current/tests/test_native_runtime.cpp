#include "aims_calibration/native_runtime.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace aims_calibration;
int main() {
  json ready = {
      {"state", "IDLE"},
      {"enabled", false},
      {"enable_count", 0},
      {"logging_error", ""},
      {"logging_dropped", 0},
      {"freshness", .2},
      {"wheel_speed", 0.},
      {"telemetry_valid", true},
      {"current_signal", "avg_iq"},
      {"authority", true},
      {"rc_valid", true},
      {"final_zero", true},
      {"publishers", {{"calibration", 1}, {"final", 1}, {"drive", 0}}},
      {"driver_startup_disabled", true},
      {"driver_parameter_age", .1},
      {"artifact_error", ""},
      {"pp", false}};
  ready["ages"] = json::object();
  for (auto name : {"wheel", "telemetry", "final", "rc", "authority"}) {
    ready["ages"][std::string(name) + "_source"] = .01;
    ready["ages"][std::string(name) + "_receipt"] = .01;
  }
  assert(check_start_native(ready).empty());
  ready["ages"]["wheel_source"] = .21;
  assert(check_start_native(ready).at(0) == "wheel_source_stale_or_future");
  ready["ages"]["wheel_source"] = .01;
  ready["ages"]["wheel_receipt"] = .21;
  assert(check_start_native(ready).at(0) == "wheel_receipt_stale_or_future");
  ready["ages"]["wheel_receipt"] = .01;
  ready["publishers"]["final"] = 2;
  assert(check_start_native(ready).at(0) ==
         "exclusive_command_producers_required");
  SteadyGuard guard;
  std::map<std::string, double> receipts;
  for (auto name : {"wheel", "telemetry", "rc", "authority", "final"})
    receipts[name] = 1.;
  assert(guard.check(5., 1., true, .2, false, 1, receipts).empty());
  for (auto &item : receipts)
    item.second = 1.21;
  assert(guard.check(5., 1.21, true, .2, false, 1, receipts) ==
         "publication_timer_stale");
  guard.reset();
  assert(guard.check(5., 1., true, .2, false, 1, receipts) ==
         "wheel_receipt_stale");
  guard.reset();
  for (int i = 0; i < 13; ++i) {
    double t = 1. + i * .02;
    for (auto &item : receipts)
      item.second = t;
    auto reason = guard.check(5., t, true, .2, false, 1, receipts);
    if (i == 12)
      assert(reason == "ros_clock_stalled");
  }
  auto dir =
      std::filesystem::temp_directory_path() / "calibration-native-writer-test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  AsyncEvidenceWriter writer(dir.string(), {{"stage", "speed_hold"}},
                             {{"current_signal", "avg_iq"}}, nullptr, 0);
  bool overflow = false;
  try {
    writer.runtime({{"state", "MEASURE"}});
  } catch (const std::runtime_error &) {
    overflow = true;
  }
  assert(overflow);
  assert(writer.dropped() == 1);
  try {
    writer.trials(json::array({{{"success", true}, {"evidence_valid", true}}}),
                  "overflow", json::array());
  } catch (const std::runtime_error &) {
  }
  writer.close();
  std::ifstream input(dir / "trials.json");
  json payload;
  input >> payload;
  assert(!payload["metadata"]["evidence_valid"].get<bool>());
  assert(!payload["trials"][0]["success"].get<bool>());
  std::filesystem::remove_all(dir);
  std::cout << "native runtime contracts PASS\n";
}
