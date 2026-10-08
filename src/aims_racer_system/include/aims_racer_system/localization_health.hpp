#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>

namespace aims_racer_system
{
using HealthFields = std::map<std::string, std::string>;
std::uint64_t protocol_unsigned(const std::string & value);
bool protocol_boolean(const std::string & value);
std::string number(double value);

// Authoritative commits own health; TF timer publications never refresh it.
class AnchorHealth
{
public:
  explicit AnchorHealth(double max_age = .5, int recovery_commits = 3);
  void invalidate(const std::string & reason);
  bool observe(const HealthFields & values, std::int64_t attempt_stamp_ns,
    std::int64_t now_ns, double received);
  HealthFields evaluate(std::int64_t now_ns, double monotonic_now, bool inputs_present);
  const std::string & epoch() const {return epoch_;}
  std::uint64_t stamp_ns() const {return last_anchor_stamp_ns_;}
  void rewind() {clock_invalid_ = true; invalidate("clock_rewind");}

private:
  double max_age_;
  int recovery_commits_;
  std::string epoch_;
  std::set<std::string> retired_epochs_;
  std::optional<std::uint64_t> event_sequence_;
  std::uint64_t anchor_sequence_{0}, last_anchor_stamp_ns_{0};
  std::optional<double> last_commit_received_;
  std::optional<std::int64_t> last_clock_;
  bool ready_{false}, recovering_{false}, rejected_{false}, clock_invalid_{false};
  std::uint64_t accepts_{0};
  std::string reason_{"waiting_for_anchor"};
};
}  // namespace aims_racer_system
