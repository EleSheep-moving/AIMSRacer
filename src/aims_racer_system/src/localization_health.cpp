#include "aims_racer_system/localization_health.hpp"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace aims_racer_system
{
std::uint64_t protocol_unsigned(const std::string & value)
{
  if (value.empty()) {throw std::invalid_argument("expected unsigned decimal integer");}
  for (char c : value) {
    if (c < '0' || c > '9') {throw std::invalid_argument("expected unsigned decimal integer");}
  }
  std::uint64_t result{};
  auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
    throw std::invalid_argument("unsigned protocol integer overflow");
  }
  return result;
}

bool protocol_boolean(const std::string & value)
{
  if (value == "true") {return true;}
  if (value == "false") {return false;}
  throw std::invalid_argument("expected protocol boolean");
}

std::string number(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

static double age_seconds(std::int64_t now, std::uint64_t stamp)
{
  return static_cast<double>((static_cast<long double>(now) - stamp) * 1e-9L);
}

AnchorHealth::AnchorHealth(double max_age, int recovery_commits)
: max_age_(max_age), recovery_commits_(recovery_commits)
{
  if (!std::isfinite(max_age) || max_age <= 0 || recovery_commits < 1) {
    throw std::invalid_argument("positive anchor age and recovery commit count required");
  }
}

void AnchorHealth::invalidate(const std::string & reason)
{
  ready_ = false;
  recovering_ = last_anchor_stamp_ns_ != 0;
  accepts_ = 0;
  reason_ = reason;
}

bool AnchorHealth::observe(const HealthFields & values, std::int64_t attempt,
  std::int64_t now, double received)
{
  std::string epoch;
  std::uint64_t sequence, anchor, stamp;
  bool committed, ready;
  try {
    if (values.at("protocol_version") != "1" || values.at("epoch").empty()) {
      throw std::invalid_argument("invalid anchor protocol");
    }
    epoch = values.at("epoch");
    sequence = protocol_unsigned(values.at("event_sequence"));
    anchor = protocol_unsigned(values.at("anchor_sequence"));
    stamp = protocol_unsigned(values.at("last_anchor_stamp_ns"));
    committed = protocol_boolean(values.at("anchor_committed"));
    ready = protocol_boolean(values.at("ready"));
  } catch (const std::exception &) {
    invalidate("invalid_anchor_protocol");
    return false;
  }
  if (retired_epochs_.count(epoch)) {return false;}
  if (epoch != epoch_) {
    if (!epoch_.empty()) {retired_epochs_.insert(epoch_);}
    epoch_ = epoch;
    event_sequence_.reset();
    anchor_sequence_ = last_anchor_stamp_ns_ = accepts_ = 0;
    last_commit_received_.reset();
    last_clock_.reset();
    ready_ = recovering_ = rejected_ = clock_invalid_ = false;
    reason_ = "new_epoch";
  }
  if (event_sequence_ && sequence <= *event_sequence_) {return false;}
  event_sequence_ = sequence;
  if (now < 0 || attempt < 0 || attempt > now) {
    invalidate("future_or_invalid_attempt_stamp");
    return false;
  }
  if (clock_invalid_) {return false;}
  if (last_commit_received_ &&
    (received - *last_commit_received_ > max_age_ ||
    static_cast<long double>(now) - last_anchor_stamp_ns_ >
    static_cast<std::int64_t>(max_age_ * 1e9)))
  {
    invalidate("anchor_timeout");
  }
  if (!committed) {
    rejected_ = true;
    accepts_ = 0;
    auto reason = values.find("reason");
    reason_ = reason == values.end() ? "anchor_rejected" : reason->second;
    if (!ready) {invalidate(reason_);}
    return true;
  }
  if (!ready || anchor <= anchor_sequence_ || stamp != static_cast<std::uint64_t>(attempt) ||
    stamp <= last_anchor_stamp_ns_ ||
    static_cast<long double>(now) - stamp > static_cast<std::int64_t>(max_age_ * 1e9))
  {
    invalidate("invalid_commit_provenance");
    return false;
  }
  anchor_sequence_ = anchor;
  last_anchor_stamp_ns_ = stamp;
  last_commit_received_ = received;
  rejected_ = false;
  auto reason = values.find("reason");
  reason_ = reason == values.end() ? "ok" : reason->second;
  ++accepts_;
  if (!recovering_ || accepts_ >= static_cast<std::uint64_t>(recovery_commits_)) {
    ready_ = true;
    recovering_ = false;
  }
  return true;
}

HealthFields AnchorHealth::evaluate(std::int64_t now, double mono, bool inputs_present)
{
  if (last_clock_ && now < *last_clock_) {rewind();}
  last_clock_ = now;
  const double age = last_anchor_stamp_ns_ ? age_seconds(now, last_anchor_stamp_ns_) :
    std::numeric_limits<double>::infinity();
  const double received_age = last_commit_received_ ? mono - *last_commit_received_ :
    std::numeric_limits<double>::infinity();
  const bool fresh = age >= 0 && age <= max_age_ && received_age >= 0 &&
    received_age <= max_age_ && inputs_present && !clock_invalid_;
  if (!fresh) {invalidate(inputs_present ? "anchor_timeout" : "inputs_stale");}
  const std::string state = ready_ && fresh ? (rejected_ ? "hold" : "tracking") :
    (fresh && accepts_ ? "recovering" : "lost");
  return {{"protocol_version", "1"}, {"epoch", epoch_},
    {"event_sequence", event_sequence_ ? std::to_string(*event_sequence_) : "-1"},
    {"anchor_sequence", std::to_string(anchor_sequence_)}, {"state", state},
    {"ready", ready_ && fresh ? "true" : "false"}, {"source_age", number(age)},
    {"receive_age", number(received_age)}, {"last_anchor_stamp_ns", std::to_string(last_anchor_stamp_ns_)},
    {"accepted_streak", std::to_string(accepts_)}, {"reason", reason_}};
}
}  // namespace aims_racer_system
