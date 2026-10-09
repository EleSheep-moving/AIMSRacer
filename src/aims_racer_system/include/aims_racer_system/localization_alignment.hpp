#pragma once
#include "aims_racer_system/localization_health.hpp"
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
namespace aims_racer_system
{
// The authoritative anchor packet is the only source of this transform.
// Diagnostic scan matching and generic TF publications cannot establish a record.
class CommittedAlignment
{
public:
  bool observe(const HealthFields & values, bool accepted,
    const std::string & health_epoch, std::uint64_t health_stamp)
  {
    if (record_ && record_->epoch != health_epoch) {clear();}
    try {
      const bool committed = protocol_boolean(values.at("anchor_committed"));
      // Even a duplicate event must not alter an already accepted sequence.
      if (record_ && values.at("epoch") == record_->epoch &&
        protocol_unsigned(values.at("anchor_sequence")) == record_->sequence)
      {
        if (protocol_unsigned(values.at("last_anchor_stamp_ns")) != record_->stamp ||
          ((committed || complete_payload(values)) && components(values) != record_->pose))
        {
          clear();
          return false;
        }
      }
      if (record_ && record_->stamp != health_stamp) {clear();}
      if (accepted && committed) {
        Record next{values.at("epoch"), protocol_unsigned(values.at("anchor_sequence")),
          protocol_unsigned(values.at("last_anchor_stamp_ns")), components(values)};
        if (next.epoch != health_epoch || next.stamp != health_stamp || !next.sequence || !next.stamp) {
          throw std::invalid_argument("alignment provenance mismatch");
        }
        record_ = std::move(next);
      }
      return true;
    } catch (const std::exception &) {
      // Invalid commits must not leave a previously accepted payload available.
      clear();
      return false;
    }
  }

  HealthFields fields(const HealthFields & health) const
  {
    HealthFields result{{"alignment_valid", "false"}, {"alignment_epoch", ""},
      {"alignment_anchor_sequence", "0"}, {"alignment_stamp_ns", "0"}};
    if (!record_) {return result;}
    try {
      if (health.at("epoch") != record_->epoch ||
        protocol_unsigned(health.at("anchor_sequence")) != record_->sequence ||
        protocol_unsigned(health.at("last_anchor_stamp_ns")) != record_->stamp) {return result;}
    } catch (const std::exception &) {return result;}
    result["alignment_valid"] = "true";
    result["alignment_epoch"] = record_->epoch;
    result["alignment_anchor_sequence"] = std::to_string(record_->sequence);
    result["alignment_stamp_ns"] = std::to_string(record_->stamp);
    for (std::size_t i = 0; i < keys_.size(); ++i) {
      result[std::string("map_odom_") + keys_[i]] = number(record_->pose[i]);
    }
    return result;
  }
  void clear() {record_.reset();}

private:
  inline static constexpr std::array<const char *, 7> keys_{"x", "y", "z", "qx", "qy", "qz", "qw"};
  struct Record
  {
    std::string epoch;
    std::uint64_t sequence, stamp;
    std::array<double, 7> pose;
  };
  static bool complete_payload(const HealthFields & values)
  {
    for (auto key : keys_) {if (!values.count(std::string("map_odom_") + key)) {return false;}}
    return true;
  }
  static std::array<double, 7> components(const HealthFields & values)
  {
    std::array<double, 7> result{};
    for (std::size_t i = 0; i < keys_.size(); ++i) {
      const auto & text = values.at(std::string("map_odom_") + keys_[i]);
      std::size_t consumed = 0;
      result[i] = std::stod(text, &consumed);
      if (consumed != text.size() || !std::isfinite(result[i])) {
        throw std::invalid_argument("finite complete alignment component required");
      }
    }
    double norm_squared = 0;
    for (std::size_t i = 3; i < 7; ++i) {norm_squared += result[i] * result[i];}
    if (!std::isfinite(norm_squared) || std::abs(std::sqrt(norm_squared) - 1.) > .01) {
      throw std::invalid_argument("unit alignment quaternion required");
    }
    return result;
  }
  std::optional<Record> record_;
};
}
