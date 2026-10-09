#pragma once
#include "aims_mpcc_rt/output.hpp"
#include <limits>
namespace aims_mpcc_rt {
struct ExecutionCertificate {
  bool success{};
  double max_utilization{},max_corridor_violation{},duration_s{};
  std::size_t checks{};
  std::string reason;
  State final_state{};
};
// Nominal held-output certificate for the existing ideal-acceleration model.
// The supplied sampler is copied; authoritative live state is never modified.
ExecutionCertificate certify_execution(const Plan&,const OutputSampler&,const State& initial,
  const Alignment&,const Config&,const Reference&,double now,double ttl,
  double speed_cap=std::numeric_limits<double>::infinity(),
  const std::vector<double>& output_intervals={});
}
