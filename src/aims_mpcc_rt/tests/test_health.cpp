#include "aims_mpcc_rt/health.hpp"
#include <cassert>
#include <iostream>
using aims_mpcc_rt::LocalizationHealth;
static LocalizationHealth::Values record(const std::string& epoch, int h, int a,
                                         long long ns, bool ready=true) {
  return {{"protocol_version","1"},{"epoch",epoch},{"health_sequence",std::to_string(h)},
          {"anchor_sequence",std::to_string(a)},{"last_anchor_stamp_ns",std::to_string(ns)},
          {"ready",ready?"true":"false"},{"state","tracking"}};
}
int main() {
  LocalizationHealth health;
  assert(!health.usable(1000000000,10));
  assert(!health.observe(record("a",1,1,1000000000),1000000000,10));
  assert(health.usable(1100000000,10.1));
  assert(!health.usable(1400000000,10.4));  // heartbeat expires independently
  health.observe(record("a",1,1,1000000000),1400000000,10.4);
  assert(!health.usable(1400000000,10.4));  // duplicate cannot refresh heartbeat
  health.observe(record("a",2,1,1000000000,false),1400000000,10.4);
  health.observe(record("a",3,1,1000000000,true),1400000000,10.4);
  assert(!health.usable(1400000000,10.4));  // loss requires a newer anchor
  health.observe(record("a",4,2,1400000000),1400000000,10.4);
  assert(health.usable(1400000000,10.4));
  assert(health.observe(record("b",1,1,1500000000),1500000000,10.5));
  assert(!health.observe(record("a",99,99,1500000000),1500000000,10.5));
  assert(health.epoch()=="b");
  assert(!health.usable(1400000000,10.5));  // ROS clock before anchor
  health.observe(record("b",2,1,1600000000),1600000000,10.6);
  assert(!health.usable(1600000000,10.6));  // changed stamp with same sequence
  auto bad=record("b",3,2,1600000000);bad["health_sequence"]="3junk";
  health.observe(bad,1600000000,10.6);
  assert(!health.usable(1600000000,10.6));
  health.observe(record("b",4,2,1700000000),1700000000,10.7);
  assert(health.usable(1700000000,10.7));
  assert(!health.usable(2800000000,11.8));  // anchor cannot be timestamp-renewed
  std::cout<<"localization protocol lifecycle passed\n";
}
