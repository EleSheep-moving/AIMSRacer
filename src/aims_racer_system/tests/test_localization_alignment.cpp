#include "aims_racer_system/localization_alignment.hpp"
#include <cassert>
#include <iostream>

using namespace aims_racer_system;
HealthFields commit(std::string epoch = "one", std::string sequence = "1", std::string stamp = "100")
{
  return {{"epoch", epoch}, {"anchor_sequence", sequence}, {"last_anchor_stamp_ns", stamp},
    {"anchor_committed", "true"}, {"map_odom_x", "1.25"}, {"map_odom_y", "-2"},
    {"map_odom_z", "3"}, {"map_odom_qx", "0"}, {"map_odom_qy", "0"},
    {"map_odom_qz", "0"}, {"map_odom_qw", "1"}};
}
int main()
{
  CommittedAlignment alignment;
  auto first = commit();
  assert(alignment.observe(first, true, "one", 100));
  auto published = alignment.fields(first);
  assert(published.at("alignment_valid") == "true");
  assert(published.at("alignment_epoch") == "one");
  assert(published.at("alignment_anchor_sequence") == "1");
  assert(published.at("alignment_stamp_ns") == "100");
  assert(published.at("map_odom_x") == "1.25");
  // Rejection does not relabel the held correction or refresh it.
  auto held = first;
  held["anchor_committed"] = "false";
  held.erase("map_odom_x");
  assert(alignment.observe(held, true, "one", 100));
  assert(alignment.fields(first).at("alignment_valid") == "true");
  // Source provenance mismatch cannot inherit a transform.
  auto wrong = first;
  wrong["anchor_sequence"] = "2";
  assert(alignment.fields(wrong).at("alignment_valid") == "false");
  wrong = first; wrong["epoch"] = "two";
  assert(alignment.fields(wrong).at("alignment_valid") == "false");
  wrong = first; wrong["last_anchor_stamp_ns"] = "101";
  assert(alignment.fields(wrong).at("alignment_valid") == "false");
  assert(alignment.observe(commit("two", "0", "0"), false, "two", 0));
  assert(alignment.fields(first).at("alignment_valid") == "false");
  // Reset clears every accepted component, even if the old health tuple remains.
  assert(alignment.observe(first, true, "one", 100));
  alignment.clear();
  assert(alignment.fields(first).at("alignment_valid") == "false");
  assert(alignment.fields(first).count("map_odom_x") == 0);
  // Replayed same sequence with a changed transform invalidates the stored record.
  assert(alignment.observe(first, true, "one", 100));
  wrong = first; wrong["map_odom_x"] = "9";
  assert(!alignment.observe(wrong, false, "one", 100));
  assert(alignment.fields(first).at("alignment_valid") == "false");
  for (const auto & malformed : {"nan", "inf", "1.0garbage", ""}) {
    wrong = first; wrong["map_odom_x"] = malformed;
    assert(!alignment.observe(wrong, true, "one", 100));
    assert(alignment.fields(first).at("alignment_valid") == "false");
  }
  wrong = first; wrong.erase("map_odom_qw");
  assert(!alignment.observe(wrong, true, "one", 100));
  wrong = first; wrong["map_odom_qw"] = "0";
  assert(!alignment.observe(wrong, true, "one", 100));
  wrong = first; wrong["map_odom_qw"] = "1.02";
  assert(!alignment.observe(wrong, true, "one", 100));
  wrong = first; wrong["last_anchor_stamp_ns"] = "101";
  assert(!alignment.observe(wrong, true, "one", 100));
  // An unaccepted first payload never establishes a record.
  assert(alignment.observe(first, false, "one", 100));
  assert(alignment.fields(first).at("alignment_valid") == "false");
  std::cout << "committed alignment contract passed\n";
}
