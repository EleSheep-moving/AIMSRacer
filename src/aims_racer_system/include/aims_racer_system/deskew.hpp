#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <optional>

namespace aims_racer_system {
struct TimedPose {
  int64_t stamp;
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
};
inline Eigen::Isometry3d pose_matrix(const TimedPose & pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.rotation.toRotationMatrix();
  result.translation() = pose.translation;
  return result;
}
inline Eigen::Vector3d deskew_point(const Eigen::Vector3d & point,
  const TimedPose & source, const TimedPose & target, const Eigen::Isometry3d & mount) {
  return (pose_matrix(target) * mount).inverse() * (pose_matrix(source) * mount) * point;
}
class PoseHistory {
public:
  bool add(TimedPose pose) {
    if (!pose.translation.allFinite() || !pose.rotation.coeffs().allFinite() ||
      std::abs(pose.rotation.norm() - 1.) > .01) return false;
    if (!poses_.empty() && pose.stamp < poses_.back().stamp) poses_.clear();
    if (!poses_.empty() && pose.stamp == poses_.back().stamp) return false;
    pose.rotation.normalize();
    poses_.push_back(pose);
    while (poses_.size() > 1 && pose.stamp - poses_.front().stamp > 2000000000LL) poses_.pop_front();
    return true;
  }
  std::optional<TimedPose> interpolate(int64_t stamp, int64_t max_gap=20000000LL) const {
    if (poses_.empty() || stamp < poses_.front().stamp || stamp > poses_.back().stamp) return {};
    const auto next = std::lower_bound(poses_.begin(), poses_.end(), stamp,
      [](const TimedPose & p, int64_t t) {return p.stamp < t;});
    if (next->stamp == stamp) return *next;
    const auto previous = std::prev(next);
    if (next->stamp - previous->stamp > max_gap) return {};
    const double fraction = double(stamp - previous->stamp) / (next->stamp - previous->stamp);
    return TimedPose{stamp, previous->translation + fraction * (next->translation - previous->translation),
      previous->rotation.slerp(fraction, next->rotation)};
  }
  bool covers(int64_t begin,int64_t end,int64_t max_gap=20000000LL) const {
    if (max_gap<=0 || end<begin || !interpolate(begin,max_gap) || !interpolate(end,max_gap)) return false;
    // Endpoint containment does not prove interior coverage. Check all segments
    // that overlap this scan, including the segments bracketing its endpoints.
    for (auto next=std::next(poses_.begin());next!=poses_.end();++next) {
      const auto previous=std::prev(next);
      if (previous->stamp>=end) break;
      if (next->stamp>begin && next->stamp-previous->stamp>max_gap) return false;
    }
    return true;
  }
  size_t size() const {return poses_.size();}
  void clear() {poses_.clear();}
  int64_t latest_stamp() const {return poses_.empty() ? -1 : poses_.back().stamp;}
private:
  std::deque<TimedPose> poses_;
};
}
