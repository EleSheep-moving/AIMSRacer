#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace aims_racer_system
{
using Pose = Eigen::Isometry3d;
using PoseHistory = std::deque<std::pair<std::int64_t, Pose>>;
Pose checked_pose(double x, double y, double z, double qx, double qy, double qz, double qw);
std::optional<Pose> interpolate_pose(const PoseHistory & history, std::int64_t stamp);

class MapOdomPackets
{
public:
  void add(std::int64_t stamp, const Pose & pose);
  std::optional<std::pair<std::int64_t, Pose>> held(std::int64_t stamp) const;
  void clear() {samples_.clear();}
private:
  std::map<std::int64_t, Pose> samples_;
};

struct QualityResult
{
  std::size_t points{};
  double inlier_fraction{}, rmse{}, time_ms{};
  std::int64_t stamp{}, correction_stamp{};
  std::uint64_t generation{};
};

// Built once on startup, read only by the single quality worker.
class MapQuality
{
public:
  explicit MapQuality(const std::string & path);
  ~MapQuality();
  MapQuality(const MapQuality &) = delete;
  MapQuality & operator=(const MapQuality &) = delete;
  const std::string & sha256() const {return sha256_;}
  QualityResult check(const sensor_msgs::msg::PointCloud2 & cloud, const Pose & transform,
    std::int64_t correction_stamp, std::uint64_t generation) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::string sha256_;
};
}  // namespace aims_racer_system
