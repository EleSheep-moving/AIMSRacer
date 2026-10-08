#include "aims_racer_system/localization_quality.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <openssl/evp.h>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace aims_racer_system
{
Pose checked_pose(double x, double y, double z, double qx, double qy, double qz, double qw)
{
  Eigen::Quaterniond q(qw, qx, qy, qz);
  if (!Eigen::Vector3d(x, y, z).allFinite() || !q.coeffs().allFinite() ||
    std::abs(q.norm() - 1.) > .01)
  {
    throw std::invalid_argument("finite pose and unit quaternion required");
  }
  Pose result = Pose::Identity();
  result.linear() = q.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(x, y, z);
  return result;
}

std::optional<Pose> interpolate_pose(const PoseHistory & history, std::int64_t stamp)
{
  if (history.empty() || stamp < history.front().first || stamp > history.back().first) {
    return std::nullopt;
  }
  const auto after = std::lower_bound(history.begin(), history.end(), stamp,
    [](const auto & item, std::int64_t value) {return item.first < value;});
  if (after->first == stamp) {return after->second;}
  const auto before = std::prev(after);
  const double fraction = static_cast<double>(stamp - before->first) / (after->first - before->first);
  Pose result = Pose::Identity();
  result.translation() = before->second.translation() +
    fraction * (after->second.translation() - before->second.translation());
  result.linear() = Eigen::Quaterniond(before->second.linear()).slerp(
    fraction, Eigen::Quaterniond(after->second.linear())).toRotationMatrix();
  return result;
}

void MapOdomPackets::add(std::int64_t stamp, const Pose & pose)
{
  samples_[stamp] = pose;
  const auto cutoff = samples_.rbegin()->first - 5000000000LL;
  samples_.erase(samples_.begin(), samples_.lower_bound(cutoff));
}

std::optional<std::pair<std::int64_t, Pose>> MapOdomPackets::held(std::int64_t stamp) const
{
  auto item = samples_.upper_bound(stamp);
  if (item == samples_.begin()) {return std::nullopt;}
  --item;
  return *item;
}

struct MapQuality::Impl
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr points{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
};

MapQuality::MapQuality(const std::string & path) : impl_(std::make_unique<Impl>())
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {throw std::runtime_error("Cannot open immutable PCD map: " + path);}
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> digest(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!digest || EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("SHA256 initialization failed");
  }
  std::array<char, 65536> buffer{};
  while (file) {
    file.read(buffer.data(), buffer.size());
    if (EVP_DigestUpdate(digest.get(), buffer.data(), static_cast<std::size_t>(file.gcount())) != 1) {
      throw std::runtime_error("SHA256 update failed");
    }
  }
  if (!file.eof()) {throw std::runtime_error("PCD map read failed");}
  std::array<unsigned char, EVP_MAX_MD_SIZE> hash{};
  unsigned int length{};
  if (EVP_DigestFinal_ex(digest.get(), hash.data(), &length) != 1) {
    throw std::runtime_error("SHA256 finalization failed");
  }
  std::ostringstream hex;
  for (unsigned int i = 0; i < length; ++i) {
    hex << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(hash[i]);
  }
  sha256_ = hex.str();
  pcl::PointCloud<pcl::PointXYZ> raw;
  if (pcl::io::loadPCDFile(path, raw) < 0) {throw std::runtime_error("Invalid PCD map");}
  for (const auto & p : raw) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {impl_->points->push_back(p);}
  }
  if (impl_->points->size() < 100) {
    throw std::runtime_error("Map must contain at least 100 finite points");
  }
  impl_->tree.setInputCloud(impl_->points);
}

MapQuality::~MapQuality() = default;

static const sensor_msgs::msg::PointField & field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & value : cloud.fields) {
    if (value.name == name && value.count == 1 &&
      (value.datatype == sensor_msgs::msg::PointField::FLOAT32 ||
      value.datatype == sensor_msgs::msg::PointField::FLOAT64))
    {
      const std::size_t bytes = value.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
      if (value.offset <= cloud.point_step && bytes <= cloud.point_step - value.offset) {return value;}
    }
  }
  throw std::runtime_error("PointCloud2 requires scalar FLOAT32/FLOAT64 " + name);
}

static double scalar(const std::uint8_t * point, const sensor_msgs::msg::PointField & f, bool reverse)
{
  const std::size_t bytes = f.datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
  std::array<std::uint8_t, 8> data{};
  std::memcpy(data.data(), point + f.offset, bytes);
  if (reverse) {std::reverse(data.begin(), data.begin() + bytes);}
  if (bytes == 4) {float result; std::memcpy(&result, data.data(), 4); return result;}
  double result; std::memcpy(&result, data.data(), 8); return result;
}

QualityResult MapQuality::check(const sensor_msgs::msg::PointCloud2 & cloud,
  const Pose & transform, std::int64_t correction_stamp, std::uint64_t generation) const
{
  const auto start = std::chrono::steady_clock::now();
  const auto & fx = field(cloud, "x");
  const auto & fy = field(cloud, "y");
  const auto & fz = field(cloud, "z");
  if (cloud.point_step == 0 || cloud.row_step < static_cast<std::uint64_t>(cloud.width) * cloud.point_step ||
    static_cast<std::uint64_t>(cloud.row_step) * cloud.height > cloud.data.size())
  {
    throw std::runtime_error("Invalid PointCloud2 layout");
  }
  const std::uint16_t endian_test = 1;
  const bool little = *reinterpret_cast<const std::uint8_t *>(&endian_test) == 1;
  const bool reverse = cloud.is_bigendian == little;
  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t column = 0; column < cloud.width; ++column) {
      const auto * p = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(column) * cloud.point_step;
      Eigen::Vector3d value(scalar(p, fx, reverse), scalar(p, fy, reverse), scalar(p, fz, reverse));
      if (value.allFinite()) {points.push_back(value);}
    }
  }
  if (points.empty()) {throw std::runtime_error("no finite scan points");}
  const std::size_t stride = std::max<std::size_t>(1, (points.size() + 1999) / 2000);
  QualityResult result;
  std::size_t inliers{};
  double squared_sum{};
  std::vector<int> indices(1);
  std::vector<float> squared(1);
  for (std::size_t i = 0; i < points.size(); i += stride) {
    const Eigen::Vector3d world = transform * points[i];
    pcl::PointXYZ p(static_cast<float>(world.x()), static_cast<float>(world.y()), static_cast<float>(world.z()));
    if (!world.allFinite() || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      throw std::runtime_error("nonfinite transformed scan point");
    }
    ++result.points;
    if (impl_->tree.nearestKSearch(p, 1, indices, squared) > 0 && squared[0] <= .25f * .25f) {
      ++inliers;
      squared_sum += squared[0];
    }
  }
  result.inlier_fraction = static_cast<double>(inliers) / result.points;
  result.rmse = inliers ? std::sqrt(squared_sum / inliers) : std::numeric_limits<double>::quiet_NaN();
  result.time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  result.stamp = static_cast<std::int64_t>(cloud.header.stamp.sec) * 1000000000LL + cloud.header.stamp.nanosec;
  result.correction_stamp = correction_stamp;
  result.generation = generation;
  return result;
}
}  // namespace aims_racer_system
