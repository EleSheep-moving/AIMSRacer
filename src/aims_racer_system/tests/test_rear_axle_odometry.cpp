#include "aims_racer_system/rear_axle_odometry.hpp"

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>

namespace ars = aims_racer_system;
namespace
{
nav_msgs::msg::Odometry input()
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "odom";
  msg.header.stamp.sec = 17;
  msg.header.stamp.nanosec = 123456789;
  msg.child_frame_id = "livox_frame";
  msg.pose.pose.orientation.w = 1.;
  msg.pose.pose.position.x = 4.;
  msg.twist.twist.linear.x = 2.;
  return msg;
}
using Matrix6 = Eigen::Matrix<double, 6, 6>;
Matrix6 covariance(const std::array<double, 36> & values)
{
  return Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(values.data());
}
}

TEST(RearAxleOdometry, RigidLeverCompensationAndSourceStamp)
{
  ars::OdometryConfig config;
  config.translation = Eigen::Vector3d(1., 0., 0.);
  auto msg = input();
  const auto out = ars::convert_odometry(msg, Eigen::Vector3d(0., 0., 2.), config);
  EXPECT_EQ(out.header, msg.header);
  EXPECT_EQ(out.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(out.pose.pose.position.x, 3.);
  EXPECT_DOUBLE_EQ(out.twist.twist.linear.x, 2.);
  EXPECT_DOUBLE_EQ(out.twist.twist.linear.y, -2.);
  EXPECT_DOUBLE_EQ(out.twist.twist.angular.z, 2.);
  EXPECT_DOUBLE_EQ(out.pose.covariance[7], .02);
  EXPECT_DOUBLE_EQ(out.pose.covariance[11], -.01);
  EXPECT_DOUBLE_EQ(out.twist.covariance[7], .05);
  EXPECT_DOUBLE_EQ(out.twist.covariance[11], -.01);
}

TEST(RearAxleOdometry, RotatesAllThreeAxesAndTransformsCovariances)
{
  ars::OdometryConfig config;
  config.translation = Eigen::Vector3d(.4, -.3, .2);
  config.quaternion = Eigen::Quaterniond(Eigen::AngleAxisd(.7, Eigen::Vector3d(1., 2., 3.).normalized()));
  auto msg = input();
  const Eigen::Quaterniond q(Eigen::AngleAxisd(-.4, Eigen::Vector3d(2., 1., -1.).normalized()));
  msg.pose.pose.orientation.x = q.x(); msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z(); msg.pose.pose.orientation.w = q.w();
  msg.twist.twist.linear.y = -.7; msg.twist.twist.linear.z = .8;
  const Eigen::Vector3d omega(.2, -.5, 1.1);
  const auto out = ars::convert_odometry(msg, omega, config);
  const auto rotation = config.quaternion.toRotationMatrix();
  const Eigen::Matrix3d r_wb = q.toRotationMatrix() * rotation.transpose();
  const Eigen::Vector3d p = Eigen::Vector3d(4., 0., 0.) - r_wb * config.translation;
  const Eigen::Vector3d w = rotation * omega;
  const Eigen::Vector3d v = rotation * Eigen::Vector3d(2., -.7, .8) - w.cross(config.translation);
  EXPECT_NEAR(out.pose.pose.position.x, p.x(), 1e-12);
  EXPECT_NEAR(out.pose.pose.position.y, p.y(), 1e-12);
  EXPECT_NEAR(out.pose.pose.position.z, p.z(), 1e-12);
  EXPECT_NEAR(out.twist.twist.linear.x, v.x(), 1e-12);
  EXPECT_NEAR(out.twist.twist.linear.y, v.y(), 1e-12);
  EXPECT_NEAR(out.twist.twist.linear.z, v.z(), 1e-12);
  Eigen::Quaterniond result(out.pose.pose.orientation.w, out.pose.pose.orientation.x,
    out.pose.pose.orientation.y, out.pose.pose.orientation.z);
  EXPECT_TRUE(result.toRotationMatrix().isApprox(r_wb, 1e-12));
  for (const auto & values : {out.pose.covariance, out.twist.covariance}) {
    const auto matrix = covariance(values);
    EXPECT_TRUE(matrix.isApprox(matrix.transpose(), 1e-12));
    EXPECT_GT(Eigen::SelfAdjointEigenSolver<Matrix6>(matrix).eigenvalues().minCoeff(), 0.);
  }
}

TEST(RearAxleOdometry, NormalizesAcceptedQuaternionsAndRejectsBadStates)
{
  ars::OdometryConfig config;
  config.quaternion.coeffs() *= 1.005;
  auto msg = input(); msg.pose.pose.orientation.w = .995;
  EXPECT_NEAR(ars::convert_odometry(msg, Eigen::Vector3d::Zero(), config).pose.pose.orientation.w, 1., 1e-12);
  msg.pose.pose.orientation.w = .98;
  EXPECT_THROW(ars::convert_odometry(msg, Eigen::Vector3d::Zero(), config), std::invalid_argument);
  msg = input(); msg.child_frame_id = "base_link";
  EXPECT_THROW(ars::convert_odometry(msg, Eigen::Vector3d::Zero(), config), std::invalid_argument);
  msg = input(); msg.header.frame_id = "map";
  EXPECT_THROW(ars::convert_odometry(msg, Eigen::Vector3d::Zero(), config), std::invalid_argument);
  msg = input(); msg.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ars::convert_odometry(msg, Eigen::Vector3d::Zero(), config), std::invalid_argument);
}

TEST(RearAxleOdometry, RejectsNonfiniteAndIndefiniteCovariancesBeforeFloors)
{
  ars::OdometryConfig config;
  auto msg = input(); msg.pose.covariance[0] = -1e-4;
  EXPECT_THROW(ars::convert_odometry(msg, Eigen::Vector3d::Zero(), config), std::invalid_argument);
  msg = input(); msg.twist.covariance[1] = 1.; msg.twist.covariance[6] = 1.;
  EXPECT_THROW(ars::convert_odometry(msg, Eigen::Vector3d::Zero(), config), std::invalid_argument);
  msg = input(); msg.pose.covariance[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(ars::convert_odometry(msg, Eigen::Vector3d::Zero(), config), std::invalid_argument);
}

TEST(RearAxleOdometry, SourceTimeInterpolationNeverExtrapolatesOrBridgesGaps)
{
  ars::GyroHistory history{{1000000000LL, Eigen::Vector3d(0., 1., 2.)},
    {1030000000LL, Eigen::Vector3d(2., 3., 4.)}};
  EXPECT_FALSE(ars::source_time_angular_velocity(history, 999999999LL));
  EXPECT_FALSE(ars::source_time_angular_velocity(history, 1030000001LL));
  const auto middle = ars::source_time_angular_velocity(history, 1015000000LL);
  ASSERT_TRUE(middle); EXPECT_TRUE(middle->isApprox(Eigen::Vector3d(1., 2., 3.)));
  history.back().stamp_ns += 1;
  EXPECT_FALSE(ars::source_time_angular_velocity(history, 1015000000LL));
  EXPECT_TRUE(ars::source_time_angular_velocity(history, history.back().stamp_ns));
}

// Oracle generated with the retained Python/SciPy converter, nonzero input
// cross covariance, nonidentity mount/attitude, and diagonal floors active.
TEST(RearAxleOdometry, MatchesPythonFullCovarianceOracle)
{
  ars::OdometryConfig config;
  config.translation = Eigen::Vector3d(.4, -.3, .2);
  config.quaternion = Eigen::Quaterniond(Eigen::AngleAxisd(.7, Eigen::Vector3d(1., 2., 3.).normalized()));
  auto msg = input();
  const Eigen::Quaterniond q(Eigen::AngleAxisd(-.4, Eigen::Vector3d(2., 1., -1.).normalized()));
  msg.pose.pose.orientation.x = q.x(); msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z(); msg.pose.pose.orientation.w = q.w();
  msg.twist.twist.linear.y = -.7; msg.twist.twist.linear.z = .8;
  Matrix6 a;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {a(row, col) = (row * 6 + col + 1) * .001;}
  }
  const Matrix6 c = a * a.transpose() + Matrix6::Identity() * .003;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      msg.pose.covariance[row * 6 + col] = c(row, col);
      msg.twist.covariance[row * 6 + col] = 2. * c(row, col);
    }
  }
  const auto out = ars::convert_odometry(msg, Eigen::Vector3d(.2, -.5, 1.1), config);
  const std::array<double, 36> expected_pose{{
    0.013191998684756994, -0.001775567304885735, -0.0025514614612023416, -0.0022612919904850762, -0.0055018414075420534, -0.0042919786363054893,
    -0.001775567304885735, 0.012967918360596322, 0.0032001294312558573, 0.0057256083506925931, 0.0028999024121793334, 0.0032971385969111072,
    -0.0025514614612023416, 0.0032001294312558565, 0.012040232956068478, 0.0046264540526922493, 0.0041234608649403232, 0.0045483895783057435,
    -0.0022612919904850762, 0.0057256083506925931, 0.0046264540526922493, 0.01, 0.003565, 0.004339,
    -0.0055018414075420534, 0.0028999024121793334, 0.0041234608649403232, 0.003565, 0.01, 0.0055450000000000004,
    -0.0042919786363054893, 0.0032971385969111072, 0.0045483895783057435, 0.004339, 0.0055450000000000004, 0.01
  }};
  const std::array<double, 36> expected_twist{{
    0.041613536925589306, 0.0012748453481951601, -0.0060938299895339004, -0.0023593472029637217, -0.0057525901245644127, -0.0062923094833958375,
    0.0012748453481951549, 0.040403267918511232, 0.00011492002248390599, 0.0010905707788731024, -0.00021218916209022703, -0.0026244675099724984,
    -0.006093829989533903, 0.00011492002248390592, 0.052684572466461785, 0.0086153292447787386, 0.015548838631920707, 0.012945536365053948,
    -0.0023593472029637217, 0.0010905707788731026, 0.0086153292447787386, 0.0093829655696177974, 0.0065268764995975179, 0.006427552163194592,
    -0.0057525901245644119, -0.00021218916209022576, 0.015548838631920707, 0.0065268764995975205, 0.018592562244541911, 0.012401309733507861,
    -0.0062923094833958375, -0.0026244675099724992, 0.012945536365053948, 0.0064275521631945928, 0.012401309733507861, 0.018218472185840287
  }};
  for (std::size_t index = 0; index < 36; ++index) {
    EXPECT_NEAR(out.pose.covariance[index], expected_pose[index], 1e-12) << index;
    EXPECT_NEAR(out.twist.covariance[index], expected_twist[index], 1e-12) << index;
  }
}
