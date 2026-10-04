#include "aims_racer_system/deskew.hpp"
#include "aims_racer_system/rear_axle_imu.hpp"
#include <cassert>
#include <limits>
#include <Eigen/Eigenvalues>

int main()
{
  using namespace aims_racer_system;
  PoseHistory poses;
  assert(poses.add({1000000000LL, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()}));
  for (int knot=1;knot<=10;++knot) {
    const double fraction=knot/10.;
    assert(poses.add({1000000000LL+knot*10000000LL, {fraction,0.,0.},
      Eigen::Quaterniond(Eigen::AngleAxisd(fraction,Eigen::Vector3d::UnitZ()))}));
  }
  assert(!poses.interpolate(999999999LL));
  auto middle = poses.interpolate(1050000000LL);
  assert(middle && std::abs(middle->translation.x() - .5) < 1e-12);
  assert(std::abs(Eigen::AngleAxisd(middle->rotation).angle() - .5) < 1e-12);
  Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
  mount.translation() = Eigen::Vector3d(0.3, 0., .03);
  const Eigen::Vector3d point(4., 1., 0.);
  auto at_start = poses.interpolate(1000000000LL);
  auto at_end = poses.interpolate(1100000000LL);
  auto output = deskew_point(point, *at_start, *at_end, mount);
  auto expected = (pose_matrix(*at_end) * mount).inverse() * (pose_matrix(*at_start) * mount) * point;
  assert((output - expected).norm() < 1e-12);
  assert((deskew_point(point, *at_end, *at_end, mount) - point).norm() < 1e-12);
  assert(!poses.interpolate(1100000001LL));
  assert(poses.add({500000000LL, {0.,0.,0.}, Eigen::Quaterniond::Identity()}));
  assert(poses.size() == 1);  // Rewind removes the old epoch.
  PoseHistory gapped;
  assert(gapped.add({1000000000LL, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()}));
  assert(gapped.add({1050000000LL, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()}));
  assert(gapped.add({1060000000LL, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()}));
  assert(!gapped.covers(1000000000LL,1060000000LL,20000000LL));
  assert(!gapped.interpolate(1020000000LL));
  assert(gapped.covers(1050000000LL,1060000000LL,20000000LL));
  ImuConfig config;
  config.gyro_only = true;
  config.mounting = Eigen::Quaterniond(Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitX()));
  RearAxleImu imu(config);
  ImuInput input{1., {0.,0.,1.}, {NAN,NAN,NAN}, Matrix3::Zero(), Matrix3::Constant(NAN)};
  auto gyro = imu.process(input);
  assert(gyro && !gyro->compensated);
  assert((gyro->omega - config.mounting * input.omega).norm() < 1e-12);
  const Vector3 rear_force(.4,.7,9.80665), omega(.1,.2,.8), alpha(.3,-.2,.4), lever(.3,0.,.03);
  const Vector3 measured = rear_force + alpha.cross(lever) + omega.cross(omega.cross(lever));
  const auto force = compensate_force(measured,omega,alpha,lever,
    Matrix3::Identity()*.01,Matrix3::Identity()*.01,Matrix3::Identity()*.01);
  assert((force.force-rear_force).norm()<1e-12);
  Eigen::SelfAdjointEigenSolver<Matrix3> eigenvalues(force.covariance);
  assert(eigenvalues.eigenvalues().minCoeff()>0.);
}
