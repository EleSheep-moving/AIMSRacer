// The force contract follows the production C++ implementation.
#include "aims_racer_system/rear_axle_imu.hpp"

#include <Eigen/Eigenvalues>
#include <cassert>

#ifdef NDEBUG
#error "Rear axle force assertions must remain active in Release builds"
#endif

int main()
{
  using aims_racer_system::Matrix3;
  using aims_racer_system::Vector3;
  const Vector3 rear_force(0.4, 0.7, 9.80665);
  const Vector3 omega(0.1, 0.2, 0.8);
  const Vector3 alpha(0.3, -0.2, 0.4);
  const Vector3 lever(0.3, 0.0, 0.03);
  const Vector3 measured = rear_force + alpha.cross(lever) + omega.cross(omega.cross(lever));
  const Matrix3 sensor_covariance = Matrix3::Identity() * 0.01;
  const auto corrected = aims_racer_system::compensate_force(
    measured, omega, alpha, lever,
    sensor_covariance, sensor_covariance, sensor_covariance);

  // Both rotational lever terms cancel while the specific force retains gravity.
  assert((corrected.force - rear_force).cwiseAbs().maxCoeff() < 1e-12);
  assert(corrected.covariance.allFinite());
  assert(corrected.covariance.isApprox(corrected.covariance.transpose(), 1e-12));
  const Eigen::SelfAdjointEigenSolver<Matrix3> solver(
    corrected.covariance, Eigen::EigenvaluesOnly);
  assert(solver.info() == Eigen::Success);
  assert(solver.eigenvalues().minCoeff() > 0.0);
  return 0;
}
