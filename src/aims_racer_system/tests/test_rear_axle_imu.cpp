#include "aims_racer_system/rear_axle_imu.hpp"

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace aims_racer_system;

namespace
{
struct Values
{
  std::vector<double> values;
  std::size_t position = 0;
  double next() {return values.at(position++);}
  Vector3 vector()
  {
    Vector3 result;
    for (int i = 0; i < 3; ++i) {result[i] = next();}
    return result;
  }
  Matrix3 matrix()
  {
    Matrix3 result;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {result(row, column) = next();}
    }
    return result;
  }
  Quaternion quaternion()
  {
    const double x = next();
    const double y = next();
    const double z = next();
    const double w = next();
    return Quaternion(w, x, y, z);
  }
};
}  // namespace

TEST(RearAxleImu, MatchesOriginalPythonCallbacks)
{
  std::ifstream fixture(IMU_REFERENCE_PATH);
  ASSERT_TRUE(fixture.good());
  std::unique_ptr<RearAxleImu> processor;
  std::string line;
  unsigned events = 0;
  unsigned compensated = 0;
  unsigned unavailable = 0;
  unsigned dropped = 0;
  while (std::getline(fixture, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    SCOPED_TRACE("reference event " + std::to_string(++events));
    std::istringstream row(line);
    std::string cell;
    std::getline(row, cell, ',');
    const std::string kind = cell;
    Values values;
    while (std::getline(row, cell, ',')) {values.values.push_back(std::stod(cell));}
    if (kind == "C") {
      ImuConfig config;
      config.lever = values.vector();
      config.mounting = values.quaternion();
      config.accel_scale = values.next();
      config.max_attitude_age = values.next();
      config.angular_acceleration_window = values.next();
      config.max_gyro_gap = values.next();
      processor = std::make_unique<RearAxleImu>(config);
    } else if (kind == "O") {
      ASSERT_TRUE(processor);
      const double stamp = values.next();
      const Quaternion quaternion = values.quaternion();
      const Matrix3 covariance = values.matrix();
      processor->add_attitude(stamp, quaternion, covariance);
    } else {
      ASSERT_EQ(kind, "I");
      ASSERT_TRUE(processor);
      ImuInput input;
      input.stamp = values.next();
      input.omega = values.vector();
      input.specific_force = values.vector();
      input.omega_covariance = values.matrix();
      input.force_covariance = values.matrix();
      const bool emitted = values.next() != 0.0;
      const auto output = processor->process(input);
      ASSERT_EQ(output.has_value(), emitted);
      if (output) {
        EXPECT_LT((output->omega - values.vector()).cwiseAbs().maxCoeff(), 1e-8);
        EXPECT_LT((output->omega_covariance - values.matrix()).cwiseAbs().maxCoeff(), 1e-8);
        const bool valid = values.next() != 0.0;
        ASSERT_EQ(output->compensated, valid);
        const Quaternion expected = values.quaternion();
        EXPECT_LT((output->orientation.toRotationMatrix() - expected.toRotationMatrix()).norm(), 1e-8);
        Matrix3 expected_orientation = values.matrix();
        if (!valid) {
          expected_orientation(0, 0) = 0.0;  // ROS unavailable sentinel is set by the adapter.
        }
        EXPECT_LT((output->orientation_covariance - expected_orientation).cwiseAbs().maxCoeff(), 1e-8);
        EXPECT_LT((output->specific_force - values.vector()).cwiseAbs().maxCoeff(), 1e-8);
        Matrix3 expected_force = values.matrix();
        if (!valid) {expected_force(0, 0) = 0.0;}
        EXPECT_LT((output->force_covariance - expected_force).cwiseAbs().maxCoeff(), 1e-8);
        valid ? ++compensated : ++unavailable;
      } else {
        ++dropped;
      }
    }
    EXPECT_EQ(values.position, values.values.size());
  }
  EXPECT_EQ(events, 244u);
  EXPECT_GT(compensated, 100u);
  EXPECT_GT(unavailable, 20u);
  EXPECT_EQ(dropped, 6u);
}

TEST(RearAxleImu, ForceCompensationRemovesBothLeverTermsAndKeepsGravity)
{
  const Vector3 rear_force(0.4, 0.7, 9.80665);
  const Vector3 omega(0.1, 0.2, 0.8);
  const Vector3 alpha(0.3, -0.2, 0.4);
  const Vector3 lever(0.3, 0.0, 0.03);
  const Vector3 measured = rear_force + alpha.cross(lever) + omega.cross(omega.cross(lever));
  const Matrix3 covariance = Matrix3::Identity() * 0.01;
  const auto result = compensate_force(measured, omega, alpha, lever, covariance, covariance, covariance);
  EXPECT_LT((result.force - rear_force).norm(), 1e-12);
  EXPECT_GT(Eigen::SelfAdjointEigenSolver<Matrix3>(result.covariance).eigenvalues().minCoeff(), 0.0);
}

TEST(RearAxleImu, CovarianceValidationAndFloors)
{
  EXPECT_TRUE(validate_covariance(Matrix3::Zero(), 0.01).isApprox(Matrix3::Identity() * 0.01));
  Matrix3 invalid = Matrix3::Identity();
  invalid(0, 0) = -1.0;
  EXPECT_THROW(validate_covariance(invalid, 0.01), std::invalid_argument);
  invalid(0, 0) = -0.1;
  EXPECT_THROW(validate_covariance(invalid, 0.01), std::invalid_argument);
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate_covariance(invalid, 0.01), std::invalid_argument);
}

TEST(AngularHistory, DerivativeAndHistoryReset)
{
  AngularHistory history;
  const Vector3 alpha(0.3, -0.2, 0.4);
  for (int i = 0; i < 10; ++i) {
    const double stamp = i * 0.005;
    EXPECT_FALSE(history.add(stamp, Vector3::Ones() + alpha * stamp, Matrix3::Identity() * 0.01));
  }
  ASSERT_TRUE(history.derivative());
  EXPECT_LT((history.derivative()->alpha - alpha).norm(), 1e-12);
  EXPECT_TRUE(history.add(0.04, Vector3::Zero(), Matrix3::Identity()));
  EXPECT_FALSE(history.derivative());
  EXPECT_TRUE(history.add(0.1, Vector3::Zero(), Matrix3::Identity()));
  EXPECT_FALSE(history.orientation(0.04, Quaternion::Identity(), 0.1));
}

TEST(AngularHistory, IntegratesBetweenSamplesAndBoundsHistory)
{
  AngularHistory history;
  const Vector3 omega(0.0, 0.0, 0.4);
  for (int i = 0; i < 410; ++i) {
    history.add(i * 0.005, omega, Matrix3::Identity() * 0.01);
  }
  EXPECT_FALSE(history.orientation(0.0, Quaternion::Identity(), 0.1));
  const auto result = history.orientation(1.013, Quaternion::Identity(), 1.108);
  ASSERT_TRUE(result);
  const Quaternion expected(Eigen::AngleAxisd(0.4 * (1.108 - 1.013), Vector3::UnitZ()));
  EXPECT_LT(result->angularDistance(expected), 1e-12);
  EXPECT_FALSE(history.orientation(1.0, Quaternion::Identity(), 2.1));
  EXPECT_FALSE(history.orientation(1.1, Quaternion::Identity(), 1.0));
}

TEST(RearAxleImu, RejectsInvalidConfiguration)
{
  ImuConfig config;
  config.mounting = Quaternion(2.0, 0.0, 0.0, 0.0);
  EXPECT_THROW(RearAxleImu processor(config), std::invalid_argument);
  config = ImuConfig{};
  config.accel_scale = 0.0;
  EXPECT_THROW(RearAxleImu processor(config), std::invalid_argument);
  config = ImuConfig{};
  config.angular_acceleration_window = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(RearAxleImu processor(config), std::invalid_argument);
}
