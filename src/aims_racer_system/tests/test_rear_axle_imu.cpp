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

TEST(RearAxleImu, PreservesOriginalGyroAndValidForceValues)
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
      values.next();  // Retired legacy attitude-age parameter in the fixture.
      config.angular_acceleration_window = values.next();
      config.max_gyro_gap = values.next();
      processor = std::make_unique<RearAxleImu>(config);
    } else if (kind == "O") {
      ASSERT_TRUE(processor);
      const double stamp = values.next();
      const Quaternion quaternion = values.quaternion();
      const Matrix3 covariance = values.matrix();
      (void)stamp; (void)quaternion; (void)covariance;  // No LIO dependency in force conversion.
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
        // Original fixture includes the retired attitude gate. Compare actual force values
        // where that implementation had a usable measurement.
        const Quaternion expected = values.quaternion();
        (void)expected;
        values.matrix();  // Retired propagated-orientation covariance.
        const Vector3 expected_force_value = values.vector();
        if (valid) {
          ASSERT_TRUE(output->compensated);
          EXPECT_LT((output->specific_force - expected_force_value).cwiseAbs().maxCoeff(), 1e-8);
        }
        values.matrix();  // Legacy covariance included a gravity/attitude inflation.
        if (output->compensated) {
          EXPECT_GT(Eigen::SelfAdjointEigenSolver<Matrix3>(output->force_covariance).eigenvalues().minCoeff(), 0.0);
        }
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

TEST(RearAxleImu, CompensatesForceWithoutLioAttitude)
{
  ImuConfig config;
  config.lever = Vector3(0.3, 0.0, 0.03);
  RearAxleImu processor(config);
  const Vector3 omega(0., 0., 0.4);
  const Vector3 rear_force(0.5, 0.2, 9.80665);
  const Vector3 sensor_force = rear_force + omega.cross(omega.cross(config.lever));
  std::optional<ImuOutput> result;
  for (int i = 0; i < 10; ++i) {
    result = processor.process({10. + i * .005, omega, sensor_force / config.accel_scale,
      Matrix3::Identity() * .01, Matrix3::Zero()});
  }
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->compensated);
  EXPECT_LT((result->specific_force - rear_force).norm(), 1e-12);
}

TEST(RearAxleImu, OnlyGyroHistoryGapsInvalidateFiniteForce)
{
  RearAxleImu processor(ImuConfig{});
  const auto sample = [](double stamp) {
      return ImuInput{stamp, Vector3(0., 0., .4), Vector3(0., 0., 1.),
        Matrix3::Identity() * .01, Matrix3::Zero()};
    };
  EXPECT_FALSE(processor.process(sample(10.))->compensated);
  EXPECT_FALSE(processor.process(sample(10.005))->compensated);
  EXPECT_TRUE(processor.process(sample(10.010))->compensated);
  for (int i = 3; i < 120; ++i) {
    ASSERT_TRUE(processor.process(sample(10. + i * .005))->compensated);
  }
  // A genuine gap resets alpha estimation; yaw rate remains available immediately.
  const auto after_gap = processor.process(sample(11.));
  ASSERT_TRUE(after_gap);
  EXPECT_FALSE(after_gap->compensated);
  EXPECT_DOUBLE_EQ(after_gap->omega.z(), .4);
  EXPECT_FALSE(processor.process(sample(11.005))->compensated);
  EXPECT_TRUE(processor.process(sample(11.010))->compensated);
}

TEST(RearAxleImu, RotatesForceAndGyroUsingFixedMounting)
{
  ImuConfig config;
  config.mounting = Quaternion(Eigen::AngleAxisd(.3, Vector3::UnitX()));
  config.lever = Vector3(.3, 0., .03);
  RearAxleImu processor(config);
  const Vector3 omega(0., 0., .4);
  const Vector3 rear_force(.5, .2, 9.80665);
  const Vector3 sensor = config.mounting.conjugate() *
    (rear_force + omega.cross(omega.cross(config.lever))) / config.accel_scale;
  std::optional<ImuOutput> result;
  for (int i = 0; i < 10; ++i) {
    result = processor.process({10. + i * .005, config.mounting.conjugate() * omega,
      sensor, Matrix3::Identity() * .01, Matrix3::Zero()});
  }
  ASSERT_TRUE(result && result->compensated);
  EXPECT_LT((result->omega - omega).norm(), 1e-12);
  EXPECT_LT((result->specific_force - rear_force).norm(), 1e-12);
}
