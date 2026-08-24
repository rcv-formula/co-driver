#include <limits>

#include <gtest/gtest.h>

#include "co_driver/vesc_speed.hpp"

namespace co_driver
{
namespace
{

TEST(VescSpeed, AppliesRedDamviConversionAndSeparateScale)
{
  const VescSpeedCalibration calibration{};
  double speed = 0.0;

  ASSERT_TRUE(vescErpmToSpeed(3172.47, calibration, &speed));
  EXPECT_NEAR(speed, 2.6, 1.0e-9);

  ASSERT_TRUE(vescErpmToSpeed(-3172.47, calibration, &speed));
  EXPECT_NEAR(speed, -2.6, 1.0e-9);
}

TEST(VescSpeed, AppliesOffsetBeforeGain)
{
  VescSpeedCalibration calibration{};
  calibration.speed_to_erpm_offset = 100.0;
  double speed = 0.0;

  ASSERT_TRUE(vescErpmToSpeed(3272.47, calibration, &speed));
  EXPECT_NEAR(speed, 2.6, 1.0e-9);
}

TEST(VescSpeed, AppliesRedDamviDeadbandBeforeScale)
{
  const VescSpeedCalibration calibration{};
  double speed = -1.0;

  ASSERT_TRUE(vescErpmToSpeed(3172.47 * 0.049, calibration, &speed));
  EXPECT_DOUBLE_EQ(speed, 0.0);

  ASSERT_TRUE(vescErpmToSpeed(3172.47 * 0.051, calibration, &speed));
  EXPECT_NEAR(speed, 0.051 * 2.6, 1.0e-9);
}

TEST(VescSpeed, RejectsInvalidCalibrationOrSample)
{
  VescSpeedCalibration calibration{};
  double speed = 0.0;

  calibration.speed_to_erpm_gain = 0.0;
  EXPECT_FALSE(vescErpmToSpeed(1000.0, calibration, &speed));

  calibration = VescSpeedCalibration{};
  calibration.wheel_speed_scale = 0.0;
  EXPECT_FALSE(vescErpmToSpeed(1000.0, calibration, &speed));

  calibration = VescSpeedCalibration{};
  EXPECT_FALSE(
    vescErpmToSpeed(std::numeric_limits<double>::quiet_NaN(), calibration, &speed));
}

TEST(VescSpeed, UsesFreshMeasurementAndFallsBackToAbsoluteCommand)
{
  EXPECT_DOUBLE_EQ(selectGeometrySpeed(true, 0.299, 0.3, 2.6, 7.0), 2.6);
  EXPECT_DOUBLE_EQ(selectGeometrySpeed(true, 0.3, 0.3, 2.6, 7.0), 2.6);
  EXPECT_DOUBLE_EQ(selectGeometrySpeed(true, 0.301, 0.3, 2.6, -7.0), 7.0);
  EXPECT_DOUBLE_EQ(selectGeometrySpeed(false, 0.0, 0.3, 2.6, -7.0), 7.0);
  EXPECT_DOUBLE_EQ(selectGeometrySpeed(true, 0.1, 0.3, 0.0, 7.0), 0.0);
}

}  // namespace
}  // namespace co_driver
