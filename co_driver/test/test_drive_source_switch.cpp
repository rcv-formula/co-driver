#include <gtest/gtest.h>

#include "co_driver/drive_source_switch.hpp"

namespace co_driver
{
namespace
{

TEST(DriveSourceSwitch, SelectsConfiguredEndpointsWithTolerance)
{
  EXPECT_EQ(chooseDriveSource(2000, 2000, 1000, 100), DriveSourceChoice::primary);
  EXPECT_EQ(chooseDriveSource(2100, 2000, 1000, 100), DriveSourceChoice::primary);
  EXPECT_EQ(chooseDriveSource(1900, 2000, 1000, 100), DriveSourceChoice::primary);
  EXPECT_EQ(chooseDriveSource(1000, 2000, 1000, 100), DriveSourceChoice::alternate);
  EXPECT_EQ(chooseDriveSource(900, 2000, 1000, 100), DriveSourceChoice::alternate);
  EXPECT_EQ(chooseDriveSource(1100, 2000, 1000, 100), DriveSourceChoice::alternate);
}

TEST(DriveSourceSwitch, KeepsCurrentSourceBetweenEndpoints)
{
  EXPECT_EQ(chooseDriveSource(1500, 2000, 1000, 100), DriveSourceChoice::keep);
  EXPECT_EQ(chooseDriveSource(0, 2000, 1000, 100), DriveSourceChoice::keep);
  EXPECT_EQ(chooseDriveSource(899, 2000, 1000, 100), DriveSourceChoice::keep);
  EXPECT_EQ(chooseDriveSource(1101, 2000, 1000, 100), DriveSourceChoice::keep);
  EXPECT_EQ(chooseDriveSource(1899, 2000, 1000, 100), DriveSourceChoice::keep);
  EXPECT_EQ(chooseDriveSource(2101, 2000, 1000, 100), DriveSourceChoice::keep);
}

TEST(DriveSourceSwitch, RefusesAmbiguousOverlappingEndpoints)
{
  EXPECT_EQ(chooseDriveSource(1500, 2000, 1000, 500), DriveSourceChoice::keep);
}

}  // namespace
}  // namespace co_driver
