#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "co_driver/scan_occupancy.hpp"

namespace co_driver
{
namespace
{

ScanOccupancy configuredOccupancy()
{
  ScanOccupancy occupancy;
  ScanOccupancy::Settings settings;
  settings.scans_kept = 1;
  settings.points_per_scan = 2;
  settings.dense_one_scan = 99;
  settings.one_scan_within_m = 10.0;
  settings.add_scan_every_m = 0.0;
  settings.add_scan_every_mps = 0.0;
  settings.car_width_until_m = 0.0;
  settings.wall_longer_than_m = 0.0;
  settings.biggest_thing_m = 1.5;
  occupancy.configure(settings);
  return occupancy;
}

std::vector<ScanOccupancy::Point> longSurface(bool preserve)
{
  std::vector<ScanOccupancy::Point> points;
  for (int i = 0; i < 2; ++i) {
    ScanOccupancy::Point point;
    point.along = 1.0 + 0.1 * i;
    point.lateral = 0.0;
    point.side = 0.0;
    point.range = 1.0;
    point.segment = 7;
    point.segment_size = 3.0;
    point.preserve_if_large = preserve;
    points.push_back(point);
  }
  return points;
}

std::vector<ScanOccupancy::Point> recoveredSurface(double lateral, bool recovered)
{
  std::vector<ScanOccupancy::Point> points;
  for (int i = 0; i < 2; ++i) {
    ScanOccupancy::Point point;
    point.along = 1.0 + 0.1 * i;
    point.lateral = lateral;
    point.side = lateral;
    point.range = 2.0;
    point.segment = 8;
    point.segment_size = 0.4;
    point.recovered_from_large = recovered;
    points.push_back(point);
  }
  return points;
}

TEST(ScanOccupancy, LegacySizeFilterRejectsLongSurface)
{
  auto occupancy = configuredOccupancy();
  occupancy.push(longSurface(false));
  EXPECT_TRUE(occupancy.occupied(0.0).empty());
}

TEST(ScanOccupancy, SweptGuardPreservesInPathReturnsFromLongSurface)
{
  auto occupancy = configuredOccupancy();
  occupancy.push(longSurface(true));
  const auto occupied = occupancy.occupied(0.0);
  ASSERT_EQ(occupied.size(), 1U);
  EXPECT_NEAR(occupied.front().along, 1.0, 1e-9);
}

TEST(ScanOccupancy, RecoveredMarginCutDoesNotNarrowOrdinaryObjects)
{
  auto occupancy = configuredOccupancy();
  auto settings = occupancy.settings();
  settings.near_line_m = 0.28;
  settings.recovered_margin_cut_m = 0.10;
  occupancy.configure(settings);
  occupancy.push(recoveredSurface(0.20, false));
  EXPECT_EQ(occupancy.occupied(0.0).size(), 1U);
}

TEST(ScanOccupancy, RecoveredMarginCutRejectsWallEdgeSplitOnly)
{
  auto occupancy = configuredOccupancy();
  auto settings = occupancy.settings();
  settings.near_line_m = 0.28;
  settings.recovered_margin_cut_m = 0.10;
  occupancy.configure(settings);
  occupancy.push(recoveredSurface(0.20, true));
  EXPECT_TRUE(occupancy.occupied(0.0).empty());
}

TEST(ScanOccupancy, RecoveredMarginCutKeepsCentralSplit)
{
  auto occupancy = configuredOccupancy();
  auto settings = occupancy.settings();
  settings.near_line_m = 0.28;
  settings.recovered_margin_cut_m = 0.10;
  occupancy.configure(settings);
  occupancy.push(recoveredSurface(0.10, true));
  EXPECT_EQ(occupancy.occupied(0.0).size(), 1U);
}

}  // namespace
}  // namespace co_driver
