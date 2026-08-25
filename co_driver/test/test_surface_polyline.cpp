#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "co_driver/surface_polyline.hpp"

namespace co_driver
{
namespace
{

TEST(SurfacePolyline, StraightWallRemainsOneLongPrimitive)
{
  std::vector<SurfaceFitPoint> points;
  for (int i = 0; i <= 20; ++i) {
    points.push_back({0.2 * i, 1.0, 3.0});
  }

  const auto primitives = splitSurfacePolyline(points, 0.05, 0.0, 2);
  ASSERT_EQ(primitives.size(), 1U);
  EXPECT_NEAR(primitives.front().length, 4.0, 1e-9);
  EXPECT_NEAR(primitives.front().max_error, 0.0, 1e-9);
}

TEST(SurfacePolyline, AttachedProtrusionSplitsAwayFromLongWall)
{
  // One continuous surface in beam order: wall, a rectangular protrusion
  // toward the path, then the same wall again. There is no range breakpoint
  // at either attachment, which is the case the legacy size filter loses.
  const std::vector<SurfaceFitPoint> points = {
    {-3.0, 1.0, 3.2}, {-2.5, 1.0, 3.0}, {-2.0, 1.0, 2.8}, {-1.5, 1.0, 2.5},
    {-1.0, 1.0, 2.2}, {-0.5, 1.0, 2.0}, {-0.4, 0.8, 1.9}, {-0.4, 0.6, 1.8},
    {-0.2, 0.5, 1.7}, {0.0, 0.5, 1.7}, {0.2, 0.5, 1.7}, {0.4, 0.6, 1.8},
    {0.4, 0.8, 1.9}, {0.5, 1.0, 2.0}, {1.0, 1.0, 2.2}, {1.5, 1.0, 2.5},
    {2.0, 1.0, 2.8}, {2.5, 1.0, 3.0}, {3.0, 1.0, 3.2}};

  const auto primitives = splitSurfacePolyline(points, 0.05, 0.0, 2);
  ASSERT_GT(primitives.size(), 1U);

  std::vector<int> covered(points.size(), 0);
  bool protrusion_is_short = false;
  bool wall_is_long = false;
  for (const auto & primitive : primitives) {
    for (std::size_t i = primitive.begin; i < primitive.end; ++i) {++covered[i];}
    if (primitive.begin <= 9 && 9 < primitive.end && primitive.length < 1.5) {
      protrusion_is_short = true;
    }
    if (primitive.length > 1.5) {wall_is_long = true;}
  }
  for (int count : covered) {EXPECT_EQ(count, 1);}
  EXPECT_TRUE(protrusion_is_short);
  EXPECT_TRUE(wall_is_long);
}

TEST(SurfacePolyline, SmoothArcBecomesGapFreePolyline)
{
  std::vector<SurfaceFitPoint> points;
  for (int i = 0; i <= 40; ++i) {
    const double a = -0.8 + 1.6 * static_cast<double>(i) / 40.0;
    points.push_back({3.0 * std::cos(a), 3.0 * std::sin(a), 3.0});
  }

  const auto primitives = splitSurfacePolyline(points, 0.03, 0.0, 2);
  ASSERT_GT(primitives.size(), 1U);
  std::size_t next = 0;
  for (const auto & primitive : primitives) {
    EXPECT_EQ(primitive.begin, next);
    EXPECT_LE(primitive.max_error, 0.03 + 1e-9);
    next = primitive.end;
  }
  EXPECT_EQ(next, points.size());
}

TEST(SurfacePolyline, DisplacedTerminalWallFragmentIsNotRejoined)
{
  // A barrier can physically move after an impact. Once its terminal face
  // bends toward the path it is collision evidence even though its semantic
  // identity is still "wall"; the splitter must leave that face available to
  // the raw-scan path test instead of folding it back into the long run.
  std::vector<SurfaceFitPoint> points;
  for (int i = 0; i <= 12; ++i) {
    points.push_back({-3.0 + 0.25 * i, 1.0, 3.0});
  }
  points.push_back({0.15, 0.85, 2.9});
  points.push_back({0.25, 0.60, 2.8});
  points.push_back({0.30, 0.30, 2.7});

  const auto primitives = splitSurfacePolyline(points, 0.03, 0.0, 2);
  ASSERT_GT(primitives.size(), 1U);
  EXPECT_GT(primitives.front().length, 1.5);
  EXPECT_LT(primitives.back().length, 1.5);
  EXPECT_EQ(primitives.back().end, points.size());

}

}  // namespace
}  // namespace co_driver
