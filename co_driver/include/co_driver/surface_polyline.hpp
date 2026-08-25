#ifndef CO_DRIVER__SURFACE_POLYLINE_HPP_
#define CO_DRIVER__SURFACE_POLYLINE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace co_driver
{

// Recursive orthogonal line fitting for one beam-ordered lidar surface.
//
// The adaptive breakpoint detector has already separated surfaces with a real
// range discontinuity.  This second stage is deliberately used only on a
// surface which the legacy size filter would discard: a long wall remains one
// long primitive, while a short protrusion attached to it creates fitting
// error and is split into its own primitive.  Smooth curves become a polyline.
struct SurfaceFitPoint
{
  double x{0.0};
  double y{0.0};
  double range{0.0};
};

struct SurfacePrimitive
{
  std::size_t begin{0};       // inclusive, in the input point array
  std::size_t end{0};         // exclusive
  double cx{0.0};             // centroid of the orthogonal line fit
  double cy{0.0};
  double dx{1.0};             // unit direction of the fitted line
  double dy{0.0};
  double length{0.0};         // measured along the raw surface
  double max_error{0.0};      // largest orthogonal residual
};

namespace detail
{

inline SurfacePrimitive fitSurfaceLine(
  const std::vector<SurfaceFitPoint> & points, std::size_t begin, std::size_t end)
{
  SurfacePrimitive out;
  out.begin = begin;
  out.end = end;
  const std::size_t n = end > begin ? end - begin : 0;
  if (n == 0) {return out;}

  for (std::size_t i = begin; i < end; ++i) {
    out.cx += points[i].x;
    out.cy += points[i].y;
    if (i > begin) {
      out.length += std::hypot(
        points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
  }
  out.cx /= static_cast<double>(n);
  out.cy /= static_cast<double>(n);

  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    const double x = points[i].x - out.cx;
    const double y = points[i].y - out.cy;
    xx += x * x;
    xy += x * y;
    yy += y * y;
  }
  if (xx + yy > 1e-12) {
    const double theta = 0.5 * std::atan2(2.0 * xy, xx - yy);
    out.dx = std::cos(theta);
    out.dy = std::sin(theta);
  } else if (n >= 2) {
    const double x = points[end - 1].x - points[begin].x;
    const double y = points[end - 1].y - points[begin].y;
    const double norm = std::hypot(x, y);
    if (norm > 1e-12) {
      out.dx = x / norm;
      out.dy = y / norm;
    }
  }

  // TLS has no direction: (dx,dy) and (-dx,-dy) describe the same line. Give
  // every primitive the direction in which the lidar beams traverse the
  // surface so adjacent-line angles retain their meaning.
  if (n >= 2) {
    const double chord_x = points[end - 1].x - points[begin].x;
    const double chord_y = points[end - 1].y - points[begin].y;
    if (out.dx * chord_x + out.dy * chord_y < 0.0) {
      out.dx = -out.dx;
      out.dy = -out.dy;
    }
  }

  for (std::size_t i = begin; i < end; ++i) {
    const double x = points[i].x - out.cx;
    const double y = points[i].y - out.cy;
    out.max_error = std::max(out.max_error, std::abs(-out.dy * x + out.dx * y));
  }
  return out;
}

inline void splitSurface(
  const std::vector<SurfaceFitPoint> & points, std::size_t begin, std::size_t end,
  double base_error, double error_per_m, std::size_t min_points,
  std::vector<SurfacePrimitive> * out)
{
  SurfacePrimitive fit = fitSurfaceLine(points, begin, end);
  const std::size_t n = end - begin;
  if (n < 2 * min_points) {
    out->push_back(fit);
    return;
  }

  double mean_range = 0.0;
  for (std::size_t i = begin; i < end; ++i) {mean_range += points[i].range;}
  mean_range /= static_cast<double>(n);
  const double tolerance = std::max(0.0, base_error) +
    std::max(0.0, error_per_m) * std::max(0.0, mean_range);

  // Split only where both resulting primitives retain enough beams to be
  // independently testable.  Choosing the largest eligible residual keeps a
  // single edge outlier from manufacturing a one-point obstacle.
  std::size_t split = begin + min_points;
  double split_error = -1.0;
  for (std::size_t i = begin + min_points; i <= end - min_points; ++i) {
    const double x = points[i].x - fit.cx;
    const double y = points[i].y - fit.cy;
    const double error = std::abs(-fit.dy * x + fit.dx * y);
    if (error > split_error) {
      split_error = error;
      split = i;
    }
  }
  if (split_error <= tolerance) {
    out->push_back(fit);
    return;
  }
  splitSurface(points, begin, split, base_error, error_per_m, min_points, out);
  splitSurface(points, split, end, base_error, error_per_m, min_points, out);
}

}  // namespace detail

inline std::vector<SurfacePrimitive> splitSurfacePolyline(
  const std::vector<SurfaceFitPoint> & points, double base_error,
  double error_per_m, int min_points)
{
  std::vector<SurfacePrimitive> out;
  if (points.empty()) {return out;}
  const std::size_t minimum = static_cast<std::size_t>(std::max(2, min_points));
  detail::splitSurface(
    points, 0, points.size(), base_error, error_per_m, minimum, &out);
  return out;
}

}  // namespace co_driver

#endif  // CO_DRIVER__SURFACE_POLYLINE_HPP_
