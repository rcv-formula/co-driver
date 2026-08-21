// The planned racing line, as a thing you can ask questions of.
//
// Judging "is this obstacle in my way" against the CURRENT STEERING ANGLE
// answers a different question than the one that matters. The commanded arc is
// where the controller is pointing this instant - it swings with every
// correction, it says nothing about the corner after this one, and when the
// controller is mid-correction it points somewhere the car will never go. What
// actually matters is whether the obstacle sits on the line the car is going to
// drive, which is the planned path.
//
// The path also carries the answer to "how fast will I be when I get there".
// The producer packs the planned speed into the z coordinate of each pose, so
// a path point is (x, y, speed). That is strictly better than the current
// command for deciding how far ahead to look: braking zones and straights have
// very different lookahead needs, and the instantaneous command does not know
// which one is coming.
//
// Two sources, in order of preference:
//   1. a nav_msgs/Path topic (transient_local, so it is there on subscribe)
//   2. a CSV of "x,y,speed" rows, for a vehicle where the planner is not
//      running but the line is known
//
// Everything here is in the path's own frame (map). Callers are responsible for
// bringing query points into it.
#ifndef CO_DRIVER__PATH_REFERENCE_HPP_
#define CO_DRIVER__PATH_REFERENCE_HPP_

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>

namespace co_driver
{

class PathReference
{
public:
  struct Point
  {
    double x{0.0};
    double y{0.0};
    double speed{0.0};   // planned speed at this point, from the pose's z
    double station{0.0}; // arc length from the first point
  };

  // Where a query point sits relative to the line.
  struct Projection
  {
    bool valid{false};
    double station{0.0};   // arc length of the closest point on the line
    double lateral{0.0};   // perpendicular distance to the line
  };

  bool empty() const {return points_.size() < 2;}
  std::size_t size() const {return points_.size();}
  // On a lap the closing segment is part of the total, or every wrap in
  // forwardDistance comes out one segment short.
  double length() const
  {
    return points_.empty() ? 0.0 : points_.back().station + total_closing_;
  }
  bool closed() const {return closed_;}
  const std::string & frame() const {return frame_;}
  const std::string & source() const {return source_;}

  void fromMessage(const nav_msgs::msg::Path & msg)
  {
    std::vector<Point> pts;
    pts.reserve(msg.poses.size());
    for (const auto & ps : msg.poses) {
      const auto & q = ps.pose.position;
      if (!std::isfinite(q.x) || !std::isfinite(q.y)) {continue;}
      pts.push_back({q.x, q.y, std::isfinite(q.z) ? q.z : 0.0, 0.0});
    }
    frame_ = msg.header.frame_id;
    source_ = "topic";
    adopt(std::move(pts));
  }

  // "x,y,speed" per line. Blank lines and anything starting with # are skipped,
  // and a non-numeric first line is treated as a header rather than an error -
  // these files are exported by hand often enough that failing on a header
  // would just be a trap.
  bool fromCsv(const std::string & path, const std::string & frame, std::string * error)
  {
    std::ifstream in(path);
    if (!in) {
      *error = "cannot open path csv: " + path;
      return false;
    }
    std::vector<Point> pts;
    std::string line;
    std::size_t line_no = 0;
    while (std::getline(in, line)) {
      ++line_no;
      if (line.empty() || line[0] == '#') {continue;}
      for (auto & ch : line) {
        if (ch == ',' || ch == ';' || ch == '\t') {ch = ' ';}
      }
      std::istringstream ss(line);
      double x, y, v;
      if (!(ss >> x >> y >> v)) {
        if (line_no == 1) {continue;}       // header row
        *error = "path csv " + path + ": line " + std::to_string(line_no) +
          " is not 'x y speed'";
        return false;
      }
      if (!std::isfinite(x) || !std::isfinite(y)) {continue;}
      pts.push_back({x, y, std::isfinite(v) ? v : 0.0, 0.0});
    }
    if (pts.size() < 2) {
      *error = "path csv " + path + " has fewer than 2 usable points";
      return false;
    }
    frame_ = frame;
    source_ = "csv:" + path;
    adopt(std::move(pts));
    return true;
  }

  // Closest point on the polyline to (px, py), as arc length plus perpendicular
  // distance. Segments are tested rather than vertices, so the answer does not
  // depend on how finely the path happens to be sampled.
  // With a window: only the stretch of line from `from` forward by `span` is
  // considered (plus a metre behind, so something just passed still reads as
  // passed rather than jumping to the far side of the lap).
  //
  // This matters because a lap comes back on itself. Taking the closest point
  // on the WHOLE line lets a thing beside the car match the leg the car will
  // drive later, and that leg's station can land just ahead of the car's own -
  // so something abeam is reported as being right in front. Measured on the
  // 0821-1 run, 37 of 495 on-path judgements came from the far leg, at a
  // median of 9.45 m away.
  //
  // Restricting the search is the honest fix, and it follows the line: the
  // region considered bends with the course instead of being a fixed wedge
  // pinned to the car's current heading.
  Projection project(double px, double py, double from, double span) const
  {
    return projectImpl(px, py, true, from, span);
  }

  Projection project(double px, double py) const
  {
    return projectImpl(px, py, false, 0.0, 0.0);
  }

private:
  Projection projectImpl(
    double px, double py, bool windowed, double from, double span) const
  {
    Projection r;
    if (empty()) {return r;}
    double best = std::numeric_limits<double>::infinity();
    const std::size_t n = points_.size();
    const std::size_t last = closed_ ? n : n - 1;
    for (std::size_t i = 0; i < last; ++i) {
      if (windowed) {
        const double d = forwardDistance(from, points_[i].station);
        if (d < -1.0 || d > span) {continue;}
      }
      const Point & a = points_[i];
      const Point & b = points_[(i + 1) % n];
      const double dx = b.x - a.x, dy = b.y - a.y;
      const double len2 = dx * dx + dy * dy;
      double t = 0.0;
      if (len2 > 1e-12) {
        t = std::clamp(((px - a.x) * dx + (py - a.y) * dy) / len2, 0.0, 1.0);
      }
      const double cx = a.x + t * dx, cy = a.y + t * dy;
      const double d = std::hypot(px - cx, py - cy);
      if (d < best) {
        best = d;
        r.station = a.station + t * std::sqrt(len2);
        r.lateral = d;
        r.valid = true;
      }
    }
    return r;
  }

public:
  // Arc length from `from` forward to `to`. On a closed path the line wraps, so
  // "behind me" and "most of a lap ahead" are the same place; anything further
  // back than behind_tolerance is reported as negative (passed) rather than as
  // a nearly-full lap ahead, which is what makes "has it gone by yet" answerable.
  double forwardDistance(double from, double to, double behind_tolerance = 5.0) const
  {
    double d = to - from;
    if (!closed_) {return d;}
    const double total = length();
    if (total <= 1e-6) {return d;}
    while (d < -total / 2.0) {d += total;}
    while (d > total / 2.0) {d -= total;}
    if (d < -behind_tolerance) {d += total;}
    return d;
  }

  // The planned speed the car will actually be carrying over the stretch
  // between two stations. Used instead of the current command because that is
  // what decides how much room the manoeuvre needs.
  double speedBetween(double from, double to) const
  {
    if (empty()) {return 0.0;}
    const double span = forwardDistance(from, to);
    if (span <= 1e-3) {return speedAt(from);}
    double sum = 0.0;
    int n = 0;
    const int steps = std::clamp(static_cast<int>(span / 0.5), 1, 200);
    for (int k = 0; k <= steps; ++k) {
      sum += speedAt(from + span * static_cast<double>(k) / steps);
      ++n;
    }
    return sum / std::max(1, n);
  }

  double speedAt(double station) const
  {
    if (empty()) {return 0.0;}
    const double total = length();
    double s = station;
    if (closed_ && total > 1e-6) {
      s = std::fmod(s, total);
      if (s < 0.0) {s += total;}
    }
    s = std::clamp(s, 0.0, total);
    // points_ is sorted by station, so a binary search lands the segment.
    const auto it = std::lower_bound(
      points_.begin(), points_.end(), s,
      [](const Point & p, double v) {return p.station < v;});
    if (it == points_.begin()) {return points_.front().speed;}
    if (it == points_.end()) {return points_.back().speed;}
    const Point & b = *it;
    const Point & a = *(it - 1);
    const double span = b.station - a.station;
    const double t = span > 1e-9 ? (s - a.station) / span : 0.0;
    return a.speed + t * (b.speed - a.speed);
  }

private:
  void adopt(std::vector<Point> && pts)
  {
    points_ = std::move(pts);
    if (points_.size() < 2) {return;}
    // A path whose ends meet is a lap, and treating it as a line would make
    // every obstacle just past the start/finish look like it was a full lap away.
    closed_ = std::hypot(
      points_.front().x - points_.back().x,
      points_.front().y - points_.back().y) < 0.5;
    if (closed_ && points_.size() > 2) {
      const double gap = std::hypot(
        points_.front().x - points_.back().x, points_.front().y - points_.back().y);
      if (gap < 1e-6) {points_.pop_back();}    // duplicate closing point
    }
    points_[0].station = 0.0;
    for (std::size_t i = 1; i < points_.size(); ++i) {
      points_[i].station = points_[i - 1].station +
        std::hypot(points_[i].x - points_[i - 1].x, points_[i].y - points_[i - 1].y);
    }
    if (closed_) {
      // Total length includes the closing segment, or the wrap arithmetic in
      // forwardDistance is short by one segment.
      total_closing_ = std::hypot(
        points_.front().x - points_.back().x, points_.front().y - points_.back().y);
    }
  }

  std::vector<Point> points_;
  std::string frame_{"map"};
  std::string source_;
  bool closed_{false};
  double total_closing_{0.0};
};

}  // namespace co_driver

#endif  // CO_DRIVER__PATH_REFERENCE_HPP_
