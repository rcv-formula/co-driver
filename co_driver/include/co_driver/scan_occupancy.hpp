// What is on the planned line ahead, taken straight from the lidar.
//
// The whole rule, in one sentence:
//
//   A place on the line is blocked when, over the last confirm_window_s, at
//   least confirm_fraction of the scans saw something at least min_width_m
//   wide there - or when any single scan saw something within instant_range_m,
//   where there is no time to wait for a second opinion.
//
// Every number in that sentence is a physical quantity, and each answers a
// question that can be asked without knowing how the code works:
//
//   corridor_m         how far off the line still counts as on it
//   min_width_m        the smallest thing worth going around
//   confirm_window_s   how long the evidence is gathered over
//   confirm_fraction   how much of that window has to agree
//   same_place_m       how far apart two sightings can be and still be one thing
//   instant_range_m    how close is too close to wait
//
// beam_stride is the one exception and it is deliberately inert: using fewer
// beams costs resolution, not sensitivity, because the width test is expressed
// in metres and converted to a number of returns using the angular step that
// the stride actually produces. Halving the beams does not halve the evidence.
//
// WHY WIDTH RATHER THAN A COUNT OF RETURNS. A count means different things at
// different distances - eight returns is a 10 cm object at 1 m and an 80 cm
// object at 8 m. Consecutive returns are range x angular step apart, so a
// width converts to a count that is right at every distance.
//
// WHY ONE VOTE PER SCAN. Pooling the returns of several scans and counting them
// together lets a handful of strays in a single scan clear any threshold on its
// own. Each scan gets one vote per place instead. Measured over the 0821-1 run
// against the four objects added to its second half, per decision:
//
//                                     false alarms   found    hit within 1 m
//     the detector's clusters               6.9%       -         91% (3 m)
//     scan, returns pooled                  0.9%     26.3%      93.5%
//     scan, one vote per scan               0.1%     23.6%      99.5%
//
// WHY A CLOSE-RANGE EXCEPTION. A moving object cannot be in the same place scan
// after scan, so the vote alone erases exactly the case that matters most.
//
// DESKEW belongs to the caller: a UST-10LX sweeps 1081 beams over 25 ms, which
// is 12 cm of travel at 5 m/s, so each beam has to be placed at the pose for
// its own instant or the accumulated scans stop agreeing with each other - and
// agreement is the whole test.
#ifndef CO_DRIVER__SCAN_OCCUPANCY_HPP_
#define CO_DRIVER__SCAN_OCCUPANCY_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace co_driver
{

class ScanOccupancy
{
public:
  struct Settings
  {
    double corridor_m{0.28};        // half-width of the band around the line
    double min_width_m{0.08};       // smallest thing worth going around
    double confirm_window_s{0.15};  // how long the evidence is gathered over
    double confirm_fraction{0.6};   // how much of that window must agree
    double same_place_m{0.30};      // two sightings this close are one thing
    double instant_range_m{2.0};    // this near, one sighting is enough
    int beam_stride{3};             // cost only - does not change the verdict
  };

  // One place on the line the scans agree is blocked.
  struct Occupied
  {
    double along{0.0};        // metres ahead along the line
    double range{0.0};        // how far the nearest supporting return was
    double seen_fraction{0.0};
    bool instant{false};      // admitted on proximity rather than agreement
  };

  // One return, already placed in the path's frame and measured against the line.
  struct Point
  {
    double along{0.0};
    double lateral{0.0};
    double range{0.0};
  };

  void configure(const Settings & s) {s_ = s;}
  const Settings & settings() const {return s_;}
  void clear() {ring_.clear();}
  bool empty() const {return ring_.empty();}
  std::size_t scans() const {return ring_.size();}

  // `angular_step` is the angle between the returns actually kept, so the
  // width test does not change when beam_stride does.
  void push(double stamp, double angular_step, std::vector<Point> && pts)
  {
    ring_.push_back({stamp, angular_step, std::move(pts)});
    while (ring_.size() > 1 &&
      ring_.back().stamp - ring_.front().stamp > s_.confirm_window_s)
    {
      ring_.pop_front();
    }
  }

  std::vector<Occupied> occupied() const
  {
    // Each scan reports the places IT thinks are blocked: runs of neighbouring
    // returns inside the corridor that are together at least min_width_m wide.
    struct Sighting
    {
      double along;
      double range;
    };
    std::vector<std::vector<Sighting>> per_scan;
    per_scan.reserve(ring_.size());
    for (const auto & scan : ring_) {
      std::vector<Point> in;
      for (const auto & p : scan.pts) {
        if (p.lateral <= s_.corridor_m && p.along > 0.05) {in.push_back(p);}
      }
      std::sort(
        in.begin(), in.end(),
        [](const Point & a, const Point & b) {return a.along < b.along;});
      std::vector<Sighting> found;
      std::size_t i = 0;
      while (i < in.size()) {
        std::size_t j = i + 1;
        while (j < in.size() && in[j].along - in[j - 1].along <= s_.same_place_m) {++j;}
        // How many of these returns a min_width_m object would produce at this
        // distance. Consecutive returns are range x angular_step apart.
        double nearest = in[i].range;
        for (std::size_t k = i; k < j; ++k) {nearest = std::min(nearest, in[k].range);}
        const double spacing = std::max(1e-4, nearest * scan.angular_step);
        const int need = std::max(1, static_cast<int>(std::round(s_.min_width_m / spacing)));
        if (static_cast<int>(j - i) >= need) {
          found.push_back({in[i].along, nearest});
        }
        i = j;
      }
      per_scan.push_back(std::move(found));
    }
    if (per_scan.empty()) {return {};}

    // A place is blocked when enough of the scans in the window saw one there.
    // Sightings are grouped by how far apart they are, not by a grid, so two
    // returns either side of a cell edge are not split into different things.
    std::vector<Sighting> all;
    for (const auto & s : per_scan) {all.insert(all.end(), s.begin(), s.end());}
    std::sort(
      all.begin(), all.end(),
      [](const Sighting & a, const Sighting & b) {return a.along < b.along;});

    std::vector<Occupied> out;
    std::size_t i = 0;
    const double n_scans = static_cast<double>(per_scan.size());
    while (i < all.size()) {
      std::size_t j = i + 1;
      while (j < all.size() && all[j].along - all[j - 1].along <= s_.same_place_m) {++j;}
      // How many DISTINCT scans contributed to this group.
      int voters = 0;
      for (const auto & s : per_scan) {
        for (const auto & x : s) {
          if (x.along >= all[i].along - s_.same_place_m &&
            x.along <= all[j - 1].along + s_.same_place_m)
          {
            ++voters;
            break;
          }
        }
      }
      double nearest = all[i].range;
      for (std::size_t k = i; k < j; ++k) {nearest = std::min(nearest, all[k].range);}
      const double frac = voters / std::max(1.0, n_scans);
      const bool close = s_.instant_range_m > 0.0 && nearest <= s_.instant_range_m;
      if (frac >= s_.confirm_fraction || close) {
        Occupied o;
        o.along = all[i].along;
        o.range = nearest;
        o.seen_fraction = frac;
        o.instant = close && frac < s_.confirm_fraction;
        out.push_back(o);
      }
      i = j;
    }
    return out;
  }

private:
  struct Scan
  {
    double stamp{0.0};
    double angular_step{0.0};
    std::vector<Point> pts;
  };
  Settings s_;
  std::deque<Scan> ring_;
};

}  // namespace co_driver

#endif  // CO_DRIVER__SCAN_OCCUPANCY_HPP_
