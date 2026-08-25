// What is on the planned line ahead, taken straight from the lidar.
//
// THE RULE, which is the whole of it:
//
//   Keep the last scans_kept scans. In each one, a place counts as seen when at
//   least points_per_scan returns landed there, within near_line_m of the line.
//   A place is occupied when enough of the kept scans saw it - or when one scan
//   alone saw dense_one_scan returns there.
//
//   How many is "enough" is not a constant. Something right in front has to be
//   acted on now and there is no time to collect a second opinion; something
//   far away is not urgent, so the evidence for it can be made to earn its
//   place. Speed does the same thing from the other side: the faster the car,
//   the further ahead it is deciding, and the weaker the evidence is out there.
//
//     one_scan_within_m    inside this, a single scan is enough - the floor
//     add_scan_every_m     each further step of this asks for one more scan
//     add_scan_every_mps   each step of speed asks for one more scan
//
//   so   scans needed = 1 + (range - one_scan_within_m) / add_scan_every_m
//                         + speed / add_scan_every_mps,   capped at scans_kept.
//
//   car_width_until_m  close in, a return has to be in the way of the CAR, not
//                      just sitting on the line. The two are the same thing
//                      only while the car is ON the line. Measured on the
//                      0821-1 run, 614 of the 658 false holds were one spot on
//                      the lap where the car ran 0.43 m wide of the line: the
//                      wall alongside it was 0.5 m from the lidar and 0.24 m
//                      ahead - 0.44 m out to the side, nowhere near the car -
//                      and yet 0.05 m from the line, because the line was over
//                      there. Beyond this distance the car will have come back
//                      to the line, so the line is the right thing to measure
//                      against again.
//   near_line_m        how far off the line still counts as being on it right
//                      in front of the car. This is the width the CAR occupies
//                      on the line - a thing inside it has to be gone around, a
//                      thing outside it is passed - so the caller derives it
//                      from the vehicle rather than tuning it here.
//   narrow_per_m       how much that width CLOSES UP per metre of range, and
//                      never_narrower_than_m where it stops closing.
//
//                      Narrows, not widens. A return far away is placed less
//                      well than a near one - the pose it is placed with is
//                      older, the beams are further apart, the sweep has moved
//                      more - so at range the wall beside the line smears
//                      towards it. The wall does not actually touch the line.
//                      Demanding that a distant return be nearer the middle of
//                      the line before believing it is what tells the two
//                      apart, and the belief costs nothing where it matters,
//                      because by the time the car is close the full width is
//                      back.
//
//                      That is the theory. Measured on the 0821-1 run it does
//                      not hold, and it is off by default: the one place flagged
//                      in the obstacle-free half was a wall reading 1.3 cm from
//                      the line at 5.9 m - nearer the line than two of the four
//                      real objects ever read. It was not smeared inward a
//                      little, it was placed on the line, so no slope separates
//                      it, and a slope steep enough to cut it (0.035) collapses
//                      the warning distance on a real object from 6.8 m to
//                      2.6 m. wall_longer_than_m does that job for nothing.
//   wall_longer_than_m a thing that runs on beside the line for longer than
//                      this is structure, not an object. A cone, a barrel,
//                      another car is short; a wall goes on for metres. This is
//                      what separates them without a map - and it has to be
//                      without a map, because subtracting one also erases
//                      whatever is leaning against the wall.
//   scans_kept         how many scans are kept
//   points_per_scan    how many returns make one scan a witness
//   dense_one_scan     this many in one scan is enough on its own
//   same_place_m       how far apart two sightings in DIFFERENT scans can be
//                      and still be the same place. Within one scan that
//                      question is answered by the caller's segmentation, not
//                      by a distance along the line - see below.
//   biggest_thing_m    a surface bigger than this is not something to go
//                      around, it is what the track is made of
//   beam_stride        use every nth beam; fewer beams means fewer returns, so
//                      the counts above are counted on the beams actually used
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
// WHY THE VOTE IS NOT SIMPLY RAISED. A moving object cannot be in the same
// place scan after scan, so demanding agreement everywhere erases exactly the
// case that matters most. The two ways past it are the close-range floor, where
// there is no time to wait anyway, and a dense return in one scan, which says
// the object is there even if it will have moved on by the next scan.
//
// WHAT COUNTS AS ONE THING, within a scan, is decided by the caller before the
// returns get here, using the adaptive breakpoint detector of Borges and Aldon
// (Line Extraction in 2D Range Images for Mobile Robotics, JIRS 2004) - the
// same segmentation the ForzaETH F1TENTH stack finds opponents with. Two
// neighbouring returns are the same surface when they are closer together than
//
//     D_max = r * sin(dphi) / sin(lambda - dphi) + 3*sigma
//
// A fixed distance cannot be right at both ends of the range, and measuring it
// showed exactly that: at 0.15 m a far object split in two and its warning
// distance fell from 7.3 m to 2.3 m, at 0.45 m detections fell by a fifth.
//
// Segmenting also gives the honest version of "that is a wall": the SIZE of the
// surface the return belongs to, measured ALONG the whole surface rather than
// across the part of it that happens to lie near the line. Along, not end to
// end, because a wall that wraps round a hairpin brings its own two ends close
// together; and over every valid return, not only the ones near enough to
// judge, because cutting a surface at the edge of the lookahead is how a long
// wall would come out small enough to look like an obstacle. Measured over 0821-1, the
// places that were a real object sat on surfaces of 0.13 m median and 0.91 m at
// the very largest, while the false ones included surfaces of 10.6 m. Anything
// from 0.6 m to 3.0 m separates those equally well - there is nothing in
// between - where the along-the-line span it replaces had a cliff at 2.5 m.
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
    double near_line_m{0.28};            // the car's width, right in front
    double car_width_until_m{1.5};       // nearer than this, measure from the car
    double narrow_per_m{0.0};            // closes up this much per metre of range
    double never_narrower_than_m{0.05};  // and stops closing here
    // Only pieces rescued by splitting a parent surface above biggest_thing_m
    // lose this much corridor margin. Legacy-sized objects keep the established
    // corridor, so suppressing a wall-edge split cannot shorten their warning.
    double recovered_margin_cut_m{0.0};
    int scans_kept{5};              // how many scans are kept
    int points_per_scan{2};         // returns that make one scan a witness
    int dense_one_scan{8};          // this many in one scan is enough on its own
    double wall_longer_than_m{1.0};      // longer than this along the line is wall
    double one_scan_within_m{2.5};  // inside this, one scan is enough - the floor
    double add_scan_every_m{2.0};   // each further step of this asks for one more
    double add_scan_every_mps{3.0}; // each step of speed asks for one more
    double same_place_m{0.30};      // same place from one scan to the next
    double biggest_thing_m{1.5};    // a surface bigger than this is the track
    int beam_stride{3};             // use every nth beam
  };

  // One place on the line the scans say is occupied.
  struct Occupied
  {
    double along{0.0};       // metres ahead along the line
    double lateral{0.0};     // how far off the line its nearest return was.
                             // REPORTED ONLY - nothing decides on it. Being
                             // here at all already means it was inside the
                             // band, and a second test on the same quantity
                             // was measured to cost more than it bought.
    double extent{0.0};      // how far it ran along the line
    double range{0.0};       // how far the nearest supporting return was
    int scans_agreed{0};     // how many scans saw it
    int scans_needed{0};     // how many it had to have, at its range and speed
  };

  // One return, already placed in the path's frame and measured against the line.
  struct Point
  {
    double along{0.0};
    double lateral{0.0};      // from the line
    double side{0.0};         // from the car's own centreline
    double range{0.0};
    int segment{-1};          // which surface it belongs to, in this scan
    double segment_size{0.0}; // how big that whole surface is
    bool recovered_from_large{false};  // polyline piece split from an oversized parent
    // A raw return inside the planned swept corridor is collision evidence
    // even when the complete beam-connected surface is a wall. This is set by
    // the experimental direct-scan guard; segmentation still groups votes but
    // is not allowed to erase the return by semantic/size classification.
    bool preserve_if_large{false};
  };

  // How near the line a return this far away has to be before it is believed.
  double nearLineAt(double range) const
  {
    return std::max(
      s_.never_narrower_than_m, s_.near_line_m - s_.narrow_per_m * std::max(0.0, range));
  }

  void configure(const Settings & s)
  {
    s_ = s;
    s_.scans_kept = std::max(1, s_.scans_kept);
    s_.points_per_scan = std::max(1, s_.points_per_scan);
    while (ring_.size() > static_cast<std::size_t>(s_.scans_kept)) {ring_.pop_front();}
  }
  const Settings & settings() const {return s_;}
  void clear() {ring_.clear();}
  bool empty() const {return ring_.empty();}
  std::size_t scans() const {return ring_.size();}

  void push(std::vector<Point> && pts)
  {
    ring_.push_back(std::move(pts));
    while (ring_.size() > static_cast<std::size_t>(s_.scans_kept)) {ring_.pop_front();}
  }

  // How many of the kept scans a thing this far ahead has to appear in, at this
  // speed. One inside one_scan_within_m, rising from there.
  int scansNeeded(double range, double speed) const
  {
    const double over = std::max(0.0, range - s_.one_scan_within_m);
    double n = 1.0;
    if (s_.add_scan_every_m > 0.0) {n += over / s_.add_scan_every_m;}
    if (s_.add_scan_every_mps > 0.0) {n += std::max(0.0, speed) / s_.add_scan_every_mps;}
    return std::clamp(static_cast<int>(n), 1, s_.scans_kept);
  }

  std::vector<Occupied> occupied(double speed) const
  {
    // What each scan on its own says it saw: runs of returns close enough
    // together to be one thing, with at least points_per_scan in the run.
    struct Sighting
    {
      double along;    // where the run started
      double ends;     // where it ended
      double range;
      double lateral;  // reported only
      int points;
    };
    std::vector<std::vector<Sighting>> per_scan;
    per_scan.reserve(ring_.size());
    for (const auto & scan : ring_) {
      std::vector<Point> in;
      for (const auto & p : scan) {
        if (p.along <= 0.05) {continue;}
        double lateral_limit = nearLineAt(p.range);
        if (p.recovered_from_large) {
          lateral_limit = std::max(
            s_.never_narrower_than_m, lateral_limit - s_.recovered_margin_cut_m);
        }
        if (p.lateral > lateral_limit) {continue;}
        // Close in, it also has to be something the car itself would hit.
        if (s_.car_width_until_m > 0.0 && p.along < s_.car_width_until_m &&
          p.side > s_.near_line_m)
        {
          continue;
        }
        // A surface this big is the track, not a thing on it.
        if (
          !p.preserve_if_large && s_.biggest_thing_m > 0.0 &&
          p.segment_size > s_.biggest_thing_m)
        {
          continue;
        }
        in.push_back(p);
      }
      // One thing per SURFACE, as segmented by the caller.
      std::sort(
        in.begin(), in.end(),
        [](const Point & a, const Point & b) {
          return a.segment != b.segment ? a.segment < b.segment : a.along < b.along;
        });
      std::vector<Sighting> found;
      std::size_t i = 0;
      while (i < in.size()) {
        std::size_t j = i + 1;
        while (j < in.size() && in[j].segment == in[i].segment) {++j;}
        const int count = static_cast<int>(j - i);
        if (count >= s_.points_per_scan) {
          double nearest = in[i].range;
          double side = in[i].lateral;
          double lo = in[i].along, hi = in[i].along;
          for (std::size_t k = i; k < j; ++k) {
            nearest = std::min(nearest, in[k].range);
            side = std::min(side, in[k].lateral);
            lo = std::min(lo, in[k].along);
            hi = std::max(hi, in[k].along);
          }
          found.push_back({lo, hi, nearest, side, count});
        }
        i = j;
      }
      std::sort(
        found.begin(), found.end(),
        [](const Sighting & a, const Sighting & b) {return a.along < b.along;});
      per_scan.push_back(std::move(found));
    }
    if (per_scan.empty()) {return {};}

    // Sightings from different scans are grouped by how far apart they are,
    // not by a grid, so two returns either side of a cell edge are not split
    // into different things.
    std::vector<Sighting> all;
    for (const auto & s : per_scan) {all.insert(all.end(), s.begin(), s.end());}
    std::sort(
      all.begin(), all.end(),
      [](const Sighting & a, const Sighting & b) {return a.along < b.along;});

    std::vector<Occupied> out;
    std::size_t i = 0;
    while (i < all.size()) {
      std::size_t j = i + 1;
      while (j < all.size() && all[j].along - all[j - 1].along <= s_.same_place_m) {++j;}
      const double lo = all[i].along - s_.same_place_m;
      const double hi = all[j - 1].along + s_.same_place_m;
      // How many DISTINCT scans contributed, and the most any one of them saw.
      int agreed = 0;
      int densest = 0;
      double nearest = all[i].range;
      double side = all[i].lateral;
      for (const auto & s : per_scan) {
        bool voted = false;
        for (const auto & x : s) {
          if (x.along < lo || x.along > hi) {continue;}
          voted = true;
          densest = std::max(densest, x.points);
          nearest = std::min(nearest, x.range);
          side = std::min(side, x.lateral);
        }
        if (voted) {++agreed;}
      }
      double ends = all[i].ends;
      for (std::size_t k = i; k < j; ++k) {ends = std::max(ends, all[k].ends);}
      const double extent = ends - all[i].along;
      // Measured over the 0821-1 run: the places that were a real object ran a
      // median of 0.25m along the line and only 1% of them ran past 1.0m, while
      // every place flagged in the obstacle-free first half ran 1.93m.
      if (s_.wall_longer_than_m > 0.0 && extent > s_.wall_longer_than_m) {
        i = j;
        continue;
      }
      const int needed = scansNeeded(nearest, speed);
      if (agreed >= needed || densest >= s_.dense_one_scan) {
        Occupied o;
        o.along = all[i].along;
        o.lateral = side;
        o.extent = extent;
        o.range = nearest;
        o.scans_agreed = agreed;
        o.scans_needed = needed;
        out.push_back(o);
      }
      i = j;
    }
    return out;
  }

private:
  Settings s_;
  std::deque<std::vector<Point>> ring_;
};

}  // namespace co_driver

#endif  // CO_DRIVER__SCAN_OCCUPANCY_HPP_
