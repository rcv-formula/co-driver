// Projecting a drive command onto the ground, shared by the scorers that ask
// "does this command run into something".
//
// A command gives a speed and a steering angle; with the wheelbase that is a
// circular arc. Everything here works in the sensor frame with x forward and
// y left, which is what both /scan and the obstacle detector publish, so no TF
// is involved.
//
// Two limits are not optional, both learned from the 0813 recording:
//
//   horizon    must be at least the clearance being demanded. Callers that
//              report "the horizon" when nothing blocks will otherwise read an
//              empty scan as blocked whenever speed x lookahead_time falls
//              below that clearance - 9 of 11 spurious vetoes came from this.
//
//   max_sweep  a command is a snapshot, not a trajectory. At full lock the arc
//              is a 30 cm circle, and following it far enough curls the
//              corridor around into whatever is beside or behind the car. The
//              remaining 2 spurious vetoes came from 140 degrees of sweep
//              while the path ahead was open at 2.86 m.
#ifndef CO_DRIVER__ARC_GEOMETRY_HPP_
#define CO_DRIVER__ARC_GEOMETRY_HPP_

#include <cmath>

namespace co_driver
{

struct ArcProjection
{
  bool straight{true};
  double radius{0.0};      // signed; positive turns left
  double horizon{1.0};     // metres of arc length to consider
  double max_sweep{M_PI / 2.0};   // radians of heading change to consider

  static ArcProjection fromCommand(
    double steering_angle, double wheelbase, double horizon_m, double max_sweep_rad)
  {
    ArcProjection a;
    a.straight = std::abs(std::tan(steering_angle)) < 1e-4;
    a.radius = a.straight ? 0.0 : wheelbase / std::tan(steering_angle);
    a.horizon = horizon_m;
    a.max_sweep = max_sweep_rad;
    return a;
  }

  // Where a point sits relative to the commanded path: how far along the arc
  // it lies, and how far it is from the arc sideways. False when the point is
  // behind us, past the horizon, or past the sweep cap.
  bool project(double px, double py, double * along, double * lateral) const
  {
    double a, l;
    if (straight) {
      a = px;
      l = std::abs(py);
    } else {
      const double qx = px;
      const double qy = py - radius;
      // Signed heading change to reach the point, in the same sign convention
      // as radius: positive for a left turn, negative for a right one.
      const double t = std::atan2(qx / radius, -qy / radius);
      // Travelling forward moves the heading in the direction of the turn, so
      // anything whose sign disagrees with radius is BEHIND the vehicle.
      //
      // This used to normalise a negative t by adding 2*pi, which is only
      // correct for a left turn. On a right turn every forward point has t < 0
      // by construction, so it became ~2*pi - |t|, always exceeded max_sweep,
      // and project() rejected it. path_clearance and obstacle_avoid were
      // therefore blind for the whole of every right-hand turn - silently,
      // reporting "clear" rather than "no data".
      if (t * radius < 0.0) {return false;}
      const double theta = std::abs(t);
      if (theta > max_sweep) {return false;}
      a = std::abs(radius) * theta;
      l = std::abs(std::hypot(qx, qy) - std::abs(radius));
    }
    if (a <= 0.0 || a > horizon) {return false;}
    *along = a;
    *lateral = l;
    return true;
  }
};

}  // namespace co_driver

#endif  // CO_DRIVER__ARC_GEOMETRY_HPP_
