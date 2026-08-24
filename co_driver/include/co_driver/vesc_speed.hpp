#ifndef CO_DRIVER__VESC_SPEED_HPP_
#define CO_DRIVER__VESC_SPEED_HPP_

#include <algorithm>
#include <cmath>

namespace co_driver
{

struct VescSpeedCalibration
{
  double speed_to_erpm_gain{3172.47};
  double speed_to_erpm_offset{0.0};
  double wheel_speed_deadband{0.05};
  double wheel_speed_scale{2.6};
};

// Same conversion used by f1_stack_for_damvi/red_damvi's vesc_to_odom and
// PPcontroller. The scale is deliberately separate: the calibrated ERPM
// conversion reads about 2.6 times below the command/vehicle-speed domain used
// by co_driver's obstacle geometry.
inline bool vescErpmToSpeed(
  double erpm, const VescSpeedCalibration & calibration, double * speed_mps)
{
  if (
    speed_mps == nullptr || !std::isfinite(erpm) ||
    !std::isfinite(calibration.speed_to_erpm_gain) ||
    std::abs(calibration.speed_to_erpm_gain) < 1.0e-6 ||
    !std::isfinite(calibration.speed_to_erpm_offset) ||
    !std::isfinite(calibration.wheel_speed_deadband) ||
    !std::isfinite(calibration.wheel_speed_scale) || calibration.wheel_speed_scale <= 0.0)
  {
    return false;
  }

  double raw_mps =
    (erpm - calibration.speed_to_erpm_offset) / calibration.speed_to_erpm_gain;
  if (!std::isfinite(raw_mps)) {return false;}
  if (std::abs(raw_mps) < std::max(0.0, calibration.wheel_speed_deadband)) {
    raw_mps = 0.0;
  }

  *speed_mps = raw_mps * calibration.wheel_speed_scale;
  return std::isfinite(*speed_mps);
}

inline double selectGeometrySpeed(
  bool has_measured_speed, double measured_age, double timeout,
  double measured_speed, double commanded_speed)
{
  if (has_measured_speed && (timeout <= 0.0 || measured_age <= timeout)) {
    return std::abs(measured_speed);
  }
  return std::abs(commanded_speed);
}

}  // namespace co_driver

#endif  // CO_DRIVER__VESC_SPEED_HPP_
