// Small, ROS-free part of the runtime drive-source selector. Keeping the
// threshold decision here makes endpoint and neutral-position behaviour easy
// to unit test without constructing the full arbitration node.
#ifndef CO_DRIVER__DRIVE_SOURCE_SWITCH_HPP_
#define CO_DRIVER__DRIVE_SOURCE_SWITCH_HPP_

#include <cstdint>

namespace co_driver
{

enum class DriveSourceChoice
{
  keep,
  primary,
  alternate
};

inline DriveSourceChoice chooseDriveSource(
  const std::uint16_t value, const std::uint16_t primary_value,
  const std::uint16_t alternate_value, const std::uint16_t tolerance)
{
  const auto distance = [](const std::uint16_t a, const std::uint16_t b) {
      return a > b ? static_cast<unsigned>(a - b) : static_cast<unsigned>(b - a);
    };
  const bool primary = distance(value, primary_value) <= tolerance;
  const bool alternate = distance(value, alternate_value) <= tolerance;
  if (primary == alternate) {
    // Neither endpoint, or an invalid overlapping configuration. Config load
    // rejects overlap; keeping the current source is safest in either case.
    return DriveSourceChoice::keep;
  }
  return primary ? DriveSourceChoice::primary : DriveSourceChoice::alternate;
}

}  // namespace co_driver

#endif  // CO_DRIVER__DRIVE_SOURCE_SWITCH_HPP_
