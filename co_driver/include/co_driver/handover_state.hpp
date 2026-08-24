#ifndef CO_DRIVER__HANDOVER_STATE_HPP_
#define CO_DRIVER__HANDOVER_STATE_HPP_

#include <algorithm>
#include <string>
#include <vector>

namespace co_driver
{

// A tick with no usable command is a gap in validity, not a handover. Keep the
// last drive that actually had the car so recovery on the next tick can still
// identify a configured gap -> PP return edge.
inline std::string rememberSelectedDrive(
  const std::string & previous, bool has_selection, const std::string & selected)
{
  return has_selection ? selected : previous;
}

inline bool isConfiguredHandback(
  bool switched, bool last_resort, const std::string & from,
  const std::string & selected, const std::vector<std::string> & configured_from,
  const std::string & configured_to)
{
  return switched && !last_resort && selected == configured_to &&
         std::find(configured_from.begin(), configured_from.end(), from) !=
         configured_from.end();
}

}  // namespace co_driver

#endif  // CO_DRIVER__HANDOVER_STATE_HPP_
