#ifndef CO_DRIVER__SPEED_HOLD_TIMING_HPP_
#define CO_DRIVER__SPEED_HOLD_TIMING_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace co_driver
{

// Timing-only views of the stateful postprocess stages. They intentionally do
// not contain ROS or message types, so the safety calculation can be unit
// tested independently of the node.
struct SwitchBlendTiming
{
  double duration_s{0.0};
  bool ema{false};
};

struct SpeedRateLimitTiming
{
  double max_accel{0.0};
  double max_decel{0.0};
};

struct SpeedOutputBounds
{
  bool bounded{false};
  double min_speed{0.0};
  double max_speed{0.0};
};

struct SpeedTransitionBudget
{
  bool bounded{true};
  double blend_s{0.0};
  double rate_limit_s{0.0};
  std::string reason;

  double total_s() const
  {
    return bounded ? blend_s + rate_limit_s : std::numeric_limits<double>::infinity();
  }
};

// Returns a conservative serial upper bound even though blend and rate-limit
// normally run together. The slowest enabled accel/decel value is used because
// a transition can cross zero or be interrupted while the blend is active.
inline SpeedTransitionBudget estimateSpeedTransitionBudget(
  double from_speed, double hold_speed, bool source_switched,
  const std::vector<SwitchBlendTiming> & blends,
  const std::vector<SpeedRateLimitTiming> & rate_limits,
  const SpeedOutputBounds & output_bounds)
{
  SpeedTransitionBudget budget;
  if (!std::isfinite(from_speed) || !std::isfinite(hold_speed)) {
    budget.bounded = false;
    budget.reason = "transition speed is not finite";
    return budget;
  }

  if (source_switched) {
    for (const auto & blend : blends) {
      if (blend.ema) {
        // EMA has no configured finish time and approaches its target
        // asymptotically. Guessing a duration here would silently shorten the
        // requested steady hold whenever its tolerance or alpha changes.
        budget.bounded = false;
        budget.reason = "switch_blend curve ema has no finite duration bound";
        return budget;
      }
      if (!std::isfinite(blend.duration_s)) {
        budget.bounded = false;
        budget.reason = "switch_blend duration is not finite";
        return budget;
      }
      budget.blend_s += std::max(0.0, blend.duration_s);
    }
  }

  double slowest_rate = std::numeric_limits<double>::infinity();
  for (const auto & limit : rate_limits) {
    // In PostProcess, non-positive/non-finite limits do not impose a finite
    // step, so only positive finite limits contribute to the upper bound.
    if (std::isfinite(limit.max_accel) && limit.max_accel > 0.0) {
      slowest_rate = std::min(slowest_rate, limit.max_accel);
    }
    if (std::isfinite(limit.max_decel) && limit.max_decel > 0.0) {
      slowest_rate = std::min(slowest_rate, limit.max_decel);
    }
  }
  if (std::isfinite(slowest_rate)) {
    // pure_pursuit counts only cycles in which it can produce output. If its
    // Path pauses after handback, co_driver may keep moving toward the cached
    // PP command while the receiver's hold clock has not started. Therefore
    // the handback-time output is not a safe endpoint for this calculation.
    // A finite final-output clamp gives the actual worst case at the first
    // held command, independent of how long that pause lasted.
    if (!output_bounds.bounded || !std::isfinite(output_bounds.min_speed) ||
      !std::isfinite(output_bounds.max_speed) ||
      output_bounds.min_speed > output_bounds.max_speed)
    {
      budget.bounded = false;
      budget.reason = "rate_limit has no finite final speed bounds";
      return budget;
    }
    budget.rate_limit_s = std::max(
      std::abs(hold_speed - output_bounds.min_speed),
      std::abs(output_bounds.max_speed - hold_speed)) / slowest_rate;
  }
  return budget;
}

inline bool speedHoldRequestDuration(
  double steady_hold_s, const SpeedTransitionBudget & budget,
  double receiver_max_s, double * request_s)
{
  if (request_s == nullptr || !budget.bounded || !std::isfinite(steady_hold_s) ||
    steady_hold_s < 0.0 || !std::isfinite(receiver_max_s) || receiver_max_s <= 0.0)
  {
    return false;
  }
  const double request = steady_hold_s + budget.total_s();
  if (!std::isfinite(request) || request > receiver_max_s) {return false;}
  *request_s = request;
  return true;
}

}  // namespace co_driver

#endif  // CO_DRIVER__SPEED_HOLD_TIMING_HPP_
