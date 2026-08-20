// Score computation -- turning input scores x into per-drive final scores p, and nothing else.
//
//   phi_ji(x_i)  response curve (linear + exponential)   shapeInfluence()
//   z_j          linear layer Sum W[j][i]*phi + b_j      computeLogit()
//   p_j          softmax(z / T)                          finalizeScores()
//
// Deciding "what information produces x" (looking at scan/map/imu/odom) is done
// entirely by the scorers in src/scorers/. Scorers subscribe to their own topics,
// so neither this file nor the node needs to know about sensors.
#ifndef CO_DRIVER__COMPUTE_HPP_
#define CO_DRIVER__COMPUTE_HPP_

#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "co_driver/config.hpp"

namespace co_driver
{

// Result one scorer produces for one drive.
//
// Four states:
//   ok          produced a score. Cached and used as-is.
//   pending     still computing (async scorer). The framework **substitutes the
//               previous score**, unless it is older than inputs[].hold_ms,
//               in which case it degrades to unavailable.
//   unavailable cannot judge (no data etc.). Explicit verdict -- not overridden by cache.
//   vetoed      disqualified.
struct ScoreResult
{
  double value{0.0};        // [0,1]. 1 is good. This is the response curve's x
  bool available{true};     // false = cannot judge this cycle -> excluded, weight and all
  bool veto{false};         // true = disqualify the drive regardless of score
  bool is_pending{false};   // true = still computing. Previous score is used for hold_ms
  std::string note;

  static ScoreResult ok(double v, const std::string & why = "")
  {
    ScoreResult r;
    r.value = v;
    r.note = why;
    return r;
  }
  static ScoreResult unavailable(const std::string & why = "")
  {
    ScoreResult r;
    r.available = false;
    r.note = why;
    return r;
  }
  static ScoreResult vetoed(const std::string & why = "")
  {
    ScoreResult r;
    r.veto = true;
    r.note = why;
    return r;
  }
  // For an async scorer to signal "no new result yet". The framework uses the previous score.
  static ScoreResult pending(const std::string & why = "")
  {
    ScoreResult r;
    r.available = false;
    r.is_pending = true;
    r.note = why;
    return r;
  }
};

// Last valid score per input. Used in place of the fresh one while pending.
struct CachedScore
{
  double value{0.0};
  rclcpp::Time stamp;
  bool valid{false};
};

// Configuration + runtime state of one drive.
struct Drive
{
  // Configuration (copy of DriveSpec)
  std::string name;
  std::string topic;
  bool enabled{true};
  double hold{0.3};       // validity window of the last command [s]
  double bias{0.0};
  std::map<std::string, Influence> influence;

  // Reception state
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  rclcpp::Time last_rx;
  bool has_cmd{false};
  // Measured receive period (EMA) -- diagnostics only. Useful when choosing hold.
  double period_ema{0.0};
  int rx_count{0};

  // Input name -> last valid score (fills async scorers' pending gaps)
  std::map<std::string, CachedScore> score_cache;

  // This cycle's results
  std::map<std::string, ScoreResult> results;   // scorer name -> score
  double raw_logit{0.0};      // z_j = Sum W*phi + b (this cycle)
  double logit{0.0};          // raw_logit after EMA. Softmax applies to this value
  bool logit_initialized{false};
  double score{0.0};          // final score. In softmax mode, the probability p_j
  // active = passed the hard gates, i.e. "usable right now".
  // Inactive drives still get scored, but drop out of softmax/ranking/selection.
  bool active{false};
  bool valid{false};          // active && score >= min_valid_score
  std::string reject;         // reason for inactive/invalid

  bool isFresh(const rclcpp::Time & now) const
  {
    if (!has_cmd) {return false;}
    return (now - last_rx).seconds() <= hold;
  }
  double age(const rclcpp::Time & now) const
  {
    if (!has_cmd) {return std::numeric_limits<double>::infinity();}
    return (now - last_rx).seconds();
  }
  double measuredHz() const {return period_ema > 1e-9 ? 1.0 / period_ema : 0.0;}

  // Updates the receive-period estimate on every message (diagnostic Hz display).
  void noteReceived(const rclcpp::Time & now)
  {
    if (has_cmd) {
      const double dt = (now - last_rx).seconds();
      // Abnormally long gaps (e.g. recovery after an outage) are excluded from the estimate.
      if (dt > 1e-6 && dt < 2.0) {
        period_ema = (rx_count < 1) ? dt : period_ema + 0.2 * (dt - period_ema);
        ++rx_count;
      }
    }
    last_rx = now;
    has_cmd = true;
  }
};

// Minimal shared state for scorers. No sensors here -- scorers subscribe themselves.
// This only holds node-internal state a scorer cannot know on its own.
struct Context
{
  rclcpp::Time now;
  double dt{0.0};                                  // time since the previous evaluation cycle [s]
  const std::vector<Drive> * drives{nullptr};      // for relative comparison across drives
  ackermann_msgs::msg::AckermannDrive last_output; // command actually published last
  bool has_last_output{false};
  std::string last_selected;                       // name of the drive selected last
};

// ---------------------------------------------------------------------------
// Response curve phi -- shapes the scorer's x in [0,1] into the linear-layer input.
//
//   u = invert ? 1-x : x
//   u = clamp((u - in_min) / (in_max - in_min), 0, 1)
//   s = (1 - exp_mix)*u + exp_mix*(e^(k*u) - 1)/(e^k - 1)
//
// The exponential term is normalized [0,1] -> [0,1], so the result stays in
// [0,1]. Changing the mix does not change the range, so no extra normalization
// is needed.
// ---------------------------------------------------------------------------
double shapeInfluence(const Influence & inf, double x);

// ---------------------------------------------------------------------------
// Score resolution -- fills an async scorer's pending from the cache, drops stale entries.
//
//   ok       -> update cache, use as-is
//   pending  -> use the cached value if within hold, else unavailable
//   others   -> pass through (explicit verdict, not overridden by cache)
//
// hold <= 0 means the cache never expires.
// held_age receives the age [s] of the cached score when it was used
// (diagnostics; negative otherwise).
// ---------------------------------------------------------------------------
ScoreResult resolveScore(
  Drive & d, const std::string & input, const ScoreResult & fresh, double hold,
  const rclcpp::Time & now, double * held_age = nullptr);

// ---------------------------------------------------------------------------
// Single-layer linear MLP + softmax
// ---------------------------------------------------------------------------
// Stage 1: one drive's linear-layer output z_j and its active flag.
//
// Scores are computed **regardless of active**. Even a drive past its hold must
// show its z so one can tell "why it wasn't picked". Instead active=false makes
// it drop out of the next stage's softmax/ranking/selection.
//
// Hard gates that clear active: disabled / never received / past hold (stale) /
//                               NaN / scorer veto / veto_below / required input unavailable
void computeLogit(Drive & d, const ScoringSpec & spec, const rclcpp::Time & now);

// Stage 2: softmax over **active drives only**, filling score / valid.
//          Inactive drives leave the denominator, so active probabilities always sum to 1.
void finalizeScores(std::vector<Drive> & drives, const ScoringSpec & spec);

}  // namespace co_driver

#endif  // CO_DRIVER__COMPUTE_HPP_
