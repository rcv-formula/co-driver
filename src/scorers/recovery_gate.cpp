// Recovery gate - once the arbitration has left a drive, that drive may only
// come back when localization is FULLY healthy, and has been for a while.
//
// The two existing pathways make *leaving* pp_main fast (confidence EMA for
// gradual degradation, the state veto for abrupt Lost). This scorer governs
// the way *back*. It is a latch with asymmetric edges:
//
//   while the drive is the incumbent   -> gate open, no effect at all.
//                                         Leaving is the other pathways' job;
//                                         this scorer never causes a handover.
//   once the arbitration has left it   -> gate closed. It reopens only after
//                                         state >= min_state AND confidence >=
//                                         min_confidence have held
//                                         **continuously** for hold_ms.
//                                         Any dip resets the clock.
//
// min_state picks the anchor of the verification:
//   2  wait for full convergence (Tracking) - the conservative original
//   1  judge from the moment global search has ATTACHED (Converging entry).
//      The dwell plus the confidence bar do the verification: a wrong attach
//      decays (measured: 0.466 -> 0.38 @1s -> 0.29 @2s before dying at 2.8 s)
//      and breaks the hold, while a correct one sits flat at 0.48+. Survival
//      alone is NOT proof - the bar must stay high enough to catch the decay,
//      which is why lowering it much below 0.35 defeats the design.
//
// Wired like the state gate: weight 0 (adds nothing to any |W| sum, so the
// calibrated thresholds are untouched) with veto_below on pp_main only. While
// the gate is closed pp_main is disqualified, so no score, margin or cooldown
// can bring it back early. After it reopens, the normal selection rules
// (switch_margin, switch_cooldown) still apply on top.
//
//   {
//     "name": "recovery", "type": "recovery_gate",
//     "params": {
//       "confidence_topic": "/localization_confidence", "confidence_index": 2,
//       "confidence_timeout_ms": 300,
//       "state_topic": "/slam_ours/state", "require_tracking": true,
//       "min_confidence": 0.35, "hold_ms": 3000
//     },
//     "influence": {"weight": 0.0, "veto_below": -1.0}   // per-drive opt-in
//   }
//
// ignore_terms works exactly as it does for the score input and the trip: the
// health test uses state_multiplier x min(terms not ignored) when the producer
// appends them, so persistent occlusion (term_skip low, pose terms fine) does
// not strand the car on the fallback for as long as an obstacle is in view.
//
// "Fully healthy" deliberately means more than "above the handover threshold":
// handover happens below confidence 0.171, but reopening demands 0.35 - the
// point where the response curve saturates and the value is fully out of the
// danger band - plus the state machine agreeing that it is Tracking, not
// Converging. A drive with nothing selected yet (startup) counts as
// not-incumbent: the first selection of pp_main also waits for sustained
// health, which is exactly the safe behaviour on a car that boots mid-track.
#include <mutex>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "co_driver/confidence_terms.hpp"
#include "co_driver/scorer.hpp"

namespace co_driver
{

class RecoveryGateScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    conf_topic_ = jstr(p, "confidence_topic", "/localization_confidence");
    conf_index_ = jint(p, "confidence_index", 2);
    conf_timeout_ = jms(p, "confidence_timeout_ms", 300.0);
    state_topic_ = jstr(p, "state_topic", "/slam_ours/state");
    // min_state: 2 = require Tracking, 1 = attached (Converging) is enough,
    // 0 = ignore the state entirely. require_tracking is the legacy spelling.
    min_state_ = jint(p, "min_state", jbool(p, "require_tracking", true) ? 2 : 0);
    min_confidence_ = jnum(p, "min_confidence", 0.35);
    ignore_terms_ = jstrs(p, "ignore_terms");
    hold_ = jms(p, "hold_ms", 3000.0);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    conf_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      conf_topic_, rclcpp::QoS(5),
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        const auto i = static_cast<std::size_t>(std::max(0, conf_index_));
        if (i < msg->data.size()) {
          conf_ = msg->data[i];
          conf_stamp_ = node_->now();
          has_conf_ = true;
        }
        readConfidenceTerms(*msg, &terms_, &labels_);
      }, opts);

    if (min_state_ > 0) {
      rclcpp::QoS qos(1);
      qos.reliable().transient_local();
      state_sub_ = node->create_subscription<std_msgs::msg::UInt8>(
        state_topic_, qos,
        [this](const std_msgs::msg::UInt8::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          state_ = msg->data;
          has_state_ = true;
        }, opts);
    }

    // Guardrail: the producer halves the confidence during Converging (state
    // multiplier 0.5), and multi-hypothesis ambiguity lowers the ceiling
    // further (conf <= 0.5 x dominant mass). A bar at or above ~0.45 can
    // therefore never be met before Tracking, which silently defeats
    // min_state 1 - the exact misconfiguration that pinned a test car on
    // gap_follow. Warn loudly instead of failing silently.
    if (min_state_ == 1 && min_confidence_ >= 0.45) {
      RCLCPP_WARN(
        node->get_logger(),
        "recovery gate '%s': min_confidence %.2f is above the Converging ceiling "
        "(~0.5 x dominant hypothesis mass). With min_state 1 the hold can never "
        "start before Tracking - the attach anchor is effectively disabled. "
        "Use 0.25-0.35, or set min_state 2 if you really want this bar.",
        name.c_str(), min_confidence_);
    }
    RCLCPP_INFO(
      node->get_logger(),
      "recovery gate '%s': reopen needs %s+ confidence >= %.2f held %.1fs",
      name.c_str(),
      min_state_ >= 2 ? "Tracking" : (min_state_ == 1 ? "attached (Converging)" : "any state"),
      min_confidence_, hold_);
    return true;
  }

  // Evaluate "fully healthy" once per cycle; per-drive score() just reads it.
  void prepare(const Context & ctx, const std::vector<Drive> &) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // Judge the same value the rest of the arbitration judges: the aggregate,
    // or the smallest non-ignored term scaled by the state multiplier.
    double value = conf_;
    if (!ignore_terms_.empty()) {
      double judged = 0.0;
      std::string judged_label;
      if (minJudgedTerm(terms_, labels_, ignore_terms_, &judged, &judged_label)) {
        const double mult = (!has_state_ || state_ == 2) ? 1.0 : (state_ == 1 ? 0.5 : 0.0);
        value = mult * judged;
      }
    }
    const bool conf_ok = has_conf_ &&
      (conf_timeout_ <= 0.0 || (ctx.now - conf_stamp_).seconds() <= conf_timeout_) &&
      std::isfinite(value) && value >= min_confidence_;
    // Silence on the state topic reads as not-attached, same as the state gate.
    const bool state_ok = min_state_ <= 0 ||
      (has_state_ && state_ != 255 && state_ >= min_state_ && state_ <= 2);
    const bool healthy = conf_ok && state_ok;

    if (healthy && !was_healthy_) {
      healthy_since_ = ctx.now;
    }
    was_healthy_ = healthy;
    held_ = healthy && (ctx.now - healthy_since_).seconds() >= hold_;
    held_for_ = healthy ? (ctx.now - healthy_since_).seconds() : 0.0;
  }

  ScoreResult score(const Drive & drive, const Context & ctx) override
  {
    // The incumbent is never gated - leaving is the other pathways' decision.
    if (!ctx.last_selected.empty() && ctx.last_selected == drive.name) {
      return ScoreResult::ok(1.0, "incumbent");
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (held_) {
      return ScoreResult::ok(1.0, "recovered");
    }
    char buf[64];
    std::snprintf(
      buf, sizeof(buf), was_healthy_ ? "recovering %.1f/%.1fs" : "not healthy",
      held_for_, hold_);
    return ScoreResult::ok(0.0, buf);
  }

private:
  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr conf_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;

  std::string conf_topic_, state_topic_;
  int conf_index_{2};
  double conf_timeout_{0.3};
  int min_state_{2};
  double min_confidence_{0.35};
  std::vector<std::string> ignore_terms_;
  double hold_{3.0};

  std::mutex mtx_;
  bool has_conf_{false}, has_state_{false};
  double conf_{0.0};
  std::vector<double> terms_;
  std::vector<std::string> labels_;
  rclcpp::Time conf_stamp_;
  uint8_t state_{255};
  bool was_healthy_{false};
  bool held_{false};
  double held_for_{0.0};
  rclcpp::Time healthy_since_;
};

CO_DRIVER_REGISTER_SCORER(RecoveryGateScorer, "recovery_gate")

}  // namespace co_driver
