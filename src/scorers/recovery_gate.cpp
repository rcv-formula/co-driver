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
//                                         state == Tracking AND confidence >=
//                                         min_confidence have held
//                                         **continuously** for hold_ms.
//                                         Any dip resets the clock.
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
    require_tracking_ = jbool(p, "require_tracking", true);
    min_confidence_ = jnum(p, "min_confidence", 0.35);
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
      }, opts);

    if (require_tracking_) {
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

    RCLCPP_INFO(
      node->get_logger(),
      "recovery gate '%s': reopen needs %s confidence >= %.2f held %.1fs",
      name.c_str(), require_tracking_ ? "Tracking +" : "", min_confidence_, hold_);
    return true;
  }

  // Evaluate "fully healthy" once per cycle; per-drive score() just reads it.
  void prepare(const Context & ctx, const std::vector<Drive> &) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const bool conf_ok = has_conf_ &&
      (conf_timeout_ <= 0.0 || (ctx.now - conf_stamp_).seconds() <= conf_timeout_) &&
      std::isfinite(conf_) && conf_ >= min_confidence_;
    // Silence on the state topic reads as not-Tracking, same as the state gate.
    const bool state_ok = !require_tracking_ || (has_state_ && state_ == 2);
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
  bool require_tracking_{true};
  double min_confidence_{0.35};
  double hold_{3.0};

  std::mutex mtx_;
  bool has_conf_{false}, has_state_{false};
  double conf_{0.0};
  rclcpp::Time conf_stamp_;
  uint8_t state_{255};
  bool was_healthy_{false};
  bool held_{false};
  double held_for_{0.0};
  rclcpp::Time healthy_since_;
};

CO_DRIVER_REGISTER_SCORER(RecoveryGateScorer, "recovery_gate")

}  // namespace co_driver
