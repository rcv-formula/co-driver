// Scorer for a localization state topic (std_msgs/UInt8).
//
// slam_ours publishes /slam_ours/state as 0=Lost, 1=Converging, 2=Tracking,
// latched (transient_local). The confidence alone cannot tell a healthy
// Converging (raw 0.9 x multiplier 0.5 = 0.45) from a broken Tracking at 0.45;
// this input carries that distinction.
//
// The score is a straight mapping: Lost -> 0.0, Converging -> 0.5,
// Tracking -> 1.0 (each overridable). How it is *used* is the config's
// decision. The red_damvi configuration uses it as a pure gate -
// weight 0 with veto_below on pp_main only - so that Lost disqualifies the
// map-based drive instantly (bypassing EMA, margin and cooldown, because an
// invalid incumbent switches immediately) while the calibrated confidence
// thresholds stay exactly as verified: a zero weight adds nothing to any
// |W| sum, so the softmax math is identical with or without this input.
//
//   {
//     "name": "loc_state", "type": "localization_state",
//     "params": {"topic": "/slam_ours/state", "timeout_ms": 0},
//     "influence": {"weight": 0.0, "veto_below": -1.0}
//   }
//
// timeout_ms 0 (the default) disables staleness: a latched publisher only
// re-sends on transitions, so age since the last message means nothing.
//
// on_missing decides what "no message yet" means:
//   "value"        (red_damvi's choice) score default_score (0.0) - i.e. treat it
//                  like Lost. slam_ours publishes nothing until its map is loaded
//                  (first message is the Lost it emits at map-ready), so silence
//                  means localization has not even started - or never will, if the
//                  map failed to load. The gate must not open for that.
//   "unavailable"  the input drops out of the combination; at weight 0 the
//                  arbitration behaves as if this input did not exist. Only for
//                  benches or stacks that genuinely have no state topic.
//
// Note the producer's enum has four states (WaitingForMap, Lost, Converging,
// Tracking) folded to three on the wire: WaitingForMap and Lost both publish 0.
// For this gate they mean the same thing - do not trust the map-based drive.
#include <mutex>
#include <string>

#include <std_msgs/msg/u_int8.hpp>

#include "co_driver/scorer.hpp"

namespace co_driver
{

class LocalizationStateScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    topic_ = jstr(p, "topic", "/slam_ours/state");
    timeout_ = jms(p, "timeout_ms", 0.0);   // 0 = latched topic, never stale
    on_missing_ = jstr(p, "on_missing", "unavailable");   // unavailable | value
    default_score_ = jnum(p, "default_score", 0.0);
    score_lost_ = jnum(p, "lost_score", 0.0);
    score_converging_ = jnum(p, "converging_score", 0.5);
    score_tracking_ = jnum(p, "tracking_score", 1.0);

    // The publisher is latched (transient_local); the subscription must match
    // that durability or it will never receive the pre-existing state.
    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    sub_ = node->create_subscription<std_msgs::msg::UInt8>(
      topic_, qos,
      [this](const std_msgs::msg::UInt8::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        state_ = msg->data;
        stamp_ = node_->now();
        has_msg_ = true;
      }, opts);

    RCLCPP_INFO(
      node->get_logger(), "scorer '%s' <- %s (transient_local, timeout=%s)",
      name.c_str(), topic_.c_str(),
      timeout_ > 0.0 ? (std::to_string(static_cast<int>(timeout_ * 1e3)) + "ms").c_str() :
      "off");
    return true;
  }

  ScoreResult score(const Drive &, const Context & ctx) override
  {
    uint8_t state;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_msg_) {
        if (on_missing_ == "value") {
          // No message = the producer never reached map-ready. Same as Lost.
          return ScoreResult::ok(default_score_, "no state yet (treated as not trusted)");
        }
        return ScoreResult::unavailable("no message on " + topic_);
      }
      if (timeout_ > 0.0 && (ctx.now - stamp_).seconds() > timeout_) {
        return ScoreResult::unavailable("stale state");
      }
      state = state_;
    }
    switch (state) {
      case 0: return ScoreResult::ok(score_lost_, "Lost (or no map)");
      case 1: return ScoreResult::ok(score_converging_, "Converging");
      case 2: return ScoreResult::ok(score_tracking_, "Tracking");
      default: return ScoreResult::unavailable("unknown state " + std::to_string(state));
    }
  }

private:
  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_;

  std::string topic_;
  double timeout_{0.0};
  std::string on_missing_{"unavailable"};
  double default_score_{0.0};
  double score_lost_{0.0};
  double score_converging_{0.5};
  double score_tracking_{1.0};

  std::mutex mtx_;
  bool has_msg_{false};
  uint8_t state_{255};
  rclcpp::Time stamp_;
};

CO_DRIVER_REGISTER_SCORER(LocalizationStateScorer, "localization_state")

}  // namespace co_driver
