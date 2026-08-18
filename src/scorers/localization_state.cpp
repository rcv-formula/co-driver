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
// If the topic never arrives at all (e.g. running against localization_pf,
// which has no state topic), the score is unavailable and - at weight 0 -
// the arbitration behaves exactly as if this input did not exist.
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
        return ScoreResult::unavailable("no message on " + topic_);
      }
      if (timeout_ > 0.0 && (ctx.now - stamp_).seconds() > timeout_) {
        return ScoreResult::unavailable("stale state");
      }
      state = state_;
    }
    switch (state) {
      case 0: return ScoreResult::ok(score_lost_, "Lost");
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
