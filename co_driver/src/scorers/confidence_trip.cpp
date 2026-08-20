// Early-degradation trip - hand over to the fallback BEFORE the score path can.
//
// The response curve saturates at in_max (0.35), so the score pathway only
// reacts deep below it: the effective handover sits at confidence 0.171. A
// localization that is quietly going wrong - hovering at 0.25-0.45 instead of
// collapsing to 0 - never crosses that and is never detected. This scorer
// watches the raw confidence directly and trips when it stays below trip_below
// for trip_ms continuously:
//
//   tripped   -> score 0.0  (wire with veto_below on the map-based drive:
//                            instant disqualification -> immediate handover)
//   otherwise -> score 1.0
//
// The sustain requirement is the point: a momentary dip proves nothing
// (confidence is noisy even when the pose is right), but a *sustained* stay
// below the bar is a real degradation. Detection tolerance therefore comes
// from trip_ms, not from lowering the bar.
//
// Clearing is symmetric: the trip releases only after confidence has stayed
// at or above trip_below for clear_ms continuously. Time is the verification
// in both directions - a sustained stay below the bar proves degradation, a
// sustained stay above it proves recovery. (The recovery latch still applies
// on top for the attach-after-Lost cases; this clear guards the 0.35-0.50
// band the latch's lower bar cannot see.)
//
// The trip is engaged ONLY while the state machine says Tracking. Lost and
// Converging belong to the other pathways (state veto and the recovery
// latch): a fresh attach legitimately sits below trip_below (Converging
// caps confidence at 0.5 x dominant mass), and gating it here would undo
// the attach-anchored return. Leaving Tracking disengages and resets the
// trip; re-entering Tracking starts fresh, because the recovery latch has
// already verified the attach by then.
//
// Relationship to the other pathways:
//   EMA score path   full collapses (-> 0), ~0.46 s      - unchanged backstop
//   state veto       declared Lost, ~20 ms               - unchanged
//   recovery latch   the way back after any handover     - unchanged
//   THIS             quiet degradation WHILE Tracking    - new, earlier
//
//   {
//     "name": "loc_trip", "type": "confidence_trip",
//     "params": {"topic": "/localization_confidence", "index": 2,
//                "state_topic": "/localization_pf/state",
//                "trip_below": 0.50, "trip_ms": 500, "clear_ms": 2500}
//   }
//   with, on the map-based drive only:
//     "loc_trip": {"weight": 0.0, "veto_below": 0.5}
//
// Caveat: trip_below 0.50 is tuned for a course where healthy confidence
// stays high. On a scene like busan2 (13.5% of healthy tracking below 0.5)
// it will trip during good driving - lengthen trip_ms or lower trip_below
// per track.
//
// Terms and ignore_terms: newer producers append the five raw terms behind
// the aggregate (data = [stamp_sec, stamp_nanosec, confidence, term_score,
// term_outlier, term_skip, term_spread, term_mass]; confidence = state
// multiplier x min of the five). When the array carries them, the trip judges
//
//     effective = min(terms not named in ignore_terms)
//
// instead of the aggregate, and the notes name the limiting term either way.
//
// This exists because one term is a nuisance meter rather than a pose-quality
// meter. term_skip reports how many beams had to be masked out as unexplained
// - i.e. how much unmapped or dynamic obstacle is in view - and on the
// producer side that masking runs BEFORE scoring, so term_score and
// term_outlier are computed on the surviving beams and stay at 1.000 through
// heavy occlusion. Judging the aggregate therefore hands the car to the
// fallback whenever another vehicle drives alongside, which is exactly the
// moment not to switch controllers. Measured on a busan2 replay: two spurious
// handovers, every pose term at 1.000 throughout.
//
// Excluding term_skip does NOT hide a real failure. The producer discards the
// whole mask once more than half the beams disagree ("if most of it does not
// match, we are probably lost"), and from that point term_score and
// term_outlier collapse together - so genuine mismatch is caught by the terms
// that remain. Keep term_spread in the judged set: it is the one that reacts
// if occlusion ever does start costing pose accuracy.
//
// The term path only applies behind the Tracking gate, because the terms are
// pre-multiplier values and only comparable to the bar where the multiplier
// is 1.0. Without state_topic the scorer keeps judging the aggregate.
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "co_driver/confidence_terms.hpp"
#include "co_driver/scorer.hpp"

namespace co_driver
{

class ConfidenceTripScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    topic_ = jstr(p, "topic", "/localization_confidence");
    index_ = jint(p, "index", 2);
    state_topic_ = jstr(p, "state_topic", "");
    trip_below_ = jnum(p, "trip_below", 0.50);
    // Producer-specific term names, so no default: the config names them.
    ignore_terms_ = jstrs(p, "ignore_terms");
    trip_ = jms(p, "trip_ms", 500.0);
    clear_ = jms(p, "clear_ms", 2500.0);
    timeout_ = jms(p, "timeout_ms", 300.0);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_, rclcpp::QoS(5),
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        const auto i = static_cast<std::size_t>(std::max(0, index_));
        if (i < msg->data.size()) {
          conf_ = msg->data[i];
          stamp_ = node_->now();
          has_msg_ = true;
        }
        // Optional appended terms; the decision itself is made in prepare().
        readConfidenceTerms(*msg, &terms_, &term_labels_);
      }, opts);

    if (!state_topic_.empty()) {
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
      "confidence trip '%s': < %.2f for %.0fms trips; >= %.2f for %.0fms clears%s%s",
      name.c_str(), trip_below_, trip_ * 1e3, trip_below_, clear_ * 1e3,
      state_topic_.empty() ? "" : " (Tracking only)",
      ignore_terms_.empty() ? "" : " (ignoring listed terms)");
    return true;
  }

  void prepare(const Context & ctx, const std::vector<Drive> &) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // Engaged only while Tracking: Lost/Converging belong to the state veto
    // and the recovery latch. Leaving Tracking resets the trip entirely.
    if (!state_topic_.empty() && !(has_state_ && state_ == 2)) {
      tripped_ = false;
      // Clear the direction as well, or score() keeps reporting the pre-excursion
      // "low ..." note for as long as the trip stays disengaged.
      was_low_ = false;
      // Drop the dwell edge too: re-entering Tracking must start measuring
      // from that moment, never from a timestamp taken before the excursion.
      edge_valid_ = false;
      low_for_ = high_for_ = 0.0;
      return;
    }
    // What the trip judges: the aggregate, or - when the producer publishes
    // the terms and the Tracking gate guarantees a multiplier of 1.0 - the
    // smallest term that is not on the ignore list.
    double effective = conf_;
    limit_.clear();
    limit_value_ = 0.0;
    eff_limit_.clear();
    obstacle_drag_ = false;
    if (!state_topic_.empty()) {
      double judged = 0.0;
      std::string judged_label;
      if (minJudgedTerm(
          terms_, term_labels_, ignore_terms_, &judged, &judged_label,
          &limit_value_, &limit_))
      {
        effective = judged;
        eff_limit_ = judged_label;
        // Worth surfacing: the aggregate looks bad only because of an ignored
        // term, i.e. obstacles are in view but the pose is fine.
        obstacle_drag_ = isIgnoredTerm(limit_, ignore_terms_) &&
          limit_value_ < trip_below_ && effective >= trip_below_;
      }
    }
    // Missing or stale confidence counts as low - a silent localization is a
    // failed one, same policy as the score input.
    const bool low = !has_msg_ ||
      (timeout_ > 0.0 && (ctx.now - stamp_).seconds() > timeout_) ||
      !std::isfinite(effective) || effective < trip_below_;
    // One edge timestamp for both directions, stamped from ctx.now the first
    // time it is needed. Keeping two separately-initialised timestamps meant
    // one of them could be read before ever being assigned - a default
    // rclcpp::Time carries the system clock, and subtracting it from the ROS
    // clock throws, killing the node.
    if (low != was_low_ || !edge_valid_) {
      edge_ = ctx.now;
      was_low_ = low;
      edge_valid_ = true;
    }
    const double dwell = (ctx.now - edge_).seconds();
    if (low) {
      low_for_ = dwell;
      high_for_ = 0.0;
      if (dwell >= trip_) {tripped_ = true;}
    } else {
      low_for_ = 0.0;
      high_for_ = dwell;
      if (tripped_ && dwell >= clear_) {tripped_ = false;}
    }
  }

  ScoreResult score(const Drive &, const Context &) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    char buf[96];
    if (tripped_) {
      if (was_low_) {
        std::snprintf(
          buf, sizeof(buf), "tripped: < %.2f for %.1fs%s%s", trip_below_, low_for_,
          eff_limit_.empty() ? "" : ", limit: ", eff_limit_.c_str());
      } else {
        std::snprintf(buf, sizeof(buf), "tripped: clearing %.1f/%.1fs", high_for_, clear_);
      }
      return ScoreResult::ok(0.0, buf);
    }
    if (was_low_) {
      std::snprintf(
        buf, sizeof(buf), "low %.1f/%.1fs%s%s", low_for_, trip_,
        eff_limit_.empty() ? "" : " ", eff_limit_.c_str());
      return ScoreResult::ok(1.0, buf);
    }
    if (obstacle_drag_) {
      // Healthy pose, obstacles in view - say so, or the low aggregate on the
      // status stream looks like an unexplained near-miss.
      std::snprintf(
        buf, sizeof(buf), "obstacles: %s %.2f (not judged)", limit_.c_str(), limit_value_);
      return ScoreResult::ok(1.0, buf);
    }
    return ScoreResult::ok(1.0);
  }

private:
  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;

  std::string topic_;
  std::string state_topic_;
  int index_{2};
  double trip_below_{0.50};
  double trip_{0.5};
  double clear_{2.5};
  double timeout_{0.3};
  std::vector<std::string> ignore_terms_;

  std::mutex mtx_;
  bool has_msg_{false};
  double conf_{0.0};
  std::vector<double> terms_;
  std::vector<std::string> term_labels_;
  std::string limit_, eff_limit_;
  double limit_value_{0.0};
  bool obstacle_drag_{false};
  rclcpp::Time stamp_;
  bool has_state_{false};
  uint8_t state_{255};
  bool was_low_{false};
  bool tripped_{false};
  double low_for_{0.0};
  double high_for_{0.0};
  bool edge_valid_{false};
  rclcpp::Time edge_;
};

CO_DRIVER_REGISTER_SCORER(ConfidenceTripScorer, "confidence_trip")

}  // namespace co_driver
