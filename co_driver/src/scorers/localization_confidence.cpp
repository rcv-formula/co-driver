// Localization confidence, with nuisance terms excluded.
//
// A specialisation of the external_score contract for one producer: the
// localization stacks that publish
//
//   data = [stamp_sec, stamp_nanosec, confidence, term_score, term_outlier,
//           term_skip, term_spread, term_mass]
//   confidence = state_multiplier x min(the five terms)
//
// Older builds publish only the first three slots, and this scorer then
// behaves exactly like external_score in scalar mode.
//
// Why it exists: term_skip is a nuisance meter, not a pose-quality meter. It
// reports how many beams had to be masked out as unexplained - how much
// unmapped or dynamic obstacle is in view - and the producer masks those beams
// BEFORE scoring, so term_score and term_outlier keep reading 1.000 through
// heavy occlusion. Feeding the aggregate into the arbitration therefore drags
// the map-based drive's score down whenever another vehicle drives alongside,
// even though the pose is perfectly good. With ignore_terms this scorer
// reports instead
//
//   effective = state_multiplier x min(terms not named in ignore_terms)
//
// so occlusion alone no longer costs the drive its score, while genuine
// mismatch still does: the producer discards the whole beam mask once more
// than half the beams disagree, and from that point term_score and
// term_outlier collapse together.
//
// The state multiplier is taken from the state topic rather than divided back
// out of the aggregate, because the aggregate is exactly 0 in the case that
// matters (term_skip can reach 0) and the division would be undefined there.
// Without a state topic the multiplier is assumed to be 1.0, which is only
// correct where the caller already knows the stack is tracking.
#include <algorithm>
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

class LocalizationConfidenceScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    topic_ = jstr(p, "topic", "/localization_confidence");
    index_ = jint(p, "index", 2);
    stamp_indices_ = jnums(p, "stamp_indices");
    timeout_ = jms(p, "timeout_ms", 300.0);
    input_min_ = jnum(p, "input_min", 0.0);
    input_max_ = jnum(p, "input_max", 1.0);
    invert_ = jbool(p, "invert", false);
    on_missing_ = jstr(p, "on_missing", "unavailable");
    default_score_ = jnum(p, "default_score", 0.0);
    ignore_terms_ = jstrs(p, "ignore_terms");
    state_topic_ = jstr(p, "state_topic", "");
    mult_lost_ = jnum(p, "multiplier_lost", 0.0);
    mult_converging_ = jnum(p, "multiplier_converging", 0.5);
    mult_tracking_ = jnum(p, "multiplier_tracking", 1.0);
    if (std::abs(input_max_ - input_min_) < 1e-9) {
      RCLCPP_ERROR(node->get_logger(), "'%s': input_min and input_max are equal.", name.c_str());
      return false;
    }

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_, rclcpp::QoS(5),
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {onMsg(msg);}, opts);

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
      node->get_logger(), "scorer '%s' <- %s (timeout=%.0fms%s)",
      name.c_str(), topic_.c_str(), timeout_ * 1e3,
      ignore_terms_.empty() ? "" : ", ignoring listed terms");
    return true;
  }

  ScoreResult score(const Drive &, const Context & ctx) override
  {
    double raw = 0.0;
    std::string why;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_msg_) {
        why = "no message on " + topic_;
      } else if (timeout_ > 0.0 && (ctx.now - stamp_).seconds() > timeout_) {
        why = "stale " + std::to_string((ctx.now - stamp_).seconds()) + "s";
      } else {
        raw = effective();
      }
    }
    if (!why.empty()) {
      if (on_missing_ == "veto") {return ScoreResult::vetoed(why);}
      if (on_missing_ == "value") {
        return ScoreResult::ok(std::clamp(default_score_, 0.0, 1.0), why);
      }
      return ScoreResult::unavailable(why);
    }
    if (!std::isfinite(raw)) {return ScoreResult::unavailable("non-finite value");}
    const double v = std::clamp((raw - input_min_) / (input_max_ - input_min_), 0.0, 1.0);
    return ScoreResult::ok(invert_ ? 1.0 - v : v, note_);
  }

private:
  // Caller holds mtx_.
  double effective()
  {
    note_.clear();
    if (terms_.size() != 5 || ignore_terms_.empty()) {
      return conf_;                       // 3-slot producer, or nothing to exclude
    }
    double judged = 0.0, overall = 0.0;
    std::string judged_label, overall_label;
    if (!minJudgedTerm(
        terms_, labels_, ignore_terms_, &judged, &judged_label, &overall, &overall_label))
    {
      return conf_;                       // every term ignored: nothing to judge on
    }
    if (isIgnoredTerm(overall_label, ignore_terms_) && overall < judged) {
      note_ = "excluded " + overall_label + " " + fmt2(overall);
    }
    return multiplier() * judged;
  }

  double multiplier() const
  {
    if (state_topic_.empty() || !has_state_) {return mult_tracking_;}
    switch (state_) {
      case 0: return mult_lost_;
      case 1: return mult_converging_;
      case 2: return mult_tracking_;
      default: return mult_tracking_;
    }
  }

  static std::string fmt2(double v)
  {
    char b[16];
    std::snprintf(b, sizeof(b), "%.2f", v);
    return b;
  }

  void onMsg(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const auto i = static_cast<std::size_t>(std::max(0, index_));
    if (i < msg->data.size()) {conf_ = msg->data[i];}

    readConfidenceTerms(*msg, &terms_, &labels_);

    // std_msgs has no header, so the stamp rides in the array when configured.
    bool stamped = false;
    if (stamp_indices_.size() == 2) {
      const auto si = static_cast<std::size_t>(stamp_indices_[0]);
      const auto ni = static_cast<std::size_t>(stamp_indices_[1]);
      if (si < msg->data.size() && ni < msg->data.size()) {
        const double sec = msg->data[si], nsec = msg->data[ni];
        // These come straight off the wire. rclcpp::Time throws on a negative
        // time point, and this runs in a subscription callback - under the
        // MultiThreadedExecutor an escaping exception is std::terminate, i.e.
        // the whole arbiter dies mid-run with no /drive. A malformed stamp is
        // not worth the car; fall back to arrival time.
        if (std::isfinite(sec) && std::isfinite(nsec) &&
          sec >= 0.0 && sec <= 4.0e18 && nsec >= 0.0 && nsec < 2.0e9)
        {
          stamp_ = rclcpp::Time(
            static_cast<int64_t>(sec), static_cast<uint32_t>(nsec),
            node_->get_clock()->get_clock_type());
          stamped = true;
        } else {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "'%s': stamp slots carry %.3f/%.3f - not a usable time; using arrival time.",
            name().c_str(), sec, nsec);
        }
      }
    }
    if (!stamped) {stamp_ = node_->now();}
    has_msg_ = true;
  }

  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;

  std::string topic_, state_topic_;
  int index_{2};
  std::vector<double> stamp_indices_;
  double timeout_{0.3};
  double input_min_{0.0}, input_max_{1.0};
  bool invert_{false};
  std::string on_missing_{"unavailable"};
  double default_score_{0.0};
  std::vector<std::string> ignore_terms_;
  double mult_lost_{0.0}, mult_converging_{0.5}, mult_tracking_{1.0};

  std::mutex mtx_;
  bool has_msg_{false};
  double conf_{0.0};
  std::vector<double> terms_;
  std::vector<std::string> labels_;
  rclcpp::Time stamp_;
  bool has_state_{false};
  uint8_t state_{255};
  std::string note_;
};

CO_DRIVER_REGISTER_SCORER(LocalizationConfidenceScorer, "localization_confidence")

}  // namespace co_driver
