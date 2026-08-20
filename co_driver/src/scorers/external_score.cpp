// Example scorer / skeleton -- subscribes to a score topic published by an
// external node and uses it as the score.
//
// Copy this file to start a new scorer. A scorer needs no header and is
// complete in a single .cpp (the registry instantiates it from its type string).
// The structure has exactly three parts.
//
//   1) configure()  read the config and, **if needed, create your own subscription**
//   2) callback     store the received data in members (different thread, so mutex)
//   3) score()      return a [0,1] score for one drive
//
// ---------------------------------------------------------------------------
// Contract supported by this scorer (type: "external_score")
//
//   Message: std_msgs/Float64MultiArray
//
//   mode: "scalar"         one array slot at `index` is a "whole vehicle" score;
//                          the same value applies to every drive. localization_pf's
//                          /localization_confidence (`[stamp_sec, stamp_nanosec,
//                          confidence]`, index=2) has this shape.
//
//   mode: "per_candidate"  per-drive scores. Matching priority:
//                            (a) the slot whose layout.dim[i].label equals the drive name
//                            (b) data[i] in the order given by params' order: ["a","b",...]
//
//   If stamp_indices is given, those slots are read as [sec, nanosec] to judge
//   freshness (timeout_ms) (std_msgs has no header; this is the convention
//   localization_pf uses).
//   Values are normalized [input_min, input_max] -> [0,1], so raw units are fine.
//   A NaN marks only that drive as "cannot judge".
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>

#include "co_driver/scorer.hpp"

namespace co_driver
{

class ExternalScoreScorer : public Scorer
{
public:
  // --- 1) Read config + create our own subscription --------------------------
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    topic_ = jstr(p, "topic", "");
    if (topic_.empty()) {
      RCLCPP_ERROR(node->get_logger(), "'%s': params.topic is required.", name.c_str());
      return false;
    }
    mode_ = jstr(p, "mode", "scalar");
    if (mode_ != "scalar" && mode_ != "per_candidate") {
      RCLCPP_ERROR(
        node->get_logger(), "'%s': mode must be scalar or per_candidate (got: %s)",
        name.c_str(), mode_.c_str());
      return false;
    }
    index_ = jint(p, "index", 0);
    stamp_indices_ = jnums(p, "stamp_indices");
    order_ = jstrs(p, "order");
    timeout_ = jms(p, "timeout_ms", 500.0);
    input_min_ = jnum(p, "input_min", 0.0);
    input_max_ = jnum(p, "input_max", 1.0);
    invert_ = jbool(p, "invert", false);
    on_missing_ = jstr(p, "on_missing", "unavailable");   // unavailable | value | veto
    default_score_ = jnum(p, "default_score", 0.5);
    if (std::abs(input_max_ - input_min_) < 1e-9) {
      RCLCPP_ERROR(node->get_logger(), "'%s': input_min and input_max are equal.", name.c_str());
      return false;
    }

    // Only the latest value of a score topic matters, so keep the queue short.
    // The callback goes in the scorer-only (Reentrant) group so it never blocks
    // the evaluation/output timers.
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_, rclcpp::QoS(5),
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {onMsg(msg);}, opts);

    RCLCPP_INFO(
      node->get_logger(), "scorer '%s' <- %s (mode=%s, index=%d, timeout=%.0fms)",
      name.c_str(), topic_.c_str(), mode_.c_str(), index_, timeout_ * 1e3);
    return true;
  }

  // --- 3) Score one drive ----------------------------------------------------
  ScoreResult score(const Drive & drive, const Context & ctx) override
  {
    double raw = 0.0;
    bool found = false;
    std::string why;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_msg_) {
        why = "no message on " + topic_;
      } else if (timeout_ > 0.0 && (ctx.now - stamp_).seconds() > timeout_) {
        why = "stale " + std::to_string((ctx.now - stamp_).seconds()) + "s";
      } else if (mode_ == "scalar") {
        const auto i = static_cast<std::size_t>(std::max(0, index_));
        if (i < data_.size()) {
          raw = data_[i];
          found = true;
        } else {
          why = "index out of range";
        }
      } else {
        // per_candidate: (a) layout label match first, (b) position in the order list
        std::size_t slot = 0;
        bool have = false;
        for (std::size_t i = 0; i < labels_.size(); ++i) {
          if (labels_[i] == drive.name) {slot = i; have = true; break;}
        }
        if (!have) {
          for (std::size_t i = 0; i < order_.size(); ++i) {
            if (order_[i] == drive.name) {slot = i; have = true; break;}
          }
        }
        if (!have) {
          why = "drive not in labels/order";
        } else if (slot < data_.size()) {
          raw = data_[slot];
          found = true;
        } else {
          why = "slot out of range";
        }
      }
    }

    if (!found) {
      if (on_missing_ == "veto") {return ScoreResult::vetoed(why);}
      if (on_missing_ == "value") {
        return ScoreResult::ok(std::clamp(default_score_, 0.0, 1.0), why);
      }
      return ScoreResult::unavailable(why);
    }
    // External nodes really do emit NaN while dying.
    if (!std::isfinite(raw)) {return ScoreResult::unavailable("non-finite value");}

    double v = std::clamp((raw - input_min_) / (input_max_ - input_min_), 0.0, 1.0);
    return ScoreResult::ok(invert_ ? 1.0 - v : v);
  }

private:
  // --- 2) Callback: only stores what arrives (scoring happens in score()) ----
  void onMsg(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = msg->data;
    labels_.clear();
    for (const auto & d : msg->layout.dim) {labels_.push_back(d.label);}

    // std_msgs has no header, so use the stamp carried in the leading array slots (if present).
    bool stamped = false;
    if (stamp_indices_.size() == 2) {
      const auto si = static_cast<std::size_t>(stamp_indices_[0]);
      const auto ni = static_cast<std::size_t>(stamp_indices_[1]);
      if (si < data_.size() && ni < data_.size()) {
        stamp_ = rclcpp::Time(
          static_cast<int64_t>(data_[si]), static_cast<uint32_t>(data_[ni]),
          node_->get_clock()->get_clock_type());
        stamped = true;
      }
    }
    if (!stamped) {stamp_ = node_->now();}
    has_msg_ = true;
  }

  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

  std::string topic_;
  std::string mode_{"scalar"};
  int index_{0};
  std::vector<double> stamp_indices_;
  std::vector<std::string> order_;
  double timeout_{0.5};
  double input_min_{0.0};
  double input_max_{1.0};
  bool invert_{false};
  std::string on_missing_{"unavailable"};
  double default_score_{0.5};

  std::mutex mtx_;
  bool has_msg_{false};
  rclcpp::Time stamp_;
  std::vector<double> data_;
  std::vector<std::string> labels_;
};

// This one line binds the type string to the class. If you copied this file, change only this.
CO_DRIVER_REGISTER_SCORER(ExternalScoreScorer, "external_score")

}  // namespace co_driver
