// Corner hold - do not hand the car back to the map controller in the middle
// of a slow corner.
//
// Coming back from the reactive controller is a change of who is steering, and
// the worst moment to make one is mid-corner: the two controllers disagree
// most where the line curves, the car is already using its grip, and the
// handover blend spends 100 ms somewhere between the two commands. On a
// straight that costs nothing. In a hairpin it is the whole margin.
//
// Two questions, both asked of things already on hand:
//
//   IS THIS A CORNER?  The planned line carries its own speed - the producer
//   packs it into the z of each pose - so the plan itself says where it is
//   slow, and slow on a raceline means tight. That is a better answer than
//   anything derived from the car's own motion, which cannot tell a corner
//   from traffic, and better than curvature computed from the points, which is
//   noisy at the sampling this path comes at.
//
//   IS THE MAP CONTROLLER MID-TURN?  Its command says so directly. The drive
//   is publishing all along even while the reactive controller has the car, so
//   what is being tested is the command it is offering to take over WITH.
//
// Both have to be true. A slow section the map controller would drive straight
// through is not a corner it can get wrong, and a large steering angle where
// the plan is fast is the controller correcting on a straight.
//
// ONLY THE RETURN IS GATED. Whoever holds the car keeps it - this never takes
// the car away from the map controller, it only declines to give it back yet.
// Leaving is the other pathways' decision, exactly as in recovery_gate.
//
// That exemption is why this is a gate on the map controller rather than a
// weight on the reactive one, which reads more like what it is for. A weight
// would have to out-score the calibrated localization term - gap_obs sits at
// 0.3 - 2.0 = -1.7 with a clear path against pp_main's 3.0 when healthy, so
// roughly 5, larger than anything else in the system - and a score has no
// incumbency to exempt. The car would then leave the map controller for the
// reactive one in EVERY slow corner, detour or no detour, which is the
// opposite of holding a detour open. The veto form does nothing at all while
// the map controller has the car.
//
// The status line for it therefore lives on the map controller, which is also
// the only honest place for it: what is measured is ITS command. Wiring this
// input onto the reactive candidates too, so their status would say why they
// were being kept, was tried and reverted - score() reads the steering of the
// drive it is scoring, so on a gap candidate it reported the gap follower
// turning 0 degrees, which is true and completely beside the point.
//
// Wired like the other gates, weight 0 with veto_below on the map-based drive,
// so it costs the calibrated localization thresholds nothing:
//
//   {
//     "name": "corner", "type": "corner_hold",
//     "params": {"path_topic": "/global_path", "base_frame": "base_link",
//                "slow_below_mps": 3.0, "steering_above_deg": 15.0,
//                "clear_ms": 300}
//   }
//
// Only the release is timed. Engaging late is what this exists to prevent, so
// it engages on the first cycle both conditions hold; clear_ms then stops one
// frame of straightened steering from handing the car back inside the corner.
//
// Silence is not a veto. With no path, or no transform, this gate has no
// opinion and says so - a car whose planner has not started must still drive.
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include "co_driver/path_reference.hpp"
#include "co_driver/scorer.hpp"

namespace co_driver
{

class CornerHoldScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    path_topic_ = jstr(p, "path_topic", "/global_path");
    path_csv_ = jstr(p, "path_csv", "");
    path_frame_ = jstr(p, "path_frame", "map");
    base_frame_ = jstr(p, "base_frame", "base_link");
    slow_below_ = jnum(p, "slow_below_mps", 3.0);
    steer_above_ = jnum(p, "steering_above_deg", 15.0) * M_PI / 180.0;
    clear_ = jms(p, "clear_ms", 300.0);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    if (!path_topic_.empty()) {
      rclcpp::QoS qos(1);
      qos.reliable().transient_local();
      path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
        path_topic_, qos,
        [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          if (msg->poses.size() >= 2) {path_.fromMessage(*msg);}
        }, opts);
    }
    if (!path_csv_.empty()) {
      std::string err;
      if (!csv_path_.fromCsv(path_csv_, path_frame_, &err)) {
        RCLCPP_WARN(node->get_logger(), "corner hold '%s': %s", name.c_str(), err.c_str());
      }
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);

    RCLCPP_INFO(
      node->get_logger(),
      "corner hold '%s': the return waits while the plan here is below %.1fm/s "
      "AND the drive is asking for more than %.0f degrees, plus %.0fms after "
      "that stops being true",
      name.c_str(), slow_below_, steer_above_ * 180.0 / M_PI, clear_ * 1e3);
    return true;
  }

  // Where the car is on the plan, and how fast the plan is there. Drive
  // independent, so it is done once per cycle.
  void prepare(const Context & ctx, const std::vector<Drive> &) override
  {
    (void)ctx;
    std::lock_guard<std::mutex> lock(mtx_);
    have_ = false;
    const PathReference * path = !path_.empty() ? &path_ :
      (!csv_path_.empty() ? &csv_path_ : nullptr);
    if (!path || !tf_buffer_) {return;}

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(path->frame(), base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;                       // no pose: no opinion, see the header
    }
    const auto pr = path->project(
      tf.transform.translation.x, tf.transform.translation.y);
    if (!pr.valid) {return;}
    plan_speed_ = path->speedAt(pr.station);
    have_ = true;
  }

  ScoreResult score(const Drive & drive, const Context & ctx) override
  {
    // The incumbent is never gated: this declines to give the car back, it
    // does not take it away.
    if (!ctx.last_selected.empty() && ctx.last_selected == drive.name) {
      return ScoreResult::ok(1.0, "incumbent");
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (!have_) {
      return ScoreResult::ok(1.0, "no line to measure against");
    }
    const double steer = std::abs(drive.cmd.drive.steering_angle);
    const bool tight = std::isfinite(steer) &&
      plan_speed_ < slow_below_ && steer > steer_above_;

    State & st = state_[drive.name];
    if (tight) {
      st.holding = true;
      st.since = ctx.now;
      st.valid = true;
    } else if (st.holding && st.valid && (ctx.now - st.since).seconds() >= clear_) {
      st.holding = false;
    }

    char buf[112];
    if (st.holding) {
      const double waited = (st.valid && !tight) ? (ctx.now - st.since).seconds() : 0.0;
      if (tight) {
        std::snprintf(
          buf, sizeof(buf), "corner: plan %.1fm/s, asking %.0f deg - not handing back",
          plan_speed_, steer * 180.0 / M_PI);
      } else {
        std::snprintf(
          buf, sizeof(buf), "corner clearing: %.1f/%.1fs", waited, clear_);
      }
      return ScoreResult::ok(0.0, buf);
    }
    std::snprintf(
      buf, sizeof(buf), "clear: plan %.1fm/s, asking %.0f deg",
      plan_speed_, steer * 180.0 / M_PI);
    return ScoreResult::ok(1.0, buf);
  }

private:
  struct State
  {
    bool holding{false};
    bool valid{false};
    rclcpp::Time since;
  };

  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  PathReference path_, csv_path_;

  std::string path_topic_, path_csv_, path_frame_, base_frame_;
  double slow_below_{3.0};
  double steer_above_{15.0 * M_PI / 180.0};
  double clear_{0.3};

  std::mutex mtx_;
  bool have_{false};
  double plan_speed_{0.0};
  std::map<std::string, State> state_;
};

CO_DRIVER_REGISTER_SCORER(CornerHoldScorer, "corner_hold")

}  // namespace co_driver
