// Cross-track gate - is the car still anywhere near the line it is supposed to
// be following?
//
// The map-based controller steers toward a planned line. That is only a sane
// thing to do while the car is on or near that line. After a collision the car
// can end up somewhere the plan never goes - on the 0814 recording it hit an
// obstacle sitting on the racing line and carried on through where the map says
// there is wall - and from there "drive back to the raceline" is a command
// pointing across whatever it is now stuck against. The reactive controller,
// which only ever looks at what the lidar can see, is the one that can get out.
//
// This is deliberately NOT a localization check. Confidence can stay perfectly
// high while the car is somewhere it should not be: the pose is right, the
// scan matches, and the car is simply in the wrong place. Nothing else in the
// stack notices that.
//
// Wired the same way as the other gates - weight 0 with veto_below on the
// map-based drive - so it costs the calibrated thresholds nothing:
//
//   {
//     "name": "off_path", "type": "path_deviation",
//     "params": {"path_topic": "/global_path", "base_frame": "base_link",
//                "max_deviation_m": 1.5, "trip_ms": 400, "clear_ms": 800}
//   }
//
// Both edges are timed, for opposite reasons. trip_ms rejects a momentary
// excursion - a wide line through a corner is not being lost. clear_ms makes
// coming back mean something: one frame near the line while sliding across it
// is not recovery. Both are short, because unlike a localization failure there
// is nothing here that needs proving over seconds; the measurement is a
// distance and it is either small or it is not.
//
// Silence is not a veto. With no path, or no transform, this gate has no
// opinion at all and says so - a car whose planner has not started yet must
// still be able to drive.
#include <cmath>
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

class PathDeviationScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    path_topic_ = jstr(p, "path_topic", "/global_path");
    const bool path_transient_local = jbool(p, "path_transient_local", true);
    path_csv_ = jstr(p, "path_csv", "");
    path_frame_ = jstr(p, "path_frame", "map");
    base_frame_ = jstr(p, "base_frame", "base_link");
    max_dev_ = jnum(p, "max_deviation_m", 1.5);
    trip_ = jms(p, "trip_ms", 400.0);
    clear_ = jms(p, "clear_ms", 800.0);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    if (!path_topic_.empty()) {
      rclcpp::QoS qos(1);
      qos.reliable();
      if (path_transient_local) {
        qos.transient_local();
      } else {
        qos.durability_volatile();
      }
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
        RCLCPP_WARN(node->get_logger(), "path deviation '%s': %s", name.c_str(), err.c_str());
      }
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);

    RCLCPP_INFO(
      node->get_logger(),
      "path deviation '%s': more than %.2fm off the line for %.0fms disqualifies; "
      "back within it for %.0fms clears",
      name.c_str(), max_dev_, trip_ * 1e3, clear_ * 1e3);
    return true;
  }

  void prepare(const Context & ctx, const std::vector<Drive> &) override
  {
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
    deviation_ = pr.lateral;
    have_ = true;

    const bool off = deviation_ > max_dev_;
    if (off != was_off_ || !edge_valid_) {
      edge_ = ctx.now;
      was_off_ = off;
      edge_valid_ = true;
    }
    const double dwell = (ctx.now - edge_).seconds();
    dwell_ = dwell;
    if (off && dwell >= trip_) {tripped_ = true;}
    if (!off && tripped_ && dwell >= clear_) {tripped_ = false;}
  }

  ScoreResult score(const Drive &, const Context &) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    char buf[96];
    if (!have_) {
      return ScoreResult::ok(1.0, "no line to measure against");
    }
    if (tripped_) {
      if (was_off_) {
        std::snprintf(buf, sizeof(buf), "off the line: %.2fm for %.1fs", deviation_, dwell_);
      } else {
        std::snprintf(buf, sizeof(buf), "returning: %.2fm, %.1f/%.1fs", deviation_, dwell_, clear_);
      }
      return ScoreResult::ok(0.0, buf);
    }
    if (was_off_) {
      std::snprintf(
        buf, sizeof(buf), "off the line: %.2fm, %.1f/%.1fs", deviation_, dwell_, trip_);
      return ScoreResult::ok(1.0, buf);
    }
    std::snprintf(buf, sizeof(buf), "on the line: %.2fm", deviation_);
    return ScoreResult::ok(1.0, buf);
  }

private:
  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  PathReference path_, csv_path_;

  std::string path_topic_, path_csv_, path_frame_, base_frame_;
  double max_dev_{1.5};
  double trip_{0.4};
  double clear_{0.8};

  std::mutex mtx_;
  bool have_{false};
  double deviation_{0.0};
  double dwell_{0.0};
  bool was_off_{false};
  bool tripped_{false};
  bool edge_valid_{false};
  rclcpp::Time edge_;
};

CO_DRIVER_REGISTER_SCORER(PathDeviationScorer, "path_deviation")

}  // namespace co_driver
