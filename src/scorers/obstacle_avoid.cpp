// Obstacle avoidance - hand the car to the fallback while an obstacle sits on
// the commanded path, and give it back once the path is clear again.
//
// Same question as path_clearance, different evidence. path_clearance reads the
// raw scan and so cannot tell a wall from a cone; this reads the obstacle
// detector's clustered output, which drops anything wider than 3 m and labels
// what it believes to be wall. That discrimination is the entire reason to
// have both.
//
// ---------------------------------------------------------------------------
// What this trusts, and what it deliberately ignores
//
// The detector's own measurements say the cluster geometry is solid (0.77 cm
// inter-scan registration residual) and the motion estimates are not: while
// driving, static objects are reported at 1-4 m/s (max 28.7), because the
// frame interval is 25 ms and the estimate is centroid-to-centroid. So:
//
//   used     center_x/y, min_x/y, max_x/y, point_count, header.stamp
//   ignored  speed, velocity_x/y, compensated_displacement, risk_weight
//   ignored  motion_label - and this one is deliberate in a second way: a
//            newly appeared cluster is UNKNOWN for its first 5 frames, which
//            is exactly when reacting matters, so filtering on the label would
//            build in a 125 ms blind spot.
//
// width is not used either. It is the bounding-box diagonal, so a thin cluster
// lying along the line of sight reports its depth as "width" while occupying
// almost nothing laterally. The bounding box is used directly instead.
//
// ---------------------------------------------------------------------------
// Two size filters, because is_wall_static alone is not enough
//
// The detector only considers a cluster for the wall label at 1.0 m and wider,
// so a fragment narrower than that arrives as an ordinary obstacle. That gap
// is not hypothetical - it is what this scorer actually detected on its first
// pass over the 0813 recording: two handovers on a 0.97 m, 280-point structure
// beside the car, and none at all on the three real objects, which are
// 0.13-0.20 m wide and 9-10 points. Exactly backwards.
//
// max_width_m cuts obvious structure, but it is a safety cap and not a
// discriminator, because no width threshold is one. Measured across five
// datasets, real foreign objects and sub-1 m mapped structure have the same
// width distribution to two decimals (p50 0.37, p75 0.55 for both); a 0.6 m
// cut misses 45% of confirmed obstacles while still passing 82% of the
// structure it was aimed at. Distance-along-the-path does not separate them
// either - tested, and it removes the real objects while keeping the wall.
//
// So the residual is accepted rather than filtered away: near a wall closer
// than the corridor is wide, this scorer will occasionally hand over when it
// did not need to. That is the safe direction. lateral_margin_m is the knob
// for trading it off per track; the width cap is not.
//
// The trigger is a distance along the commanded path, not a braking margin:
// an obstacle anywhere on the path within trigger_time seconds of travel is
// reason to go around it. That is the difference from path_clearance, which
// asks the last-moment question of whether there is room to keep going.
//
//   {
//     "name": "obstacles", "type": "obstacle_avoid",
//     "params": {"topic": "/obstacle_clusters", "trigger_time": 1.5}
//   }
//   with, on the map-based drive only:
//     "obstacles": {"weight": 0.0, "veto_below": 0.5}
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <obstacle_context_msgs/msg/obstacle_cluster_array.hpp>

#include "co_driver/arc_geometry.hpp"
#include "co_driver/scorer.hpp"

namespace co_driver
{

class ObstacleAvoidScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    topic_ = jstr(p, "topic", "/obstacle_clusters");
    wheelbase_ = jnum(p, "wheelbase", 0.324);
    half_width_ = jnum(p, "vehicle_half_width", 0.18);
    margin_ = jnum(p, "lateral_margin_m", 0.10);
    trigger_time_ = jnum(p, "trigger_time", 1.5);
    min_trigger_ = jnum(p, "min_trigger_m", 1.0);
    max_trigger_ = jnum(p, "max_trigger_m", 4.0);
    max_sweep_ = jnum(p, "max_sweep_deg", 90.0) * M_PI / 180.0;
    min_speed_ = jnum(p, "min_speed", 0.2);
    ignore_wall_static_ = jbool(p, "ignore_wall_static", true);
    max_width_ = jnum(p, "max_width_m", 0.0);        // 0 = no size filter
    min_points_ = jint(p, "min_point_count", 0);
    sample_step_ = std::max(0.02, jnum(p, "sample_step_m", 0.08));
    block_ = jms(p, "block_ms", 200.0);
    clear_ = jms(p, "clear_ms", 700.0);
    timeout_ = jms(p, "timeout_ms", 500.0);

    // The detector publishes SensorDataQoS; a RELIABLE subscription would not
    // match it and would receive nothing at all.
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    sub_ = node->create_subscription<obstacle_context_msgs::msg::ObstacleClusterArray>(
      topic_, rclcpp::SensorDataQoS(),
      [this](const obstacle_context_msgs::msg::ObstacleClusterArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        clusters_ = msg;
        rx_ = node_->now();
      }, opts);

    RCLCPP_INFO(
      node->get_logger(),
      "obstacle avoidance '%s' <- %s (%.1fs of travel ahead, sustained %.0fms%s)",
      name.c_str(), topic_.c_str(), trigger_time_, block_ * 1e3,
      ignore_wall_static_ ? ", wall-labelled clusters ignored" : "");
    if (max_width_ > 0.0) {
      RCLCPP_INFO(
        node->get_logger(), "obstacle avoidance '%s': clusters wider than %.2fm "
        "are treated as structure", name.c_str(), max_width_);
    }
    return true;
  }

  ScoreResult score(const Drive & drive, const Context & ctx) override
  {
    obstacle_context_msgs::msg::ObstacleClusterArray::ConstSharedPtr clusters;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!clusters_) {return ScoreResult::unavailable("no clusters on " + topic_);}
      if (timeout_ > 0.0 && (ctx.now - rx_).seconds() > timeout_) {
        return ScoreResult::unavailable("stale clusters");
      }
      clusters = clusters_;
    }

    const double v = drive.cmd.drive.speed;
    const double delta = drive.cmd.drive.steering_angle;
    if (!std::isfinite(v) || !std::isfinite(delta)) {
      return ScoreResult::unavailable("command is NaN/inf");
    }
    if (std::abs(v) < min_speed_) {
      return setState(drive.name, ctx, true, 0.0, 0, "below min_speed");
    }

    // How far along the commanded path an obstacle still matters.
    const double trigger = std::clamp(
      trigger_time_ * std::abs(v), min_trigger_, max_trigger_);
    const auto arc = ArcProjection::fromCommand(delta, wheelbase_, trigger, max_sweep_);

    double nearest = std::numeric_limits<double>::infinity();
    int considered = 0;
    for (const auto & c : clusters->clusters) {
      if (ignore_wall_static_ && c.is_wall_static) {continue;}
      // Wider than this is structure, not something to steer around.
      if (max_width_ > 0.0 && c.width > max_width_) {continue;}
      if (min_points_ > 0 && static_cast<int>(c.point_count) < min_points_) {continue;}
      ++considered;
      const double d = nearestOnArc(arc, c);
      nearest = std::min(nearest, d);
    }
    // Anything the projection returned is already within the trigger distance.
    const bool clear_now = !std::isfinite(nearest);
    return setState(drive.name, ctx, clear_now, nearest, considered, "");
  }

private:
  // Closest approach of a cluster's footprint to the commanded arc, as an arc
  // length. The bounding box is walked rather than reduced to its centre: a
  // corridor 0.36 m wide cannot cross a box without touching its perimeter,
  // and a box entirely inside the corridor is caught by its corners.
  double nearestOnArc(
    const ArcProjection & arc, const obstacle_context_msgs::msg::ObstacleCluster & c) const
  {
    const double x0 = std::min(c.min_x, c.max_x);
    const double x1 = std::max(c.min_x, c.max_x);
    const double y0 = std::min(c.min_y, c.max_y);
    const double y1 = std::max(c.min_y, c.max_y);
    const double limit = half_width_ + margin_;

    double best = std::numeric_limits<double>::infinity();
    auto test = [&](double px, double py) {
        double along, lateral;
        if (!arc.project(px, py, &along, &lateral)) {return;}
        if (lateral <= limit && along < best) {best = along;}
      };

    test(c.center_x, c.center_y);
    const int nx = static_cast<int>(std::ceil((x1 - x0) / sample_step_));
    const int ny = static_cast<int>(std::ceil((y1 - y0) / sample_step_));
    for (int i = 0; i <= nx; ++i) {
      const double x = (nx == 0) ? x0 : x0 + (x1 - x0) * static_cast<double>(i) / nx;
      test(x, y0);
      test(x, y1);
    }
    for (int j = 0; j <= ny; ++j) {
      const double y = (ny == 0) ? y0 : y0 + (y1 - y0) * static_cast<double>(j) / ny;
      test(x0, y);
      test(x1, y);
    }
    return best;
  }

  // Per-drive hysteresis, one edge timestamp each, stamped when the direction
  // changes so nothing is read before it has been assigned.
  ScoreResult setState(
    const std::string & drive, const Context & ctx, bool clear_now, double dist,
    int considered, const std::string & reason)
  {
    State & st = state_[drive];
    if (clear_now != st.was_clear || !st.valid) {
      st.edge = ctx.now;
      st.was_clear = clear_now;
      st.valid = true;
    }
    const double dwell = (ctx.now - st.edge).seconds();
    if (!clear_now && dwell >= block_) {st.blocked = true;}
    if (clear_now && st.blocked && dwell >= clear_) {st.blocked = false;}

    char buf[110];
    if (st.blocked) {
      if (clear_now) {
        std::snprintf(buf, sizeof(buf), "obstacle cleared, waiting %.1f/%.1fs", dwell, clear_);
      } else {
        std::snprintf(buf, sizeof(buf), "obstacle on path at %.2fm", dist);
      }
      return ScoreResult::ok(0.0, buf);
    }
    if (!clear_now) {
      std::snprintf(buf, sizeof(buf), "obstacle closing: %.2fm %.1f/%.1fs", dist, dwell, block_);
      return ScoreResult::ok(1.0, buf);
    }
    if (!reason.empty()) {return ScoreResult::ok(1.0, reason);}
    std::snprintf(buf, sizeof(buf), "clear (%d clusters considered)", considered);
    return ScoreResult::ok(1.0, buf);
  }

  struct State
  {
    bool was_clear{true};
    bool valid{false};
    bool blocked{false};
    rclcpp::Time edge;
  };

  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<obstacle_context_msgs::msg::ObstacleClusterArray>::SharedPtr sub_;

  std::string topic_;
  double wheelbase_{0.324};
  double half_width_{0.18};
  double margin_{0.10};
  double trigger_time_{1.5};
  double min_trigger_{1.0};
  double max_trigger_{4.0};
  double max_sweep_{M_PI / 2.0};
  double min_speed_{0.2};
  bool ignore_wall_static_{true};
  double max_width_{0.0};
  int min_points_{0};
  double sample_step_{0.08};
  double block_{0.2};
  double clear_{0.7};
  double timeout_{0.5};

  std::mutex mtx_;
  obstacle_context_msgs::msg::ObstacleClusterArray::ConstSharedPtr clusters_;
  rclcpp::Time rx_;
  std::map<std::string, State> state_;
};

CO_DRIVER_REGISTER_SCORER(ObstacleAvoidScorer, "obstacle_avoid")

}  // namespace co_driver
