// Obstacle avoidance - hand the car to the fallback while an obstacle sits on
// the commanded path, and give it back once the path is clear again.
//
// This is a detour, not a loss of trust - which is what separates it from
// path_clearance. There the drive was steering into structure, meaning its
// world model was wrong; here the world model is fine and something is simply
// lying in the way. So the drive gets its job back as soon as the path is
// clear, with only enough hysteresis to not flicker, while path_clearance
// makes it wait several times longer.
//
// The evidence differs too. path_clearance reads the raw scan and cannot tell
// a wall from a cone; this reads the detector's clustered output, which drops
// anything wider than 3 m.
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
// HOW THE DEMAND IS GRADED
//
//   demand = urgency x confidence,   score = 1 - max(demand over clusters)
//
//   urgency     how soon the arc reaches it, measured in TIME rather than
//               metres, so speed decides what counts as close. Full inside
//               full_urgency_time seconds of travel, falling to zero at the
//               trigger distance. Seeing an obstacle is not a reason to hand
//               over; being about to arrive at one is.
//
//               Clusters the detector labels DYNAMIC get a shorter full-urgency
//               window (dynamic_urgency_scale). The label is velocity-derived
//               and unreliable in general, but it only ever appears when the
//               relative speed is high, and something moving fast may not be
//               there when we arrive - so the response is to commit later, not
//               to trust the label. Using it only to become MORE conservative
//               about detouring means a wrong label costs nothing.
//   NOT MEASURED is not the same as NOT TRUSTED. The detector derives
//               max_dense_weight, observed_count, is_wall_static and track_id
//               from an odom transform and zeroes all of them when odom is
//               unavailable, so a zero here means "could not measure", not
//               "not solid". An all-zero trust signature therefore falls back
//               to judging that cluster on geometry alone.
//
//               This is narrow insurance, not a critical path. Losing odom
//               also collapses the localization confidence, which takes the
//               map-based drive out through its own pathway long before this
//               matters - the arbitration is already on the fallback. The one
//               case it covers is the detector losing odom on its own while
//               localization stays healthy, where the drive is still selected
//               and only obstacle avoidance would have gone quiet.
//
//   confidence  is this a real, solid thing? max_dense_weight is a spatial
//               accumulation over the last ~7 frames, normalised so it does
//               not vary with range, and independent of the velocity
//               estimates. Low values are transient or noise clusters - the
//               detector measured one spurious corridor intrusion at
//               max_dense_weight 0.0 with 7 points. observed_count adds a
//               minimum maturity so a single-frame flicker cannot demand a
//               handover.
//
// What is deliberately NOT in the grade:
//
//   risk_weight            it is a function of motion_label alone
//                          (DYNAMIC 1.5 / UNKNOWN 1.3 / STATIC 1.0), and
//                          motion_label is velocity-derived. On the car,
//                          where DYNAMIC is overwritten, it collapses to a
//                          proxy for observed_count < 5. Measured separation
//                          between real objects and structure: AUC 0.536,
//                          i.e. none.
//   ego_motion_gate_active always false - the IMU motion gate is disabled in
//                          the detector's config. AUC 0.500.
//   any wall-versus-obstacle grading
//                          no published field separates them. The best is
//                          max_dense_weight at AUC 0.724, and it points the
//                          wrong way (structure scores higher, being larger
//                          and more persistent). So it is used as trust, not
//                          as threat, and reacting to a wall is accepted.
//   point_count raw        inversely related to range, so it mixes size with
//                          distance; the real objects here are 9-25 points
//                          and the wall that used to trigger was 280.
//   hit counts             derived from the velocity estimate.
//
// This node does not detect obstacles. It receives them.
//
// Everything about what is or is not an obstacle - clustering, the 3 m width
// drop, the wall label - is the detector's judgement and arrives on its topic.
// The only decision made here is geometric: does that obstacle sit on the path
// this drive is commanding. Nothing in this file re-classifies a cluster.
//
// max_width_m exists as a blunt cap for stacks whose detector does no size
// filtering of its own, and is off in the red_damvi configuration. Using it to
// separate walls from obstacles was tried and abandoned: across five datasets
// real objects and sub-1 m structure share a width distribution identical to
// two decimals, and a 0.6 m cut missed 45% of confirmed obstacles while still
// passing 82% of the structure it was aimed at. That judgement does not live
// here.
//
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
    full_urgency_time_ = jnum(p, "full_urgency_time", 0.8);
    min_full_urgency_ = jnum(p, "min_full_urgency_m", 0.3);
    dynamic_scale_ = jnum(p, "dynamic_urgency_scale", 0.6);
    dynamic_delay_max_ = jnum(p, "dynamic_delay_max_m", 0.5);
    dense_full_ = jnum(p, "dense_full", 20.0);
    min_observed_ = jint(p, "min_observed", 2);
    min_trigger_ = jnum(p, "min_trigger_m", 1.0);
    max_trigger_ = jnum(p, "max_trigger_m", 4.0);
    max_sweep_ = jnum(p, "max_sweep_deg", 90.0) * M_PI / 180.0;
    min_speed_ = jnum(p, "min_speed", 0.2);
    ignore_wall_static_ = jbool(p, "ignore_wall_static", true);
    max_width_ = jnum(p, "max_width_m", 0.0);        // 0 = no size filter
    min_points_ = jint(p, "min_point_count", 0);
    sample_step_ = std::max(0.02, jnum(p, "sample_step_m", 0.08));
    block_ = jms(p, "block_ms", 200.0);
    // Short on purpose: nothing was wrong with the drive, the path was simply
    // occupied. Compare path_clearance, which is reluctant by design.
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
      // Reported as "clear", never as unavailable. This input carries a real
      // weight, and missing:mask rescales the logit by the weights that were
      // usable - so dropping out would silently move the calibrated
      // localization thresholds every time the detector is not running.
      if (!clusters_) {
        return ScoreResult::ok(1.0, "no clusters on " + topic_);
      }
      if (timeout_ > 0.0 && (ctx.now - rx_).seconds() > timeout_) {
        return ScoreResult::ok(1.0, "stale clusters");
      }
      clusters = clusters_;
    }

    const double v = drive.cmd.drive.speed;
    const double delta = drive.cmd.drive.steering_angle;
    if (!std::isfinite(v) || !std::isfinite(delta)) {
      return ScoreResult::unavailable("command is NaN/inf");
    }
    if (std::abs(v) < min_speed_) {
      return setState(drive.name, ctx, 0.0, 0.0, 0, "below min_speed");
    }

    // How far along the commanded path an obstacle still matters.
    const double trigger = std::clamp(
      trigger_time_ * std::abs(v), min_trigger_, max_trigger_);
    const auto arc = ArcProjection::fromCommand(delta, wheelbase_, trigger, max_sweep_);

    double demand = 0.0;
    double nearest = std::numeric_limits<double>::infinity();
    int considered = 0;
    int unmeasured = 0;
    for (const auto & c : clusters->clusters) {
      if (ignore_wall_static_ && c.is_wall_static) {continue;}
      if (max_width_ > 0.0 && c.width > max_width_) {continue;}
      if (min_points_ > 0 && static_cast<int>(c.point_count) < min_points_) {continue;}
      ++considered;
      const double along = nearestOnArc(arc, c);
      if (!std::isfinite(along)) {continue;}          // not on the path
      nearest = std::min(nearest, along);

      // Urgency in time-of-travel, so the same obstacle is urgent sooner when
      // moving quickly. A DYNAMIC label shortens the window: commit later.
      double full_dist = full_urgency_time_ * std::abs(v);
      if (c.motion_label == 2) {                                 // DYNAMIC
        // Bounded in metres, not just scaled. The detector's false-DYNAMIC
        // rate rises with ego speed (5% below 1 m/s, 31% at 5-6 m/s) while the
        // cost of committing late rises with speed too - a 0.2 s delay is 1 m
        // at 5 m/s. Scaling alone lets both grow together; this caps how much
        // ground the label can ever cost.
        full_dist = std::max(
          full_dist * dynamic_scale_, full_dist - dynamic_delay_max_);
      }
      full_dist = std::clamp(full_dist, min_full_urgency_, trigger * 0.9);
      const double span = std::max(1e-3, trigger - full_dist);
      const double urgency = std::clamp((trigger - along) / span, 0.0, 1.0);

      // Confidence: a real solid return, not a flicker - unless the detector
      // could not measure it at all, in which case fall back to geometry.
      const bool trust_unmeasured =
        c.max_dense_weight == 0.0f && c.observed_count == 0u && c.track_id == 0u;
      double confidence = 1.0;
      if (!trust_unmeasured) {
        confidence = (dense_full_ > 0.0) ?
          std::clamp(static_cast<double>(c.max_dense_weight) / dense_full_, 0.0, 1.0) : 1.0;
        if (min_observed_ > 0 && static_cast<int>(c.observed_count) < min_observed_) {
          confidence = 0.0;
        }
      } else {
        ++unmeasured;
      }
      demand = std::max(demand, urgency * confidence);
    }
    if (unmeasured > 0 && considered > 0) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "obstacle trust fields unmeasured on %d/%d clusters (odom lost?) - "
        "judging on geometry alone", unmeasured, considered);
    }
    return setState(drive.name, ctx, demand, nearest, considered, "");
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
  // Hysteresis on the *presence* of a demand, so a flicker cannot start or end
  // a detour, while the demand itself passes through graded once established.
  ScoreResult setState(
    const std::string & drive, const Context & ctx, double demand, double dist,
    int considered, const std::string & reason)
  {
    const bool clear_now = demand <= 0.0;
    State & st = state_[drive];
    if (clear_now != st.was_clear || !st.valid) {
      st.edge = ctx.now;
      st.was_clear = clear_now;
      st.valid = true;
    }
    const double dwell = (ctx.now - st.edge).seconds();
    if (!clear_now && dwell >= block_) {st.blocked = true;}
    if (clear_now && st.blocked && dwell >= clear_) {st.blocked = false;}

    char buf[120];
    if (st.blocked) {
      if (clear_now) {
        std::snprintf(buf, sizeof(buf), "path clearing, holding %.1f/%.1fs", dwell, clear_);
        return ScoreResult::ok(0.0, buf);        // stay committed to the detour
      }
      std::snprintf(
        buf, sizeof(buf), "obstacle at %.2fm, demand %.2f", dist, demand);
      return ScoreResult::ok(std::clamp(1.0 - demand, 0.0, 1.0), buf);
    }
    if (!clear_now) {
      std::snprintf(
        buf, sizeof(buf), "obstacle at %.2fm, demand %.2f, confirming %.1f/%.1fs",
        dist, demand, dwell, block_);
      return ScoreResult::ok(1.0, buf);          // not confirmed yet
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
  double full_urgency_time_{0.8};
  double min_full_urgency_{0.3};
  double dynamic_scale_{0.6};
  double dynamic_delay_max_{0.5};
  double dense_full_{20.0};
  int min_observed_{2};
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
