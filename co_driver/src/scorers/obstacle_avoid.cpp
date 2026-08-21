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

#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>

#include "co_driver/arc_geometry.hpp"
#include "co_driver/path_reference.hpp"
#include <nav_msgs/msg/odometry.hpp>

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
    // What a cluster is worth before its track has matured. 1.0 disables the
    // discount entirely; 0.0 restores the old behaviour of erasing it.
    immature_scale_ = std::clamp(jnum(p, "immature_scale", 1.0), 0.0, 1.0);
    min_trigger_ = jnum(p, "min_trigger_m", 1.0);
    max_trigger_ = jnum(p, "max_trigger_m", 4.0);
    max_sweep_ = jnum(p, "max_sweep_deg", 90.0) * M_PI / 180.0;
    min_speed_ = jnum(p, "min_speed", 0.2);
    ignore_wall_static_ = jbool(p, "ignore_wall_static", true);
    max_width_ = jnum(p, "max_width_m", 0.0);        // 0 = no size filter
    min_points_ = jint(p, "min_point_count", 0);
    sample_step_ = std::max(0.02, jnum(p, "sample_step_m", 0.08));
    // ---- planned path -----------------------------------------------------
    // Judging against the planned line rather than the instantaneous steering
    // angle. See path_reference.hpp for why the command is the wrong question.
    path_topic_ = jstr(p, "path_topic", "/global_path");
    path_csv_ = jstr(p, "path_csv", "");
    path_frame_ = jstr(p, "path_frame", "map");
    base_frame_ = jstr(p, "base_frame", "base_link");
    tf_timeout_ = jms(p, "tf_timeout_ms", 200.0);
    use_path_speed_ = jbool(p, "use_path_speed", true);

    // ---- how late we may commit -------------------------------------------
    // The commit distance is derived, not tuned: it is the shortest distance in
    // which the fallback can still get around the obstacle. Staying on the
    // faster map-based controller until exactly that point is the whole point.
    //
    //   commit = speed x (reaction + sqrt(2 x lateral_need / lateral_accel))
    //
    // reaction covers detection confirmation plus the handover blend; the
    // square root is the time to translate sideways far enough to miss.
    // Speed for the geometry: MEASURED, not commanded.
    //
    // Every distance here is derived from speed, so the source matters. On the
    // 0814 recording, differentiating the map-frame pose gives the truth, and:
    //     pose / odom        = 1.00   (p10 0.99, p90 1.04)  <- odom is correct
    //     pose / odom_wheel  = 2.66                          <- wheel is slow
    //     pose / drive_main  = 2.24, spread p10 1.15 to p90 3.50
    // The command is not a scaled version of the truth, it is a request the
    // car tracks loosely, so no constant factor can repair it - at p99 the car
    // was doing 5.73 m/s while commanding 2.06. Believing the command made the
    // lookahead short by that much, which is the "it switches far too late"
    // behaviour. Note /odom_wheel is the 2.6x-slow one; /odom is already
    // corrected and is what this uses.
    //
    // The command remains the fallback for when odometry is silent, because a
    // wrong lookahead beats none, and the note says which one is in use.
    speed_topic_ = jstr(p, "speed_topic", "/odom");
    speed_timeout_ = jms(p, "speed_timeout_ms", 300.0);
    reaction_time_ = jnum(p, "reaction_time", 0.5);
    lateral_accel_ = std::max(0.5, jnum(p, "lateral_accel", 5.0));
    commit_margin_ = jnum(p, "commit_margin_m", 0.5);
    // The demand at which the arbitration disqualifies the map controller.
    // Must match veto_below on that drive (score = 1 - demand), or the commit
    // distance stops being the distance the car actually commits at.
    commit_demand_ = std::clamp(jnum(p, "commit_demand", 0.85), 0.05, 1.0);
    // Hysteresis on the commitment itself. Once the detour is on, it stays on
    // until the demand has fallen well below the level that started it - not
    // merely back under it. Without this the demand crossing its threshold
    // twice near one obstacle produces two handovers, and on the 0814 drive
    // that came to 140 of them in 200 s. This is where the damping belongs:
    // the arbitration's switch_cooldown used to supply it by accident, at the
    // cost of also delaying every return.
    release_demand_ = std::clamp(
      jnum(p, "release_demand", 0.6), 0.0, commit_demand_);

    // ---- measurement jitter ------------------------------------------------
    // A cluster's reported position wanders, and it wanders more the further
    // away it is (fewer beams on it, larger angular quantisation) and the
    // faster we are going (more ego motion between the scan and the decision).
    // Rather than pretend the lateral offset is exact, the on-path test is
    // softened by this spread, so a cluster near the corridor edge contributes
    // partial demand instead of flickering in and out of it.
    jitter_base_ = jnum(p, "jitter_base_m", 0.10);
    jitter_per_m_ = jnum(p, "jitter_per_m", 0.03);
    jitter_per_speed_ = jnum(p, "jitter_per_mps", 0.04);

    block_ = jms(p, "block_ms", 200.0);
    // Once the obstacle's station is behind us it is passed, and that is a
    // fact about geometry rather than a timeout. Returning then is quick.
    passed_clear_ = jms(p, "passed_clear_ms", 250.0);
    // Longest a detour may be held on memory alone, with nothing currently
    // detected and the obstacle's place on the plan still ahead.
    remember_cap_ = jms(p, "remember_cap_ms", 0.0);
    // Short on purpose: nothing was wrong with the drive, the path was simply
    // occupied. Compare path_clearance, which is reluctant by design.
    clear_ = jms(p, "clear_ms", 700.0);
    timeout_ = jms(p, "timeout_ms", 500.0);

    // The detector publishes SensorDataQoS; a RELIABLE subscription would not
    // match it and would receive nothing at all.
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    if (!speed_topic_.empty()) {
      speed_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        speed_topic_, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          measured_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
          speed_stamp_ = node_->now();
          has_speed_ = true;
        }, opts);
    }
    sub_ = node->create_subscription<obstacle_context_msgs::msg::ObstacleClusterArray>(
      topic_, rclcpp::SensorDataQoS(),
      [this](const obstacle_context_msgs::msg::ObstacleClusterArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        clusters_ = msg;
        rx_ = node_->now();
      }, opts);

    // The path is latched, so a plain subscription would miss the only message
    // it will ever send.
    if (!path_topic_.empty()) {
      rclcpp::QoS qos(1);
      qos.reliable().transient_local();
      path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
        path_topic_, qos,
        [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          if (msg->poses.size() < 2) {return;}
          path_.fromMessage(*msg);
          RCLCPP_INFO(
            node_->get_logger(), "obstacle avoidance: path from %s - %zu points, "
            "%.1fm%s, frame %s", path_topic_.c_str(), path_.size(), path_.length(),
            path_.closed() ? " (closed)" : "", path_.frame().c_str());
        }, opts);
    }
    // CSV is the standby, loaded now so a missing file is a startup error
    // rather than a surprise the first time the topic is late.
    if (!path_csv_.empty()) {
      std::string err;
      if (csv_path_.fromCsv(path_csv_, path_frame_, &err)) {
        RCLCPP_INFO(
          node->get_logger(), "obstacle avoidance: standby path from %s - %zu points, "
          "%.1fm%s", path_csv_.c_str(), csv_path_.size(), csv_path_.length(),
          csv_path_.closed() ? " (closed)" : "");
      } else {
        RCLCPP_WARN(node->get_logger(), "obstacle avoidance: %s", err.c_str());
      }
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);

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

    // Which line are we judging against: the plan if we have it and can place
    // ourselves on it, the commanded arc if not. Falling back is loud - a
    // scorer that quietly answers "clear" because it lost its reference is
    // exactly the failure mode that made path_clearance untrustworthy.
    PathRef ref;
    const bool on_plan = buildPathRef(clusters->header, &ref);

    // Speed that decides the geometry: the one the car is ACTUALLY carrying.
    //
    // The plan's speed profile is tempting here - it is right there in the z
    // coordinate - but it answers the wrong question. What sets both the time
    // to arrival and the room the manoeuvre needs is how fast the car is
    // going, not how fast the plan wishes it were. Measured on the 0814
    // recording, where the car was driven at 0.85 m/s along a line planned for
    // up to 8.0 m/s: taking the planned speed put the commit distance at
    // 6.75 m instead of 1.38 m, so objects five metres away all arrived at
    // full demand, pp_main held only 48.6% of the run against the 68.6% the
    // car itself managed, and the handovers went from 38 to 60.
    //
    // Nothing is lost by using the current speed. This is re-evaluated at
    // 50 Hz, so a car that accelerates gets the longer lookahead immediately -
    // long before it arrives anywhere. A crawling car genuinely does not need
    // to worry about something eight metres away.
    bool speed_from_odom = false;
    const double v_now = geometrySpeed(v, ctx, &speed_from_odom);
    const double v_plan = v_now;
    const double v_planned_here = (on_plan && use_path_speed_) ?
      ref.path->speedAt(ref.ego_station) : 0.0;
    (void)v_planned_here;

    const double trigger = std::clamp(
      trigger_time_ * v_plan, min_trigger_, max_trigger_);

    // Commit distance: the last point at which the fallback can still get
    // around. Derived from the manoeuvre, not tuned.
    const double lateral_need = half_width_ + margin_;
    const double commit_raw = v_plan *
      (reaction_time_ + std::sqrt(2.0 * lateral_need / lateral_accel_)) + commit_margin_;
    const double commit = std::clamp(
      std::max(commit_raw, min_full_urgency_), min_full_urgency_, trigger * 0.9);

    const auto arc = ArcProjection::fromCommand(delta, wheelbase_, trigger, max_sweep_);

    double demand = 0.0;
    double nearest = std::numeric_limits<double>::infinity();
    int considered = 0;
    int unmeasured = 0;
    bool any_ahead = false;
    for (const auto & c : clusters->clusters) {
      if (ignore_wall_static_ && c.is_wall_static) {continue;}
      if (max_width_ > 0.0 && c.width > max_width_) {continue;}
      if (min_points_ > 0 && static_cast<int>(c.point_count) < min_points_) {continue;}
      ++considered;

      double along = std::numeric_limits<double>::quiet_NaN();
      double on_path = 0.0;
      const double range = std::hypot(c.center_x, c.center_y);
      // How much the reported position can be expected to wander. Grows with
      // range (fewer beams, coarser angular quantisation) and with speed (more
      // ego motion between the scan and this decision).
      const double jitter =
        jitter_base_ + jitter_per_m_ * range + jitter_per_speed_ * v_now;

      if (on_plan) {
        double station = 0.0, lateral = 0.0;
        if (!nearestOnPath(ref, c, &station, &lateral)) {continue;}
        station_of_max_ = station;
        along = ref.path->forwardDistance(ref.ego_station, station);
        if (along < 0.0) {
          // Already passed. Recorded, because "has it gone by" is what makes
          // the return prompt rather than a timeout.
          continue;
        }
        // Graded rather than a hard edge: at the corridor edge the answer is
        // genuinely uncertain, and a hard cut there turns jitter into flapping.
        on_path = std::clamp(
          (lateral_need + jitter - lateral) / std::max(1e-3, 2.0 * jitter), 0.0, 1.0);
      } else {
        along = nearestOnArc(arc, c);
        if (!std::isfinite(along)) {continue;}
        on_path = 1.0;
      }
      if (on_path <= 0.0 || along > trigger) {continue;}
      any_ahead = true;
      nearest = std::min(nearest, along);

      // DYNAMIC shortens the range at which we start paying attention, so
      // something that may have moved on by the time we arrive is noticed
      // later. It deliberately does NOT move the commit distance: that is the
      // last point at which the fallback can still get around, and deferring
      // past it does not make the car patient, it makes the obstacle
      // unavoidable. Reacting late is a choice; arriving late is not.
      double reach = trigger;
      if (c.motion_label == 2) {
        reach = std::max(
          std::max(trigger * dynamic_scale_, trigger - dynamic_delay_max_),
          commit + 0.1);
      }
      if (along > reach) {continue;}
      // Scaled so the demand reaches the veto threshold EXACTLY at the commit
      // distance. Without this the veto fires wherever the threshold happens
      // to land on the ramp - always earlier - and the car leaves the fast
      // controller sooner than it has to. Staying on it until the last
      // avoidable moment is the requirement, so the geometry has to be what
      // decides, not an arbitrary point on a slope.
      const double span = std::max(1e-3, reach - commit);
      const double urgency = std::clamp(
        commit_demand_ * (reach - along) / span, 0.0, 1.0);

      // Confidence: a real solid return, not a flicker - unless the detector
      // could not measure it at all, in which case fall back to geometry.
      const bool trust_unmeasured =
        c.max_dense_weight == 0.0f && c.observed_count == 0u && c.track_id == 0u;
      double confidence = 1.0;
      if (!trust_unmeasured) {
        confidence = (dense_full_ > 0.0) ?
          std::clamp(static_cast<double>(c.max_dense_weight) / dense_full_, 0.0, 1.0) : 1.0;
        // Track age is NOT used to erase a cluster any more.
        //
        // observed_count resets whenever the detector loses association, and
        // association is the first thing that breaks with speed: measured at
        // 5-6 m/s the centroid moves up to 49.8 cm between frames against a
        // 60 cm association gate, and the success rate falls from 90-97% to
        // 81%. Zeroing confidence on a young track therefore made real
        // obstacles vanish exactly when the car was going fast enough to need
        // them - a silent failure with speed as its trigger.
        //
        // Flicker rejection does not need this gate. block_ms already requires
        // the demand to persist 200 ms, which is eight frames at 40 Hz, and no
        // spurious return survives that. A re-associated real object does.
        // So an immature track is discounted rather than deleted, and the
        // decision to act still rests on time.
        if (min_observed_ > 1 && static_cast<int>(c.observed_count) < min_observed_) {
          confidence *= immature_scale_;
        }
      } else {
        ++unmeasured;
      }
      const double d = urgency * confidence * on_path;
      if (d > demand) {
        demand = d;
        // Remember where on the plan the thing that is blocking us sits. That
        // is what makes "it went behind me" answerable later: without it, an
        // obstacle that stopped being reported and an obstacle that was passed
        // look identical, and the prompt return fires on detection dropouts -
        // which at speed is often, since association breaks on about a quarter
        // of tracks above 5 m/s.
        if (on_plan) {
          block_station_[drive.name] = station_of_max_;   // absolute, on the plan
          have_block_station_[drive.name] = true;
        }
      }
    }
    if (unmeasured > 0 && considered > 0) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "obstacle trust fields unmeasured on %d/%d clusters (odom lost?) - "
        "judging on geometry alone", unmeasured, considered);
    }
    last_on_plan_[drive.name] = on_plan;
    last_ahead_[drive.name] = any_ahead;
    // Has the thing that blocked us actually gone by? Answerable only because
    // its place on the plan was remembered: an obstacle that stopped being
    // reported and one that was passed are indistinguishable from the absence
    // of detections alone, and at speed the first happens often - association
    // breaks on about a quarter of tracks above 5 m/s.
    bool passed = false;
    if (on_plan && have_block_station_[drive.name]) {
      passed = ref.path->forwardDistance(
        ref.ego_station, block_station_[drive.name]) < 0.0;
    }
    last_passed_[drive.name] = passed;
    return setState(drive.name, ctx, demand, nearest, considered, "");
  }

private:
  // Measured speed if it is fresh, otherwise the command. `from_odom` tells the
  // caller which, so the note can say so rather than quietly using the worse one.
  double geometrySpeed(double commanded, const Context & ctx, bool * from_odom) const
  {
    if (has_speed_ &&
      (speed_timeout_ <= 0.0 || (ctx.now - speed_stamp_).seconds() <= speed_timeout_))
    {
      *from_odom = true;
      return std::abs(measured_speed_);
    }
    *from_odom = false;
    return std::abs(commanded);
  }

  // Everything needed to place a cluster on the plan: which path, where we are
  // on it, and how to get from the cluster's frame into the path's.
  struct PathRef
  {
    const PathReference * path{nullptr};
    double ego_station{0.0};
    double cos_yaw{1.0}, sin_yaw{0.0}, tx{0.0}, ty{0.0};   // cluster frame -> path frame
  };

  // Resolve the plan and the transform for this scan. Returns false - and says
  // why, throttled - whenever the plan cannot be used, so the caller falls back
  // to the commanded arc rather than silently reporting a clear path.
  bool buildPathRef(const std_msgs::msg::Header & header, PathRef * out)
  {
    const PathReference * path = nullptr;
    {
      // mtx_ is already held by the caller.
      if (!path_.empty()) {
        path = &path_;
      } else if (!csv_path_.empty()) {
        path = &csv_path_;
      }
    }
    if (!path) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "obstacle avoidance: no planned path (%s silent, no usable csv) - "
        "judging against the commanded steering instead",
        path_topic_.c_str());
      return false;
    }
    if (!tf_buffer_) {return false;}
    geometry_msgs::msg::TransformStamped tf_cluster, tf_ego;
    try {
      // Zero timeout, deliberately. A blocking lookup here waits for data that
      // this node's own executor has to deliver, which tf2 warns about and
      // which would stall the evaluation tick. /tf runs an order of magnitude
      // faster than the scan, so the stamp is normally already in the buffer;
      // when it is not, fall back to the newest transform available and judge
      // it on age rather than blocking.
      tf_cluster = lookupOrLatest(path->frame(), header.frame_id, header.stamp);
      tf_ego = lookupOrLatest(path->frame(), base_frame_, header.stamp);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "obstacle avoidance: cannot place the car on the plan (%s) - judging "
        "against the commanded steering instead. This is expected while "
        "localization is down, and the confidence pathway handles that case.",
        e.what());
      return false;
    }
    const auto & q = tf_cluster.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    out->path = path;
    out->cos_yaw = std::cos(yaw);
    out->sin_yaw = std::sin(yaw);
    out->tx = tf_cluster.transform.translation.x;
    out->ty = tf_cluster.transform.translation.y;
    const auto ego = path->project(
      tf_ego.transform.translation.x, tf_ego.transform.translation.y);
    if (!ego.valid) {return false;}
    out->ego_station = ego.station;
    return true;
  }

  // Transform at `stamp` if the buffer has it, otherwise the newest one, and
  // only if that is fresh enough to still describe where the car is. Anything
  // older is worse than no answer: it would place the obstacle against a
  // stale pose and report a confident, wrong verdict.
  geometry_msgs::msg::TransformStamped lookupOrLatest(
    const std::string & target, const std::string & source,
    const builtin_interfaces::msg::Time & stamp) const
  {
    try {
      return tf_buffer_->lookupTransform(target, source, tf2_ros::fromMsg(stamp));
    } catch (const tf2::TransformException &) {
      const auto latest = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      const double age =
        (rclcpp::Time(stamp) - rclcpp::Time(latest.header.stamp)).seconds();
      if (std::abs(age) > tf_timeout_) {
        throw tf2::ExtrapolationException(
          "newest " + target + "<-" + source + " is " + std::to_string(age) +
          "s from the scan");
      }
      return latest;
    }
  }

  // Closest approach of a cluster's footprint to the PLANNED line. Same
  // perimeter walk as the arc version and for the same reason: a corridor
  // narrower than the box cannot cross it without touching the perimeter.
  bool nearestOnPath(
    const PathRef & ref, const obstacle_context_msgs::msg::ObstacleCluster & c,
    double * station, double * lateral) const
  {
    const double x0 = std::min(c.min_x, c.max_x), x1 = std::max(c.min_x, c.max_x);
    const double y0 = std::min(c.min_y, c.max_y), y1 = std::max(c.min_y, c.max_y);
    if (!std::isfinite(x0) || !std::isfinite(x1) ||
      !std::isfinite(y0) || !std::isfinite(y1))
    {
      return false;
    }
    double best_lat = std::numeric_limits<double>::infinity();
    double best_station = 0.0;
    bool have = false;
    auto test = [&](double px, double py) {
        if (!std::isfinite(px) || !std::isfinite(py)) {return;}
        const double mx = ref.tx + ref.cos_yaw * px - ref.sin_yaw * py;
        const double my = ref.ty + ref.sin_yaw * px + ref.cos_yaw * py;
        const auto pr = ref.path->project(mx, my);
        if (!pr.valid || pr.lateral >= best_lat) {return;}
        best_lat = pr.lateral;
        best_station = pr.station;
        have = true;
      };
    test(c.center_x, c.center_y);
    // Bounded sample counts. ceil() of a wild bounding box is undefined
    // behaviour once it leaves int range, and on aarch64 it saturates to
    // INT_MAX - a loop that never ends while holding the evaluation lock.
    const int nx = boxSamples(x1 - x0);
    const int ny = boxSamples(y1 - y0);
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
    if (!have) {return false;}
    *station = best_station;
    *lateral = best_lat;
    return true;
  }

  // Sample count for one side of a bounding box, clamped hard.
  int boxSamples(double extent) const
  {
    if (!std::isfinite(extent) || extent <= 0.0) {return 0;}
    const double n = std::ceil(extent / sample_step_);
    return static_cast<int>(std::clamp(n, 0.0, 64.0));
  }

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
    // Commitment latch, separate from the presence latch below: presence
    // decides whether we are in a detour at all, this decides whether the
    // drive stays disqualified while we are in one.
    if (demand >= commit_demand_) {
      st.committed = true;
    } else if (st.committed && demand <= release_demand_) {
      st.committed = false;
    }
    if (clear_now != st.was_clear || !st.valid) {
      st.edge = ctx.now;
      st.was_clear = clear_now;
      st.valid = true;
    }
    const double dwell = (ctx.now - st.edge).seconds();
    // Releasing the detour. When we are judging against the plan we can tell
    // the difference between "the obstacle went behind us" - a fact about
    // geometry - and "it stopped being reported", which might just be a gap in
    // detection. The first earns a prompt return; the second still has to wait
    // out the full hold, because coming back early on a detection dropout puts
    // the map controller back in charge while the thing is still alongside.
    // Passed means the blocking obstacle's own place on the plan is behind
    // us - a measurement, not the absence of a detection.
    const bool by_station = last_passed_[drive];
    const double release = by_station ? passed_clear_ : clear_;
    if (!clear_now && dwell >= block_) {st.blocked = true;}
    // An obstacle we have not driven past yet is not forgotten because it
    // stopped being reported. Detection gaps are ordinary - the detector loses
    // association on about a quarter of tracks above 5 m/s - and releasing on
    // one splits a single encounter into several detours, which is what the
    // handover count on the 0814 drive was made of. So while its place on the
    // plan is still ahead, the detour holds, up to remember_cap_ so that a
    // spurious detection cannot block the drive indefinitely.
    // Holding the detour on memory alone, while the obstacle's place on the
    // plan is still ahead, was tried and measured worse: it cost 4 points of
    // occupancy on the 0814 drive and did not reduce handovers at all, so the
    // splitting it was meant to prevent was never coming from forgetting.
    // remember_cap_ms is left configurable but defaults to no effect.
    const bool still_ahead = have_block_station_[drive] && !by_station;
    const double cap = (remember_cap_ > 0.0 && still_ahead) ?
      std::max(release, remember_cap_) : release;
    if (clear_now && st.blocked && dwell >= cap) {
      st.blocked = false;
      have_block_station_[drive] = false;   // that encounter is over
    }

    char buf[140];
    if (st.blocked) {
      if (clear_now) {
        std::snprintf(
          buf, sizeof(buf), "%s, holding %.1f/%.1fs",
          by_station ? "obstacle passed" :
          (still_ahead ? "not past it yet" : "path clearing"), dwell, cap);
        return ScoreResult::ok(0.0, buf);        // stay committed to the detour
      }
      std::snprintf(
        buf, sizeof(buf), "obstacle at %.2fm %s, demand %.2f%s", dist,
        last_on_plan_[drive] ? "on the plan" : "on the commanded arc", demand,
        st.committed ? ", committed" : "");
      // While committed the drive stays disqualified even as the demand eases,
      // so one obstacle costs one handover rather than one per threshold
      // crossing.
      const double s = std::clamp(1.0 - demand, 0.0, 1.0);
      return ScoreResult::ok(st.committed ? std::min(s, 0.0) : s, buf);
    }
    if (!clear_now) {
      std::snprintf(
        buf, sizeof(buf), "obstacle at %.2fm %s, demand %.2f, confirming %.1f/%.1fs",
        dist, last_on_plan_[drive] ? "on the plan" : "on the arc", demand, dwell, block_);
      return ScoreResult::ok(1.0, buf);          // not confirmed yet
    }
    if (!reason.empty()) {return ScoreResult::ok(1.0, reason);}
    std::snprintf(
      buf, sizeof(buf), "clear (%d clusters considered%s)", considered,
      last_on_plan_[drive] ? "" : ", NO PLAN - using the commanded arc");
    return ScoreResult::ok(1.0, buf);
  }

  std::map<std::string, bool> last_on_plan_;
  std::map<std::string, bool> last_passed_;
  std::map<std::string, bool> have_block_station_;
  std::map<std::string, double> block_station_;
  double station_of_max_{0.0};
  std::map<std::string, bool> last_ahead_;

  struct State
  {
    bool was_clear{true};
    bool valid{false};
    bool blocked{false};
    bool committed{false};
    rclcpp::Time edge;
  };

  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
  std::string speed_topic_;
  double speed_timeout_{0.3};
  bool has_speed_{false};
  double measured_speed_{0.0};
  rclcpp::Time speed_stamp_;
  rclcpp::Subscription<obstacle_context_msgs::msg::ObstacleClusterArray>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  PathReference path_;        // from the topic
  PathReference csv_path_;    // standby

  std::string path_topic_, path_csv_, path_frame_, base_frame_;
  double tf_timeout_{0.2};
  bool use_path_speed_{true};
  double reaction_time_{0.5};

  double lateral_accel_{5.0};
  double commit_margin_{0.5};
  double commit_demand_{0.85};
  double release_demand_{0.6};
  double jitter_base_{0.10}, jitter_per_m_{0.03}, jitter_per_speed_{0.04};
  double passed_clear_{0.25};
  double remember_cap_{0.0};

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
  double immature_scale_{1.0};
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
