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
#include <cstdint>
#include <limits>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <obstacle_context_msgs/msg/obstacle_cluster_array.hpp>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "co_driver/arc_geometry.hpp"
#include "co_driver/path_reference.hpp"
#include "co_driver/scan_occupancy.hpp"
#include "co_driver/surface_polyline.hpp"
#include "co_driver/vesc_speed.hpp"

#include "co_driver/scorer.hpp"

namespace co_driver
{

class ObstacleAvoidScorer : public Scorer
{
private:
  // Everything needed to place evidence on one immutable path snapshot.
  struct PathRef
  {
    const PathReference * path{nullptr};
    double ego_station{0.0};
    double cos_yaw{1.0}, sin_yaw{0.0}, tx{0.0}, ty{0.0};
  };

public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    // Where "what is on the line" comes from.
    //   "clusters"  the detector's ObstacleClusterArray
    //   "scan"      the lidar directly, accumulated and voted (scan_occupancy.hpp)
    source_ = jstr(p, "source", "scan");
    if (source_ != "clusters" && source_ != "scan") {
      RCLCPP_ERROR(
        node->get_logger(), "obstacle avoidance '%s': source '%s' is neither "
        "\"clusters\" nor \"scan\" - falling back to clusters.",
        name.c_str(), source_.c_str());
      source_ = "clusters";
    }
    scan_topic_ = jstr(p, "scan_topic", "/scan");
    deskew_ = jbool(p, "deskew", true);
    topic_ = jstr(p, "topic", "/obstacle_clusters");
    wheelbase_ = jnum(p, "wheelbase", 0.324);
    half_width_ = jnum(p, "vehicle_half_width", 0.18);
    margin_ = jnum(p, "lateral_margin_m", 0.10);
    {
      ScanOccupancy::Settings s;
      // The band around the line is the width the CAR needs, not a number of
      // its own: the same half_width + margin the cluster path measures against.
      // Something inside it has to be gone around; something outside it is
      // passed. Tuning it separately would have meant two different answers to
      // "is this in my way" depending only on where the evidence came from.
      s.near_line_m = half_width_ + margin_;
      s.car_width_until_m = jnum(p, "car_width_until_m", 1.5);
      s.scans_kept = jint(p, "scans_kept", 5);
      s.points_per_scan = jint(p, "points_per_scan", 2);
      s.dense_one_scan = jint(p, "dense_one_scan", 8);
      s.narrow_per_m = jnum(p, "narrow_per_m", 0.0);
      s.never_narrower_than_m = jnum(p, "never_narrower_than_m", 0.05);
      s.recovered_margin_cut_m = std::max(
        0.0, jnum(p, "surface_recovered_margin_cut_m", 0.0));
      s.wall_longer_than_m = jnum(p, "wall_longer_than_m", 1.0);
      s.one_scan_within_m = jnum(p, "one_scan_within_m", 2.5);
      s.add_scan_every_m = jnum(p, "add_scan_every_m", 2.0);
      s.add_scan_every_mps = jnum(p, "add_scan_every_mps", 3.0);
      s.same_place_m = jnum(p, "same_place_m", 0.30);
      s.biggest_thing_m = jnum(p, "biggest_thing_m", 1.5);
      grazing_ = jnum(p, "surface_grazing_deg", 10.0) * M_PI / 180.0;
      range_noise_ = jnum(p, "range_noise_m", 0.03);
      surface_model_ = jstr(p, "surface_model", "size");
      if (
        surface_model_ != "size" && surface_model_ != "polyline" &&
        surface_model_ != "swept")
      {
        RCLCPP_ERROR(
          node->get_logger(), "obstacle avoidance '%s': surface_model '%s' is neither "
          "\"size\", \"polyline\" nor \"swept\" - keeping the legacy size filter.",
          name.c_str(), surface_model_.c_str());
        surface_model_ = "size";
      }
      surface_fit_error_ = std::max(0.0, jnum(p, "surface_fit_error_m", 0.05));
      surface_fit_error_per_m_ = std::max(
        0.0, jnum(p, "surface_fit_error_per_m", 0.0));
      surface_fit_min_points_ = std::max(2, jint(p, "surface_fit_min_points", 2));
      s.beam_stride = std::max(1, jint(p, "beam_stride", 3));
      scan_.configure(s);
    }
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
    // How much narrower the corridor is for a wall-labelled cluster. The
    // default takes the whole margin away, leaving the vehicle's own width.
    wall_margin_cut_ = jnum(p, "wall_margin_cut_m", 0.10);
    max_width_ = jnum(p, "max_width_m", 0.0);        // 0 = no size filter
    min_points_ = jint(p, "min_point_count", 0);
    sample_step_ = std::max(0.02, jnum(p, "sample_step_m", 0.08));
    // ---- planned path -----------------------------------------------------
    // Judging against the planned line rather than the instantaneous steering
    // angle. See path_reference.hpp for why the command is the wrong question.
    path_topic_ = jstr(p, "path_topic", "/global_path");
    const bool path_transient_local = jbool(p, "path_transient_local", true);
    path_csv_ = jstr(p, "path_csv", "");
    path_frame_ = jstr(p, "path_frame", "map");
    base_frame_ = jstr(p, "base_frame", "base_link");
    tf_timeout_ = jms(p, "tf_timeout_ms", 200.0);
    use_path_speed_ = jbool(p, "use_path_speed", true);

    // An obstacle is a property of the scene, not of whichever candidate is
    // being scored.  In particular, when VESC telemetry is stale, using each
    // candidate's own command would give a fast pp_main a long lookahead while
    // a slow gap_loc saw the exact same obstacle as clear.  A configured live
    // drive supplies one speed/steering snapshot to every candidate instead.
    // Leaving this empty preserves the legacy per-candidate fallback.
    reference_drive_ = jstr(p, "reference_drive", "");

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
    // Read the VESC telemetry directly so a localization jump cannot become a
    // speed spike here. f1_stack_for_damvi/red_damvi converts state.speed (ERPM)
    // with (erpm - offset) / gain and applies a 0.05 m/s deadband. This car's
    // result is about 2.6 times below the speed domain used by the controller,
    // so that correction remains an explicit, independently tunable scale.
    vesc_state_topic_ = jstr(p, "vesc_state_topic", "/sensors/core");
    speed_calibration_.speed_to_erpm_gain =
      jnum(p, "speed_to_erpm_gain", 3172.47);
    speed_calibration_.speed_to_erpm_offset =
      jnum(p, "speed_to_erpm_offset", 0.0);
    speed_calibration_.wheel_speed_deadband =
      jnum(p, "wheel_speed_deadband", 0.05);
    speed_calibration_.wheel_speed_scale =
      jnum(p, "wheel_speed_scale", 2.6);
    if (
      !std::isfinite(speed_calibration_.speed_to_erpm_gain) ||
      std::abs(speed_calibration_.speed_to_erpm_gain) < 1.0e-6 ||
      !std::isfinite(speed_calibration_.speed_to_erpm_offset) ||
      !std::isfinite(speed_calibration_.wheel_speed_deadband) ||
      speed_calibration_.wheel_speed_deadband < 0.0 ||
      !std::isfinite(speed_calibration_.wheel_speed_scale) ||
      speed_calibration_.wheel_speed_scale <= 0.0)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "obstacle avoidance '%s': invalid VESC speed calibration "
        "(gain %.6f, offset %.6f, deadband %.3f, scale %.3f)",
        name.c_str(), speed_calibration_.speed_to_erpm_gain,
        speed_calibration_.speed_to_erpm_offset,
        speed_calibration_.wheel_speed_deadband,
        speed_calibration_.wheel_speed_scale);
      return false;
    }
    // The prepared reference command (or the candidate when no reference can
    // be resolved) remains the fallback when VESC telemetry is stale.
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

    // Presence is judged over a WINDOW of scans, not on the current one.
    //
    // A real object is reported lap after lap from every angle; what comes and
    // goes is structure the detector did not label as wall, thrown onto the
    // planned line by map distortion and pose jitter at the corners. Measured
    // on 0814 over the stretch of the lap with no object on it at all, 67.8%
    // of samples were disqualified on obstacles, against detections reported
    // 2.6-4.0 m ahead "on the plan". Those flicker; the two placed objects do
    // not.
    //
    // Judging the fraction of the window that saw something, rather than an
    // unbroken run, also survives the detector losing association - which it
    // does on about a quarter of tracks above 5 m/s - so the same filter
    // handles both the false positives and the dropouts.
    window_ = jms(p, "presence_window_ms", 900.0);
    enter_frac_ = std::clamp(jnum(p, "presence_enter", 0.55), 0.0, 1.0);
    exit_frac_ = std::clamp(jnum(p, "presence_exit", 0.20), 0.0, enter_frac_);
    // Accumulating over more scans only helps if the scans have to AGREE. A
    // fixed object keeps the same place on the planned line every time it is
    // seen; structure thrown onto the line by map distortion moves around,
    // because what is being reported is a different piece of wall each time.
    // So a sample counts toward presence only if it names the same place as
    // the rest of the window - which filters without costing reaction time,
    // where simply lengthening the window would delay the commit and there is
    // no range to spare for that (the detector stops at 10 m).
    station_tol_ = jnum(p, "presence_station_tolerance_m", 1.0);

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

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    if (!vesc_state_topic_.empty()) {
      speed_sub_ = node->create_subscription<vesc_msgs::msg::VescStateStamped>(
        vesc_state_topic_, rclcpp::QoS(10),
        [this](const vesc_msgs::msg::VescStateStamped::ConstSharedPtr msg) {
          double speed = 0.0;
          if (!vescErpmToSpeed(msg->state.speed, speed_calibration_, &speed)) {
            RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "obstacle avoidance: invalid VESC speed sample on %s",
              vesc_state_topic_.c_str());
            return;
          }
          std::lock_guard<std::mutex> lock(mtx_);
          measured_speed_ = speed;
          speed_stamp_ = node_->now();
          has_speed_ = true;
        }, opts);
      RCLCPP_INFO(
        node->get_logger(),
        "obstacle avoidance speed <- %s: (ERPM - %.2f) / %.2f, "
        "deadband %.2fm/s, x%.2f; command fallback after %.0fms",
        vesc_state_topic_.c_str(), speed_calibration_.speed_to_erpm_offset,
        speed_calibration_.speed_to_erpm_gain,
        speed_calibration_.wheel_speed_deadband,
        speed_calibration_.wheel_speed_scale, speed_timeout_ * 1e3);
    }
    // The detector publishes SensorDataQoS; a RELIABLE subscription would not
    // match it and would receive nothing at all.
    sub_ = node->create_subscription<obstacle_context_msgs::msg::ObstacleClusterArray>(
      topic_, rclcpp::SensorDataQoS(),
      [this](const obstacle_context_msgs::msg::ObstacleClusterArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        clusters_ = msg;
        rx_ = node_->now();
      }, opts);

    // A fixed global path is commonly latched, while a live local/planned path
    // is commonly reliable + volatile. Request exactly the configured
    // durability: a transient-local subscription is incompatible with a
    // volatile publisher and would silently receive no live path at all.
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
          if (msg->poses.size() < 2) {return;}
          path_.fromMessage(*msg);
          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "obstacle avoidance: path from %s - %zu points, "
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
    if (source_ == "scan") {
      scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {onScan(msg);},
        opts);
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, false);

    if (source_ == "scan") {
      const auto & s = scan_.settings();
      RCLCPP_INFO(
        node->get_logger(),
        "obstacle avoidance '%s' <- %s DIRECTLY: %d returns within %.2fm of the "
        "line (the car's own %.2fm + %.2fm), closing to %.2fm by %.1fm out; "
        "measured from the car itself inside %.1fm; surfaces bigger than %.1fm "
        "are the track; in 1 "
        "of the last %d scans within %.1fm rising to %d by %.1fm (+1 per %.1fm/s) "
        "- or %d returns in one scan.%s Deskew %s. The detector's clusters are "
        "NOT used.",
        name.c_str(), scan_topic_.c_str(), s.points_per_scan, s.near_line_m,
        half_width_, margin_, s.never_narrower_than_m,
        s.narrow_per_m > 0.0 ?
        (s.near_line_m - s.never_narrower_than_m) / s.narrow_per_m : 0.0,
        s.car_width_until_m, s.biggest_thing_m, s.scans_kept, s.one_scan_within_m,
        s.scans_kept,
        s.one_scan_within_m + s.add_scan_every_m * (s.scans_kept - 1),
        s.add_scan_every_mps, s.dense_one_scan,
        s.wall_longer_than_m > 0.0 ? " Longer than a metre along the line is wall." : "",
        deskew_ ? "on" : "off");
      if (surface_model_ == "polyline") {
        RCLCPP_WARN(
          node->get_logger(),
          "obstacle avoidance '%s': EXPERIMENTAL %s surface model is on; "
          "surfaces above %.2fm are recursively split at %.3fm + %.4f*range "
          "orthogonal error (at least %d beams per side). Fitted lines only split "
          "surfaces; path occupancy still uses the raw deskewed returns. Rescued "
          "pieces lose %.2fm of corridor margin.",
          name.c_str(), surface_model_.c_str(), s.biggest_thing_m, surface_fit_error_,
          surface_fit_error_per_m_, surface_fit_min_points_, s.recovered_margin_cut_m);
      } else if (surface_model_ == "swept") {
        RCLCPP_WARN(
          node->get_logger(),
          "obstacle avoidance '%s': EXPERIMENTAL swept guard is on; every raw "
          "deskewed return inside the planned vehicle corridor bypasses the %.2fm "
          "surface-size filter. Segments still group evidence but cannot erase it.",
          name.c_str(), s.biggest_thing_m);
      }
    } else {
      RCLCPP_INFO(
        node->get_logger(),
        "obstacle avoidance '%s' <- %s (%.1fs of travel ahead, sustained %.0fms%s)",
        name.c_str(), topic_.c_str(), trigger_time_, block_ * 1e3,
        ignore_wall_static_ ? ", wall-labelled clusters held to a tighter corridor" : "");
    }
    if (max_width_ > 0.0 && source_ == "clusters") {
      RCLCPP_INFO(
        node->get_logger(), "obstacle avoidance '%s': clusters wider than %.2fm "
        "are treated as structure", name.c_str(), max_width_);
    }
    return true;
  }

  void prepare(const Context & ctx, const std::vector<Drive> & drives) override
  {
    have_reference_command_ = false;
    resolved_reference_drive_.clear();

    const Drive * reference = nullptr;
    const auto usable_reference = [&ctx](const Drive & drive) {
        // Drive::isLive currently includes these finite checks. Keep them
        // explicit at this safety boundary so a future relaxation of command
        // liveness cannot turn one bad named command into every obstacle input
        // becoming unavailable.
        return drive.isLive(ctx.now) &&
               std::isfinite(drive.cmd.drive.speed) &&
               std::isfinite(drive.cmd.drive.steering_angle);
      };
    if (!reference_drive_.empty()) {
      for (const auto & drive : drives) {
        if (drive.name == reference_drive_ && usable_reference(drive)) {
          reference = &drive;
          break;
        }
      }
    }

    // The named producer may be starting up or briefly stale.  Use the live
    // command with the greatest speed magnitude as the conservative common
    // geometry in that case.  Speed and steering stay paired: combining the
    // speed of one command with another command's steering would describe a
    // path that no controller actually requested.
    if (reference == nullptr && !reference_drive_.empty()) {
      for (const auto & drive : drives) {
        if (!usable_reference(drive)) {continue;}
        if (
          reference == nullptr ||
          std::abs(drive.cmd.drive.speed) > std::abs(reference->cmd.drive.speed))
        {
          reference = &drive;
        }
      }
    }

    if (reference == nullptr && !reference_drive_.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "obstacle avoidance: reference drive '%s' has no live command and no "
        "live fallback exists; using each candidate command",
        reference_drive_.c_str());
    }

    if (reference != nullptr) {
      reference_speed_ = reference->cmd.drive.speed;
      reference_steering_ = reference->cmd.drive.steering_angle;
      resolved_reference_drive_ = reference->name;
      have_reference_command_ = true;
    }

    if (reference != nullptr && reference->name != reference_drive_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "obstacle avoidance: reference drive '%s' is not live; using fastest "
        "live drive '%s' for common obstacle geometry",
        reference_drive_.c_str(), resolved_reference_drive_.c_str());
    }

    // Freeze telemetry at the same evaluation boundary as the command. A
    // VESC callback can run between score(pp_main) and score(gap_loc); letting
    // each score read it independently would still give one scene two answers.
    // A later sample is intentionally visible only after the next prepare().
    {
      std::lock_guard<std::mutex> lock(mtx_);
      geometry_snapshot_stamp_ = ctx.now;
      geometry_snapshot_has_speed_ = has_speed_;
      geometry_snapshot_speed_age_ = has_speed_ ?
        (ctx.now - speed_stamp_).seconds() : 0.0;
      geometry_snapshot_measured_speed_ = measured_speed_;
      geometry_snapshot_valid_ = true;

      // A common speed is already known here when the reference command exists,
      // so a scan arriving before the first score in this tick sizes its horizon
      // from the same snapshot as every candidate score.
      if (have_reference_command_) {
        last_speed_ = selectGeometrySpeed(
          geometry_snapshot_has_speed_, geometry_snapshot_speed_age_, speed_timeout_,
          geometry_snapshot_measured_speed_, reference_speed_);
      }

      // Freeze scan health, occupancy, header, invalidation generation and the
      // selected path at the same evaluation boundary. A newer valid scan or
      // path is intentionally first visible on the next prepare(), while an
      // invalidation is still noticed by the generation check at commit.
      scan_evaluation_valid_ = false;
      if (source_ == "scan") {
        // Expire the live ring at the prepare boundary, before it is frozen.
        // Doing this later in score() is ambiguous: a fresh callback may have
        // arrived after prepare, and clearing there would destroy next tick's
        // good data. Here mtx_ proves scan_rx_ still names the ring we clear.
        if (
          has_scan_ && scan_usable_ && timeout_ > 0.0 &&
          (ctx.now - scan_rx_).seconds() > timeout_)
        {
          const std::string why = scan_blocked_.empty() ?
            ("no scan on " + scan_topic_) : scan_blocked_;
          invalidateScan(why);
        }
        scan_evaluation_stamp_ = ctx.now;
        scan_evaluation_has_scan_ = has_scan_;
        scan_evaluation_usable_ = scan_usable_;
        scan_evaluation_rx_ = scan_rx_;
        scan_evaluation_blocked_ = scan_blocked_;
        scan_evaluation_header_ = scan_header_;
        scan_evaluation_invalidation_generation_ = scan_invalidation_generation_;
        if (scan_usable_) {scan_evaluation_snapshot_ = scan_;}
        if (!path_.empty()) {
          scan_evaluation_path_ = path_;
          scan_evaluation_have_path_ = true;
        } else if (!csv_path_.empty()) {
          scan_evaluation_path_ = csv_path_;
          scan_evaluation_have_path_ = true;
        } else {
          scan_evaluation_have_path_ = false;
        }
      }
    }

    // Resolve TF once per evaluation as well. Keeping this outside mtx_ avoids
    // holding the scan callback behind TF work; the path being queried is the
    // immutable copy made above.
    if (source_ == "scan") {
      PathRef ref;
      const bool on_plan = scan_evaluation_usable_ && scan_evaluation_have_path_ &&
        resolvePathRef(scan_evaluation_header_, &scan_evaluation_path_, &ref);
      std::lock_guard<std::mutex> lock(mtx_);
      scan_evaluation_path_ref_ = ref;
      scan_evaluation_on_plan_ = on_plan;
      scan_evaluation_valid_ = true;
    }
  }

  ScoreResult score(const Drive & drive, const Context & ctx) override
  {
    obstacle_context_msgs::msg::ObstacleClusterArray::ConstSharedPtr clusters;
    ScanOccupancy scan_snapshot;
    std::uint64_t scan_invalidation_generation = 0;
    std_msgs::msg::Header ref_header;   // what the path lookup is stamped with
    PathReference path_snapshot;
    bool have_path_snapshot = false;
    bool have_prepared_path_ref = false;
    bool prepared_on_plan = false;
    PathRef prepared_path_ref;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      // EVERY EXIT HERE REPORTS "clear", never unavailable. This input carries
      // a real weight, and missing:mask rescales the logit by the weights that
      // were usable - so dropping out would silently move the calibrated
      // localization thresholds every time a producer is not running.
      //
      // Reporting a clear path because nothing can be SEEN is not the same as
      // reporting one because nothing is THERE, and a score of 1.0 cannot tell
      // the two apart downstream. Failing open is still right - the reactive
      // controller needs the same lidar, so disqualifying the map controller
      // would only hand the car to something equally blind - but it must not
      // be silent, so each of these says so.
      if (source_ == "scan") {
        const bool use_prepared = scan_evaluation_valid_ &&
          scan_evaluation_stamp_ == ctx.now;
        const bool snapshot_has_scan = use_prepared ?
          scan_evaluation_has_scan_ : has_scan_;
        const bool snapshot_usable = use_prepared ?
          scan_evaluation_usable_ : scan_usable_;
        const rclcpp::Time snapshot_rx = use_prepared ?
          scan_evaluation_rx_ : scan_rx_;
        const std::string snapshot_blocked = use_prepared ?
          scan_evaluation_blocked_ : scan_blocked_;
        // Deliberately before any mention of the detector. Asking for a
        // cluster message here - which this branch never reads - meant that on
        // a car with the detector not running, this input reported a clear
        // path for the entire run and never once looked at the lidar.
        if (!snapshot_has_scan) {
          resetScanDecision(drive.name);
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "obstacle avoidance: nothing has arrived on %s - reporting a clear "
            "path because nothing can be seen, not because nothing is there.",
            scan_topic_.c_str());
          return ScoreResult::ok(1.0, "no scan yet on " + scan_topic_);
        }
        if (timeout_ > 0.0 && (ctx.now - snapshot_rx).seconds() > timeout_) {
          // Two very different faults reach here and they need different
          // people: the lidar stopped, or it is fine and the pose that places
          // its returns is gone. Losing localization produces the second one,
          // and calling that "no scan" costs whoever is debugging it an hour.
          const std::string why = snapshot_blocked.empty() ?
            ("no scan on " + scan_topic_) : snapshot_blocked;
          // A fresh scan may have arrived after prepare(). Do not destroy it
          // merely because this evaluation's frozen snapshot is old.
          if (!use_prepared) {invalidateScan(why);}
          resetScanDecision(drive.name);
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "obstacle avoidance: %s for %.1fs - reporting a clear path because "
            "nothing can be seen, not because nothing is there.",
            why.c_str(), (ctx.now - snapshot_rx).seconds());
          return ScoreResult::ok(1.0, why + " - nothing can be seen");
        }
        if (!snapshot_usable) {
          const std::string why = snapshot_blocked.empty() ?
            ("latest scan on " + scan_topic_ + " could not be processed") : snapshot_blocked;
          resetScanDecision(drive.name);
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "obstacle avoidance: %s - reporting a clear path because nothing "
            "can be seen, not because nothing is there.", why.c_str());
          return ScoreResult::ok(1.0, why + " - nothing can be seen");
        }
        // Prefer the evaluation-wide copy made by prepare(); direct test/tool
        // callers that skip prepare still take one coherent live copy here.
        scan_snapshot = use_prepared ? scan_evaluation_snapshot_ : scan_;
        scan_invalidation_generation = use_prepared ?
          scan_evaluation_invalidation_generation_ : scan_invalidation_generation_;
        ref_header = use_prepared ? scan_evaluation_header_ : scan_header_;
        if (use_prepared) {
          have_prepared_path_ref = true;
          prepared_on_plan = scan_evaluation_on_plan_;
          prepared_path_ref = scan_evaluation_path_ref_;
        } else if (!path_.empty()) {
          path_snapshot = path_;
          have_path_snapshot = true;
        } else if (!csv_path_.empty()) {
          path_snapshot = csv_path_;
          have_path_snapshot = true;
        }
      } else {
        if (!clusters_) {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "obstacle avoidance: nothing has arrived on %s - reporting a clear "
            "path because nothing can be seen, not because nothing is there.",
            topic_.c_str());
          return ScoreResult::ok(1.0, "no clusters on " + topic_);
        }
        if (timeout_ > 0.0 && (ctx.now - rx_).seconds() > timeout_) {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "obstacle avoidance: no clusters on %s for %.1fs - reporting a "
            "clear path because nothing can be seen, not because nothing is "
            "there.", topic_.c_str(), (ctx.now - rx_).seconds());
          return ScoreResult::ok(1.0, "stale clusters");
        }
        clusters = clusters_;
        ref_header = clusters_->header;
        if (!path_.empty()) {
          path_snapshot = path_;
          have_path_snapshot = true;
        } else if (!csv_path_.empty()) {
          path_snapshot = csv_path_;
          have_path_snapshot = true;
        }
      }
    }

    // Only the geometry command is shared.  Every presence/state update below
    // remains keyed by drive.name, so one evaluation tick still contributes
    // exactly one sample to each drive rather than N samples to a shared key.
    const double v = have_reference_command_ ?
      reference_speed_ : drive.cmd.drive.speed;
    const double delta = have_reference_command_ ?
      reference_steering_ : drive.cmd.drive.steering_angle;
    if (!std::isfinite(v) || !std::isfinite(delta)) {
      return ScoreResult::unavailable("obstacle geometry command is NaN/inf");
    }
    const double v_now = geometrySpeed(v, ctx);
    if (v_now < min_speed_) {
      if (source_ == "scan") {
        return setScanStateIfCurrent(
          scan_invalidation_generation, drive.name, ctx,
          0.0, 0.0, 0, "below min_speed");
      }
      return setState(drive.name, ctx, 0.0, 0.0, 0, "below min_speed");
    }

    // Which line are we judging against: the plan if we have it and can place
    // ourselves on it, the commanded arc if not. Falling back is loud - a
    // scorer that quietly answers "clear" because it lost its reference is
    // exactly the failure mode that made path_clearance untrustworthy.
    PathRef ref = have_prepared_path_ref ? prepared_path_ref : PathRef{};
    const bool on_plan = have_prepared_path_ref ? prepared_on_plan :
      resolvePathRef(
        ref_header, have_path_snapshot ? &path_snapshot : nullptr, &ref);

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

    if (source_ == "scan") {
      // The lidar answers directly: places on the line ahead that enough scans
      // agree are occupied. No clustering, no labels, no reported centre - see
      // scan_occupancy.hpp for why each of those was worth leaving out.
      if (!on_plan) {
        return setScanStateIfCurrent(
          scan_invalidation_generation, drive.name, ctx, 0.0, 0.0, 0,
          "no plan - the scan source has nothing to measure against");
      }
      const auto occ = scan_snapshot.occupied(v_now);
      considered = static_cast<int>(occ.size());
      bool have_new_hit = false;
      ScanOccupancy::Occupied new_hit;
      double new_block_station = 0.0;
      for (const auto & o : occ) {
        if (o.along > trigger) {continue;}
        any_ahead = true;
        nearest = std::min(nearest, o.along);
        const double span = std::max(1e-3, trigger - commit);
        const double urgency = std::clamp(
          commit_demand_ * (trigger - o.along) / span, 0.0, 1.0);
        // Nothing further is asked of it. Being reported at all already means
        // the returns were within near_line_m of the line and the scans agreed,
        // so the only thing left to decide is how soon it is reached.
        if (urgency > demand) {
          demand = urgency;
          new_block_station = ref.ego_station + o.along;
          new_hit = o;
          have_new_hit = true;
        }
      }

      // onScan() may invalidate the ring while occupied() works on its local
      // copy. Recheck the invalidation generation, then publish every derived
      // map/state update in the same mutex transaction. A failed scan can
      // therefore never be followed by a stale snapshot resurrecting a commit.
      std::lock_guard<std::mutex> lock(mtx_);
      if (
        !scan_usable_ ||
        scan_invalidation_generation != scan_invalidation_generation_)
      {
        return discardStaleScanDecision(drive.name);
      }
      have_hit_[drive.name] = have_new_hit;
      if (have_new_hit) {
        block_station_[drive.name] = new_block_station;
        have_block_station_[drive.name] = true;
        hit_[drive.name] = new_hit;
      }
      last_on_plan_[drive.name] = true;
      bool passed = false;
      if (have_block_station_[drive.name]) {
        passed = ref.path->forwardDistance(
          ref.ego_station, block_station_[drive.name]) < 0.0;
      }
      last_ahead_[drive.name] = any_ahead;
      last_passed_[drive.name] = passed;
      return setState(drive.name, ctx, demand, nearest, considered, "");
    }

    for (const auto & c : clusters->clusters) {
      // A wall-labelled cluster is not discarded outright any more - it is
      // held to a tighter corridor instead.
      //
      // The detector merges an object into the wall it is touching and labels
      // the result wall, so anything placed against the barrier disappears
      // from this scorer entirely. That is how the 0821-2 obstacles were
      // placed: differencing the first half of that run against the second
      // finds no new scan returns at all, because the new object is inside a
      // cell the wall already occupied.
      //
      // Geometry can tell them apart where the label cannot. The racing line
      // is drawn through drivable space, so a mapped wall is never on it; a
      // return sitting on the line is something that was not there when the
      // line was planned. Wall-labelled clusters therefore have to be closer
      // to the line to count - inside the vehicle's own width, with none of
      // the margin an unlabelled one gets.
      const bool wall_labelled = ignore_wall_static_ && c.is_wall_static;
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
      const double corridor = wall_labelled ?
        std::max(0.0, lateral_need - wall_margin_cut_) : lateral_need;
      if (corridor <= 0.0) {continue;}

      if (on_plan) {
        double station = 0.0, lateral = 0.0;
        if (!nearestOnPath(ref, c, trigger, &station, &lateral)) {continue;}
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
          (corridor + jitter - lateral) / std::max(1e-3, 2.0 * jitter), 0.0, 1.0);
      } else {
        // No plan to measure against, so the label is all there is.
        if (wall_labelled) {continue;}
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
        // Remember where the thing that STARTED this detour sits, and stop
        // remembering anything else until the detour is over.
        //
        // Updating it every cycle to whatever is most urgent now looked
        // harmless and was not: the two objects on the 0814 track are 9.3 m
        // apart on a 44.2 m lap, so the moment one is passed the next is
        // already in view, the remembered place jumps forward to it, and "have
        // I driven past it" can never become true. The detour then always ran
        // out the full clear_ms instead of ending the moment it was behind -
        // measured, every single obstacle veto in the stretch where there was
        // room to spare was that tail, not a new commitment.
        if (on_plan && !state_[drive.name].blocked) {
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
  // Corrected VESC speed if it is fresh, otherwise the prepared common
  // reference command (or the candidate command when reference_drive is off).
  double geometrySpeed(double commanded, const Context & ctx)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // prepare() is the evaluation transaction boundary. Keep using that VESC
    // sample even if a subscription callback updates the live fields between
    // two candidate scores. Direct callers that skip prepare retain the old
    // read-at-score behaviour.
    const bool use_snapshot = geometry_snapshot_valid_ &&
      geometry_snapshot_stamp_ == ctx.now;
    const bool has_speed = use_snapshot ? geometry_snapshot_has_speed_ : has_speed_;
    const double age = use_snapshot ? geometry_snapshot_speed_age_ :
      (has_speed_ ? (ctx.now - speed_stamp_).seconds() : 0.0);
    const double measured_speed = use_snapshot ?
      geometry_snapshot_measured_speed_ : measured_speed_;
    const bool fresh = has_speed &&
      (speed_timeout_ <= 0.0 || age <= speed_timeout_);
    if (!fresh && !vesc_state_topic_.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "obstacle avoidance: VESC speed on %s is %s; using geometry command %.2fm/s",
        vesc_state_topic_.c_str(), has_speed ? "stale" : "not received",
        std::abs(commanded));
    }
    last_speed_ = selectGeometrySpeed(
      has_speed, age, speed_timeout_, measured_speed, commanded);
    return last_speed_;
  }

  // One scan, deskewed and measured against the line.
  //
  // A UST-10LX sweeps its 1081 beams over 25 ms. At 5 m/s that is 12 cm of
  // travel, so placing the whole scan at one pose smears every object by that
  // much and the accumulated scans stop agreeing with each other - which is
  // exactly what the vote depends on. Each beam is placed at the pose for its
  // own instant, interpolated between the transform at the start of the sweep
  // and at the end. Two lookups per scan; a lookup per beam would cost more
  // than the whole scorer.
  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // Record reception before any processing. A missing path or transform is
    // not a silent lidar: the diagnostic must say that scans are arriving but
    // cannot be used. scan_rx_ therefore tracks raw topic freshness, while
    // scan_usable_ says whether the newest message produced valid occupancy.
    has_scan_ = true;
    scan_rx_ = node_->now();
    const PathReference * path = !path_.empty() ? &path_ :
      (!csv_path_.empty() ? &csv_path_ : nullptr);
    if (!path) {
      invalidateScan("scan received but no planned path is available");
      return;
    }
    if (!tf_buffer_) {
      invalidateScan("scan received but the transform buffer is unavailable");
      return;
    }
    if (msg->ranges.empty()) {
      invalidateScan("scan received with no ranges");
      return;
    }

    const double sweep = msg->scan_time > 0.0 ? msg->scan_time :
      msg->time_increment * static_cast<double>(msg->ranges.size());
    geometry_msgs::msg::TransformStamped t0, t1, ego;
    try {
      t0 = lookupOrLatest(path->frame(), msg->header.frame_id, msg->header.stamp);
      const rclcpp::Time end = rclcpp::Time(msg->header.stamp) +
        rclcpp::Duration::from_seconds(std::max(0.0, sweep));
      t1 = deskew_ ? lookupOrLatest(path->frame(), msg->header.frame_id, end.operator builtin_interfaces::msg::Time()) : t0;
      ego = lookupOrLatest(path->frame(), base_frame_, msg->header.stamp);
    } catch (const tf2::TransformException &) {
      // Scans ARE arriving; they cannot be placed. Saying "no scan" here sends
      // whoever reads it to check the lidar, which is working.
      invalidateScan("cannot place the scan on the plan (no transform)");
      return;
    }
    const auto e = path->project(ego.transform.translation.x, ego.transform.translation.y);
    if (!e.valid) {
      invalidateScan("the car does not project onto the plan");
      return;
    }
    scan_ego_station_ = e.station;

    const double span = std::clamp(
      trigger_time_ * lastSpeed(), min_trigger_, max_trigger_);
    const auto yaw_of = [](const geometry_msgs::msg::TransformStamped & tf) {
        const auto & q = tf.transform.rotation;
        return std::atan2(
          2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      };
    const double x0 = t0.transform.translation.x, y0 = t0.transform.translation.y;
    const double x1 = t1.transform.translation.x, y1 = t1.transform.translation.y;
    const double a0 = yaw_of(t0);
    double da = yaw_of(t1) - a0;
    while (da > M_PI) {da -= 2.0 * M_PI;}
    while (da < -M_PI) {da += 2.0 * M_PI;}

    const std::size_t n = msg->ranges.size();
    const int stride = std::max(1, scan_.settings().beam_stride);

    // Place every usable beam first, then cut the scan into surfaces IN BEAM
    // ORDER. Both have to happen before anything is thrown away: a surface's
    // size is only meaningful measured across the whole of it, and the part
    // near the line is exactly the part that says nothing about how big it is.
    struct Beam
    {
      double x, y, range, side;
      bool usable;      // near enough to matter; a far one still holds its surface together
    };
    std::vector<Beam> beams;
    std::vector<int> breaks;             // index in `beams` where a surface starts
    beams.reserve(n / stride);
    // Borges and Aldon: neighbouring returns are one surface while they are
    // closer than a surface seen at `grazing_` would put them, plus noise.
    const double dphi = std::abs(msg->angle_increment) * static_cast<double>(stride);
    const double slope = (grazing_ > dphi + 1e-6) ?
      std::sin(dphi) / std::sin(grazing_ - dphi) : 1.0;
    bool have_prev = false;
    for (std::size_t k = 0; k < n; k += stride) {
      const double d = msg->ranges[k];
      if (!std::isfinite(d) || d <= msg->range_min || d >= msg->range_max) {
        have_prev = false;             // a real hole in the scan ends the surface
        continue;
      }
      // A return beyond the lookahead is not worth judging, but it is still
      // part of whatever surface it sits on, and dropping it here would cut
      // that surface at the edge of the window - which is exactly how a long
      // wall would come out looking small enough to be an obstacle.
      const double f = static_cast<double>(k) /
        static_cast<double>(std::max<std::size_t>(1, n - 1));
      const double cyaw = a0 + f * da;
      const double a = msg->angle_min + static_cast<double>(k) * msg->angle_increment;
      const double bx = d * std::cos(a), by = d * std::sin(a);
      Beam b;
      b.x = x0 + f * (x1 - x0) + std::cos(cyaw) * bx - std::sin(cyaw) * by;
      b.y = y0 + f * (y1 - y0) + std::sin(cyaw) * bx + std::cos(cyaw) * by;
      b.range = d;
      b.side = std::abs(by);
      b.usable = d <= span + 2.0;
      if (!have_prev) {
        breaks.push_back(static_cast<int>(beams.size()));
      } else {
        const Beam & q = beams.back();
        const double gap = std::hypot(b.x - q.x, b.y - q.y);
        if (gap > q.range * slope + 3.0 * range_noise_) {
          breaks.push_back(static_cast<int>(beams.size()));
        }
      }
      beams.push_back(b);
      have_prev = true;
    }
    // How big each surface is, measured ALONG it rather than end to end. A wall
    // that wraps round a hairpin has its two ends close together, so a straight
    // line between them says it is small when it is the largest thing in view.
    std::vector<int> segment_of(beams.size(), -1);
    std::vector<double> segment_size;
    for (std::size_t sgi = 0; sgi < breaks.size(); ++sgi) {
      const int from = breaks[sgi];
      const int to = (sgi + 1 < breaks.size()) ?
        breaks[sgi + 1] : static_cast<int>(beams.size());
      double len = 0.0;
      for (int k = from + 1; k < to; ++k) {
        len += std::hypot(beams[k].x - beams[k - 1].x, beams[k].y - beams[k - 1].y);
      }
      for (int k = from; k < to; ++k) {segment_of[k] = static_cast<int>(sgi);}
      segment_size.push_back(len);
    }

    // Legacy mode assigns every beam the size of its complete adaptive-breakpoint
    // surface. Experimental polyline mode touches only surfaces that legacy
    // would discard, recursively splitting them into locally straight pieces.
    // A wall remains a long piece and is still removed; a protrusion attached
    // to it gets its own short piece. The fit is only a partition: occupancy is
    // always measured at the raw deskewed return, because moving a return onto
    // a fitted wall would recreate the information loss of the cluster source.
    std::vector<double> judged_size(beams.size(), 0.0);
    std::vector<bool> recovered_from_large(beams.size(), false);
    for (std::size_t k = 0; k < beams.size(); ++k) {
      judged_size[k] = segment_size[segment_of[k]];
    }
    if (surface_model_ == "polyline" && scan_.settings().biggest_thing_m > 0.0) {
      int next_segment = static_cast<int>(segment_size.size());
      for (std::size_t sgi = 0; sgi < breaks.size(); ++sgi) {
        if (segment_size[sgi] <= scan_.settings().biggest_thing_m) {continue;}
        const int from = breaks[sgi];
        const int to = (sgi + 1 < breaks.size()) ?
          breaks[sgi + 1] : static_cast<int>(beams.size());
        for (int k = from; k < to; ++k) {
          recovered_from_large[static_cast<std::size_t>(k)] = true;
        }
        std::vector<SurfaceFitPoint> surface;
        surface.reserve(static_cast<std::size_t>(to - from));
        for (int k = from; k < to; ++k) {
          surface.push_back({beams[k].x, beams[k].y, beams[k].range});
        }
        const auto primitives = splitSurfacePolyline(
          surface, surface_fit_error_, surface_fit_error_per_m_, surface_fit_min_points_);
        for (std::size_t pi = 0; pi < primitives.size(); ++pi) {
          const auto & primitive = primitives[pi];
          const int primitive_id = next_segment++;
          for (std::size_t local = primitive.begin; local < primitive.end; ++local) {
            const std::size_t k = static_cast<std::size_t>(from) + local;
            judged_size[k] = primitive.length;
            segment_of[k] = primitive_id;
          }
        }
      }
    }

    std::vector<ScanOccupancy::Point> pts;
    pts.reserve(beams.size());
    for (std::size_t k = 0; k < beams.size(); ++k) {
      if (!beams[k].usable) {continue;}
      const auto pr = path->project(beams[k].x, beams[k].y, e.station, span);
      if (!pr.valid) {continue;}
      ScanOccupancy::Point p;
      p.lateral = pr.lateral;
      if (p.lateral > scan_.settings().near_line_m * 2.0) {continue;}   // cheap cull
      p.side = beams[k].side;      // how far out to the side of the car it is
      p.along = path->forwardDistance(e.station, pr.station);
      p.range = beams[k].range;
      p.segment = segment_of[k];
      p.segment_size = judged_size[k];
      p.recovered_from_large = recovered_from_large[k];
      p.preserve_if_large = surface_model_ == "swept";
      pts.push_back(p);
    }
    scan_.push(std::move(pts));
    scan_header_ = msg->header;   // what the path lookup gets stamped with
    scan_blocked_.clear();
    scan_usable_ = true;
  }

  // Called only with mtx_ held. A failed latest scan invalidates the complete
  // accumulation window: keeping older occupancy would turn a fail-open input
  // into a stale obstacle veto even though the status says the scan is blind.
  void invalidateScan(const std::string & why)
  {
    scan_.clear();
    scan_usable_ = false;
    scan_blocked_ = why;
    ++scan_invalidation_generation_;

    // Invalidation is also a transaction boundary for every decision derived
    // from the old ring. If it lands after a score committed, clear that state
    // here; if it lands before commit, the generation check rejects the stale
    // result. Both interleavings therefore end in the same fail-open state.
    state_.clear();
    last_on_plan_.clear();
    last_passed_.clear();
    have_block_station_.clear();
    block_station_.clear();
    last_ahead_.clear();
    hit_.clear();
    have_hit_.clear();
  }

  // score() calls this while mtx_ is held. Failing open must also discard the
  // per-drive commitment latch; otherwise an obstacle seen before blindness
  // can reappear as a held veto as soon as an empty, valid scan recovers.
  void resetScanDecision(const std::string & drive)
  {
    state_.erase(drive);
    last_on_plan_.erase(drive);
    last_passed_.erase(drive);
    have_block_station_.erase(drive);
    block_station_.erase(drive);
    last_ahead_.erase(drive);
    hit_.erase(drive);
    have_hit_.erase(drive);
  }

  // Called after scan geometry has been computed locally. The generation
  // validation and setState() must share this lock with invalidateScan().
  ScoreResult setScanStateIfCurrent(
    std::uint64_t generation, const std::string & drive, const Context & ctx,
    double demand, double dist, int considered, const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!scan_usable_ || generation != scan_invalidation_generation_) {
      return discardStaleScanDecision(drive);
    }
    return setState(drive, ctx, demand, dist, considered, reason);
  }

  // mtx_ is held by the caller. invalidateScan() has already cleared all
  // decisions, but erase this key as well to keep the helper correct if it is
  // ever used for a non-usability transition that does not clear globally.
  ScoreResult discardStaleScanDecision(const std::string & drive)
  {
    resetScanDecision(drive);
    const std::string why = scan_blocked_.empty() ?
      "scan was invalidated while scoring" : scan_blocked_;
    return ScoreResult::ok(1.0, why + " - stale scan decision discarded");
  }

  // Speed last used for the geometry, so the scan callback sizes its window the
  // same way the decision does.
  double lastSpeed() const {return last_speed_ > 0.0 ? last_speed_ : 1.0;}

  // Resolve the plan and the transform for this scan. Returns false - and says
  // why, throttled - whenever the plan cannot be used, so the caller falls back
  // to the commanded arc rather than silently reporting a clear path.
  bool resolvePathRef(
    const std_msgs::msg::Header & header, const PathReference * path, PathRef * out)
  {
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
    double span, double * station, double * lateral) const
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
        const auto pr = ref.path->project(mx, my, ref.ego_station, span);
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
    State & st = state_[drive];
    // Rolling window of what each recent cycle saw.
    // The scan source has already required a range- and speed-dependent number
    // of scans to agree before reporting anything - see scan_occupancy.hpp.
    // Running this second window on top of it asks the same question twice and
    // only delays the answer, so for that source the test is the current cycle.
    st.seen.push_back(
      {ctx.now, demand > 0.0, block_station_[drive], have_block_station_[drive]});
    if (source_ == "scan") {
      while (st.seen.size() > 1) {st.seen.pop_front();}
    }
    while (!st.seen.empty() && (ctx.now - st.seen.front().when).seconds() > window_) {
      st.seen.pop_front();
    }
    // Presence counts only the samples that agree on WHERE. The reference is
    // the median of the located ones, so a run of consistent sightings is not
    // outvoted by a couple of wild ones.
    std::vector<double> located;
    for (const auto & s : st.seen) {
      if (s.present && s.located) {located.push_back(s.station);}
    }
    double ref = 0.0;
    if (!located.empty()) {
      std::nth_element(located.begin(), located.begin() + located.size() / 2, located.end());
      ref = located[located.size() / 2];
    }
    int hits = 0;
    for (const auto & s : st.seen) {
      if (!s.present) {continue;}
      // Judged on the arc, with no place on the plan to compare - count it,
      // since refusing would make the fallback path stricter than the main one.
      if (!s.located || located.empty() || std::abs(s.station - ref) <= station_tol_) {
        ++hits;
      }
    }
    const double presence = st.seen.empty() ?
      0.0 : static_cast<double>(hits) / static_cast<double>(st.seen.size());
    // Hysteresis on the fraction, so a detection sitting near the bar does not
    // toggle the latch every cycle.
    if (presence >= enter_frac_) {
      st.present = true;
    } else if (presence <= exit_frac_) {
      st.present = false;
    }
    st.presence = presence;
    const bool clear_now = !st.present;
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

    char buf[180];
    if (st.blocked) {
      if (clear_now) {
        std::snprintf(
          buf, sizeof(buf), "%s, holding %.1f/%.1fs",
          by_station ? "obstacle passed" :
          ((remember_cap_ > 0.0 && still_ahead) ? "not past it yet" : "path clearing"),
          dwell, cap);
        return ScoreResult::ok(0.0, buf);        // stay committed to the detour
      }
      if (!std::isfinite(dist)) {
        // The window is holding through a gap in detection - nothing is being
        // reported this cycle, so there is no distance to quote.
        std::snprintf(
          buf, sizeof(buf), "holding through a gap, seen %.0f%% of %.1fs",
          100.0 * st.presence, window_);
        return ScoreResult::ok(std::clamp(1.0 - demand, 0.0, 1.0), buf);
      }
      if (source_ == "scan" && have_hit_[drive]) {
        const auto & h = hit_[drive];
        std::snprintf(
          buf, sizeof(buf),
          "%.2fm along, %.2fm away, %.2fm off, %.2fm long, %d/%d scans, "
          "demand %.2f%s",
          h.along, h.range, h.lateral, h.extent, h.scans_agreed, h.scans_needed,
          demand, st.committed ? ", committed" : "");
      } else {
        std::snprintf(
          buf, sizeof(buf), "obstacle at %.2fm %s, demand %.2f%s", dist,
          last_on_plan_[drive] ? "on the plan" : "on the commanded arc", demand,
          st.committed ? ", committed" : "");
      }
      // While committed the drive stays disqualified even as the demand eases,
      // so one obstacle costs one handover rather than one per threshold
      // crossing.
      const double s = std::clamp(1.0 - demand, 0.0, 1.0);
      return ScoreResult::ok(st.committed ? std::min(s, 0.0) : s, buf);
    }
    if (!clear_now) {
      if (source_ == "scan" && have_hit_[drive]) {
        const auto & h = hit_[drive];
        std::snprintf(
          buf, sizeof(buf),
          "%.2fm along, %.2fm away, %.2fm off, %.2fm long, %d/%d scans, "
          "demand %.2f, confirming %.1f/%.1fs",
          h.along, h.range, h.lateral, h.extent, h.scans_agreed, h.scans_needed,
          demand, dwell, block_);
      } else {
        std::snprintf(
          buf, sizeof(buf), "obstacle at %.2fm %s, demand %.2f, confirming %.1f/%.1fs",
          dist, last_on_plan_[drive] ? "on the plan" : "on the arc", demand, dwell, block_);
      }
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

  struct Seen
  {
    rclcpp::Time when;
    bool present{false};
    double station{0.0};      // where on the plan it was, when present
    bool located{false};      // false when judged on the arc, with no station
  };

  struct State
  {
    std::deque<Seen> seen;
    bool present{false};
    double presence{0.0};
    bool was_clear{true};
    bool valid{false};
    bool blocked{false};
    bool committed{false};
    rclcpp::Time edge;
  };

  rclcpp::Node * node_{nullptr};
  std::string reference_drive_;
  std::string resolved_reference_drive_;
  bool have_reference_command_{false};
  double reference_speed_{0.0};
  double reference_steering_{0.0};
  bool geometry_snapshot_valid_{false};
  rclcpp::Time geometry_snapshot_stamp_;
  bool geometry_snapshot_has_speed_{false};
  double geometry_snapshot_speed_age_{0.0};
  double geometry_snapshot_measured_speed_{0.0};
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr speed_sub_;
  std::string vesc_state_topic_;
  VescSpeedCalibration speed_calibration_;
  double speed_timeout_{0.3};
  bool has_speed_{false};
  double measured_speed_{0.0};
  rclcpp::Time speed_stamp_;
  rclcpp::Subscription<obstacle_context_msgs::msg::ObstacleClusterArray>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  ScanOccupancy scan_;
  std_msgs::msg::Header scan_header_;
  // Why the last scan was dropped before it reached the ring, if it was.
  std::string scan_blocked_;
  // What the scan source is actually looking at, per drive. Diagnostic only -
  // "obstacle at 0.24m" says how far along the line it is and nothing about
  // what it is, and on a hairpin the wall beside the car projects onto the leg
  // ahead at exactly that sort of distance. Reporting the range, the offset,
  // the length and the vote separates those cases without another replay.
  std::map<std::string, ScanOccupancy::Occupied> hit_;
  std::map<std::string, bool> have_hit_;
  double grazing_{10.0 * M_PI / 180.0};   // most oblique a surface may be seen at
  double range_noise_{0.03};              // lidar range noise, one sigma
  std::string surface_model_{"size"};  // size, polyline split, or raw swept-corridor guard
  double surface_fit_error_{0.05};
  double surface_fit_error_per_m_{0.0};
  int surface_fit_min_points_{2};
  std::string source_{"clusters"};
  std::string scan_topic_{"/scan"};
  bool deskew_{true};
  double scan_ego_station_{0.0};
  rclcpp::Time scan_rx_;
  bool has_scan_{false};
  bool scan_usable_{false};
  std::uint64_t scan_invalidation_generation_{0};
  bool scan_evaluation_valid_{false};
  rclcpp::Time scan_evaluation_stamp_;
  bool scan_evaluation_has_scan_{false};
  bool scan_evaluation_usable_{false};
  rclcpp::Time scan_evaluation_rx_;
  std::string scan_evaluation_blocked_;
  ScanOccupancy scan_evaluation_snapshot_;
  std_msgs::msg::Header scan_evaluation_header_;
  std::uint64_t scan_evaluation_invalidation_generation_{0};
  PathReference scan_evaluation_path_;
  bool scan_evaluation_have_path_{false};
  PathRef scan_evaluation_path_ref_;
  bool scan_evaluation_on_plan_{false};
  double last_speed_{0.0};
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
  double wall_margin_cut_{0.10};
  double max_width_{0.0};
  int min_points_{0};
  double sample_step_{0.08};
  double block_{0.2};
  double window_{0.9};
  double enter_frac_{0.55};
  double exit_frac_{0.20};
  double station_tol_{1.0};
  double clear_{0.7};
  double timeout_{0.5};

  std::mutex mtx_;
  obstacle_context_msgs::msg::ObstacleClusterArray::ConstSharedPtr clusters_;
  rclcpp::Time rx_;
  std::map<std::string, State> state_;
};

CO_DRIVER_REGISTER_SCORER(ObstacleAvoidScorer, "obstacle_avoid")

}  // namespace co_driver
