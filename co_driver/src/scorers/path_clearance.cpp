// Path clearance - the drive is steering into solid geometry, so its idea of
// where it is must be wrong.
//
// This is NOT obstacle detection and NOT obstacle avoidance. It never decides
// that anything is an obstacle - it asks only whether the geometry the lidar
// reports is compatible with the path this drive is commanding. Deciding what
// counts as an obstacle belongs to the obstacle detector, and reaches the
// arbitration through obstacle_avoid.
//
// The difference between the two is the whole point of having both. obstacle_avoid handles "the map is right, the raceline is
// right, something is lying in the way" - a detour, taken and given back. This
// one handles "the commanded path leads into structure", which on a map-based
// controller means its world model is wrong, not that the world changed.
//
//   obstacle_avoid   localization fine, path fine, something in the way
//                    -> go around it, come back promptly, nothing was wrong
//                       with the drive itself
//   path_clearance   the drive is commanding a path into solid geometry
//                    -> it has just demonstrated it cannot be trusted, so
//                       coming back is deliberately reluctant
//
// That asymmetry lives in clear_ms, and it has to clear a floor to exist at
// all: any return costs about 2.2 s regardless (switch cooldown plus the
// selection margin), so a clear time under that is simply absorbed. This one
// is set well above the floor; obstacle_avoid's sits below it, where the floor
// is the binding constraint and the detour comes back as fast as the
// arbitration allows.
//
// Every other judgement in this arbitration asks the localization whether it
// believes itself. This one does not ask anyone: it projects the drive's own
// command forward and checks it against the live scan. That makes it the only
// detector that survives a localization which is confidently wrong.
//
// The case it exists for: a map where a straight section forks and the two
// exits look alike. Global search attaches to the wrong branch. The straight
// walls still match, so only the diverging sector disagrees - and the producer
// masks exactly those beams (their particle consensus is zero), which leaves
// term_score and term_outlier computed on the surviving, matching beams at
// ~1.000. The whole disagreement lands in term_skip, and term_skip is excluded
// from the score to keep passing vehicles from causing handovers. So the
// confidence pathway reports a healthy localization while the raceline
// controller steers into the wrong branch. Only geometry catches that.
//
// The check, in the scan frame, with no map and no TF: take the arc the
// command describes (from the wheelbase and the steering angle) and ask, for
// every scan return, how far it sits from that arc sideways and how far along
// the arc it lies. A return within half a vehicle width of the arc blocks it,
// at its own arc length. The nearest such blockage is the free distance.
//
// Measuring each return against the arc, rather than sweeping an angular
// corridor outward, is what keeps the near field sane: a corridor half a
// vehicle wide subtends about 74 degrees at 5 cm, so an angular test would let
// any close return anywhere in front veto every possible command.
//
// The projection stops after max_sweep_deg of heading change. A command is a
// snapshot, not a trajectory: at full lock the arc is a 30 cm circle, so
// following it far enough curls the corridor around into whatever is beside or
// behind the car, and the check stops answering "am I about to drive into
// something" and starts answering "if I pirouette, will I clip my own tail".
// On the 0813 recording that produced two vetoes totalling 2.3 s from a fixed
// object 0.75 m off the right flank, reached only after 140 degrees of sweep,
// while the detector and the raw scan both showed the path ahead open at
// 2.86 m. Any cap at or below 120 degrees removes them; gentle steering
// sweeps only a few degrees over the whole horizon, so normal driving is
// untouched.
//
// Blocked and clear are both sustained, for the same reason the confidence
// trip sustains: one noisy sweep is not evidence. block_ms is short because
// this fires late by nature - the car is already pointed at the obstacle.
//
//   {
//     "name": "clearance", "type": "path_clearance",
//     "params": {"topic": "/scan", "min_clearance_m": 0.8, "block_ms": 200}
//   }
//   with, on the map-based drive only:
//     "clearance": {"weight": 0.0, "veto_below": 0.5}
//
// The score is computed for every drive, so status shows what each candidate's
// own command would run into - but only the drive whose influence carries a
// veto_below is actually disqualified. Vetoing every candidate at once would
// leave nothing to drive with and hand the car to timeout_stop, which is a
// deliberate decision to make explicitly rather than by accident.
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "co_driver/arc_geometry.hpp"
#include "co_driver/scorer.hpp"
#include "co_driver/vesc_speed.hpp"

namespace co_driver
{

class PathClearanceScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    topic_ = jstr(p, "topic", "/scan");
    wheelbase_ = jnum(p, "wheelbase", 0.324);
    half_width_ = jnum(p, "vehicle_half_width", 0.18);
    lookahead_time_ = jnum(p, "lookahead_time", 1.0);
    min_lookahead_ = jnum(p, "min_lookahead_m", 0.5);
    max_lookahead_ = jnum(p, "max_lookahead_m", 3.0);
    step_ = std::max(0.02, jnum(p, "step_m", 0.05));
    min_clearance_ = jnum(p, "min_clearance_m", 0.8);
    // Speed for the geometry: MEASURED, not commanded.
    //
    // Keep this localization-independent just like the scan geometry itself.
    // The conversion and the separate 2.6 correction match obstacle_avoid and
    // PPcontroller; the command is used only while VESC telemetry is stale.
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
        "path clearance '%s': invalid VESC speed calibration "
        "(gain %.6f, offset %.6f, deadband %.3f, scale %.3f)",
        name.c_str(), speed_calibration_.speed_to_erpm_gain,
        speed_calibration_.speed_to_erpm_offset,
        speed_calibration_.wheel_speed_deadband,
        speed_calibration_.wheel_speed_scale);
      return false;
    }
    speed_timeout_ = jms(p, "speed_timeout_ms", 300.0);
    min_speed_ = jnum(p, "min_speed", 0.2);
    max_sweep_ = jnum(p, "max_sweep_deg", 90.0) * M_PI / 180.0;
    block_ = jms(p, "block_ms", 200.0);
    // Deliberately long, and long enough to matter: returning to a drive
    // costs about 2.2 s anyway (switch cooldown plus the selection margin), so
    // a clear time below that is absorbed by the floor and changes nothing.
    // Measured: at 2000 ms this returned in 2.11 s against the obstacle
    // detour's 2.25 s - no asymmetry at all, despite the intent.
    clear_ = jms(p, "clear_ms", 5000.0);
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
              "path clearance: invalid VESC speed sample on %s",
              vesc_state_topic_.c_str());
            return;
          }
          std::lock_guard<std::mutex> lock(mtx_);
          measured_speed_ = speed;
          speed_stamp_ = node_->now();
          has_speed_ = true;
        }, opts);
    }
    // Lidar drivers and rosbag replays publish BEST_EFFORT; a RELIABLE
    // subscription here would silently receive nothing.
    sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        scan_ = msg;
        stamp_ = node_->now();
      }, opts);

    RCLCPP_INFO(
      node->get_logger(),
      "path clearance '%s' <- %s (block below %.2fm sustained %.0fms, "
      "sweep capped at %.0fdeg)",
      name.c_str(), topic_.c_str(), min_clearance_, block_ * 1e3,
      max_sweep_ * 180.0 / M_PI);
    if (!vesc_state_topic_.empty()) {
      RCLCPP_INFO(
        node->get_logger(),
        "path clearance speed <- %s: (ERPM - %.2f) / %.2f, "
        "deadband %.2fm/s, x%.2f; command fallback after %.0fms",
        vesc_state_topic_.c_str(), speed_calibration_.speed_to_erpm_offset,
        speed_calibration_.speed_to_erpm_gain,
        speed_calibration_.wheel_speed_deadband,
        speed_calibration_.wheel_speed_scale, speed_timeout_ * 1e3);
    }
    return true;
  }

  ScoreResult score(const Drive & drive, const Context & ctx) override
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!scan_) {return ScoreResult::unavailable("no scan on " + topic_);}
      if (timeout_ > 0.0 && (ctx.now - stamp_).seconds() > timeout_) {
        return ScoreResult::unavailable("stale scan");
      }
      scan = scan_;
    }

    const double v = geometrySpeed(drive.cmd.drive.speed, ctx);
    const double delta = drive.cmd.drive.steering_angle;
    if (!std::isfinite(v) || !std::isfinite(delta)) {
      return ScoreResult::unavailable("command is NaN/inf");
    }
    // A stopped or crawling command cannot run into anything soon; judging it
    // would only produce noise while the car is manoeuvring at low speed.
    if (std::abs(v) < min_speed_) {
      return setState(drive.name, ctx, true, max_lookahead_, "below min_speed");
    }

    // Never look less far than the clearance we demand. freeDistance reports
    // the horizon when nothing blocks, so a horizon shorter than
    // min_clearance_m would read as "blocked" on an empty scan - which at
    // 0.2-0.8 m/s vetoed the drive continuously regardless of what the lidar
    // saw. Measured on the 0813 recording: 9 of 11 vetoes came from this
    // alone, on 361 frames that had no return on the path at all.
    const double horizon = std::max(
      std::clamp(lookahead_time_ * std::abs(v), min_lookahead_, max_lookahead_),
      min_clearance_);
    const double free_m = freeDistance(*scan, delta, horizon);
    return setState(drive.name, ctx, free_m >= min_clearance_, free_m, "");
  }

private:
  // Arc length at which the commanded path first meets a scan return that is
  // within half a vehicle width of it.
  // Corrected VESC speed if it is fresh, otherwise the candidate command.
  double geometrySpeed(double commanded, const Context & ctx)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const double age = has_speed_ ? (ctx.now - speed_stamp_).seconds() : 0.0;
    const bool fresh = has_speed_ &&
      (speed_timeout_ <= 0.0 || age <= speed_timeout_);
    if (!fresh && !vesc_state_topic_.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "path clearance: VESC speed on %s is %s; using candidate command %.2fm/s",
        vesc_state_topic_.c_str(), has_speed_ ? "stale" : "not received",
        std::abs(commanded));
    }
    return selectGeometrySpeed(
      has_speed_, age, speed_timeout_, measured_speed_, commanded);
  }

  double freeDistance(
    const sensor_msgs::msg::LaserScan & scan, double delta, double horizon) const
  {
    if (scan.ranges.empty() || scan.angle_increment <= 0.0) {return horizon;}
    const auto arc = ArcProjection::fromCommand(delta, wheelbase_, horizon, max_sweep_);

    double free_m = horizon;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double r = scan.ranges[i];
      // Dropouts arrive as NaN/inf or out of band; absence of a return is not
      // evidence of free space, so they simply do not count.
      if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {continue;}
      const double a = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      double along, lateral;
      if (!arc.project(r * std::cos(a), r * std::sin(a), &along, &lateral)) {continue;}
      if (along < free_m && lateral <= half_width_) {free_m = along;}
    }
    return free_m;
  }

  // Per-drive hysteresis. One edge timestamp per drive, stamped when the
  // direction changes, so nothing is ever read before it has been assigned.
  ScoreResult setState(
    const std::string & drive, const Context & ctx, bool clear_now, double free_m,
    const std::string & reason)
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

    char buf[96];
    if (st.blocked) {
      std::snprintf(
        buf, sizeof(buf), clear_now ? "blocked: clearing %.1f/%.1fs" :
        "blocked: %.2fm ahead (need %.2fm)",
        clear_now ? dwell : free_m, clear_now ? clear_: min_clearance_);
      return ScoreResult::ok(0.0, buf);
    }
    if (!clear_now) {
      std::snprintf(buf, sizeof(buf), "closing: %.2fm ahead %.1f/%.1fs", free_m, dwell, block_);
      return ScoreResult::ok(1.0, buf);
    }
    if (!reason.empty()) {return ScoreResult::ok(1.0, reason);}
    std::snprintf(buf, sizeof(buf), "clear %.2fm", free_m);
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
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr speed_sub_;
  std::string vesc_state_topic_;
  VescSpeedCalibration speed_calibration_;
  double speed_timeout_{0.3};
  bool has_speed_{false};
  double measured_speed_{0.0};
  rclcpp::Time speed_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

  std::string topic_;
  double wheelbase_{0.324};
  double half_width_{0.18};
  double lookahead_time_{1.0};
  double min_lookahead_{0.5};
  double max_lookahead_{3.0};
  double step_{0.05};
  double min_clearance_{0.8};
  double min_speed_{0.2};

  double max_sweep_{M_PI / 2.0};
  double block_{0.2};
  double clear_{5.0};
  double timeout_{0.5};

  std::mutex mtx_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  rclcpp::Time stamp_;
  std::map<std::string, State> state_;
};

CO_DRIVER_REGISTER_SCORER(PathClearanceScorer, "path_clearance")

}  // namespace co_driver
