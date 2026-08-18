// co_driver main node.
//
//   /drive_* subs --> scorers (src/scorers/) --> compute --> select --> postprocess --> /drive
//
// This node subscribes to **no sensors.** Judging from scan/map/imu/odom is done
// by each scorer through its own subscriptions. The node does only four things:
//   1) read config, create the scorers and the /drive subscriptions
//   2) each evaluation tick: score -> compute -> select
//   3) each output tick: postprocess -> publish /drive
//   4) publish diagnostics / hot-reload service
//
// Evaluation (may be slow) and output (fixed 100Hz) run on separate timers, so
// heavy scoring cannot disturb the output rate.
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "co_driver/compute.hpp"
#include "co_driver/config.hpp"
#include "co_driver/scorer.hpp"

namespace co_driver
{

namespace
{

constexpr double kDeg2Rad = M_PI / 180.0;

std::string esc(const std::string & s)
{
  std::string out;
  for (char c : s) {
    if (c == '"') {out += "\\\"";} else if (c == '\\') {out += "\\\\";} else if (c == '\n') {
      out += "\\n";
    } else {out += c;}
  }
  return out;
}

std::string num(double v)
{
  if (!std::isfinite(v)) {return "null";}
  std::ostringstream os;
  os.precision(6);              // so softmax probabilities read as summing to 1
  os << std::fixed << v;
  return os.str();
}

}  // namespace

// ===========================================================================
// Selection -- which drive to use. Hysteresis + minimum hold time against chattering.
// ===========================================================================
class Selector
{
public:
  struct Result
  {
    bool has_selection{false};
    std::string name;          // the drive actually used
    std::string reason;
    bool switched{false};
    // Score runner-up -- ranked by combined score only. Independent of the
    // selection (name); with hysteresis, if selected is not score rank 1 the
    // runner-up may equal selected. (Per-drive ranks are all in status "rank".)
    std::string runner_up;
    bool has_runner_up{false};
  };

  void configure(const SelectionSpec & spec) {spec_ = spec;}
  const std::string & current() const {return current_;}

  Result select(const std::vector<Drive> & drives, const rclcpp::Time & now)
  {
    Result r = selectFirst(drives, now);

    // Score ranking -- order valid drives by combined score only, take ranks 1 and 2.
    // The selection (r.name) carries hysteresis and may differ from rank 1; this ignores that.
    const Drive * first = nullptr;
    const Drive * second = nullptr;
    for (const auto & d : drives) {
      if (!d.valid) {continue;}
      if (!first || d.score > first->score) {
        second = first;
        first = &d;
      } else if (!second || d.score > second->score) {
        second = &d;
      }
    }
    if (second) {
      r.has_runner_up = true;
      r.runner_up = second->name;
    }
    return r;
  }

private:
  Result selectFirst(const std::vector<Drive> & drives, const rclcpp::Time & now)
  {
    Result r;

    // valid already implies active (inside the trust window + hard gates passed),
    // so everything considered here is a currently-live drive.
    const Drive * incumbent = nullptr;
    for (const auto & d : drives) {
      if (d.name == current_) {incumbent = &d; break;}
    }
    const bool incumbent_ok = incumbent && incumbent->valid;

    const Drive * best = nullptr;
    for (const auto & d : drives) {
      if (!d.valid) {continue;}
      if (!best || d.score > best->score) {best = &d;}
    }

    if (!best) {
      // nothing usable -> try the fallback
      if (!spec_.fallback.empty()) {
        for (const auto & d : drives) {
          if (d.name != spec_.fallback) {continue;}
          // The fallback must itself be valid (= active); never revive a dead command.
          if (d.valid) {return commit(d.name, "fallback", now);}
        }
      }
      r.reason = "no valid drive";
      current_.clear();
      return r;
    }

    if (!incumbent_ok) {
      // If the incumbent died, switch immediately without hysteresis.
      return commit(best->name, current_.empty() ? "initial selection" : "incumbent invalid -> immediate switch", now);
    }

    r.has_selection = true;
    if (best->name == current_) {
      r.name = current_;
      r.reason = "hold";
      return r;
    }

    const double held = switched_once_ ?
      (now - last_switch_).seconds() : spec_.switch_cooldown + 1.0;
    if (spec_.switch_cooldown > 0.0 && held < spec_.switch_cooldown) {
      r.name = current_;
      r.reason = "waiting on switch_cooldown";
      return r;
    }
    if (best->score < incumbent->score + spec_.switch_margin) {
      r.name = current_;
      r.reason = "switch_margin not met";
      return r;
    }
    return commit(best->name, "switched on score margin", now);
  }

  Result commit(const std::string & name, const std::string & reason, const rclcpp::Time & now)
  {
    Result r;
    r.has_selection = true;
    r.name = name;
    r.reason = reason;
    r.switched = (current_ != name);
    if (r.switched) {
      current_ = name;
      last_switch_ = now;
      switched_once_ = true;
    }
    return r;
  }

  SelectionSpec spec_;
  std::string current_;
  rclcpp::Time last_switch_;
  bool switched_once_{false};
};

// ===========================================================================
// Postprocess -- refines the final command in yaml postprocess.pipeline order.
// To add a stage, add one case each to the enum, the parsing, and apply below.
// ===========================================================================
struct PostContext
{
  double dt{0.0};
  bool has_command{false};   // was there a usable drive this cycle
  bool switched{false};      // did the selection change this cycle
  ackermann_msgs::msg::AckermannDrive previous;
  bool has_previous{false};
};

class PostProcess
{
public:
  bool configure(rclcpp::Node * node, const std::vector<StageSpec> & specs, rclcpp::Logger log)
  {
    node_ = node;
    stages_.clear();
    if (specs.empty()) {
      RCLCPP_WARN(log, "postprocess is empty. Publishing the command unmodified.");
      return true;
    }
    for (const auto & spec : specs) {
      Stage st;
      st.name = spec.name;
      const Json & p = spec.params;

      if (spec.type == "timeout_stop") {
        st.type = Type::TimeoutStop;
        st.decel = jnum(p, "decel", 12.0);
        st.hold_steer = jbool(p, "hold_steer", true);
      } else if (spec.type == "switch_blend") {
        st.type = Type::SwitchBlend;
        st.duration = jms(p, "duration_ms", 250.0);
        st.curve = jstr(p, "curve", "smooth");
        st.ema_alpha = std::clamp(jnum(p, "ema_alpha", 0.2), 0.0, 1.0);
        if (st.curve != "linear" && st.curve != "smooth" && st.curve != "ema") {
          RCLCPP_WARN(log, "unknown switch_blend.curve '%s' -> smooth", st.curve.c_str());
          st.curve = "smooth";
        }
      } else if (spec.type == "rate_limit") {
        st.type = Type::RateLimit;
        st.max_steer_rate_deg = jnum(p, "max_steer_rate_deg", 400.0);
        st.max_accel = jnum(p, "max_accel", 6.0);
        st.max_decel = jnum(p, "max_decel", 12.0);
      } else if (spec.type == "speed_scale") {
        st.type = Type::SpeedScale;
        st.scale = jnum(p, "scale", 1.0);
        // Declared as a parameter so it can be changed at runtime via ros2 param set.
        st.scale_key = "postprocess." + spec.name + ".scale";
        if (node_->has_parameter(st.scale_key)) {
          node_->set_parameter(rclcpp::Parameter(st.scale_key, st.scale));
        } else {
          node_->declare_parameter(st.scale_key, st.scale);
        }
      } else if (spec.type == "deadband") {
        st.type = Type::Deadband;
        st.speed_deadband = jnum(p, "speed", 0.0);
        st.steer_deadband_deg = jnum(p, "steer_deg", 0.0);
      } else if (spec.type == "clamp") {
        st.type = Type::Clamp;
        st.max_steering_deg = jnum(p, "max_steering_deg", 50.0);
        st.max_speed = jnum(p, "max_speed", 8.0);
        st.min_speed = jnum(p, "min_speed", 0.0);
      } else {
        RCLCPP_ERROR(
          log, "postprocess '%s' has unknown type '%s'. Available: "
          "timeout_stop switch_blend rate_limit speed_scale deadband clamp",
          spec.name.c_str(), spec.type.c_str());
        return false;
      }
      stages_.push_back(st);
      RCLCPP_INFO(
        log, "postprocess[%zu] %s (type=%s)", stages_.size() - 1,
        spec.name.c_str(), spec.type.c_str());
    }
    return true;
  }

  void reset()
  {
    for (auto & s : stages_) {
      s.blend_active = false;
      s.blend_elapsed = 0.0;
    }
  }

  void apply(ackermann_msgs::msg::AckermannDrive & cmd, const PostContext & pc)
  {
    for (auto & s : stages_) {
      switch (s.type) {
        case Type::TimeoutStop: {
            if (pc.has_command) {break;}   // only intervenes when no drive is usable
            if (s.decel > 0.0 && pc.has_previous && pc.dt > 0.0) {
              const double step = s.decel * pc.dt;
              const double prev = pc.previous.speed;
              cmd.speed = static_cast<float>(
                (std::abs(prev) <= step) ? 0.0 : prev - std::copysign(step, prev));
            } else {
              cmd.speed = 0.0f;
            }
            cmd.steering_angle =
              (s.hold_steer && pc.has_previous) ? pc.previous.steering_angle : 0.0f;
            break;
          }

        case Type::SwitchBlend: {
            if (pc.switched && pc.has_previous) {
              // On another switch mid-blend, restart from the value currently going out.
              s.blend_from = pc.previous;
              s.blend_elapsed = 0.0;
              s.blend_active = true;
            }
            if (!s.blend_active) {break;}

            if (s.curve == "ema") {
              const double ts = cmd.steering_angle, tv = cmd.speed;
              s.blend_from.steering_angle +=
                static_cast<float>(s.ema_alpha * (ts - s.blend_from.steering_angle));
              s.blend_from.speed += static_cast<float>(s.ema_alpha * (tv - s.blend_from.speed));
              cmd.steering_angle = s.blend_from.steering_angle;
              cmd.speed = s.blend_from.speed;
              if (std::abs(ts - cmd.steering_angle) < 1e-4 && std::abs(tv - cmd.speed) < 1e-3) {
                s.blend_active = false;
              }
              break;
            }

            s.blend_elapsed += pc.dt;
            if (s.duration <= 0.0 || s.blend_elapsed >= s.duration) {
              s.blend_active = false;
              break;
            }
            double a = std::clamp(s.blend_elapsed / s.duration, 0.0, 1.0);
            if (s.curve == "smooth") {a = a * a * (3.0 - 2.0 * a);}   // smoothstep
            cmd.steering_angle = static_cast<float>(
              s.blend_from.steering_angle +
              a * (cmd.steering_angle - s.blend_from.steering_angle));
            cmd.speed = static_cast<float>(
              s.blend_from.speed + a * (cmd.speed - s.blend_from.speed));
            break;
          }

        case Type::RateLimit: {
            if (!pc.has_previous || pc.dt <= 0.0) {break;}
            if (s.max_steer_rate_deg > 0.0) {
              const double max_step = s.max_steer_rate_deg * kDeg2Rad * pc.dt;
              const double delta = cmd.steering_angle - pc.previous.steering_angle;
              if (std::abs(delta) > max_step) {
                cmd.steering_angle = static_cast<float>(
                  pc.previous.steering_angle + std::copysign(max_step, delta));
              }
            }
            // Accel vs decel is decided by whether |speed| grows (holds in reverse too).
            const double dv = cmd.speed - pc.previous.speed;
            const bool speeding_up = std::abs(cmd.speed) > std::abs(pc.previous.speed);
            const double limit = (speeding_up ? s.max_accel : s.max_decel) * pc.dt;
            if (limit > 0.0 && std::abs(dv) > limit) {
              cmd.speed = static_cast<float>(pc.previous.speed + std::copysign(limit, dv));
            }
            break;
          }

        case Type::SpeedScale: {
            // Read every cycle so runtime changes take effect immediately.
            double scale = s.scale;
            if (node_ && node_->has_parameter(s.scale_key)) {
              const auto p = node_->get_parameter(s.scale_key);
              if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                scale = p.as_double();
              } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                scale = static_cast<double>(p.as_int());
              }
            }
            cmd.speed = static_cast<float>(cmd.speed * scale);
            break;
          }

        case Type::Deadband: {
            if (s.speed_deadband > 0.0 && std::abs(cmd.speed) < s.speed_deadband) {
              cmd.speed = 0.0f;
            }
            if (s.steer_deadband_deg > 0.0 &&
              std::abs(cmd.steering_angle) < s.steer_deadband_deg * kDeg2Rad)
            {
              cmd.steering_angle = 0.0f;
            }
            break;
          }

        case Type::Clamp: {
            if (!std::isfinite(cmd.steering_angle)) {cmd.steering_angle = 0.0f;}
            if (!std::isfinite(cmd.speed)) {cmd.speed = 0.0f;}
            const double lim = s.max_steering_deg * kDeg2Rad;
            cmd.steering_angle = static_cast<float>(
              std::clamp(static_cast<double>(cmd.steering_angle), -lim, lim));
            cmd.speed = static_cast<float>(
              std::clamp(static_cast<double>(cmd.speed), s.min_speed, s.max_speed));
            break;
          }
      }
    }
  }

private:
  enum class Type { TimeoutStop, SwitchBlend, RateLimit, SpeedScale, Deadband, Clamp };

  struct Stage
  {
    Type type;
    std::string name;
    double decel{12.0};
    bool hold_steer{true};
    double duration{0.25};
    std::string curve{"smooth"};
    double ema_alpha{0.2};
    double max_steer_rate_deg{400.0};
    double max_accel{6.0};
    double max_decel{12.0};
    std::string scale_key;
    double scale{1.0};
    double speed_deadband{0.0};
    double steer_deadband_deg{0.0};
    double max_steering_deg{50.0};
    double max_speed{8.0};
    double min_speed{0.0};
    // switch_blend state
    bool blend_active{false};
    double blend_elapsed{0.0};
    ackermann_msgs::msg::AckermannDrive blend_from;
  };

  rclcpp::Node * node_{nullptr};
  std::vector<Stage> stages_;
};

// ===========================================================================
// Node
// ===========================================================================
class CoDriverNode : public rclcpp::Node
{
public:
  explicit CoDriverNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("co_driver_node", rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true))
  {
    // Base config comes from yaml (ROS parameters); growing lists come from this JSON.
    if (!has_parameter("topics_file")) {declare_parameter<std::string>("topics_file", "");}
    topics_path_ = get_parameter("topics_file").as_string();

    // Separate callback groups are needed for the MultiThreadedExecutor to actually
    // run in parallel. The default group is MutuallyExclusive; left alone, one heavy
    // scorer would also block the output timer and jitter the /drive rate.
    eval_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    output_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scorer_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    std::string err;
    if (!Config::load(this, topics_path_, &cfg_, &err)) {throw std::runtime_error(err);}
    if (!build(&err)) {throw std::runtime_error(err);}

    // --- /drive candidate subscriptions (the node's only subscriptions) ---
    drive_subs_.resize(drives_.size());
    for (std::size_t i = 0; i < drives_.size(); ++i) {
      const auto & d = drives_[i];
      drive_subs_[i] = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        d.topic, rclcpp::QoS(10),
        [this, i](const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mtx_);
          drives_[i].cmd = *m;
          // Some publishers leave the stamp unset, so use the receive time.
          // noteReceived also updates the receive-rate EMA -- feeds the auto trust window.
          drives_[i].noteReceived(now());
        });
      RCLCPP_INFO(
        get_logger(), "drive [%zu] %s <- %s (hold %.0fms)",
        i, d.name.c_str(), d.topic.c_str(), d.hold * 1e3);
    }

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      cfg_.output.drive_topic, rclcpp::QoS(10));
    if (cfg_.output.publish_status) {
      selected_pub_ = create_publisher<std_msgs::msg::String>("~/selected", rclcpp::QoS(10));
      runner_up_pub_ = create_publisher<std_msgs::msg::String>("~/runner_up", rclcpp::QoS(10));
      scores_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/scores", rclcpp::QoS(10));
      status_pub_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(10));
    }

    reload_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reload",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::string msg;
        res->success = reload(&msg);
        res->message = msg;
        RCLCPP_INFO(get_logger(), "%s", msg.c_str());
      });

    using namespace std::chrono;
    eval_timer_ = create_wall_timer(
      duration_cast<nanoseconds>(duration<double>(1.0 / std::max(1.0, cfg_.evaluation_rate_hz))),
      [this]() {evaluationTick();}, eval_group_);
    output_timer_ = create_wall_timer(
      duration_cast<nanoseconds>(duration<double>(1.0 / std::max(1.0, cfg_.output.rate_hz))),
      [this]() {outputTick();}, output_group_);

    RCLCPP_INFO(
      get_logger(), "co_driver up: %zu scorers x %zu drives -> %s (eval %.0fHz / output %.0fHz)",
      scorers_.size(), drives_.size(), cfg_.output.drive_topic.c_str(),
      cfg_.evaluation_rate_hz, cfg_.output.rate_hz);
  }

private:
  // -------------------------------------------------------------------------
  // Config -> scorers / drives / postprocess
  // -------------------------------------------------------------------------
  bool build(std::string * error)
  {
    scorers_.clear();
    for (const auto & in : cfg_.inputs) {
      if (!in.enabled) {
        RCLCPP_INFO(get_logger(), "input '%s' is enabled=false, skipping.", in.name.c_str());
        continue;
      }
      auto s = ScorerRegistry::instance().create(in.type);
      if (!s) {
        std::string known;
        for (const auto & t : ScorerRegistry::instance().types()) {known += t + " ";}
        *error = "input '" + in.name + "': unknown type '" + in.type + "'. Available: " + known;
        return false;
      }
      s->setContext(in.name, in.type, scorer_group_);
      // The scorer creates its own subscriptions here.
      if (!s->configure(this, in.name, in.params)) {
        *error = "input '" + in.name + "' failed to configure";
        return false;
      }
      scorers_.push_back({s, in.hold});
    }

    drives_.clear();
    for (const auto & spec : cfg_.drives) {
      Drive d;
      d.name = spec.name;
      d.topic = spec.topic;
      d.enabled = spec.enabled;
      d.hold = spec.hold;
      d.bias = spec.bias;
      d.influence = spec.influence;
      d.last_rx = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      drives_.push_back(std::move(d));
    }

    selector_.configure(cfg_.selection);
    if (!post_.configure(this, cfg_.pipeline, get_logger())) {
      *error = "postprocess pipeline failed to configure";
      return false;
    }
    return true;
  }

  // Re-reads coefficient-only changes without a restart (refused if subscriptions must be rebuilt).
  bool reload(std::string * message)
  {
    Config fresh;
    std::string err;
    if (!Config::load(this, topics_path_, &fresh, &err)) {
      *message = "reload failed: " + err;
      return false;
    }
    if (!fresh.sameTopology(cfg_)) {
      *message =
        "reload failed: inputs/drives topology changed (name/type/topic). "
        "Subscriptions must be rebuilt, so restart the node.";
      return false;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    // Keep each drive's receive state; swap only the coefficients.
    for (const auto & spec : fresh.drives) {
      for (auto & d : drives_) {
        if (d.name != spec.name) {continue;}
        d.enabled = spec.enabled;
        d.hold = spec.hold;
        d.bias = spec.bias;
        d.influence = spec.influence;
      }
    }
    cfg_ = fresh;   // topics/rates are bound to subscriptions/timers and are not applied
    selector_.configure(cfg_.selection);
    if (!post_.configure(this, cfg_.pipeline, get_logger())) {
      *message = "reload failed: postprocess pipeline configuration error";
      return false;
    }
    post_.reset();
    *message = "reload complete (yaml parameters + " + topics_path_ + ")";
    return true;
  }

  // -------------------------------------------------------------------------
  // Score -> compute -> select
  // -------------------------------------------------------------------------
  void evaluationTick()
  {
    const rclcpp::Time t = now();
    Selector::Result sel;

    {
      std::lock_guard<std::mutex> lock(mtx_);

      ctx_.now = t;
      ctx_.dt = eval_t_valid_ ? std::max(0.0, (t - last_eval_).seconds()) : 0.0;
      last_eval_ = t;
      eval_t_valid_ = true;
      ctx_.drives = &drives_;
      ctx_.has_last_output = has_output_;
      ctx_.last_output = output_;
      ctx_.last_selected = selector_.current();

      for (auto & e : scorers_) {e.scorer->prepare(ctx_, drives_);}

      for (auto & d : drives_) {
        d.results.clear();
        if (d.enabled) {
          for (auto & e : scorers_) {
            // Zero-weight inputs are scored too -- needed for veto and status display.
            // If an async scorer reports pending, the previous score substitutes for hold.
            const auto & name = e.scorer->name();
            double held = -1.0;
            d.results[name] =
              resolveScore(d, name, e.scorer->score(d, ctx_), e.hold, t, &held);
            held_age_[d.name][name] = held;
          }
        }
        computeLogit(d, cfg_.scoring, t);
      }
      finalizeScores(drives_, cfg_.scoring);

      sel = selector_.select(drives_, t);
      have_command_ = sel.has_selection;
      selected_ = sel.name;
      if (sel.switched) {switch_pending_ = true;}
    }

    publishStatus(sel);
  }

  // -------------------------------------------------------------------------
  // Postprocess -> publish
  // -------------------------------------------------------------------------
  void outputTick()
  {
    const rclcpp::Time t = now();
    ackermann_msgs::msg::AckermannDrive cmd;
    PostContext pc;

    {
      std::lock_guard<std::mutex> lock(mtx_);

      pc.dt = out_t_valid_ ? std::max(0.0, (t - last_out_).seconds()) : 0.0;
      last_out_ = t;
      out_t_valid_ = true;
      pc.has_previous = has_output_;
      pc.previous = output_;
      pc.has_command = have_command_;
      // switched must be consumed exactly once by the output tick so the blend arms exactly once.
      pc.switched = switch_pending_;
      switch_pending_ = false;

      if (have_command_ && !selected_.empty()) {
        for (const auto & d : drives_) {
          if (d.name == selected_) {cmd = d.cmd.drive; break;}
        }
      } else if (has_output_) {
        // With no usable drive, start from the last output so timeout_stop decelerates it.
        cmd = output_;
      }

      post_.apply(cmd, pc);
      output_ = cmd;
      has_output_ = true;
    }

    // Keep publishing even with no usable drive -- timeout_stop's deceleration must reach the car.
    ackermann_msgs::msg::AckermannDriveStamped out;
    out.header.stamp = t;
    out.header.frame_id = cfg_.output.frame_id;
    out.drive = cmd;
    drive_pub_->publish(out);
  }

  // -------------------------------------------------------------------------
  // Diagnostics publishing
  // -------------------------------------------------------------------------
  void publishStatus(const Selector::Result & sel)
  {
    if (!cfg_.output.publish_status) {return;}
    std::lock_guard<std::mutex> lock(mtx_);

    if (selected_pub_) {
      std_msgs::msg::String s;
      s.data = sel.has_selection ? sel.name : "";
      selected_pub_->publish(s);
    }
    if (runner_up_pub_) {
      // Runner-up by combined score. Empty if only one valid drive.
      std_msgs::msg::String s;
      s.data = sel.has_runner_up ? sel.runner_up : "";
      runner_up_pub_->publish(s);
    }
    if (scores_pub_) {
      // Names ride in layout.dim[i].label so consumers can match without indices.
      std_msgs::msg::Float64MultiArray m;
      m.layout.dim.resize(drives_.size());
      for (std::size_t i = 0; i < drives_.size(); ++i) {
        m.layout.dim[i].label = drives_[i].name;
        m.layout.dim[i].size = 1;
        m.layout.dim[i].stride = 1;
        m.data.push_back(drives_[i].score);
      }
      scores_pub_->publish(m);
    }
    if (!status_pub_) {return;}

    // Order valid drives by score to assign rank (1, 2, ...).
    // With hysteresis, selected may not be rank 1; making that gap visible is the point.
    std::vector<const Drive *> ranked;
    for (const auto & d : drives_) {
      if (d.valid) {ranked.push_back(&d);}   // only active drives are ranked
    }
    std::sort(
      ranked.begin(), ranked.end(),
      [](const Drive * a, const Drive * b) {return a->score > b->score;});
    std::map<std::string, int> rank_of;
    for (std::size_t i = 0; i < ranked.size(); ++i) {rank_of[ranked[i]->name] = static_cast<int>(i) + 1;}

    // Expose every stage before the linear layer: x (raw scorer score) -> s (phi shaping) -> c (W*phi)
    std::ostringstream os;
    os << "{\"stamp\":" << num(ctx_.now.seconds())
       << ",\"selected\":\"" << esc(sel.has_selection ? sel.name : "") << "\""
       << ",\"runner_up\":\"" << esc(sel.has_runner_up ? sel.runner_up : "") << "\""
       << ",\"reason\":\"" << esc(sel.reason) << "\""
       << ",\"switched\":" << (sel.switched ? "true" : "false")
       << ",\"combine\":\"" << esc(cfg_.scoring.combine) << "\""
       << ",\"temperature\":" << num(cfg_.scoring.temperature)
       << ",\"drives\":[";
    for (std::size_t i = 0; i < drives_.size(); ++i) {
      const auto & d = drives_[i];
      if (i) {os << ',';}
      const auto rk = rank_of.find(d.name);
      os << "{\"name\":\"" << esc(d.name) << "\""
         << ",\"rank\":" << (rk == rank_of.end() ? "null" : std::to_string(rk->second))
         << ",\"score\":" << num(d.score)
         << ",\"logit\":" << num(d.logit)
         << ",\"raw\":" << num(d.raw_logit)
         << ",\"bias\":" << num(d.bias)
         // active = currently live, eligible for ranking/selection. valid = active + min_valid_score.
         << ",\"active\":" << (d.active ? "true" : "false")
         << ",\"valid\":" << (d.valid ? "true" : "false")
         << ",\"age_ms\":" << num(d.age(ctx_.now) * 1e3)
         << ",\"hold_ms\":" << num(d.hold * 1e3)          // validity window of the last command
         << ",\"hz\":" << num(d.measuredHz())         // measured receive rate
         << ",\"speed\":" << num(d.cmd.drive.speed)
         << ",\"steer\":" << num(d.cmd.drive.steering_angle);
      if (!d.reject.empty()) {os << ",\"reject\":\"" << esc(d.reject) << "\"";}
      os << ",\"inputs\":{";
      bool first = true;
      for (const auto & kv : d.results) {
        if (!first) {os << ',';}
        first = false;
        os << "\"" << esc(kv.first) << "\":";
        if (kv.second.veto) {
          os << "\"veto\"";
        } else if (!kv.second.available) {
          os << "null";
        } else {
          const auto it = d.influence.find(kv.first);
          const double x = std::clamp(kv.second.value, 0.0, 1.0);
          if (it == d.influence.end()) {
            os << "{\"x\":" << num(x) << "}";
          } else {
            os << "{\"x\":" << num(x)
               << ",\"s\":" << num(shapeInfluence(it->second, x))
               << ",\"w\":" << num(it->second.weight)
               << ",\"c\":" << num(it->second.weight * shapeInfluence(it->second, x));
            // If an async scorer's previous score was substituted, also show its age.
            const auto ha = held_age_.find(d.name);
            if (ha != held_age_.end()) {
              const auto hb = ha->second.find(kv.first);
              if (hb != ha->second.end() && hb->second >= 0.0) {
                os << ",\"held_ms\":" << num(hb->second * 1e3);
              }
            }
            // A scorer's note is its live progress report (e.g. the recovery
            // gate's "recovering 1.2/3.0s") - the visualization reads it.
            if (!kv.second.note.empty()) {
              os << ",\"note\":\"" << esc(kv.second.note) << "\"";
            }
            os << "}";
          }
        }
      }
      os << "}}";
    }
    os << "]}";

    std_msgs::msg::String s;
    s.data = os.str();
    status_pub_->publish(s);
  }

  // --- state ---
  std::string topics_path_;
  Config cfg_;
  // Pairs each scorer with its input's hold (previous-score validity time).
  struct ScorerEntry
  {
    ScorerPtr scorer;
    double hold{0.5};
  };
  std::vector<ScorerEntry> scorers_;
  // Diagnostics: age [s] of scores filled from cache this cycle (negative otherwise)
  std::map<std::string, std::map<std::string, double>> held_age_;
  std::vector<Drive> drives_;
  Selector selector_;
  PostProcess post_;
  Context ctx_;
  std::mutex mtx_;

  ackermann_msgs::msg::AckermannDrive output_;
  bool has_output_{false};
  bool have_command_{false};
  bool switch_pending_{false};
  std::string selected_;
  rclcpp::Time last_eval_, last_out_;
  bool eval_t_valid_{false}, out_t_valid_{false};

  std::vector<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr>
  drive_subs_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_pub_, runner_up_pub_, status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr scores_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::TimerBase::SharedPtr eval_timer_, output_timer_;
  rclcpp::CallbackGroup::SharedPtr eval_group_, output_group_, scorer_group_;
};

}  // namespace co_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // Multithreaded so scoring (possibly heavy) and output (fixed rate) don't block each other.
    // Shared state is guarded by the node's internal mutex.
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<co_driver::CoDriverNode>(rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    const std::string what = e.what();
    RCLCPP_FATAL(rclcpp::get_logger("co_driver"), "%s", what.c_str());
    // An empty yaml list (`pipeline: []`) has no type and blows up automatic parameter declaration.
    if (what.find("No parameter value set") != std::string::npos) {
      RCLCPP_FATAL(
        rclcpp::get_logger("co_driver"),
        "An empty list (`[]`) or a valueless key in the yaml never reaches ROS as a parameter. "
        "Delete the entry or give it a value (e.g. to disable postprocessing, remove the whole postprocess block).");
    }
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
