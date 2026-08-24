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
#include <atomic>
#include <fstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
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
// Return assist -- a short, time-limited hand on the map controller's tuning
// at the moment it gets the car back.
//
// Coming back with the steering already hard over means the car is a long way
// from the attitude the controller wants, and the derivative term is what acts
// on how fast that error is closing. Raising it helps only while the error is
// being taken out; left raised it amplifies noise for the rest of the lap. So
// it goes up on the handover and comes back down on a timer, and the value it
// comes back to is the one that was actually there, read before it was
// touched. If that read fails, nothing is changed at all: a raised gain with
// no way back is the worst thing this could leave behind.
//
// It runs on its own thread because it cannot run on the evaluation tick: a
// parameter service round trip is tens of milliseconds and the hold is
// seconds. It captures only values, never the node, so it cannot outlive
// anything it points at.
// ===========================================================================
// What the assist is doing right now, for the status topic. Shared with the
// thread, which is the only writer.
struct AssistState
{
  std::mutex m;
  bool active{false};
  std::string what;          // "K_d 0.5000->1.0000, ..."
  double hold{0.0};
  rclcpp::Time until;
  std::string last{"idle"};  // how the previous one ended
};

// Hold a fixed speed, then let the controller ramp out of it by itself. The
// duration is independent of the gain-assist hold: the two actions start on the
// same hand-back edge, but may need different tuning on the car.
struct SpeedHold
{
  bool enabled{false};
  std::string topic{"/launch_speed_hold"};
  double speed{1.5};
  double duration{2.0};
};

struct ReturnAssist
{
  bool enabled{false};
  std::string node{"/pure_pursuit"};
  // name -> what to multiply it by, applied and restored together.
  std::vector<std::pair<std::string, double>> parameters;
  double steering_above{15.0 * M_PI / 180.0};
  double hold{2.0};
  double service_timeout{0.5};
  SpeedHold speed_hold;
};

inline void runReturnAssist(
  const ReturnAssist a, const double steer_deg,
  std::shared_ptr<std::atomic<bool>> busy,
  std::shared_ptr<std::atomic<bool>> abort,
  std::shared_ptr<AssistState> state, const rclcpp::Time started,
  const unsigned seq)
{
  const auto log = rclcpp::get_logger("co_driver.return_assist");
  const auto timeout = std::chrono::milliseconds(
    static_cast<int>(a.service_timeout * 1e3));
  const auto note = [&state](const std::string & how) {
      std::lock_guard<std::mutex> lock(state->m);
      state->active = false;
      state->last = how;
    };
  try {
    auto helper = std::make_shared<rclcpp::Node>(
      "co_driver_return_assist_" + std::to_string(seq));
    auto client = std::make_shared<rclcpp::SyncParametersClient>(helper, a.node);
    if (!client->wait_for_service(timeout)) {
      RCLCPP_WARN(
        log, "%s has no parameter service - leaving its tuning alone",
        a.node.c_str());
      note("no parameter service");
      busy->store(false);
      return;
    }
    std::vector<std::string> names;
    for (const auto & kv : a.parameters) {names.push_back(kv.first);}
    const auto got = client->get_parameters(names, timeout);
    // All or nothing, in both directions. Reading one and failing on the next
    // would leave a gain raised whose original nobody wrote down.
    if (got.size() != names.size()) {
      RCLCPP_WARN(
        log, "%s returned %zu of %zu parameters - changing none of them",
        a.node.c_str(), got.size(), names.size());
      note("could not read them all");
      busy->store(false);
      return;
    }
    std::vector<rclcpp::Parameter> raised, original;
    std::string summary;
    for (std::size_t i = 0; i < got.size(); ++i) {
      if (got[i].get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        RCLCPP_WARN(
          log, "%s %s is not a double - changing none of them, because there "
          "would be no way back", a.node.c_str(), names[i].c_str());
        note(names[i] + " is not a double");
        busy->store(false);
        return;
      }
      const double base = got[i].as_double();
      const double up = base * a.parameters[i].second;
      original.emplace_back(names[i], base);
      raised.emplace_back(names[i], up);
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%s %.4f->%.4f", names[i].c_str(), base, up);
      summary += (summary.empty() ? "" : ", ") + std::string(buf);
    }
    const auto set = client->set_parameters_atomically(raised, timeout);
    if (!set.successful) {
      RCLCPP_WARN(
        log, "%s refused the change (%s) - nothing was altered",
        a.node.c_str(), set.reason.c_str());
      note("refused: " + set.reason);
      busy->store(false);
      return;
    }
    RCLCPP_INFO(
      log, "handed back at %.0f deg of steering: %s %s for %.1fs",
      steer_deg, a.node.c_str(), summary.c_str(), a.hold);
    {
      std::lock_guard<std::mutex> lock(state->m);
      state->active = true;
      state->what = summary;
      state->hold = a.hold;
      state->until = started + rclcpp::Duration::from_seconds(a.hold);
      state->last = "raised";
    }

    // Slice the wait so shutdown does not have to sit through it. A process
    // that exits mid-hold leaves the gains raised with nobody left who knows
    // what they were, which is the one outcome this whole thing must not have.
    const auto slice = std::chrono::milliseconds(20);
    auto left = std::chrono::milliseconds(static_cast<int>(a.hold * 1e3));
    while (left.count() > 0 && !abort->load()) {
      const auto step = std::min(slice, left);
      std::this_thread::sleep_for(step);
      left -= step;
    }
    const bool cut_short = left.count() > 0;

    // Put back only what is still ours. Anything that moved underneath us in
    // the meantime - a person tuning live, another node - belongs to whoever
    // moved it, and writing our remembered value over theirs would be worse
    // than leaving it.
    std::vector<rclcpp::Parameter> restore;
    std::string kept;
    const auto now = client->get_parameters(names, timeout);
    for (std::size_t i = 0; i < original.size(); ++i) {
      const bool readable = i < now.size() &&
        now[i].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (readable && std::abs(now[i].as_double() - raised[i].as_double()) > 1e-9) {
        kept += (kept.empty() ? "" : ", ") + names[i];
        continue;                    // somebody else owns it now
      }
      restore.push_back(original[i]);
    }
    if (!kept.empty()) {
      RCLCPP_WARN(
        log, "%s changed underneath the assist and was left alone", kept.c_str());
    }
    // Retry: one refused call must not be the end of it, because there is no
    // later pass that would notice and no state left anywhere else that says
    // what these were.
    bool restored = restore.empty();
    for (int attempt = 0; attempt < 3 && !restored; ++attempt) {
      if (attempt) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
      restored = client->set_parameters_atomically(restore, timeout).successful;
    }
    if (restored) {
      RCLCPP_INFO(
        log, "%s back to what it was%s", a.node.c_str(),
        cut_short ? " (cut short by shutdown)" : "");
      note(cut_short ? "restored early" : "restored");
    } else {
      RCLCPP_ERROR(
        log, "%s REFUSED the restore three times - %s is still raised. Set it "
        "back by hand: ros2 param set %s ...",
        a.node.c_str(), a.parameters.empty() ? "the tuning" :
        a.parameters.front().first.c_str(), a.node.c_str());
      note("RESTORE FAILED - still raised");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(log, "return assist gave up: %s", e.what());
    note(std::string("gave up: ") + e.what());
  }
  busy->store(false);
}

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
    // Chosen by the last rung of the fallback ladder, with every gate unhappy.
    // It drives, but it does not become the incumbent: a gate that exempts
    // whoever is currently selected would be permanently opened by one such
    // pick, which is the opposite of what a last resort is for.
    bool last_resort{false};
  };

  void configure(const SelectionSpec & spec) {spec_ = spec;}
  // Who the gates should treat as the incumbent. A last-resort pick is driving
  // on sufferance, not holding the car, so it is nobody - and reading current_
  // instead would re-open every gate that exempts the incumbent, which is the
  // whole reason this accessor exists. current_ itself is private.
  const std::string & incumbent() const {return last_resort_ ? none_ : current_;}

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

    // A stopped car is not driving anywhere, so there is nothing for a
    // handover to achieve and nothing it can protect against. Freezing the
    // selection there is not a safety compromise - whatever is selected is
    // commanding a standstill - and it removes churn the driver can see:
    // localization cannot discriminate its position without motion (the
    // producer says so itself), so the state wanders Lost/Converging while
    // the car sits, disqualifying and releasing the map controller for no
    // reason anyone could act on. Measured over 71 s of standstill in the
    // 0814 recording, that accounted for 12.9% of samples.
    //
    // The freeze lifts on the first cycle the car is moving again, and never
    // holds a drive whose command has gone stale.
    if (incumbent_ok && spec_.freeze_below_speed > 0.0 &&
      std::abs(incumbent->cmd.drive.speed) < spec_.freeze_below_speed)
    {
      r.has_selection = true;
      r.name = current_;
      r.reason = "stopped - selection frozen";
      return r;
    }

    const Drive * best = nullptr;
    for (const auto & d : drives) {
      if (!d.valid) {continue;}
      if (!best || d.score > best->score) {best = &d;}
    }

    if (!best) {
      // NOTHING CLEARED THE BAR. What follows is the only part of this file
      // that runs when the car would otherwise be stopped, so it is written as
      // a ladder: each rung gives up one more thing, and says which one.
      //
      // Rung 1 - the configured fallback, if it is live. A score floor is a
      // preference between drives, not a safety gate; being under it is not a
      // reason to stop a car that has a controller talking to it.
      //
      // This asked for `valid` before, which is exactly what is false for
      // every drive by the time we are here - the branch could not run at all.
      // Rungs 1 and 2 only bite where min_valid_score is above 0; both shipped
      // configs leave it at 0, which makes valid == active and sends every
      // such case straight to rung 3.
      if (!spec_.fallback.empty()) {
        for (const auto & d : drives) {
          if (d.name == spec_.fallback && d.active) {
            return commit(d.name, "fallback", now);
          }
        }
      }
      // Rung 2 - any live drive, the incumbent first if it is one of them.
      // Picking argmax(score) here would trade the car every tick between two
      // drives whose scores cross, with none of the margin or cooldown the
      // main path uses to stop exactly that.
      for (const auto & d : drives) {
        if (d.active && d.name == current_) {best = &d; break;}
      }
      if (!best) {
        for (const auto & d : drives) {
          if (d.active && (!best || d.score > best->score)) {best = &d;}
        }
      }
      if (best) {return commit(best->name, "fallback: only live drive", now);}

      // Rung 3 - every drive failed a gate, and a gate says "not this one",
      // which presumes there is another. There is not. The choice left is
      // between stopping the car and driving it with a command whose gate is
      // unhappy, and on a cold start the unhappy gate is usually just
      // localization never having started: nothing is wrong with the command,
      // there is only nothing to check it against.
      //
      // Freshness is still required. If no topic is talking there is nothing
      // to fall back TO, and the pipeline's timeout_stop takes the car down -
      // which is what should happen when the controllers go silent.
      //
      // What it may NOT step over is a gate that called the command dangerous
      // rather than un-preferred. Driving past "there is an object on the line"
      // or "the car is not on the line any more" replaces a controlled stop
      // with driving at the thing the gate was watching, and by then
      // timeout_stop cannot help either - it only acts when nothing was
      // selected. Influence::last_resort_ok marks the gates that may be
      // stepped over, and defaults to false.
      if (spec_.last_resort) {
        const Drive * live = nullptr;
        auto usable = [&now](const Drive & d) {return d.isLive(now) && d.forceable;};
        for (const auto & d : drives) {                       // the incumbent first
          if (usable(d) && d.name == current_) {live = &d; break;}
        }
        if (!live) {
          for (const auto & d : drives) {                     // then the fallback
            if (usable(d) && d.name == spec_.fallback) {live = &d; break;}
          }
        }
        if (!live) {
          for (const auto & d : drives) {                     // then the freshest
            if (usable(d) && (!live || d.age(now) < live->age(now))) {live = &d;}
          }
        }
        if (live) {
          return commit(
            live->name, "last resort: " + live->name + " is the only command "
            "still arriving (" + live->reject + ")", now, true);
        }
      }
      r.reason = "no valid drive";
      // current_ is deliberately NOT cleared. It is the memory of who was
      // driving, and a gap in validity is not a handover. Clearing it made the
      // next tick re-commit the SAME drive with switched=true, which re-armed
      // switch_blend (dragging the output through a 300 ms ramp from whatever
      // timeout_stop had decelerated to) and reset last_switch_, locking out a
      // genuine score-margin handover for a full switch_cooldown afterwards.
      // One missed 20 ms tick was enough. Externally that reads exactly as the
      // command coming out shoved and delayed, with nothing selected in
      // between. Keeping it costs nothing: a drive can only be selected while
      // it is valid, so a stale incumbent still cannot drive.
      return r;
    }

    if (!incumbent_ok) {
      // If the incumbent died, switch immediately without hysteresis.
      forced_ = !current_.empty();
      return commit(best->name, current_.empty() ? "initial selection" : "incumbent invalid -> immediate switch", now);
    }

    r.has_selection = true;
    if (best->name == current_) {
      r.name = current_;
      r.reason = "hold";
      return r;
    }

    // The cooldown belongs to whoever HAS the car, not to the arbitration as a
    // whole: it is that drive saying how long it keeps what it was given. A
    // localization fallback wants to be sticky, an obstacle detour wants to end
    // the moment the obstacle is behind - and they were sharing one number.
    const double cooldown = (incumbent && incumbent->keep >= 0.0) ?
      incumbent->keep : spec_.switch_cooldown;
    const double held = switched_once_ ?
      (now - last_switch_).seconds() : cooldown + 1.0;
    // The cooldown guards against SCORE oscillation - two drives trading places
    // because their probabilities are close. It has no business governing the
    // return from a handover that a hard gate forced: the gate has its own
    // hysteresis on both edges (confirm before it engages, hold before it
    // releases), so waiting again afterwards is the same debounce charged
    // twice, and it is charged exactly when the map controller is worth most.
    //
    // Measured at 4-8 m/s on the 0814 drive: 29.6% of samples had pp_main
    // valid, un-vetoed and outscoring gap_follow 0.971 to 0.029, sitting out a
    // cooldown started by an obstacle that was already behind the car. That is
    // more of the run than the obstacles themselves accounted for at 5-8 m/s.
    //
    // MEASURED AND REVERTED. Letting a gate-forced return skip the cooldown
    // did raise occupancy - 46.5% to 50.0% overall, 31.0% to 37.5% at
    // 5-8 m/s - and it destroyed the thing that made those numbers worth
    // having: handovers went from 52 to 140 over the same drive, and the
    // median time a controller kept the car fell from 1.54 s to 0.12 s. A
    // controller that changes ten times a second is not driving whatever the
    // occupancy says. Widening the obstacle scorer's own hysteresis recovered
    // part of it (94 handovers, 0.28 s) but never came close.
    //
    // So the cooldown stays, on every return. The damping it provides is real
    // and nothing here replaces it; the cost is that a detour lasts at least
    // switch_cooldown_ms, which is the price of not oscillating. The delay
    // that WAS worth removing is in the recovery latch - see recovery_gate.cpp
    // - because that one was making an obstacle detour wait out a proof about
    // localization that nothing had called into question.
    // Two candidates reading the same topic are the same command wearing two
    // labels - the fallback appears once per reason it can be chosen. Moving
    // between them changes nothing the car can feel, so there is no flapping to
    // guard against and the cooldown does not apply. Without this the label
    // that happened to be reached first held the car for its own dwell: an
    // obstacle detour would be booked under the localization fallback, and
    // released on that one's 1500 ms instead of its own 400 ms, which is the
    // entire thing splitting them was meant to fix.
    const bool same_source = incumbent && incumbent->topic == best->topic;
    if (!same_source && cooldown > 0.0 && held < cooldown) {
      r.name = current_;
      r.reason = "waiting on switch_cooldown";
      return r;
    }
    if (best->score < incumbent->score + spec_.switch_margin) {
      r.name = current_;
      r.reason = "switch_margin not met";
      return r;
    }
    const bool was_forced = forced_;
    forced_ = false;
    return commit(
      best->name, was_forced ? "gate cleared -> returning" : "switched on score margin", now);
  }

  Result commit(
    const std::string & name, const std::string & reason, const rclcpp::Time & now,
    bool last_resort = false)
  {
    // The flag belongs to current_, not to this tick. Setting it from each
    // Result meant a single tick where nothing was live - which returns "no
    // valid drive" and does NOT clear current_ - reported false while current_
    // still named a last-resort pick, handing it the incumbent exemption it
    // was specifically denied.
    last_resort_ = last_resort;
    Result r;
    r.has_selection = true;
    r.name = name;
    r.reason = reason;
    r.last_resort = last_resort;
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
  // The last handover was forced by a gate, not won on score.
  bool forced_{false};
  // Set when the current selection came from the last rung of the ladder.
  bool last_resort_{false};
  const std::string none_;
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
  // Going away in the middle of a hold would leave the map controller's gains
  // raised with nobody left who knows what they were. Ask the assist to cut
  // the wait and put them back, and give it a moment to do it.
  ~CoDriverNode() override
  {
    if (!assist_busy_->load()) {return;}
    assist_abort_->store(true);
    for (int i = 0; i < 200 && assist_busy_->load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (assist_busy_->load()) {
      RCLCPP_ERROR(
        get_logger(), "shutting down while the return assist was still "
        "restoring %s - check its gains by hand", assist_.node.c_str());
    }
  }

  explicit CoDriverNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("co_driver_node", rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true))
  {
    // Base config comes from yaml (ROS parameters); growing lists come from this JSON.
    if (!has_parameter("return_assist_file")) {
      declare_parameter<std::string>("return_assist_file", "");
    }
    loadReturnAssist(get_parameter("return_assist_file").as_string());
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

    if (assist_.speed_hold.enabled) {
      speed_hold_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        assist_.speed_hold.topic, rclcpp::QoS(1));
    }
    if (!cfg_.ramp_on_return.topic.empty()) {
      ramp_pub_ = create_publisher<std_msgs::msg::Bool>(
        cfg_.ramp_on_return.topic, rclcpp::QoS(1));
      std::string from;
      for (const auto & f : cfg_.ramp_on_return.from) {
        from += (from.empty() ? "" : "/") + f;
      }
      RCLCPP_INFO(
        get_logger(), "ramp on return: %s -> %s publishes true on %s",
        from.empty() ? "(nothing)" : from.c_str(), cfg_.ramp_on_return.to.c_str(),
        cfg_.ramp_on_return.topic.c_str());
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
    // Both ticks are wrapped. MultiThreadedExecutor dispatches callbacks onto
    // worker std::threads, where an escaping exception is std::terminate - not
    // something main()'s try/catch can ever see. For an arbiter whose entire
    // safety story is "keep publishing, decelerate on timeout", losing the
    // process is the worst available outcome: better to drop one tick, say so,
    // and keep the output alive.
    eval_timer_ = create_wall_timer(
      duration_cast<nanoseconds>(duration<double>(1.0 / std::max(1.0, cfg_.evaluation_rate_hz))),
      [this]() {guarded("evaluation", [this]() {evaluationTick();});}, eval_group_);
    output_timer_ = create_wall_timer(
      duration_cast<nanoseconds>(duration<double>(1.0 / std::max(1.0, cfg_.output.rate_hz))),
      [this]() {guarded("output", [this]() {outputTick();});}, output_group_);

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
        // An unknown type is nearly always an optional scorer whose message
        // package is absent on this machine - obstacle_avoid needs
        // obstacle_context_msgs, and CMake drops the whole file when it is not
        // found. Refusing to start here takes the ENTIRE arbitration down: no
        // /drive, no /co_driver_node/status, and a car with nothing driving it.
        // Coming up without that one input is strictly safer, so do that and
        // make the degradation impossible to miss.
        std::string known;
        for (const auto & t : ScorerRegistry::instance().types()) {known += t + " ";}
        RCLCPP_ERROR(
          get_logger(),
          "input '%s': unknown type '%s' - starting WITHOUT it. Available: %s",
          in.name.c_str(), in.type.c_str(), known.c_str());
        // Neutralise it everywhere so it cannot disqualify a drive it can no
        // longer judge, and so `missing: mask` does not rescale the logit by a
        // weight nothing is contributing. This does NOT restore the
        // calibration: a weight that was compensated by a drive's bias leaves
        // that bias behind, biased toward the fallback. Say so, per drive.
        for (auto & spec : cfg_.drives) {
          auto inf = spec.influence.find(in.name);
          if (inf == spec.influence.end()) {continue;}
          const bool weighted = std::abs(inf->second.weight) > 1e-12;
          const bool vetoing = inf->second.veto_below >= 0.0;
          inf->second.weight = 0.0;
          inf->second.veto_below = -1.0;
          inf->second.required = false;
          if (weighted || vetoing) {
            RCLCPP_ERROR(
              get_logger(),
              "  drive '%s' scored input '%s' (weight %.2f%s). It is now inert, "
              "but drive bias %.2f was calibrated WITH it - the handover "
              "thresholds have moved. Load a config that drops this input and "
              "restores the uncompensated bias before trusting this run.",
              spec.name.c_str(), in.name.c_str(), inf->second.weight,
              vetoing ? ", vetoing" : "", spec.bias);
          }
        }
        missing_inputs_.push_back(in.name + " (" + in.type + ")");
        continue;
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
      d.keep = spec.keep;
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
  // Scoring runs on a SNAPSHOT, with no node lock held.
  //
  // It used to run inside one lock_guard that covered prepare() and every
  // score() call for every drive. The callback groups made it look decoupled -
  // eval, output and the scorer subscriptions each have their own - but they
  // all take this same mutex, so the split bought nothing: outputTick() and the
  // /drive_main // /drive_gf callbacks queued behind the whole scoring pass.
  // The heavy work lives in score(): a full arc projection over the scan and
  // over every cluster, once PER DRIVE. On a vehicle with few cores that is
  // long enough for the 100 Hz output timer to miss firings, and /drive
  // disappears in stretches while the node is still perfectly alive.
  //
  // Now the lock is held only to take a consistent copy and to write the result
  // back - microseconds either side. The scorers see a snapshot that is at most
  // one scoring pass old, which is also more consistent than before, where a
  // command could change halfway through a drive's own scoring.
  // Run one tick, swallowing anything it throws. A scorer is third-party code
  // as far as this node is concerned; a bad message should cost a cycle, not
  // the vehicle. Throttled so a persistent fault is visible without flooding.
  template<typename F>
  void guarded(const char * what, F && f)
  {
    try {
      f();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s tick threw (%s) - skipping this cycle. The output keeps running; "
        "if this repeats, a scorer is faulting on its input.", what, e.what());
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s tick threw a non-std exception - skipping this cycle.", what);
    }
  }

  void evaluationTick()
  {
    const rclcpp::Time t = now();
    Selector::Result sel;

    std::vector<Drive> snap;
    Context ctx;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      ctx_.now = t;
      ctx_.dt = eval_t_valid_ ? std::max(0.0, (t - last_eval_).seconds()) : 0.0;
      last_eval_ = t;
      eval_t_valid_ = true;
      ctx_.has_last_output = has_output_;
      ctx_.last_output = output_;
      ctx_.last_selected = selector_.incumbent();
      snap = drives_;
      ctx = ctx_;
    }
    ctx.drives = &snap;

    // ---- unlocked: the expensive part ----
    const auto t0 = std::chrono::steady_clock::now();
    std::map<std::string, std::map<std::string, double>> held_age;
    for (auto & e : scorers_) {e.scorer->prepare(ctx, snap);}
    for (auto & d : snap) {
      d.results.clear();
      if (d.enabled) {
        for (auto & e : scorers_) {
          // Zero-weight inputs are scored too -- needed for veto and status display.
          // If an async scorer reports pending, the previous score substitutes for hold.
          const auto & name = e.scorer->name();
          double held = -1.0;
          d.results[name] =
            resolveScore(d, name, e.scorer->score(d, ctx), e.hold, t, &held);
          held_age[d.name][name] = held;
        }
      }
      computeLogit(d, cfg_.scoring, t);
    }
    finalizeScores(snap, cfg_.scoring);
    const double score_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    {
      std::lock_guard<std::mutex> lock(mtx_);
      // Merge back only what scoring produced. Reception state (cmd, last_rx,
      // has_cmd, the rate EMA) belongs to the subscription callbacks, which were
      // free to run while this pass was unlocked - copying the snapshot wholesale
      // would throw away commands that arrived meanwhile.
      for (auto & d : drives_) {
        auto it = std::find_if(
          snap.begin(), snap.end(), [&d](const Drive & s) {return s.name == d.name;});
        if (it == snap.end()) {continue;}
        d.results = std::move(it->results);
        d.score_cache = std::move(it->score_cache);
        d.raw_logit = it->raw_logit;
        d.logit = it->logit;
        d.logit_initialized = it->logit_initialized;
        d.score = it->score;
        d.active = it->active;
        d.valid = it->valid;
        d.forceable = it->forceable;
        d.reject = it->reject;
      }
      held_age_ = std::move(held_age);
      eval_ms_ = score_ms;
      eval_ms_max_ = std::max(eval_ms_max_, score_ms);

      sel = selector_.select(drives_, t);
      have_command_ = sel.has_selection;
      const std::string handed_from = selected_;
      selected_ = sel.name;
      // Handing the car back is invisible from the map controller's side - it
      // was publishing all along - so the moment has to be told to it. Not on
      // a last-resort pick: that is not a return, it is the only command left.
      //
      // Nothing fires here while the corner hold is refusing the handover,
      // because then there is no handover - that gate keeps the reactive
      // controller driving, and both of these are things to do at the moment
      // the map controller starts.
      //
      // The edge is computed once and both actions hang off it, so they happen
      // on the same tick without one being able to switch the other off:
      // clearing ramp_on_return.topic used to take the assist with it.
      const bool handed_back = sel.switched && !sel.last_resort &&
        sel.name == cfg_.ramp_on_return.to &&
        std::find(
          cfg_.ramp_on_return.from.begin(), cfg_.ramp_on_return.from.end(),
          handed_from) != cfg_.ramp_on_return.from.end();
      if (handed_back) {
        // One or the other, never both. The controller takes the most recent
        // of these as the winner, so sending the ramp flag after a speed hold
        // would cancel the hold - which is why turning the hold on turns the
        // ramp flag off rather than leaving both to be configured.
        // Not while one we asked for is still running. Measured on the 0822
        // recording: the arbitration churned three hand-backs inside 1.4 s and
        // each one restarted the hold, so the car sat pinned at the hold speed
        // for 44% of the eight seconds that followed. The gain assist already
        // refuses to stack for the same reason; this had nothing.
        const bool holding = speed_hold_valid_ && t < speed_hold_until_;
        if (speed_hold_pub_ && holding) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "%s -> %s again %.2fs into a %.1fs hold - not restarting it",
            handed_from.c_str(), sel.name.c_str(),
            assist_.speed_hold.duration - (speed_hold_until_ - t).seconds(),
            assist_.speed_hold.duration);
        } else if (speed_hold_pub_) {
          std_msgs::msg::Float64MultiArray msg;
          msg.data = {assist_.speed_hold.speed, assist_.speed_hold.duration};
          speed_hold_pub_->publish(msg);
          speed_hold_until_ =
            t + rclcpp::Duration::from_seconds(assist_.speed_hold.duration);
          speed_hold_valid_ = true;
          RCLCPP_INFO(
            get_logger(), "%s -> %s: holding %.2fm/s for %.1fs, then %s ramps "
            "out of it", handed_from.c_str(), sel.name.c_str(),
            assist_.speed_hold.speed, assist_.speed_hold.duration, assist_.node.c_str());
        } else if (ramp_pub_) {
          std_msgs::msg::Bool msg;
          msg.data = true;
          ramp_pub_->publish(msg);
          RCLCPP_INFO(
            get_logger(), "%s -> %s: asking %s to ramp the speed up rather than "
            "step to it", handed_from.c_str(), sel.name.c_str(),
            cfg_.ramp_on_return.topic.c_str());
        }
        startReturnAssist(sel.name);
      }
      // Two candidates may read the same topic - the fallback appears once per
      // reason it can be chosen. Swapping between those is a bookkeeping
      // change, not a change of command, so blending across it would drag a
      // 300 ms ramp through an output that did not move.
      if (sel.switched) {
        std::string topic;
        for (const auto & d : drives_) {
          if (d.name == sel.name) {topic = d.topic; break;}
        }
        if (topic != last_output_topic_) {switch_pending_ = true;}
        last_output_topic_ = topic;
      }
    }

    // Neither of these is visible from the car without being said out loud.
    // The status topic carries the reason, but the person watching a car that
    // will not move is reading the console.
    if (sel.last_resort) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "FAIL-SAFE: %s. Every drive is disqualified, so the freshest command "
        "still arriving is being used. Fix what the gate is complaining about.",
        sel.reason.c_str());
    } else if (!sel.has_selection && (t - last_no_drive_warn_).seconds() >= 2.0) {
      // Throttled by hand rather than by the macro: the reason string walks
      // every drive under the mutex, and the macro would have it built fifty
      // times a second to discard forty-nine of them.
      last_no_drive_warn_ = t;
      std::string why;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto & d : drives_) {
          why += (why.empty() ? "" : ", ") + d.name + ": " +
            (d.reject.empty() ? "-" : d.reject);
        }
      }
      RCLCPP_WARN(
        get_logger(), "no drive selected - the output is decelerating to a "
        "stop. %s", why.c_str());
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
  // What the handover assist has done to the map controller's tuning, so it is
  // visible while it is happening rather than only in the log afterwards.
  std::string assistJson()
  {
    std::ostringstream os;
    bool active;
    std::string what, last;
    double left = 0.0, hold = 0.0;
    {
      std::lock_guard<std::mutex> lock(assist_state_->m);
      active = assist_state_->active;
      what = assist_state_->what;
      last = assist_state_->last;
      hold = assist_state_->hold;
      if (active) {left = std::max(0.0, (assist_state_->until - now()).seconds());}
    }
    double hold_left = 0.0;
    if (speed_hold_valid_) {
      hold_left = std::max(0.0, (speed_hold_until_ - now()).seconds());
    }
    os << "{\"active\":" << (active ? "true" : "false")
       << ",\"enabled\":" << (assist_.enabled ? "true" : "false")
       << ",\"node\":\"" << esc(assist_.node) << "\""
       << ",\"raised\":\"" << esc(what) << "\""
       << ",\"left_s\":" << num(left)
       << ",\"hold_s\":" << num(hold)
       << ",\"last\":\"" << esc(last) << "\""
       << ",\"speed_hold\":{\"enabled\":"
       << (assist_.speed_hold.enabled ? "true" : "false")
       << ",\"speed\":" << num(assist_.speed_hold.speed)
       << ",\"duration_s\":" << num(assist_.speed_hold.duration)
       << ",\"left_s\":" << num(hold_left) << "}}";
    return os.str();
  }

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
       << ",\"assist\":" << assistJson()
       << ",\"switched\":" << (sel.switched ? "true" : "false")
       << ",\"combine\":\"" << esc(cfg_.scoring.combine) << "\""
       // Cost of the scoring pass. If eval_ms approaches the evaluation period
       // the stack is too heavy for its rate - visible here rather than as an
       // unexplained stutter on /drive.
       << ",\"eval_ms\":" << num(eval_ms_)
       << ",\"eval_ms_max\":" << num(eval_ms_max_)
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
  // Inputs whose scorer type was never registered - the optional message
  // package they need is absent on this machine. Kept for the status stream.
  std::vector<std::string> missing_inputs_;
  // Diagnostics: age [s] of scores filled from cache this cycle (negative otherwise)
  std::map<std::string, std::map<std::string, double>> held_age_;
  // Cost of one scoring pass, in ms. Published in the status so a stack that
  // is too slow for its evaluation rate is visible instead of guessed at.
  double eval_ms_{0.0}, eval_ms_max_{0.0};
  std::vector<Drive> drives_;
  Selector selector_;
  PostProcess post_;
  Context ctx_;
  std::mutex mtx_;

  ackermann_msgs::msg::AckermannDrive output_;
  bool has_output_{false};
  // Read from its own file, so tuning that belongs to the moment of handover
  // does not live in the arbitration's config.
  void loadReturnAssist(const std::string & path)
  {
    if (path.empty()) {return;}
    std::ifstream in(path);
    if (!in) {
      RCLCPP_WARN(get_logger(), "return_assist_file: cannot open %s", path.c_str());
      return;
    }
    Json j;
    try {
      j = Json::parse(in, nullptr, true, /*ignore_comments=*/ true);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "return_assist_file %s: %s", path.c_str(), e.what());
      return;
    }
    assist_.enabled = jbool(j, "enabled", true);
    assist_.node = jstr(j, "node", assist_.node);
    assist_.steering_above = jnum(j, "steering_above_deg", 15.0) * M_PI / 180.0;
    assist_.hold = jms(j, "hold_ms", assist_.hold * 1e3);
    assist_.service_timeout = jms(j, "service_timeout_ms", assist_.service_timeout * 1e3);
    const auto sh = j.find("speed_hold");
    if (sh != j.end() && sh->is_object()) {
      assist_.speed_hold.enabled = jbool(*sh, "enabled", false);
      assist_.speed_hold.topic = jstr(*sh, "topic", assist_.speed_hold.topic);
      assist_.speed_hold.speed = jnum(*sh, "speed", assist_.speed_hold.speed);
      // Backward compatibility: configurations without duration_ms keep using
      // the gain-assist hold, which was the hard-wired behaviour before this
      // field existed.
      assist_.speed_hold.duration =
        jms(*sh, "duration_ms", assist_.hold * 1e3);
      if (!std::isfinite(assist_.speed_hold.speed) || assist_.speed_hold.speed < 0.0 ||
        !std::isfinite(assist_.speed_hold.duration) || assist_.speed_hold.duration < 0.0 ||
        assist_.speed_hold.topic.empty())
      {
        RCLCPP_WARN(
          get_logger(), "return assist: speed_hold needs a topic, a finite speed "
          "at or above 0, and a finite duration_ms at or above 0 - disabled");
        assist_.speed_hold.enabled = false;
      }
    }
    assist_.parameters.clear();
    const auto it = j.find("parameters");
    if (it != j.end() && it->is_object()) {
      for (auto p = it->begin(); p != it->end(); ++p) {
        if (!p.value().is_number()) {continue;}
        assist_.parameters.emplace_back(p.key(), p.value().get<double>());
      }
    } else if (j.contains("parameter")) {          // the one-parameter shorthand
      assist_.parameters.emplace_back(
        jstr(j, "parameter", "K_d"), jnum(j, "multiply_by", 2.0));
    }
    bool sane = assist_.hold > 0.0 && !assist_.parameters.empty();
    for (const auto & kv : assist_.parameters) {
      if (kv.second <= 0.0) {sane = false;}
    }
    if (!sane) {
      RCLCPP_WARN(
        get_logger(), "return assist: needs at least one parameter, every "
        "multiplier above 0 and hold above 0 - disabled");
      assist_.enabled = false;
    }
    if (assist_.speed_hold.enabled) {
      RCLCPP_INFO(
        get_logger(),
        "return assist: handing back holds %.2fm/s on %s for %.1fs, and the "
        "controller ramps out of that by itself - so the ramp flag is NOT sent",
        assist_.speed_hold.speed, assist_.speed_hold.topic.c_str(),
        assist_.speed_hold.duration);
    }
    if (!assist_.enabled) {
      RCLCPP_INFO(get_logger(), "return assist: gains left alone");
      return;
    }
    std::string what;
    for (const auto & kv : assist_.parameters) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s x%.2f", kv.first.c_str(), kv.second);
      what += (what.empty() ? "" : ", ") + std::string(buf);
    }
    RCLCPP_INFO(
      get_logger(),
      "return assist: handing back above %.0f degrees of steering sets %s %s "
      "for %.1fs",
      assist_.steering_above * 180.0 / M_PI, assist_.node.c_str(),
      what.c_str(), assist_.hold);
  }

  // Only when the drive taking the car back is already hard over. One at a
  // time: overlapping handovers would each read a value the other had raised,
  // and the second restore would put the raised one back as the baseline.
  void startReturnAssist(const std::string & to)
  {
    if (!assist_.enabled) {return;}
    double steer = 0.0;
    for (const auto & d : drives_) {          // mtx_ is held by the caller
      if (d.name == to) {steer = std::abs(d.cmd.drive.steering_angle); break;}
    }
    if (!std::isfinite(steer) || steer <= assist_.steering_above) {return;}
    bool expected = false;
    if (!assist_busy_->compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(
        get_logger(), "return assist already running - skipping this handover");
      return;
    }
    assist_abort_->store(false);
    std::thread(
      runReturnAssist, assist_, steer * 180.0 / M_PI, assist_busy_, assist_abort_,
      assist_state_, now(), ++assist_seq_)
    .detach();
  }

  bool have_command_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ramp_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr speed_hold_pub_;
  rclcpp::Time speed_hold_until_;
  bool speed_hold_valid_{false};
  ReturnAssist assist_;
  std::shared_ptr<std::atomic<bool>> assist_busy_{
    std::make_shared<std::atomic<bool>>(false)};
  std::shared_ptr<std::atomic<bool>> assist_abort_{
    std::make_shared<std::atomic<bool>>(false)};
  std::shared_ptr<AssistState> assist_state_{std::make_shared<AssistState>()};
  unsigned assist_seq_{0};
  bool switch_pending_{false};
  // Topic behind the last selection, so the blend arms on a real source change.
  std::string last_output_topic_;
  std::string selected_;
  rclcpp::Time last_eval_, last_out_;
  // Hand-rolled throttle for the no-drive warning; see where it is used.
  rclcpp::Time last_no_drive_warn_{0, 0, RCL_ROS_TIME};
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
