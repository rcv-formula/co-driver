// Configuration -- reads yaml (ROS parameters) + a topic-list JSON into structs.
//
//   yaml : output / evaluation / defaults.influence / scoring / selection
//          / postprocess                           -- fixed-size base settings
//   json : inputs[] / drives[]                     -- ever-growing topic lists
//
// This file holds "values" only. All computation on them lives in compute.hpp.
#ifndef CO_DRIVER__CONFIG_HPP_
#define CO_DRIVER__CONFIG_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace co_driver
{

using Json = nlohmann::json;

// ---------------------------------------------------------------------------
// Effect of one input on one drive's score.
//
// Scoring is a single-layer linear MLP + softmax.
//   z_j = Sum_i W[j][i]*phi_ji(x_i) + b_j     W = weight, b = DriveSpec::bias
//   p   = softmax(z / temperature)
// phi is the linear+exponential response curve defined by the parameters below;
// the actual computation is shapeInfluence() in compute.hpp.
// ---------------------------------------------------------------------------
struct Influence
{
  double weight{0.0};     // linear-layer weight W. May be negative (penalty). 0 = ignored
  // Single mixing ratio between the linear and exponential components.
  //   0 = pure linear, 1 = pure exponential, 0.7 = 7 parts exp : 3 parts linear
  // (the old separate linear/exp pair only mattered as a ratio, leaving a spare
  // degree of freedom)
  double exp_mix{0.0};
  double exp_k{3.0};      // exponential curvature. >0 convex, <0 concave, ~0 linear
  bool invert{false};     // flip metrics where smaller is better
  double in_min{0.0};     // renormalization of the interval of interest
  double in_max{1.0};
  double veto_below{-1.0};  // disqualify the drive if the shaped value is below this (negative = off)
  bool required{false};     // disqualify the drive when this input is unavailable
  // May the last resort drive past this gate? A gate answers one of two very
  // different questions, and only the drive's author knows which: "another
  // drive would be better" (localization is down, so prefer the reactive one)
  // or "this command is dangerous" (there is an object on the line, or the car
  // has left the line entirely). The first is worth stepping over when there
  // is no other drive left; the second never is, and stepping over it turns a
  // controlled stop into driving at the thing the gate was watching.
  //
  // Default false: a gate is assumed to mean the dangerous one until someone
  // says otherwise, because that is the assumption whose failure mode is a
  // stopped car rather than a crashed one.
  bool last_resort_ok{false};

  // A bare number overrides weight; an object overrides field by field.
  void merge(const Json & j);
};

// One input = one Scorer instance. Element of JSON inputs[].
//
// hold -- how long to keep using the previous score while the scorer reports
// pending (still computing). Async scorers only; it has no effect on synchronous
// scorers that produce a value every cycle.
struct InputSpec
{
  std::string name;
  std::string type;       // empty = use name as type
  bool enabled{true};
  double hold{0.5};       // validity window of the previous score [s] (JSON: hold_ms, <=0 = unlimited)
  Json params;            // passed to the scorer verbatim (topic names to subscribe, etc.)
  Influence influence;    // this input's default influence, shared across drives
};

// Optional runtime choice between two command topics for one drive candidate.
// `channel` is deliberately one-based in the config, matching RC channel names
// (CH10 means 10), and is converted to an array index only at the callback.
struct DriveTopicSwitchSpec
{
  bool enabled{false};
  std::string selector_topic;
  std::size_t channel{1};
  std::uint16_t primary_value{2000};
  std::uint16_t alternate_value{1000};
  std::uint16_t tolerance{100};
  std::string alternate_topic;
};

// One drive = one /drive candidate. Element of JSON drives[].
//
// hold -- "how long to keep the last received command". This is the only
// time-related setting per drive. Past this window the drive goes inactive and
// drops out of ranking/selection.
//
// Topics with different Hz can each get their own value here (typically 3-5x
// the topic's period). The measured Hz shows up as hz in status -- use that to
// pick a value.
struct DriveSpec
{
  std::string name;
  std::string topic;
  DriveTopicSwitchSpec topic_switch;
  bool enabled{true};
  double hold{0.3};       // validity window of the last command [s] (JSON: hold_ms)
  // How long this drive keeps the car once it has it, overriding the
  // selection's switch_cooldown. Negative means "use the global one".
  //
  // The two kinds of handover this arbitration makes want opposite values and
  // were sharing one number. Leaving the map controller because localization
  // failed should be sticky - the confidence wanders near its threshold and
  // flipping back and forth is the failure mode the cooldown exists for.
  // Leaving it to go round an obstacle should end the moment the obstacle is
  // behind the car. One number cannot be both, and the compromise was costing
  // the second case badly.
  double keep{-1.0};      // [s] (jsonc: keep_ms)
  double bias{0.0};       // linear-layer bias b_j
  std::map<std::string, Influence> influence;   // input name -> influence (filled for all inputs)
};

struct ScoringSpec
{
  std::string combine{"softmax"};   // softmax | weighted_sum
  double temperature{1.0};          // lower = winner-take-all / higher = flatter
  std::string missing{"mask"};      // handling of unavailable inputs: mask | zero
  double ema_alpha{1.0};            // low-pass on the logit before softmax
  double min_valid_score{0.0};
};

struct SelectionSpec
{
  double switch_margin{0.05};
  // After a switch, no further switching for this long (anti-chattering).
  // Unrelated to a drive's hold -- called cooldown to avoid confusion.
  double switch_cooldown{0.5};     // [s] (yaml: switch_cooldown_ms)
  // Preferred drive when nothing clears min_valid_score. It has to be live,
  // not valid - by the time this is consulted, nothing is valid.
  std::string fallback;
  // With every drive failing a gate, drive the freshest command that is still
  // arriving rather than stopping the car. A gate says "not this drive", which
  // presumes another exists; when none does, the honest choice is between a
  // gated command and no command. Freshness is still required, so silence
  // still stops the car through the pipeline's timeout_stop.
  // Off by default. It trades a controlled stop for driving a command whose
  // gate is unhappy, which is a decision about a particular vehicle, not a
  // default that should arrive with a rebuild.
  bool last_resort{false};  // (yaml: selection.last_resort)
  // Below this commanded speed the selection is frozen: a stopped car has
  // nothing to gain from a handover. 0 disables. (yaml: freeze_below_speed)
  double freeze_below_speed{0.0};
};

// One postprocess stage. Only the fields its type uses are filled (see compute.cpp).
struct StageSpec
{
  std::string name;
  std::string type;
  Json params;
};

struct OutputSpec
{
  std::string drive_topic{"/drive"};
  std::string frame_id{"base_link"};
  double rate_hz{100.0};
  bool publish_status{true};
};

// One-shot signal sent to the map controller when the car is handed back to
// it, so it can pick the speed up from where the car actually is rather than
// stepping to whatever the plan asks for here.
//
// The controller already knows how to do this - it is the launch-start latch,
// which ramps from the measured wheel speed at a set acceleration and releases
// itself once it has caught up. What it cannot know is WHEN it has just been
// given the car back, because from its side nothing happened: it was
// publishing all along.
struct RampOnReturnSpec
{
  bool enabled{false};                 // direct /launch_start_reset fallback
  std::string topic;                  // empty disables the direct ramp signal
  std::vector<std::string> from;      // handing over FROM one of these
  std::string to;                     // ...TO this drive
};

struct Config
{
  OutputSpec output;
  RampOnReturnSpec ramp_on_return;
  double evaluation_rate_hz{50.0};
  ScoringSpec scoring;
  SelectionSpec selection;
  std::vector<StageSpec> pipeline;
  Influence default_influence;

  std::vector<InputSpec> inputs;
  std::vector<DriveSpec> drives;

  // Reads yaml (the node's ROS parameters) + topics_path (JSON).
  // On failure returns false and a human-readable reason.
  static bool load(
    rclcpp::Node * node, const std::string & topics_path, Config * out, std::string * error);

  // Whether the inputs/drives lists match (decides if hot-reload is allowed).
  bool sameTopology(const Config & other) const;
};

}  // namespace co_driver

#endif  // CO_DRIVER__CONFIG_HPP_
