#include "co_driver/config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>

namespace co_driver
{

// ---------------------------------------------------------------------------
// JSON helpers (also declared in scorer.hpp so scorer implementations can use them)
// ---------------------------------------------------------------------------
double jnum(const Json & j, const std::string & key, double fallback)
{
  if (!j.is_object()) {return fallback;}
  const auto it = j.find(key);
  return (it == j.end() || !it->is_number()) ? fallback : it->get<double>();
}

// All time values in the config are written in ms; internal math uses seconds, so convert here.
double jms(const Json & j, const std::string & key, double fallback_ms)
{
  return jnum(j, key, fallback_ms) * 1e-3;
}

int jint(const Json & j, const std::string & key, int fallback)
{
  return static_cast<int>(std::lround(jnum(j, key, static_cast<double>(fallback))));
}

bool jbool(const Json & j, const std::string & key, bool fallback)
{
  if (!j.is_object()) {return fallback;}
  const auto it = j.find(key);
  return (it == j.end() || !it->is_boolean()) ? fallback : it->get<bool>();
}

std::string jstr(const Json & j, const std::string & key, const std::string & fallback)
{
  if (!j.is_object()) {return fallback;}
  const auto it = j.find(key);
  return (it == j.end() || !it->is_string()) ? fallback : it->get<std::string>();
}

std::vector<std::string> jstrs(const Json & j, const std::string & key)
{
  std::vector<std::string> out;
  if (!j.is_object()) {return out;}
  const auto it = j.find(key);
  if (it == j.end() || !it->is_array()) {return out;}
  for (const auto & e : *it) {
    if (e.is_string()) {out.push_back(e.get<std::string>());}
  }
  return out;
}

std::vector<double> jnums(const Json & j, const std::string & key)
{
  std::vector<double> out;
  if (!j.is_object()) {return out;}
  const auto it = j.find(key);
  if (it == j.end() || !it->is_array()) {return out;}
  for (const auto & e : *it) {
    if (e.is_number()) {out.push_back(e.get<double>());}
  }
  return out;
}

void Influence::merge(const Json & j)
{
  // A bare number = shorthand that specifies only the weight.
  if (j.is_number()) {
    weight = j.get<double>();
    return;
  }
  if (!j.is_object()) {return;}
  weight = jnum(j, "weight", weight);
  exp_mix = jnum(j, "exp_mix", exp_mix);
  exp_k = jnum(j, "exp_k", exp_k);
  invert = jbool(j, "invert", invert);
  in_min = jnum(j, "in_min", in_min);
  in_max = jnum(j, "in_max", in_max);
  veto_below = jnum(j, "veto_below", veto_below);
  required = jbool(j, "required", required);
  last_resort_ok = jbool(j, "last_resort_ok", last_resort_ok);
}

// ---------------------------------------------------------------------------
// yaml (ROS parameters) -- the node starts with automatically_declare_parameters_from_overrides,
// so every key written in the yaml shows up as a parameter. Missing keys keep their defaults.
// ---------------------------------------------------------------------------
namespace
{

double pnum(rclcpp::Node * n, const std::string & key, double fb)
{
  if (!n->has_parameter(key)) {return fb;}
  const auto p = n->get_parameter(key);
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(p.as_int());
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {return p.as_double();}
  return fb;
}

bool pbool(rclcpp::Node * n, const std::string & key, bool fb)
{
  if (!n->has_parameter(key)) {return fb;}
  const auto p = n->get_parameter(key);
  return p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ? p.as_bool() : fb;
}

// yaml-side time values are likewise ms -> s.
double pms(rclcpp::Node * n, const std::string & key, double fb_ms)
{
  return pnum(n, key, fb_ms) * 1e-3;
}

std::string pstr(rclcpp::Node * n, const std::string & key, const std::string & fb)
{
  if (!n->has_parameter(key)) {return fb;}
  const auto p = n->get_parameter(key);
  return p.get_type() == rclcpp::ParameterType::PARAMETER_STRING ? p.as_string() : fb;
}

std::vector<std::string> pstrs(rclcpp::Node * n, const std::string & key)
{
  if (!n->has_parameter(key)) {return {};}
  const auto p = n->get_parameter(key);
  return p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY ?
         p.as_string_array() : std::vector<std::string>{};
}

// Copies the parameter subtree under `prefix` into a JSON object.
// Bridge that lets postprocess stage params and defaults.influence share the JSON-side code.
Json paramsToJson(rclcpp::Node * n, const std::string & prefix)
{
  Json out = Json::object();
  const std::string dotted = prefix + ".";
  for (const auto & full : n->list_parameters({prefix}, 0).names) {
    if (full.size() <= dotted.size() || full.compare(0, dotted.size(), dotted) != 0) {continue;}
    const std::string rest = full.substr(dotted.size());

    Json * cursor = &out;
    std::size_t start = 0;
    while (true) {
      const auto dot = rest.find('.', start);
      const std::string token = rest.substr(start, dot - start);
      if (dot == std::string::npos) {
        const auto p = n->get_parameter(full);
        switch (p.get_type()) {
          case rclcpp::ParameterType::PARAMETER_BOOL: (*cursor)[token] = p.as_bool(); break;
          case rclcpp::ParameterType::PARAMETER_INTEGER: (*cursor)[token] = p.as_int(); break;
          case rclcpp::ParameterType::PARAMETER_DOUBLE: (*cursor)[token] = p.as_double(); break;
          case rclcpp::ParameterType::PARAMETER_STRING: (*cursor)[token] = p.as_string(); break;
          case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            (*cursor)[token] = p.as_double_array(); break;
          case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            (*cursor)[token] = p.as_integer_array(); break;
          case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            (*cursor)[token] = p.as_string_array(); break;
          default: break;
        }
        break;
      }
      if (!cursor->contains(token) || !(*cursor)[token].is_object()) {
        (*cursor)[token] = Json::object();
      }
      cursor = &(*cursor)[token];
      start = dot + 1;
    }
  }
  return out;
}

bool loadYaml(rclcpp::Node * n, Config * c, std::string * error)
{
  auto & o = c->output;
  o.drive_topic = pstr(n, "output.drive_topic", o.drive_topic);
  o.frame_id = pstr(n, "output.frame_id", o.frame_id);
  o.rate_hz = pnum(n, "output.rate_hz", o.rate_hz);
  o.publish_status = pbool(n, "output.publish_status", o.publish_status);

  c->evaluation_rate_hz = pnum(n, "evaluation.rate_hz", c->evaluation_rate_hz);
  c->default_influence.merge(paramsToJson(n, "defaults.influence"));

  auto & s = c->scoring;
  s.combine = pstr(n, "scoring.combine", s.combine);
  s.temperature = pnum(n, "scoring.temperature", s.temperature);
  s.missing = pstr(n, "scoring.missing", s.missing);
  s.ema_alpha = pnum(n, "scoring.ema_alpha", s.ema_alpha);
  s.min_valid_score = pnum(n, "scoring.min_valid_score", s.min_valid_score);
  if (s.combine != "softmax" && s.combine != "weighted_sum") {
    *error = "scoring.combine must be softmax or weighted_sum: " + s.combine;
    return false;
  }
  if (s.missing != "mask" && s.missing != "zero") {
    *error = "scoring.missing must be mask or zero: " + s.missing;
    return false;
  }
  if (s.temperature <= 0.0) {
    *error = "scoring.temperature must be greater than 0.";
    return false;
  }

  auto & sel = c->selection;
  sel.switch_margin = pnum(n, "selection.switch_margin", sel.switch_margin);
  sel.switch_cooldown = pms(n, "selection.switch_cooldown_ms", sel.switch_cooldown * 1e3);
  sel.freeze_below_speed = pnum(n, "selection.freeze_below_speed", sel.freeze_below_speed);
  sel.fallback = pstr(n, "selection.fallback", sel.fallback);
  sel.last_resort = pbool(n, "selection.last_resort", sel.last_resort);

  c->pipeline.clear();
  for (const auto & name : pstrs(n, "postprocess.pipeline")) {
    const std::string prefix = "postprocess." + name;
    StageSpec st;
    st.name = name;
    // If type is omitted, the name is the type.
    st.type = pstr(n, prefix + ".type", name);
    st.params = paramsToJson(n, prefix);
    c->pipeline.push_back(std::move(st));
  }
  return true;
}

// ---------------------------------------------------------------------------
// Tuning overlay - scorer parameters kept apart from the wiring
// ---------------------------------------------------------------------------
// The topics file describes the wiring: which inputs and drives exist, and
// which topics they read. The optional tuning file (`tuning_file` parameter)
// carries only numbers - thresholds, hold times, weights, curve shapes - and
// is deep-merged onto the topics file before parsing, keyed by name:
//
//   {"inputs":  {"<input name>": {"params": {...}, "influence": {...}}},
//    "drives":  {"<drive name>": {"bias": ..., "hold_ms": ...,
//                                 "influence": {"<input name>": {...}}}}}
//
// A name that does not exist in the topics file is an error (typo guard).
void deepMerge(Json & base, const Json & over)
{
  if (!base.is_object() || !over.is_object()) {
    base = over;
    return;
  }
  for (auto it = over.begin(); it != over.end(); ++it) {
    if (base.contains(it.key()) && base[it.key()].is_object() && it.value().is_object()) {
      deepMerge(base[it.key()], it.value());
    } else {
      base[it.key()] = it.value();
    }
  }
}

bool applyTuning(Json & root, const std::string & path, std::string * error)
{
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open tuning file: " + path;
    return false;
  }
  Json tuning;
  try {
    tuning = Json::parse(in, nullptr, true, /*ignore_comments=*/ true);
  } catch (const std::exception & e) {
    *error = std::string("JSON parse failed (") + path + "): " + e.what();
    return false;
  }
  if (!tuning.is_object()) {
    *error = "top level of the tuning file is not a JSON object.";
    return false;
  }
  for (const char * section : {"inputs", "drives"}) {
    if (!tuning.contains(section)) {continue;}
    if (!tuning[section].is_object() || !root.contains(section)) {
      *error = std::string("tuning `") + section + "` must be an object keyed by name.";
      return false;
    }
    for (auto it = tuning[section].begin(); it != tuning[section].end(); ++it) {
      bool found = false;
      for (auto & entry : root[section]) {
        if (jstr(entry, "name", "") == it.key()) {
          deepMerge(entry, it.value());
          found = true;
          break;
        }
      }
      if (!found) {
        *error = std::string("tuning file refers to unknown ") + section + " entry '" +
          it.key() + "'.";
        return false;
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Topics JSON (inputs / drives)
// ---------------------------------------------------------------------------
bool loadTopics(
  const std::string & path, const std::string & tuning_path, Config * c, std::string * error)
{
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open topics file: " + path;
    return false;
  }
  Json root;
  try {
    // Allow comments (//, /* */) so the config file can carry explanations.
    root = Json::parse(in, nullptr, true, /*ignore_comments=*/ true);
  } catch (const std::exception & e) {
    *error = std::string("JSON parse failed (") + path + "): " + e.what();
    return false;
  }
  if (!root.is_object()) {
    *error = "top level of the topics file is not a JSON object.";
    return false;
  }
  // tuning_file may name several files, comma separated, applied in order.
  // The scorers' own parameters are split by subject - localization in one
  // file, obstacle geometry in another - so that changing how obstacles are
  // judged cannot disturb a localization calibration, and so each file can be
  // read on its own without wading through the other.
  if (!tuning_path.empty()) {
    std::size_t from = 0;
    while (from <= tuning_path.size()) {
      const std::size_t comma = tuning_path.find(',', from);
      std::string one = tuning_path.substr(
        from, comma == std::string::npos ? std::string::npos : comma - from);
      // trim
      const auto b = one.find_first_not_of(" \t");
      const auto e = one.find_last_not_of(" \t");
      one = (b == std::string::npos) ? "" : one.substr(b, e - b + 1);
      if (!one.empty() && !applyTuning(root, one, error)) {return false;}
      if (comma == std::string::npos) {break;}
      from = comma + 1;
    }
  }

  // --- inputs ---
  if (!root.contains("inputs") || !root["inputs"].is_array() || root["inputs"].empty()) {
    *error = "an `inputs` array is required (at least one entry).";
    return false;
  }
  std::set<std::string> input_names;
  for (const auto & e : root["inputs"]) {
    InputSpec s;
    s.name = jstr(e, "name", "");
    if (s.name.empty()) {
      *error = "an `inputs` entry has no name.";
      return false;
    }
    if (!input_names.insert(s.name).second) {
      *error = "duplicate input name: " + s.name;
      return false;
    }
    s.type = jstr(e, "type", s.name);
    s.enabled = jbool(e, "enabled", true);
    s.hold = jms(e, "hold_ms", 500.0);
    s.params = e.contains("params") ? e["params"] : Json::object();
    s.influence = c->default_influence;          // start from the yaml defaults
    if (e.contains("influence")) {s.influence.merge(e["influence"]);}
    c->inputs.push_back(std::move(s));
  }

  // --- drives ---
  if (!root.contains("drives") || !root["drives"].is_array() || root["drives"].empty()) {
    *error = "a `drives` array is required (at least one entry).";
    return false;
  }
  std::set<std::string> drive_names;
  for (const auto & e : root["drives"]) {
    DriveSpec d;
    d.name = jstr(e, "name", "");
    if (d.name.empty()) {
      *error = "a `drives` entry has no name.";
      return false;
    }
    if (!drive_names.insert(d.name).second) {
      *error = "duplicate drive name: " + d.name;
      return false;
    }
    d.topic = jstr(e, "topic", "");
    if (d.topic.empty()) {
      *error = "drive '" + d.name + "' has no topic.";
      return false;
    }
    d.enabled = jbool(e, "enabled", true);
    d.hold = jms(e, "hold_ms", 300.0);
    d.keep = e.contains("keep_ms") ? jms(e, "keep_ms", 0.0) : -1.0;
    d.bias = jnum(e, "bias", 0.0);
    if (d.hold <= 0.0) {
      *error = "drive '" + d.name + "': hold_ms must be greater than 0.";
      return false;
    }

    // Influence matrix: yaml defaults <- per-input defaults <- per-drive overrides
    const Json per_drive = (e.contains("influence") && e["influence"].is_object()) ?
      e["influence"] : Json::object();
    for (const auto & in_spec : c->inputs) {
      Influence inf = in_spec.influence;
      const auto it = per_drive.find(in_spec.name);
      if (it != per_drive.end()) {inf.merge(*it);}
      d.influence[in_spec.name] = inf;
    }
    // An entry pointing at a nonexistent input is most likely a typo, so treat it as an error.
    for (auto it = per_drive.begin(); it != per_drive.end(); ++it) {
      if (input_names.find(it.key()) == input_names.end()) {
        *error = "drive '" + d.name + "': influence refers to unknown input '" + it.key() + "'.";
        return false;
      }
    }
    c->drives.push_back(std::move(d));
  }

  if (!c->selection.fallback.empty() &&
    drive_names.find(c->selection.fallback) == drive_names.end())
  {
    *error = "selection.fallback '" + c->selection.fallback + "' is not in the drives list.";
    return false;
  }
  return true;
}

}  // namespace

bool Config::load(
  rclcpp::Node * node, const std::string & topics_path, Config * out, std::string * error)
{
  Config c;
  std::string err;
  if (!loadYaml(node, &c, &err)) {
    *error = err;
    return false;
  }
  if (topics_path.empty()) {
    *error = "set the `topics_file` parameter to the path of the topics JSON.";
    return false;
  }
  if (!loadTopics(topics_path, pstr(node, "tuning_file", ""), &c, &err)) {
    *error = err;
    return false;
  }
  *out = std::move(c);
  return true;
}

bool Config::sameTopology(const Config & other) const
{
  if (inputs.size() != other.inputs.size() || drives.size() != other.drives.size()) {
    return false;
  }
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    if (inputs[i].name != other.inputs[i].name || inputs[i].type != other.inputs[i].type) {
      return false;
    }
  }
  for (std::size_t i = 0; i < drives.size(); ++i) {
    if (drives[i].name != other.drives[i].name || drives[i].topic != other.drives[i].topic) {
      return false;
    }
  }
  return true;
}

}  // namespace co_driver
