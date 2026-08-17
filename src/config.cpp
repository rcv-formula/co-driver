#include "co_driver/config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>

namespace co_driver
{

// ---------------------------------------------------------------------------
// JSON 헬퍼 (scorer.hpp 에도 선언되어 있어 채점기 구현에서 쓸 수 있습니다)
// ---------------------------------------------------------------------------
double jnum(const Json & j, const std::string & key, double fallback)
{
  if (!j.is_object()) {return fallback;}
  const auto it = j.find(key);
  return (it == j.end() || !it->is_number()) ? fallback : it->get<double>();
}

// 설정의 시간 값은 전부 ms 로 적습니다. 내부 계산은 초로 하므로 여기서 변환합니다.
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
  // 숫자 하나 = 가중치만 지정한 축약형.
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
}

// ---------------------------------------------------------------------------
// yaml (ROS 파라미터) — 노드는 automatically_declare_parameters_from_overrides
// 로 뜨므로 yaml 에 적힌 키가 그대로 파라미터로 올라옵니다. 없는 키는 기본값.
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

// yaml 쪽 시간 값도 동일하게 ms -> s.
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

// `prefix` 아래 파라미터 하위 트리를 JSON 객체로 옮깁니다.
// 후처리 단계 params 와 defaults.influence 를 JSON 쪽과 같은 코드로 다루기 위한 다리.
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
  sel.fallback = pstr(n, "selection.fallback", sel.fallback);

  c->pipeline.clear();
  for (const auto & name : pstrs(n, "postprocess.pipeline")) {
    const std::string prefix = "postprocess." + name;
    StageSpec st;
    st.name = name;
    // type 을 생략하면 이름이 곧 타입입니다.
    st.type = pstr(n, prefix + ".type", name);
    st.params = paramsToJson(n, prefix);
    c->pipeline.push_back(std::move(st));
  }
  return true;
}

// ---------------------------------------------------------------------------
// 토픽 목록 JSON (inputs / drives)
// ---------------------------------------------------------------------------
bool loadTopics(const std::string & path, Config * c, std::string * error)
{
  std::ifstream in(path);
  if (!in) {
    *error = "cannot open topics file: " + path;
    return false;
  }
  Json root;
  try {
    // 주석(//, /* */)을 허용해 설정 파일에 설명을 달 수 있게 합니다.
    root = Json::parse(in, nullptr, true, /*ignore_comments=*/ true);
  } catch (const std::exception & e) {
    *error = std::string("JSON parse failed (") + path + "): " + e.what();
    return false;
  }
  if (!root.is_object()) {
    *error = "top level of the topics file is not a JSON object.";
    return false;
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
    s.influence = c->default_influence;          // yaml defaults 에서 출발
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
    d.bias = jnum(e, "bias", 0.0);
    if (d.hold <= 0.0) {
      *error = "drive '" + d.name + "': hold_ms must be greater than 0.";
      return false;
    }

    // 영향 행렬: yaml defaults <- 입력별 기본 <- 드라이브별 지정
    const Json per_drive = (e.contains("influence") && e["influence"].is_object()) ?
      e["influence"] : Json::object();
    for (const auto & in_spec : c->inputs) {
      Influence inf = in_spec.influence;
      const auto it = per_drive.find(in_spec.name);
      if (it != per_drive.end()) {inf.merge(*it);}
      d.influence[in_spec.name] = inf;
    }
    // 없는 입력을 가리키는 항목은 오타일 확률이 높아 오류로 잡습니다.
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
  if (!loadTopics(topics_path, &c, &err)) {
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
