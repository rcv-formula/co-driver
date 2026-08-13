// co_driver 메인 노드.
//
//   /drive_* 구독 ──▶ 채점기들(src/scorers/) ──▶ 점수 연산(compute) ──▶ 선택 ──▶ 후처리 ──▶ /drive
//
// 이 노드는 **센서를 구독하지 않습니다.** scan/map/imu/odom 을 보고 판단하는 일은
// 각 채점기가 자기 구독자로 직접 합니다. 노드가 하는 일은 넷뿐입니다.
//   1) 설정을 읽어 채점기와 /drive 구독을 만든다
//   2) 평가 주기마다: 채점 -> 점수 연산 -> 선택
//   3) 출력 주기마다: 후처리 -> /drive 발행
//   4) 진단 발행 / 핫리로드 서비스
//
// 평가(느려도 됨)와 출력(100Hz 고정)을 별도 타이머로 분리해, 채점이 무거워져도
// 출력 주기가 흔들리지 않습니다.
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
  os.precision(6);              // softmax 확률의 합이 1 로 읽히도록
  os << std::fixed << v;
  return os.str();
}

}  // namespace

// ===========================================================================
// 선택 — 어느 드라이브를 쓸지. 채터링 방지를 위한 히스테리시스 + 최소 유지 시간.
// ===========================================================================
class Selector
{
public:
  struct Result
  {
    bool has_selection{false};
    std::string name;          // 1등 = 실제로 쓰는 드라이브
    std::string reason;
    bool switched{false};
    // 점수 2등 — 합산 점수만으로 매긴 순위입니다. 선택(name)과는 무관하며,
    // 히스테리시스 때문에 selected 가 점수 1등이 아니면 selected 와 같을 수도 있습니다.
    // (드라이브별 점수 순위는 status 의 rank 필드에 전부 실립니다)
    std::string runner_up;
    bool has_runner_up{false};
  };

  void configure(const SelectionSpec & spec) {spec_ = spec;}
  const std::string & current() const {return current_;}

  Result select(const std::vector<Drive> & drives, const rclcpp::Time & now)
  {
    Result r = selectFirst(drives, now);

    // 점수 순위 — 유효 드라이브를 합산 점수로만 줄 세워 1·2등을 뽑습니다.
    // 선택(r.name)은 히스테리시스가 걸려 있어 1등과 다를 수 있고, 여기는 그와 무관합니다.
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

    // valid 는 active(신뢰 창 안 + 하드 게이트 통과)를 이미 함의합니다.
    // 즉 여기 들어오는 것은 전부 "지금 살아 있는" 드라이브뿐입니다.
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
      // 아무도 못 씀 -> fallback 시도
      if (!spec_.fallback.empty()) {
        for (const auto & d : drives) {
          if (d.name != spec_.fallback) {continue;}
          // fallback 도 유효(=활성)해야 씁니다. 죽은 명령을 되살려 쓰지 않습니다.
          if (d.valid) {return commit(d.name, "fallback", now);}
        }
      }
      r.reason = "유효한 드라이브 없음";
      current_.clear();
      return r;
    }

    if (!incumbent_ok) {
      // 현재 것이 죽었으면 히스테리시스 없이 즉시 갈아탑니다.
      return commit(best->name, current_.empty() ? "초기 선택" : "현재 드라이브 무효 -> 즉시 전환", now);
    }

    r.has_selection = true;
    if (best->name == current_) {
      r.name = current_;
      r.reason = "유지";
      return r;
    }

    const double held = switched_once_ ?
      (now - last_switch_).seconds() : spec_.switch_cooldown + 1.0;
    if (spec_.switch_cooldown > 0.0 && held < spec_.switch_cooldown) {
      r.name = current_;
      r.reason = "switch_cooldown 대기 중";
      return r;
    }
    if (best->score < incumbent->score + spec_.switch_margin) {
      r.name = current_;
      r.reason = "switch_margin 미달";
      return r;
    }
    return commit(best->name, "점수 우위로 전환", now);
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
// 후처리 — yaml 의 postprocess.pipeline 순서대로 최종 명령을 다듬습니다.
// 새 단계를 넣으려면 아래 enum + 파싱 + apply 에 각각 한 case 씩 추가하세요.
// ===========================================================================
struct PostContext
{
  double dt{0.0};
  bool has_command{false};   // 이번 주기에 쓸 드라이브가 있었는가
  bool switched{false};      // 이번 주기에 선택이 바뀌었는가
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
      RCLCPP_WARN(log, "postprocess 가 비어 있습니다. 후처리 없이 그대로 내보냅니다.");
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
          RCLCPP_WARN(log, "switch_blend.curve '%s' 알 수 없음 -> smooth", st.curve.c_str());
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
        // 런타임에 ros2 param set 으로 바꿀 수 있게 선언해 둡니다.
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
          log, "후처리 '%s' 의 type '%s' 을(를) 모릅니다. 사용 가능: "
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
            if (pc.has_command) {break;}   // 쓸 드라이브가 없을 때만 개입
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
              // 블렌딩 도중 또 바뀌면 "지금 나가고 있던 값"에서 다시 시작합니다.
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
            // 가속/감속은 속도의 절댓값이 커지는가로 구분합니다(후진에서도 성립).
            const double dv = cmd.speed - pc.previous.speed;
            const bool speeding_up = std::abs(cmd.speed) > std::abs(pc.previous.speed);
            const double limit = (speeding_up ? s.max_accel : s.max_decel) * pc.dt;
            if (limit > 0.0 && std::abs(dv) > limit) {
              cmd.speed = static_cast<float>(pc.previous.speed + std::copysign(limit, dv));
            }
            break;
          }

        case Type::SpeedScale: {
            // 매 주기 읽어야 런타임 변경이 즉시 반영됩니다.
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
    // switch_blend 상태
    bool blend_active{false};
    double blend_elapsed{0.0};
    ackermann_msgs::msg::AckermannDrive blend_from;
  };

  rclcpp::Node * node_{nullptr};
  std::vector<Stage> stages_;
};

// ===========================================================================
// 노드
// ===========================================================================
class CoDriverNode : public rclcpp::Node
{
public:
  explicit CoDriverNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("co_driver_node", rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true))
  {
    // 기본 설정은 yaml(ROS 파라미터), 늘어나는 목록은 이 JSON 에서 옵니다.
    if (!has_parameter("topics_file")) {declare_parameter<std::string>("topics_file", "");}
    topics_path_ = get_parameter("topics_file").as_string();

    // 콜백 그룹을 나눠야 MultiThreadedExecutor 가 실제로 병렬로 돕니다.
    // 기본 그룹은 MutuallyExclusive 라, 그대로 두면 무거운 채점기 하나가
    // 출력 타이머까지 막아 /drive 주기가 흔들립니다.
    eval_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    output_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scorer_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    std::string err;
    if (!Config::load(this, topics_path_, &cfg_, &err)) {throw std::runtime_error(err);}
    if (!build(&err)) {throw std::runtime_error(err);}

    // --- /drive 후보 구독 (노드가 구독하는 것은 이것뿐입니다) ---
    drive_subs_.resize(drives_.size());
    for (std::size_t i = 0; i < drives_.size(); ++i) {
      const auto & d = drives_[i];
      drive_subs_[i] = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        d.topic, rclcpp::QoS(10),
        [this, i](const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock(mtx_);
          drives_[i].cmd = *m;
          // 퍼블리셔가 stamp 를 안 채우는 경우가 있어 수신 시각을 씁니다.
          // noteReceived 가 수신 주기(EMA)도 같이 갱신합니다 — 신뢰 창 자동 산출용.
          drives_[i].noteReceived(now());
        });
      RCLCPP_INFO(
        get_logger(), "드라이브 [%zu] %s <- %s (hold %.0fms)",
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
      get_logger(), "co_driver 기동: 채점기 %zu개 × 드라이브 %zu개 -> %s (평가 %.0fHz / 출력 %.0fHz)",
      scorers_.size(), drives_.size(), cfg_.output.drive_topic.c_str(),
      cfg_.evaluation_rate_hz, cfg_.output.rate_hz);
  }

private:
  // -------------------------------------------------------------------------
  // 설정 -> 채점기 / 드라이브 / 후처리
  // -------------------------------------------------------------------------
  bool build(std::string * error)
  {
    scorers_.clear();
    for (const auto & in : cfg_.inputs) {
      if (!in.enabled) {
        RCLCPP_INFO(get_logger(), "입력 '%s' 는 enabled=false 라 건너뜁니다.", in.name.c_str());
        continue;
      }
      auto s = ScorerRegistry::instance().create(in.type);
      if (!s) {
        std::string known;
        for (const auto & t : ScorerRegistry::instance().types()) {known += t + " ";}
        *error = "입력 '" + in.name + "': type '" + in.type + "' 을(를) 모릅니다. 사용 가능: " + known;
        return false;
      }
      s->setContext(in.name, in.type, scorer_group_);
      // 채점기가 여기서 자기 구독자를 만듭니다.
      if (!s->configure(this, in.name, in.params)) {
        *error = "입력 '" + in.name + "' 설정 실패";
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
      *error = "후처리 파이프라인 설정 실패";
      return false;
    }
    return true;
  }

  // 계수만 바꿨을 때 재시작 없이 다시 읽습니다(구독을 새로 만들어야 하면 거부).
  bool reload(std::string * message)
  {
    Config fresh;
    std::string err;
    if (!Config::load(this, topics_path_, &fresh, &err)) {
      *message = "리로드 실패: " + err;
      return false;
    }
    if (!fresh.sameTopology(cfg_)) {
      *message =
        "리로드 실패: inputs/drives 구성이 바뀌었습니다(이름·타입·토픽). "
        "구독을 다시 만들어야 하므로 노드를 재시작하세요.";
      return false;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    // 드라이브의 수신 상태는 보존하고 계수만 갈아끼웁니다.
    for (const auto & spec : fresh.drives) {
      for (auto & d : drives_) {
        if (d.name != spec.name) {continue;}
        d.enabled = spec.enabled;
        d.hold = spec.hold;
        d.bias = spec.bias;
        d.influence = spec.influence;
      }
    }
    cfg_ = fresh;   // 토픽·주기는 구독/타이머에 묶여 있어 반영되지 않습니다
    selector_.configure(cfg_.selection);
    if (!post_.configure(this, cfg_.pipeline, get_logger())) {
      *message = "리로드 실패: 후처리 파이프라인 설정 오류";
      return false;
    }
    post_.reset();
    *message = "리로드 완료 (yaml 파라미터 + " + topics_path_ + ")";
    return true;
  }

  // -------------------------------------------------------------------------
  // 채점 -> 점수 연산 -> 선택
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
            // 가중치 0 인 입력도 채점합니다 — veto 판정과 상태 표시에 필요합니다.
            // 비동기 채점기가 pending 을 내면 직전 점수를 hold 동안 대신 씁니다.
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
  // 후처리 -> 발행
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
      // switched 는 출력 주기가 한 번만 소비해야 blend 가 정확히 한 번 무장됩니다.
      pc.switched = switch_pending_;
      switch_pending_ = false;

      if (have_command_ && !selected_.empty()) {
        for (const auto & d : drives_) {
          if (d.name == selected_) {cmd = d.cmd.drive; break;}
        }
      } else if (has_output_) {
        // 쓸 드라이브가 없으면 직전 출력에서 출발해 timeout_stop 이 감속시킵니다.
        cmd = output_;
      }

      post_.apply(cmd, pc);
      output_ = cmd;
      has_output_ = true;
    }

    // 쓸 드라이브가 없어도 계속 발행합니다 — timeout_stop 의 감속이 차에 닿아야 합니다.
    ackermann_msgs::msg::AckermannDriveStamped out;
    out.header.stamp = t;
    out.header.frame_id = cfg_.output.frame_id;
    out.drive = cmd;
    drive_pub_->publish(out);
  }

  // -------------------------------------------------------------------------
  // 진단 발행
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
      // 합산 점수 2등. 유효 드라이브가 1개뿐이면 빈 문자열.
      std_msgs::msg::String s;
      s.data = sel.has_runner_up ? sel.runner_up : "";
      runner_up_pub_->publish(s);
    }
    if (scores_pub_) {
      // layout.dim[i].label 에 이름을 실어 외부에서 인덱스 없이 짝지을 수 있게 합니다.
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

    // 유효 드라이브를 점수순으로 세워 rank(1등, 2등, ...)를 매깁니다.
    // 히스테리시스 때문에 selected 가 rank 1 이 아닐 수 있고, 그 차이가 보이는 게 목적입니다.
    std::vector<const Drive *> ranked;
    for (const auto & d : drives_) {
      if (d.valid) {ranked.push_back(&d);}   // 활성 드라이브만 순위에 들어갑니다
    }
    std::sort(
      ranked.begin(), ranked.end(),
      [](const Drive * a, const Drive * b) {return a->score > b->score;});
    std::map<std::string, int> rank_of;
    for (std::size_t i = 0; i < ranked.size(); ++i) {rank_of[ranked[i]->name] = static_cast<int>(i) + 1;}

    // 선형층의 전 단계를 그대로 보여 줍니다: x(채점기 원점수) -> s(φ 성형) -> c(W·φ)
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
         // active = 지금 살아 있어 순위·선택 대상인가. valid = active + min_valid_score.
         << ",\"active\":" << (d.active ? "true" : "false")
         << ",\"valid\":" << (d.valid ? "true" : "false")
         << ",\"age_ms\":" << num(d.age(ctx_.now) * 1e3)
         << ",\"hold_ms\":" << num(d.hold * 1e3)          // 마지막 명령의 유효 시간
         << ",\"hz\":" << num(d.measuredHz())         // 실측 수신 주파수
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
            // 비동기 채점기의 직전 점수를 대신 쓴 경우 그 나이를 함께 보여 줍니다.
            const auto ha = held_age_.find(d.name);
            if (ha != held_age_.end()) {
              const auto hb = ha->second.find(kv.first);
              if (hb != ha->second.end() && hb->second >= 0.0) {
                os << ",\"held_ms\":" << num(hb->second * 1e3);
              }
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

  // --- 상태 ---
  std::string topics_path_;
  Config cfg_;
  // 채점기와 그 입력의 hold(직전 점수 유효 시간)를 함께 들고 있습니다.
  struct ScorerEntry
  {
    ScorerPtr scorer;
    double hold{0.5};
  };
  std::vector<ScorerEntry> scorers_;
  // 진단용: 이번 주기에 캐시로 메운 점수의 나이[s] (아니면 음수)
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
    // 채점(무거울 수 있음)과 출력(고정 주기)이 서로를 막지 않도록 멀티스레드로 돕니다.
    // 공유 상태는 노드 내부 뮤텍스가 지킵니다.
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<co_driver::CoDriverNode>(rclcpp::NodeOptions());
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    const std::string what = e.what();
    RCLCPP_FATAL(rclcpp::get_logger("co_driver"), "%s", what.c_str());
    // yaml 의 빈 리스트(`pipeline: []`)는 타입이 없어 파라미터 자동 선언에서 터집니다.
    if (what.find("No parameter value set") != std::string::npos) {
      RCLCPP_FATAL(
        rclcpp::get_logger("co_driver"),
        "yaml 에 빈 리스트(`[]`)나 값 없는 키가 있으면 ROS 파라미터로 올라오지 못합니다. "
        "해당 항목을 지우거나 값을 채우세요 (예: 후처리를 끄려면 postprocess 블록 전체 삭제).");
    }
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
