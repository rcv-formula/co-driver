// 점수 연산 — 입력 점수 x 를 드라이브별 최종 점수 p 로 바꾸는 것, 그것만 있습니다.
//
//   φ_ji(x_i)  응답 곡선 (선형 + 지수)         shapeInfluence()
//   z_j        선형층 Σ W[j][i]·φ + b_j        computeLogit()
//   p_j        softmax(z / T)                  finalizeScores()
//
// "어떤 정보로 x 를 만들 것인가"(scan/map/imu/odom 을 보고 판단하는 일)는 전부
// src/scorers/ 의 채점기가 합니다. 채점기는 자기가 필요한 토픽을 스스로 구독하므로
// 이 파일도, 노드도 센서를 알 필요가 없습니다.
#ifndef CO_DRIVER__COMPUTE_HPP_
#define CO_DRIVER__COMPUTE_HPP_

#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "co_driver/config.hpp"

namespace co_driver
{

// 채점기 하나가 드라이브 하나에 대해 내놓는 결과.
//
// 네 가지 상태가 있습니다.
//   ok          점수를 냈다. 캐시에 저장되고 그대로 쓰입니다.
//   pending     아직 계산 중이다(비동기 채점기). 프레임워크가 **직전 점수를 대신 씁니다**.
//               단 그 점수가 inputs[].hold_ms 를 넘겼으면 unavailable 로 떨어집니다.
//   unavailable 판단 불가(데이터 없음 등). 명시적 선언이라 캐시로 덮지 않습니다.
//   vetoed      실격.
struct ScoreResult
{
  double value{0.0};        // [0,1]. 1이 좋음. 이 값이 응답 곡선의 x
  bool available{true};     // false = 이번 주기 판단 불가 -> 가중치째 제외
  bool veto{false};         // true = 점수와 무관하게 드라이브 실격
  bool is_pending{false};   // true = 계산 중. 직전 점수를 hold_ms 동안 대신 씀
  std::string note;

  static ScoreResult ok(double v, const std::string & why = "")
  {
    ScoreResult r;
    r.value = v;
    r.note = why;
    return r;
  }
  static ScoreResult unavailable(const std::string & why = "")
  {
    ScoreResult r;
    r.available = false;
    r.note = why;
    return r;
  }
  static ScoreResult vetoed(const std::string & why = "")
  {
    ScoreResult r;
    r.veto = true;
    r.note = why;
    return r;
  }
  // 비동기 채점기가 "아직 새 결과 없음"을 알릴 때. 프레임워크가 직전 점수를 씁니다.
  static ScoreResult pending(const std::string & why = "")
  {
    ScoreResult r;
    r.available = false;
    r.is_pending = true;
    r.note = why;
    return r;
  }
};

// 입력별 마지막 유효 점수. pending 동안 이 값을 대신 씁니다.
struct CachedScore
{
  double value{0.0};
  rclcpp::Time stamp;
  bool valid{false};
};

// 드라이브 하나의 설정 + 런타임 상태.
struct Drive
{
  // 설정 (DriveSpec 사본)
  std::string name;
  std::string topic;
  bool enabled{true};
  double hold{0.3};       // 마지막 명령의 유효 시간[s]
  double bias{0.0};
  std::map<std::string, Influence> influence;

  // 수신 상태
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  rclcpp::Time last_rx;
  bool has_cmd{false};
  // 실제 수신 주기(EMA) — 진단용입니다. hold 를 얼마로 잡을지 고를 때 참고하세요.
  double period_ema{0.0};
  int rx_count{0};

  // 입력 이름 -> 마지막 유효 점수 (비동기 채점기의 pending 구간을 메웁니다)
  std::map<std::string, CachedScore> score_cache;

  // 이번 주기 결과
  std::map<std::string, ScoreResult> results;   // 채점기 이름 -> 점수
  double raw_logit{0.0};      // z_j = ΣW·φ + b (이번 주기)
  double logit{0.0};          // raw_logit 에 EMA 를 먹인 값. softmax 는 이 값에 걸림
  bool logit_initialized{false};
  double score{0.0};          // 최종 점수. softmax 모드면 확률 p_j
  // active = 하드 게이트를 통과해 "지금 쓸 수 있는" 상태.
  // 비활성 드라이브는 점수는 계속 계산되지만 softmax·순위·선택에서 빠집니다.
  bool active{false};
  bool valid{false};          // active && score >= min_valid_score
  std::string reject;         // 비활성/무효 사유

  bool isFresh(const rclcpp::Time & now) const
  {
    if (!has_cmd) {return false;}
    return (now - last_rx).seconds() <= hold;
  }
  double age(const rclcpp::Time & now) const
  {
    if (!has_cmd) {return std::numeric_limits<double>::infinity();}
    return (now - last_rx).seconds();
  }
  double measuredHz() const {return period_ema > 1e-9 ? 1.0 / period_ema : 0.0;}

  // 메시지를 받을 때마다 수신 주기를 갱신합니다(진단용 Hz 표시).
  void noteReceived(const rclcpp::Time & now)
  {
    if (has_cmd) {
      const double dt = (now - last_rx).seconds();
      // 두절 후 복귀처럼 비정상적으로 긴 간격은 주기 추정에 넣지 않습니다.
      if (dt > 1e-6 && dt < 2.0) {
        period_ema = (rx_count < 1) ? dt : period_ema + 0.2 * (dt - period_ema);
        ++rx_count;
      }
    }
    last_rx = now;
    has_cmd = true;
  }
};

// 채점기가 공유하는 최소 정보. 센서는 여기 없습니다 — 채점기가 직접 구독합니다.
// 여기 있는 것은 "채점기가 스스로 알 수 없는" 노드 내부 상태뿐입니다.
struct Context
{
  rclcpp::Time now;
  double dt{0.0};                                  // 직전 평가 주기와의 간격[s]
  const std::vector<Drive> * drives{nullptr};      // 드라이브 간 상대 비교용
  ackermann_msgs::msg::AckermannDrive last_output; // 직전에 실제로 발행한 명령
  bool has_last_output{false};
  std::string last_selected;                       // 직전에 선택된 드라이브 이름
};

// ---------------------------------------------------------------------------
// 응답 곡선 φ — 채점기가 낸 x ∈ [0,1] 을 선형층 입력으로 성형합니다.
//
//   u = invert ? 1-x : x
//   u = clamp((u - in_min) / (in_max - in_min), 0, 1)
//   s = (1 - exp_mix)·u + exp_mix·(e^(k·u) - 1)/(e^k - 1)
//
// 지수항이 [0,1] -> [0,1] 정규화형이라 결과도 [0,1] 입니다. 배합비를 바꿔도
// 범위가 변하지 않으므로 별도 정규화가 필요 없습니다.
// ---------------------------------------------------------------------------
double shapeInfluence(const Influence & inf, double x);

// ---------------------------------------------------------------------------
// 채점 결과 확정 — 비동기 채점기의 pending 을 캐시로 메우고, 오래된 것은 버립니다.
//
//   ok       -> 캐시 갱신 후 그대로 사용
//   pending  -> 캐시가 hold 안이면 그 값을 사용, 넘었으면 unavailable
//   그 외    -> 그대로 통과 (명시적 판단이므로 캐시로 덮지 않음)
//
// hold <= 0 이면 만료 없이 캐시를 계속 씁니다.
// held_age 에는 캐시를 썼을 때 그 점수의 나이[s]가 담깁니다(진단용, 아니면 음수).
// ---------------------------------------------------------------------------
ScoreResult resolveScore(
  Drive & d, const std::string & input, const ScoreResult & fresh, double hold,
  const rclcpp::Time & now, double * held_age = nullptr);

// ---------------------------------------------------------------------------
// 1층 선형 MLP + softmax
// ---------------------------------------------------------------------------
// 1단계: 드라이브 하나의 선형층 출력 z_j 와 활성 여부(active).
//
// 점수는 **활성 여부와 무관하게 계산**합니다. hold 를 넘긴 드라이브도 z 를 그대로
// 볼 수 있어야 "왜 안 뽑혔는지"를 판단할 수 있기 때문입니다. 대신 active=false 가 되어
// 다음 단계의 softmax·순위·선택에서 빠집니다.
//
// active 를 끄는 하드 게이트: disabled / 미수신 / hold 초과(stale) / NaN /
//                             채점기 veto / veto_below / required 입력 판단 불가
void computeLogit(Drive & d, const ScoringSpec & spec, const rclcpp::Time & now);

// 2단계: **활성 드라이브들만** 모아 softmax 를 걸어 score / valid 를 채웁니다.
//        비활성은 분모에서 빠지므로 활성인 것들의 확률 합이 항상 1 입니다.
void finalizeScores(std::vector<Drive> & drives, const ScoringSpec & spec);

}  // namespace co_driver

#endif  // CO_DRIVER__COMPUTE_HPP_
