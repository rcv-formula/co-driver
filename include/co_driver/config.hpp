// 설정 — yaml(ROS 파라미터) + 토픽 목록 JSON 을 읽어 구조체로 만듭니다.
//
//   yaml : output / evaluation / defaults.influence / scoring / selection
//          / postprocess                           — 개수가 고정된 기본 설정
//   json : inputs[] / drives[]                     — 계속 늘어나는 토픽 목록
//
// 이 파일에는 "값"만 있습니다. 값을 가지고 하는 계산은 전부 compute.hpp 에 있습니다.
#ifndef CO_DRIVER__CONFIG_HPP_
#define CO_DRIVER__CONFIG_HPP_

#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace co_driver
{

using Json = nlohmann::json;

// ---------------------------------------------------------------------------
// 입력 하나가 드라이브 하나의 점수에 미치는 영향.
//
// 점수 산정은 1층 선형 MLP + softmax 입니다.
//   z_j = Σ_i W[j][i]·φ_ji(x_i) + b_j     W = weight, b = DriveSpec::bias
//   p   = softmax(z / temperature)
// φ 는 아래 파라미터가 정의하는 선형+지수 응답 곡선이고, 실제 계산은
// compute.hpp 의 shapeInfluence() 에 있습니다.
// ---------------------------------------------------------------------------
struct Influence
{
  double weight{0.0};     // 선형층 가중치 W. 음수 가능(감점 항). 0이면 점수 미반영
  // 선형 성분과 지수 성분의 배합비 하나로 표현합니다.
  //   0 = 순수 선형, 1 = 순수 지수, 0.7 = 지수 7 : 선형 3
  // (예전의 linear/exp 두 값은 비율만 의미가 있어 자유도가 하나 남았습니다)
  double exp_mix{0.0};
  double exp_k{3.0};      // 지수 곡률. >0 볼록, <0 오목, ~0 선형
  bool invert{false};     // 작을수록 좋은 지표를 뒤집음
  double in_min{0.0};     // 관심 구간 재정규화
  double in_max{1.0};
  double veto_below{-1.0};  // 성형 후 값이 이 미만이면 드라이브 실격(음수면 끔)
  bool required{false};     // 이 입력이 판단 불가일 때 드라이브 실격

  // 숫자 하나면 weight 로, 객체면 필드별로 덮어씁니다.
  void merge(const Json & j);
};

// 입력 하나 = 채점기(Scorer) 인스턴스 하나. JSON inputs[] 의 원소.
//
// hold — 채점기가 pending(계산 중)을 낼 때 직전 점수를 얼마나 더 쓸 것인가.
// 비동기 채점기 전용이고, 매 주기 값을 내는 동기 채점기에는 영향이 없습니다.
struct InputSpec
{
  std::string name;
  std::string type;       // 비우면 name 을 type 으로
  bool enabled{true};
  double hold{0.5};       // 직전 점수의 유효 시간[s] (JSON: hold_ms, 0 이하면 무제한)
  Json params;            // 채점기에 그대로 전달 (구독할 토픽 이름 등)
  Influence influence;    // 이 입력의 드라이브 공통 기본 영향
};

// 드라이브 하나 = /drive 후보. JSON drives[] 의 원소.
//
// hold — "마지막으로 받은 명령을 언제까지 붙들고 있을 것인가". 시간 관련 설정은
// 드라이브당 이것 하나뿐입니다. 이 시간을 넘기면 비활성(inactive)이 되어
// 순위·선택에서 빠집니다.
//
// 토픽마다 Hz 가 달라도 여기에 각자 값을 적으면 됩니다(보통 그 토픽 주기의 3~5배).
// 실측 Hz 는 status 의 hz 에 찍히니 그걸 보고 정하세요.
struct DriveSpec
{
  std::string name;
  std::string topic;
  bool enabled{true};
  double hold{0.3};       // 마지막 명령의 유효 시간[s] (JSON: hold_ms)
  double bias{0.0};       // 선형층 편향 b_j
  std::map<std::string, Influence> influence;   // 입력 이름 -> 영향 (전 입력에 대해 채워짐)
};

struct ScoringSpec
{
  std::string combine{"softmax"};   // softmax | weighted_sum
  double temperature{1.0};          // ↓ 승자독식 / ↑ 평탄
  std::string missing{"mask"};      // 판단 불가 입력 처리: mask | zero
  double ema_alpha{1.0};            // softmax 이전 logit 에 거는 저역통과
  double min_valid_score{0.0};
};

struct SelectionSpec
{
  double switch_margin{0.05};
  // 전환 직후 이 시간 동안은 다시 전환하지 않습니다(채터링 방지).
  // 드라이브의 hold 와는 무관합니다 — 이름이 헷갈리지 않게 cooldown 으로 부릅니다.
  double switch_cooldown{0.5};     // [s] (yaml: switch_cooldown_ms)
  std::string fallback;   // 전원 실격일 때 마지막으로 시도할 드라이브(이것도 유효해야 함)
};

// 후처리 단계 하나. type 별로 쓰는 필드만 채워집니다(compute.cpp 참고).
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

struct Config
{
  OutputSpec output;
  double evaluation_rate_hz{50.0};
  ScoringSpec scoring;
  SelectionSpec selection;
  std::vector<StageSpec> pipeline;
  Influence default_influence;

  std::vector<InputSpec> inputs;
  std::vector<DriveSpec> drives;

  // yaml(노드의 ROS 파라미터) + topics_path(JSON) 을 읽습니다.
  // 실패하면 false 와 사람이 읽을 수 있는 사유.
  static bool load(
    rclcpp::Node * node, const std::string & topics_path, Config * out, std::string * error);

  // inputs/drives 목록이 같은가 (핫리로드 허용 여부 판정용).
  bool sameTopology(const Config & other) const;
};

}  // namespace co_driver

#endif  // CO_DRIVER__CONFIG_HPP_
