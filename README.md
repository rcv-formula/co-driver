# co_driver

여러 개의 `/drive` 후보 중 하나를 **점수로 골라** 후처리한 뒤 VESC 스택으로 내보내는 중재 노드.

```
PPcontroller #1 ──▶ /drive_main  ┐
PPcontroller #2 ──▶ /drive_left  ├─▶ co_driver ──▶ /drive ──▶ ackermann_mux ──▶ ackermann_to_vesc ──▶ VESC
PPcontroller #3 ──▶ /drive_right ┘      ▲
                                        │
                              채점기들이 각자 필요한 토픽을 직접 구독
                              (/localization_confidence, /scan, /map, ...)
```

출력은 `ackermann_mux` 의 **navigation 입력**(`f1tenth_stack/config/mux.yaml` 의 `drive`)으로
갑니다. 이렇게 해야 조이스틱(priority 100) 오버라이드와 mux 의 timeout 안전장치가 그대로
살아 있습니다. mux 를 우회하려면 `output.drive_topic` 을 `/ackermann_drive` 로 바꾸세요.

---

## 구조

파일은 넷뿐이고 역할이 겹치지 않습니다.

| 파일 | 하는 일 |
|---|---|
| [src/co_driver_node.cpp](src/co_driver_node.cpp) | 메인 노드 — `/drive_*` 구독, 선택, 후처리, 발행, 핫리로드 |
| [src/config.cpp](src/config.cpp) | 설정 읽기 — yaml(ROS 파라미터) + 토픽 목록 JSON |
| [src/compute.cpp](src/compute.cpp) | **점수 연산만** — 응답 곡선 φ, 선형층, softmax |
| [src/scorers/](src/scorers/) | **채점기** — 하나에 .cpp 하나. 각자 필요한 토픽을 스스로 구독, 동기·비동기 모두 가능 |

```
/drive_* ──▶ 채점기들 ──▶ 점수 연산 ──▶ 선택 ──▶ 후처리 ──▶ /drive
             (scorers)    (compute)     (node)   (node)     100Hz
                 ▲
        각 채점기가 자기 구독자로
        scan / map / imu / 외부 점수 토픽을 직접 받음
```

**노드는 센서를 구독하지 않습니다.** `/scan` 을 보고 판단하고 싶으면 노드가 아니라
`src/scorers/` 에 파일 하나를 추가합니다. 그래서 판단 로직이 늘어나도 노드·설정 로더·
점수 연산은 그대로입니다.

평가(느려도 됨)와 출력(100Hz 고정)은 **별도 타이머 + 별도 콜백 그룹**이라, 무거운
채점기를 붙여도 출력 주기가 흔들리지 않습니다.

---

## 설정 파일 두 개

| 파일 | 내용 |
|---|---|
| [config/co_driver.yaml](config/co_driver.yaml) | `output` · `evaluation` · `defaults.influence` · `scoring` · `selection` · `postprocess` — 개수가 고정된 기본 설정 |
| [config/co_driver_topics.jsonc](config/co_driver_topics.jsonc) | `inputs[]` · `drives[]` — 계속 늘어나는 토픽 목록과 영향 행렬 |

ROS 파라미터는 yaml 로 들어가고(런타임 `ros2 param set` 가능), `topics_file` 하나가
JSON 경로를 가리킵니다.

```
inputs[]   — 채점기 인스턴스. 원소를 넣으면 늘어남 (params 에 구독할 토픽 이름)
drives[]   — /drive 후보. 원소를 넣으면 늘어남 (+ hold_ms)
influence  — 입력 i 가 드라이브 j 의 점수에 얼마나, 어떤 모양으로 영향을 주는가
```

> **yaml 주의**: 빈 리스트(`pipeline: []`)나 값 없는 키는 ROS 파라미터에 타입이 없어
> 노드가 기동하지 못합니다. 후처리를 전부 끄려면 `postprocess` 블록을 통째로 지우세요.

---

## 점수 = 1층 선형 MLP + softmax

```
        x₁ ─┐   ← 채점기 1이 낸 점수
        x₂ ─┼─▶  φ (응답 곡선: 선형 + 지수)
         ⋮  │            │
        xₙ ─┘            ▼
                 z_j = Σᵢ W[j][i]·φ_ji(xᵢ) + b_j        ← 1층 선형 레이어
                         │
                         ▼
                 p = softmax(z / T)                      ← 드라이브별 확률
```

| MLP 용어 | co_driver 설정 |
|---|---|
| 입력 벡터 `x` | 각 채점기의 출력 (항상 [0,1]) |
| 특징 변환 `φ` | `influence` 의 `exp_mix`/`exp_k`/`in_min`/`in_max`/`invert` |
| 가중치 행렬 `W[j][i]` | `drives[j].influence[입력 i].weight` — **음수 허용**(감점 항) |
| 편향 `b_j` | `drives[j].bias` |
| 온도 `T` | `scoring.temperature` |
| 출력 `p_j` | `/co_driver_node/scores`, status 의 `score` |

- `temperature` ↓ = 승자독식(전환 민감), ↑ = 평탄(둔감).
- **실격 드라이브는 softmax 분모에서 제외**됩니다. 살아남은 것들의 확률 합은 항상 1.
- EMA(`scoring.ema_alpha`)는 **softmax 이전 logit `z`** 에 걸립니다.
- `scoring.missing` — 채점기가 "판단 불가"를 냈을 때. `"mask"`(기본)는 그 항을 빼고
  남은 `Σ|W|` 비율로 `z` 를 되스케일해 스케일을 보존합니다(센서 하나가 죽어도 그
  드라이브만 밀려나지 않음). `"zero"` 는 `φ=0` 으로 그대로 더하는 순수 MLP 의미입니다.

> **`switch_margin` 의 단위**: `combine: softmax` 에서 `score` 는 확률이므로
> `selection.switch_margin` 도 "확률 차이"입니다. 드라이브 N개면 균등이 1/N 이라는
> 점을 기준으로 잡으세요(기본값 0.10).

### 활성(active) — 살아 있는 토픽끼리만 경쟁

`/drive` 후보들은 발행 시점도 Hz 도 제각각이고, 언제든 끊길 수 있습니다. 그래서
**"지금 살아 있는가"(active)** 를 먼저 판정하고, **활성인 것들끼리만** softmax·순위·선택을
돌립니다.

| active 를 끄는 조건 | |
|---|---|
| 메시지를 한 번도 못 받음 | 점수도 못 냄 |
| **`hold_ms` 초과 (stale)** | 점수는 계속 계산 |
| 명령이 NaN/inf | 점수도 못 냄 |
| 채점기가 `vetoed()` 반환 | 점수는 계속 계산 |
| `influence.veto_below` 미만 / `required` 인데 판단 불가 | 점수는 계속 계산 |
| `enabled: false` | 점수는 계속 계산 |

비활성이어도 **점수(logit)는 계속 계산**됩니다. "왜 안 뽑혔는지"를 봐야 하기 때문입니다.
다만 `score`(확률)는 0 이고 `rank` 는 `null` 이며, 활성 드라이브들의 확률 합만 1 이 됩니다.

### hold — 마지막 명령을 언제까지 붙들고 있을 것인가

드라이브의 시간 설정은 **`hold_ms` 하나뿐**입니다.

```jsonc
{"name": "pp_main", "topic": "/drive_main", "hold_ms": 300}
```

토픽마다 Hz 가 달라도 각자 값을 적으면 됩니다. **그 토픽 주기의 3~5배**가 무난합니다:

| 발행 Hz | 주기 | 권장 `hold_ms` |
|---|---|---|
| 100 Hz | 10 ms | 30 ~ 50 |
| 20 Hz | 50 ms | 150 ~ 250 |
| 5 Hz | 200 ms | 600 ~ 1000 |

짧게 잡을수록 두절을 빨리 잡아내지만 지터·드롭에 예민해집니다. 실측 Hz 는 `status` 의
`hz` 에 찍히니(설정과 무관하게 항상 측정됩니다) 그 값을 보고 정하세요.

> **단위**: 설정의 시간 값은 전부 **ms** 이고 키 이름이 `_ms` 로 끝납니다
> (`hold_ms`, `switch_cooldown_ms`, `duration_ms`, 채점기의 `params.timeout_ms`).
> 주파수는 `_hz` 입니다.

---

## 응답 곡선 φ

채점기는 항상 `x ∈ [0,1]` 을 냅니다. 그 `x` 가 (드라이브, 입력) 쌍마다 지정된 곡선을
거쳐 선형 레이어의 입력이 됩니다.

```
u = invert ? 1-x : x
u = clamp((u - in_min) / (in_max - in_min), 0, 1)      ← 관심 구간만 잘라 확대
e = (e^(k·u) - 1) / (e^k - 1)                          ← 지수 성분 ([0,1] 정규화형)
s = (1 - exp_mix)·u + exp_mix·e                        ← 선형 ↔ 지수 배합
기여도 = weight · s
```

선형항 `u` 와 지수항 `e` 가 둘 다 `[0,1]` 이라 **배합비를 바꿔도 `s` 는 항상 `[0,1]`** 입니다.

| 필드 | 뜻 |
|---|---|
| `weight` | 선형 레이어의 `W[j][i]`. **음수 가능**(감점 항), **0이면 점수 미반영(veto 는 유효)** |
| `exp_mix` | 선형↔지수 배합비. `0`=순수 선형, `1`=순수 지수, `0.7`=지수 7 : 선형 3 |
| `exp_k` | 지수 곡률. **k>0 볼록**("아주 좋아야 점수를 준다"), **k<0 오목**("조금만 나빠도 크게 깎인다") |
| `invert` | 작을수록 좋은 지표를 뒤집음 |
| `in_min` / `in_max` | 관심 구간 재정규화. 예: `in_min: 0.6` = 0.6 이하는 전부 0점 |
| `veto_below` | 성형 후 정규화값이 이 값 미만이면 **그 드라이브 실격** (음수면 끔) |
| `required` | true 면 이 입력이 "판단 불가"일 때 드라이브 실격 |

```
 k = +4  (볼록)          k = 0 (선형)         k = -4  (오목)
 1 ┤            ╭        1 ┤        ╭─        1 ┤    ╭────────
   │          ╭─╯          │      ╭─╯           │  ╭─╯
   │        ╭─╯            │    ╭─╯             │ ╭╯
   │  ╭──╌──╯              │  ╭─╯               │╭╯
 0 ┼──────────────       0 ┼──────────        0 ┼──────────────
   0            1          0        1          0            1
 "아주 좋아야 점수"       비례                "조금만 나빠도 급락"
```

숫자 하나만 쓰면 `weight` 만 지정한 축약형입니다:

```jsonc
"localization": 1.0   ≡   {"weight": 1.0, "exp_mix": 0.0}
```

### 값을 채우는 순서 (뒤로 갈수록 우선)

1. 내장 기본값
2. `defaults.influence` (**yaml**) — 모든 (드라이브 × 입력) 쌍의 출발점
3. `inputs[].influence` (**json**) — 그 입력의 드라이브 공통 기본값
4. `drives[].influence["입력이름"]` (**json**) — 그 한 칸만

덕분에 입력을 새로 넣을 때 `inputs[].influence` 한 줄만 적으면 모든 드라이브에 적용되고,
특정 드라이브만 다르게 반응시키고 싶을 때만 그 드라이브에 덮어씁니다.

---

## 채점기 만들기

**`src/scorers/` 에 .cpp 파일 하나만 추가하면 끝입니다.** 헤더도, 노드 수정도 필요
없습니다. [src/scorers/external_score.cpp](src/scorers/external_score.cpp) 를 복사해서
시작하세요.

```cpp
#include "co_driver/scorer.hpp"

namespace co_driver
{
class ScanClearanceScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string &, const Json & p) override
  {
    // 필요한 토픽을 여기서 직접 구독합니다.
    sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      jstr(p, "topic", "/scan"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        scan_ = m;
      });
    min_clearance_ = jnum(p, "min_clearance", 0.15);   // params 가 그대로 옵니다
    return true;
  }

  ScoreResult score(const Drive & d, const Context & ctx) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!scan_) {return ScoreResult::unavailable("no scan");}
    // d.cmd.drive.speed / steering_angle 를 보고 판단
    if (충돌_임박) {return ScoreResult::vetoed("여유거리 부족");}
    return ScoreResult::ok(점수);   // [0,1], 1이 좋음
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  std::mutex mtx_;
  double min_clearance_{0.15};
};

CO_DRIVER_REGISTER_SCORER(ScanClearanceScorer, "scan_clearance")
}  // namespace co_driver
```

그 다음 두 곳에 한 줄씩:

```cmake
# CMakeLists.txt
set(scorer_sources
  src/scorers/external_score.cpp
  src/scorers/scan_clearance.cpp    # <- 추가
)
```

```jsonc
// config/co_driver_topics.jsonc 의 inputs 에
{
  "name": "clearance", "type": "scan_clearance",
  "params": {"topic": "/scan", "min_clearance": 0.15},
  "influence": {"weight": 2.0, "exp_mix": 0.8, "exp_k": -3.0}
}
```

### 계약

| 반환 | 뜻 |
|---|---|
| `ScoreResult::ok(v)` | `v ∈ [0,1]`, 1이 좋음. 이 값이 응답 곡선의 `x`. 캐시에 저장됩니다 |
| `ScoreResult::pending(why)` | **아직 계산 중**(비동기). 프레임워크가 직전 점수를 대신 씁니다 |
| `ScoreResult::unavailable(why)` | 판단 불가. **가중치째 결합에서 제외** — 데이터 없는 채점기가 점수를 끌어내리지 않습니다 |
| `ScoreResult::vetoed(why)` | 점수와 무관하게 드라이브 실격. hard constraint 전용 |

### 비동기 채점기

`score()` 는 평가 주기(기본 50Hz)마다 호출되므로 오래 걸리면 안 됩니다. 무거운 계산은
**자기 타이머·콜백에서** 하고, `score()` 에서는 결과만 꺼내 주세요.

```cpp
bool configure(rclcpp::Node * node, const std::string &, const Json & p) override
{
  // 반드시 group() 에 넣으세요 — 기본 콜백 그룹은 MutuallyExclusive 라
  // 평가·출력 타이머까지 막습니다.
  timer_ = node->create_wall_timer(200ms, [this]{ compute(); }, group());
  return true;
}

ScoreResult score(const Drive & d, const Context &) override
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!fresh_) {return ScoreResult::pending("계산 중");}   // 직전 점수를 대신 씀
  fresh_ = false;
  return ScoreResult::ok(value_);
}
```

`pending` 이면 프레임워크가 **직전 점수를 대신 쓰고**, 그 점수가 `inputs[].hold_ms` 를
넘기면 자동으로 `unavailable` 이 되어 결합에서 빠집니다. 캐시와 만료는 프레임워크가
처리하므로 채점기는 `pending` 만 내면 됩니다.

```jsonc
{"name": "heavy", "type": "example_async", "hold_ms": 800,
 "params": {"rate_hz": 5.0}, "influence": {"weight": 1.0}}
```

> **콜백 그룹**: 노드는 평가 타이머 / 출력 타이머 / 채점기(Reentrant)를 **서로 다른
> 콜백 그룹**에 둡니다. 그래서 MultiThreadedExecutor 위에서 실제로 병렬로 돕니다.
> 채점기가 만든 구독·타이머를 `group()` 에 넣지 않으면 기본 그룹(MutuallyExclusive)에
> 들어가 출력 주기까지 흔들립니다.
>
> 실측: 40ms 계산을 2Hz 로 도는 채점기를 붙였을 때 —
> 그룹을 안 나누면 `/drive` p99 **48.7ms**, 나누면 **10.1ms**.

동작하는 예시: [src/scorers/example_async.cpp](src/scorers/example_async.cpp)

`Context` 에는 센서가 없습니다. 노드만 알 수 있는 것(`now`, `dt`, 드라이브 목록,
직전 발행 명령, 직전 선택)만 들어 있고, 나머지는 채점기가 스스로 구독합니다.
드라이브 간 상대 비교가 필요하면 `prepare()` 를 오버라이드해 `ctx.drives` 를 훑으세요.

### Python 으로 실험하기 (C++ 빌드 불필요)

`std_msgs/Float64MultiArray` 로 점수를 쏘면 기본 제공되는 `external_score` 채점기가
받아 줍니다. 동작하는 예제:

```bash
ros2 run co_driver example_python_scorer.py
```

배포 설정에 `"external"` 이라는 이름으로 이미 들어 있으므로, 이 노드를 띄우기만 하면
반영됩니다.

---

## 기본 제공 채점기: `external_score`

외부 노드가 쏘는 점수를 받는 채점기 하나만 들어 있습니다(스켈레톤 겸용).
`std_msgs/Float64MultiArray` 로 두 계약을 지원합니다.

### mode: `scalar` — 차량 전체 점수

`localization_pf` 의 `/localization_confidence` 가 이 형태입니다
(`data = [stamp_sec, stamp_nanosec, confidence]`):

```jsonc
{
  "name": "localization", "type": "external_score",
  "params": {
    "topic": "/localization_confidence",
    "mode": "scalar",
    "index": 2,                 // confidence 가 들어있는 칸
    "stamp_indices": [0, 1],    // std_msgs 엔 header 가 없어 앞 두 칸이 stamp
    "timeout_ms": 500,
    "input_min": 0.0, "input_max": 1.0,
    "on_missing": "unavailable" // unavailable | value | veto
  },
  "influence": {"weight": 1.0, "exp_mix": 0.7, "exp_k": 2.5, "in_min": 0.6}
}
```

### mode: `per_candidate` — 드라이브별 점수

짝짓기는 **`layout.dim[i].label` 이 드라이브 이름과 같은 칸**이 1순위, 없으면
`params.order` 순서로 `data[i]`. 값이 `NaN` 이면 그 드라이브만 "판단 불가"가 됩니다.

`params.input_min/input_max/invert` 는 **원 단위 → [0,1]** 변환이고, `influence` 의
`in_min/in_max/invert` 는 그 뒤의 **응답 곡선 성형**입니다.

---

## 선택과 후처리 (yaml)

```yaml
scoring:
  combine: "softmax"        # softmax | weighted_sum
  temperature: 1.0
  missing: "mask"           # mask | zero
  ema_alpha: 0.4
  min_valid_score: 0.0

selection:
  switch_margin: 0.10       # 확률 차이
  switch_cooldown_ms: 500   # 전환 직후 이 시간 동안은 다시 전환하지 않음
  fallback: "pp_main"       # 전원 실격일 때 마지막 시도 (이것도 유효해야 씀)
```

- 현재 드라이브가 **살아 있으면** `switch_margin` + `switch_cooldown_ms` 를 둘 다 만족해야 전환.
- 현재 드라이브가 **실격되면** 히스테리시스 없이 즉시 전환.
- 전부 실격이면 `fallback` → 그것도 안 되면 "선택 없음" → `timeout_stop` 이 감속 정지.
- 쓸 드라이브가 없어도 `/drive` 발행은 계속됩니다(그래야 감속 명령이 차에 닿습니다).

`postprocess.pipeline` **목록 순서가 곧 적용 순서**입니다.

| type | 하는 일 |
|---|---|
| `timeout_stop` | 쓸 드라이브가 없을 때 `decel` 로 감속해 정지 |
| `switch_blend` | A→B 전환 순간의 점프를 `duration_ms` 동안 녹임 (`linear`/`smooth`/`ema`) |
| `rate_limit` | 조향 각속도[deg/s], 가/감속[m/s²] 제한 |
| `speed_scale` | 최종 속도 배율. 런타임 `ros2 param set` 가능 |
| `deadband` | 미세 명령을 0으로 눌러 서보 잔떨림 억제 |
| `clamp` | 절대 한계. **마지막에 두어** 앞 단계가 한계를 넘기지 못하게 함 |

```bash
ros2 param set /co_driver_node postprocess.speed_scale.scale 0.5
```

새 단계는 [src/co_driver_node.cpp](src/co_driver_node.cpp) 의 `PostProcess` 에
enum + 파싱 + apply 각각 한 case 씩 추가하면 됩니다.

---

## 튜닝 루프 (핫리로드)

계수만 바꿨다면 재시작 없이 다시 읽습니다.

```bash
# JSON 을 편집했거나 —
ros2 param set /co_driver_node scoring.temperature 0.2   # yaml 쪽 값을 바꿨다면
ros2 service call /co_driver_node/reload std_srvs/srv/Trigger
```

반영: **influence 행렬, scoring, selection, postprocess, drives 의 timeout/bias/enabled**.
반영 안 됨: 토픽 이름, 주기, inputs/drives 목록 자체 — 구독을 다시 만들어야 하므로
서비스가 거부하고 이유를 알려 줍니다.

---

## 진단 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/co_driver_node/selected` | `std_msgs/String` | **실제로 쓰는 드라이브** — `/drive` 로 나가는 명령의 주인 |
| `/co_driver_node/runner_up` | `std_msgs/String` | **합산 점수 2등** |
| `/co_driver_node/scores` | `std_msgs/Float64MultiArray` | 드라이브별 softmax 확률. `layout.dim[i].label` 에 이름 |
| `/co_driver_node/status` | `std_msgs/String` | JSON 한 줄. rank / logit / 입력별 기여도까지 |

`runner_up` 은 **순수 점수 순위 2위**입니다. 선택 로직과 무관하므로, 히스테리시스로
선택이 점수 1등이 아닐 때는 `selected` 와 같은 값이 나올 수 있습니다(정상). 유효
드라이브가 하나뿐이면 빈 문자열입니다.

점수 3등 이하가 필요하면 `status` 의 `rank` 필드나 `/scores` 를 정렬해 쓰세요.

```jsonc
{"name": "pp_left", "rank": 2, "score": 0.424, "logit": 1.9143, "raw": 1.9143, "bias": 0.0,
 "active": true, "valid": true,
 "age_ms": 26.5,    // 마지막 수신으로부터 경과
 "hold_ms": 300.0,  // 설정된 유효 시간
 "hz": 20.0,        // 실측 수신 주파수 (설정과 무관하게 항상 측정)
 "inputs": {
   "localization": {"x": 0.9, "s": 0.640885, "w": 1.2, "c": 0.769062}
   //                 x = 채점기 원점수, s = φ 성형 후, w = W[j][i], c = logit 기여분 W·φ
 }}
```

두절된 드라이브는 이렇게 보입니다 — 점수는 남아 있고 순위에서만 빠집니다:

```
  pp_main   hz= 99.0  hold=300ms  age=    5.5ms  active=True  rank=2  p=0.3989 logit=1.1798
  pp_left   hz= 20.0  hold=300ms  age= 1546.5ms  active=False rank=None p=0.0000 logit=1.9143
                                                     stale 1546ms (hold 300ms)
  pp_right  hz=  5.0  hold=300ms  age=  103.0ms  active=True  rank=1  p=0.6011 logit=1.5898
```

`rank` 는 **점수 순위**(유효 드라이브만, 실격이면 `null`)이고 `selected` 는 **실제 선택**
입니다. 히스테리시스(`switch_margin` / `switch_cooldown_ms`) 때문에 둘이 어긋날 수 있고,
그 차이가 보이는 것이 목적입니다.

```
    /selected = 'pp_left'   /runner_up = 'pp_left'
      rank 1  pp_right  p=0.4313     <- 점수는 이쪽이 1등인데
      rank 2  pp_left   p=0.4240     <- switch_margin 미달이라 아직 이쪽을 씀 (= /runner_up)
      rank 3  pp_main   p=0.1447
```

`inputs` 의 `null` 은 판단 불가, `"veto"` 는 실격. 어떤 것이 드라이브를 죽였는지는
`reject` 필드에 사유가 그대로 찍힙니다.

---

## 빌드 · 실행

```bash
sudo apt install ros-jazzy-ackermann-msgs nlohmann-json3-dev

cd ~/Desktop/co-driver
colcon build --packages-select co_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch co_driver co_driver.launch.py
# 다른 설정으로
ros2 launch co_driver co_driver.launch.py config:=my.yaml topics:=my_topics.jsonc
# launch 없이
ros2 run co_driver co_driver_node --ros-args \
    --params-file my.yaml -p topics_file:=my_topics.jsonc
```

### PPcontroller 쪽 설정

각 인스턴스가 서로 다른 토픽으로 내보내게 `drive_topic` 을 바꿉니다
(`PPcontroller/config/config.yaml`):

```yaml
pure_pursuit:
  ros__parameters:
    drive_topic: "/drive_main"     # 인스턴스마다 /drive_left, /drive_right ...
```

`co_driver_topics.jsonc` 의 `drives[].topic` 이 이 이름들과 맞기만 하면 됩니다.

---

## 안전 관련 메모

- 쓸 드라이브가 하나도 없어도 `/drive` 발행은 멈추지 않습니다. 그래야 `timeout_stop` 의
  감속이 차에 전달됩니다(발행을 멈추면 mux 자체 timeout 0.2s 로 급정지).
- `clamp.min_speed: 0.0` 이 기본이라 후진 명령은 나가지 않습니다.
- 조이스틱 e-stop 은 co_driver 를 거치지 않고 mux 에서 처리됩니다. 출력 토픽을
  `/ackermann_drive` 로 바꿔 mux 를 우회하면 **그 보호가 사라집니다**.
- `exp_k` 를 크게 음수로 주면 점수가 거의 계단처럼 떨어집니다. 안전 지표에는 유용하지만
  전환이 잦아질 수 있으니 `switch_margin` / `switch_cooldown_ms` 와 같이 보세요.
