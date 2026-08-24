# 런치 스타트 토픽 사용법

`pure_pursuit` 노드가 새로 쓰는 토픽 3개입니다. 전부 **구독**이고, 노드 이름은
`pure_pursuit` 기준입니다.

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/launch_speed_hold` | `std_msgs/Float64MultiArray` | n초 동안 m m/s 유지 후 램프업 |
| `/launch_start_reset` | `std_msgs/Bool` | 런치 스타트 발동 / 취소 |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | 휠 속도 입력 (VESC가 발행) |

속도 단위는 전부 **명령 속도 도메인**입니다. 경로 speed·drive 명령과 같은
단위이고, 실제 차속으로는 약 1/2.6 수준입니다 (`wheel_speed_scale` 참고).

---

## 1. `/launch_speed_hold` — 고정 속도 유지 후 램프업

`data = [speed, duration]`. 받는 즉시 `speed`로 고정하고, `duration`초가 지나면
그 속도에서 곧바로 램프업으로 넘어갑니다.

```bash
# 1.5초 동안 1.2 m/s 유지 -> 이후 목표 속도까지 램프업
ros2 topic pub --once /launch_speed_hold std_msgs/msg/Float64MultiArray "{data: [1.2, 1.5]}"

# duration 생략 -> speed_hold_default_duration(기본 1.0초)
ros2 topic pub --once /launch_speed_hold std_msgs/msg/Float64MultiArray "{data: [1.2]}"

# duration 0 -> 유지 없이 그 속도에서 바로 램프업
ros2 topic pub --once /launch_speed_hold std_msgs/msg/Float64MultiArray "{data: [1.2, 0.0]}"
```

동작 예 (1.2 m/s를 1.5초 유지, 목표 4.25 m/s, 가속 3.0 m/s²):

```
t=0.0s  4.25   요청 직전
t=0.2s  1.20   즉시 고정
t=1.4s  1.20   유지 중
t=1.5s  1.27   유지 종료 -> 유지 속도에서 램프 시작
t=2.3s  3.54   3.0 m/s^2 로 상승
t=2.5s  4.25   목표에 붙어 해제, 기존 동작 복귀
```

**거부되는 요청** (로그에 이유가 남고 기존 동작 유지):

| 입력 | 결과 |
|---|---|
| `[]` | 거부 |
| `[-1.0, 1.0]` | 거부 (음수 속도) |
| `[.nan, 1.0]` | 거부 (NaN/inf) |
| `[1.0, 99.0]` | `speed_hold_max_duration`(기본 10초)로 잘림 |

**참고**

- 램프 시작점은 측정 휠 속도가 아니라 **유지하던 속도**입니다. 명령이 튀지
  않고, VESC가 없어도 이 경로는 동작합니다.
- 램프업 진입에 `launch_start_engage_diff` 조건을 걸지 않습니다. 유지 속도가
  이미 목표에 붙어 있으면 즉시 해제됩니다.
- `launch_start_enabled`가 false면 유지만 하고 램프업 없이 복귀합니다.

---

## 2. `/launch_start_reset` — 런치 스타트 발동 / 취소

```bash
ros2 topic pub --once /launch_start_reset std_msgs/msg/Bool "{data: true}"   # 발동
ros2 topic pub --once /launch_start_reset std_msgs/msg/Bool "{data: false}"  # 취소
```

`true`면 **현재 휠 속도**에서 램프를 다시 시작합니다. 단, `/sensors/core`가
살아 있어야 하고 `|목표 속도 - 휠 속도| >= launch_start_engage_diff`(기본 1.0)
여야 발동합니다. 이미 속도가 붙어 있으면 로그에 `skipped`가 남습니다.

`false`는 진행 중인 램프와 speed hold를 모두 취소합니다.

RF 스위치로도 같은 동작을 합니다 — `/rf.data[5]`가 1800 **이하였다가 초과로
올라가는 순간** 한 번 발동합니다. 계속 올려두면 재발동하지 않습니다.

---

## 3. `/sensors/core` — 휠 속도 입력

VESC 드라이버가 발행합니다. 노드는 전기 RPM을 vesc_to_odom과 같은 식으로
변환해서 씁니다.

```
speed[m/s] = (state.speed[erpm] - speed_to_erpm_offset) / speed_to_erpm_gain
```

이 값이 없으면 `/launch_start_reset`과 RF 트리거는 **무시됩니다**
(`/launch_speed_hold`는 영향 없음). 5초마다 경고가 뜹니다:

```
[WARN] Launch start is armed but /sensors/core has never published: a trigger will be ignored
```

---

## 우선순위

가장 최근 입력이 이깁니다.

```
speed hold  >  런치 램프  >  기존 블렌딩 제어
```

- hold 중에 새 hold가 오면 교체
- hold 중에 RF 트리거나 `/launch_start_reset true`가 오면 hold 취소 후 일반 런치
- 어느 경우든 `max_speed_limit_percentage` 상한은 그대로 적용

---

## 로그로 확인하기

| 로그 | 의미 |
|---|---|
| `Speed hold: 1.20 m/s for 1.50s, then ramp` | hold 시작 |
| `Speed hold finished at 1.20 m/s` | 유지 종료 |
| `Launch ramp starting from 1.20 m/s (speed hold)` | 램프 시작 |
| `Launch ramp: 1.95 -> target 4.25 m/s (...)` | 램프 진행 (250ms 주기) |
| `Launch start released: ramp 3.75, target 4.25` | 해제, 기존 동작 복귀 |
| `Launch start skipped: \|target X - wheel Y\| < 1.00` | 이미 속도가 붙어 미발동 |
| `Launch start channel index 5 is unavailable` | `/rf` 배열이 짧음 |
| `armed but /sensors/core has never published` | VESC 미수신 |

관련 파라미터는 [USAGE_GUIDE.md](USAGE_GUIDE.md)의 런치 스타트 절을 보세요.
