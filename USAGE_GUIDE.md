# Usage Guide

To use this node on the physical car, run

```bash
ros2 launch pure_pursuit pure_pursuit.launch.py
```

This will use the parameters from the config file `config/config.yaml`

If you want to test it out in simulation, which uses different topic names, run

```bash
ros2 launch pure_pursuit sim_pure_pursuit_launch.py
```

### Trying different parameters without rebuilding

Setting the gain
```bash
ros2 param set pure_pursuit K_p 0.1
```

Setting the velocity profile

```bash
ros2 param set pure_pursuit velocity_percentage 0.7
```

Reading a value back, or listing everything that is tunable

```bash
ros2 param get pure_pursuit K_p
ros2 param list pure_pursuit
ros2 param dump pure_pursuit          # 현재 값 전체를 yaml로 저장
```

Any node can do the same over `/pure_pursuit/get_parameters` and
`/pure_pursuit/set_parameters` (묶음 단위로 전부 반영/전부 거부하려면
`/pure_pursuit/set_parameters_atomically`), and can watch changes without
polling by subscribing to `/parameter_events`.

### What can be changed at runtime

`kRuntimeParameterNames` (in `src/pure_pursuit.cpp`) 에 있는 파라미터는 모두
런타임에 조회/변경할 수 있고, 다음 제어 주기부터 바로 반영됩니다.

- steering: `K_p`, `K_i`, `K_d`, `heading_error_gain`, `steering_limit`,
  `steering_expo_gain`, `steering_expo_curve`, `steer_latest_blend`,
  `steer_large_change_blend`, `steer_blend_change_threshold_deg`,
  `steer_speed_filter_*`, `steer_reduction_*`, `max_allowed_steer_drop_deg`
- lookahead / tracking: `min_lookahead`, `max_lookahead`, `lookahead_ratio`,
  `min_searching_idx_offset`, `max_searching_idx_offset`,
  `speed_profile_distance_offset`
- speed: `velocity_percentage`, `max_speed_limit_percentage`,
  `speed_latest_blend`, `speed_reduction_*`, `slow_with_obs`, `obs_slow_th`,
  `obs_slow_percentage`
- output / RF: `drive_topic`, `drive_test_topic`, `test_mode`,
  `drive_output_rate_hz`, `publish_drive_on_odom`, `visualization_rate_hz`,
  `rf_*`

시작할 때만 반영되는 값: 토픽/프레임 이름 (`odom_topic`, `path_topic`,
`global_refFrame`, `car_refFrame`, `rf_topic`, `rviz_*_topic`) 과
`path_is_circular` — 구독 생성이나 경로 재계산이 필요해서 런타임 변경 대상이
아닙니다.

값은 반영 전에 검사합니다. NaN/inf는 모든 숫자 파라미터에서 거부되고
(한 번 들어가면 적분항까지 오염되어 재시작 전에는 복구되지 않습니다),
`kRuntimeParameterBounds` 에 등록된 항목은 범위를 벗어나면 거부됩니다.
거부되면 기존 값이 그대로 유지되고, 이유가 로그와 서비스 응답에 남습니다.

```bash
$ ros2 param set pure_pursuit K_p -1.0
Setting parameter failed: K_p must be in [0, inf] (got -1)
```

RF 스위치(`rf_enable_channel`)가 올라가 있는 동안에는 `/rf` 가 매 메시지마다
`velocity_percentage` 와 `max_speed_limit_percentage` 를 덮어씁니다. 이 두 값을
외부에서 제어하려면 스위치를 내려 두세요.


## Launch start

정지 상태에서 트리거가 들어오면, 경로 목표 속도로 바로 뛰지 않고 설정한
가속도로만 올라가는 래치입니다. 목표 속도는 차량 위치에 따라 계속 바뀌고,
램프가 그 목표에 `launch_start_release_diff` 이내로 붙으면 래치가 풀려
기존 블렌딩 동작으로 돌아갑니다. 한 번 풀리면 새 트리거가 오기 전까지
다시 걸리지 않습니다.

### 휠 속도

`vesc_state_topic`(기본 `sensors/core`, `vesc_msgs/VescStateStamped`)의
전기 RPM을 vesc_to_odom과 같은 식으로 변환해 씁니다.

```
speed[m/s] = (state.speed[erpm] - speed_to_erpm_offset) / speed_to_erpm_gain
```

`wheel_speed_deadband`(0.05) 이하는 0으로 봅니다. 변환된 값은 명령 속도
(path speed) 도메인보다 약 2.6배 작게 나오므로 `wheel_speed_scale`을 곱해
맞춘 뒤 목표 속도와 비교합니다. **아래 diff/accel 값은 전부 이 변환 뒤의
명령 속도 기준**이라 실제 차속으로는 약 1/2.6 수준입니다.
`wheel_speed_timeout`(0.5초)보다 오래된 값만 있으면 래치를 걸지 않습니다.

### 트리거

1. **RF 채널 상승 엣지** — `launch_start_channel`(기본 6) 값이
   `launch_start_channel_threshold`(1800) **이하였다가 초과로 올라가는 순간**
   한 번만 발동합니다. 계속 위에 머물러 있으면 재발동하지 않고, 노드가 뜬 뒤
   첫 메시지는 직전 값이 없어 엣지로 보지 않습니다. RF enable 스위치와는
   무관하게 동작합니다.
2. **토픽** — `launch_start_reset_topic`(기본 `/launch_start_reset`,
   `std_msgs/Bool`)에 `true`가 오면 현재 휠 속도 기준으로 래치를 다시 걸고,
   `false`면 해제합니다.

트리거가 와도 `|목표 속도 - 휠 속도| < launch_start_engage_diff`(1.0)이면
이미 속도가 붙은 것으로 보고 래치를 걸지 않습니다(로그에 `skipped`).

```bash
# 수동 발동 / 해제
ros2 topic pub --once /launch_start_reset std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /launch_start_reset std_msgs/msg/Bool "{data: false}"

# 통째로 끄기
ros2 param set pure_pursuit launch_start_enabled false
```

래치가 걸린 동안에도 `max_speed_limit_percentage` 상한은 그대로 적용되고,
RViz의 `/pp_runtime_params` 마커에 휠 속도와 `LAUNCH <ramp> -> <target>` 이
표시됩니다.
