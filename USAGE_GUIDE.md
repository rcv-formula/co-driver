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
