# co_driver

Arbitrates multiple `/drive` candidates by score, post-processes the winner, publishes it to `/drive`.

## Layout

    co_driver/               the node
    obstacle_context_msgs/   cluster message definitions, vendored
    vesc_msgs/               VESC telemetry message definitions, vendored

The node links both message packages' typesupport libraries, so their source
definitions live in this repository rather than depending on whichever overlay
happened to be sourced at build time. `obstacle_context_msgs` matches the
obstacle detector. `vesc_msgs` matches both the package used by red_damvi and
`controller/src/vesc_msgs`; its upstream BSD license is retained alongside it.
Keep every field, constant, type and ordering interface-identical to those
publishers, or ROS type hashes no longer match and the topics will not connect.

## Build

```bash
sudo apt install ros-jazzy-ackermann-msgs nlohmann-json3-dev
cd ~/Desktop/co-driver
colcon build --packages-up-to co_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--packages-up-to` is intentional: it discovers and builds the vendored
`vesc_msgs` and `obstacle_context_msgs` source dependencies before `co_driver`.
`--packages-select co_driver` alone assumes those packages are already installed
in an external underlay and is therefore not a standalone build.

## Run

```bash
# default configuration
ros2 launch co_driver co_driver.launch.py

# PPcontroller (/drive_main) vs gap_follow (/drive_gf), arbitrated by localization health
ros2 launch co_driver red_damvi.launch.py

# with a rosbag replay or any BEST_EFFORT /scan publisher
ros2 launch co_driver red_damvi.launch.py scan_bridge:=true
```

## Configure

| File | Edit it to |
|---|---|
| `config/co_driver.yaml` | change output/scoring/selection/post-processing |
| `config/co_driver_topics.jsonc` | add or remove input topics, drive candidates, weights |

red_damvi splits its configuration by responsibility:

| File | Holds |
|---|---|
| `co_driver_red_damvi.yaml` | output / scoring / selection / post-processing |
| `co_driver_red_damvi_topics.jsonc` | wiring plus the base arbitration layer (weights, bias, curves, vetoes, hold_ms) |
| `localization_scoring.jsonc` | localization scorer parameters and related drive overrides |
| `obstacle_scoring.jsonc` | obstacle/clearance scorer parameters plus the obstacle safety-veto overlay, including `gap_loc` |
| `return_assist.jsonc` | hand-back speed hold, ramp and gain-assist settings |

The tuning files are deep-merged over the topics file in the order listed by the
`tuning_file` parameter. Most arbitration defaults remain in the topics file;
an override that must track a scorer parameter stays with that scorer instead.
Reload coefficients without restarting:

```bash
ros2 service call /co_driver_node/reload std_srvs/srv/Trigger
```

This reload covers the YAML/topics tuning layers, not `return_assist.jsonc`,
which is read only at startup. It also resets post-processing immediately and
does not recompute an already-published speed-hold request. Reload or change the
runtime speed scale between hold sessions; doing so during one invalidates its
conditional steady-time estimate and is reported in status/logs.

## Monitor

```bash
ros2 run co_driver drive_monitor.py   # who is driving, switches, oscillation verdict
ros2 topic echo /co_driver_node/status   # full per-drive scores as JSON
```

## Testing

Stand-ins that fabricate vehicle input - dummy drive commands, synthetic
confidence - are not part of this package, so nothing that invents controller
or sensor data can be installed onto a car by accident.

## Add a scorer

Copy `src/scorers/external_score.cpp`, add the file to `scorer_sources` in `CMakeLists.txt`, add an entry to the topics JSON `inputs`.
