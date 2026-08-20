# co_driver

Arbitrates multiple `/drive` candidates by score, post-processes the winner, publishes it to `/drive`.

## Layout

    co_driver/               the node
    obstacle_context_msgs/   cluster message definitions, vendored

`obstacle_context_msgs` is owned by the obstacle detector and copied here so
this repository builds and runs on its own. The node links its typesupport
library, so a build made where the package exists will not start where it does
not - it fails in the dynamic linker, before `main()`. Keep the `.msg` files
byte-identical to the detector's, or publisher and subscriber stop matching
with no error reported.

## Build

```bash
sudo apt install ros-jazzy-ackermann-msgs nlohmann-json3-dev
cd ~/Desktop/co-driver
colcon build --packages-select co_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

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

red_damvi splits its configuration three ways:

| File | Holds |
|---|---|
| `co_driver_red_damvi.yaml` | output / scoring / selection / post-processing |
| `co_driver_red_damvi_topics.jsonc` | co_driver's own: wiring plus the arbitration layer (weights, bias, curves, vetoes, hold_ms) |
| `localization_scoring.jsonc` | the scorers' own: how confidence becomes a score (timeouts, missing-topic policy, recovery conditions) |

The scoring file is deep-merged over the topics file (`tuning_file` parameter), so
each side can be edited without touching the other.
Reload coefficients without restarting:

```bash
ros2 service call /co_driver_node/reload std_srvs/srv/Trigger
```

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
