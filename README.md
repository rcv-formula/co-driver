# co_driver

An arbitration node that **scores** several `/drive` candidates, picks one,
post-processes it, and sends the result to the VESC stack.

```
PPcontroller #1 --> /drive_main  ┐
PPcontroller #2 --> /drive_left  ├--> co_driver --> /drive --> ackermann_mux --> ackermann_to_vesc --> VESC
PPcontroller #3 --> /drive_right ┘        ^
                                          │
                             each scorer subscribes to whatever it needs
                             (/localization_confidence, /scan, /map, ...)
```

The output goes to the **navigation input** of `ackermann_mux` (the `drive` entry in
`f1tenth_stack/config/mux.yaml`). Keeping it there preserves the joystick override
(priority 100) and the mux's own timeout safety net. To bypass the mux, point
`output.drive_topic` at `/ackermann_drive`.

---

## Structure

Four files, with no overlap in responsibility.

| File | Does |
|---|---|
| [src/co_driver_node.cpp](src/co_driver_node.cpp) | Main node - subscribes `/drive_*`, selects, post-processes, publishes, hot-reloads |
| [src/config.cpp](src/config.cpp) | Configuration - yaml (ROS parameters) + the topic list JSON |
| [src/compute.cpp](src/compute.cpp) | **Scoring only** - response curve phi, linear layer, softmax |
| [src/scorers/](src/scorers/) | **Scorers** - one .cpp each, each subscribing to its own topics; sync or async |

```
/drive_* --> scorers --> scoring --> selection --> post-process --> /drive
             (scorers)  (compute)     (node)         (node)         100 Hz
                 ^
        each scorer subscribes for itself:
        scan / map / imu / external score topics
```

**The node subscribes to no sensors.** If you want to judge on `/scan`, you add a file
to `src/scorers/` rather than touching the node. Judgement logic can grow without the
node, the config loader or the scoring code changing at all.

Evaluation (allowed to be slow) and output (fixed 100 Hz) run on **separate timers in
separate callback groups**, so a heavy scorer does not disturb the output rate.

---

## Two configuration files

| File | Contents |
|---|---|
| [config/co_driver.yaml](config/co_driver.yaml) | `output`, `evaluation`, `defaults.influence`, `scoring`, `selection`, `postprocess` - the settings with a fixed shape |
| [config/co_driver_topics.jsonc](config/co_driver_topics.jsonc) | `inputs[]`, `drives[]` - the lists that keep growing, plus the influence matrix |

ROS parameters come from the yaml (so `ros2 param set` works at runtime), and a single
`topics_file` parameter points at the JSON.

```
inputs[]   scorer instances; add an element to add one (params carries topic names)
drives[]   /drive candidates; add an element to add one (plus hold_ms)
influence  how much, and in what shape, input i affects the score of drive j
```

> **yaml caveat**: an empty list (`pipeline: []`) or a valueless key has no type as a
> ROS parameter and the node will not start. To turn off post-processing, delete the
> whole `postprocess` block.

---

## Scoring = one linear MLP layer + softmax

```
        x1 --┐   <- score from scorer 1
        x2 --┼-->  phi (response curve: linear + exponential)
         :   │            │
        xn --┘            v
                 z_j = sum_i W[j][i] * phi_ji(x_i) + b_j     <- one linear layer
                         │
                         v
                 p = softmax(z / T)                          <- probability per drive
```

| MLP term | co_driver setting |
|---|---|
| input vector `x` | each scorer's output (always in [0,1]) |
| feature transform `phi` | `influence`'s `exp_mix` / `exp_k` / `in_min` / `in_max` / `invert` |
| weight matrix `W[j][i]` | `drives[j].influence[input i].weight` - **negatives allowed** (penalty term) |
| bias `b_j` | `drives[j].bias` |
| temperature `T` | `scoring.temperature` |
| output `p_j` | `/co_driver_node/scores`, the `score` field in status |

- `temperature` down = winner-take-all (switches readily), up = flat (insensitive).
- **Disqualified drives are excluded from the softmax denominator.** The survivors'
  probabilities always sum to 1.
- The EMA (`scoring.ema_alpha`) is applied to the logit `z`, **before** the softmax.
- `scoring.missing` - what to do when a scorer reports "cannot judge". `"mask"` (default)
  drops the term and rescales `z` by the remaining `sum|W|` ratio, preserving the scale
  so one dead sensor does not push that drive down on its own. `"zero"` adds `phi = 0`,
  which is the literal MLP reading.

> **Units of `switch_margin`**: with `combine: softmax` the `score` is a probability, so
> `selection.switch_margin` is a difference of probabilities. With N drives, uniform is
> 1/N - calibrate against that (default 0.10).

### active - only live topics compete

`/drive` candidates publish at different times and different rates, and any of them can
stop. So co_driver first decides **"is this alive right now"** (active), and runs the
softmax, the ranking and the selection **over the active ones only**.

| What clears active | |
|---|---|
| never received a message | no score either |
| **older than `hold_ms` (stale)** | score still computed |
| command is NaN/inf | no score either |
| a scorer returned `vetoed()` | score still computed |
| below `influence.veto_below`, or a `required` input cannot judge | score still computed |
| `enabled: false` | score still computed |

Even when inactive, **the logit keeps being computed** - you need to see *why* something
was not picked. Its `score` (probability) is 0 and its `rank` is `null`, and only the
active drives' probabilities sum to 1.

### hold - how long to keep trusting the last command

A drive has exactly **one** time setting, `hold_ms`.

```jsonc
{"name": "pp_main", "topic": "/drive_main", "hold_ms": 300}
```

Topics can run at different rates; give each its own value. **Three to five times its
period** is a reasonable starting point:

| publish rate | period | suggested `hold_ms` |
|---|---|---|
| 100 Hz | 10 ms | 30 - 50 |
| 20 Hz | 50 ms | 150 - 250 |
| 5 Hz | 200 ms | 600 - 1000 |

Shorter catches a dropout sooner but is more sensitive to jitter and drops. The measured
rate is reported as `hz` in `status` (always measured, independent of the config) - use
that to decide.

> **Units**: every time value in the configuration is in **ms** and its key ends in `_ms`
> (`hold_ms`, `switch_cooldown_ms`, `duration_ms`, a scorer's `params.timeout_ms`).
> Rates end in `_hz`.

---

## The response curve phi

A scorer always emits `x` in [0,1]. That `x` passes through a curve specified per
(drive, input) pair before it reaches the linear layer.

```
u = invert ? 1-x : x
u = clamp((u - in_min) / (in_max - in_min), 0, 1)      <- crop and stretch the range of interest
e = (e^(k*u) - 1) / (e^k - 1)                          <- exponential component, normalised to [0,1]
s = (1 - exp_mix)*u + exp_mix*e                        <- blend linear and exponential
contribution = weight * s
```

Since the linear term `u` and the exponential term `e` are both in [0,1], **`s` stays in
[0,1] whatever the blend**.

| Field | Meaning |
|---|---|
| `weight` | `W[j][i]` of the linear layer. **May be negative** (penalty term). **0 means it does not affect the score, but veto still applies** |
| `exp_mix` | linear/exponential blend. `0` = pure linear, `1` = pure exponential, `0.7` = 7 parts exponential to 3 linear |
| `exp_k` | exponential curvature. **k > 0 convex** ("only a very good value earns points"), **k < 0 concave** ("a small drop already costs a lot") |
| `invert` | flips a metric where smaller is better |
| `in_min` / `in_max` | renormalise the range of interest. e.g. `in_min: 0.6` scores everything at or below 0.6 as zero |
| `veto_below` | if the shaped value is below this, **that drive is disqualified** (negative disables) |
| `required` | if true, the drive is disqualified whenever this input cannot judge |

```
 k = +4  (convex)        k = 0 (linear)       k = -4  (concave)
 1 ┤            ╭        1 ┤        ╭─        1 ┤    ╭────────
   │          ╭─╯          │      ╭─╯           │  ╭─╯
   │        ╭─╯            │    ╭─╯             │ ╭╯
   │  ╭──╌──╯              │  ╭─╯               │╭╯
 0 ┼──────────────       0 ┼──────────        0 ┼──────────────
   0            1          0        1          0            1
 "must be very good"      proportional        "punishes small drops"
```

A bare number is shorthand for `weight` alone:

```jsonc
"localization": 1.0   ==   {"weight": 1.0, "exp_mix": 0.0}
```

### Order in which values are filled (later wins)

1. built-in defaults
2. `defaults.influence` (**yaml**) - the starting point for every (drive x input) pair
3. `inputs[].influence` (**json**) - that input's default across all drives
4. `drives[].influence["input name"]` (**json**) - that one cell

So adding an input needs one line in `inputs[].influence` to apply everywhere, and you
only override per drive when you want that drive to react differently.

---

## Writing a scorer

**Add one .cpp to `src/scorers/` and you are done.** No header, no changes to the node.
Copy [src/scorers/external_score.cpp](src/scorers/external_score.cpp) to start.

```cpp
#include "co_driver/scorer.hpp"

namespace co_driver
{
class ScanClearanceScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string &, const Json & p) override
  {
    // Subscribe to whatever you need, right here.
    sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      jstr(p, "topic", "/scan"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mtx_);
        scan_ = m;
      });
    min_clearance_ = jnum(p, "min_clearance", 0.15);   // params arrives verbatim
    return true;
  }

  ScoreResult score(const Drive & d, const Context & ctx) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!scan_) {return ScoreResult::unavailable("no scan");}
    // judge using d.cmd.drive.speed / steering_angle
    if (collision_imminent) {return ScoreResult::vetoed("not enough clearance");}
    return ScoreResult::ok(value);   // [0,1], 1 is good
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

Then one line in each of two places:

```cmake
# CMakeLists.txt
set(scorer_sources
  src/scorers/external_score.cpp
  src/scorers/scan_clearance.cpp    # <- added
)
```

```jsonc
// in the inputs list of config/co_driver_topics.jsonc
{
  "name": "clearance", "type": "scan_clearance",
  "params": {"topic": "/scan", "min_clearance": 0.15},
  "influence": {"weight": 2.0, "exp_mix": 0.8, "exp_k": -3.0}
}
```

### The contract

| Return | Meaning |
|---|---|
| `ScoreResult::ok(v)` | `v` in [0,1], 1 is good. This is the `x` of the response curve. Cached |
| `ScoreResult::pending(why)` | **still computing** (async). The framework substitutes the previous score |
| `ScoreResult::unavailable(why)` | cannot judge. **Excluded from the combination, weight and all** - a scorer with no data does not drag the score down |
| `ScoreResult::vetoed(why)` | disqualifies the drive regardless of score. For hard constraints only |

### Async scorers

`score()` is called every evaluation tick (50 Hz by default), so it must not be slow. Do
heavy work **in your own timer or callback** and have `score()` only hand back the result.

```cpp
bool configure(rclcpp::Node * node, const std::string &, const Json & p) override
{
  // Must go in group(). The default callback group is MutuallyExclusive and would
  // block the evaluation and output timers.
  timer_ = node->create_wall_timer(200ms, [this]{ compute(); }, group());
  return true;
}

ScoreResult score(const Drive & d, const Context &) override
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!fresh_) {return ScoreResult::pending("computing");}   // previous score is used
  fresh_ = false;
  return ScoreResult::ok(value_);
}
```

On `pending` the framework **substitutes the previous score**, and once that score is
older than `inputs[].hold_ms` it automatically becomes `unavailable` and drops out of the
combination. Caching and expiry are the framework's job; a scorer only has to say
`pending`.

```jsonc
{"name": "heavy", "type": "example_async", "hold_ms": 800,
 "params": {"rate_hz": 5.0}, "influence": {"weight": 1.0}}
```

> **Callback groups**: the node puts the evaluation timer, the output timer and the
> scorers (Reentrant) in **separate callback groups**, so they genuinely run in parallel
> on a MultiThreadedExecutor. A subscription or timer a scorer creates without passing
> `group()` lands in the default (MutuallyExclusive) group and will disturb the output
> rate.
>
> Measured: with a scorer doing 40 ms of work at 2 Hz - without separate groups `/drive`
> p99 was **48.7 ms**, with them **10.1 ms**.

Working example: [src/scorers/example_async.cpp](src/scorers/example_async.cpp)

`Context` has no sensors in it. Only what the node alone knows (`now`, `dt`, the drive
list, the last published command, the last selection); everything else a scorer
subscribes to itself. If you need to compare drives against each other, override
`prepare()` and walk `ctx.drives`.

### Experimenting from Python (no C++ build)

Publish scores as `std_msgs/Float64MultiArray` and the built-in `external_score` scorer
picks them up. Working example:

```bash
ros2 run co_driver example_python_scorer.py
```

The shipped configuration already includes it under the name `"external"`, so just
starting the node is enough.

---

## The built-in scorer: `external_score`

The one bundled scorer receives scores published by another node, and doubles as the
skeleton. It supports two contracts over `std_msgs/Float64MultiArray`.

### mode: `scalar` - one score for the whole vehicle

`localization_pf`'s `/localization_confidence` has this shape
(`data = [stamp_sec, stamp_nanosec, confidence]`):

```jsonc
{
  "name": "localization", "type": "external_score",
  "params": {
    "topic": "/localization_confidence",
    "mode": "scalar",
    "index": 2,                 // the slot holding the confidence
    "stamp_indices": [0, 1],    // std_msgs has no header, so the first two slots are the stamp
    "timeout_ms": 500,
    "input_min": 0.0, "input_max": 1.0,
    "on_missing": "unavailable" // unavailable | value | veto
  },
  "influence": {"weight": 1.0, "exp_mix": 0.7, "exp_k": 2.5, "in_min": 0.6}
}
```

### mode: `per_candidate` - one score per drive

Matching prefers **the slot whose `layout.dim[i].label` equals the drive name**, and falls
back to the order in `params.order` for `data[i]`. A `NaN` makes that one drive
"cannot judge".

`params.input_min/input_max/invert` convert raw units to [0,1]; `influence`'s
`in_min/in_max/invert` are the response-curve shaping that happens afterwards.

---

## Selection and post-processing (yaml)

```yaml
scoring:
  combine: "softmax"        # softmax | weighted_sum
  temperature: 1.0
  missing: "mask"           # mask | zero
  ema_alpha: 0.4
  min_valid_score: 0.0

selection:
  switch_margin: 0.10       # difference of probabilities
  switch_cooldown_ms: 500   # no further switch for this long after one
  fallback: "pp_main"       # last resort when everything is disqualified (must itself be valid)
```

- While the incumbent is **alive**, a switch needs both `switch_margin` and
  `switch_cooldown_ms` to be satisfied.
- When the incumbent is **disqualified**, the switch is immediate, with no hysteresis.
- If everything is disqualified: `fallback`; if that fails too, "no selection" and
  `timeout_stop` decelerates to a halt.
- `/drive` keeps being published even with nothing to drive with - that is how the
  deceleration command reaches the car.

The order of `postprocess.pipeline` **is** the order of application.

| type | Does |
|---|---|
| `timeout_stop` | decelerate to a stop at `decel` when there is no drive to use |
| `switch_blend` | smooth the jump at an A->B switch over `duration_ms` (`linear`/`smooth`/`ema`) |
| `rate_limit` | limit steering rate [deg/s] and acceleration/deceleration [m/s^2] |
| `speed_scale` | final speed multiplier; settable at runtime with `ros2 param set` |
| `deadband` | squash tiny commands to zero to stop servo chatter |
| `clamp` | absolute limits. **Put it last** so no earlier stage can exceed them |

```bash
ros2 param set /co_driver_node postprocess.speed_scale.scale 0.5
```

A new stage means one case each in the enum, the parser and the apply of `PostProcess` in
[src/co_driver_node.cpp](src/co_driver_node.cpp).

---

## Tuning loop (hot reload)

If only coefficients changed, they are re-read without a restart.

```bash
# after editing the JSON, or -
ros2 param set /co_driver_node scoring.temperature 0.2   # if you changed a yaml value
ros2 service call /co_driver_node/reload std_srvs/srv/Trigger
```

Applies to: the influence matrix, `scoring`, `selection`, `postprocess`, and a drive's
timeout / bias / enabled. Does **not** apply to: topic names, rates, or the inputs/drives
lists themselves - those need subscriptions rebuilt, so the service refuses and says why.

---

## Diagnostic topics

| Topic | Type | Contents |
|---|---|---|
| `/co_driver_node/selected` | `std_msgs/String` | **the drive actually in use** - the owner of what goes out on `/drive` |
| `/co_driver_node/runner_up` | `std_msgs/String` | **second place by summed score** |
| `/co_driver_node/scores` | `std_msgs/Float64MultiArray` | softmax probability per drive; name in `layout.dim[i].label` |
| `/co_driver_node/status` | `std_msgs/String` | one line of JSON: rank, logit, per-input contribution |

`runner_up` is **pure score rank 2**. It has nothing to do with the selection logic, so
when hysteresis keeps the selection off rank 1 it can equal `selected` (that is correct).
With only one valid drive it is an empty string.

For third place and below, read the `rank` field in `status` or sort `/scores`.

```jsonc
{"name": "pp_left", "rank": 2, "score": 0.424, "logit": 1.9143, "raw": 1.9143, "bias": 0.0,
 "active": true, "valid": true,
 "age_ms": 26.5,    // since the last message
 "hold_ms": 300.0,  // configured validity window
 "hz": 20.0,        // measured receive rate (always measured, independent of config)
 "inputs": {
   "localization": {"x": 0.9, "s": 0.640885, "w": 1.2, "c": 0.769062}
   //                 x = raw scorer value, s = after phi, w = W[j][i], c = contribution W*phi
 }}
```

A dropped-out drive looks like this - the score survives, only the ranking loses it:

```
  pp_main   hz= 99.0  hold=300ms  age=    5.5ms  active=True  rank=2  p=0.3989 logit=1.1798
  pp_left   hz= 20.0  hold=300ms  age= 1546.5ms  active=False rank=None p=0.0000 logit=1.9143
                                                     stale 1546ms (hold 300ms)
  pp_right  hz=  5.0  hold=300ms  age=  103.0ms  active=True  rank=1  p=0.6011 logit=1.5898
```

`rank` is the **score ranking** (valid drives only, `null` when disqualified) and
`selected` is the **actual choice**. Hysteresis (`switch_margin` / `switch_cooldown_ms`)
can put them out of step, and seeing that difference is the point.

```
    /selected = 'pp_left'   /runner_up = 'pp_left'
      rank 1  pp_right  p=0.4313     <- higher score
      rank 2  pp_left   p=0.4240     <- but switch_margin not met, so this one still drives (= /runner_up)
      rank 3  pp_main   p=0.1447
```

In `inputs`, `null` means cannot judge and `"veto"` means disqualified. Whatever killed a
drive is spelled out in its `reject` field.

---

## red_damvi: gap_follow as the localization fallback

A second, self-contained configuration that arbitrates two candidates using the
localization confidence alone:

| Candidate | Topic | Needs |
|---|---|---|
| `pp_main` (PPcontroller) | `/drive_main` | a map and a working localization |
| `gap_follow` | `/drive_gf` | `/scan` only |

**When localization collapses, the car is handed to gap_follow, and taken back when it
recovers.**

```bash
ros2 launch co_driver red_damvi.launch.py
# with a rosbag replay or any BEST_EFFORT scan publisher:
ros2 launch co_driver red_damvi.launch.py scan_bridge:=true
```

| Launch argument | Default | Meaning |
|---|---|---|
| `gap_follow` | `true` | also start gap_follow, forced onto `/drive_gf` |
| `monitor` | `true` | also start `drive_monitor` |
| `csv` | `''` | path for the monitor's per-sample CSV |
| `scan_bridge` | `false` | relay `/scan` BEST_EFFORT -> `/scan_reliable` RELIABLE |
| `scan_topic` | `/scan` | source scan topic |

Files: [config/co_driver_red_damvi.yaml](config/co_driver_red_damvi.yaml),
[config/co_driver_red_damvi_topics.jsonc](config/co_driver_red_damvi_topics.jsonc),
[launch/red_damvi.launch.py](launch/red_damvi.launch.py).

### How the two are scored

`pp_main` follows a map, so localization *is* its competence: a large positive weight, and
its logit collapses with the confidence. `gap_follow` is equally good with or without
localization, so it gets a constant floor from `bias` plus a small negative weight that
makes it step aside once confidence returns.

```
z(pp_main)    = 3.0 * phi(confidence)
z(gap_follow) = 1.0 - 1.5 * phi(confidence)
phi: in_min 0.02, in_max 0.35, exp_mix 1.0, exp_k 3.0
```

### Why the thresholds are so low

They come from measured confidence distributions rather than intuition. Over four
replayed scenes (51k samples), the 5th percentile of confidence **during normal tracking**
was:

| scene | Tracking p50 | Tracking p05 |
|---|---|---|
| icra | 0.983 | 0.877 |
| 0522-2 | 0.987 | 0.867 |
| 0526-1 | 0.927 | 0.530 |
| busan2 | 0.794 | 0.361 |

busan2 spends 13.5% of healthy tracking below 0.5, so a threshold anywhere near 0.5 would
hand the car to gap_follow during perfectly good driving. Real failures, meanwhile, take
the confidence to 0.0. So the whole decision lives at the bottom of the scale:

| | |
|---|---|
| hand over to gap_follow | confidence falls below **0.171** |
| hand back to pp_main | confidence rises above **0.226** |
| deadband 0.171 - 0.226 | whatever is driving keeps driving |

### Three defences against flapping

| Layer | Setting | Effect |
|---|---|---|
| EMA on the logits | `scoring.ema_alpha: 0.08` | ~250 ms time constant: a **sustained** collapse switches in **0.46 s**, a dip of **400 ms or less is rejected outright** |
| Hysteresis | `selection.switch_margin: 0.15` | the 0.171 - 0.226 deadband above |
| Cooldown | `selection.switch_cooldown_ms: 1500` | gap_follow keeps the car for at least that long once it has it |

`selection.fallback` is `gap_follow`: "everything is disqualified" usually means
localization went with it, and the controller that only needs `/scan` is the last one
worth trusting.

### Measured behaviour

Against a live localization replaying the `0526-1` dataset (132 s of confidence at
100 Hz, `/scan` at 37 Hz driving a real gap_follow, a stand-in publisher on
`/drive_main`), with `drive_monitor` recording every switch:

| Event in the trace | Switches | Expected |
|---|---|---|
| startup convergence, confidence to 0.0 twice | 4 | 2 per collapse |
| 36-42 s collapse to 0.0 | 2 | 2 |
| 59-66 s collapse to 0.0, twice | 4 | 2 per collapse |
| 83-89 s collapse to 0.0 | 2 | 2 |
| 105-107 s dip, min 0.29, entering the shaped band four times in half a second | **0** | 0 |
| 121-130 s wobble between 0.58 and 0.60 | **0** | 0 |

227 s observed, 12 switches, **0 flaps**, gap_follow occupancy 5.0%, status 50.0 Hz,
output 100.0 Hz. The two silent rows are the interesting ones: those are the shapes that
make a naive threshold chatter, and they produced nothing.

Two behaviours worth knowing about, both confirmed on real data rather than fixtures:

- **A candidate that is not alive never gets the car.** When the replay ended, the
  confidence went to 0.0 (topic silent, so `on_missing: value` scored it 0) *and*
  gap_follow went stale from the loss of `/scan`. co_driver kept pp_main rather than
  handing over to a dead candidate.
- **Back-to-back dropouts produce short stints on gap_follow.** At 59-66 s the
  localization genuinely recovered for 2.4 s between two collapses, so the car was handed
  back and then taken again. The shortest dwell was the cooldown floor exactly. This is
  the arbitration tracking a real signal, not chatter - but it is the reason
  `switch_cooldown_ms` is worth thinking about, and why raising it much further is the
  wrong answer (see the comment in the yaml).

### The state gate

slam_ours computes the confidence as `(state multiplier) x min(5 terms)` where the state
multiplier is Lost 0.0 / Converging 0.5 / Tracking 1.0. A middling value is therefore
ambiguous - 0.45 can be a healthy Converging (raw 0.9) or a broken Tracking. The
`localization_state` scorer subscribes `/slam_ours/state` (`std_msgs/UInt8`, 0=Lost
1=Converging 2=Tracking, latched) to carry that distinction.

It is wired as a **gate, not a score**: weight 0 everywhere, with `veto_below: 0.25` on
pp_main only. Because a zero weight adds nothing to any `sum|W|`, the calibrated
confidence thresholds are numerically untouched whether the topic exists or not - against
localization_pf (which has no state topic) the input reports unavailable and the
arbitration behaves exactly as if it were not configured. Verified: with the topic
absent, per-drive probabilities are bit-identical to the pre-gate configuration.

What it adds:

- **Lost disqualifies pp_main instantly.** An invalid incumbent switches without
  hysteresis, so a declared Lost hands over in ~20 ms (measured) versus ~460 ms through
  the confidence EMA. The confidence pathway remains as the backstop for the "Tracking
  but broken" case, where the state stays 2 while the terms collapse.
- **Converging is not a failure.** State 1 passes the gate, so a healthy convergence
  (confidence structurally halved) cannot be mistaken for a breakdown.

The raw-value contract is deliberate: the localization side ships `confidence` and
`state` unfiltered, and every bit of smoothing, hysteresis and dwell logic lives here.
One owner for the filtering, one place to tune it.

### gap_follow gotchas

Two things will silently produce no `/drive_gf` at all:

1. **QoS.** gap_follow builds its LaserScan subscription from a plain depth int, which
   rclcpp turns into a **RELIABLE** subscription, while LiDAR drivers and rosbag replays
   publish `/scan` **BEST_EFFORT**. Incompatible, so gap_follow receives nothing.
   `scan_bridge:=true` relays across the mismatch; the real fix is `SensorDataQoS()` in
   gap_follow.
2. **Namespace.** gap_follow's config yaml is keyed `gap_follow/gap_follow`, so the node
   must run in the `gap_follow` namespace or none of its parameters are declared and it
   aborts on its first `get_parameter()`. The launch file here does that.

Also note gap_follow's own default is `drive_topic: /drive`, which is co_driver's **output**
topic. The launch file always overrides it to `/drive_gf`; `drive_monitor` logs an ERROR
if it ever sees two publishers on the output.

---

## drive_monitor - which topic is driving, and is it flapping?

[scripts/drive_monitor.py](scripts/drive_monitor.py) reads `~/status` and the output
`/drive`, and needs nothing else.

```bash
ros2 run co_driver drive_monitor.py --ros-args \
  -p status_topic:=/co_driver_node/status -p output_topic:=/drive \
  -p csv_path:=/tmp/arbitration.csv
```

Periodic line - who is selected, and every candidate's probability, rank, measured rate,
age and command:

```
[  28.00s] selected=pp_main (held 11.3s)  localization=0.230  switches=2 (last 10s: 0)
   | gap_follow p=0.514 r1 37Hz age=1ms v=1.25 s=+16.6 | *pp_main p=0.486 r2 50Hz ... | out v=2.59 s=-11.9
```

That line is worth reading twice: gap_follow holds **rank 1** while pp_main stays
**selected**. That is `switch_margin` doing its job.

On every switch it logs the transition, how long the previous drive was held, the input
that caused it, and **how far the output actually jumped**:

```
SWITCH #1  [  11.05s]  pp_main -> gap_follow   (held 10.85s)   localization=0.000
           p: pp_main=0.416 gap_follow=0.584   reason: switched on score margin
   output discontinuity over 150ms: dv=0.48 m/s  dsteer=15.3 deg
```

| Detection | Parameter | Default |
|---|---|---|
| too many switches in a window | `window_s`, `max_switches_per_window` | 10 s, 3 |
| A->B->A flap | `flap_dwell_s` | 1.5 s |
| two publishers on the output | - | always on |

Ctrl-C prints a summary: occupancy per drive, switches per minute, dwell min/median/max,
worst output jump at a switch, flap count, and a PASS/WARN verdict.

### Bench fixtures

For testing the arbitration with no car and no localization stack:

| Script | Does |
|---|---|
| [scripts/fake_localization_confidence.py](scripts/fake_localization_confidence.py) | synthetic `/localization_confidence`: a scripted sequence of collapse, glitch, threshold hover and silence, or `const` / `sweep` / `hover` profiles |
| [scripts/fake_drive.py](scripts/fake_drive.py) | a smooth continuous command on one drive topic, so any discontinuity at a handover is the arbitration's, not the source's |
| [scripts/scan_qos_bridge.py](scripts/scan_qos_bridge.py) | `/scan` BEST_EFFORT -> RELIABLE relay (see gap_follow gotchas) |

```bash
ros2 run co_driver fake_drive.py --ros-args -p topic:=/drive_main -p speed:=2.5
ros2 run co_driver fake_localization_confidence.py --ros-args -p profile:=script
ros2 launch co_driver red_damvi.launch.py gap_follow:=false csv:=/tmp/bench.csv
```

---

## Build and run

```bash
sudo apt install ros-jazzy-ackermann-msgs nlohmann-json3-dev

cd ~/Desktop/co-driver
colcon build --packages-select co_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch co_driver co_driver.launch.py
# with a different configuration
ros2 launch co_driver co_driver.launch.py config:=my.yaml topics:=my_topics.jsonc
# without launch
ros2 run co_driver co_driver_node --ros-args \
    --params-file my.yaml -p topics_file:=my_topics.jsonc
```

### Configuring the PPcontroller side

Point each instance at its own topic via `drive_topic`
(`PPcontroller/config/config.yaml`):

```yaml
pure_pursuit:
  ros__parameters:
    drive_topic: "/drive_main"     # /drive_left, /drive_right, ... per instance
```

All that matters is that `drives[].topic` in `co_driver_topics.jsonc` matches these names.

---

## Safety notes

- `/drive` keeps publishing even when there is nothing to drive with, so `timeout_stop`'s
  deceleration reaches the car. (Stopping the publication instead would trip the mux's own
  0.2 s timeout and stop hard.)
- `clamp.min_speed: 0.0` is the default, so no reverse command is ever emitted.
- The joystick e-stop is handled by the mux, not by co_driver. Pointing the output at
  `/ackermann_drive` to bypass the mux **removes that protection**.
- A large negative `exp_k` makes the score fall almost like a step. That is useful for a
  safety metric but invites frequent switching, so read it together with `switch_margin`
  and `switch_cooldown_ms`.
