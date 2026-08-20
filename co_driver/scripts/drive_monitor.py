#!/usr/bin/env python3
"""co_driver arbitration monitor - "which topic is driving, and is it flapping?"

co_driver puts everything it decided on ~/status as JSON. This node needs only
that plus the actual output /drive, and reports:

  1) Periodic line   selected drive, per-candidate probability / rank / measured
                     Hz / speed / steering, and the raw scorer inputs.
  2) Switch events   when, from what to what, how long the previous drive was
                     held, and **how far the output jumped** at that instant
                     (delta speed / delta steering).
  3) Oscillation     switches inside a rolling window, A->B->A flap detection,
                     shortest dwell. Anything past the thresholds logs a WARN.
  4) Wiring check    more than one publisher on the output topic is an ERROR.
                     gap_follow defaults to drive_topic:=/drive, so this happens.
  5) Exit summary    on Ctrl-C: total switches, switches per minute, dwell
                     distribution, occupancy per drive, worst output jump at a
                     switch, and a PASS/WARN verdict.

Run:
  ros2 run co_driver drive_monitor.py --ros-args \
    -p status_topic:=/co_driver_node/status -p output_topic:=/drive \
    -p csv_path:=/tmp/arbitration.csv
"""
import json
import math
import statistics
import sys
from collections import defaultdict, deque

import rclpy
from rclpy.executors import ExternalShutdownException
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

DEG = 180.0 / math.pi


class DriveMonitor(Node):
    def __init__(self):
        super().__init__("drive_monitor")

        p = self.declare_parameter
        self.status_topic = p("status_topic", "/co_driver_node/status").value
        self.output_topic = p("output_topic", "/drive").value
        self.log_period = p("log_period_s", 1.0).value
        # Name of the scorer input to display as the headline number.
        self.watch_input = p("watch_input", "localization").value
        # --- oscillation thresholds ---------------------------------------
        # More than max_switches inside window_s means "switching too often".
        self.window_s = p("window_s", 10.0).value
        self.max_switches = p("max_switches_per_window", 3).value
        # A->B->A where both dwells are shorter than this is a flap.
        self.flap_dwell_s = p("flap_dwell_s", 1.5).value
        self.csv_path = p("csv_path", "").value

        qos = QoSProfile(depth=10)
        self.create_subscription(String, self.status_topic, self.on_status, qos)
        self.create_subscription(AckermannDriveStamped, self.output_topic, self.on_output, qos)

        self.t0 = self.now()
        self._last_t = self.t0
        self.selected = None
        self.selected_since = self.t0
        self.prev_selected = None       # the one before last, for flap detection
        self.prev_dwell = None
        self.switches = []              # [t, from, to, dwell, x, jump_v, jump_d, reason]
        self.recent = deque()           # switch times inside the rolling window
        self.dwells = []
        self.occupancy = defaultdict(float)
        self.flaps = 0
        self.warned_window = False
        self.last_status = None
        self.status_count = 0
        self.out_hist = deque(maxlen=400)   # (t, speed, steer); ~4 s at 100 Hz
        self.out_count = 0
        self.pending_jump = None            # switch we are still measuring
        self.wiring_bad = False
        self.last_pubs = None      # publisher set at the last wiring check

        self.csv = None
        if self.csv_path:
            self.csv = open(self.csv_path, "w", buffering=1)
            self.csv.write("t,selected,switched,reason,%s,out_speed,out_steer_deg\n"
                           % self.watch_input)

        self.create_timer(self.log_period, self.tick)
        self.create_timer(5.0, self.check_wiring)
        self.get_logger().info(
            f"monitoring status={self.status_topic} output={self.output_topic} "
            f"(oscillation thresholds: >{self.max_switches} switches per "
            f"{self.window_s:.0f}s, flap dwell <{self.flap_dwell_s:.1f}s)")

    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        return self.now() - self.t0

    # --- wiring -------------------------------------------------------
    def check_wiring(self):
        """A second publisher on the output topic means two nodes are fighting.

        The arbiter itself is the expected publisher, so it is never the
        conflict. Everything else is named as it actually is: this used to
        blame gap_follow's drive_topic on every repeat, which is misleading
        advice when gap_follow is correctly on its own topic and the extra
        publisher is a mux, a teleop or a safety node.
        """
        pubs = self.get_publishers_info_by_topic(self.output_topic)
        names = [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in pubs]
        if not pubs:
            if self.last_pubs is None or self.last_pubs:
                self.get_logger().warn(f"no publisher on {self.output_topic}")
            self.last_pubs = []
            return

        # The arbiter is supposed to be here; anything else is a co-publisher.
        others = [n for n in names if "co_driver" not in n]
        self.wiring_bad = len(others) > 0

        # Only speak when the picture changes - this runs every 5 s, and a
        # standing condition repeated forever is noise, not a diagnosis.
        if self.last_pubs is not None and sorted(names) == sorted(self.last_pubs):
            return
        self.last_pubs = names

        if not others:
            return
        culprits = [n for n in others if "gap_follow" in n]
        hint = (
            f" gap_follow is one of them - set its drive_topic to /drive_gf."
            if culprits else
            f" gap_follow is NOT among them, so its drive_topic is fine; these are"
            f" whatever else publishes here (mux input, teleop, safety stop)."
            f" If that is intended, point this monitor at the topic the arbiter"
            f" actually owns with -p output_topic:=<topic>.")
        self.get_logger().error(
            f"{self.output_topic} has {len(others)} publisher(s) besides the "
            f"arbiter: {others}.{hint}")

    # --- output -------------------------------------------------------
    def on_output(self, msg):
        self.out_count += 1
        t = self.now()
        self.out_hist.append((t, msg.drive.speed, msg.drive.steering_angle))
        if self.pending_jump is not None:
            t_sw, v0, d0 = self.pending_jump
            if t - t_sw >= 0.15:        # largest excursion in 150 ms after the switch
                seg = [s for s in self.out_hist if t_sw <= s[0] <= t]
                jv = max((abs(s[1] - v0) for s in seg), default=0.0)
                jd = max((abs(s[2] - d0) for s in seg), default=0.0) * DEG
                self.switches[-1][5] = jv
                self.switches[-1][6] = jd
                self.get_logger().info(
                    f"   output discontinuity over 150ms: dv={jv:.2f} m/s  "
                    f"dsteer={jd:.1f} deg")
                self.pending_jump = None

    def out_now(self):
        return self.out_hist[-1][1:] if self.out_hist else (float("nan"), float("nan"))

    # --- status -------------------------------------------------------
    def on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("failed to parse status JSON")
            return
        self.status_count += 1
        self.last_status = st

        sel = st.get("selected") or None
        t = self.now()

        if self.selected is not None:
            self.occupancy[self.selected] += t - self._last_t
        self._last_t = t

        if sel != self.selected:
            if self.selected is None:
                self.get_logger().info(
                    f"[{self.elapsed():7.2f}s] first selection = {name(sel)}   "
                    f"(reason: {st.get('reason', '')})")
            else:
                dwell = t - self.selected_since
                self.dwells.append(dwell)
                self.recent.append(t)
                v0, d0 = self.out_now()
                x = self.input_x(st, self.watch_input)
                self.switches.append(
                    [t, self.selected, sel, dwell, x, 0.0, 0.0, st.get("reason", "")])
                pm = {d["name"]: num(d, "score") for d in st.get("drives", [])}
                self.get_logger().warn(
                    f"SWITCH #{len(self.switches)}  [{self.elapsed():7.2f}s]  "
                    f"{name(self.selected)} -> {name(sel)}   (held {dwell:.2f}s)   "
                    f"{self.watch_input}={fmt(x)}   "
                    f"p: {self.selected}={pm.get(self.selected, 0):.3f} "
                    f"{sel}={pm.get(sel, 0):.3f}   reason: {st.get('reason', '')}")
                if not math.isnan(v0):
                    self.pending_jump = (t, v0, d0)

                if (self.prev_selected == sel and self.prev_dwell is not None
                        and dwell < self.flap_dwell_s and self.prev_dwell < self.flap_dwell_s):
                    self.flaps += 1
                    self.get_logger().error(
                        f"   FLAP #{self.flaps}: {sel} -> {self.selected} -> {sel} "
                        f"({self.prev_dwell:.2f}s / {dwell:.2f}s, both under "
                        f"{self.flap_dwell_s:.1f}s)")
                self.prev_selected = self.selected
                self.prev_dwell = dwell
            self.selected = sel
            self.selected_since = t

        while self.recent and t - self.recent[0] > self.window_s:
            self.recent.popleft()
        if len(self.recent) > self.max_switches:
            if not self.warned_window:
                self.get_logger().error(
                    f"   switching too often - {len(self.recent)} switches in the last "
                    f"{self.window_s:.0f}s (threshold {self.max_switches}). "
                    f"Look at switch_margin / switch_cooldown_ms / scoring.ema_alpha.")
                self.warned_window = True
        else:
            self.warned_window = False

        if self.csv:
            v, d = self.out_now()
            self.csv.write(
                f"{self.elapsed():.3f},{sel},{st.get('switched')},"
                f"\"{st.get('reason', '')}\",{fmt(self.input_x(st, self.watch_input))},"
                f"{v:.4f},{d * DEG:.3f}\n")

    @staticmethod
    def input_x(st, input_name):
        """Raw scorer value x for that input, from whichever drive reports it."""
        for d in st.get("drives", []):
            e = (d.get("inputs") or {}).get(input_name)
            if isinstance(e, dict) and "x" in e:
                return e["x"]
        return float("nan")

    # --- periodic line ------------------------------------------------
    def tick(self):
        st = self.last_status
        if st is None:
            self.get_logger().warn(f"nothing received on {self.status_topic} yet")
            return
        held = self.now() - self.selected_since
        parts = []
        for d in sorted(st.get("drives", []), key=lambda x: -num(x, "score")):
            mark = "*" if d["name"] == self.selected else " "
            rank = d.get("rank")
            flag = "" if d.get("active") else f" OUT:{(d.get('reject') or '')[:24]}"
            parts.append(
                f"{mark}{d['name']} p={num(d, 'score'):.3f} "
                f"r{rank if rank else '-'} {num(d, 'hz'):.0f}Hz "
                f"age={num(d, 'age_ms'):.0f}ms "
                f"v={num(d, 'speed'):.2f} s={num(d, 'steer') * DEG:+.1f}{flag}")
        v, dd = self.out_now()
        self.get_logger().info(
            f"[{self.elapsed():7.2f}s] selected={name(self.selected)} (held {held:.1f}s)  "
            f"{self.watch_input}={fmt(self.input_x(st, self.watch_input))}  "
            f"switches={len(self.switches)} (last {self.window_s:.0f}s: {len(self.recent)})  "
            f"| " + " | ".join(parts) +
            f" | out v={v:.2f} s={dd * DEG:+.1f}")

    # --- exit summary -------------------------------------------------
    def summary(self):
        T = self.elapsed()
        out = []
        add = out.append
        add("")
        add("=" * 78)
        add(f" co_driver arbitration summary - {T:.1f}s observed")
        add("=" * 78)
        add(f"  status {self.status_count} msgs ({self.status_count / max(T, 1e-9):.1f} Hz), "
            f"output {self.out_count} msgs ({self.out_count / max(T, 1e-9):.1f} Hz)")
        add("")
        total_occ = sum(self.occupancy.values()) or 1.0
        add("  occupancy - which topic actually drove")
        for drive, sec in sorted(self.occupancy.items(), key=lambda kv: -kv[1]):
            add(f"    {name(drive):<14} {sec:7.2f}s  {100 * sec / total_occ:5.1f}%")
        add("")
        n = len(self.switches)
        add(f"  switches: {n}  ({60 * n / max(T, 1e-9):.2f} per minute)")
        if self.dwells:
            add(f"    dwell   min {min(self.dwells):.2f}s / "
                f"median {statistics.median(self.dwells):.2f}s / "
                f"max {max(self.dwells):.2f}s")
        jv = [s[5] for s in self.switches]
        jd = [s[6] for s in self.switches]
        if jv:
            add(f"    worst output jump at a switch: dv={max(jv):.2f} m/s, "
                f"dsteer={max(jd):.1f} deg")
        add(f"    flaps (A->B->A): {self.flaps}")
        add("")
        for i, (t, a, b, dw, x, v, d, why) in enumerate(self.switches, 1):
            # b is None when the switch was to "no selection at all".
            add(f"    #{i:<2} t={t - self.t0:7.2f}s  {name(a):>12} -> {name(b):<12} "
                f"held {dw:6.2f}s  {self.watch_input}={fmt(x)}  "
                f"dv={v:.2f} dsteer={d:.1f}  {why}")
        add("")

        verdict = []
        if self.wiring_bad:
            verdict.append(
                f"FAIL  {self.output_topic} has a publisher besides the arbiter")
        if self.flaps:
            verdict.append(f"WARN  {self.flaps} flap(s) detected")
        short = [d for d in self.dwells if d < self.flap_dwell_s]
        if short:
            verdict.append(
                f"WARN  {len(short)} dwell(s) under {self.flap_dwell_s:.1f}s "
                f"(shortest {min(self.dwells):.2f}s)")
        rate = 60 * n / max(T, 1e-9)
        if rate > 60 * self.max_switches / self.window_s:
            verdict.append(f"WARN  average switch rate {rate:.1f}/min exceeds the threshold")
        if not verdict:
            verdict.append("PASS  no oscillation - every switch outlasted the dwell threshold")
        add("  verdict")
        for line in verdict:
            add(f"    {line}")
        add("=" * 78)
        text = "\n".join(out)
        print(text, file=sys.stderr)
        if self.csv:
            self.csv.write("\n".join("# " + line for line in out) + "\n")
            self.csv.close()
            print(f"  CSV: {self.csv_path}", file=sys.stderr)


def fmt(x):
    return "  n/a" if x is None or x != x else f"{x:.3f}"


def name(n):
    """A drive name, or the marker for "nothing selected"."""
    return n if n else "(none)"


def num(d, key, default=0.0):
    """status JSON writes non-finite numbers as null, so never format blindly."""
    v = d.get(key)
    return default if v is None or v != v else v


def main():
    rclpy.init()
    node = DriveMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ros2 launch sends SIGINT; rclpy surfaces it as ExternalShutdownException.
        pass
    except Exception:
        # The signal handler can tear the context down mid-spin, which surfaces as an
        # RCLError instead. Only swallow it once rclpy is actually shut down.
        if rclpy.ok():
            raise
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
