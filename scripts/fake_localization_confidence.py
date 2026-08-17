#!/usr/bin/env python3
"""Test fixture - publishes a synthetic /localization_confidence.

For bench-testing the arbitration without a localization stack. It reproduces the
shapes that matter: a clean collapse, a short glitch, and a noisy hover right on
the switching threshold (the case that actually causes flapping).

  ros2 run co_driver fake_localization_confidence.py --ros-args -p profile:=script

profile:
  script  (default) runs the scripted sequence below, then holds the last value
  const   holds `value`
  sweep   sawtooths between 0 and 1 with period `sweep_period_s`
  hover   `value` plus uniform noise of +/- `noise`, i.e. the flapping probe

The scripted sequence, in seconds from start:
   0- 6  0.95            healthy
   6- 6.2 0.0            200 ms glitch      -> must NOT switch
   6.2-12 0.95           healthy
  12-18   0.0            sustained collapse -> must switch to gap_follow
  18-24   0.95           recovery           -> must switch back
  24-34   0.20 +/- 0.05  hovering in the deadband -> must NOT switch at all
  34-40   0.36           busan2's healthy p05     -> must NOT switch
  40-46   (silence)      publisher stops          -> must switch to gap_follow
  46-52   0.95           back
"""
import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

SCRIPT = [
    (0.0, 6.0, "const", 0.95, 0.0),
    (6.0, 6.2, "const", 0.00, 0.0),
    (6.2, 12.0, "const", 0.95, 0.0),
    (12.0, 18.0, "const", 0.00, 0.0),
    (18.0, 24.0, "const", 0.95, 0.0),
    (24.0, 34.0, "hover", 0.20, 0.05),
    (34.0, 40.0, "const", 0.36, 0.0),
    (40.0, 46.0, "silent", 0.0, 0.0),
    (46.0, 52.0, "const", 0.95, 0.0),
]


class FakeConfidence(Node):
    def __init__(self):
        super().__init__("fake_localization_confidence")
        p = self.declare_parameter
        self.topic = p("topic", "/localization_confidence").value
        self.rate = p("rate_hz", 100.0).value
        self.profile = p("profile", "script").value
        self.value = p("value", 0.95).value
        self.noise = p("noise", 0.05).value
        self.sweep_period = p("sweep_period_s", 20.0).value
        self.rng = random.Random(20260818)

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.t0 = self.now()
        self.phase = None
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"fake confidence on {self.topic} at {self.rate:.0f} Hz, profile={self.profile}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def current(self, t):
        """Returns (value, publish?) and logs each phase change once."""
        if self.profile == "const":
            return self.value, True
        if self.profile == "hover":
            return self.value + self.rng.uniform(-self.noise, self.noise), True
        if self.profile == "sweep":
            return 0.5 * (1.0 - math.cos(2 * math.pi * t / self.sweep_period)), True
        for i, (a, b, kind, v, n) in enumerate(SCRIPT):
            if a <= t < b:
                if self.phase != i:
                    self.phase = i
                    self.get_logger().info(
                        f"[{t:6.2f}s] phase {i}: {kind} {v:.2f}"
                        + (f" +/- {n:.2f}" if n else "")
                        + f"  (until {b:.1f}s)")
                if kind == "silent":
                    return 0.0, False
                if kind == "hover":
                    return v + self.rng.uniform(-n, n), True
                return v, True
        return SCRIPT[-1][3], True

    def tick(self):
        t = self.now() - self.t0
        value, publish = self.current(t)
        if not publish:
            return
        stamp = self.get_clock().now()
        msg = Float64MultiArray()
        for label in ("stamp_sec", "stamp_nanosec", "confidence"):
            d = MultiArrayDimension()
            d.label, d.size, d.stride = label, 1, 1
            msg.layout.dim.append(d)
        sec, nsec = divmod(stamp.nanoseconds, 1_000_000_000)
        msg.data = [float(sec), float(nsec), float(min(max(value, 0.0), 1.0))]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeConfidence()
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
