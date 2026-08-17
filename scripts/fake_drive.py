#!/usr/bin/env python3
"""Test fixture - publishes a smooth AckermannDriveStamped on one topic.

Stands in for a controller that is not available on the bench (PPcontroller, or
gap_follow when there is no /scan to feed it). The command is smooth and
continuous so that any discontinuity drive_monitor reports at a handover comes
from the arbitration, not from the source.

  ros2 run co_driver fake_drive.py --ros-args \
    -p topic:=/drive_main -p speed:=2.5 -p steer_amp_deg:=12 -p steer_period_s:=8
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class FakeDrive(Node):
    def __init__(self):
        super().__init__("fake_drive")
        p = self.declare_parameter
        self.topic = p("topic", "/drive_main").value
        self.rate = p("rate_hz", 50.0).value
        self.speed = p("speed", 2.5).value
        self.speed_amp = p("speed_amp", 0.4).value
        self.speed_period = p("speed_period_s", 11.0).value
        self.steer_amp = math.radians(p("steer_amp_deg", 12.0).value)
        self.steer_period = p("steer_period_s", 7.0).value
        self.frame = p("frame_id", "base_link").value

        self.pub = self.create_publisher(AckermannDriveStamped, self.topic, 10)
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"fake drive on {self.topic} at {self.rate:.0f} Hz "
            f"(speed {self.speed:.2f} +/- {self.speed_amp:.2f} m/s)")

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.drive.speed = self.speed + self.speed_amp * math.sin(
            2 * math.pi * t / self.speed_period)
        msg.drive.steering_angle = self.steer_amp * math.sin(
            2 * math.pi * t / self.steer_period)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeDrive()
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
