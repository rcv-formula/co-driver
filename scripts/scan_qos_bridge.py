#!/usr/bin/env python3
"""Test fixture - republish a topic under a different QoS reliability.

Why this exists: gap_follow creates its LaserScan subscription with a plain depth
(`default_qos`), which rclcpp turns into a **RELIABLE** subscription. Most LiDAR
drivers and every rosbag replay publish /scan as **BEST_EFFORT**. Those two are
incompatible, so gap_follow silently receives nothing and never publishes a drive
command - and co_driver, correctly, refuses to hand the car to a candidate that
has never sent anything.

This relay subscribes BEST_EFFORT and republishes RELIABLE so gap_follow can be
tested against a replay without modifying it:

  ros2 run co_driver scan_qos_bridge.py --ros-args \
    -p in_topic:=/scan -p out_topic:=/scan_reliable
  # then run gap_follow with lidar_scan_topic:=/scan_reliable

The real fix belongs in gap_follow (use rclcpp::SensorDataQoS() for the scan).
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanQosBridge(Node):
    def __init__(self):
        super().__init__("scan_qos_bridge")
        p = self.declare_parameter
        self.in_topic = p("in_topic", "/scan").value
        self.out_topic = p("out_topic", "/scan_reliable").value
        depth = p("depth", 5).value

        sub_qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(LaserScan, self.out_topic, pub_qos)
        self.create_subscription(LaserScan, self.in_topic, self.relay, sub_qos)
        self.n = 0
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"relaying {self.in_topic} (BEST_EFFORT) -> {self.out_topic} (RELIABLE)")

    def relay(self, msg):
        self.n += 1
        self.pub.publish(msg)

    def report(self):
        if self.n == 0:
            self.get_logger().warn(f"nothing received on {self.in_topic} yet")
        else:
            self.get_logger().info(f"relayed {self.n} scans")


def main():
    rclpy.init()
    node = ScanQosBridge()
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
