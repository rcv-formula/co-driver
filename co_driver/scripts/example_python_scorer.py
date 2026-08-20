#!/usr/bin/env python3
"""External (Python) scorer example -- demo of co_driver's external_score contract.

A template for experimenting with decision logic without rebuilding the C++.
This node subscribes directly to the candidates' /drive topics and whatever
sensors you want, computes scores, and publishes them as a single
Float64MultiArray. Add one block like this to the inputs of the co_driver
topic-list JSON:

    {
      "name": "python_score",
      "type": "external_score",
      "params": {
        "topic": "/co_driver/external_scores",
        "mode": "per_candidate",
        "order": ["pp_main", "pp_left", "pp_right"],
        "timeout": 0.5,
        "input_min": 0.0,
        "input_max": 1.0
      },
      "influence": {"weight": 1.0, "linear": 0.3, "exp": 0.7, "exp_k": -3.0}
    }

and it joins the scoring right away -- the shipped config already includes it
under the name "external", so just launching this node makes it take effect.
To make only a specific drive react differently, override that drive's
influence with the same name.

Contract (same as src/scorers/external_score.cpp):
  * The type is std_msgs/Float64MultiArray.
  * mode: per_candidate -- putting the drive name into layout.dim[i].label is
    the safest option (pairing survives reordering). If the label is empty,
    the order from params' order is used.
  * mode: scalar -- a single index holds one score for the whole vehicle.
  * With stamp_indices, those slots are read as [sec, nanosec] to judge
    freshness. localization_pf's /localization_confidence uses this shape.
  * Values are normalized to [0,1] via params' [input_min, input_max], so you
    may publish in raw units. The linear/exponential curves from influence are
    applied afterwards. Here we publish in [0,1] for clarity.

The example logic included here is trivially simple: "prefer commands with
gentle steering". Replace it with your real decision logic.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


class ExamplePythonScorer(Node):

    def __init__(self):
        super().__init__('co_driver_example_scorer')

        self.declare_parameter('candidates', ['pp_main', 'pp_left', 'pp_right'])
        self.declare_parameter('drive_topics', ['/drive_main', '/drive_left', '/drive_right'])
        self.declare_parameter('score_topic', '/co_driver/external_scores')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('timeout', 0.2)
        # Normalized so the score reaches 0 at this steering angle (degrees).
        self.declare_parameter('steer_tolerance_deg', 30.0)

        self.names = list(self.get_parameter('candidates').value)
        topics = list(self.get_parameter('drive_topics').value)
        if len(topics) != len(self.names):
            raise RuntimeError('candidates and drive_topics differ in length.')

        self.timeout = float(self.get_parameter('timeout').value)
        self.steer_tol = math.radians(float(self.get_parameter('steer_tolerance_deg').value))

        self.latest = {name: None for name in self.names}      # AckermannDrive
        self.last_rx = {name: None for name in self.names}     # rclpy.time.Time
        self.scan = None

        self.subs = []
        for name, topic in zip(self.names, topics):
            self.subs.append(
                self.create_subscription(
                    AckermannDriveStamped, topic,
                    self._make_drive_cb(name), 10))

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)

        self.pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('score_topic').value, 10)

        period = 1.0 / max(1.0, float(self.get_parameter('rate_hz').value))
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'External scorer started: {self.names} -> '
            f'{self.get_parameter("score_topic").value}')

    def _make_drive_cb(self, name):
        def cb(msg):
            self.latest[name] = msg.drive
            self.last_rx[name] = self.get_clock().now()
        return cb

    def _on_scan(self, msg):
        self.scan = msg

    def _fresh(self, name):
        rx = self.last_rx[name]
        if rx is None:
            return False
        return (self.get_clock().now() - rx).nanoseconds * 1e-9 <= self.timeout

    def _score(self, name):
        """Return one candidate's score in [0,1]. Replace this with your own.

        Returns None when no value is available; below it is sent as NaN.
        co_driver treats non-finite values as "no opinion" and excludes them
        from the weighted average.
        """
        drive = self.latest[name]
        if drive is None or not self._fresh(name):
            return None
        if not math.isfinite(drive.steering_angle):
            return None
        # Example: the gentler the steering, the higher the score.
        return max(0.0, 1.0 - abs(drive.steering_angle) / self.steer_tol)

    def _tick(self):
        msg = Float64MultiArray()
        for name in self.names:
            dim = MultiArrayDimension()
            # Putting the candidate name in label lets co_driver pair scores
            # regardless of ordering.
            dim.label = name
            dim.size = 1
            dim.stride = 1
            msg.layout.dim.append(dim)

            score = self._score(name)
            msg.data.append(float('nan') if score is None else float(score))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExamplePythonScorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
