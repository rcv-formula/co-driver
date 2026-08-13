#!/usr/bin/env python3
"""외부(Python) 채점기 예제 — co_driver 의 external_score 계약 데모.

C++ 을 다시 빌드하지 않고 판단 로직을 실험할 때 쓰는 틀입니다. 이 노드는
드라이브들의 /drive 와 원하는 센서를 직접 구독해서 점수를 계산하고,
Float64MultiArray 하나로 내보냅니다. co_driver 토픽 목록 JSON 의 inputs 에

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

한 덩어리만 추가하면 바로 점수 체계에 들어옵니다 — 배포 설정에는 "external" 이라는
이름으로 이미 들어 있으므로, 이 노드를 띄우기만 하면 반영됩니다.
특정 드라이브만 다르게 반응시키려면 그 드라이브의 influence 에 같은 이름을 적어
덮어쓰면 됩니다.

계약 (src/scorers/external_score.cpp 와 동일):
  * 타입은 std_msgs/Float64MultiArray.
  * mode: per_candidate 면 layout.dim[i].label 에 드라이브 이름을 넣는 것이 가장
    안전합니다(순서가 바뀌어도 짝이 맞음). label 이 비면 params 의 order 순서.
  * mode: scalar 면 index 한 칸이 차량 전체 점수.
  * stamp_indices 를 쓰면 그 칸들을 [sec, nanosec] 로 읽어 신선도를 판정합니다.
    localization_pf 의 /localization_confidence 가 이 형태입니다.
  * 값은 params 의 [input_min, input_max] 로 [0,1] 정규화되므로 원 단위 그대로
    내도 됩니다. 그 뒤 influence 의 선형/지수 곡선이 적용됩니다.
    여기서는 이해하기 쉽게 [0,1] 로 내보냅니다.

여기 담긴 예시 로직은 "조향이 완만한 명령을 선호"하는 아주 단순한 것입니다.
실제 판단 로직으로 바꿔 쓰세요.
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
        # 이 조향각(도)에서 점수가 0 이 되도록 정규화합니다.
        self.declare_parameter('steer_tolerance_deg', 30.0)

        self.names = list(self.get_parameter('candidates').value)
        topics = list(self.get_parameter('drive_topics').value)
        if len(topics) != len(self.names):
            raise RuntimeError('candidates 와 drive_topics 의 개수가 다릅니다.')

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
            f'외부 채점기 시작: {self.names} -> '
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
        """후보 하나의 점수를 [0,1] 로 돌려줍니다. 여기를 바꿔 쓰세요.

        값이 없으면 None 을 돌려주고, 아래에서 NaN 으로 실어 보냅니다. co_driver 는
        non-finite 값을 '판단 불가'로 보고 가중 평균에서 제외합니다.
        """
        drive = self.latest[name]
        if drive is None or not self._fresh(name):
            return None
        if not math.isfinite(drive.steering_angle):
            return None
        # 예시: 조향이 완만할수록 고득점.
        return max(0.0, 1.0 - abs(drive.steering_angle) / self.steer_tol)

    def _tick(self):
        msg = Float64MultiArray()
        for name in self.names:
            dim = MultiArrayDimension()
            # label 에 후보 이름을 넣으면 co_driver 가 순서와 무관하게 짝지어 줍니다.
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
