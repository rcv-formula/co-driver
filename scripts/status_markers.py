#!/usr/bin/env python3
"""RViz markers for the arbitration - which controller is driving, and why.

Reads only /co_driver_node/status (JSON) and TF, publishes one text block above
the vehicle:

    > gap_follow                      <- colored: pp_main green / gap_follow
    recovery gate: recovering 1.2/3.0s   orange / nothing red
    conf 0.42  p: main 0.03 gf 0.97

Placement is coordinated with the slam_ours eval markers, which sit at z=0.6
above the vehicle (ns "slip") and at the map centre (reloc_*): this one uses
ns "co_driver" at z=1.2 so both stacks are readable at once.

  ros2 run co_driver status_markers.py
  # RViz: add MarkerArray on /co_driver/markers, fixed frame "map"
"""
import json
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

COLORS = {
    "pp_main": (0.20, 0.90, 0.30),     # green - raceline controller
    "gap_follow": (1.00, 0.60, 0.10),  # orange - reactive fallback
    None: (0.95, 0.15, 0.15),          # red - nothing valid
}


class StatusMarkers(Node):
    def __init__(self):
        super().__init__("co_driver_markers")
        p = self.declare_parameter
        self.frame = p("frame", "map").value
        self.base = p("base_frame", "base_link").value
        self.z = p("z", 1.2).value
        self.scale = p("text_height", 0.35).value
        self.rate = p("rate_hz", 10.0).value

        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self)
        self.pub = self.create_publisher(MarkerArray, "/co_driver/markers", 10)
        self.create_subscription(String, "/co_driver_node/status", self.on_status, 10)
        self.st = None
        self.last_pos = (0.0, 0.0)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"markers on /co_driver/markers (ns co_driver, {self.base} + z={self.z})")

    def on_status(self, msg):
        try:
            self.st = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def reason_line(self, st):
        """One line saying which pathway is in charge right now."""
        pp = next((d for d in st["drives"] if d["name"] == "pp_main"), None)
        if pp is None:
            return st.get("reason", "")
        rej = pp.get("reject", "")
        if "veto_below[loc_state]" in rej:
            return "STATE VETO: Lost / no map"
        if "veto_below[recovery]" in rej:
            note = (pp.get("inputs", {}).get("recovery") or {})
            n = note.get("note", "") if isinstance(note, dict) else ""
            return f"recovery gate: {n or 'waiting for sustained health'}"
        if rej:
            return rej
        return st.get("reason", "")

    def tick(self):
        if self.st is None:
            return
        st = self.st
        try:
            tr = self.buf.lookup_transform(
                self.frame, self.base, rclpy.time.Time(), Duration(seconds=0.05))
            self.last_pos = (tr.transform.translation.x, tr.transform.translation.y)
        except Exception:
            pass  # keep the last known spot; markers should never crash the run
        x, y = self.last_pos

        sel = st.get("selected") or None
        r, g, b = COLORS.get(sel, COLORS[None])
        probs = "  ".join(
            f"{d['name'].replace('pp_main', 'main').replace('gap_follow', 'gf')} "
            f"{d.get('score') or 0:.2f}" for d in st.get("drives", []))
        conf = ""
        for d in st.get("drives", []):
            e = (d.get("inputs") or {}).get("localization")
            if isinstance(e, dict) and "x" in e:
                conf = f"conf {e['x']:.2f}  "
                break
        text = (f"> {sel or 'NO DRIVE'}\n"
                f"{self.reason_line(st)}\n"
                f"{conf}p: {probs}")

        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "co_driver"
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = self.z
        m.pose.orientation.w = 1.0
        m.scale.z = self.scale
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        m.text = text
        m.lifetime = Duration(seconds=1.0).to_msg()
        self.pub.publish(MarkerArray(markers=[m]))


def main():
    rclpy.init()
    node = StatusMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
