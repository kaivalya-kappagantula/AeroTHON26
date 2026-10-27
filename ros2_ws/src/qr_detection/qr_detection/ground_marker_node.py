# ground_marker_node.py — publishes which corridor marker colour is under the
# vehicle, from the downward camera.
#
#   in : /down_frame                (sensor_msgs/Image, same feed as the QR scanner)
#   out: /corridor/ground_marker    (std_msgs/String: "green" | "blue" | "none")
#
# green = corridor entrance, blue = corridor exit. A colour is reported only
# after gate_frames consecutive frames agree. path_planner reads this during
# corridor legs only — the arena is grass, so "green" elsewhere is meaningless.
# Colour logic and the tuning note are in ground_marker.py.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from qr_detection.ground_marker import (BLUE, GREEN, ColourBand, MarkerGate,
                                        classify, colour_fractions)

TOPIC = "/corridor/ground_marker"
NAMES = ("green", "blue")


class GroundMarkerNode(Node):
    def __init__(self):
        super().__init__("ground_marker_node")
        self.declare_parameter("marker_rate_hz", 10.0)
        self.declare_parameter("marker_min_fraction", 0.20)
        self.declare_parameter("marker_gate_frames", 3)
        for name, band in (("green", GREEN), ("blue", BLUE)):
            self.declare_parameter(
                f"{name}_hsv", [band.h_lo, band.h_hi, band.s_min, band.v_min])
        gp = lambda n: self.get_parameter(n).value          # noqa: E731
        self.bands = [ColourBand(*[int(v) for v in gp(f"{n}_hsv")]) for n in NAMES]
        self.min_fraction = float(gp("marker_min_fraction"))
        self.gate = MarkerGate(int(gp("marker_gate_frames")))
        self.period = 1.0 / float(gp("marker_rate_hz"))
        self._last_t = 0.0
        self._bridge = CvBridge()

        self.pub = self.create_publisher(String, TOPIC, 10)
        self.create_subscription(
            Image, "/down_frame", self._on_frame,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.get_logger().info(
            f"[ground_marker] green={gp('green_hsv')} blue={gp('blue_hsv')} "
            f"min_fraction={self.min_fraction} -> {TOPIC}")

    def _on_frame(self, msg: Image):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_t < self.period:
            return
        self._last_t = now
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:      # cv_bridge raises its own error types
            self.get_logger().warn(f"[ground_marker] bad frame: {e}",
                                   throttle_duration_sec=5.0)
            return
        fr = colour_fractions(bgr, self.bands)
        state = self.gate.update(classify(fr, NAMES, self.min_fraction))
        self.pub.publish(String(data=state))
        self.get_logger().info(
            f"[ground_marker] green={fr[0]:.2f} blue={fr[1]:.2f} -> {state}",
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GroundMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
