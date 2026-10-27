# sitl_qr_simulator.py — Simulates QR detections for SITL testing
#
# Since there is no real camera in SITL, this node watches the drone's
# GPS position and publishes a QR detection on /qr/detection when the
# drone flies within detection_radius_m of a pre-configured QR location.
#
# Usage:
#   ros2 run mission_manager sitl_qr_simulator
#   ros2 run mission_manager sitl_qr_simulator --ros-args \
#       -p qr_location_lat:=12.9718 -p qr_location_lon:=77.5948

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

from mission_common import mavros_utils as mav


class SitlQrSimulator(Node):
    def __init__(self):
        super().__init__("sitl_qr_simulator")
        # QoS profile for high-frequency telemetry / sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── QR target parameters ─────────────────────────────────
        self.declare_parameter("qr_location_lat", 12.9718)
        self.declare_parameter("qr_location_lon", 77.5948)
        self.declare_parameter("qr_content", "QR_TARGET_DEMO_ID")
        self.declare_parameter("detection_radius_m", 5.0)

        self.qr_lat    = self.get_parameter("qr_location_lat").value
        self.qr_lon    = self.get_parameter("qr_location_lon").value
        self.qr_content = self.get_parameter("qr_content").value
        self.det_radius = self.get_parameter("detection_radius_m").value

        # ── State ────────────────────────────────────────────────
        # Re-publish (throttled) for as long as the drone is in range. A
        # one-shot detection is lost if it fires while mission_node is not
        # yet in ARENA_SEARCH (e.g. while flying corridor 1 past the QR).
        self._last_pub = 0.0
        self._republish_period_s = 1.0

        # ── Publisher ────────────────────────────────────────────
        self._pub = self.create_publisher(String, "/qr/detection", 10)

        # ── GPS subscriber ───────────────────────────────────────
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._gps_cb, sensor_qos,
        )

        self.get_logger().info(
            f"[sitl_qr] Simulating QR '{self.qr_content}' at "
            f"({self.qr_lat:.7f}, {self.qr_lon:.7f}), "
            f"detect within {self.det_radius} m"
        )

    def _gps_cb(self, msg: NavSatFix):
        now = time.monotonic()
        if now - self._last_pub < self._republish_period_s:
            return

        dist = mav.haversine(msg.latitude, msg.longitude,
                             self.qr_lat, self.qr_lon)

        if dist <= self.det_radius:
            self._last_pub = now
            # Publish in the same format qr_scanner_node uses
            detection = f"{self.qr_content}|{self.qr_lat}|{self.qr_lon}"
            out = String()
            out.data = detection
            self._pub.publish(out)
            self.get_logger().info(
                f"[sitl_qr] QR DETECTED at {dist:.1f} m — "
                f"published '{detection}'"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SitlQrSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[sitl_qr] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
