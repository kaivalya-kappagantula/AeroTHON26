# synthetic_sector_node.py — Stage 1.
#
# Proves the ArduPilot leg of the pipeline with zero camera involvement: it
# publishes a hand-authored LaserScan (one obstacle you place by parameter) plus
# a constant coverage of 1.0. Because it bins the obstacle through the exact same
# geometry.bearing_span_to_bins() the real builder uses, a green Stage 1 means
# any later misbehaviour is in the depth path, not the ArduPilot path.
#
# The obstacle parameters are runtime-settable so Gate 1 can sweep them live:
#   ros2 param set /synthetic_sector_node obstacle_bearing_deg -30.0

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from obstacle_avoidance import geometry as g


class SyntheticSectorNode(Node):

    def __init__(self):
        super().__init__('synthetic_sector_node')

        # --- config-file parameters (shared avoidance.yaml) ---
        self.declare_parameter('n_bins', 72)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('range_min_m', 0.40)
        self.declare_parameter('range_max_report_m', 6.0)
        self.declare_parameter('scan_frame_id', 'oak_forward')

        # --- runtime-settable synthetic obstacle (declared on the node) ---
        # obstacle_bearing_deg is a CAMERA bearing: POSITIVE = RIGHT.
        self.declare_parameter('obstacle_bearing_deg', 0.0)
        self.declare_parameter('obstacle_range_m', 3.0)
        self.declare_parameter('obstacle_width_deg', 10.0)

        self.n_bins = int(self.get_parameter('n_bins').value)
        self.range_min_m = float(self.get_parameter('range_min_m').value)
        self.range_max_report_m = float(
            self.get_parameter('range_max_report_m').value)
        self.frame_id = str(self.get_parameter('scan_frame_id').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._amin, self._amax, self._ainc = g.laserscan_layout(self.n_bins)

        # /avoidance/scan feeds MAVROS's obstacle plugin, which SUBSCRIBES
        # RELIABLE. A BEST_EFFORT publisher is QoS-incompatible with it — zero
        # messages delivered, and neither side warns. Publish RELIABLE (spec
        # 3.4: direction sets the profile — we subscribe BEST_EFFORT to MAVROS
        # sensor topics but publish RELIABLE to /mavros/obstacle/send; verify
        # with `ros2 topic info -v /mavros/obstacle/send`). Coverage stays
        # BEST_EFFORT — its consumers (guard, viz) are local, not MAVROS.
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.scan_pub = self.create_publisher(
            LaserScan, '/avoidance/scan', scan_qos)
        self.cov_pub = self.create_publisher(
            Float32, '/avoidance/coverage', qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'synthetic_sector_node up: {self.n_bins} bins @ {rate:.0f} Hz, '
            f'frame "{self.frame_id}". Obstacle params are live-settable.')

    def _on_timer(self):
        bearing_deg = float(self.get_parameter('obstacle_bearing_deg').value)
        rng = float(self.get_parameter('obstacle_range_m').value)
        width_deg = float(self.get_parameter('obstacle_width_deg').value)

        # Start fully open, then stamp the obstacle into its bins. inf, never a
        # large finite number: MAVROS turns inf into the MAVLink unknown marker,
        # while a finite value is a positive assertion of clear space.
        ranges = np.full(self.n_bins, math.inf, dtype=np.float64)

        if rng > 0.0 and width_deg > 0.0:
            bins = g.bearing_span_to_bins(
                bearing_deg, width_deg, n_bins=self.n_bins)
            ranges[bins] = rng

        now = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = self.frame_id
        scan.angle_min = float(self._amin)
        scan.angle_max = float(self._amax)
        scan.angle_increment = float(self._ainc)
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min_m
        scan.range_max = self.range_max_report_m
        scan.ranges = [float(r) for r in ranges]
        scan.intensities = []
        self.scan_pub.publish(scan)

        cov = Float32()
        cov.data = 1.0
        self.cov_pub.publish(cov)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticSectorNode()
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
