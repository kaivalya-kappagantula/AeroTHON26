# obstacle_publisher_node.py — FALLBACK path only (Section 4.2).
#
# The PRIMARY path is to remap /avoidance/scan -> /mavros/obstacle/send and let
# MAVROS's obstacle_distance plugin build the MAVLink. Build/run this node ONLY
# if Gate 1A or 1B shows that path is unavailable or broken. It packs a
# LaserScan into MAVLink OBSTACLE_DISTANCE ourselves.
#
# Two transports, selected by the `transport` parameter (a launch argument, not
# a code branch you flip by editing):
#
#   transport:=pymavlink   Open a SECOND MAVLink connection (SITL only — use a
#                          spare port such as udpout:127.0.0.1:14551, never
#                          fight MAVROS for 5760). On hardware there is a single
#                          UART; do NOT open a second connection to it.
#
#   transport:=mavros      Wrap the packed frame as mavros_msgs/Mavlink and
#                          publish to /uas1/mavlink_sink. One connection, works
#                          on hardware. Preferred there.
#
# The mirror flip (Trap #1) lives in geometry.laserscan_bin_to_mavlink_index():
# LaserScan is CCW/left-positive, OBSTACLE_DISTANCE is CW/right-indexed. Gate 1B
# verifies the result empirically.

import os

# mavros.mavlink.convert_to_rosmsg() emits a MAVLink v2 frame ONLY when this
# variable is set; otherwise it writes a v1 header (magic 0xFE). OBSTACLE_DISTANCE
# is msg id 330, which does not exist in v1, so mavros_router silently drops
# every frame and ArduPilot reports "PRX1: No Data" (and refuses to arm).
# Must be set before pymavlink is imported anywhere in this process.
os.environ.setdefault('MAVLINK20', '1')

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402

from sensor_msgs.msg import LaserScan  # noqa: E402

from obstacle_avoidance import geometry as g  # noqa: E402

MAVLINK_UNKNOWN = 65535          # UINT16_MAX: "no reading" in OBSTACLE_DISTANCE


class ObstaclePublisherNode(Node):

    def __init__(self):
        super().__init__('obstacle_publisher_node')

        self.declare_parameter('n_bins', 72)
        self.declare_parameter('bin_width_deg', 5.0)
        self.declare_parameter('range_min_m', 0.40)
        self.declare_parameter('range_max_report_m', 6.0)

        # transport: 'pymavlink' (2nd connection, SITL) or 'mavros' (sink).
        self.declare_parameter('transport', 'pymavlink')
        self.declare_parameter('mavlink_url', 'udpout:127.0.0.1:14551')

        self.n_bins = int(self.get_parameter('n_bins').value)
        self.bin_width_deg = float(self.get_parameter('bin_width_deg').value)
        self.range_min_cm = int(
            round(float(self.get_parameter('range_min_m').value) * 100.0))
        self.range_max_cm = int(
            round(float(self.get_parameter('range_max_report_m').value) * 100.0))
        self.transport = str(self.get_parameter('transport').value).lower()

        # angle_offset: index 0 sits at -180 deg; increment is +CW (right).
        self.angle_offset_deg = -180.0

        self._mav = None            # pymavlink connection (transport=pymavlink)
        self._sink_pub = None       # /uas1/mavlink_sink publisher (transport=mavros)
        self._convert = None        # mavros.mavlink.convert_to_rosmsg
        self._setup_transport()

        self.sub = self.create_subscription(
            LaserScan, '/avoidance/scan', self._on_scan,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'obstacle_publisher_node up (FALLBACK), transport={self.transport}, '
            f'{self.n_bins} bins.')

    # ----------------------------------------------------------------- setup
    def _setup_transport(self):
        if self.transport == 'pymavlink':
            from pymavlink import mavutil
            url = str(self.get_parameter('mavlink_url').value)
            self.get_logger().info(f'Opening pymavlink connection: {url}')
            # source_system 1 so ArduPilot accepts it as the companion.
            self._mav = mavutil.mavlink_connection(
                url, source_system=1, source_component=195)
            self._mav_mod = mavutil.mavlink
        elif self.transport == 'mavros':
            from pymavlink.dialects.v20 import ardupilotmega as mav_dialect
            from mavros.mavlink import convert_to_rosmsg
            from mavros_msgs.msg import Mavlink
            self._convert = convert_to_rosmsg
            # A headless MAVLink object just to *encode* frames; not connected.
            self._encoder = mav_dialect.MAVLink(
                file=None, srcSystem=1, srcComponent=195)
            self._mav_mod = mav_dialect
            self._sink_pub = self.create_publisher(
                Mavlink, '/uas1/mavlink_sink', 10)
        else:
            raise ValueError(
                f"unknown transport '{self.transport}' "
                "(expected 'pymavlink' or 'mavros')")

    # ------------------------------------------------------------ conversion
    def _scan_to_distances_cm(self, scan):
        """LaserScan -> 72-element uint16 cm array with the CCW->CW mirror flip."""
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size != self.n_bins:
            self.get_logger().warn(
                f'scan has {ranges.size} bins, expected {self.n_bins}; skipping',
                once=True)
            return None

        # metres -> cm; inf/nan/out-of-band -> unknown marker.
        finite = np.isfinite(ranges)
        cm = np.full(self.n_bins, MAVLINK_UNKNOWN, dtype=np.int64)
        valid = finite & (ranges > 0.0)
        cm_vals = np.rint(ranges[valid] * 100.0).astype(np.int64)
        cm_vals = np.clip(cm_vals, self.range_min_cm, self.range_max_cm)
        cm[valid] = cm_vals

        # THE FLIP: LaserScan bin b -> MAVLink index (n - b) % n.
        out = np.full(self.n_bins, MAVLINK_UNKNOWN, dtype=np.int64)
        mav_idx = g.laserscan_bin_to_mavlink_index(
            np.arange(self.n_bins), n_bins=self.n_bins)
        out[mav_idx] = cm
        return out.astype(np.uint16)

    # ------------------------------------------------------------- callbacks
    def _on_scan(self, scan):
        distances = self._scan_to_distances_cm(scan)
        if distances is None:
            return

        now_us = self.get_clock().now().nanoseconds // 1000
        args = dict(
            time_usec=now_us,
            sensor_type=self._mav_mod.MAV_DISTANCE_SENSOR_LASER,
            distances=distances.tolist(),
            increment=int(round(self.bin_width_deg)),
            min_distance=self.range_min_cm,
            max_distance=self.range_max_cm,
            increment_f=float(self.bin_width_deg),
            angle_offset=float(self.angle_offset_deg),
            frame=self._mav_mod.MAV_FRAME_BODY_FRD,
        )

        if self.transport == 'pymavlink':
            self._mav.mav.obstacle_distance_send(**args)
        else:
            msg = self._encoder.obstacle_distance_encode(**args)
            # pack() needs the encoder as the mav backend to fill CRC/seq;
            # it does not advance seq (only send() does), so do it here.
            msg.pack(self._encoder)
            self._encoder.seq = (self._encoder.seq + 1) % 256
            self._sink_pub.publish(self._convert(msg))


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisherNode()
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
