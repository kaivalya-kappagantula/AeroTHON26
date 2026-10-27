# sim_down_camera_node.py — SITL stand-in for the downward Cam3 + camera_ros.
#
# Renders what the nadir camera would see from the vehicle's MAVROS pose:
# grass, red zones and QR codes lying on flat ground. Publishes the same
# /down_frame feed camera_ros does, so the REAL qr_scanner_node and
# red_zone_node run unchanged in SITL.
#
#   in : /mavros/imu/data, /mavros/global_position/global, /mavros/global_position/rel_alt
#   out: /down_frame (sensor_msgs/Image, bgr8), /camera_info
#
# Ground objects are in metres East (x) / North (y) of the first GPS fix —
# start it with the vehicle on the takeoff point:
#   red_zones  JSON polygons      '[[[4,3],[6,3],[6,5],[4,5]]]'
#   qr_codes   JSON [x, y, text]  '[[0,1,"AT-42"],[2.5,11.5,"AT-42"],[8,5,"AT-17"]]'
#   marker_size_m  side of the QR's black modules (the 2-module white quiet
#                  zone is added around it)
#
#   ros2 run qr_detection sim_down_camera_node --ros-args \
#       -p red_zones:="'[[[4,3],[6,3],[6,5],[4,5]]]'" -p qr_codes:="'[[0,1,\"A\"]]'"

import json
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from qr_detection.red_zone import (CameraModel, VehicleState, euler_ned_from_enu_quat,
                                   qr_image, render_ground_image, render_sim_frame)

_M_PER_DEG_LAT = 111_320.0


class SimDownCamera(Node):
    def __init__(self):
        super().__init__("sim_down_camera")
        gp = lambda n, d: self.declare_parameter(n, d).value        # noqa: E731
        w, h = int(gp("width", 1280)), int(gp("height", 720))
        self.cam = CameraModel.cam3_default(w, h, float(gp("mount_yaw_deg", 0.0)))
        marker = float(gp("marker_size_m", 1.0))
        # JSON strings; tolerate the YAML quotes `ros2 run -p` needs if they
        # come through a launch argument verbatim
        js = lambda n: json.loads(str(gp(n, "[]")).strip().strip("'"))  # noqa: E731
        # x East / y North -> [north, east]
        self.zones = [np.array([[y, x] for x, y in z], np.float64) for z in js("red_zones")]
        self.qrs = []
        for x, y, text in js("qr_codes"):
            tex = qr_image(str(text))
            self.qrs.append(((float(y), float(x)), tex, marker * tex.shape[0] / (tex.shape[0] - 4)))
        self.background = render_sim_frame(self.cam, VehicleState(0, 0, 1, 0, 0, 0), [],
                                           np.random.default_rng(0))
        self._bridge = CvBridge()
        self._origin = self._ne = self._att = self._alt = None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Imu, "/mavros/imu/data", self._imu_cb, qos)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._gps_cb, qos)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt", self._alt_cb, qos)
        self.pub = self.create_publisher(
            Image, "/down_frame", QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                             history=HistoryPolicy.KEEP_LAST, depth=1))
        self.info_pub = self.create_publisher(CameraInfo, "/camera_info", 10)
        self.create_timer(1.0 / float(gp("fps", 30.0)), self._tick)
        self.get_logger().info(
            f"[sim_cam] {w}x{h}, {len(self.zones)} red zone(s), {len(self.qrs)} QR(s)")

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self._att = euler_ned_from_enu_quat(q.x, q.y, q.z, q.w)

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
        lat0, lon0 = self._origin
        self._ne = ((msg.latitude - lat0) * _M_PER_DEG_LAT,
                    (msg.longitude - lon0) * _M_PER_DEG_LAT * math.cos(math.radians(lat0)))

    def _alt_cb(self, msg: Float64):
        self._alt = float(msg.data)

    def _tick(self):
        if self._ne is None or self._att is None or self._alt is None:
            return
        # On the ground the camera sits ~0.15 m up: never project from <= 0.
        st = VehicleState(self._ne[0], self._ne[1], max(self._alt, 0.15), *self._att)
        img = render_sim_frame(self.cam, st, self.zones, self.background)
        for centre, tex, side in self.qrs:
            render_ground_image(img, self.cam, st, tex, centre, side)
        stamp = self.get_clock().now().to_msg()
        msg = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = "down_camera"
        self.pub.publish(msg)
        self._n = getattr(self, "_n", 0) + 1
        self.get_logger().info(f"[sim_cam] {self._n} frames published",
                               throttle_duration_sec=10.0)
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = self.cam.width, self.cam.height
        info.k = [self.cam.fx, 0.0, self.cam.cx, 0.0, self.cam.fy, self.cam.cy, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = SimDownCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
