# sitl_corridor_sim.py — SITL stand-in for the OAK-D and the downward camera
# in the corridors. NOT for the vehicle.
#
# Ray-casts the forward depth image (corridor_render) from the SITL vehicle
# pose into the two 5 ft corridors defined by the mission's corridor
# waypoints, with box obstacles, and publishes it where the OAK driver would
# (/oak/stereo/image_raw 32FC1 + camera_info). The REAL sector_builder,
# coverage_guard and corridor_perception nodes then run on it, so path_planner
# is exercised closed-loop. Also publishes /corridor/ground_marker (green over
# the entrance strip, blue over the exit strip) in place of ground_marker_node.
#
# Green banner (entry corridor): banner_width_m x banner_height_m, centred
# on corridor 1's mouth, its bottom edge banner_above_walls_m above corridor
# 1's wall tops (0 = sitting on the walls). It is drawn in the colour stream
# (/oak/rgb/image_raw bgr8 + camera_info, OAK-D Lite colour 640x360, for the
# REAL banner_node) wherever corridor 1's walls do not hide it, from any
# distance. The detector locks on it face-on from up to ~10-15 m; seen
# edge-on it is a sliver and is not found. It gives no stereo depth (plain
# cloth): its pixels are NaN in the depth image, so path_planner measures its
# height from where the walls begin.
#
# gps_drift_m / gps_drift_period_s add a slowly DRIFTING lateral error on top
# (offset + drift * sin(2 pi t / period)): the walls move against the GPS
# while the vehicle flies, as GPS wander does. 0 = off.
# Ground truth: after each pass through a corridor the true lateral offset
# from its centreline (max, RMS) and the closest wall are logged
# ("[sitl_corridor_sim] corridorN pass: ...") — how well it stayed centred.
#
# gps_offset_m shifts the world laterally against the EKF (true position =
# EKF + offset, to the LEFT of each corridor): GPS-only flight would hit a
# wall; the camera-based centring must correct it.
#
# Corridor geometry: the first waypoint is mouth_offset_m before the mouth,
# the walls end exit_offset_m before the last waypoint. Each corridor's size
# can be set on its own (corridor1_width_m, corridor1_wall_height_m,
# corridor1_length_m, same for 2); 0 = corridor_width_m, wall_height_m, and
# the length from the waypoints. Obstacles per corridor: flat list
# [s_front, lateral_centre(+left), width, top_height, ...] with s measured
# from the mouth; each box is obstacle_depth_m long.

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix
from std_msgs.msg import Float64, String

from mission_common import mavros_utils as mav
from obstacle_avoidance import geometry as g
from obstacle_avoidance.corridor_render import (
    COLOUR_CAMERA,
    Banner,
    Camera,
    Scene,
    banner_depth,
    render,
    render_colour,
)

_DA = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
M_PER_DEG = 111_320.0


class SitlCorridorSim(Node):
    def __init__(self):
        super().__init__('sitl_corridor_sim')
        dp = self.declare_parameter
        dp('origin_lat', 12.9716)
        dp('origin_lon', 77.5946)
        for i, (x, y) in ((1, ([-3.0, 13.0], [15.0, 15.0])),
                          (2, ([13.0, -3.0], [-5.0, -5.0]))):
            dp(f'corridor{i}_waypoints_x', x, _DA)
            dp(f'corridor{i}_waypoints_y', y, _DA)
            dp(f'corridor{i}_obstacles', [0.0], _DA)
            for size in ('width_m', 'wall_height_m', 'length_m'):
                dp(f'corridor{i}_{size}', 0.0)
        dp('corridor_width_m', 1.524)
        dp('wall_height_m', 3.0)
        dp('mouth_offset_m', 2.5)
        dp('exit_offset_m', 1.0)
        dp('obstacle_depth_m', 0.4)
        dp('gps_offset_m', 0.0)
        dp('gps_drift_m', 0.0)
        dp('gps_drift_period_s', 20.0)
        dp('depth_rate_hz', 5.0)
        dp('render_scale', 0.5)        # 320x240: ray casting 640x480 takes ~90 ms
        dp('marker_strip_m', 1.0)
        dp('banner_width_m', 3.5052)       # 11.5 ft
        dp('banner_height_m', 0.4572)      # 1.5 ft
        dp('banner_above_walls_m', 0.0)
        gp = lambda n: self.get_parameter(n).value          # noqa: E731

        def size(i, name, default):
            v = float(gp(f'corridor{i}_{name}'))
            return v if v > 0.0 else default

        lat0, lon0 = float(gp('origin_lat')), float(gp('origin_lon'))

        def to_gps(x, y):
            return (lat0 + y / M_PER_DEG,
                    lon0 + x / (M_PER_DEG * math.cos(math.radians(lat0))))

        self.corridors = []
        for i in (1, 2):
            xs, ys = gp(f'corridor{i}_waypoints_x'), gp(f'corridor{i}_waypoints_y')
            first, last = to_gps(xs[0], ys[0]), to_gps(xs[-1], ys[-1])
            d_n, d_e = mav.ne_offset(*first, *last)
            L = math.hypot(d_n, d_e)
            mouth = float(gp('mouth_offset_m'))
            length = size(i, 'length_m', L - mouth - float(gp('exit_offset_m')))
            ob = list(gp(f'corridor{i}_obstacles'))
            boxes = []
            for k in range(0, len(ob) - 3, 4):
                s, c, w, top = ob[k:k + 4]
                boxes.append((s, s + float(gp('obstacle_depth_m')), c - w / 2, c + w / 2, top))
            self.corridors.append(dict(
                name=f'corridor{i}', first=first, axis=(d_n / L, d_e / L),
                yaw=math.atan2(d_n, d_e), mouth=mouth, length=length,
                scene=Scene(width=size(i, 'width_m', float(gp('corridor_width_m'))),
                            x_start=0.0, x_end=length,
                            wall_height=size(i, 'wall_height_m', float(gp('wall_height_m'))),
                            boxes=boxes)))
            sc = self.corridors[-1]['scene']
            self.get_logger().info(
                f'[sitl_corridor_sim] corridor{i}: {sc.width:.2f} m wide, '
                f'{sc.wall_height:.2f} m walls, {length:.1f} m long, '
                f'{len(boxes)} obstacles {boxes}')
        bottom = self.corridors[0]['scene'].wall_height + float(gp('banner_above_walls_m'))
        half = 0.5 * float(gp('banner_width_m'))
        self.banner = Banner(x=0.0, y_min=-half, y_max=half, z_bottom=bottom,
                             z_top=bottom + float(gp('banner_height_m')))
        self.get_logger().info(
            f'[sitl_corridor_sim] green banner on corridor1: {2 * half:.2f} m wide, '
            f'{bottom:.2f}-{self.banner.z_top:.2f} m')
        self.offset = float(gp('gps_offset_m'))
        self.drift = float(gp('gps_drift_m'))
        self.drift_period = max(1.0, float(gp('gps_drift_period_s')))
        self._t0 = time.monotonic()
        self._pass = {}                 # corridor name -> [lateral offsets] while inside
        self.strip = float(gp('marker_strip_m'))
        k = float(gp('render_scale'))
        c = Camera()
        self.cam = Camera(fx=c.fx * k, fy=c.fy * k, cx=c.cx * k, cy=c.cy * k,
                          width=int(c.width * k), height=int(c.height * k))
        # Colour at the OAK's full 640x360 (~60 ms), NOT render_scale: the
        # banner detector's filters are in pixels (at half size its edge test
        # rejects the banner from 10 m).
        self.colour_cam = COLOUR_CAMERA

        self._gps = self._alt = self._yaw = None
        self._pitch = self._roll = 0.0
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 lambda m: setattr(self, '_gps', (m.latitude, m.longitude)), best)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 lambda m: setattr(self, '_alt', m.data), best)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg',
                                 lambda m: setattr(self, '_yaw', math.radians(90.0 - m.data)), best)
        self.create_subscription(Imu, '/mavros/imu/data', self._on_imu, best)
        self.depth_pub = self.create_publisher(Image, '/oak/stereo/image_raw', best)
        self.info_pub = self.create_publisher(CameraInfo, '/oak/stereo/camera_info', best)
        self.rgb_pub = self.create_publisher(Image, '/oak/rgb/image_raw', best)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/oak/rgb/camera_info', best)
        self.marker_pub = self.create_publisher(String, '/corridor/ground_marker', 10)
        self.create_timer(1.0 / float(gp('depth_rate_hz')), self._tick)

    def _on_imu(self, m):
        q = m.orientation
        self._pitch = g.pitch_from_quaternion(q.x, q.y, q.z, q.w)
        self._roll = g.roll_from_quaternion(q.x, q.y, q.z, q.w)

    def _corridor_pose(self, c):
        """Vehicle (x along from the mouth, y left, yaw rel.) in corridor c,
        with the deliberate EKF offset applied."""
        d_n, d_e = mav.ne_offset(*c['first'], *self._gps)
        a_n, a_e = c['axis']
        x = d_n * a_n + d_e * a_e - c['mouth']
        drift = self.drift * math.sin(2 * math.pi * (time.monotonic() - self._t0)
                                      / self.drift_period)
        y = d_n * a_e - d_e * a_n + self.offset + drift
        return x, y, math.atan2(math.sin(self._yaw - c['yaw']), math.cos(self._yaw - c['yaw']))

    def _tick(self):
        if self._gps is None or self._alt is None or self._yaw is None:
            return
        poses = [(c, *self._corridor_pose(c)) for c in self.corridors]
        c, x, y, yaw = min(poses, key=lambda p: abs(p[2]) + max(0.0, -p[1] - 6.0)
                           + max(0.0, p[1] - p[0]['length'] - 6.0))
        near = abs(y) < 6.0 and -8.0 < x < c['length'] + 8.0
        scene = c['scene'] if near else Scene(x_start=1e3, x_end=1e3 + 1.0)
        z_cam = max(self._alt, 0.05)
        z = render(scene, self.cam, (x, y, z_cam), yaw, self._pitch, self._roll)
        if near and c is self.corridors[0]:
            # Plain cloth: no stereo depth on the banner, and none through it.
            tb = banner_depth(self.banner, self.cam, (x, y, z_cam), yaw, self._pitch, self._roll)
            z[tb < np.nan_to_num(z, nan=np.inf)] = np.nan
        # Colour: corridor 1 (the banner's) from wherever the vehicle is.
        c1, x1, y1, yaw1 = poses[0]
        bgr = render_colour(c1['scene'], self.banner, self.colour_cam, (x1, y1, z_cam), yaw1,
                            self._pitch, self._roll)
        stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(self._info(stamp, self.cam, 'oak_forward'))
        self.depth_pub.publish(self._image(z, stamp))
        self.rgb_info_pub.publish(self._info(stamp, self.colour_cam, 'oak_rgb'))
        self.rgb_pub.publish(self._bgr(bgr, stamp))

        self._track_pass(c, x, y)
        marker = 'none'
        if near and abs(y) < c['scene'].width / 2 + 0.5:
            if -self.strip <= x <= 0.0:
                marker = 'green'
            elif c['length'] <= x <= c['length'] + self.strip:
                marker = 'blue'
        self.marker_pub.publish(String(data=marker))

    def _track_pass(self, c, x, y):
        """Ground truth: true lateral offset while between the walls; log
        it when the vehicle leaves them."""
        half = c['scene'].width / 2
        inside = 0.0 <= x <= c['length'] and abs(y) < half
        offs = self._pass.setdefault(c['name'], [])
        if inside:
            offs.append(y)
            self.get_logger().info(
                f"[sitl_corridor_sim] {c['name']} at {x:4.1f} m: true centre offset "
                f"{y:+.2f} m", throttle_duration_sec=0.5)
        elif offs:
            rms = math.sqrt(sum(v * v for v in offs) / len(offs))
            worst = max(offs, key=abs)
            self.get_logger().info(
                f"[sitl_corridor_sim] {c['name']} pass: centre offset max {abs(worst):.2f} m "
                f"({'left' if worst > 0 else 'right'}), RMS {rms:.2f} m over {len(offs)} "
                f"frames; closest wall {half - abs(worst):.2f} m from the centre point "
                f"(corridor {2 * half:.2f} m wide)")
            self._pass[c['name']] = []

    def _info(self, stamp, cam, frame_id):
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, frame_id
        m.width, m.height = cam.width, cam.height
        m.k = [cam.fx, 0.0, cam.cx, 0.0, cam.fy, cam.cy, 0.0, 0.0, 1.0]
        return m

    def _bgr(self, img, stamp):
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, 'oak_rgb'
        m.height, m.width = img.shape[:2]
        m.encoding, m.is_bigendian = 'bgr8', 0
        m.step = img.shape[1] * 3
        m.data = np.ascontiguousarray(img).tobytes()
        return m

    def _image(self, z, stamp):
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, 'oak_forward'
        m.height, m.width = z.shape
        m.encoding, m.is_bigendian = '32FC1', 0
        m.step = z.shape[1] * 4
        m.data = np.ascontiguousarray(z, dtype=np.float32).tobytes()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = SitlCorridorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
