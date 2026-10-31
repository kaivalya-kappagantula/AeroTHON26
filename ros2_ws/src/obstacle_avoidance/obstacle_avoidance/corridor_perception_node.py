# corridor_perception_node.py — corridor walls, centring error, obstacles,
# entrance and exit from the forward OAK-D depth.
#
# Runs beside sector_builder_node on the same depth topic. sector_builder feeds
# ArduPilot's proximity map (OBSTACLE_DISTANCE; used by simple avoidance when
# the pilot flies LOITER). This node measures the corridor for path_planner,
# which keeps the vehicle centred and picks the side (or height) to pass an
# obstacle. It only publishes measurements; it commands nothing.
#
#   in : ~/depth, ~/camera_info   (remapped to the OAK driver, like the builder)
#        ~/rgb, ~/rgb_camera_info (OAK colour stream; optional — without it
#                                  the walls come from depth only)
#        /mavros/imu/data         (pitch + roll; the slice tilts with the vehicle)
#        /mavros/global_position/rel_alt (ground cut, RGB wall-base scale, and
#                                  cam_alt in the output)
#        /mavros/global_position/compass_hdg (cam_yaw in the output)
#   out: /corridor/perception     (Float32MultiArray, layout in
#                                  mission_common/corridor_msg.py)
#
# Processing runs on a timer at rate_hz on the newest frame, so a slow Pi 5
# frame drops stale depth instead of queueing it. A message is published for
# every processed frame, walls = 0 when no corridor is in view, so the consumer
# can tell "no corridor" (message, walls 0) from "node dead" (no messages).
#
# Corridor WIDTH is not configured (corridors differ): walls are paired when
# they are corridor_width_min_m..max_m apart, and every two-wall fit is a
# width measurement. The median of the last corridor_width_window of them
# places the unseen wall when only one is in view, and from then on only
# pairs within corridor_width_tol_m of it are walls (squeezed between a wall
# and an obstacle, the obstacle's side would pass as the other wall); until a pair has been seen
# a single wall gives no fit — unless corridor_width_prior_m is set (the
# competition corridors are 11.5 ft): then one wall gives a fit from the
# first frame, and pairs must be within corridor_width_tol_m of the prior
# until measured. In an 11.5 ft corridor both walls are rarely in view at
# once (OAK-D bench: 81 of ~2000 frames), so without the prior most frames
# were "no corridor". With no wall fit at all for corridor_width_forget_s
# the measured width is forgotten (back to the prior, or unknown).
#
# Frames with no wall fit still report the nearest obstacle (in a band around
# the nose, rotated by the last fitted corridor direction:
# corridor_geometry.find_obstacle_unfitted) — the planner flies on GPS then,
# and an obstacle filling the view hides the walls exactly when it matters.
#
# Wall sources (corridor_walls_from): depth (stereo, needs texture: on white
# walls only the sponsor logos give depth), rgb (wall bases against the
# ground, floor_boundary.py; scaled by the camera height; a white board's
# base is not taken for a black wall's: test_floor_boundary), or auto: both every
# frame, the one that sees more walls wins (depth on a tie). Obstacles always
# come from depth, also between RGB-fitted walls. A wall end measured by one
# source is dropped if the other still sees both walls with no end in range
# (white walls: depth stops where a logo stops).
#
# Geometry in corridor_geometry.py (tested against ray-cast depth in
# test/test_corridor_geometry.py).

import collections
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Float32MultiArray, Float64

from mission_common import corridor_msg
from obstacle_avoidance import corridor_geometry as cg
from obstacle_avoidance import floor_boundary as fb
from obstacle_avoidance import geometry as g
from obstacle_avoidance.depth_image import (DepthDecodeError, decode_bgr, decode_depth,
                                            scale_intrinsics, scaled_stride, shrink_bgr)

TOPIC = '/corridor/perception'


class CorridorPerceptionNode(Node):

    def __init__(self):
        super().__init__('corridor_perception_node')
        d = cg.CorridorParams()
        self.declare_parameter('corridor_rate_hz', 5.0)
        self.declare_parameter('corridor_stride_px', 4)
        self.declare_parameter('camera_pitch_offset_deg', 0.0)
        self.declare_parameter('pitch_compensation', True)
        self.declare_parameter('corridor_width_min_m', d.width_min_m)
        self.declare_parameter('corridor_width_max_m', d.width_max_m)
        self.declare_parameter('corridor_width_tol_m', d.width_tol_m)
        self.declare_parameter('corridor_path_half_width_m', d.path_half_width_m)
        self.declare_parameter('corridor_width_window', 15)
        self.declare_parameter('corridor_width_forget_s', 5.0)
        self.declare_parameter('corridor_width_prior_m', 0.0)        # 0 = none
        self.declare_parameter('corridor_max_single_offset_m', d.max_single_offset_m)
        self.declare_parameter('corridor_unfitted_margin_m', d.unfitted_margin_m)
        self.declare_parameter('corridor_slice_lo_m', d.slice_lo_m)
        self.declare_parameter('corridor_slice_hi_m', d.slice_hi_m)
        self.declare_parameter('corridor_max_range_m', d.max_range_m)
        self.declare_parameter('corridor_obstacle_range_m', d.obstacle_range_m)
        self.declare_parameter('corridor_obstacle_depth_m', d.obstacle_depth_m)
        self.declare_parameter('corridor_walls_from', 'auto')
        self.declare_parameter('corridor_rgb_floor_thr', 16.0)
        self.declare_parameter('corridor_rgb_min_wall_dist_m', 0.375)
        self.declare_parameter('rgb_process_width', 640)
        gp = lambda n: self.get_parameter(n).value          # noqa: E731
        self.params = cg.CorridorParams(
            width_min_m=float(gp('corridor_width_min_m')),
            width_max_m=float(gp('corridor_width_max_m')),
            width_tol_m=float(gp('corridor_width_tol_m')),
            path_half_width_m=float(gp('corridor_path_half_width_m')),
            slice_lo_m=float(gp('corridor_slice_lo_m')),
            slice_hi_m=float(gp('corridor_slice_hi_m')),
            max_range_m=float(gp('corridor_max_range_m')),
            obstacle_range_m=float(gp('corridor_obstacle_range_m')),
            obstacle_depth_m=float(gp('corridor_obstacle_depth_m')),
            max_single_offset_m=float(gp('corridor_max_single_offset_m')),
            unfitted_margin_m=float(gp('corridor_unfitted_margin_m')),
        )
        prior = float(gp('corridor_width_prior_m'))
        self.width_prior = prior if prior > 0.0 else None
        self.stride = int(gp('corridor_stride_px'))
        self.pitch_offset = math.radians(float(gp('camera_pitch_offset_deg')))
        self.pitch_compensation = bool(gp('pitch_compensation'))
        self.walls_from = str(gp('corridor_walls_from'))
        if self.walls_from not in ('auto', 'depth', 'rgb'):
            raise ValueError("corridor_walls_from must be auto, depth or rgb")
        self.floor_thr = float(gp('corridor_rgb_floor_thr'))
        self.rgb_min_wall = float(gp('corridor_rgb_min_wall_dist_m'))
        self.rgb_width = int(gp('rgb_process_width'))
        self.width_forget_s = float(gp('corridor_width_forget_s'))
        self._widths = collections.deque(maxlen=int(gp('corridor_width_window')))
        self._last_walls_s = None   # last frame with any wall fit
        self._axis = None           # (world axis ENU rad, s) of the last wall fit

        self._K = None              # (fx, fy, cx, cy, width)
        self._fov = None
        self._depth_msg = None      # (newest unprocessed frame, alt at arrival)
        self._K_rgb = None          # colour (fx, fy, cx, cy, width, height)
        self._fb = None             # FloorBoundary, built with the colour intrinsics
        self._fov_rgb = None
        self._rgb_msg = None        # (newest colour frame, arrival s)
        self._pitch = self._roll = 0.0
        self._pitch_stamp_s = None
        self._alt = None            # (rel_alt m, stamp s)
        self._yaw = None            # (ENU yaw rad, stamp s)
        self._logged_error = False
        self._logged_rgb_error = False

        self.pub = self.create_publisher(
            Float32MultiArray, TOPIC,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=5))
        self.create_subscription(Image, '~/depth', self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '~/camera_info', self._on_info,
                                 qos_profile_sensor_data)
        if self.walls_from != 'depth':
            self.create_subscription(
                Image, '~/rgb', lambda m: setattr(self, '_rgb_msg', (m, self._now_s())),
                qos_profile_sensor_data)
            self.create_subscription(CameraInfo, '~/rgb_camera_info', self._on_rgb_info,
                                     qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self._on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            lambda m: setattr(self, '_alt', (float(m.data), self._now_s())),
            qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg',
            lambda m: setattr(self, '_yaw', (math.radians(90.0 - float(m.data)),
                                             self._now_s())),
            qos_profile_sensor_data)
        self.create_timer(1.0 / float(gp('corridor_rate_hz')), self._on_timer)
        self.get_logger().info(
            f'corridor_perception_node up: width measured '
            f'({self.params.width_min_m:.2f}..{self.params.width_max_m:.2f} m accepted), '
            f'prior {self.width_prior or 0.0:.2f} m (0 = none), '
            f'walls from {self.walls_from}, '
            f'slice {self.params.slice_lo_m:+.2f}..{self.params.slice_hi_m:+.2f} m, '
            f'range {self.params.max_range_m:.1f} m -> {TOPIC}')

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, msg):
        if self._K is not None or msg.k[0] == 0.0 or msg.k[4] == 0.0:
            return                                   # latch the first valid
        self._K = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]),
                   float(msg.k[5]), int(msg.width))
        self._fov = cg.view_limits(self._K[0], self._K[1], self._K[2],
                                   self._K[3], self._K[4])
        self.get_logger().info(
            'Latched intrinsics fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} '
            'W={}; HFOV L/R {:.1f}/{:.1f} deg'.format(
                *self._K, math.degrees(self._fov[0]), math.degrees(self._fov[1])))

    def _on_rgb_info(self, msg):
        if self._K_rgb is not None or msg.k[0] == 0.0 or msg.k[4] == 0.0:
            return                                   # latch the first valid
        self._K_rgb = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]),
                       float(msg.k[5]), int(msg.width), int(msg.height))
        self.get_logger().info(
            'Latched colour intrinsics fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} '
            '{}x{} — RGB wall bases on'.format(*self._K_rgb))

    def _rgb_fitter(self, msg, w, h):
        """FloorBoundary + FOV for w x h (shrunk) colour frames (built once).
        The intrinsics are scaled to them; camera_info that does not match the
        driver's frames (msg) is warned about."""
        if self._fb is None:
            iw, ih = self._K_rgb[4:]
            if (iw, ih) != (msg.width, msg.height):
                self.get_logger().warn(
                    f'colour camera_info is {iw}x{ih} but frames are '
                    f'{msg.width}x{msg.height}: scaling the intrinsics — check the OAK rgb params')
            K = (*scale_intrinsics(*self._K_rgb, w, h), w)
            self._fb = fb.FloorBoundary(K, thr=self.floor_thr)
            self._fov_rgb = cg.view_limits(*K)
        return self._fb, self._fov_rgb

    def _width(self, now):
        """Measured corridor width (median of recent wall pairs), else the
        prior (None if neither)."""
        if self._last_walls_s is None or now - self._last_walls_s > self.width_forget_s:
            self._widths.clear()
        return statistics.median(self._widths) if self._widths else self.width_prior

    def _on_imu(self, msg):
        q = msg.orientation
        self._pitch = g.pitch_from_quaternion(q.x, q.y, q.z, q.w)
        self._roll = g.roll_from_quaternion(q.x, q.y, q.z, q.w)
        self._pitch_stamp_s = self._now_s()

    def _fresh(self, item):
        if item is not None and self._now_s() - item[1] < 1.0:
            return item[0]
        return float('nan')

    def _on_depth(self, msg):
        # Altitude and attitude are latched with the frame: the vehicle keeps
        # moving while the frame waits for the timer.
        self._depth_msg = (msg, self._fresh(self._alt), self._fresh(self._yaw),
                           self._pitch, self._roll, self._pitch_stamp_s)

    def _on_timer(self):
        item, self._depth_msg = self._depth_msg, None
        if item is None or self._K is None:
            return
        msg, alt, yaw, pitch_raw, roll_raw, att_stamp = item
        try:
            z = decode_depth(msg)
        except DepthDecodeError as e:
            if not self._logged_error:
                self.get_logger().error(f'{e}')
                self._logged_error = True
            return
        fresh = att_stamp is not None and self._now_s() - att_stamp < 0.5
        use = self.pitch_compensation and fresh
        pitch = (pitch_raw - self.pitch_offset) if use else 0.0
        roll = roll_raw if use else 0.0
        if self.pitch_compensation and not fresh:
            self.get_logger().warn(
                'no fresh /mavros/imu/data — corridor fit assumes level camera',
                throttle_duration_sec=5.0)
        fx, fy, cx, cy, _ = self._K
        x, y, h = cg.level_points(z, fx, fy, cx, cy, pitch,
                                  scaled_stride(self.stride, z.shape[1]), roll)
        left, right, up = self._fov
        # Height above ground cuts the ground out of the slice (flat ground at
        # home level assumed; ignored if unknown or implausibly low).
        hag = alt if alt > 0.3 else None            # NaN > 0.3 is False
        now = self._now_s()
        width = self._width(now)
        fov = (left, right, up + pitch)
        r_d = cg.fit_corridor(x, y, h, self.params, fov=fov, height_above_ground=hag,
                              width=width) \
            if self.walls_from != 'rgb' else fb.no_fit(self.params)
        r_c = self._fit_rgb(pitch, roll, hag, width, now)
        r, source = r_d, 'depth'
        if r_c is not None:
            if self.walls_from == 'rgb' or r_c['walls'] > r_d['walls']:
                cg.find_obstacle(r_c, x, y, h, self.params, fov, hag)
                r, source = r_c, 'rgb'
            other = r_c if r is r_d else r_d
            if r['walls'] and other['walls'] == 2 and math.isnan(other['walls_end']) and \
                    abs(_centre(other) - _centre(r)) < 0.3:
                r['walls_end'] = float('nan')       # the other source sees the walls go on
        if r['walls']:
            self._last_walls_s = now
            if r['walls'] == 2:
                self._widths.append(r['wall_left'] - r['wall_right'])
            if not math.isnan(yaw):
                self._axis = (yaw + r['heading_err'], now)
        else:
            # No wall fit: obstacles in a band around the nose, turned by the
            # last fitted corridor direction (nose = corridor if none / old).
            heading = 0.0
            if self._axis is not None and now - self._axis[1] < 10.0 and not math.isnan(yaw):
                a = self._axis[0] - yaw
                heading = math.atan2(math.sin(a), math.cos(a))
                if abs(heading) > math.radians(20.0):
                    heading = 0.0
            cg.find_obstacle_unfitted(r, x, y, h, self.params, width, heading, fov, hag)
        r['slice_lo'] = self.params.slice_lo_m
        r['path_dist'] = cg.path_distance(x, y, h, self.params, hag)
        r['cam_alt'] = alt
        r['cam_yaw'] = yaw
        self.pub.publish(corridor_msg.encode(r))
        if r['walls']:
            w = self._width(now)
            self.get_logger().info(
                f"[{source}] walls={r['walls']} L={r['wall_left']:+.2f} R={r['wall_right']:+.2f} "
                f"width={'?' if w is None else f'{w:.2f}'}m "
                f"hdg={math.degrees(r['heading_err']):+.1f}deg "
                f"obs={r['obs_dist']:.2f}m [{r['obs_right']:+.2f},{r['obs_left']:+.2f}] "
                f"top={r['obs_top']:+.2f} mouth={r['mouth_dist']:.2f} "
                f"end={r['walls_end']:.2f}",
                throttle_duration_sec=1.0)

    def _fit_rgb(self, pitch, roll, hag, width, now):
        """Corridor fit from the colour wall bases, or None (off, no colour
        frame/intrinsics yet, or height unknown — it scales the distances)."""
        if self.walls_from == 'depth' or self._K_rgb is None or hag is None:
            return None
        item = self._rgb_msg
        if item is None or now - item[1] > 0.5:
            return None
        try:
            bgr = decode_bgr(item[0])
        except DepthDecodeError as e:
            if not self._logged_rgb_error:
                self.get_logger().error(f'{e}')
                self._logged_rgb_error = True
            return None
        bgr = shrink_bgr(bgr, self.rgb_width)
        fitter, fov_rgb = self._rgb_fitter(item[0], bgr.shape[1], bgr.shape[0])
        r, _, _ = fb.fit_rgb(fitter, bgr, self.params, fov_rgb, pitch, roll, hag,
                             width=width, min_wall_dist_m=self.rgb_min_wall)
        return r


def _centre(r):
    return 0.5 * (r['wall_left'] + r['wall_right'])


def main(args=None):
    rclpy.init(args=args)
    node = CorridorPerceptionNode()
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
