# banner_node.py — green corridor-entrance banner from the forward OAK-D
# colour camera.
#
#   in : ~/rgb, ~/rgb_camera_info  (remapped to the OAK colour stream)
#        ~/depth, ~/camera_info    (OAK stereo depth: distance to the banner)
#        /mavros/imu/data          (pitch + roll: angles in the LEVEL frame)
#        /mavros/global_position/rel_alt, /mavros/global_position/compass_hdg
#   out: /corridor/banner          (Float32MultiArray, layout in
#                                   mission_common/banner_msg.py)
#        /corridor/banner/debug    (Image bgr8, only with banner_debug_image)
#
# Two corridors stand side by side; the green banner across the top of one
# entrance says which to enter. path_planner (entry corridor) searches for it,
# flies toward it, measures its height (bottom edge) to fly at the middle of
# the opening, and checks the walls it lines up on are the ones under it.
#
# Detection and tracking: banner.py (green_detect.py's, same defaults; the
# banner_* parameters in avoidance.yaml). The newest colour frame is processed
# on a timer at banner_rate_hz; a message is published for every processed
# frame, state 0 when there is no banner, so the consumer can tell "no banner"
# (message, state 0) from "node dead" (no messages).
#
# Distance: depth points whose bearing/elevation fall inside the central part
# of the banner box; median horizontal distance if there are at least
# banner_depth_min_points. Plain cloth gives little stereo depth, so NaN is
# normal; path_planner then uses where the walls begin instead. (The colour
# and depth cameras are 3.75 cm apart on the OAK-D Lite: negligible here.)

import math
from dataclasses import fields

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Float32MultiArray, Float64

from mission_common import banner_msg
from obstacle_avoidance import corridor_geometry as cg
from obstacle_avoidance import geometry as g
from obstacle_avoidance.banner import BannerParams, Detector, Tracker
from obstacle_avoidance.depth_image import DepthDecodeError, decode_bgr, decode_depth

TOPIC = '/corridor/banner'
DEBUG_TOPIC = '/corridor/banner/debug'
STATE = {'HOLD': banner_msg.HOLD, 'LOCK': banner_msg.LOCK}


class BannerNode(Node):

    def __init__(self):
        super().__init__('banner_node')
        d = BannerParams()
        self.declare_parameter('banner_rate_hz', 15.0)
        for f in fields(BannerParams):
            self.declare_parameter(f'banner_{f.name}', getattr(d, f.name))
        self.declare_parameter('banner_depth_min_points', 20)
        self.declare_parameter('banner_debug_image', False)
        self.declare_parameter('corridor_stride_px', 4)
        self.declare_parameter('camera_pitch_offset_deg', 0.0)
        self.declare_parameter('pitch_compensation', True)
        gp = lambda n: self.get_parameter(n).value          # noqa: E731
        self.params = BannerParams(**{
            f.name: type(getattr(d, f.name))(gp(f'banner_{f.name}')) for f in fields(BannerParams)})
        self.detector = Detector(self.params)
        self.tracker = Tracker(self.params)
        self.min_points = int(gp('banner_depth_min_points'))
        self.debug = bool(gp('banner_debug_image'))
        self.stride = int(gp('corridor_stride_px'))
        self.pitch_offset = math.radians(float(gp('camera_pitch_offset_deg')))
        self.pitch_compensation = bool(gp('pitch_compensation'))

        self._K = None              # colour (fx, fy, cx, cy, width, height) from camera_info
        self._K_depth = None        # depth (fx, fy, cx, cy, width)
        self._rgb_msg = None        # newest unprocessed colour frame + state at arrival
        self._depth_msg = None      # (newest depth frame, arrival s)
        self._pitch = self._roll = 0.0
        self._att_stamp_s = None
        self._alt = None            # (rel_alt m, stamp s)
        self._yaw = None            # (ENU yaw rad, stamp s)
        self._warned = set()

        self.pub = self.create_publisher(
            Float32MultiArray, TOPIC,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=5))
        self.debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 1) if self.debug else None
        self.create_subscription(Image, '~/rgb', self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '~/rgb_camera_info', self._on_rgb_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '~/depth',
                                 lambda m: setattr(self, '_depth_msg', (m, self._now_s())),
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '~/camera_info', self._on_depth_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/mavros/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            lambda m: setattr(self, '_alt', (float(m.data), self._now_s())),
            qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg',
            lambda m: setattr(self, '_yaw', (math.radians(90.0 - float(m.data)),
                                             self._now_s())),
            qos_profile_sensor_data)
        self.create_timer(1.0 / float(gp('banner_rate_hz')), self._on_timer)
        p = self.params
        self.get_logger().info(
            f'banner_node up: H[{p.h_min},{p.h_max}] S>{p.s_min} V>{p.v_min} '
            f'pale a*<-{p.a_min} V>{p.v_floor}, min area {p.min_area:.3f} -> {TOPIC}')

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(text)

    def _on_rgb_info(self, msg):
        if self._K is not None or msg.k[0] == 0.0 or msg.k[4] == 0.0:
            return                                   # latch the first valid
        self._K = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]),
                   int(msg.width), int(msg.height))
        self.get_logger().info(
            'Colour intrinsics fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} {}x{}'.format(*self._K))

    def _on_depth_info(self, msg):
        if self._K_depth is not None or msg.k[0] == 0.0 or msg.k[4] == 0.0:
            return
        self._K_depth = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]),
                         float(msg.k[5]), int(msg.width))

    def _on_imu(self, msg):
        q = msg.orientation
        self._pitch = g.pitch_from_quaternion(q.x, q.y, q.z, q.w)
        self._roll = g.roll_from_quaternion(q.x, q.y, q.z, q.w)
        self._att_stamp_s = self._now_s()

    def _fresh(self, item):
        if item is not None and self._now_s() - item[1] < 1.0:
            return item[0]
        return float('nan')

    def _on_rgb(self, msg):
        # Altitude and attitude are latched with the frame (as in
        # corridor_perception_node): the vehicle keeps moving meanwhile.
        self._rgb_msg = (msg, self._now_s(), self._fresh(self._alt), self._fresh(self._yaw),
                         self._pitch, self._roll, self._att_stamp_s)

    def _intrinsics(self, w, h):
        """Colour intrinsics for a w x h frame. camera_info that does not match
        the image (the OAK driver has published 720p intrinsics with smaller
        images before) is scaled to it, with a warning."""
        fx, fy, cx, cy, iw, ih = self._K
        if (iw, ih) != (w, h):
            self._warn_once('size', f'colour camera_info is {iw}x{ih} but frames are {w}x{h}: '
                                    'scaling the intrinsics — check the OAK rgb params')
            sx, sy = w / iw, h / ih
            return fx * sx, fy * sy, (cx + 0.5) * sx - 0.5, (cy + 0.5) * sy - 0.5
        return fx, fy, cx, cy

    def _on_timer(self):
        item, self._rgb_msg = self._rgb_msg, None
        if item is None:
            return
        if self._K is None:
            self._warn_once('info', 'no colour camera_info yet — banner angles unknown, waiting')
            return
        msg, t_frame, alt, yaw, pitch_raw, roll_raw, att_stamp = item
        try:
            bgr = decode_bgr(msg)
        except DepthDecodeError as e:
            self._warn_once('decode', f'{e}')
            return
        fresh = att_stamp is not None and t_frame - att_stamp < 0.5
        use = self.pitch_compensation and fresh
        pitch = (pitch_raw - self.pitch_offset) if use else 0.0
        roll = roll_raw if use else 0.0
        if self.pitch_compensation and not fresh:
            self.get_logger().warn(
                'no fresh /mavros/imu/data — banner angles assume a level camera',
                throttle_duration_sec=5.0)

        h_img, w_img = bgr.shape[:2]
        cands, rejected, _ = self.detector.detect(bgr)
        det = self.tracker.update(cands, w_img, h_img)
        out = dict(state=banner_msg.NONE, cam_alt=alt, cam_yaw=yaw)
        if det is not None:
            out.update(self._measure(det, w_img, h_img, pitch, roll, t_frame))
        self.pub.publish(banner_msg.encode(out))
        if self.debug_pub is not None:
            self.debug_pub.publish(self._debug_image(bgr, det, cands, rejected, out, msg.header))
        if det is not None:
            self.get_logger().info(
                f"[banner] {det['state']} bearing {math.degrees(out['bearing']):+.1f} deg, "
                f"bottom {math.degrees(out['elev_bottom']):+.1f} deg, dist {out['dist']:.2f} m, "
                f"area {100 * out['area_frac']:.1f}%",
                throttle_duration_sec=1.0)

    def _measure(self, det, w_img, h_img, pitch, roll, t_frame):
        fx, fy, cx, cy = self._intrinsics(w_img, h_img)
        x0, y0, bw, bh = det['bbox']
        (ul, vl), (ur, vr) = det['bottom_left'], det['bottom_right']
        # centre, left edge, right edge, bottom edge, top edge, bottom corners
        us = [det['cx'], x0, x0 + bw, det['cx'], det['cx'], ul, ur]
        vs = [det['cy'], det['cy'], det['cy'], y0 + bh, y0, vl, vr]
        f, l, u = cg.level_rays(us, vs, fx, fy, cx, cy, pitch, roll)
        brg = np.arctan2(l, f)
        elev = np.arctan2(u, np.hypot(f, l))
        out = dict(state=STATE[det['state']], bearing=float(brg[0]),
                   bearing_left=float(brg[1]), bearing_right=float(brg[2]),
                   elev_bottom=float(elev[3]), elev_top=float(elev[4]),
                   elev_bottom_left=float(elev[5]), elev_bottom_right=float(elev[6]),
                   area_frac=float(det['area_frac']), dist=float('nan'))
        out['dist'] = self._depth_dist(brg[1], brg[2], elev[3], elev[4], pitch, roll, t_frame)
        # An end or edge on the image border is the view's, not the banner's.
        nan = float('nan')
        if det['cut_left']:
            out['bearing_left'] = out['elev_bottom_left'] = nan
        if det['cut_right']:
            out['bearing_right'] = out['elev_bottom_right'] = nan
        if det['edge']:
            out['elev_bottom'] = nan
            for key, v in (('elev_bottom_left', vl), ('elev_bottom_right', vr)):
                if v >= h_img - 2:
                    out[key] = nan
        if y0 <= 1:
            out['elev_top'] = nan
        return out

    def _depth_dist(self, b_left, b_right, e_bottom, e_top, pitch, roll, t_frame):
        """Median horizontal distance of the depth points inside the central
        half (across) and middle 60 % (up) of the banner box; NaN if too few."""
        item = self._depth_msg
        if item is None or self._K_depth is None or abs(item[1] - t_frame) > 0.5:
            return float('nan')
        try:
            z = decode_depth(item[0])
        except DepthDecodeError as e:
            self._warn_once('depth', f'{e}')
            return float('nan')
        fx, fy, cx, cy, _ = self._K_depth
        x, y, h = cg.level_points(z, fx, fy, cx, cy, pitch, self.stride, roll)
        rng = np.hypot(x, y)
        brg = np.arctan2(y, x)
        elev = np.arctan2(h, rng)
        db, de = 0.25 * (b_left - b_right), 0.2 * (e_top - e_bottom)
        m = (x > 0) & (brg <= b_left - db) & (brg >= b_right + db) & \
            (elev >= e_bottom + de) & (elev <= e_top - de)
        if np.count_nonzero(m) < self.min_points:
            return float('nan')
        return float(np.median(rng[m]))

    def _debug_image(self, bgr, det, cands, rejected, out, header):
        img = bgr.copy()
        h, w = img.shape[:2]
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        for (x, y, bw, bh), why in rejected:        # grey = failed a filter
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (160, 160, 160), 1)
            cv2.putText(img, why, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (160, 160, 160), 1)
        for c in cands:                             # thin green = candidates
            x, y, bw, bh = c['bbox']
            cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 200, 0), 1)
        if det is not None:
            x, y, bw, bh = (int(round(q)) for q in det['bbox'])
            col = (0, 0, 255) if det['state'] == 'LOCK' else (0, 165, 255)
            cv2.rectangle(img, (x, y), (x + bw, y + bh), col, 2)
            text = (f"{det['state']} brg {math.degrees(out['bearing']):+.1f} "
                    f"bottom {math.degrees(out['elev_bottom']):+.1f} deg dist {out['dist']:.2f} m")
        else:
            text = 'no banner'
        cv2.putText(img, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(img, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return Image(header=header, height=h, width=w, encoding='bgr8', is_bigendian=0,
                     step=w * 3, data=img.tobytes())


def main(args=None):
    rclpy.init(args=args)
    node = BannerNode()
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
