#!/usr/bin/env python3
"""Live colorised depth view with the sector bins drawn on top.

This is the "what is the camera actually seeing" window. rqt_image_view is no
use here: the depth topic is 16UC1 millimetres, so it renders as near-black, and
/avoidance/debug_band is a few pixels tall. This colourises depth over the range
the builder actually reports, then overlays the bin grid and the scan result so
you can see an object and the bin it landed in at the same time.

Bearings follow the package convention (REP-103, CCW positive): +30 deg is LEFT
of the camera. On screen, left of centre IS +30 deg — the image is drawn as seen
from behind the camera, so what you see matches what you'd point at.

    python3 scripts/depth_view.py

Keys: q or ESC to quit.  Run it alongside the driver and sector_builder_hw.
"""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32

DEPTH_TOPIC = '/oak/stereo/image_raw'
INFO_TOPIC = '/oak/stereo/camera_info'

# Colour ramp limits, metres. Matches range_min_m / range_max_report_m in
# avoidance.yaml so the picture and the scan agree about what "too far" means.
CLIP_MIN_M = 0.40
CLIP_MAX_M = 6.0

# Bin grid is drawn every this many degrees (bin_width_deg in avoidance.yaml).
BIN_DEG = 5.0
LABEL_EVERY_DEG = 15.0

# A scan is considered stale after this long with no message. Deliberately
# shorter than the builder's own watchdog so the window says STALE before the
# scan stops arriving entirely.
SCAN_STALE_S = 0.5


class DepthView(Node):
    def __init__(self):
        super().__init__('depth_view')
        self._bridge = CvBridge()
        self._fx = None
        self._cx = None
        self._scan = None
        self._scan_t = 0.0
        self._coverage = float('nan')
        self._depth_t = 0.0
        self._frames = 0
        self._fps = 0.0
        self._fps_t0 = time.monotonic()

        self.create_subscription(CameraInfo, INFO_TOPIC, self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/avoidance/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32, '/avoidance/coverage', self._on_cov,
                                 qos_profile_sensor_data)

        cv2.namedWindow('depth view', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('depth view', 960, 720)
        # Rendering is driven by the depth callback, so with no driver running
        # the window would stay blank and look like a broken viewer. This timer
        # paints an explicit "no driver" screen instead.
        self.create_timer(0.5, self._tick_idle)
        self.get_logger().info(f'depth_view up, waiting for {DEPTH_TOPIC}')

    def _tick_idle(self):
        """Paint a diagnostic screen whenever depth has stopped arriving."""
        age = time.monotonic() - self._depth_t
        if self._depth_t > 0.0 and age < 1.0:
            return  # depth is flowing; the real render owns the window

        panel = np.zeros((480, 640, 3), np.uint8)
        if self._depth_t == 0.0:
            # ASCII only: cv2's Hershey fonts render anything else as '?'.
            head, colour = 'NO DEPTH - is the driver running?', (0, 165, 255)
            hint = 'ros2 launch depthai_ros_driver camera.launch.py \\'
        else:
            head, colour = f'DEPTH LOST {age:.0f}s ago', (0, 0, 255)
            hint = 'check: lsusb | grep 03e7   (2485 = dead, f63b = alive)'
        cv2.putText(panel, head, (20, 12 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    colour, 2)
        cv2.putText(panel, f'topic: {DEPTH_TOPIC}', (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        cv2.putText(panel, hint, (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (180, 180, 180), 1)
        if self._depth_t == 0.0:
            cv2.putText(panel, '  params_file:=<share>/config/oak_d_lite_depth.yaml',
                        (20, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (180, 180, 180), 1)
        cv2.imshow('depth view', panel)
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            raise KeyboardInterrupt

    def _on_info(self, msg):
        self._fx = msg.k[0]
        self._cx = msg.k[2]

    def _on_scan(self, msg):
        self._scan = msg
        self._scan_t = time.monotonic()

    def _on_cov(self, msg):
        self._coverage = msg.data

    def _on_depth(self, msg):
        self._depth_t = time.monotonic()
        self._frames += 1
        now = time.monotonic()
        if now - self._fps_t0 >= 1.0:
            self._fps = self._frames / (now - self._fps_t0)
            self._frames = 0
            self._fps_t0 = now

        raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if msg.encoding == '16UC1':
            metres = raw.astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            metres = raw.astype(np.float32)
        else:
            self.get_logger().warn(f'unsupported encoding {msg.encoding}',
                                   throttle_duration_sec=5.0)
            return

        self._render(metres)

    def _render(self, metres):
        # Zero means "no return" on OAK depth, not "zero metres" — colouring it
        # like a very close obstacle would paint the whole frame red.
        invalid = (metres <= 0.0) | ~np.isfinite(metres)
        clipped = np.clip(metres, CLIP_MIN_M, CLIP_MAX_M)
        norm = (clipped - CLIP_MIN_M) / (CLIP_MAX_M - CLIP_MIN_M)
        # Invert so near = red, far = blue, which is the intuition people have.
        vis = cv2.applyColorMap(((1.0 - norm) * 255).astype(np.uint8),
                                cv2.COLORMAP_TURBO)
        vis[invalid] = (40, 40, 40)  # dark grey = no depth data

        h, w = vis.shape[:2]
        self._draw_bins(vis, w, h)
        self._draw_hud(vis, w, h)

        cv2.imshow('depth view', vis)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            raise KeyboardInterrupt

    def _draw_bins(self, vis, w, h):
        """Vertical lines at bin edges, using the real intrinsics."""
        if self._fx is None:
            return
        deg = -90.0
        while deg <= 90.0:
            # +deg is LEFT in REP-103; image column increases to the RIGHT.
            u = int(round(self._cx - self._fx * math.tan(math.radians(deg))))
            if 0 <= u < w:
                labelled = abs(deg % LABEL_EVERY_DEG) < 1e-6
                shade = 200 if labelled else 90
                cv2.line(vis, (u, 0), (u, h), (shade, shade, shade), 1)
                if labelled:
                    cv2.putText(vis, f'{deg:+.0f}', (u + 3, h - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            deg += BIN_DEG

        # Centreline last so it sits on top of the grid.
        u0 = int(round(self._cx))
        if 0 <= u0 < w:
            cv2.line(vis, (u0, 0), (u0, h), (255, 255, 255), 1)

    def _draw_hud(self, vis, w, h):
        cv2.rectangle(vis, (0, 0), (w, 54), (0, 0, 0), -1)

        stale = (time.monotonic() - self._scan_t) > SCAN_STALE_S
        if self._scan is None:
            scan_txt = 'scan: none yet'
            colour = (120, 120, 120)
        elif stale:
            age = time.monotonic() - self._scan_t
            scan_txt = f'scan: STALE {age:4.1f}s'
            colour = (0, 0, 255)
        else:
            best_r, best_deg = float('inf'), None
            for i, r in enumerate(self._scan.ranges):
                if math.isfinite(r) and r < best_r:
                    best_r = r
                    best_deg = math.degrees(
                        self._scan.angle_min + i * self._scan.angle_increment)
            if best_deg is None:
                scan_txt = 'scan: all inf (nothing in range)'
                colour = (0, 200, 255)
            else:
                side = 'LEFT' if best_deg > 2 else ('RIGHT' if best_deg < -2
                                                    else 'AHEAD')
                scan_txt = f'nearest {best_r:4.2f} m @ {best_deg:+5.1f} deg {side}'
                colour = (0, 255, 0)
                # Mark the winning bin on the image.
                if self._fx is not None:
                    u = int(round(self._cx
                                  - self._fx * math.tan(math.radians(best_deg))))
                    if 0 <= u < w:
                        cv2.line(vis, (u, 54), (u, h), (0, 255, 0), 2)

        cv2.putText(vis, scan_txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    colour, 2)
        cov = f'{self._coverage:4.2f}' if not math.isnan(self._coverage) else ' -- '
        cv2.putText(vis, f'depth {self._fps:4.1f} Hz   coverage {cov}   '
                         f'near=red far=blue  grey=no data',
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
