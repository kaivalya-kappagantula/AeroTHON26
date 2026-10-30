#!/usr/bin/env python3
"""Test 2 of docs/OA_EVALUATION.md: steady depth on the REAL obstacle
material at 1.5, 2.5 and 3.5 m. No planner works without this.

Put the obstacle (the competition material, face-on, centred in the view)
at a TAPED distance from the camera lens and run, once per distance:

    python3 scripts/depth_material_check.py --dist 1.5
    python3 scripts/depth_material_check.py --dist 2.5
    python3 scripts/depth_material_check.py --dist 3.5

Over --secs it measures two things:
  image   centre ROI of the depth image (--roi-deg wide, +-3 deg tall, around
          the image centre): valid-pixel fraction and median depth per frame
  scan    the /avoidance/scan bins within +-roi/2 of straight ahead — what
          ArduPilot is actually sent: fraction of scans with a finite range,
          and its median

PASS needs all of: scan finite in >= 95 % of scans, scan median within
max(0.10 m, 5 %) of --dist, frame-to-frame jitter (std of the per-frame
image medians) <= 0.02 m + 3 % of --dist, and image valid fraction >= the
builder's min_valid_px_fraction_per_bin (0.15) in >= 95 % of frames.
Run with the OAK driver (our params file) and sector_builder_hw.launch.py up.
"""

import argparse
import math
import statistics
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from obstacle_avoidance.depth_image import DepthDecodeError, decode_depth

MIN_VALID = 0.15               # avoidance.yaml min_valid_px_fraction_per_bin
RANGE = (0.40, 15.0)


class MaterialCheck(Node):
    def __init__(self, a):
        super().__init__('depth_material_check')
        self.a = a
        self.K = None
        self.frames = []          # (valid fraction, median or nan)
        self.scans = []           # nearest finite range in the forward bins, or inf
        self.create_subscription(CameraInfo, a.info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, a.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/avoidance/scan', self._on_scan,
                                 qos_profile_sensor_data)

    def _on_info(self, m):
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_depth(self, m):
        if self.K is None:
            return
        try:
            z = decode_depth(m)
        except DepthDecodeError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        fx, fy, cx, cy = self.K
        hw = fx * math.tan(math.radians(self.a.roi_deg / 2))
        hh = fy * math.tan(math.radians(3.0))
        roi = z[max(0, int(cy - hh)):int(cy + hh) + 1, max(0, int(cx - hw)):int(cx + hw) + 1]
        ok = np.isfinite(roi) & (roi >= RANGE[0]) & (roi <= RANGE[1])
        frac = float(ok.mean()) if roi.size else 0.0
        self.frames.append((frac, float(np.median(roi[ok])) if ok.any() else math.nan))

    def _on_scan(self, s):
        half = math.radians(self.a.roi_deg / 2)
        near = math.inf
        for i, r in enumerate(s.ranges):
            if abs(s.angle_min + i * s.angle_increment) <= half and math.isfinite(r):
                near = min(near, r)
        self.scans.append(near)

    def report(self):
        d = self.a.dist
        print(f"\n=== depth on the obstacle at {d:.2f} m ({len(self.frames)} depth frames, "
              f"{len(self.scans)} scans) ===")
        if not self.frames:
            print("no depth frames — is the driver up (and camera_info)?")
            return False
        fracs = [f for f, _ in self.frames]
        meds = [m for _, m in self.frames if math.isfinite(m)]
        frac_ok = sum(f >= MIN_VALID for f in fracs) / len(fracs)
        print(f"image: valid px mean {100 * statistics.mean(fracs):.0f} %, worst "
              f"{100 * min(fracs):.0f} %, frames >= {100 * MIN_VALID:.0f} %: {100 * frac_ok:.0f} %")
        jitter = statistics.pstdev(meds) if len(meds) > 1 else math.inf
        if meds:
            print(f"image: median {statistics.median(meds):.3f} m, jitter (std) {jitter:.3f} m")
        fin = [r for r in self.scans if math.isfinite(r)]
        scan_ok = len(fin) / len(self.scans) if self.scans else 0.0
        scan_med = statistics.median(fin) if fin else math.nan
        print(f"scan : finite in {100 * scan_ok:.0f} % of scans, median {scan_med:.3f} m "
              f"(error {scan_med - d:+.3f} m)")
        checks = {
            'scan finite >= 95 %': scan_ok >= 0.95,
            f'scan median within {max(0.10, 0.05 * d):.2f} m': abs(scan_med - d) <= max(0.10, 0.05 * d),
            f'jitter <= {0.02 + 0.03 * d:.3f} m': jitter <= 0.02 + 0.03 * d,
            'image valid >= 15 % in >= 95 % of frames': frac_ok >= 0.95,
        }
        for k, v in checks.items():
            print(f"  [{'x' if v else ' '}] {k}")
        ok = all(checks.values())
        print(f"TEST 2 at {d:.2f} m: {'PASS' if ok else 'FAIL'}")
        return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--dist', type=float, required=True, help='taped distance to the face, m')
    ap.add_argument('--secs', type=float, default=20.0)
    ap.add_argument('--roi-deg', type=float, default=10.0)
    ap.add_argument('--depth-topic', default='/oak/stereo/image_raw')
    ap.add_argument('--info-topic', default='/oak/stereo/camera_info')
    a = ap.parse_args()
    rclpy.init()
    node = MaterialCheck(a)
    end = time.monotonic() + a.secs
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = node.report()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
