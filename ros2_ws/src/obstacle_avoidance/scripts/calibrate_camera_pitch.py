#!/usr/bin/env python3
"""calibrate_camera_pitch.py — find camera_pitch_offset_deg (Gate 3C).

Runs against the REAL Pixhawk IMU over MAVROS (not SITL, not an override) with
the airframe LEVEL and the OAK-D facing a flat wall at a known distance.

Principle: the closest point on a vertical wall is reached by a WORLD-HORIZONTAL
ray (the perpendicular foot), so the image row with the minimum range is the
true horizon — independent of how the camera is mounted. We find that row
(v_true) from the depth, read the IMU pitch, and report the camera_pitch_offset
that makes the sector builder's band centre land on it:

    v_horizon = cy + fy*tan(imu_pitch - radians(offset))      (same as runtime)
    => offset = imu_pitch - atan((v_true - cy)/fy)

It PRINTS the recommended value. It does NOT write the config — you set
camera_pitch_offset_deg in avoidance.yaml yourself.

Run (hardware, after sourcing the workspace):
    python3 scripts/calibrate_camera_pitch.py --wall-distance 2.0
"""

import argparse
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo, Imu

from obstacle_avoidance import geometry as g


def decode_depth(msg):
    if msg.encoding == '16UC1':
        dtype, scale = np.uint16, 0.001
    elif msg.encoding == '32FC1':
        dtype, scale = np.float32, 1.0
    else:
        return None
    itemsize = np.dtype(dtype).itemsize
    cols = msg.step // itemsize if msg.step else msg.width
    raw = np.frombuffer(msg.data, dtype=dtype)
    if raw.size < cols * msg.height:
        return None
    raw = raw.reshape(msg.height, cols)[:, :msg.width]
    if msg.is_bigendian:
        raw = raw.byteswap()
    z = raw.astype(np.float32) * scale
    z[(~np.isfinite(z)) | (z <= 0)] = np.nan
    return z


class Calib(Node):
    def __init__(self, args):
        super().__init__('calibrate_camera_pitch')
        self.args = args
        self._lock = threading.Lock()
        self._depths = []
        self._info = None
        self._pitches = []
        self.create_subscription(Image, args.depth_topic, self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.camera_info_topic,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Imu, args.imu_topic, self._on_imu,
                                 qos_profile_sensor_data)

    def _on_info(self, msg):
        with self._lock:
            if self._info is None and msg.k[0] != 0.0:
                self._info = (float(msg.k[0]), float(msg.k[4]),
                              float(msg.k[2]), float(msg.k[5]),
                              int(msg.width), int(msg.height))

    def _on_depth(self, msg):
        z = decode_depth(msg)
        if z is None:
            return
        with self._lock:
            if len(self._depths) < self.args.samples:
                self._depths.append(z)

    def _on_imu(self, msg):
        q = msg.orientation
        with self._lock:
            self._pitches.append(g.pitch_from_quaternion(q.x, q.y, q.z, q.w))


def collect(node, args):
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        with node._lock:
            if len(node._depths) >= args.samples and node._info is not None:
                return
    return


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--wall-distance', type=float, required=True,
                    help='known distance to the flat wall (m)')
    ap.add_argument('--samples', type=int, default=30,
                    help='depth frames to median-average')
    ap.add_argument('--central-deg', type=float, default=5.0,
                    help='half-width of the central column band used (deg)')
    ap.add_argument('--sweep', default='-15,15,0.25',
                    help='min,max,step for the offset sweep (deg)')
    ap.add_argument('--depth-topic', default='/oak/stereo/image_raw')
    ap.add_argument('--camera-info-topic', default='/oak/stereo/camera_info')
    ap.add_argument('--imu-topic', default='/mavros/imu/data')
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = Calib(args)
    print(f"collecting {args.samples} frames from {args.depth_topic} + "
          f"{args.imu_topic} ...")
    collect(node, args)
    with node._lock:
        depths = list(node._depths)
        info = node._info
        pitches = list(node._pitches)
    node.destroy_node()
    rclpy.shutdown()

    if info is None or not depths:
        print("FATAL: no CameraInfo/depth received — check the topics "
              f"({args.camera_info_topic}, {args.depth_topic})")
        raise SystemExit(2)

    fx, fy, cx, cy, W, H = info
    z = np.nanmedian(np.stack(depths), axis=0)          # (H, W), noise-averaged
    D = args.wall_distance

    if pitches:
        imu_pitch = float(np.median(pitches))
    else:
        imu_pitch = 0.0
        print("WARN: no /mavros/imu/data — assuming IMU pitch 0 (airframe level)")
    print(f"intrinsics fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} "
          f"{W}x{H} | IMU pitch {math.degrees(imu_pitch):+.2f} deg "
          f"({len(pitches)} samples)")

    # radial range over a central column band (theta ~ 0 there)
    theta = g.camera_bearing(np.arange(W), cx, fx)
    central = np.abs(theta) < math.radians(args.central_deg)
    r = g.radial_range(z, theta[None, :])
    rc = r[:, central]
    # keep only plausibly-wall pixels (rejects floor/ceiling/outliers)
    good = np.isfinite(rc) & (rc > 0.5 * D) & (rc < 2.0 * D)
    need = max(3, int(0.25 * central.sum()))
    row_range = np.full(H, np.nan)
    for v in range(H):
        vals = rc[v][good[v]]
        if vals.size >= need:
            row_range[v] = np.median(vals)

    rows = np.where(np.isfinite(row_range))[0]
    if rows.size < 10:
        print("FATAL: the wall is not visible across enough central rows — "
              "point the camera squarely at a flat wall filling the view")
        raise SystemExit(2)

    # The range-vs-row curve is a shallow parabola with its minimum at the
    # horizontal ray. Fit a quadratic for a noise-robust, sub-pixel vertex.
    c2, c1, c0 = np.polyfit(rows, row_range[rows], 2)
    if c2 > 1e-9:
        v_true = -c1 / (2.0 * c2)
        min_range = c0 + c1 * v_true + c2 * v_true ** 2
    else:                                    # not convex: fall back to argmin
        v_true = float(rows[np.argmin(row_range[rows])])
        min_range = float(np.nanmin(row_range))
    v_true = float(np.clip(v_true, 0, H - 1))

    recommended = math.degrees(imu_pitch - math.atan((v_true - cy) / fy))

    # sweep, as requested: which offset puts the band centre on v_true
    smin, smax, step = (float(x) for x in args.sweep.split(','))
    offsets = np.arange(smin, smax + 1e-9, step)
    v_band = np.array([g.horizon_row(cy, fy, imu_pitch - math.radians(o))
                       for o in offsets])
    err = np.abs(v_band - v_true)
    best_i = int(np.argmin(err))

    print("\n=== calibration result ===")
    print(f"true horizon row  v_true = {v_true:.1f}  (image centre cy = {cy:.1f})")
    print(f"range at v_true          = {min_range:.2f} m   (wall distance "
          f"D = {D:.2f} m) {'OK' if abs(min_range - D) <= 0.25 else 'CHECK: differs from D — is it a flat wall at D?'}")
    print(f"sweep min |band-centre - v_true|: offset={offsets[best_i]:+.2f} deg "
          f"-> band row {v_band[best_i]:.1f} (err {err[best_i]:.2f} px)")
    print(f"\n>>> recommended camera_pitch_offset_deg = {recommended:+.2f}")
    print(">>> set this in config/avoidance.yaml (NOT written automatically).")
    if abs(math.degrees(imu_pitch)) > 3.0:
        print(">>> NOTE: IMU pitch is not near level; the value already accounts "
              "for it, but recheck with the airframe level for a clean bench read.")


if __name__ == '__main__':
    main()
