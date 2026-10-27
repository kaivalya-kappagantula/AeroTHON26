# sector_builder_node.py — the core node (Section 4.3, Stage 3).
#
# Turns a forward-facing depth image into a 72-bin LaserScan plus a coverage
# scalar, following the 11-step algorithm in the build spec EXACTLY and in
# order. Every per-pixel operation is vectorised NumPy — a Python loop over
# pixels will not hold 10 Hz on a Pi 5.
#
# Two ideas drive the whole design and are easy to get subtly wrong:
#
#   * Two validity masks, not one. Coverage measures whether the SENSOR is
#     returning data; obstacle bins measure whether something is CLOSE. An open
#     corridor (no obstacle) and a blind camera (no data) must look different,
#     or the coverage guard clamps you to a stop in clear air.
#
#   * Unknown bins are inf, and when the stream dies we STOP publishing rather
#     than publish all-inf. inf -> MAVLink unknown -> ArduPilot ignores it and
#     eventually marks the sensor unhealthy (correct, visible). A finite value,
#     or a steady all-inf scan, both read as "everything clear".

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy)

from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
from std_msgs.msg import Float32

from obstacle_avoidance import geometry as g
from obstacle_avoidance.depth_image import DepthDecodeError, decode_depth


class SectorBuilderNode(Node):

    def __init__(self):
        super().__init__('sector_builder_node')
        self._declare_params()
        self._read_params()

        self._amin, self._amax, self._ainc = g.laserscan_layout(self.n_bins)

        # Intrinsics, latched from the first CameraInfo and never overwritten.
        self._have_info = False
        self.fx = self.fy = self.cx = self.cy = None
        self.W = self.H = None
        # Column caches (valid while intrinsics unchanged).
        self._theta = None          # (W,) camera bearing, +right
        self._col_bin = None        # (W,) LaserScan bin index per column
        self._bin_col_count = None  # (n_bins,) columns falling in each bin

        # Latest pitch from IMU (nose-up positive) and when it arrived.
        self._pitch = 0.0
        self._pitch_stamp_s = None

        # Latest computed products, published on a fixed-rate timer.
        self._latest_scan = None
        self._latest_coverage = 0.0
        self._scan_is_new = False       # set per depth frame, cleared on publish
        self._last_depth_s = None
        self._logged_encoding_error = False

        # --- pubs/subs ---
        # /avoidance/scan -> MAVROS obstacle plugin (RELIABLE subscriber). Must
        # be published RELIABLE or the QoS is incompatible and nothing arrives
        # (spec 3.4: publish direction is reliable). Coverage/debug stay
        # BEST_EFFORT — local, sensor-style consumers only.
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.scan_pub = self.create_publisher(
            LaserScan, '/avoidance/scan', scan_qos)
        self.cov_pub = self.create_publisher(
            Float32, '/avoidance/coverage', qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(
            Image, '/avoidance/debug_band', qos_profile_sensor_data)

        self.create_subscription(
            Image, '~/depth', self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '~/camera_info', self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/mavros/imu/data', self._on_imu, qos_profile_sensor_data)

        self.pub_timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._on_publish_timer)

        self.get_logger().info(
            f'sector_builder_node up: {self.n_bins} bins @ '
            f'{self.publish_rate_hz:.0f} Hz, frame "{self.scan_frame_id}", '
            f'pitch_compensation={self.pitch_compensation}. Waiting for '
            'CameraInfo + depth on ~/camera_info, ~/depth.')

    # --------------------------------------------------------------- params
    def _declare_params(self):
        self.declare_parameter('n_bins', 72)
        self.declare_parameter('band_half_angle_deg', 3.0)
        self.declare_parameter('pitch_compensation', True)
        self.declare_parameter('camera_pitch_offset_deg', 0.0)
        self.declare_parameter('range_min_m', 0.40)
        self.declare_parameter('range_max_report_m', 6.0)
        self.declare_parameter('range_max_valid_m', 15.0)
        self.declare_parameter('min_valid_px_fraction_per_bin', 0.15)
        self.declare_parameter('ground_reject_height_m', -0.40)
        self.declare_parameter('ceiling_reject_height_m', 0.80)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('depth_watchdog_timeout_s', 0.50)
        self.declare_parameter('scan_frame_id', 'oak_forward')
        self.declare_parameter('sim_dropout_fraction', 0.0)
        self.declare_parameter('publish_debug_band', True)

    def _read_params(self):
        gp = self.get_parameter
        self.n_bins = int(gp('n_bins').value)
        self.band_half_angle_deg = float(gp('band_half_angle_deg').value)
        self.pitch_compensation = bool(gp('pitch_compensation').value)
        self.camera_pitch_offset_deg = float(
            gp('camera_pitch_offset_deg').value)
        self.range_min_m = float(gp('range_min_m').value)
        self.range_max_report_m = float(gp('range_max_report_m').value)
        self.range_max_valid_m = float(gp('range_max_valid_m').value)
        self.min_valid_px_fraction = float(
            gp('min_valid_px_fraction_per_bin').value)
        self.ground_reject_h = float(gp('ground_reject_height_m').value)
        self.ceiling_reject_h = float(gp('ceiling_reject_height_m').value)
        self.publish_rate_hz = float(gp('publish_rate_hz').value)
        self.watchdog_timeout_s = float(gp('depth_watchdog_timeout_s').value)
        self.scan_frame_id = str(gp('scan_frame_id').value)
        self.sim_dropout_fraction = float(gp('sim_dropout_fraction').value)
        self.publish_debug_band = bool(gp('publish_debug_band').value)

    # ------------------------------------------------------------ callbacks
    def _on_info(self, msg):
        if self._have_info:
            return                                  # latch first, ignore rest
        k = msg.k
        if k[0] == 0.0 or k[4] == 0.0:
            self.get_logger().warn(
                'CameraInfo has zero fx/fy; ignoring until a valid one arrives',
                once=True)
            return
        self.fx, self.fy = float(k[0]), float(k[4])
        self.cx, self.cy = float(k[2]), float(k[5])
        self.W, self.H = int(msg.width), int(msg.height)
        self._build_column_cache()
        self._have_info = True
        self.get_logger().info(
            f'Latched intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}  {self.W}x{self.H}')

    def _on_imu(self, msg):
        q = msg.orientation
        self._pitch = g.pitch_from_quaternion(q.x, q.y, q.z, q.w)
        self._pitch_stamp_s = self._now_s()

    def _on_depth(self, msg):
        if not self._have_info:
            self.get_logger().warn(
                'depth arriving before CameraInfo; cannot bin without '
                'intrinsics', throttle_duration_sec=2.0)
            return
        z = self._decode_depth(msg)
        if z is None:
            return
        scan, coverage, debug = self._process(z, msg.header.stamp)
        self._latest_scan = scan
        self._latest_coverage = coverage
        self._scan_is_new = True
        self._last_depth_s = self._now_s()
        if debug is not None and self.publish_debug_band:
            self.debug_pub.publish(debug)

    def _on_publish_timer(self):
        # Step 10 — watchdog. If depth has gone silent, stop publishing the scan
        # entirely (do NOT emit all-inf) and force coverage to zero.
        stale = (self._last_depth_s is None or
                 (self._now_s() - self._last_depth_s) > self.watchdog_timeout_s)
        if stale:
            if self._last_depth_s is not None:
                self.get_logger().warn(
                    'depth watchdog: no frame in '
                    f'{self.watchdog_timeout_s:.2f}s — halting scan publication',
                    throttle_duration_sec=1.0)
            self._publish_coverage(0.0)
            return
        # Publish each scan ONCE, stamped with its depth frame's time. Re-sending
        # the last scan with a refreshed stamp would feed ArduPilot a frozen
        # obstacle map as if it were live, and would hide a dead depth stream
        # from the coverage guard (which measures scan arrival) until this
        # watchdog fires. A depth stream slower than publish_rate_hz simply
        # yields fewer scans.
        if self._latest_scan is not None and self._scan_is_new:
            self._scan_is_new = False
            self.scan_pub.publish(self._latest_scan)
            self._publish_coverage(self._latest_coverage)

    # ---------------------------------------------------------------- helpers
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_coverage(self, value):
        m = Float32()
        m.data = float(value)
        self.cov_pub.publish(m)

    def _build_column_cache(self):
        # Step 4 — bearing per column, computed once while intrinsics hold.
        u = np.arange(self.W)
        self._theta = g.camera_bearing(u, self.cx, self.fx)     # +right
        scan_angle = g.bearing_to_scan_angle(self._theta)       # +left
        self._col_bin = g.bearing_to_bin_index(
            scan_angle, n_bins=self.n_bins,
            angle_min=self._amin, angle_increment=self._ainc)   # (W,)
        # Columns per bin (constant per-column pixel count is filled in later).
        self._bin_col_count = np.bincount(
            self._col_bin, minlength=self.n_bins).astype(np.float64)

    def _decode_depth(self, msg):
        """Step 1 — decode depth to metres, float32. Invalid -> NaN."""
        try:
            return decode_depth(msg)
        except DepthDecodeError as e:
            if 'encoding' in str(e):
                if not self._logged_encoding_error:
                    self.get_logger().error(str(e))
                    self._logged_encoding_error = True
            else:
                self.get_logger().warn(f'{e}; dropping frame',
                                       throttle_duration_sec=2.0)
            return None

    # ----------------------------------------------------------- core maths
    def _process(self, z, stamp):
        # Step 2 — horizon row from pitch (fall back to cy when no fresh IMU).
        pitch_fresh = (self._pitch_stamp_s is not None and
                       (self._now_s() - self._pitch_stamp_s) < 0.5)
        if self.pitch_compensation and pitch_fresh:
            # Subtract the fixed camera-to-FC mounting bias from the IMU pitch:
            # the IMU is in the FC, so a tilted camera mount offsets the horizon
            # row by a constant. Calibrate with calibrate_camera_pitch.py.
            pitch_used = self._pitch - np.radians(self.camera_pitch_offset_deg)
            v_horizon = g.horizon_row(self.cy, self.fy, pitch_used)
        else:
            v_horizon = self.cy
        v_horizon = float(np.clip(v_horizon, 0, self.H - 1))

        # Step 3 — crop the band. Everything after this is band-sized (~45x640).
        v_lo, v_hi = g.band_row_bounds(
            v_horizon, self.fy, self.band_half_angle_deg, self.H)
        band = z[v_lo:v_hi + 1, :]                       # (Hb, W), axial metres
        n_rows = band.shape[0]

        # Base validity from decode (NaN where no reading).
        finite = np.isfinite(band)

        # Step 11 — sim dropout injection (only path to exercise the guard in
        # noiseless Gazebo depth). Invalidate a random fraction BEFORE Step 7.
        if self.sim_dropout_fraction > 0.0:
            drop = np.random.random(band.shape) < self.sim_dropout_fraction
            finite = finite & ~drop

        # Step 5 — radial range r = Z / cos(theta). Pass the ANGLE; radial_range
        # takes the cosine internally.
        r = g.radial_range(band, self._theta[None, :])

        # Step 6 — per-pixel height relative to the camera; reject floor/ceiling.
        v_rows = np.arange(v_lo, v_hi + 1)[:, None]
        height = g.pixel_height(v_rows, v_horizon, self.fy, band)
        height_ok = (height >= self.ground_reject_h) & \
                    (height <= self.ceiling_reject_h)

        # Step 7 — TWO masks. Coverage = sensor returning data; obstacle = that
        # AND within reporting range.
        valid_for_coverage = (finite &
                              (band > self.range_min_m) &
                              (band < self.range_max_valid_m) &
                              height_ok)
        valid_for_obstacle = valid_for_coverage & (r < self.range_max_report_m)

        # Step 8 — bin. min(r) over obstacle pixels per bin; require a minimum
        # coverage fraction per bin or the bin is inf.
        r_obst = np.where(valid_for_obstacle, r, np.inf)
        col_min_r = r_obst.min(axis=0)                   # (W,) min over rows
        bin_min = np.full(self.n_bins, np.inf, dtype=np.float64)
        np.minimum.at(bin_min, self._col_bin, col_min_r)

        cov_count_col = valid_for_coverage.sum(axis=0).astype(np.float64)  # (W,)
        bin_cov_count = np.zeros(self.n_bins, dtype=np.float64)
        np.add.at(bin_cov_count, self._col_bin, cov_count_col)
        bin_total_px = self._bin_col_count * n_rows      # (n_bins,)
        with np.errstate(invalid='ignore', divide='ignore'):
            bin_frac = np.where(bin_total_px > 0,
                                bin_cov_count / bin_total_px, 0.0)

        ranges = np.where(bin_frac >= self.min_valid_px_fraction, bin_min,
                          np.inf)
        # Bins with no columns (outside FOV) are already inf via bin_min.

        # Step 9 — coverage over the whole band (all columns are within FOV).
        total_band_px = n_rows * self.W
        coverage = (float(valid_for_coverage.sum()) / total_band_px
                    if total_band_px > 0 else 0.0)

        scan = self._make_scan(ranges, stamp)
        debug = self._make_debug_band(band, valid_for_coverage, stamp)
        return scan, coverage, debug

    def _make_scan(self, ranges, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp                        # source frame time
        scan.header.frame_id = self.scan_frame_id
        scan.angle_min = float(self._amin)
        scan.angle_max = float(self._amax)
        scan.angle_increment = float(self._ainc)
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min_m
        scan.range_max = self.range_max_report_m
        scan.ranges = [float(x) for x in ranges]
        scan.intensities = []
        return scan

    def _make_debug_band(self, band, valid, stamp):
        # mono8 view of the band: valid pixels scaled by range, invalid black.
        if not self.publish_debug_band:
            return None
        vis = np.zeros(band.shape, dtype=np.uint8)
        if valid.any():
            scaled = np.clip(band / self.range_max_report_m, 0.0, 1.0)
            vis[valid] = (scaled[valid] * 255.0).astype(np.uint8)
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.scan_frame_id
        img.height, img.width = vis.shape
        img.encoding = 'mono8'
        img.is_bigendian = 0
        img.step = vis.shape[1]
        img.data = vis.tobytes()
        return img


def main(args=None):
    rclpy.init(args=args)
    node = SectorBuilderNode()
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
