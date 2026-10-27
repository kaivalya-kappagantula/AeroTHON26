# sector_viz.py — Stage 3 visual supplement (Section 4.4).
#
# RViz2 is the primary tool; this catches the things RViz doesn't show and,
# above all, makes the left/right mirror bug (Trap #1) visible: 0 deg is drawn
# straight up as "FORWARD" and the sides are labelled LEFT / RIGHT, so a scan
# that dodges the wrong way is obvious at a glance.
#
# Two panes, refreshing at 5 Hz:
#   Left  — the depth image with the selected band as a horizontal box and
#           v_horizon as a line.
#   Right — a polar plot of /avoidance/scan. inf bins are DRAWN (in a distinct
#           colour), not omitted — you need to see the unknowns. A bottom stats
#           strip shows the active source ([DEPTH]/[SYNTHETIC], orange when
#           synthetic so a fake all-clear can't pass for real), coverage, and
#           the corridor L/R/width readout, plus a dashed margin ring.
#
# Run it on a machine with a display; over SSH use `ros2 run ... sector_viz`
# with X-forwarding or an Agg-free backend. It is a debug tool, never launched
# as part of the flight stack.

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy)

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from std_msgs.msg import Float32

from obstacle_avoidance import geometry as g

import matplotlib
matplotlib.use('TkAgg')                     # interactive; falls back gracefully
import matplotlib.pyplot as plt             # noqa: E402


class SectorViz(Node):

    def __init__(self):
        super().__init__('sector_viz')
        self.declare_parameter('n_bins', 72)
        self.declare_parameter('band_half_angle_deg', 3.0)
        self.declare_parameter('range_max_report_m', 6.0)
        self.declare_parameter('pitch_compensation', True)
        self.declare_parameter('viz_margin_ring_m', 0.6)
        self.declare_parameter('depth_topic', '/oak/depth')
        self.declare_parameter('camera_info_topic', '/oak/depth/camera_info')

        self.n_bins = int(self.get_parameter('n_bins').value)
        self.band_half_angle_deg = float(
            self.get_parameter('band_half_angle_deg').value)
        self.range_max = float(self.get_parameter('range_max_report_m').value)
        self.margin_ring_m = float(
            self.get_parameter('viz_margin_ring_m').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)

        self._amin, self._amax, self._ainc = g.laserscan_layout(self.n_bins)

        self._lock = threading.Lock()
        self._depth = None          # (H, W) float32 metres
        self._last_depth_mono = None  # monotonic time of last depth frame
        self._ranges = None         # (n_bins,) float
        self._coverage = None
        self.fx = self.fy = self.cx = self.cy = None
        self._have_info = False
        self._stats_artist = None   # persistent figure-level stats text

        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        # /avoidance/scan is published RELIABLE (to match MAVROS). A BEST_EFFORT
        # subscriber would still receive it, but match RELIABLE so the viz never
        # depends on that subtlety. (depth/camera_info/coverage stay BEST_EFFORT
        # — their publishers are BEST_EFFORT, and a RELIABLE sub would get
        # nothing from them.)
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            LaserScan, '/avoidance/scan', self._on_scan, scan_qos)
        self.create_subscription(
            Float32, '/avoidance/coverage', self._on_cov,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'sector_viz up. depth="{depth_topic}", scan="/avoidance/scan". '
            '0 deg is FORWARD (up); CCW +ve = LEFT (REP-103).')
        self._print_bin_alignment_check()

    def _print_bin_alignment_check(self):
        """Startup self-check of the SHARED bin maths.

        Uses geometry.bearing_span_to_bins — the exact function the synthetic
        node fills bins with — so the console shows which bins a forward
        obstacle lands in and where their centres are. A forward obstacle
        (bearing 0, width 10) must be bins 35/36/37 centred at -5/0/+5 deg. If
        it lands in 36/37/38 instead, the rounding in bearing_to_bin_index() is
        off — that is shared code, not a viz bug.
        """
        ref_bearing, ref_width = 0.0, 10.0
        bins = g.bearing_span_to_bins(ref_bearing, ref_width, n_bins=self.n_bins)
        centres = [round(float(np.degrees(self._amin + int(b) * self._ainc)), 1)
                   for b in bins]
        self.get_logger().info(
            'bin-alignment self-check (shared geometry.bearing_span_to_bins): '
            f'obstacle_bearing_deg={ref_bearing}, width_deg={ref_width} -> '
            f'bins {bins.tolist()} centred at {centres} deg')
        self.get_logger().info(
            '  expect bins [35, 36, 37] at [-5.0, 0.0, 5.0] deg; if you see '
            '36/37/38, fix rounding in geometry.bearing_to_bin_index()')

    # ------------------------------------------------------------ callbacks
    def _on_info(self, msg):
        if self._have_info or msg.k[0] == 0.0:
            return
        self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
        self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
        self._have_info = True

    def _on_depth(self, msg):
        enc = msg.encoding
        if enc == '16UC1':
            dtype, scale = np.uint16, 0.001
        elif enc == '32FC1':
            dtype, scale = np.float32, 1.0
        else:
            return
        itemsize = np.dtype(dtype).itemsize
        cols = msg.step // itemsize if msg.step else msg.width
        raw = np.frombuffer(msg.data, dtype=dtype)
        if raw.size < cols * msg.height:
            return
        raw = raw.reshape(msg.height, cols)[:, :msg.width]
        z = raw.astype(np.float32) * scale
        with self._lock:
            self._depth = z
            self._last_depth_mono = time.monotonic()

    def _on_scan(self, msg):
        with self._lock:
            self._ranges = np.asarray(msg.ranges, dtype=np.float64)

    def _on_cov(self, msg):
        with self._lock:
            self._coverage = float(msg.data)

    # ------------------------------------------------------ corridor width
    @staticmethod
    def _wall_distance(ranges, angles, lo_deg, hi_deg):
        """Perpendicular distance to a side wall over one arc, or None.

        For a wall running parallel to the flight direction at lateral offset d,
        a beam at bearing theta returns range r with r*sin(theta) == d. So the
        lateral component r*sin(theta) is the perpendicular wall distance; we
        take the median across the arc for robustness. Bearings are LaserScan
        angles (positive = left), so the right arc's values are negative and we
        report the magnitude. Returns None if fewer than 3 finite bins in the
        arc — too little to trust.
        """
        lo, hi = np.radians(lo_deg), np.radians(hi_deg)
        in_arc = (angles >= lo) & (angles <= hi) & np.isfinite(ranges)
        if int(np.count_nonzero(in_arc)) < 3:
            return None
        lateral = ranges[in_arc] * np.sin(angles[in_arc])
        return abs(float(np.median(lateral)))

    # ---------------------------------------------------------------- render
    def render(self, fig, ax_img, ax_pol):
        with self._lock:
            depth = None if self._depth is None else self._depth.copy()
            ranges = None if self._ranges is None else self._ranges.copy()
            coverage = self._coverage
            have_info = self._have_info
            cy, fy = self.cy, self.fy
            last_depth_mono = self._last_depth_mono

        # ---- left: depth + band box + horizon line ----
        ax_img.clear()
        ax_img.set_title('depth + band')
        if depth is not None:
            disp = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            ax_img.imshow(disp, cmap='viridis', vmin=0.0, vmax=self.range_max)
            if have_info:
                H = depth.shape[0]
                v_h = float(np.clip(cy, 0, H - 1))     # viz uses level horizon
                half_px = fy * np.tan(np.radians(self.band_half_angle_deg))
                v_lo = np.clip(v_h - half_px, 0, H - 1)
                v_hi = np.clip(v_h + half_px, 0, H - 1)
                ax_img.axhline(v_h, color='red', lw=1.0, label='v_horizon')
                ax_img.axhspan(v_lo, v_hi, color='yellow', alpha=0.25)
                ax_img.legend(loc='upper right', fontsize=7)
        else:
            ax_img.text(0.5, 0.5, 'no depth', ha='center', va='center',
                        transform=ax_img.transAxes)

        # ---- right: polar scan, inf drawn distinctly ----
        ax_pol.clear()
        ax_pol.set_title('scan   (0 deg = FORWARD)', pad=20)
        # Small note under the title: the plot is CCW-positive (REP-103), so
        # positive theta is the drone's LEFT. This is the mirror-bug check.
        ax_pol.annotate('(CCW +ve = LEFT)', xy=(0.5, 1.0),
                        xycoords='axes fraction', ha='center', va='bottom',
                        fontsize=7, color='gray')
        ax_pol.set_theta_zero_location('N')
        ax_pol.set_theta_direction(1)           # CCW positive = left
        ax_pol.set_rmax(self.range_max)

        # Dashed reference ring at the BendyRuler margin (OA_MARGIN_MAX).
        ring_theta = np.linspace(0.0, 2.0 * np.pi, 120)
        ax_pol.plot(ring_theta,
                    np.full_like(ring_theta, self.margin_ring_m),
                    linestyle='--', color='orange', lw=1.0,
                    label=f'margin {self.margin_ring_m:.1f} m')

        width_txt = 'L: --   R: --   width: --'
        if ranges is not None and ranges.size == self.n_bins:
            angles = self._amin + np.arange(self.n_bins) * self._ainc
            finite = np.isfinite(ranges)
            ax_pol.scatter(angles[finite], ranges[finite], s=18,
                           c='tab:blue', label='obstacle')
            # unknown/inf bins: draw at rmax in a distinct colour so they are
            # visible rather than silently missing.
            inf_ang = angles[~finite]
            ax_pol.scatter(inf_ang, np.full(inf_ang.shape, self.range_max),
                           s=8, c='0.75', marker='x', label='inf')
            ax_pol.legend(loc='lower left', fontsize=7,
                          bbox_to_anchor=(-0.15, -0.12))

            # Corridor width: side arcs are LaserScan angles (positive = left).
            left = self._wall_distance(ranges, angles, 10.0, 80.0)
            right = self._wall_distance(ranges, angles, -80.0, -10.0)
            ls = '--' if left is None else f'{left:.1f} m'
            rs = '--' if right is None else f'{right:.1f} m'
            ws = ('--' if (left is None or right is None)
                  else f'{left + right:.1f} m')
            width_txt = f'L: {ls}   R: {rs}   width: {ws}'

        cov_txt = 'coverage: --' if coverage is None else f'coverage: {coverage:.2f}'
        # CCW-positive plot: LEFT is at +45 deg (upper-left), RIGHT at 315 deg
        # (= -45 deg, upper-right). Getting these swapped is exactly the bug
        # this tool exists to catch, so they must match the theta convention.
        ax_pol.text(np.radians(45), self.range_max * 1.25, 'LEFT',
                    ha='center', color='green', fontsize=9)
        ax_pol.text(np.radians(315), self.range_max * 1.25, 'RIGHT',
                    ha='center', color='green', fontsize=9)

        # Active scan source, inferred from depth freshness. Coverage 1.00 with
        # NO depth stream is the synthetic node, not a real all-clear — flag it
        # loudly (orange) so it can never be mistaken for valid sensor data.
        if ranges is None:
            source, src_color = 'NO SCAN', 'gray'
        elif (last_depth_mono is not None and
              (time.monotonic() - last_depth_mono) < 1.0):
            source, src_color = 'DEPTH', 'black'
        else:
            source, src_color = 'SYNTHETIC', 'darkorange'

        stats = f'[{source}]     {cov_txt}      {width_txt}'
        # One persistent figure-level annotation (created once, updated in
        # place) at the bottom — a fresh fig.text every frame would leak
        # artists, and the old suptitle overlapped the polar subplot title.
        if self._stats_artist is None:
            self._stats_artist = fig.text(
                0.5, 0.04, stats, ha='center', va='center', fontsize=11,
                family='monospace')
        self._stats_artist.set_text(stats)
        self._stats_artist.set_color(src_color)


def main(args=None):
    rclpy.init(args=args)
    node = SectorViz()

    # Spin ROS in a background thread; matplotlib owns the main thread.
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plt.ion()
    fig = plt.figure(figsize=(11, 5.5))
    ax_img = fig.add_subplot(1, 2, 1)
    ax_pol = fig.add_subplot(1, 2, 2, projection='polar')
    # Reserve headroom for the subplot titles and a bottom strip for the stats
    # annotation so nothing overlaps (the old suptitle collided with the polar
    # title). Set once; ax.clear() in render does not disturb figure margins.
    fig.subplots_adjust(left=0.05, right=0.96, top=0.84, bottom=0.14,
                        wspace=0.30)

    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            node.render(fig, ax_img, ax_pol)
            fig.canvas.draw_idle()
            plt.pause(0.2)                       # ~5 Hz
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
