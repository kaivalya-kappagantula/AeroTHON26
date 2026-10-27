#!/usr/bin/env python3
"""Live bench monitor for /avoidance/scan — the desk-test tool for Gate 1A.

Prints, once a second: scan rate, coverage, the nearest return and its bearing,
and an ASCII bar of the forward +/-60 deg arc. Point the camera at a wall or hold
a box off to one side and read the bearing off directly; that is the whole L/R
sanity check, no RViz and no vehicle needed.

Bearings follow the LaserScan convention the package uses throughout: REP-103,
CCW positive, so +30 deg is LEFT of the camera and -30 deg is RIGHT.

Run it straight from src, like the other scripts here:

    python3 scripts/bench_scan_monitor.py
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

# Forward arc drawn in the ASCII strip. Wider than this is behind the camera's
# ~70 deg HFOV and is always inf, so it would just pad the display with dots.
ARC_DEG = 60.0

# Past this with no scan, report STALE instead of redrawing the last one.
STALE_S = 0.5


class BenchMonitor(Node):
    def __init__(self):
        super().__init__('bench_scan_monitor')
        self._count = 0
        self._last_scan = None
        self._last_scan_t = 0.0
        self._coverage = float('nan')
        self._t0 = time.monotonic()

        self.create_subscription(
            LaserScan, '/avoidance/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Float32, '/avoidance/coverage', self._on_cov, qos_profile_sensor_data)
        self.create_timer(1.0, self._report)
        self.get_logger().info(
            f'watching /avoidance/scan (+/-{ARC_DEG:.0f} deg arc, + = LEFT)')

    def _on_scan(self, msg):
        self._count += 1
        self._last_scan = msg
        self._last_scan_t = time.monotonic()

    def _on_cov(self, msg):
        self._coverage = msg.data

    def _report(self):
        now = time.monotonic()
        hz = self._count / max(now - self._t0, 1e-6)
        self._count = 0
        self._t0 = now

        scan = self._last_scan
        if scan is None:
            print(f'  {hz:5.1f} Hz   -- no scan yet --', flush=True)
            return

        # Never redraw a dead scan as if it were live — a frozen bearing reads
        # exactly like a stable one, which is the worst possible display during
        # the failure it is most likely to be showing.
        age = now - self._last_scan_t
        if age > STALE_S:
            print(f'  {hz:5.1f} Hz   *** SCAN STALE {age:5.1f}s '
                  f'— no data from the builder ***', flush=True)
            return

        # Nearest finite return anywhere in the scan, with its bearing.
        best_r, best_deg = float('inf'), None
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and r < best_r:
                best_r = r
                best_deg = math.degrees(scan.angle_min + i * scan.angle_increment)

        # ASCII strip, drawn left-to-right as seen from behind the camera, so
        # positive (left) bearings must appear on the left of the strip.
        cells = []
        for i, r in enumerate(scan.ranges):
            deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            if abs(deg) > ARC_DEG:
                continue
            if not math.isfinite(r):
                ch = '.'
            elif r < 1.0:
                ch = '#'
            elif r < 2.0:
                ch = '+'
            elif r < 4.0:
                ch = '-'
            else:
                ch = ':'
            cells.append((deg, ch))
        cells.sort(key=lambda c: -c[0])
        strip = ''.join(c[1] for c in cells)

        near = f'{best_r:4.2f} m @ {best_deg:+6.1f} deg' if best_deg is not None \
            else '   -- all inf --      '
        side = ''
        if best_deg is not None and abs(best_deg) > 2.0:
            side = ' LEFT' if best_deg > 0 else ' RIGHT'
        print(f'  {hz:5.1f} Hz  cov {self._coverage:4.2f}  near {near}{side:6s} '
              f'|L {strip} R|', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = BenchMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C and SIGTERM are the normal ways to stop a bench tool.
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
