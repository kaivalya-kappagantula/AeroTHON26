#!/usr/bin/env python3
# bcd_sweep_mission.py — standalone BCD sweep flight over a small area.
#
# Sequence (no FSM, no perception):
#   1. wait for FCU ready + GPS fix; the takeoff point becomes home / origin
#   2. GUIDED + arm, then sit armed on the ground for arm_hold_s (5 s)
#   3. takeoff to `altitude` (2 m)
#   4. BCD lawnmower over area_width_m x area_height_m (5 m x 5 m), using
#      path_planner.waypoint_gen (strip spacing from the camera footprint,
#      or strip_width_m if > 0)
#   5. fly back to the starting point at altitude, LAND, confirm disarm
#
# Area frame: local ENU metres about the takeoff point, x = East, y = North.
# The area spans [0, area_width_m] East and [0, area_height_m] North, so the
# takeoff point is the SOUTH-WEST corner of the area. Place the drone there.
#
# Any failed step after arming -> LAND in place. The RC transmitter (mode
# switch) is the manual abort; Ctrl+C stops this node but does NOT land.
#
# FCU PARAMETERS ARE NEVER WRITTEN. Only runtime commands are sent: set mode,
# arm, takeoff, position targets, MAV_CMD_DO_CHANGE_SPEED (a per-flight speed
# limit, not saved — WP_SPD / WPNAV_SPEED in Mission Planner unchanged) and
# SET_MESSAGE_INTERVAL (telemetry rates, reset on reboot). Do not add
# mav.set_fcu_param here.
#
# If the FCU rejects the 0.25 m/s speed command the sweep is aborted (LAND),
# rather than flown at the FCU's default WP_SPD (10 m/s on Copter 4.7).
#
# A 1 Hz [status] log line reports mode, armed, local position (m E / m N of
# the start), altitude, and the current target with distance to it.
#
# Usage:
#   ros2 run mission_manager bcd_sweep_mission
#   ros2 run mission_manager bcd_sweep_mission --ros-args -p altitude:=3.0 \
#       -p strip_width_m:=2.0 -p speed_ms:=0.5

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from mavros_msgs.msg import State

from mission_common import mavros_utils as mav
from path_planner.waypoint_gen import (
    build_flyable,
    compute_strip_width,
    generate_lawnmower,
    route_waypoints,
)


class BcdSweepMission(Node):
    def __init__(self):
        super().__init__("bcd_sweep_mission")

        # ── Parameters ────────────────────────────────────────────────────
        gp = lambda n, d: self.declare_parameter(n, d).value        # noqa: E731
        self.area_w = float(gp("area_width_m", 5.0))
        self.area_h = float(gp("area_height_m", 5.0))
        self.altitude = float(gp("altitude", 2.0))
        self.speed = float(gp("speed_ms", 0.25))
        self.hfov = float(gp("hfov_deg", 66.0))
        self.overlap = float(gp("overlap", 0.20))
        self.strip_width = float(gp("strip_width_m", 0.0))          # 0 = from camera
        self.edge_margin = float(gp("edge_margin_m", 0.0))
        self.arm_hold = float(gp("arm_hold_s", 5.0))
        self.arrival_tol = float(gp("arrival_tolerance_m", 0.3))
        self.altitude_tol = float(gp("altitude_tolerance_m", 0.5))
        self.altitude_timeout = float(gp("altitude_timeout_s", 40.0))
        self.land_timeout = float(gp("land_timeout_s", 60.0))

        # ── BCD sweep (local ENU, about the takeoff point) ───────────────
        flyable = build_flyable(self.area_w, self.area_h, [], 0.0, self.edge_margin)
        if self.strip_width <= 0.0:
            self.strip_width = compute_strip_width(self.altitude, self.hfov, self.overlap)
        # The strip width is a maximum: shrink it so a whole number of strips
        # spans the area exactly (lines at w/2, 3w/2, ...). Otherwise the
        # last strip can stop short of the far edge (5 m @ 2 m alt: lines at
        # 1.04 / 3.12 m leave 4.42-5 m unseen).
        _, min_y, _, max_y = flyable.bounds
        n_strips = max(1, math.ceil((max_y - min_y) / self.strip_width - 1e-6))
        self.strip_width = (max_y - min_y) / n_strips
        sweep = generate_lawnmower(flyable, self.strip_width)
        if not sweep:
            raise ValueError("BCD sweep is empty — check area size / strip width")
        # Start (0, 0) -> sweep -> back to start.
        self.route = route_waypoints([(0.0, 0.0)] + sweep + [(0.0, 0.0)], flyable)[1:]
        dist = sum(math.dist(a, b) for a, b in zip([(0.0, 0.0)] + self.route, self.route))
        self.get_logger().info(
            f"BCD sweep {self.area_w:.0f} x {self.area_h:.0f} m @ {self.altitude:.1f} m: "
            f"strip {self.strip_width:.2f} m, {len(sweep) // 2} strips, "
            f"{len(self.route)} waypoints, {dist:.0f} m (~{dist / self.speed:.0f} s)")
        for i, (x, y) in enumerate(self.route):
            self.get_logger().info(f"  WP{i:02d}  x={x:6.2f} m E   y={y:6.2f} m N")

        # ── State (updated by subscription callbacks) ────────────────────
        self._state: State | None = None
        self._gps: tuple[float, float] | None = None
        self._rel_alt: float | None = None
        self._home: tuple[float, float] | None = None
        self._phase = "init"
        self._target: tuple[str, float, float, float] | None = None   # label, x, y, alt

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(State, "/mavros/state", self._state_cb, qos)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._gps_cb, qos)
        self.create_subscription(
            Float64, "/mavros/global_position/rel_alt", self._alt_cb, qos)

        self.create_timer(1.0, self._status_cb)

        # Mission runs off-thread; main thread spins (see mavros_utils
        # THREADING CONTRACT).
        self._result: bool | None = None
        threading.Thread(target=self._run, daemon=True).start()

    # ── Callbacks ────────────────────────────────────────────────────────
    def _state_cb(self, msg: State):
        self._state = msg

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self._gps = (msg.latitude, msg.longitude)

    def _alt_cb(self, msg: Float64):
        self._rel_alt = float(msg.data)

    def _local(self):
        """(x East, y North) metres of the vehicle from the start point."""
        d_n, d_e = mav.ne_offset(self._home[0], self._home[1], self._gps[0], self._gps[1])
        return d_e, d_n

    def _status_cb(self):
        if self._phase in ("init", "done"):
            return
        s = self._state
        mode = s.mode if s is not None else "?"
        armed = "ARMED" if self._armed() else "disarmed"
        alt = f"{self._rel_alt:5.2f} m" if self._rel_alt is not None else "  n/a"
        if self._home is None or self._gps is None:
            self.get_logger().info(f"[status] {self._phase} | {mode} {armed} | alt {alt}")
            return
        x, y = self._local()
        line = (f"[status] {self._phase} | {mode} {armed} | "
                f"pos E {x:6.2f} N {y:6.2f} m | alt {alt}")
        if self._target is not None:
            label, tx, ty, talt = self._target
            line += (f" | -> {label} (E {tx:5.2f}, N {ty:5.2f}, alt {talt:.1f})"
                     f" {math.hypot(tx - x, ty - y):5.2f} m away")
        self.get_logger().info(line)

    # ── Blocking waiters (sleep-poll; main thread does the spin) ─────────
    def _sleep(self, secs: float):
        end = time.time() + secs
        while time.time() < end:
            if not rclpy.ok():
                raise KeyboardInterrupt
            time.sleep(min(0.1, end - time.time()))

    def _armed(self) -> bool:
        return self._state is not None and self._state.armed

    def _wait_ready(self, timeout: float = 60.0) -> bool:
        self.get_logger().info("Waiting for FCU ready (connected + standby)...")
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            s = self._state
            # system_status 3 == MAV_STATE_STANDBY (ready to arm)
            if s is not None and s.connected and s.system_status == 3:
                return True
            time.sleep(0.25)
        self.get_logger().error("FCU not ready (timeout)")
        return False

    def _acquire_home(self, timeout: float = 30.0) -> bool:
        self.get_logger().info("Acquiring home GPS fix...")
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            if self._gps is not None:
                self._home = self._gps
                self.get_logger().info(
                    f"Home / sweep origin: lat={self._home[0]:.7f}, lon={self._home[1]:.7f}")
                return True
            time.sleep(0.25)
        self.get_logger().error("No GPS fix (timeout)")
        return False

    def _wait_altitude(self, target: float) -> bool:
        deadline = time.time() + self.altitude_timeout
        while time.time() < deadline and rclpy.ok():
            if self._rel_alt is not None and abs(self._rel_alt - target) <= self.altitude_tol:
                self.get_logger().info(f"  reached altitude {self._rel_alt:.2f} m")
                return True
            time.sleep(0.2)
        cur = f"{self._rel_alt:.2f}" if self._rel_alt is not None else "n/a"
        self.get_logger().error(f"  ALTITUDE TIMEOUT (at {cur} m, target {target:.1f} m)")
        return False

    def _wait_arrival(self, lat: float, lon: float, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            if not self._armed():
                self.get_logger().error("  vehicle disarmed in flight")
                return False
            d = mav.haversine(self._gps[0], self._gps[1], lat, lon)
            if d <= self.arrival_tol:
                self.get_logger().info(f"  arrived ({d:.2f} m from target)")
                return True
            time.sleep(0.2)
        self.get_logger().error("  ARRIVAL TIMEOUT")
        return False

    def _wait_disarm(self) -> bool:
        deadline = time.time() + self.land_timeout
        while time.time() < deadline and rclpy.ok():
            if not self._armed():
                self.get_logger().info("  disarmed (touchdown confirmed)")
                return True
            time.sleep(0.25)
        self.get_logger().error("  LAND/DISARM TIMEOUT (still armed)")
        return False

    # ── Steps ────────────────────────────────────────────────────────────
    def _arm(self, timeout: float = 90.0) -> bool:
        """GUIDED + arm (retried while the EKF settles), then hold armed on
        the ground for arm_hold_s."""
        mode = self._state.mode if self._state is not None else "?"
        self.get_logger().info(f"  current mode {mode} — requesting GUIDED")
        if not mav.set_mode(self, "GUIDED"):
            return False
        self._sleep(1.0)
        deadline = time.time() + timeout
        attempt = 0
        while time.time() < deadline and rclpy.ok():
            attempt += 1
            if mav.arm(self):
                t = time.time() + 5.0
                while time.time() < t and rclpy.ok():
                    if self._armed():
                        self.get_logger().info(
                            f"  armed — holding on the ground {self.arm_hold:.0f} s")
                        self._sleep(self.arm_hold)
                        # ArduPilot auto-disarms after DISARM_DELAY (10 s) idle.
                        return self._armed()
                    time.sleep(0.1)
            self.get_logger().warn(f"  arm attempt {attempt} not confirmed, retrying")
            self._sleep(2.0)
        self.get_logger().error("  ARM FAILED (timeout)")
        return False

    def _takeoff(self) -> bool:
        self._target = ("takeoff", 0.0, 0.0, self.altitude)
        self.get_logger().info(f"  climbing straight up to {self.altitude:.1f} m")
        if not mav.takeoff(self, self.altitude):
            return False
        if not self._wait_altitude(self.altitude):
            return False
        self._sleep(2.0)    # settle
        return True

    def _sweep(self) -> bool:
        # Enter the position-target submode first (resets the speed limit),
        # then set the sweep speed — same order as path_planner_node.
        mav.goto(self, self._gps[0], self._gps[1], self.altitude, publish_count=5)
        if not mav.change_speed(self, self.speed):
            self.get_logger().error(
                f"  FCU rejected speed {self.speed:.2f} m/s — not flying at default speed")
            return False
        self.get_logger().info(f"  sweep speed {self.speed:.2f} m/s accepted by FCU")
        n = len(self.route)
        for i, (x, y) in enumerate(self.route):
            lat, lon = mav.offset_gps(self._home[0], self._home[1], y, x)
            label = "HOME" if i == n - 1 else f"WP{i:02d}/{n - 2:02d}"
            dist = mav.haversine(self._gps[0], self._gps[1], lat, lon)
            self._target = (label, x, y, self.altitude)
            self.get_logger().info(
                f"[sweep] heading to {label}: E {x:.2f} m, N {y:.2f} m, alt "
                f"{self.altitude:.1f} m ({dist:.1f} m away)")
            mav.goto(self, lat, lon, self.altitude)
            if not self._wait_arrival(lat, lon, 15.0 + 2.5 * dist / max(self.speed, 0.1)):
                return False
        return True

    def _land(self) -> bool:
        x, y = self._local()
        self._target = ("touchdown", 0.0, 0.0, 0.0)
        self.get_logger().info(f"  landing at E {x:.2f} m, N {y:.2f} m from start")
        if not mav.land(self):
            return False
        return self._wait_disarm()

    def _emergency_land(self):
        """After a failed step: LAND in place so the vehicle is not left
        hovering in GUIDED."""
        try:
            if not self._armed():
                return
            self.get_logger().warn("ABORT: switching to LAND")
            mav.set_mode(self, "LAND")
            self._wait_disarm()
        except Exception as e:  # never let cleanup mask the original failure
            self.get_logger().warn(f"cleanup error: {e}")

    # ── Mission ──────────────────────────────────────────────────────────
    def _run(self):
        try:
            if not self._wait_ready():
                self._result = False
                return
            mav.request_message_intervals(self)
            if not self._acquire_home():
                self._result = False
                return

            steps = [
                ("arm + hold", self._arm),
                ("takeoff",    self._takeoff),
                ("BCD sweep",  self._sweep),
                ("land",       self._land),
            ]
            for name, fn in steps:
                self._phase = name
                self.get_logger().info(f"═══ {name} ═══")
                if not fn():
                    self.get_logger().error(f"✗ MISSION FAILED at step '{name}'")
                    self._emergency_land()
                    self._result = False
                    return
            x, y = self._local()
            self.get_logger().info(
                f"✓ BCD sweep mission complete — landed {math.hypot(x, y):.2f} m from start")
            self._result = True
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted")
            self._result = False
            return
        except Exception as e:
            self.get_logger().error(f"Mission crashed: {type(e).__name__}: {e}")
            self._emergency_land()
            self._result = False


def main():
    rclpy.init()
    node = BcdSweepMission()
    try:
        while rclpy.ok() and node._result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        result = node._result
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    if result is False:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
