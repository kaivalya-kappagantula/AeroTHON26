#!/usr/bin/env python3
# bcd_redzone_mission.py — BCD sweep that detours around red zones found in
# flight. Red zones may NOT be overflown at any altitude, so the only way
# past one is around it.
#
# Same flight as bcd_sweep_mission.py (arm + hold, takeoff, lawnmower over
# the arena, home, LAND), plus:
#
#   - red_zone_node reports NEW red in the downward camera
#     (/red_zone/sighting) -> brake and hover where the red was first seen
#   - /red_zone/confirm: the node accumulates window_s (4 s) of frames while
#     hovering and returns every zone confirmed so far
#   - the zones, inflated by red_margin_m, are cut out of the arena; if the
#     vehicle stopped inside that margin it first moves straight out of it
#     (away from the red); the rest of the sweep is re-planned: the strip
#     being flown continues from where it stopped, every remaining strip is
#     clipped, and all hops are routed around the zones
#     (path_planner.sweep_replan). Unreachable strips are skipped, never
#     flown to in a straight line.
#   - false alarm (nothing confirmed) -> the node marks the spot as cleared
#     and the sweep resumes
#   - camera silent > detector_timeout_s (node dead, frames rejected,
#     telemetry stale) -> brake and hold; still blind after blind_hold_s ->
#     LAND in place
#
# The lookahead is what the camera sees ahead of the vehicle: at 2 m AGL
# with the nominal Cam3 lens, 0.77 m to the image top/bottom edges and
# 1.36 m to the sides (red_zone_node logs both at startup). The stopping
# distance at speed_ms plus half the frame size must stay inside it: GUIDED
# overshoots ~0.6 m after a brake at 1 m/s (SITL). Hence speed_ms 0.5 and a
# fixed heading_deg.
#
# Frame: local ENU metres about the takeoff point, x = East, y = North. The
# arena spans [area_x0_m, +area_width_m] East and [area_y0_m, +area_height_m]
# North; with the defaults (0, 0) the takeoff point is its SOUTH-WEST corner.
# FCU PARAMETERS ARE NEVER WRITTEN (same runtime-only commands as
# bcd_sweep_mission.py). Any failed step after arming -> LAND in place. The
# RC transmitter (mode switch) is the manual abort; Ctrl+C does NOT land.
#
# Usage (MAVROS and red_zone_node running):
#   ros2 run mission_manager bcd_redzone_mission
#   ros2 run mission_manager bcd_redzone_mission --ros-args -p speed_ms:=0.5
# or everything together:
#   ros2 launch mission_bringup bcd_redzone_mission.launch.py

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64
from mavros_msgs.msg import State
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from mission_common import mavros_utils as mav
from mission_common.srv_helpers import call_trigger
from path_planner.waypoint_gen import (
    compute_strip_width,
    generate_lawnmower,
    gps_to_local,
)
from path_planner.sweep_replan import (
    cut_at_progress,
    clip_segments,
    escape_point,
    exits_cleanly,
    plan_hops,
    polygon,
    routing_space,
    safe_path,
    segments_from_sweep,
    sweep_area,
)

SRV_RED_ZONE_CONFIRM = "/red_zone/confirm"


class BcdRedZoneMission(Node):
    def __init__(self, node_name: str = "bcd_redzone_mission"):
        super().__init__(node_name)

        # ── Parameters ────────────────────────────────────────────────────
        gp = lambda n, d: self.declare_parameter(n, d).value        # noqa: E731
        self.gp = gp
        self.area_x0 = float(gp("area_x0_m", 0.0))
        self.area_y0 = float(gp("area_y0_m", 0.0))
        self.area_w = float(gp("area_width_m", 10.0))
        self.area_h = float(gp("area_height_m", 10.0))
        self.altitude = float(gp("altitude", 2.0))
        # 0.5, not 1.0: SITL at 2 m AGL, 1 m margin — 1.0 m/s overshoots
        # ~0.6 m after the brake and came within 0.13 m of red (moving
        # North, 0.77 m lookahead); 0.5 m/s kept 0.96 m.
        self.speed = float(gp("speed_ms", 0.5))
        self.hfov = float(gp("hfov_deg", 66.0))
        self.overlap = float(gp("overlap", 0.20))
        self.strip_width = float(gp("strip_width_m", 0.0))          # 0 = from camera
        self.edge_margin = float(gp("edge_margin_m", 0.0))
        self.red_margin = float(gp("red_margin_m", 1.0))
        # Compass heading held during the sweep (deg, 0 = North); < 0 leaves
        # it to ArduPilot, which in SITL sometimes faced the direction of
        # travel and sometimes did not.
        hdg = float(gp("heading_deg", self._default_heading()))
        self.yaw_enu = None if hdg < 0 else math.radians(90.0 - hdg)
        self.window_s = float(gp("confirm_window_s", 4.0))          # = red_zone_node window_s
        self.brake_settle = float(gp("brake_settle_s", 1.5))
        self.detector_timeout = float(gp("detector_timeout_s", 0.5))
        self.blind_hold = float(gp("blind_hold_s", 10.0))
        self.max_stops = int(gp("max_stops", 30))
        self.arm_hold = float(gp("arm_hold_s", 7.0))
        self.arrival_tol = float(gp("arrival_tolerance_m", 0.3))
        self.altitude_tol = float(gp("altitude_tolerance_m", 0.5))
        self.altitude_timeout = float(gp("altitude_timeout_s", 40.0))
        self.land_timeout = float(gp("land_timeout_s", 60.0))
        self._fly_alt = self.altitude             # altitude of every position target

        # ── BCD sweep with no zones known yet ────────────────────────────
        area = sweep_area(self.area_x0, self.area_y0, self.area_w, self.area_h,
                          [], 0.0, self.edge_margin)
        if self.strip_width <= 0.0:
            self.strip_width = self._default_strip_width()
        # Shrink the strip width so a whole number of strips spans the area
        # exactly (see bcd_sweep_mission.py).
        _, min_y, _, max_y = area.bounds
        n_strips = max(1, math.ceil((max_y - min_y) / self.strip_width - 1e-6))
        self.strip_width = (max_y - min_y) / n_strips
        self.segments = segments_from_sweep(generate_lawnmower(area, self.strip_width))
        if not self.segments:
            raise ValueError("BCD sweep is empty — check area size / strip width")
        dist = sum(math.dist(s.a, s.b) for s in self.segments)
        self.get_logger().info(
            f"BCD red-zone sweep {self.area_w:.0f} x {self.area_h:.0f} m from "
            f"(E {self.area_x0:.1f}, N {self.area_y0:.1f}) @ {self.altitude:.1f} m, "
            f"{self.speed:.2f} m/s, heading {hdg:.0f}: strip {self.strip_width:.2f} m, "
            f"{n_strips} strips, {dist:.0f} m of strips; red margin {self.red_margin:.2f} m")
        for s in self.segments:
            self.get_logger().info(
                f"  strip {s.strip}: E {s.a[0]:5.2f} -> {s.b[0]:5.2f} m at N {s.a[1]:5.2f} m")

        # ── State (updated by subscription callbacks) ────────────────────
        self._state: State | None = None
        self._gps: tuple[float, float] | None = None
        self._rel_alt: float | None = None
        self._home: tuple[float, float] | None = None
        self._phase = "init"
        self._target: tuple[str, float, float, float] | None = None   # label, x, y, alt
        self._zones: list[Polygon] = []           # confirmed red outlines (local)
        self._stops = 0
        self._det_t = 0.0                         # last /red_zone/sighting (monotonic)
        self._sight_t = 0.0                       # last True
        self._sight_after = 0.0                   # sightings before this are handled
        self._sweep_end = "none"                  # "done" | an _extra_event() value

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
        self.create_subscription(Bool, "/red_zone/sighting", self._sighting_cb, 10)

        self.create_timer(1.0, self._status_cb)
        self._result: bool | None = None

    def start(self):
        """Run the mission off-thread; the main thread spins (see mavros_utils
        THREADING CONTRACT). Called by main() once the node (and any
        subclass) is fully built."""
        threading.Thread(target=self._run, daemon=True).start()

    # ── Hooks for subclasses ─────────────────────────────────────────────
    def _default_heading(self) -> float:
        # Held North with the camera's image top at the nose (cam_yaw 0), the
        # image's WIDE axis (1.36 m each side at 2 m) looks along the
        # East-West strips instead of the short one (0.77 m).
        return 0.0

    def _default_strip_width(self) -> float:
        return compute_strip_width(self.altitude, self.hfov, self.overlap)

    def _extra_event(self) -> str | None:
        """Polled while flying; a non-None value ends the sweep (the vehicle
        brakes) and becomes self._sweep_end."""
        return None

    # ── Callbacks ────────────────────────────────────────────────────────
    def _state_cb(self, msg: State):
        self._state = msg

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self._gps = (msg.latitude, msg.longitude)

    def _alt_cb(self, msg: Float64):
        self._rel_alt = float(msg.data)

    def _sighting_cb(self, msg: Bool):
        now = time.monotonic()
        self._det_t = now
        if msg.data:
            self._sight_t = now

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
        cam = "cam live" if self._detector_live() else "CAM BLIND"
        line = (f"[status] {self._phase} | {mode} {armed} | "
                f"pos E {x:6.2f} N {y:6.2f} m | alt {alt} | {cam}, "
                f"{len(self._zones)} zone(s), {self._stops} stop(s)")
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

    def _detector_live(self) -> bool:
        return time.monotonic() - self._det_t <= self.detector_timeout

    def _new_sighting(self) -> bool:
        return self._sight_t > self._sight_after

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

    def _wait_detector(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            if self._detector_live():
                return True
            time.sleep(0.1)
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

    # ── Flying ───────────────────────────────────────────────────────────
    def _goto_local(self, x: float, y: float, publish_count: int = 20):
        lat, lon = mav.offset_gps(self._home[0], self._home[1], y, x)
        mav.goto(self, lat, lon, self._fly_alt, publish_count=publish_count,
                 yaw_enu=self.yaw_enu, log=False)
        return lat, lon

    def _enter_guided_speed(self) -> bool:
        """Hold position (enters the position-target submode, which resets
        the speed limit), then set speed_ms — same order as path_planner_node.
        Refuse to fly at the FCU's default WP_SPD."""
        x, y = self._local()
        self._goto_local(x, y, publish_count=5)
        if not mav.change_speed(self, self.speed):
            self.get_logger().error(
                f"  FCU rejected speed {self.speed:.2f} m/s — not flying at default speed")
            return False
        self.get_logger().info(f"  speed {self.speed:.2f} m/s accepted by FCU")
        return True

    def _change_altitude(self, alt: float) -> bool:
        x, y = self._local()
        self._fly_alt = alt
        self._target = ("altitude", x, y, alt)
        self.get_logger().info(f"  changing altitude to {alt:.1f} m")
        self._goto_local(x, y, publish_count=10)
        return self._wait_altitude(alt)

    def _fly_to(self, x: float, y: float, label: str, watch: bool = True) -> str:
        """Fly to local (x, y). Returns "arrived", "sighting" (new red in
        view), "blind" (camera silent), an _extra_event() value, or "fail".
        With watch=False the camera is ignored (moving out of a zone's
        margin, or flying outside the arena)."""
        self._target = (label, x, y, self._fly_alt)
        lat, lon = mav.offset_gps(self._home[0], self._home[1], y, x)
        dist = mav.haversine(self._gps[0], self._gps[1], lat, lon)
        self.get_logger().info(
            f"[fly] heading to {label}: E {x:.2f} m, N {y:.2f} m ({dist:.1f} m away)")
        # Stream the target in short bursts so a sighting interrupts within
        # ~0.1 s instead of after a 2 s publish.
        self._goto_local(x, y, publish_count=1)
        deadline = time.time() + 15.0 + 2.5 * dist / max(self.speed, 0.1)
        n = 0
        while time.time() < deadline and rclpy.ok():
            if not self._armed():
                self.get_logger().error("  vehicle disarmed in flight")
                return "fail"
            if watch and self._new_sighting():
                return "sighting"
            if watch and not self._detector_live():
                return "blind"
            extra = self._extra_event() if watch else None
            if extra is not None:
                return extra
            d = mav.haversine(self._gps[0], self._gps[1], lat, lon)
            if d <= self.arrival_tol:
                self.get_logger().info(f"  arrived ({d:.2f} m from target)")
                return "arrived"
            n += 1
            if n < 20 and n % 2 == 0:
                self._goto_local(x, y, publish_count=1)
            time.sleep(0.05)
        self.get_logger().error("  ARRIVAL TIMEOUT")
        return "fail"

    def _brake(self) -> tuple[float, float]:
        """Hold position where the vehicle is now; returns that point."""
        x, y = self._local()
        self._target = ("brake", x, y, self._fly_alt)
        self._goto_local(x, y, publish_count=3)
        self.get_logger().warn(f"  BRAKE at E {x:.2f} m, N {y:.2f} m")
        self._sleep(self.brake_settle)
        return x, y

    def _confirm(self) -> bool:
        """Hover-confirm via red_zone_node; refresh self._zones."""
        res = call_trigger(self, SRV_RED_ZONE_CONFIRM, call_timeout=self.window_s + 15.0)
        self._sight_after = time.monotonic() + 0.2
        if res is None or not res.success:
            self.get_logger().error("  red-zone confirmation failed — cannot see, not moving on")
            return False
        data = json.loads(res.message)
        polys = list(self._zones)        # zones are only ever added, never forgotten
        for z in data["zones"]:
            poly = polygon([gps_to_local(lat, lon, self._home[0], self._home[1])
                            for lat, lon in z])
            if not poly.is_empty:
                polys.append(poly)
        merged = unary_union(polys) if polys else None
        zones = [] if merged is None else list(getattr(merged, "geoms", [merged]))
        self._zones = [z for z in zones if z.geom_type == "Polygon"]
        self.get_logger().info(
            f"  confirmed +{data['new_area_m2']:.2f} m^2 red, {len(zones)} zone(s) known"
            + "".join(f"\n    zone {i}: centre E {z.centroid.x:.2f} N {z.centroid.y:.2f} m, "
                      f"{z.area:.2f} m^2" for i, z in enumerate(zones)))
        return True

    def _area(self):
        """Where sweep strips may be flown: arena minus inflated zones."""
        return sweep_area(self.area_x0, self.area_y0, self.area_w, self.area_h,
                          self._zones, self.red_margin, self.edge_margin)

    def _space(self, *points):
        """Where any hop may go: around the arena, home and points, minus
        inflated zones."""
        return routing_space([(0.0, 0.0), *points], self._area(), self._zones,
                             self.red_margin)

    def _handle_stop(self, reason: str) -> tuple | None:
        """Brake, confirm (or wait out blindness), get clear of the margin.
        Returns the start point for the re-plan, or None to abort."""
        # Only red stops count: a blind stop is a brief brake-and-wait, and a
        # camera that stays silent ends the mission through blind_hold_s.
        if reason == "sighting":
            self._stops += 1
            if self._stops > self.max_stops:
                self.get_logger().error(f"  more than {self.max_stops} red stops — giving up")
                return None
        stop = self._brake()
        if reason == "blind":
            self.get_logger().warn(
                f"  camera silent ({time.monotonic() - self._det_t:.2f} s since the last "
                f"frame report) — holding up to {self.blind_hold:.0f} s")
            if not self._wait_detector(self.blind_hold):
                self.get_logger().error("  camera still silent — aborting")
                return None
            self._sleep(0.5)            # let fresh frames report what is ahead
            if self._new_sighting():
                reason = "sighting"
        if reason == "sighting":
            self.get_logger().warn("  NEW RED IN VIEW — hover-confirming")
            if not self._confirm():
                return None
        if any(z.covers(Point(stop)) for z in self._zones):
            self.get_logger().error("  !!! vehicle stopped OVER a confirmed red zone")
        start = escape_point(stop, self._space(stop))
        if start is None:
            self.get_logger().error("  no flyable area left — aborting")
            return None
        if math.dist(start, stop) > 1e-3:
            if not exits_cleanly(stop, start, self._zones):
                self.get_logger().error("  cannot leave the margin without crossing red — aborting")
                return None
            self.get_logger().warn(
                f"  inside the {self.red_margin:.1f} m margin — moving out "
                f"{math.dist(start, stop):.2f} m")
            if self._fly_to(start[0], start[1], "out of margin", watch=False) != "arrived":
                return None
            self._sight_after = time.monotonic() + 0.2
        return start

    def _fly_route(self, start, goal, label: str) -> str:
        """Fly start -> goal routed around the zones, re-planning after every
        red stop. Returns "arrived", "unreachable", an _extra_event() value
        or "fail"."""
        while True:
            path = safe_path(start, goal, self._space(start, goal))
            if path is None:
                return "unreachable"
            for x, y in path[1:]:
                r = self._fly_to(x, y, label)
                if r == "arrived":
                    start = (x, y)
                    continue
                if r in ("sighting", "blind"):
                    start = self._handle_stop(r)
                    if start is None:
                        return "fail"
                    break
                return r
            else:
                return "arrived"

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
        self._target = ("takeoff", 0.0, 0.0, self._fly_alt)
        self.get_logger().info(f"  climbing straight up to {self._fly_alt:.1f} m")
        if not mav.takeoff(self, self._fly_alt):
            return False
        if not self._wait_altitude(self._fly_alt):
            return False
        self._sleep(2.0)    # settle
        return True

    def _sweep(self) -> bool:
        """BCD sweep from wherever the vehicle is, back to home. Sets
        self._sweep_end: "done", or the _extra_event() that ended it (the
        vehicle is braked and hovering)."""
        if not self._wait_detector(10.0):
            self.get_logger().error(
                "  red_zone_node not reporting on /red_zone/sighting — not sweeping blind")
            return False
        self.get_logger().info("  red-zone detector live")
        if not self._enter_guided_speed():
            return False
        self._sight_after = time.monotonic()

        remaining = list(self.segments)
        start = escape_point(self._local(), self._space(self._local()))
        while True:
            area = self._area()
            remaining = clip_segments(remaining, area)
            hops, skipped = plan_hops(start, remaining, (0.0, 0.0), self._space(start))
            for s in skipped:
                self.get_logger().warn(
                    f"  strip {s.strip} piece E {s.a[0]:.2f}->{s.b[0]:.2f} unreachable — skipped")
            remaining = [s for s in remaining if not any(s is k for k in skipped)]
            self.get_logger().info(
                f"[plan] {len(remaining)} strip piece(s), {len(hops)} hop(s) from "
                f"E {start[0]:.2f} N {start[1]:.2f}")
            stopped = None
            for hop in hops:
                r = self._fly_to(hop.x, hop.y, hop.label)
                if r == "arrived":
                    if hop.seg is not None:
                        remaining = [s for s in remaining if s is not hop.seg]
                    continue
                if r == "fail":
                    return False
                stopped = (r, hop)
                break
            if stopped is None:
                if not hops or hops[-1].label != "HOME":
                    self.get_logger().error("  home unreachable around the zones — landing here")
                self._sweep_end = "done"
                return True
            reason, hop = stopped
            if reason not in ("sighting", "blind"):
                self._brake()
                self._sweep_end = reason
                return True
            start = self._handle_stop(reason)
            if start is None:
                return False
            # The strip being flown continues from where the vehicle stopped.
            if hop.seg is not None:
                cut = cut_at_progress(hop.seg, start)
                remaining = [cut if s is hop.seg else s for s in remaining if
                             s is not hop.seg or cut is not None]

    def _land(self) -> bool:
        x, y = self._local()
        self._target = ("touchdown", x, y, 0.0)
        self.get_logger().info(f"  landing at E {x:.2f} m, N {y:.2f} m from start")
        if not mav.land(self):
            return False
        return self._wait_disarm()

    def _emergency_land(self):
        """After a failed step: LAND in place so the vehicle is not left
        hovering in GUIDED. In place is never over red: the vehicle only
        ever stops short of a zone."""
        try:
            if not self._armed():
                return
            self.get_logger().warn("ABORT: switching to LAND")
            mav.set_mode(self, "LAND")
            self._wait_disarm()
        except Exception as e:  # never let cleanup mask the original failure
            self.get_logger().warn(f"cleanup error: {e}")

    # ── Mission ──────────────────────────────────────────────────────────
    def _steps(self) -> list:
        return [
            ("arm + hold", self._arm),
            ("takeoff",    self._takeoff),
            ("BCD sweep",  self._sweep),
            ("land",       self._land),
        ]

    def _summary(self) -> str:
        x, y = self._local()
        return (f"✓ BCD red-zone mission complete — {len(self._zones)} zone(s), "
                f"{self._stops} stop(s), landed {math.hypot(x, y):.2f} m from start")

    def _run(self):
        try:
            if not self._wait_ready():
                self._result = False
                return
            mav.request_message_intervals(self)
            # ATTITUDE + GLOBAL_POSITION_INT feed the red-zone projection:
            # 20 Hz halves the telemetry gaps (runtime request, not a param)
            mav.request_message_intervals(self, (30, 33), rate_hz=20.0)
            if not self._acquire_home():
                self._result = False
                return

            for name, fn in self._steps():
                self._phase = name
                self.get_logger().info(f"═══ {name} ═══")
                if not fn():
                    self.get_logger().error(f"✗ MISSION FAILED at step '{name}'")
                    self._emergency_land()
                    self._result = False
                    return
            self.get_logger().info(self._summary())
            self._result = True
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted")
            self._result = False
            return
        except Exception as e:
            self.get_logger().error(f"Mission crashed: {type(e).__name__}: {e}")
            self._emergency_land()
            self._result = False


def run(node_cls):
    rclpy.init()
    node = node_cls()
    node.start()
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


def main():
    run(BcdRedZoneMission)


if __name__ == "__main__":
    main()
