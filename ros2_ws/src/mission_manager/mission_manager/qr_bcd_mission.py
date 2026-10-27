#!/usr/bin/env python3
# qr_bcd_mission.py — full search mission: read the reference QR in the air,
# BCD-sweep the arena for the ONE QR that matches it while avoiding red
# zones, then land on that QR.
#
#   1. arm + hold, take off to scan_altitude_m
#   2. REFERENCE QR (outside the arena, ~1 m ahead of the takeoff point):
#      /perception/locate_qr reports where a QR is relative to the drone ->
#      centre over it (qr_center_passes); none in view -> step forward
#      qr_step_m along the heading the drone was placed with (qr_max_steps).
#      Then /perception/scan_qr reads it (qr_scan_attempts, re-centring
#      between attempts) and qr_scanner_node stores it — its text (may be
#      empty) and its picture — as the QR to match. Not read -> LAND
#      (nothing to search for).
#   3. change to the sweep altitude if it differs
#   4. ARENA SEARCH: /perception/start_qr_scan (qr_scanner_node Phase 5), then
#      the BCD sweep with red-zone avoidance (bcd_redzone_mission.py). Strips
#      are spaced so a whole marker_size_m QR fits in the frame between two
#      strips. A confirmed match on /qr/detection ("content|lat|lon", after
#      the scanner's 3-frame gate) ends the sweep: brake.
#   5. LAND ON IT: fly to the QR (routed around red zones, still watching for
#      red), centre over it with /perception/locate_qr, hover land_hover_s,
#      LAND, confirm disarm.
#   Sweep finished with no match -> it is already back at the takeoff point:
#   LAND there and report "target not found".
#
# Matching is qr_scanner_node's: a sweep QR matches when its decoded text
# equals the reference's, or — when either has no text — when its picture
# (module grid) is the reference's. The reference and target are the same QR
# printed twice; the rest differ.
#
# Needs: MAVROS, the downward camera on /down_frame (camera_ros, or
# sim_down_camera_node in SITL), qr_scanner_node, red_zone_node.
#   ros2 launch mission_bringup qr_bcd_mission.launch.py
#
# Frame and safety rules as bcd_redzone_mission.py. The red-zone camera
# watch starts with the arena search: the reference QR is assumed clear of
# red.

import math

from std_msgs.msg import Float64, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mission_common import srv_helpers as srv
from path_planner.waypoint_gen import gps_to_local
from mission_manager.bcd_redzone_mission import BcdRedZoneMission, run

SRV_LOCATE_QR = "/perception/locate_qr"


class QrBcdMission(BcdRedZoneMission):
    def __init__(self):
        super().__init__("qr_bcd_mission")
        gp = self.gp
        self.scan_alt = float(gp("scan_altitude_m", 2.0))
        self.qr_step = float(gp("qr_step_m", 0.5))
        self.qr_max_steps = int(gp("qr_max_steps", 4))
        self.qr_center_tol = float(gp("qr_center_tol_m", 0.3))
        self.qr_center_passes = int(gp("qr_center_passes", 2))
        self.qr_max_jump = float(gp("qr_max_jump_m", 3.0))
        self.qr_max_radius = float(gp("qr_max_radius_m", 4.0))     # reference search
        self.qr_scan_attempts = int(gp("qr_scan_attempts", 3))
        self.qr_scan_timeout = float(gp("qr_scan_timeout_s", 20.0))  # = scanner scan_timeout_s
        self.qr_locate_timeout = float(gp("qr_locate_timeout_s", 8.0))  # >= scanner's
        self.land_hover = float(gp("land_hover_s", 7.0))
        self.ref_exclusion = float(gp("ref_exclusion_m", 1.5))
        self._fly_alt = self.scan_alt             # takeoff goes to the scan altitude

        self._ref_id: str | None = None
        self._ref_xy: tuple[float, float] | None = None
        self._match: tuple[str, float, float] | None = None   # content, x, y (local)
        self._placed_hdg: float | None = None
        self._hdg: float | None = None
        mavros_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self._hdg_cb, mavros_qos)
        self.create_subscription(String, "/qr/detection", self._qr_cb, 10)

    # ── Sweep geometry: the QR must fit in the frame between two strips ──
    def _default_heading(self) -> float:
        # Nose East: the image's wide axis (1.36 m each side at 2 m) lies
        # ACROSS the East-West strips, so strips can be 1.5 m apart and a
        # 1 m QR still fits whole in the frame. Along the strips the camera
        # sees 0.77 m ahead — fine for red zones at 0.5 m/s (SITL).
        return 90.0

    def _default_strip_width(self) -> float:
        marker = float(self.gp("marker_size_m", 1.0))
        hdg = float(self.get_parameter("heading_deg").value)
        # cross-track field of view: image width if the nose is along the
        # strips (E/W), image height if it is across them
        along = abs(math.sin(math.radians(hdg))) > 0.7
        fov = self.hfov if along else 2 * math.degrees(
            math.atan(math.tan(math.radians(self.hfov) / 2) * 9 / 16))
        cross = 2 * self.altitude * math.tan(math.radians(fov) / 2)
        width = cross - marker - 0.2
        self.get_logger().info(
            f"  cross-track footprint {cross:.2f} m - marker {marker:.2f} m - 0.2 m "
            f"-> strip width {width:.2f} m")
        if width < 0.3:
            raise ValueError(f"a {marker} m QR does not fit across a {cross:.2f} m footprint "
                             f"at {self.altitude} m — fly higher")
        return width

    # ── Callbacks ────────────────────────────────────────────────────────
    def _hdg_cb(self, msg: Float64):
        self._hdg = float(msg.data)

    def _qr_cb(self, msg: String):
        try:
            content, lat, lon = msg.data.rsplit("|", 2)
            lat, lon = float(lat), float(lon)
        except ValueError:
            self.get_logger().warn(f"[qr] bad /qr/detection '{msg.data}'")
            return
        if self._home is None or self._match is not None:
            return
        x, y = gps_to_local(lat, lon, self._home[0], self._home[1])
        if self._ref_xy is not None and math.dist((x, y), self._ref_xy) < self.ref_exclusion:
            self.get_logger().warn(
                f"[qr] match at E {x:.2f} N {y:.2f} is the REFERENCE QR — ignored")
            self._rescan = True
            return
        self.get_logger().info(f"[qr] MATCHING QR '{content}' at E {x:.2f} m, N {y:.2f} m")
        self._match = (content, x, y)

    def _extra_event(self) -> str | None:
        if self._phase != "arena search":
            return None
        if getattr(self, "_rescan", False):
            self._rescan = False
            srv.call_trigger(self, srv.SRV_START_QR_SCAN, call_timeout=5.0)
        return "target" if self._match is not None else None

    # ── QR helpers ───────────────────────────────────────────────────────
    def _locate(self):
        """(north, east) of the QR from the drone, or None."""
        res = srv.call_trigger(self, SRV_LOCATE_QR, call_timeout=self.qr_locate_timeout + 5.0)
        if res is None or not res.success:
            return None
        try:
            n, e, f, r, _alt = (float(v) for v in res.message.split(","))
        except ValueError:
            self.get_logger().warn(f"  bad locate reply '{res.message}'")
            return None
        self.get_logger().info(f"  QR seen {f:+.2f} m forward, {r:+.2f} m right "
                               f"(N {n:+.2f}, E {e:+.2f})")
        return n, e

    def _move_by(self, d_north: float, d_east: float, label: str,
                 max_radius: float | None = None) -> bool:
        """Short correction from the current position, clamped."""
        dist = math.hypot(d_north, d_east)
        if dist > self.qr_max_jump:
            self.get_logger().warn(f"  {label}: {dist:.1f} m > qr_max_jump_m — not moving")
            return False
        x, y = self._local()
        tx, ty = x + d_east, y + d_north
        if max_radius is not None and math.hypot(tx, ty) > max_radius:
            self.get_logger().warn(f"  {label}: target {math.hypot(tx, ty):.1f} m from "
                                   f"takeoff > {max_radius:.1f} m — not moving")
            return False
        r = self._fly_to(tx, ty, label, watch=False)
        self._sleep(1.5)                  # settle before the camera looks again
        return r == "arrived"

    def _centre(self, max_radius: float | None = None) -> bool:
        found = self._locate()
        for i in range(self.qr_center_passes):
            if found is None:
                return False
            n, e = found
            if math.hypot(n, e) <= self.qr_center_tol:
                break
            self._move_by(n, e, f"centre pass {i + 1}", max_radius)
            found = self._locate()
        if found is not None:
            self.get_logger().info(
                f"  centred: QR {math.hypot(*found):.2f} m from the camera axis")
        return found is not None

    # ── Steps ────────────────────────────────────────────────────────────
    def _arm(self, timeout: float = 90.0) -> bool:
        self._placed_hdg = self._hdg
        return super()._arm(timeout)

    def _reference_qr(self) -> bool:
        if self._placed_hdg is None:
            self._placed_hdg = self._hdg
        sweep_yaw, self.yaw_enu = self.yaw_enu, None   # keep the placement heading
        try:
            if not self._enter_guided_speed():
                return False
            found = self._locate()
            steps = 0
            while found is None and steps < self.qr_max_steps:
                if self._placed_hdg is None:
                    self.get_logger().error("  no compass heading — cannot step forward")
                    break
                steps += 1
                self.get_logger().info(f"  no QR in view — step {steps}/{self.qr_max_steps} "
                                       f"forward {self.qr_step:.1f} m")
                h = math.radians(self._placed_hdg)
                self._move_by(self.qr_step * math.cos(h), self.qr_step * math.sin(h),
                              "search step", self.qr_max_radius)
                found = self._locate()
            if found is None:
                self.get_logger().error("  reference QR not found")
                return False
            for attempt in range(1, self.qr_scan_attempts + 1):
                self._centre(self.qr_max_radius)
                self.get_logger().info(f"  reading reference QR, attempt {attempt}/"
                                       f"{self.qr_scan_attempts}")
                res = srv.call_trigger(self, srv.SRV_SCAN_QR,
                                       call_timeout=self.qr_scan_timeout + 10.0)
                if res is not None and res.success:     # "" = no text: matched by picture
                    self._ref_id = res.message
                    self._ref_xy = self._local()
                    self.get_logger().info(
                        f"  REFERENCE QR READ: '{self._ref_id}' at E {self._ref_xy[0]:.2f} "
                        f"N {self._ref_xy[1]:.2f}")
                    return True
            self.get_logger().error("  reference QR seen but not read — nothing to search for")
            return False
        finally:
            self.yaw_enu = sweep_yaw

    def _sweep_altitude(self) -> bool:
        if abs(self._fly_alt - self.altitude) < 0.1:
            return True
        return self._change_altitude(self.altitude)

    def _arena_search(self) -> bool:
        res = srv.call_trigger(self, srv.SRV_START_QR_SCAN, call_timeout=10.0)
        if res is None or not res.success:
            self.get_logger().error("  qr_scanner_node did not start the arena scan")
            return False
        self.get_logger().info(f"  matching QRs against '{self._ref_id}'")
        return self._sweep()

    def _land_on_target(self) -> bool:
        if self._sweep_end != "target":
            self.get_logger().warn("  sweep finished with NO matching QR — landing at home")
            return self._land()
        content, tx, ty = self._match
        # 1. The QR was in view a moment ago: measure it from right here
        #    rather than trusting the projected position (GPS / processing
        #    latency put it ~1 m off in SITL).
        self._sleep(1.0)
        self.get_logger().info("  locating the matching QR from the brake point")
        self._centred = self._centre()
        # 2. Out of view after braking: go to the projected position, then
        #    look around it.
        if not self._centred:
            for i, (dn, de) in enumerate([(0, 0), (0.6, 0), (0, 0.6), (-0.6, 0), (0, -0.6)]):
                gx, gy = tx + de, ty + dn
                label = "matching QR" if i == 0 else f"QR search point {i}"
                r = self._fly_route(self._local(), (gx, gy), label)
                if r == "unreachable":
                    continue
                if r != "arrived":
                    return False
                self._sleep(1.0)
                self._centred = self._centre()
                if self._centred:
                    break
        if not self._centred:
            self.get_logger().warn("  could not re-locate the QR — landing on its "
                                   "projected position")
            r = self._fly_route(self._local(), (tx, ty), "matching QR (projected)")
            if r not in ("arrived", "unreachable"):
                return False
        x, y = self._local()
        self._target = ("hover over QR", x, y, self._fly_alt)
        self.get_logger().info(f"  hovering {self.land_hover:.0f} s over the QR")
        self._sleep(self.land_hover)
        return self._land()

    def _steps(self) -> list:
        return [
            ("arm + hold",       self._arm),
            ("takeoff",          self._takeoff),
            ("reference QR",     self._reference_qr),
            ("sweep altitude",   self._sweep_altitude),
            ("arena search",     self._arena_search),
            ("land on QR",       self._land_on_target),
        ]

    def _summary(self) -> str:
        x, y = self._local()
        if self._sweep_end == "target":
            how = ("centred over it by the camera" if getattr(self, "_centred", False)
                   else "NOT re-located — on its projected position")
            return (f"✓ QR mission complete — landed on the matching QR '{self._ref_id}' "
                    f"({how}) at E {x:.2f} N {y:.2f}; "
                    f"{len(self._zones)} red zone(s), {self._stops} red stop(s)")
        return (f"✓ QR mission complete — NO matching QR found; landed "
                f"{math.hypot(x, y):.2f} m from the takeoff point; "
                f"{len(self._zones)} red zone(s), {self._stops} stop(s)")


def main():
    run(QrBcdMission)


if __name__ == "__main__":
    main()
