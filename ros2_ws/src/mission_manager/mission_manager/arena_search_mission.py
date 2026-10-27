# arena_search_mission.py — PHASE 3 test flight (the arena search) on its
# own: take off -> climb to arena_alt_m -> the mission's arena search -> over
# the matching QR: hover -> land there. With arena_test_scan_reference:=true,
# PHASES 1 + 3: the reference QR is read in the air first (the mission's
# phase 1 code), then the sweep matches against it.
#
# Flies exactly the mission's phase 3 code (mission_manager/arena_search.py).
# Same node as mission_node (same name, so the mission_node: section of
# mission_params.yaml applies unchanged) with a shorter sequence:
#   1. wait for MAVROS and the pilot's RC CH7 switch          (mission_node)
#   2. GPS fix (+ obstacle sensor if required)                 (mission_node)
#   3. gradual takeoff, holding the placed heading: armed ARM_WAIT_S on the
#      ground, lift-off to takeoff_liftoff_alt_m, then a steady
#      takeoff_climb_rate_ms climb over the lift-off point to
#      cruise_altitude_m and on to arena_alt_m (straight to arena_alt_m
#      when that is lower: no climb above the sweep)          (mission_node)
#   3b. (arena_test_scan_reference) phase 1: find, centre over and read the
#      reference QR at cruise_altitude_m (mission_node._reference_qr); not
#      read -> abort -> LAND
#   4. arena search: BCD sweep, never over red (stop, confirm, re-plan
#      around), matching every QR against the stored reference
#   5. match: over it, hover arena_test_hover_s, LAND there. No match after
#      the whole arena: LAND where the sweep ended.
# Every STATUS_LOG_S the terminal gets one status line (state, FC mode,
# altitude, distance from the lift-off point, ground speed, battery).
#
# The test arena: it starts arena_test_offset_m ahead of the takeoff point
# (past the reference QR when phase 1 is flown too: a match outside the
# arena is ignored) and runs arena_test_length_m on along the nose the drone
# was placed with, arena_test_width_m to the right of the nose line. arena_test_use_config:=true flies the
# configured arena_corners_lat/lon instead (a field with known GPS corners).
#
# The reference QR comes from qr_scanner_node: read in this flight
# (arena_test_scan_reference), or scanned in an earlier phase 1 flight
# (stored in ~/aerothon_captures/reference_qr.json) and loaded with its
# reference_file parameter.
#
# State machine: IDLE -> TAKEOFF -> SCAN_REFERENCE_QR -> ENTRY_CORRIDOR
# (both passed through) -> ARENA_SEARCH [-> TARGET_FOUND] -> LANDING
# (arena_test_done) -> LANDED.
# Aborts as mission_node: CH7 LOW -> LOITER; failure -> LAND; FC left
# GUIDED -> stop commanding.
#
#   ros2 launch mission_bringup arena_search_mission.launch.py        (drone)
#   ros2 launch mission_bringup arena_search_mission.launch.py sim:=true

import math
import time

import rclpy

from mission_common import mavros_utils as mav
from mission_manager.mission_node import MissionAbort, MissionNode
from path_planner.arena_frame import M_PER_DEG_LAT

STATUS_LOG_S = 2.0

class ArenaSearchMission(MissionNode):

    def __init__(self):
        super().__init__()
        # (The mission thread only reads them after MAVROS connects.)
        self.declare_parameter("arena_test_length_m", 25.0)   # ahead along the nose
        self.declare_parameter("arena_test_width_m", 25.0)    # to the right
        self.declare_parameter("arena_test_use_config", False)
        self.declare_parameter("arena_test_hover_s", 5.0)
        self.declare_parameter("arena_test_offset_m", 0.0)    # arena start, ahead
        self.declare_parameter("arena_test_scan_reference", False)
        self._last_xy = None            # (east, north, t) for the ground speed
        self.create_timer(STATUS_LOG_S, self._log_status)

    def _log_status(self):
        """One status line in the terminal (main-thread timer)."""
        st = self.mavros_state
        if st is None or not st.connected:
            self.get_logger().info("[status] waiting for the FC (MAVROS not connected)")
            return
        now = time.monotonic()
        speed = None
        if self.local_xy is not None:
            if self._last_xy is not None:
                dx = self.local_xy[0] - self._last_xy[0]
                dy = self.local_xy[1] - self._last_xy[1]
                speed = math.hypot(dx, dy) / max(now - self._last_xy[2], 1e-3)
            self._last_xy = (self.local_xy[0], self.local_xy[1], now)
        home = (f"{mav.haversine(*self.gps, *self.home):.1f} m"
                if self.gps is not None and self.home is not None else "--")
        alt = f"{self.rel_alt:.2f} m" if self.rel_alt is not None else "--"
        batt = f"{self.battery_voltage:.2f} V" if self.battery_voltage else "--"
        self.get_logger().info(
            f"[status] {self.sm.state_id} | {st.mode} {'ARMED' if st.armed else 'disarmed'}"
            f" | alt {alt} | from home {home}"
            f" | gs {f'{speed:.2f} m/s' if speed is not None else '--'}"
            f" | hdg {f'{self.heading_deg:.0f}' if self.heading_deg is not None else '--'}"
            f" | batt {batt} | gps {'fix' if self.gps is not None else 'NO FIX'}")

    def _p(self, name):
        return self.get_parameter(name).value

    def _test_corners(self):
        """The test arena's GPS corners: its near-left corner (offset ahead
        of home), then ahead, ahead + right, right."""
        lat0, lon0 = self.home
        h = math.radians(self.placed_heading_deg)
        fwd = (math.cos(h), math.sin(h))                 # (north, east)
        right = (-math.sin(h), math.cos(h))
        L, W = float(self._p("arena_test_length_m")), float(self._p("arena_test_width_m"))
        o = float(self._p("arena_test_offset_m"))
        k = M_PER_DEG_LAT * math.cos(math.radians(lat0))

        def at(a, b):
            n, e = a * fwd[0] + b * right[0], a * fwd[1] + b * right[1]
            return (lat0 + n / M_PER_DEG_LAT, lon0 + e / k)
        return [at(o, 0), at(o + L, 0), at(o + L, W), at(o, W)]

    def _run_mission(self):
        try:
            self._wait_for_start()
            self.sm.auto_activate()
            self._preflight()
            arena_alt = float(self.cfg.arena_alt_m)
            self._takeoff(min(float(self.cfg.cruise_altitude_m), arena_alt))
            # Same steady climb on up to the sweep altitude (arena.run would
            # otherwise jump the altitude target straight to it).
            if self.rel_alt is not None and arena_alt - self.rel_alt > self.cfg.altitude_tolerance_m:
                self._climb_gradually(arena_alt)
                self.get_logger().info(f"[arena_search_mission] At {self.rel_alt:.2f} m")

            self.sm.airborne()
            if self._p("arena_test_scan_reference"):
                text = self._reference_qr()
                if text is None:
                    raise MissionAbort("failure", "reference QR not read — not sweeping")
                self.get_logger().info(
                    "[arena_search_mission] Reference stored: "
                    + (repr(text) if text else "(no text — matched by picture)")
                    + " — starting the sweep")
                self.sm.mission_target_qr = text
            else:
                self.sm.mission_target_qr = ""  # loaded from file; the scanner holds it
            self.sm.qr_scanned()
            self.sm.entry_complete()        # no corridor in this sequence
            corners = None if self._p("arena_test_use_config") else self._test_corners()
            found = self.arena.run(corners)

            if found is not None:
                self.sm.qr_matched()
                hover = float(self._p("arena_test_hover_s"))
                self.get_logger().info(
                    f"[arena_search_mission] Over the matching QR — hovering {hover:.0f} s")
                self._sleep(hover)
            else:
                self.get_logger().warn("[arena_search_mission] No matching QR — landing here")
            self.sm.arena_test_done()
            self._land_and_wait()
            self.sm.touchdown()
            self._mission_complete = True
            self.get_logger().info(
                "[arena_search_mission] Complete — landed "
                + ("on the matching QR" if found is not None else "(no match)"))
        except MissionAbort as a:
            self._handle_abort(a)
        except Exception as e:
            import traceback
            self.get_logger().error(f"[arena_search_mission] thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))


def main(args=None):
    rclpy.init(args=args)
    node = ArenaSearchMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[arena_search_mission] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
