# arena_search_mission.py — PHASE 3 test flight (the arena search) on its
# own: take off -> climb to arena_alt_m -> the mission's arena search -> over
# the matching QR: hover -> land there.
#
# Flies exactly the mission's phase 3 code (mission_manager/arena_search.py).
# Same node as mission_node (same name, so the mission_node: section of
# mission_params.yaml applies unchanged) with a shorter sequence:
#   1. wait for MAVROS and the pilot's RC CH7 switch          (mission_node)
#   2. GPS fix (+ obstacle sensor if required)                 (mission_node)
#   3. gradual takeoff to cruise_altitude_m, holding the placed heading,
#      then straight up to arena_alt_m                         (mission_node)
#   4. arena search: BCD sweep, never over red (stop, confirm, re-plan
#      around), matching every QR against the stored reference
#   5. match: over it, hover arena_test_hover_s, LAND there. No match after
#      the whole arena: LAND where the sweep ended.
#
# The test arena: the drone takes off at its corner; it runs
# arena_test_length_m ahead along the nose the drone was placed with and
# arena_test_width_m to the right of it. arena_test_use_config:=true flies the
# configured arena_corners_lat/lon instead (a field with known GPS corners).
#
# The reference QR comes from qr_scanner_node: scanned in a phase 1 flight
# (stored in ~/aerothon_captures/reference_qr.json) and loaded with its
# reference_file parameter — this flight does not scan it.
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

import rclpy

from mission_manager.mission_node import MissionAbort, MissionNode
from path_planner.arena_frame import M_PER_DEG_LAT


class ArenaSearchMission(MissionNode):

    def __init__(self):
        super().__init__()
        # (The mission thread only reads them after MAVROS connects.)
        self.declare_parameter("arena_test_length_m", 25.0)   # ahead along the nose
        self.declare_parameter("arena_test_width_m", 25.0)    # to the right
        self.declare_parameter("arena_test_use_config", False)
        self.declare_parameter("arena_test_hover_s", 5.0)

    def _p(self, name):
        return self.get_parameter(name).value

    def _test_corners(self):
        """The test arena's GPS corners: home, then ahead, ahead + right, right."""
        lat0, lon0 = self.home
        h = math.radians(self.placed_heading_deg)
        fwd = (math.cos(h), math.sin(h))                 # (north, east)
        right = (-math.sin(h), math.cos(h))
        L, W = float(self._p("arena_test_length_m")), float(self._p("arena_test_width_m"))
        k = M_PER_DEG_LAT * math.cos(math.radians(lat0))

        def at(a, b):
            n, e = a * fwd[0] + b * right[0], a * fwd[1] + b * right[1]
            return (lat0 + n / M_PER_DEG_LAT, lon0 + e / k)
        return [at(0, 0), at(L, 0), at(L, W), at(0, W)]

    def _run_mission(self):
        try:
            self._wait_for_start()
            self.sm.auto_activate()
            self._preflight()
            self._takeoff()

            self.sm.airborne()
            self.sm.qr_scanned()            # no reference QR scan: loaded from file
            self.sm.mission_target_qr = ""  # the scanner holds the reference
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
