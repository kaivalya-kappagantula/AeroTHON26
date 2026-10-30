# exit_corridor_mission.py — PHASE 5 test flight (the exit corridor), two ways:
#
# exit_test_after_entry false (default; drone): placed FACING the exit
#   corridor, as for the entry corridor test: take off -> down to
#   corridor_start_alt_m over the takeoff point on the placed heading ->
#   /path_planner/exit_corridor (+-90 deg blue banner search around that
#   heading, line-up, through with obstacles passed beside, never climbing
#   over) -> hover -> land. No prediction, one search.
# exit_test_after_entry true (SITL, or a field with both corridors, a red
#   detector and arena_corners set): take off -> the mission's phase 2 (entry
#   corridor) -> the mission's phase 5 (MissionNode._exit_corridor_phase:
#   corridor 1 mirrored across the takeoff point, transit at arena_alt_m
#   around red, search, shifted searches) -> hover -> land. No arena search,
#   no drop.
#
# Same node as mission_node (same name, so the mission_node: section of
# mission_params.yaml applies unchanged). Aborts as mission_node: CH7 LOW ->
# LOITER; failure (banner never found, BLOCKED, ...) -> LAND in place.
# State machine: IDLE -> TAKEOFF -> SCAN_REFERENCE_QR -> ENTRY_CORRIDOR ->
# ARENA_SEARCH (passed through) -> CORRIDOR_2_TRANSIT -> LANDING
# (exit_test_done) -> LANDED.
#
#   drone: ros2 launch mission_bringup exit_corridor.launch.py takeoff_alt:=5
#   SITL:  ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true \
#            mission:=exit_corridor

import rclpy

from mission_common import srv_helpers as srv
from mission_manager.mission_node import MissionAbort, MissionNode


class ExitCorridorMission(MissionNode):

    def __init__(self):
        super().__init__()
        # (The mission thread only reads them after MAVROS connects.)
        self.declare_parameter("exit_test_hover_s", 5.0)
        self.declare_parameter("exit_test_after_entry", False)

    def _run_mission(self):
        try:
            self._wait_for_start()
            self.sm.auto_activate()
            self._preflight()
            self._takeoff()

            self.sm.airborne()
            self.sm.qr_scanned()            # no reference QR in this sequence
            if bool(self.get_parameter("exit_test_after_entry").value):
                self._entry_corridor_phase()
                self.sm.entry_complete()
                self.sm.search_exhausted()  # no arena search, no drop
                if not self._exit_corridor_phase():
                    return                  # failsafe: above the walls, home, landed
            else:
                self.sm.entry_complete()
                self.sm.search_exhausted()
                self._climb_in_place(self.cfg.corridor_start_alt_m, self.placed_yaw_enu())
                res = self._call_leg(srv.SRV_EXIT_CORRIDOR, self.cfg.exit_corridor_timeout_s)
                if res is None or not res.success:
                    self._exit_failsafe("no response from path_planner" if res is None
                                        else res.message)
                    return

            hover = float(self.get_parameter("exit_test_hover_s").value)
            self.get_logger().info(
                f"[exit_corridor_mission] Past the exit corridor — hovering {hover:.0f} s")
            self._sleep(hover)
            self.sm.exit_test_done()
            self._land_and_wait()
            self.sm.touchdown()
            self._mission_complete = True
            self.get_logger().info("[exit_corridor_mission] Complete — landed")
        except MissionAbort as a:
            self._handle_abort(a)
        except Exception as e:
            import traceback
            self.get_logger().error(f"[exit_corridor_mission] thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))


def main(args=None):
    rclpy.init(args=args)
    node = ExitCorridorMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[exit_corridor_mission] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
