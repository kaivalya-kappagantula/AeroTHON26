# entry_corridor_mission.py — PHASE 2 test flight (the entry corridor) on its
# own, without phase 1: take off -> the mission's phase 2 -> hover -> land.
#
# Flies exactly the mission's phase 2 code (MissionNode._entry_corridor_phase),
# straight after the takeoff instead of after the reference QR. Same node as
# mission_node (same name, so the mission_node: section of mission_params.yaml
# applies unchanged) with a shorter sequence:
#   1. wait for MAVROS and the pilot's RC CH7 switch          (mission_node)
#   2. GPS fix + obstacle sensor live                          (mission_node)
#   3. gradual takeoff to cruise_altitude_m (launch arg takeoff_alt on the
#      drone), holding the heading it was placed with           (mission_node)
#   4. phase 2: surveyed entry corridor -> over it at cruise_altitude_m to
#      the midpoint of its exit corners; not surveyed -> over the takeoff
#      point, down to corridor_start_alt_m, then /path_planner/entry_corridor
#      (green banner search, line-up, through the middle until the walls end,
#      path_planner/entry_corridor.py) — then straight up to
#      post_entry_alt_m                                          (mission_node)
#   5. hover entry_test_hover_s, LAND in place, wait for auto-disarm
# State machine: IDLE -> TAKEOFF -> SCAN_REFERENCE_QR (passed through, no
# scan) -> ENTRY_CORRIDOR -> LANDING (entry_test_done) -> LANDED.
# Aborts as mission_node: CH7 LOW -> LOITER; failure -> LAND; FC left
# GUIDED -> stop commanding.
#
# The service blocks for the whole leg (search + approach + corridor);
# entry_corridor_timeout_s only bounds how long this node waits for it — the
# planner enforces each phase's own deadline.
#
#   drone: ros2 launch mission_bringup entry_corridor.launch.py takeoff_alt:=5
#   SITL:  ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true \
#            mission:=entry_corridor

import rclpy

from mission_manager.mission_node import MissionAbort, MissionNode


class EntryCorridorMission(MissionNode):

    def __init__(self):
        super().__init__()
        # (The mission thread only reads it after MAVROS connects.)
        self.declare_parameter("entry_test_hover_s", 5.0)

    def _run_mission(self):
        try:
            self._wait_for_start()
            self.sm.auto_activate()
            self._preflight()
            self._takeoff()

            self.sm.airborne()
            self.sm.qr_scanned()            # no reference QR in this sequence
            self._entry_corridor_phase()

            hover = float(self.get_parameter("entry_test_hover_s").value)
            self.get_logger().info(
                f"[entry_corridor_mission] Past the corridor — hovering {hover:.0f} s")
            self._sleep(hover)
            self.sm.entry_test_done()
            self._land_and_wait()
            self.sm.touchdown()
            self._mission_complete = True
            self.get_logger().info("[entry_corridor_mission] Complete — landed")
        except MissionAbort as a:
            self._handle_abort(a)
        except Exception as e:
            import traceback
            self.get_logger().error(f"[entry_corridor_mission] thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))


def main(args=None):
    rclpy.init(args=args)
    node = EntryCorridorMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[entry_corridor_mission] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
