# takeoff_test.py — test flight for the mission's takeoff only: gradual
# takeoff -> hover takeoff_test_hover_s at cruise_altitude_m -> land.
#
# Same node as mission_node (same name, so the mission_node: section of
# mission_params.yaml applies unchanged) with the shortest sequence:
#   1. wait for MAVROS and the pilot's RC CH7 switch          (mission_node)
#   2. GPS fix (+ obstacle sensor if required)                 (mission_node)
#   3. gradual takeoff, exactly as the mission flies it        (mission_node):
#      position estimate steady on the ground, GUIDED, arm, sit ARM_WAIT_S
#      (drift still checked), lift off to takeoff_liftoff_alt_m, climb at
#      takeoff_climb_rate_ms to cruise_altitude_m holding the placed heading
#   4. hover takeoff_test_hover_s, LAND in place, wait for auto-disarm
# State machine: IDLE -> TAKEOFF -> LANDING (takeoff_test_done) -> LANDED.
# Aborts as mission_node: CH7 LOW -> LOITER; failure -> LAND; FC left
# GUIDED -> stop commanding.

import rclpy

from mission_manager.mission_node import MissionAbort, MissionNode


class TakeoffTest(MissionNode):

    def __init__(self):
        super().__init__()
        # (The mission thread only reads it after MAVROS connects.)
        self.declare_parameter("takeoff_test_hover_s", 10.0)

    def _run_mission(self):
        try:
            self._wait_for_start()
            self.sm.auto_activate()
            self._preflight()
            self._takeoff()

            hover = float(self.get_parameter("takeoff_test_hover_s").value)
            self.get_logger().info(f"[takeoff_test] Hovering {hover:.0f} s")
            self._sleep(hover)
            self.sm.takeoff_test_done()
            self._land_and_wait()
            self.sm.touchdown()
            self._mission_complete = True
            self.get_logger().info("[takeoff_test] Complete — landed")
        except MissionAbort as a:
            self._handle_abort(a)
        except Exception as e:
            import traceback
            self.get_logger().error(f"[takeoff_test] thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[takeoff_test] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
