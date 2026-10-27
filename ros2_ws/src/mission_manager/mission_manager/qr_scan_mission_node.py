# qr_scan_mission_node.py — PHASE 1 test flight: gradual takeoff -> 5 m ->
# find + centre over the reference QR -> read + store it -> land. No
# corridor, no arena, no payload.
#
# Flies exactly the mission's phase 1 code, then lands instead of carrying
# on to the entry corridor:
#   1. wait for MAVROS (and the RC start switch if require_rc_switch)
#   2. GPS fix (+ obstacle sensor if required)                 (mission_node)
#   3. gradual takeoff to cruise_altitude_m                    (mission_node)
#   4. reference QR: find, centre, read; qr_scanner_node stores it in
#      ~/aerothon_captures/reference_qr.json + .png            (reference_qr)
#   5. hover qr_hover_after_s, LAND in place, wait for auto-disarm
#
# State machine: IDLE -> TAKEOFF -> SCAN_REFERENCE_QR -> LANDING
# (qr_test_done) -> LANDED.
# Aborts as mission_node: RC switch LOW -> LOITER; failure -> LAND; FC left
# GUIDED (e.g. pilot moves the flight-mode switch) -> stop commanding.
# A failed find/read is NOT an abort: it lands normally and reports it.
#
# Place the vehicle with the QR at most ~1 m ahead, nose pointing at it.

import rclpy

from mission_manager.mission_node import MissionAbort, MissionNode


class QrScanMissionNode(MissionNode):

    def __init__(self):
        super().__init__()
        # (The mission thread only reads it after MAVROS connects.)
        self.declare_parameter("qr_hover_after_s", 2.0)

    def _run_mission(self):
        try:
            self._wait_for_start()
            self.sm.auto_activate()
            self._preflight()
            self._takeoff()

            self.sm.airborne()
            self._sleep(1.0)
            text = self._reference_qr()
            if text is not None:
                self.sm.mission_target_qr = text            # shows in /mission/telemetry

            self._sleep(float(self.get_parameter("qr_hover_after_s").value))
            self.sm.qr_test_done()
            self._land_and_wait()
            self.sm.touchdown()
            self._mission_complete = True
            if text is None:
                result = "QR NOT read"
            else:
                result = repr(text) if text else "QR read — no text, stored by picture"
            self.get_logger().info(f"[qr_mission] Complete — landed. Result: {result}")
        except MissionAbort as a:
            self._handle_abort(a)
        except Exception as e:
            import traceback
            self.get_logger().error(f"[qr_mission] thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))


def main(args=None):
    rclpy.init(args=args)
    node = QrScanMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[qr_mission] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
