# payload_drop_mission.py — PHASE 4 test flight (the payload drop) on its
# own: take off -> climb to arena_alt_m -> fly drop_test_ahead_m ahead along
# the nose -> find the QR below -> the mission's phase 4 (straight down to
# drop_altitude_m, camera-centred, release, straight back up to arena_alt_m)
# -> back over the takeoff point -> LAND.
#
# Flies exactly the mission's phase 4 code (mission_manager/payload_drop.py).
# Same node as mission_node (same name, so the mission_node: section of
# mission_params.yaml applies unchanged) with a shorter sequence:
#   1. wait for MAVROS and the pilot's RC CH7 switch          (mission_node)
#   2. GPS fix (+ obstacle sensor if required)                 (mission_node)
#   3. gradual takeoff holding the placed heading (armed ARM_WAIT_S first),
#      then the same steady climb on to arena_alt_m           (mission_node)
#   4. drop_test_ahead_m ahead along the placed heading, level; locate the
#      QR there (the largest in view). Not seen -> LAND at home, payload kept
#   5. phase 4 on it; then back over home at arena_alt_m and LAND
# There are no red zones in this test (no red_zone_node): the centring moves
# are only limited by drop_center_max_offset_m.
#
# State machine: IDLE -> TAKEOFF -> SCAN_REFERENCE_QR -> ENTRY_CORRIDOR ->
# ARENA_SEARCH (passed through) -> TARGET_FOUND -> DROPPING -> LANDING
# (drop_test_done) -> LANDED.
# Aborts as mission_node: CH7 LOW -> LOITER; failure -> LAND; FC left
# GUIDED -> stop commanding.
#
#   ros2 launch mission_bringup payload_drop_mission.launch.py        (drone)
#   ros2 launch mission_bringup payload_drop_mission.launch.py sim:=true

import math

import rclpy

from mission_common import mavros_utils as mav
from mission_manager.mission_node import MissionAbort, MissionNode
from mission_manager.payload_drop import PayloadDrop


class PayloadDropMission(MissionNode):

    def __init__(self):
        super().__init__()
        self.declare_parameter("drop_test_ahead_m", 5.0)    # the QR, ahead along the nose

    def _find_qr_ahead(self):
        """Level to drop_test_ahead_m ahead, locate the QR. (lat, lon) or None."""
        cfg = self.cfg
        ahead = float(self.get_parameter("drop_test_ahead_m").value)
        h = math.radians(self.placed_heading_deg)
        lat, lon = mav.offset_gps(self.home[0], self.home[1],
                                  ahead * math.cos(h), ahead * math.sin(h))
        self.get_logger().info(f"[payload_drop_mission] {ahead:.1f} m ahead to the QR")
        self._goto_and_wait(lat, lon, float(cfg.arena_alt_m), float(cfg.drop_arrival_tolerance_m),
                            float(cfg.drop_arrival_timeout_s), float(cfg.qr_move_speed_ms),
                            face_target=False, hold_yaw_enu=self.placed_yaw_enu())
        self._sleep(2.0)
        found = PayloadDrop(self)._locate()
        if found is None:
            return None
        n, e, f, r = found
        self.get_logger().info(
            f"[payload_drop_mission] QR seen {f:+.2f} m forward, {r:+.2f} m right")
        return mav.offset_gps(self.gps[0], self.gps[1], n, e)

    def _run_mission(self):
        try:
            self._wait_for_start()
            self.sm.auto_activate()
            self._preflight()
            arena_alt = float(self.cfg.arena_alt_m)
            self._focus_camera(arena_alt)
            self._takeoff(min(float(self.cfg.cruise_altitude_m), arena_alt))
            if self.rel_alt is not None and \
                    arena_alt - self.rel_alt > self.cfg.altitude_tolerance_m:
                self._climb_gradually(arena_alt)
            self.sm.airborne()
            self.sm.mission_target_qr = ""
            self.sm.qr_scanned()
            self.sm.entry_complete()        # no corridor, no sweep in this sequence

            target = self._find_qr_ahead()
            released = False
            if target is None:
                self.get_logger().warn("[payload_drop_mission] No QR found — keeping the "
                                       "payload, landing at home")
                self.sm.arena_test_done()
            else:
                self.sm.qr_matched()
                released = PayloadDrop(self).run(*target)
                self.sm.drop_test_done()
            self._goto_and_wait(*self.home, arena_alt, self.cfg.arrival_tolerance_m,
                                self.cfg.arrival_timeout_s, float(self.cfg.qr_move_speed_ms),
                                face_target=False, hold_yaw_enu=self.placed_yaw_enu())
            self._land_and_wait()
            self.sm.touchdown()
            self._mission_complete = True
            self.get_logger().info("[payload_drop_mission] Complete — "
                                   + ("payload released" if released else "payload NOT released"))
        except MissionAbort as a:
            self._handle_abort(a)
        except Exception as e:
            import traceback
            self.get_logger().error(f"[payload_drop_mission] thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))


def main(args=None):
    rclpy.init(args=args)
    node = PayloadDropMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[payload_drop_mission] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
