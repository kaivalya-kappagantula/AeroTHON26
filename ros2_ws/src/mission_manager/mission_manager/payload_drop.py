# payload_drop.py — mission phase 4: the payload dropped on the matching QR.
#
# Shared by every mission node that flies it (mission_node, and the
# payload_drop_mission / arena_search_mission test flights), so the tests fly
# exactly the code the mission does. The caller is a MissionNode in the air
# (at arena_alt_m after phase 3) and hands over the QR's (lat, lon).
#
#   1. OVER IT at the current altitude (level; in the mission routed around
#      the red zones by the arena search's go_to). No route -> the payload is
#      KEPT (never over red).
#   2. DOWN straight to drop_altitude_m (no slanted descent) and wait for
#      the altitude (altitude_tolerance_m). Not reached in
#      drop_arrival_timeout_s -> logged, carries on.
#   3. CENTRE with the down camera: /perception/locate_qr (the largest QR in
#      view = the one below) -> move by its offset, up to drop_center_passes
#      times, until within drop_center_tol_m. A QR further than
#      drop_center_max_offset_m from the matched point (another QR?), a move
#      into a red zone's margin, or no QR seen -> no move: drop on GPS.
#   4. RELEASE: settle drop_settle_s, then /payload/release (winch off: the
#      thread unwinds and the payload drops; winch on again to reel the thread
#      in — held in place for all of it, up to drop_release_timeout_s). Settled
#      or not, the payload is released (user's choice: a rough drop scores,
#      carrying it home does not).
#   5. UP straight to arena_alt_m (above the other QRs and the red zone for
#      the way out).
# Returns True if payload_node confirmed the release.

import math

from mission_common import mavros_utils as mav
from mission_common import srv_helpers as srv

SRV_LOCATE_QR = "/perception/locate_qr"
SETTLE_AFTER_MOVE_S = 2.0       # before the camera looks again (as phase 1)


class PayloadDrop:

    def __init__(self, node):
        self.node = node
        self.cfg = node.cfg

    def _log(self):
        return self.node.get_logger()

    def _yaw(self):
        """Current heading as an ENU yaw (held through the drop)."""
        h = self.node.heading_deg
        return None if h is None else math.radians(90.0 - h)

    # -------------------------------------------------------------------

    def _locate(self):
        """(north, east, forward, right) of the QR below, or None."""
        try:
            res = srv.call_trigger(self.node, SRV_LOCATE_QR,
                                   call_timeout=float(self.cfg.qr_locate_timeout_s) + 5.0,
                                   should_abort=self.node._should_abort)
        except srv.CallAborted:
            self.node._tick()             # re-raise the MissionAbort behind it
            raise
        if res is None or not res.success:
            return None
        try:
            n, e, f, r, _alt = (float(x) for x in res.message.split(","))
        except ValueError:
            self._log().warn(f"[drop] Bad locate reply: '{res.message}'")
            return None
        return n, e, f, r

    def _descend(self, alt, yaw):
        node = self.node
        if node.gps is not None:
            mav.goto(node, node.gps[0], node.gps[1], alt,
                     publish_count=self.cfg.goto_publish_count, yaw_enu=yaw)
        ok = node._wait_for(
            lambda: node.rel_alt is not None
            and abs(node.rel_alt - alt) <= self.cfg.altitude_tolerance_m,
            float(self.cfg.drop_arrival_timeout_s), f"drop altitude {alt} m")
        if not ok:
            self._log().warn(f"[drop] {alt:.1f} m not reached — dropping from "
                             f"{node.rel_alt if node.rel_alt is not None else float('nan'):.1f} m")
        return ok

    def _centre(self, lat0, lon0, alt, yaw, clear):
        """Centre over the QR below with the camera. Returns True if centred."""
        node, cfg = self.node, self.cfg
        tol = float(cfg.drop_center_tol_m)
        max_off = float(cfg.drop_center_max_offset_m)
        for i in range(int(cfg.drop_center_passes)):
            found = self._locate()
            if found is None:
                self._log().warn("[drop] No QR in view — dropping on GPS")
                return False
            n, e, f, r = found
            off = math.hypot(f, r)
            self._log().info(f"[drop] pass {i + 1}: QR {f:+.2f} m forward, {r:+.2f} m right "
                             f"({off:.2f} m off)")
            if off <= tol:
                self._log().info("[drop] Centred over the QR")
                return True
            lat, lon = mav.offset_gps(node.gps[0], node.gps[1], n, e)
            if mav.haversine(lat0, lon0, lat, lon) > max_off:
                self._log().warn(f"[drop] That QR is > {max_off:.1f} m from the match "
                                 "(another QR?) — not moving, dropping here")
                return False
            if clear is not None and not clear(lat, lon):
                self._log().warn("[drop] Centring move would enter a red zone's margin — "
                                 "not moving, dropping here")
                return False
            node._goto_and_wait(lat, lon, alt, float(cfg.qr_move_tolerance_m),
                                max(15.0, 4.0 * off / max(float(cfg.qr_move_speed_ms), 0.1)),
                                float(cfg.qr_move_speed_ms), face_target=False,
                                hold_yaw_enu=yaw)
            node._sleep(SETTLE_AFTER_MOVE_S)
        self._log().warn("[drop] Not within tolerance after the last pass — dropping here")
        return False

    # -------------------------------------------------------------------

    def run(self, lat, lon, approach=None, clear=None) -> bool:
        """approach(lat, lon) -> bool flies over the QR at the current altitude
        (the mission: around the red zones); None = a straight level goto.
        clear(lat, lon) -> bool: may a centring move go there? None = yes."""
        node, cfg = self.node, self.cfg
        yaw = self._yaw()
        drop_alt = float(cfg.drop_altitude_m)
        node.sm.drop_position_reached()
        self._log().info(f"[drop] Phase 4: over the QR, down to {drop_alt:.1f} m, centre, release")

        # 1. over it
        if approach is not None:
            if not approach(lat, lon):
                self._log().error("[drop] No route to the QR clear of the red zones — "
                                  "KEEPING the payload")
                return False
            yaw = self._yaw()
        else:
            alt = node.rel_alt if node.rel_alt is not None else float(cfg.arena_alt_m)
            if not node._goto_and_wait(lat, lon, alt, float(cfg.drop_arrival_tolerance_m),
                                       float(cfg.drop_arrival_timeout_s),
                                       float(cfg.arena_speed_ms), face_target=False,
                                       hold_yaw_enu=yaw):
                self._log().warn("[drop] Not settled over the QR — descending here")

        # 2. straight down
        node._focus_camera(drop_alt)
        self._descend(drop_alt, yaw)

        # 3. centre with the camera
        centred = self._centre(lat, lon, drop_alt, yaw, clear)

        # 4. release
        node._sleep(float(cfg.drop_settle_s))
        off = mav.haversine(*node.gps, lat, lon) if node.gps is not None else float("nan")
        alt = f"{node.rel_alt:.2f}" if node.rel_alt is not None else "--"
        self._log().info(f"[drop] RELEASING at {alt} m, {off:.2f} m from the "
                         f"matched GPS point ({'camera-centred' if centred else 'GPS only'})")
        res = srv.call_trigger(node, srv.SRV_PAYLOAD_RELEASE,
                               call_timeout=float(self.cfg.drop_release_timeout_s))
        node.sm.drop_confirmed = bool(res is not None and res.success)
        if node.sm.drop_confirmed:
            self._log().info("[drop] Payload released")
        else:
            self._log().error("[drop] Payload release NOT confirmed"
                              + (f": {res.message}" if res is not None else " (no reply)"))

        # 5. straight back up
        node._climb_in_place(float(cfg.arena_alt_m), yaw)
        return node.sm.drop_confirmed
