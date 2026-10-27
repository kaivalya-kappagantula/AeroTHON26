# reference_qr.py — mission phase 1, second half: find, centre over and read
# the reference QR from the air, and have qr_scanner_node store it.
#
# Shared by every mission node that flies it (mission_node, and the
# qr_scan_mission_node test flight), so the test flies exactly the code the
# mission does. The caller is a MissionNode that has already taken off to
# cruise_altitude_m over the launch point.
#
#   0. AHEAD: fly qr_initial_advance_m (1 m) forward along the heading the
#      drone was placed with — the reference QR is placed that far in front
#      of it — so the search starts over it.
#   1. FIND: /perception/locate_qr reports where a QR is relative to the
#      drone (detection only — works before the QR is readable, and for a QR
#      that holds no text). None in view -> hover a little FORWARD or BACK of
#      that point along the placed heading, never sideways
#      (qr_search_offsets_m, in order) and look again. The camera's short
#      side runs along the nose (+-1.15 m at 3 m), its long side across.
#   2. CENTRE over it, along the nose only (forward/back): locate -> move,
#      qr_center_passes times. Sideways it only has to be in the frame.
#   3. READ: /perception/scan_qr (same QR in 3 frames, by text or by
#      picture); a timed-out read re-centres and tries again
#      (qr_scan_attempts). qr_scanner_node stores the result — text ("" if
#      the QR holds none) and its module grid — in
#      ~/aerothon_captures/reference_qr.json + .png and on /qr/reference.
#
# Safety: no move ever takes the drone more than qr_max_radius_m from the
# takeoff point, and no single correction is longer than qr_max_jump_m.
# A failed find/read is NOT an abort: run() returns None and the caller
# decides what happens next.

import math

from mission_common import mavros_utils as mav
from mission_common import srv_helpers as srv

SRV_LOCATE_QR = "/perception/locate_qr"


class ReferenceQr:

    def __init__(self, node):
        self.node = node
        self.cfg = node.cfg

    def _log(self):
        return self.node.get_logger()

    # -------------------------------------------------------------------
    # Moves
    # -------------------------------------------------------------------

    def _move_by(self, d_north: float, d_east: float, label: str) -> bool:
        """Short GUIDED move by a world offset, clamped for safety. Never
        aborts on a missed arrival — GPS is only good to ~1 m."""
        node, cfg = self.node, self.cfg
        if node.gps is None:
            self._log().warn(f"[reference_qr] {label}: no GPS — not moving")
            return False
        dist = math.hypot(d_north, d_east)
        if dist > float(cfg.qr_max_jump_m):
            self._log().warn(f"[reference_qr] {label}: {dist:.1f} m > qr_max_jump_m — not moving")
            return False

        lat, lon = mav.offset_gps(node.gps[0], node.gps[1], d_north, d_east)
        if node.home is not None:
            from_home = mav.haversine(node.home[0], node.home[1], lat, lon)
            if from_home > float(cfg.qr_max_radius_m):
                self._log().warn(
                    f"[reference_qr] {label}: target {from_home:.1f} m from takeoff "
                    f"> qr_max_radius_m — not moving")
                return False

        speed = float(cfg.qr_move_speed_ms)
        self._log().info(f"[reference_qr] {label}: moving N={d_north:+.2f} m E={d_east:+.2f} m")
        # hold the placed heading: the banner search after this phase is
        # centred on it (with yaw left free, GUIDED turns the nose to the target)
        if not node._goto_and_wait(
                lat, lon, cfg.cruise_altitude_m, float(cfg.qr_move_tolerance_m),
                max(15.0, 4.0 * dist / max(speed, 0.1)), speed, face_target=False,
                hold_yaw_enu=node.placed_yaw_enu()):
            self._log().warn(f"[reference_qr] {label}: arrival not confirmed — carrying on")
        node._sleep(2.0)                  # settle before the camera looks again
        return True

    def _step_forward(self, dist_m: float) -> bool:
        hdg = self.node.placed_heading_deg
        if hdg is None:
            self._log().warn("[reference_qr] No placement heading — cannot step forward")
            return False
        h = math.radians(hdg)
        return self._move_by(dist_m * math.cos(h), dist_m * math.sin(h), "search step")

    # -------------------------------------------------------------------
    # Perception calls
    # -------------------------------------------------------------------

    def _call(self, name: str, timeout: float):
        try:
            return srv.call_trigger(self.node, name, call_timeout=timeout,
                                    should_abort=self.node._should_abort)
        except srv.CallAborted:
            self.node._tick()             # re-raise the MissionAbort behind it
            raise

    def _locate(self):
        """Returns (north, east, forward, right) of the QR from the drone, or None."""
        res = self._call(SRV_LOCATE_QR, float(self.cfg.qr_locate_timeout_s) + 5.0)
        if res is None or not res.success:
            return None
        try:
            n, e, f, r, _alt = (float(x) for x in res.message.split(","))
        except ValueError:
            self._log().warn(f"[reference_qr] Bad locate reply: '{res.message}'")
            return None
        self._log().info(
            f"[reference_qr] QR seen {f:+.2f} m forward, {r:+.2f} m right "
            f"(N={n:+.2f} E={e:+.2f})")
        return n, e, f, r

    def _find_and_center(self) -> bool:
        """Look here, then a little forward / back along the nose until a QR
        is in view; then centre over it along the nose."""
        cfg = self.cfg
        found = self._locate()
        at = 0.0                          # along-nose offset from where the search began
        for target in [float(x) for x in cfg.qr_search_offsets_m]:
            if found is not None:
                break
            self._log().info(f"[reference_qr] No QR in view — trying {target:+.1f} m "
                             f"along the nose")
            if not self._step_forward(target - at):
                break
            at = target
            found = self._locate()
        if found is None:
            self._log().warn("[reference_qr] QR not found")
            return False

        tol = float(cfg.qr_center_tol_m)
        for i in range(int(cfg.qr_center_passes)):
            _, _, fwd, _ = found
            if abs(fwd) <= tol:
                self._log().info("[reference_qr] Centred over the QR (along the nose)")
                return True
            self._step_forward(fwd)
            found = self._locate()
            if found is None:
                self._log().warn("[reference_qr] Lost the QR after moving — reading here")
                return True
        return True

    def _read(self):
        """/perception/scan_qr, re-centring between attempts. Returns the
        QR's text ("" if it holds none), or None if it was not read."""
        attempts = int(self.cfg.qr_scan_attempts)
        for attempt in range(1, attempts + 1):
            self._log().info(f"[reference_qr] QR read attempt {attempt}/{attempts}")
            res = self._call(srv.SRV_SCAN_QR, self.cfg.qr_scan_timeout_s + 10.0)
            if res is not None and res.success:
                return res.message
            if attempt < attempts:
                self._find_and_center()
        return None

    # -------------------------------------------------------------------

    def run(self):
        """Fly ahead over it, find, centre, read. Returns the QR's text ("" =
        read, no text in it — matched by picture), or None if it was not
        found / not read."""
        ahead = float(self.cfg.qr_initial_advance_m)
        if ahead > 0.0:
            self._log().info(f"[reference_qr] Moving {ahead:.1f} m ahead over the reference QR")
            self._step_forward(ahead)
        if not self._find_and_center():
            return None
        text = self._read()
        if text is None:
            self._log().warn("[reference_qr] QR seen but NOT read")
        else:
            shown = f"'{text}'" if text else "(no text — stored by picture)"
            self._log().info(f"[reference_qr] REFERENCE QR READ AND STORED: {shown}")
        return text
