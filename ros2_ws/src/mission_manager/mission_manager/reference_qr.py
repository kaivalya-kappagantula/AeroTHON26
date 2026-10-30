# reference_qr.py — mission phase 1, second half: find, centre over and read
# the reference QR from the air, and have qr_scanner_node store it.
#
# Shared by every mission node that flies it (mission_node, and the
# qr_scan_mission_node test flight), so the test flies exactly the code the
# mission does. The caller is a MissionNode that has already taken off to
# cruise_altitude_m over the launch point.
#
#   0. EXPECT: the QR lies at survey_qr (site survey, anchored on today's
#      home fix — mission_common/survey.py), else ~qr_initial_advance_m
#      (1 m) straight ahead of the takeoff point, along the heading the
#      drone was placed with. That spot is sent to qr_scanner_node (/qr_scanner/locate_hint): it uses
#      only a QR within qr_expected_radius_m of it (GPS drift allowed) and
#      the one nearest it — clutter elsewhere is never flown to.
#   1. FIND: /perception/locate_qr reports where a QR is relative to the
#      drone (detection only — works before the QR is readable, and for a QR
#      that holds no text). Only PART of the QR in view (cut by the frame
#      edge) counts: its centre is worked out from that part and the QR's
#      known size, and the drone flies straight there — no blind search.
#      It looks first from over the takeoff point: the
#      reference QR is placed ~1 m ahead, inside the frame at 5 m (+-1.9 m
#      along the nose, +-3.4 m across). Only if none is in view does it fly
#      to the surveyed spot (else qr_initial_advance_m ahead), then a little FORWARD or BACK of
#      that point (qr_search_offsets_m, in order) and look again. Those
#      blind steps (only when nothing of a QR is in view) go along the
#      compass heading the drone was placed with.
#   2. CENTRE over it, forward AND sideways, from what the camera sees:
#      locate -> move by the seen offset, re-measured every pass
#      (qr_center_passes). A wrong compass heading only turns each
#      correction a little off line — the next look catches what it missed —
#      instead of steering the drone diagonally away from the QR.
#   3. READ: /perception/scan_qr (same QR in 3 frames, by text or by
#      picture); a timed-out read re-centres and tries again
#      (qr_scan_attempts). qr_scanner_node stores the result — text ("" if
#      the QR holds none) and its module grid — in
#      ~/aerothon_captures/reference_qr.json + .png and on /qr/reference.
#
# Safety: no move ever takes the drone more than qr_max_radius_m from the
# takeoff point, and no single correction is longer than qr_max_jump_m
# (a longer one is cut to qr_max_jump_m in the same direction).
# A failed find/read is NOT an abort: run() returns None and the caller
# decides what happens next.

import math
import time

from std_msgs.msg import String

from mission_common import mavros_utils as mav
from mission_common import srv_helpers as srv

SRV_LOCATE_QR = "/perception/locate_qr"


def wait_stopped(node, cfg, label: str) -> None:
    """Wait until the drone has stopped: horizontal speed below
    qr_settle_speed_ms for qr_settle_hold_s, at most qr_settle_timeout_s.
    A goto counts as arrived within qr_move_tolerance_m — mid-move for short
    steps — and a camera look while moving (and tilted, and on a curved path
    when the compass is off) mis-measures the QR. Also used by the arena
    search before it looks at a match again (arena_search._refine)."""
    limit = float(cfg.qr_settle_speed_ms)
    hold = float(cfg.qr_settle_hold_s)
    timeout = float(cfg.qr_settle_timeout_s)
    log = node.get_logger()
    t0 = time.time()
    still_since = None
    while time.time() - t0 < timeout:
        node._tick()
        v = node.ground_speed
        now = time.time()
        if v is not None and v < limit:
            if still_since is None:
                still_since = now
            if now - still_since >= hold:
                log.info(f"{label}: stopped ({now - t0:.1f} s, {v:.2f} m/s)")
                return
        else:
            still_since = None
        time.sleep(0.1)
    v = node.ground_speed
    log.warn(f"{label}: still moving after {timeout:.0f} s "
             f"({'?' if v is None else f'{v:.2f}'} m/s) — looking anyway")


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
        max_jump = float(cfg.qr_max_jump_m)
        if dist > max_jump:
            # e.g. the centre of a QR only partly in view, estimated past the
            # frame: go qr_max_jump_m towards it — the next look sees more
            self._log().info(f"[reference_qr] {label}: {dist:.1f} m > qr_max_jump_m — "
                             f"moving {max_jump:.1f} m towards it")
            d_north, d_east = d_north * max_jump / dist, d_east * max_jump / dist
            dist = max_jump

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
        self._wait_stopped(label)         # settle before the camera looks again
        return True

    def _wait_stopped(self, label: str) -> None:
        wait_stopped(self.node, self.cfg, f"[reference_qr] {label}")

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

    def _search(self, ahead: float = 0.0):
        """Look here; none in view -> step ahead (compass heading), then a
        little forward / back along the nose until a QR is in view. Returns
        the first locate reply, or None."""
        found = self._locate()
        surveyed = self._surveyed_spot()
        if found is None and ahead > 0.0 and surveyed is not None and self.node.gps is not None:
            d_n, d_e = mav.ne_offset(*self.node.gps, *surveyed)
            self._log().info(f"[reference_qr] No QR in view — moving to its surveyed spot "
                             f"({math.hypot(d_n, d_e):.1f} m)")
            self._move_by(d_n, d_e, "to the surveyed QR")
            found = self._locate()
        elif found is None and ahead > 0.0:
            self._log().info(f"[reference_qr] No QR in view — moving {ahead:.1f} m ahead")
            self._step_forward(ahead)
            found = self._locate()
        at = 0.0                          # along-nose offset from where the search began
        for target in [float(x) for x in self.cfg.qr_search_offsets_m]:
            if found is not None:
                break
            self._log().info(f"[reference_qr] No QR in view — trying {target:+.1f} m "
                             f"along the nose")
            if not self._step_forward(target - at):
                break
            at = target
            found = self._locate()
        return found

    def _center(self, found) -> None:
        """Move by the offset the camera sees (forward and sideways), look
        again, up to qr_center_passes times. Each pass starts from a fresh
        look, so a heading error does not add up."""
        tol = float(self.cfg.qr_center_tol_m)
        for i in range(1, int(self.cfg.qr_center_passes) + 1):
            n, e, fwd, rgt = found
            off = math.hypot(fwd, rgt)
            if off <= tol:
                self._log().info(f"[reference_qr] Centred over the QR ({off:.2f} m off)")
                return
            if not self._move_by(n, e, f"centre pass {i}"):
                return
            found = self._locate()
            if found is None:
                self._log().warn("[reference_qr] Lost the QR after moving — reading here")
                return
        _, _, fwd, rgt = found
        self._log().warn(f"[reference_qr] Still {math.hypot(fwd, rgt):.2f} m off after "
                         f"{int(self.cfg.qr_center_passes)} passes — reading here")

    def _find_and_center(self, ahead: float = 0.0) -> bool:
        found = self._search(ahead)
        if found is None:
            self._log().warn("[reference_qr] QR not found")
            return False
        self._center(found)
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

    def _surveyed_spot(self):
        """(lat, lon) of survey_qr (anchored), or None without a survey."""
        survey = getattr(self.node, "survey", None)
        if survey is None or survey.qr is None:
            return None
        return survey.gps(survey.qr)

    def _expected_spot(self):
        """(lat, lon) the QR should be at: the surveyed spot, else
        qr_initial_advance_m ahead of takeoff along the placed heading, or
        None if either is unknown."""
        node = self.node
        if self._surveyed_spot() is not None:
            return self._surveyed_spot()
        if node.home is None or node.placed_heading_deg is None:
            return None
        d, h = float(self.cfg.qr_initial_advance_m), math.radians(node.placed_heading_deg)
        return mav.offset_gps(node.home[0], node.home[1], d * math.cos(h), d * math.sin(h))

    def _send_hint(self, spot) -> None:
        pub = getattr(self.node, "locate_hint_pub", None)
        if pub is None:
            return
        radius = float(self.cfg.qr_expected_radius_m)
        pub.publish(String(data="" if spot is None else f"{spot[0]:.8f},{spot[1]:.8f},{radius:.2f}"))
        if spot is not None:
            where = ("at its surveyed spot" if self._surveyed_spot() is not None else
                     f"{self.cfg.qr_initial_advance_m:.1f} m ahead of takeoff")
            self._log().info(f"[reference_qr] Expecting the QR {where} "
                             f"(using only a QR within {radius:.1f} m of it)")

    def run(self):
        """Find (from here first), centre, read. Returns the QR's text ("" =
        read, no text in it — matched by picture), or None if it was not
        found / not read."""
        self._send_hint(self._expected_spot())
        try:
            return self._run()
        finally:
            self._send_hint(None)          # other locates (payload drop) look anywhere

    def _run(self):
        if not self._find_and_center(float(self.cfg.qr_initial_advance_m)):
            return None
        text = self._read()
        if text is None:
            self._log().warn("[reference_qr] QR seen but NOT read")
        else:
            shown = f"'{text}'" if text else "(no text — stored by picture)"
            self._log().info(f"[reference_qr] REFERENCE QR READ AND STORED: {shown}")
        return text
