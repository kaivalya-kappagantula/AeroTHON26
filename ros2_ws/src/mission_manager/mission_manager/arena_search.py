# arena_search.py — mission phase 3: BCD sweep of the arena at arena_alt_m,
# never over red, until the QR that matches the stored reference is found.
#
# Shared by every mission node that flies it (mission_node, and the
# arena_search_mission test flight), so the test flies exactly the code the
# mission does. The caller is a MissionNode in the air.
#
#   1. plan: the arena from its 4 GPS corners (arena_frame.py); strips along
#      the side that makes the sweep shortest from where the drone is, strip
#      0 nearest it; strips arena_strip spacing apart so a whole
#      arena_qr_size_m QR is always in the frame between two strips (the
#      nose points along the strips: the image's wide axis looks across)
#   2. qr_scanner_node Phase 5 (/perception/start_qr_scan) matches every QR
#      in view against the reference stored in phase 1 (text when both have
#      one, else the picture) and publishes a match on /qr/detection
#      ("content|lat|lon", after its 3-frame gate). A match outside the arena
#      (e.g. the reference QR itself) is ignored and the scan restarted.
#   3. fly the strips at arena_sweep_speed_ms with the method proven in
#      bcd_sweep_mission.py (hold, then DO_CHANGE_SPEED, then short position
#      target bursts). Red zones are confirmed ON THE MOVE: red_zone_node
#      (grow_known_zones + confirm_in_flight) confirms red seen in enough
#      frames while the drone flies and publishes the outline on
#      /red_zone/zones -> the zone + arena_red_margin_m is cut out of the
#      arena and the rest of the sweep is re-planned around it without
#      stopping (sweep_replan.py; a strip piece that cannot be reached is
#      skipped; the drone brakes and moves out only if it ended up inside
#      the margin). Unconfirmed red that comes within near_m of the drone
#      (/red_zone/new_red_dist; near_m = margin + braking distance + buffer)
#      -> brake -> /red_zone/confirm (hover window), as bcd_redzone_mission.py.
#      Camera silent: fly on while what it last saw ahead still covers the
#      drone (blind_budget_s, from altitude, FOV, speed and near_m); longer
#      -> brake and hold; still blind after BLIND_HOLD_S -> abort.
#   4. on a match: brake, fly over its position (around the zones), hover,
#      scan again to refine it (arena_match_passes). Returns its (lat, lon).
#      Whole arena swept with no match -> None.
#   5. every QR in view while scanning (/qr/seen, matching or not) is kept:
#      random_qr() picks one inside the arena, clear of red, for the drop
#      when no match was found (mission_node).
#
# Hops stay inside the box around the arena and the drone (no padding: the
# arena edge is the fence). Altitude: arena_alt_m throughout.
#
# go_to(lat, lon): phase 5's transit to the exit corridor — the same flying
# (around the known zones, red camera watching, stops and re-plans) to one
# point, no sweep and no QR matching. Keeps the zones the sweep confirmed.
#
# Aborts are the mission's (_tick: RC switch, FC mode, battery, obstacle
# sensor): MissionAbort propagates. Failures that leave the drone unsafe to
# continue (speed rejected, arrival timeout) raise MissionAbort("failure")
# -> the mission lands in place, which is never over red: the drone only ever
# stops short of a zone. Red detector dead (after the hold) raises
# MissionAbort("sensor") -> RTL (LAND if RTL does not engage).

import json
import math
import random
import time

from shapely.geometry import Point
from shapely.ops import unary_union
from std_msgs.msg import Bool, Float32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mission_common import mavros_utils as mav
from mission_common import srv_helpers as srv
from mission_manager.reference_qr import wait_stopped
from path_planner.arena_frame import M_PER_DEG_LAT, best_sweep, strip_width_for
from path_planner.sweep_replan import (clip_segments, cut_at_progress, escape_point,
                                       exits_cleanly, plan_hops, polygon, routing_space,
                                       safe_path, sweep_area)

SRV_RED_ZONE_CONFIRM = "/red_zone/confirm"

# As bcd_redzone_mission.py (proven at 2 m; the camera sees far more ahead at
# 9 m, so they are only safer here)
CONFIRM_WINDOW_S = 4.0       # = red_zone_node window_s
BRAKE_SETTLE_S = 1.5
DETECTOR_TIMEOUT_S = 0.5     # /red_zone/sighting older than this = blind
                             # (start-up check; the sweep itself uses blind_budget_s)
BLIND_HOLD_S = 10.0
MAX_STOPS = 30
ARRIVAL_TOL_M = 0.3
MATCH_EDGE_TOL_M = 1.0       # a match this far outside the arena still counts
SEEN_MERGE_M = 1.5           # /qr/seen sightings this close are the same QR
SEEN_MIN_FRAMES = 3          # a seen QR counts once in this many frames (misreads)
# Braking distance = speed * BRAKE_LATENCY_S + speed^2 / (2 * BRAKE_DECEL_MS2)
BRAKE_LATENCY_S = 0.5        # frame -> red_zone_node -> mission -> FC
BRAKE_DECEL_MS2 = 1.0        # well under ArduCopter's guided deceleration
NEAR_BUFFER_M = 0.5          # on top of the margin and the braking distance
IMAGE_ASPECT = 720 / 1280    # Cam3 frame height / width: the look-ahead axis


class ArenaSearch:

    def __init__(self, node):
        self.node = node
        self.cfg = node.cfg
        self._det_t = 0.0            # last /red_zone/sighting (monotonic)
        self._sight_t = 0.0          # last unconfirmed red within near_m
        self._sight_after = 0.0      # sightings before this are handled
        self._red_dist = -1.0        # last /red_zone/new_red_dist (-1 = none)
        self.near_m = 0.0            # set in run()
        self.blind_budget_s = DETECTOR_TIMEOUT_S
        self._match = None           # (lat, lon) of a match inside the arena
        self._rescan = False         # a match outside the arena: scan again
        self._watch_match = False    # record matches (sweep + refine)
        self._refining = False       # flying over a match: matches do not stop it
        self._reference = None       # qr_scanner_node's stored reference (dict)
        self.seen = []               # [lat, lon, frames]: every QR seen while scanning
        self.frame = None
        self.zones = []              # confirmed red outlines, arena frame
        self.stops = 0
        self._zones_msg = None       # newest /red_zone/zones (dict)
        self._zones_ver = 0          # bumped by every /red_zone/zones message
        self._zones_used = 0         # version last applied
        node.create_subscription(Bool, "/red_zone/sighting", self._sighting_cb, 10)
        node.create_subscription(Float32, "/red_zone/new_red_dist", self._dist_cb, 10)
        node.create_subscription(String, "/qr/detection", self._qr_cb, 10)
        node.create_subscription(String, "/qr/seen", self._seen_cb, 50)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        node.create_subscription(String, "/qr/reference", self._reference_cb, latched)
        node.create_subscription(String, "/red_zone/zones", self._zones_cb, latched)

    def _log(self):
        return self.node.get_logger()

    # ── Callbacks (record only) ──────────────────────────────────────────
    def _sighting_cb(self, msg: Bool):
        self._det_t = time.monotonic()

    def _dist_cb(self, msg: Float32):
        # unconfirmed red only stops the drone once it is near
        self._red_dist = float(msg.data)
        if 0.0 <= self._red_dist <= self.near_m:
            self._sight_t = time.monotonic()

    def _zones_cb(self, msg: String):
        try:
            self._zones_msg = json.loads(msg.data)
        except ValueError:
            return
        self._zones_ver += 1

    def _reference_cb(self, msg: String):
        try:
            self._reference = json.loads(msg.data)
        except ValueError:
            pass

    def _qr_cb(self, msg: String):
        # "content|lat|lon", published by qr_scanner_node only for a confirmed
        # match against the reference
        if not self._watch_match or self.frame is None:
            return
        try:
            _, lat, lon = msg.data.rsplit("|", 2)
            lat, lon = float(lat), float(lon)
        except ValueError:
            self._log().warn(f"[arena] bad /qr/detection '{msg.data}'")
            return
        u, v = self.frame.from_gps(lat, lon)
        if not self.frame.contains(u, v, MATCH_EDGE_TOL_M):
            self._log().warn(f"[arena] match at ({u:.1f}, {v:.1f}) m is OUTSIDE the arena "
                             "(the reference QR?) — ignored")
            self._rescan = True
            return
        if self._match is None:
            self._log().info(f"[arena] MATCHING QR at ({u:.1f}, {v:.1f}) m in the arena")
        self._match = (lat, lon)

    def _seen_cb(self, msg: String):
        # "content|lat|lon": any QR in view while scanning (merged by position)
        try:
            _, lat, lon = msg.data.rsplit("|", 2)
            lat, lon = float(lat), float(lon)
        except ValueError:
            return
        for q in self.seen:
            if mav.haversine(q[0], q[1], lat, lon) <= SEEN_MERGE_M:
                n = q[2]
                q[0], q[1], q[2] = (q[0] * n + lat) / (n + 1), (q[1] * n + lon) / (n + 1), n + 1
                return
        self.seen.append([lat, lon, 1])

    def random_qr(self):
        """(lat, lon) of a random QR seen while scanning — inside the arena
        (not the reference QR beside it), seen in SEEN_MIN_FRAMES frames,
        clear of the red zones' margins — or None."""
        if self.frame is None:
            return None
        picks = [(q[0], q[1]) for q in list(self.seen) if q[2] >= SEEN_MIN_FRAMES
                 and self.frame.contains(*self.frame.from_gps(q[0], q[1]), MATCH_EDGE_TOL_M)
                 and self.clear_of_red(q[0], q[1])]
        self._log().info(f"[arena] {len(picks)} QR(s) to drop on at random "
                         f"({len(self.seen)} seen)")
        return random.choice(picks) if picks else None

    # ── Helpers ──────────────────────────────────────────────────────────
    def corners(self):
        """The arena's 4 GPS corners from the parameters; all zero = not set
        -> the rectangle arena_width_m (East) x arena_height_m (North) from
        (origin_lat, origin_lon). Surveyed with the rest of the site: moved
        by the same shift as the survey (anchored on today's home fix)."""
        c = self.cfg
        lats, lons = list(c.arena_corners_lat), list(c.arena_corners_lon)
        if len(lats) == 4 and len(lons) == 4 and any(lats) and any(lons):
            survey = getattr(self.node, "survey", None)
            d_e, d_n = survey.shift if survey is not None else (0.0, 0.0)
            return [mav.offset_gps(lat, lon, d_n, d_e) for lat, lon in zip(lats, lons)]
        lat0, lon0 = c.origin_lat, c.origin_lon
        k = M_PER_DEG_LAT * math.cos(math.radians(lat0))
        w, h = c.arena_width_m, c.arena_height_m
        self._log().warn("[arena] arena_corners_lat/lon not set — using the "
                         f"{w:.0f} x {h:.0f} m rectangle at origin_lat/lon")
        return [(lat0, lon0), (lat0, lon0 + w / k),
                (lat0 + h / M_PER_DEG_LAT, lon0 + w / k), (lat0 + h / M_PER_DEG_LAT, lon0)]

    def _here(self):
        return self.frame.from_gps(*self.node.gps)

    def _detector_live(self):
        return time.monotonic() - self._det_t <= DETECTOR_TIMEOUT_S

    def _blind_too_long(self):
        """Silent longer than what the camera last saw ahead covers."""
        silent = time.monotonic() - self._det_t
        if silent <= DETECTOR_TIMEOUT_S:
            return False
        budget = self.blind_budget_s
        if self._red_dist >= 0.0:          # unconfirmed red already in view
            budget = min(budget, max(0.0, (self._red_dist - self.near_m) / self.speed))
        return silent > max(DETECTOR_TIMEOUT_S, budget)

    def _new_sighting(self):
        return self._sight_t > self._sight_after

    def _wait_detector(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.node._tick()
            if self._detector_live():
                return True
            time.sleep(0.1)
        return False

    def _area(self):
        return sweep_area(0.0, 0.0, self.frame.width, self.frame.height, self.zones,
                          self.cfg.arena_red_margin_m, self.cfg.arena_edge_margin_m)

    def _space(self, *points):
        return routing_space(list(points), self._area(), self.zones,
                             self.cfg.arena_red_margin_m, pad=0.0)

    def _goto(self, u, v, publish_count=1):
        lat, lon = self.frame.to_gps(u, v)
        mav.goto(self.node, lat, lon, self.alt, publish_count=publish_count,
                 yaw_enu=self.yaw, log=False)
        return lat, lon

    def _start_scan(self):
        res = srv.call_trigger(self.node, srv.SRV_START_QR_SCAN, call_timeout=10.0)
        return res is not None and res.success

    def _extra(self):
        """A sweep-ending event, or None."""
        if not self._watch_match or self._refining:
            return None
        if self._rescan:
            self._rescan = False
            self._start_scan()
        return "target" if self._match is not None else None

    # ── Flying (bcd_redzone_mission.py's, in the arena frame) ────────────
    def _fly_to(self, u, v, label, watch=True):
        """Fly to (u, v). Returns "arrived", "sighting", "blind", "zones"
        (the known zone grew), "target". Arrival timeout -> MissionAbort."""
        lat, lon = self._goto(u, v)
        dist = mav.haversine(*self.node.gps, lat, lon)
        self._log().info(f"[arena] -> {label}: ({u:.1f}, {v:.1f}) m, {dist:.1f} m away")
        deadline = time.time() + 15.0 + 2.5 * dist / max(self.speed, 0.1)
        n = 0
        while time.time() < deadline:
            self.node._tick()
            if watch and self._new_sighting():
                return "sighting"
            if watch and self._blind_too_long():
                return "blind"
            if (watch and self._zones_ver != self._zones_used
                    and self._zones_msg and self._zones_msg.get("zones")):
                return "zones"
            extra = self._extra() if watch else None
            if extra is not None:
                return extra
            if mav.haversine(*self.node.gps, lat, lon) <= ARRIVAL_TOL_M:
                return "arrived"
            n += 1
            if n < 20 and n % 2 == 0:
                self._goto(u, v)
            time.sleep(0.05)
        raise self.node.abort_type("failure", f"arena: arrival timeout at {label}")

    def _brake(self):
        u, v = self._here()
        self._goto(u, v, publish_count=3)
        self._log().warn(f"[arena] BRAKE at ({u:.1f}, {v:.1f}) m")
        self.node._sleep(BRAKE_SETTLE_S)
        return u, v

    def _confirm(self):
        """Hover-confirm via red_zone_node; refresh self.zones."""
        try:
            res = srv.call_trigger(self.node, SRV_RED_ZONE_CONFIRM,
                                   call_timeout=CONFIRM_WINDOW_S + 15.0,
                                   should_abort=self.node._should_abort)
        except srv.CallAborted:
            self.node._tick()
            raise
        self._sight_after = time.monotonic() + 0.2
        if res is None or not res.success:
            raise self.node.abort_type("failure", "arena: red-zone confirmation failed")
        self._apply_zones(json.loads(res.message))
        self._zones_used = self._zones_ver      # the confirm also published them

    def _apply_zones(self, data):
        """Merge red_zone_node's zones (lat/lon) into self.zones."""
        polys = list(self.zones)        # zones are only ever added
        for z in data["zones"]:
            poly = polygon([self.frame.from_gps(lat, lon) for lat, lon in z])
            if not poly.is_empty:
                polys.append(poly)
        merged = unary_union(polys) if polys else None
        zones = [] if merged is None else list(getattr(merged, "geoms", [merged]))
        self.zones = [z for z in zones if z.geom_type == "Polygon"]
        self._log().info(
            f"[arena] confirmed +{data['new_area_m2']:.2f} m^2 red, {len(self.zones)} zone(s)"
            + "".join(f"\n    zone {i}: centre ({z.centroid.x:.1f}, {z.centroid.y:.1f}) m, "
                      f"{z.area:.1f} m^2" for i, z in enumerate(self.zones)))

    def _handle_stop(self, reason):
        """Brake, confirm (or wait out blindness), get out of the margin.
        Returns the start point for the re-plan."""
        if reason == "sighting":
            self.stops += 1
            if self.stops > MAX_STOPS:
                raise self.node.abort_type("failure", f"arena: more than {MAX_STOPS} red stops")
        stop = self._brake()
        if reason == "blind":
            self._log().warn(f"[arena] red camera silent — holding up to {BLIND_HOLD_S:.0f} s")
            if not self._wait_detector(BLIND_HOLD_S):
                raise self.node.abort_type("sensor", "arena: red-zone camera silent")
            self.node._sleep(0.5)       # let fresh frames report what is ahead
            if self._new_sighting():
                reason = "sighting"
        if reason == "sighting":
            self._log().warn("[arena] NEW RED IN VIEW — hover-confirming")
            self._confirm()
        if any(z.covers(Point(stop)) for z in self.zones):
            self._log().error("[arena] !!! stopped OVER a confirmed red zone")
        start = escape_point(stop, self._space(stop))
        if start is None:
            raise self.node.abort_type("failure", "arena: no flyable area left")
        if math.dist(start, stop) > 1e-3:
            if not exits_cleanly(stop, start, self.zones):
                raise self.node.abort_type("failure", "arena: cannot leave the margin "
                                                      "without crossing red")
            self._log().warn(f"[arena] inside the {self.cfg.arena_red_margin_m:.1f} m margin "
                             f"— moving out {math.dist(start, stop):.2f} m")
            self._fly_to(start[0], start[1], "out of margin", watch=False)
            self._sight_after = time.monotonic() + 0.2
        return start

    def _handle_growth(self):
        """A zone was confirmed or grew in flight: take the outline. No brake
        unless the drone is now inside the margin (then out of it, as after
        a stop). Returns the start point for the re-plan."""
        ver = self._zones_ver
        self._apply_zones(self._zones_msg)
        self._zones_used = ver
        here = self._here()
        start = escape_point(here, self._space(here))
        if start is None:
            raise self.node.abort_type("failure", "arena: no flyable area left")
        if math.dist(start, here) > 0.3:
            stop = self._brake()
            start = escape_point(stop, self._space(stop))
            if not exits_cleanly(stop, start, self.zones):
                raise self.node.abort_type("failure", "arena: cannot leave the margin "
                                                      "without crossing red")
            self._log().warn(f"[arena] the red zone reaches the drone's margin — moving out "
                             f"{math.dist(start, stop):.2f} m")
            self._fly_to(start[0], start[1], "out of margin", watch=False)
            self._sight_after = time.monotonic() + 0.2
        # else: at most 0.3 m into the margin (the route hugs it and the
        # outline grew a little) — plan from the escape point, no brake;
        # planning from inside the margin finds no route at all
        return start

    def _fly_route(self, start, goal, label):
        """start -> goal around the zones, re-planning after every red stop.
        Returns "arrived", "unreachable" or "target"."""
        while True:
            path = safe_path(start, goal, self._space(start, goal))
            if path is None:
                return "unreachable"
            for u, v in path[1:]:
                r = self._fly_to(u, v, label)
                if r == "arrived":
                    start = (u, v)
                    continue
                if r in ("sighting", "blind"):
                    start = self._handle_stop(r)
                    break
                if r == "zones":
                    start = self._handle_growth()
                    break
                return r
            else:
                return "arrived"

    def _enter_speed(self):
        """Hold (enters the position-target submode, which resets the speed
        limit), then DO_CHANGE_SPEED — bcd_sweep_mission.py's order. Refuse
        to sweep at the FC's default WP_SPD."""
        u, v = self._here()
        self._goto(u, v, publish_count=5)
        if not mav.change_speed(self.node, self.speed):
            raise self.node.abort_type("failure", f"arena: FC rejected {self.speed:.2f} m/s")

    # ── The sweep ────────────────────────────────────────────────────────
    def _sweep(self, segments, deadline):
        """Fly the strips; returns "target" (braked) or "done"."""
        remaining = list(segments)
        here = self._here()
        start = escape_point(here, self._space(here))
        while True:
            if time.time() > deadline:
                self._log().error("[arena] arena time limit — ending the sweep")
                return "done"
            remaining = clip_segments(remaining, self._area())
            hops, skipped = plan_hops(start, remaining, start, self._space(start))
            hops = [h for h in hops if h.label not in ("HOME", "to HOME")]
            for s in skipped:
                self._log().warn(f"[arena] strip {s.strip} piece unreachable — skipped")
            remaining = [s for s in remaining if not any(s is k for k in skipped)]
            self._log().info(f"[arena] plan: {len(remaining)} strip piece(s), {len(hops)} hop(s)")
            stopped = None
            for hop in hops:
                r = self._fly_to(hop.x, hop.y, hop.label)
                if r == "arrived":
                    if hop.seg is not None:
                        remaining = [s for s in remaining if s is not hop.seg]
                    continue
                stopped = (r, hop)
                break
            if stopped is None:
                return "done"
            reason, hop = stopped
            if reason == "target":
                self._brake()
                return "target"
            start = (self._handle_growth() if reason == "zones"
                     else self._handle_stop(reason))
            if hop.seg is not None:          # that strip continues from here
                cut = cut_at_progress(hop.seg, start)
                remaining = [cut if s is hop.seg else s for s in remaining
                             if s is not hop.seg or cut is not None]

    def _refine(self):
        """Over the match, scan again; up to arena_match_passes times."""
        best = self._match
        for i in range(int(self.cfg.arena_match_passes)):
            goal = self.frame.from_gps(*best)
            # from just outside the margin, as the sweep plans: braked next to
            # a zone the drone may sit a few cm inside it, and a route planned
            # from in there finds no way at all ("unreachable" in SITL)
            here = self._here()
            start = escape_point(here, self._space(here, goal))
            r = ("unreachable" if start is None
                 else self._fly_route(start, goal, "over the matching QR"))
            if r == "unreachable":
                self._log().warn("[arena] matching QR unreachable around the red zones")
                break
            # look only once stopped, as phase 1 does: a moving, tilted camera
            # puts the QR in the wrong place
            wait_stopped(self.node, self.cfg, f"[arena] pass {i + 1}")
            self._match = None
            if not self._start_scan():
                break
            end = time.time() + float(self.cfg.arena_rescan_timeout_s)
            while self._match is None and time.time() < end:
                self.node._tick()
                if self._rescan:
                    self._rescan = False
                    self._start_scan()
                time.sleep(0.1)
            if self._match is None:
                self._log().warn(f"[arena] pass {i + 1}: match not seen again — keeping "
                                 "the last position")
                break
            off = mav.haversine(*self.node.gps, *self._match)
            best = self._match
            self._log().info(f"[arena] pass {i + 1}: match {off:.2f} m from the drone")
            if off <= float(self.cfg.arena_match_tol_m):
                break
        return best

    def _setup(self, corners=None):
        """Arena frame, altitude, speed and the red-stop distances (zones
        kept). Returns the sweep plan and how far ahead the camera sees."""
        cfg, node = self.cfg, self.node
        self.alt = float(cfg.arena_alt_m)
        self.speed = float(cfg.arena_sweep_speed_ms)
        brake_m = self.speed * BRAKE_LATENCY_S + self.speed ** 2 / (2.0 * BRAKE_DECEL_MS2)
        self.near_m = float(cfg.arena_red_margin_m) + brake_m + NEAR_BUFFER_M
        ahead_m = (self.alt * math.tan(math.radians(cfg.down_cam_hfov_deg) / 2.0)
                   * IMAGE_ASPECT)
        self.blind_budget_s = max(DETECTOR_TIMEOUT_S, (ahead_m - self.near_m) / self.speed)
        w_max = strip_width_for(self.alt, cfg.down_cam_hfov_deg, cfg.arena_qr_size_m)
        sweep = best_sweep(corners or self.corners(), node.gps, w_max, cfg.arena_edge_margin_m)
        self.frame = sweep.frame
        self.yaw = self.frame.u_yaw_enu          # nose along the strips
        return sweep, ahead_m

    def _start_flying(self):
        """Up to arena_alt_m where the drone is; the red detector must be live."""
        node = self.node
        node._climb_in_place(self.alt, self.yaw)
        # ATTITUDE + GLOBAL_POSITION_INT feed red_zone_node's projection: at
        # 20 Hz the telemetry gaps halve (a stale gap > 0.75 s silences it
        # and the sweep brakes "blind"). Runtime request, as
        # bcd_redzone_mission.py does — not a parameter.
        mav.request_message_intervals(node, (30, 33), rate_hz=20.0)
        if not self._wait_detector(10.0):
            raise node.abort_type("sensor", "arena: red_zone_node not reporting on "
                                             "/red_zone/sighting — not flying blind")

    def clear_of_red(self, lat, lon):
        """May the drone go to (lat, lon)? Outside every red zone's margin and
        inside the routing space (phase 4's centring moves). No arena planned
        yet -> True."""
        if self.frame is None:
            return True
        p = self.frame.from_gps(lat, lon)
        return self._space(p).covers(Point(p))

    def go_to(self, lat, lon, label):
        """Fly to (lat, lon) at arena_alt_m around the known red zones, the red
        camera watching (stop, confirm, re-plan as in the sweep). Returns True
        on arrival, False if no route (e.g. the point is inside a zone's
        margin)."""
        if self.frame is None:
            self._setup()
        self._start_flying()
        self._enter_speed()
        self._sight_after = time.monotonic()
        here = self._here()
        start = escape_point(here, self._space(here))
        goal = self.frame.from_gps(lat, lon)
        r = self._fly_route(start, goal, label)
        if r != "arrived":
            self._log().warn(f"[arena] {label}: no route around the red zones")
        return r == "arrived"

    def run(self, corners=None, until=None):
        """Phase 3. Returns the matching QR's (lat, lon), or None.
        until: time.time() by which the sweep must end (the mission clock);
        lawnmower_timeout_s applies too."""
        cfg, node = self.cfg, self.node
        self.zones, self.stops, self._match, self._rescan = [], 0, None, False
        self.seen = []
        sweep, ahead_m = self._setup(corners)
        self._log().info(
            f"[arena] {self.frame.width:.0f} x {self.frame.height:.0f} m arena: "
            f"{len(sweep.segments)} strips {sweep.strip_width:.1f} m apart along "
            f"{math.degrees(self.yaw):.0f} deg ENU, {sweep.length:.0f} m to fly at "
            f"{self.speed:.2f} m/s, {self.alt:.0f} m; red margin "
            f"{cfg.arena_red_margin_m:.1f} m; brakes for unconfirmed red within "
            f"{self.near_m:.2f} m (camera sees {ahead_m:.2f} m ahead), flies through "
            f"camera gaps up to {self.blind_budget_s:.1f} s")
        ref = self._reference
        if ref is None:
            self._log().warn("[arena] no reference on /qr/reference — qr_scanner_node may "
                             "have nothing to match against")
        else:
            self._log().info(f"[arena] matching against the reference "
                             f"({repr(ref.get('text')) if ref.get('text') else 'picture only'}, "
                             f"stored {ref.get('time')})")
        deadline = time.time() + float(cfg.lawnmower_timeout_s)
        if until is not None:
            deadline = min(deadline, until)

        node._focus_camera(self.alt)        # fixed focus (no autofocus on the Cam3)
        self._start_flying()
        target = node.sm.mission_target_qr
        if target:
            node.delivery_id_pub.publish(String(data=target))
        # Listen BEFORE the scan starts: a QR already under the drone (the
        # reference, read just before) is confirmed within a few frames, and
        # a match dropped here was never re-scanned — the scanner stopped and
        # the whole sweep went without matching (SITL, phase 1 + 3 at 3 m).
        self._watch_match = True
        try:
            if not self._start_scan():
                self._log().error("[arena] qr_scanner_node did not start the arena scan — "
                                  "sweeping without QR matching")
            self._enter_speed()
            self._sight_after = time.monotonic()
            end = self._sweep(sweep.segments, deadline)
            if end != "target":
                self._log().warn(f"[arena] swept the arena: NO matching QR "
                                 f"({len(self.zones)} red zone(s), {self.stops} stop(s))")
                return None
            self._refining = True
            found = self._refine()
        finally:
            self._watch_match = self._refining = False
        u, v = self.frame.from_gps(*found)
        self._log().info(f"[arena] MATCHING QR at ({u:.1f}, {v:.1f}) m — "
                         f"{len(self.zones)} red zone(s), {self.stops} red stop(s)")
        return found
