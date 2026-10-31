# mission_node.py — ROS2 node that owns and drives the Mission 2 sequence
#
# The whole mission, one command (fly.sh / mission.launch.py), phase by phase.
# Every phase begins and ends on the HARDCODED waypoints in
# mission_params.yaml (/** wp_* block, from the site survey; shifted onto
# today's home fix by mission_common/survey.py). Only the phases that need
# logic run any; the rest are plain waypoints:
#
#   0 preflight      GPS fix, waypoints checked, downward camera up
#   1 takeoff_qr     takeoff over wp_home -> wp_qr at 5 m -> find, centre,
#                    read the reference QR (LOGIC: reference_qr.py)
#   2 entry_corridor waypoints, THROUGH the corridor (GPS only, no obstacle
#                    sensor): entry_start_standoff_m in front of
#                    wp_entry_mouth at 5 m -> straight down to 1.22 m ->
#                    wp_entry_mouth -> wp_entry_end -> corridor_clear_m past
#                    it -> straight up to 9.25 m
#   3 arena_search   BCD sweep of the arena around the red mat until the
#                    matching QR (LOGIC: arena_search.py + red_zone_node)
#   4 payload_drop   over the QR, down to 5 m, centre, release, back up
#                    (LOGIC: payload_drop.py + payload_node)
#   5 exit_corridor  waypoints, OVER the corridor: exit_search_standoff_m in
#                    front of wp_exit_mouth (around red at 9.25 m) -> down to
#                    exit_overfly_alt_m (5 m) -> wp_exit_mouth -> wp_exit_end
#   6 home_land      5 m -> wp_home -> land
#
# Nodes per phase (mission_manager/stages.py): MAVROS, payload_node and
# qr_scanner_node run throughout; the downward camera (with its live view)
# from preflight to landing; red_zone_node from phase 3 to the exit
# corridor's start point. No forward / obstacle camera.
# Logs: one text file per phase (phase_log.py).
#
# Architecture:
#   - rclpy.spin() runs in the main thread. Every ROS callback here only
#     records data / sets flags and returns immediately — no service calls,
#     no state transitions that do work. (A blocking callback would freeze the
#     single-threaded executor, and with it every service response.)
#   - The mission runs linearly in a worker thread (_run_mission). It calls
#     services and sleep-polls their futures while the main thread spins.
#   - Abort conditions (FC left GUIDED, battery critical, red camera dead)
#     are checked continuously by _tick() inside every wait, so the mission can
#     be stopped mid-leg, not only between phases.
#   - PHASE FAILSAFE: a failure inside a phase (leg failed, timeout, crash)
#     moves the mission on to the next phase (_phase); the FC, battery and
#     dead-sensor aborts still end it. The exit corridor's own failsafe
#     (straight up to 5 m, home, land) is its "next phase".
#   - Failsafes (battery, fence, GCS/radio loss, EKF) are ArduPilot's job. When
#     the FC switches out of GUIDED for any reason, the mission stops commanding.
#
# Usage:
#   ros2 run mission_manager mission_node

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State as MavrosState, RCIn
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus

from mission_common import mavros_utils as mav
from mission_common import survey as site_survey
from mission_common.config_loader import load_mission_config
from mission_manager.arena_search import ArenaSearch
from mission_manager.payload_drop import PayloadDrop
from mission_manager.phase_log import PhaseLog
from mission_manager.reference_qr import ReferenceQr
from mission_manager.stages import Stages
from mission_manager.state_machine import MissionSM

# Sit armed on the ground this long before every takeoff command (precaution).
# Keep it under ArduPilot's DISARM_DELAY (default 10 s) or the FC disarms on
# its own first.
ARM_WAIT_S = 5.0
# RTL failsafe: the FC must report RTL within this, else we command LAND.
RTL_CONFIRM_S = 3.0


class MissionAbort(Exception):
    """Raised inside the mission thread to leave the sequence.

    kind:
      'pilot'   — RC auto switch low: pilot takes over (LOITER).
      'battery' — battery critical: ArduPilot BATT_FS is primary; if it has
                  not taken over (still GUIDED) we command LAND ourselves.
      'fc'      — the FC left GUIDED in flight (its own failsafe or a pilot
                  mode change): stop commanding so we never fight it.
      'sensor'  — a sensor stayed dead after the hold-and-retry (red-zone
                  camera over the arena): RTL; RTL refused / not engaged -> LAND.
      'failure' — companion-detected failure (sensor dead, leg failed, ...):
                  LAND if airborne, disarm/stay down if on the ground.
    """

    def __init__(self, kind: str, reason: str):
        super().__init__(f"{kind}: {reason}")
        self.kind = kind
        self.reason = reason


class MissionNode(Node):
    abort_type = MissionAbort       # for the phase modules (no circular import)
    def __init__(self):
        super().__init__("mission_node")

        self.cfg = load_mission_config(self)
        self.sm = MissionSM(logger=self.get_logger())

        # ----- Publishers -----
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.telemetry_pub = self.create_publisher(String, "/mission/telemetry", 10)
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Target ID for the qr_scanner_node Phase-5 matcher (latched so a late
        # subscriber still gets it).
        self.delivery_id_pub = self.create_publisher(String, "/qr_scanner/delivery_id", latched)
        # Where the reference QR should be ("lat,lon,radius_m"; "" = anywhere):
        # qr_scanner_node's locate uses only a QR near it (reference_qr.py).
        self.locate_hint_pub = self.create_publisher(String, "/qr_scanner/locate_hint", latched)
        # Site survey in use ("d_east,d_north" shift onto today's home fix, or
        # "off"): path_planner flies the surveyed corridors only after this.
        self.survey_pub = self.create_publisher(String, "/mission/survey_offset", latched)

        # MAVROS publishes BEST_EFFORT; a RELIABLE subscriber gets nothing.
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ----- Live data (written by callbacks, read by the mission thread) -----
        self.mavros_state = None
        self.battery_voltage = None
        self.gps = None                 # (lat, lon) with a valid fix
        self.rel_alt = None             # metres above home
        self.home = None                # (lat, lon) recorded before arming
        self.heading_deg = None         # compass, deg clockwise from North
        self.placed_heading_deg = None  # heading as placed on the ground (held in the climb)
        self.local_xy = None            # (east, north) of the EKF local position, m
        self.ground_speed = None        # horizontal EKF speed, m/s
        self.mission_t0 = None          # time.time() at lift-off: the mission clock
        self.survey = None              # waypoints anchored at takeoff (survey.py), or None
        self._rc_auto_active = False
        self._red_watch = False         # red_zone_node up over the arena: the exit
                                        # failsafe routes home around red

        self._abort = None             # (kind, reason) once requested
        self._airborne = False
        self._landing = False           # we commanded LAND: mode change expected
        self._mission_complete = False

        self.create_subscription(MavrosState, "/mavros/state", self._mavros_state_cb, mavros_qos)
        self.create_subscription(BatteryState, "/mavros/battery", self._battery_cb, mavros_qos)
        self.create_subscription(RCIn, "/mavros/rc/in", self._rc_in_cb, mavros_qos)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._gps_cb, mavros_qos)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt", self._rel_alt_cb, mavros_qos)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg",
                                 self._heading_cb, mavros_qos)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._local_pose_cb, mavros_qos)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                                 self._local_vel_cb, mavros_qos)

        self.create_timer(1.0 / self.cfg.tick_rate_hz, self._publish_telemetry)
        self.arena = ArenaSearch(self)          # phase 3 (subscribes now)

        # Downward camera and per-phase nodes, started phase by phase
        self.stages = Stages(self, self.cfg.staged_launch, self.cfg.stage_launch_args,
                             self.cfg.log_dir)
        self.plog = PhaseLog(self, self.cfg.log_dir)    # one text log per phase

        self._mission_thread = threading.Thread(target=self._run_mission, daemon=True)
        self._mission_thread.start()
        self.get_logger().info("[mission_node] Initialized. Mission thread started.")

    # ===================================================================
    # Subscriber callbacks — record + flag only, never block
    # ===================================================================

    def _mavros_state_cb(self, msg: MavrosState):
        self.mavros_state = msg
        if self._airborne and not self._landing and msg.mode != "GUIDED":
            self._request_abort("fc", f"FC switched to {msg.mode}")

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.gps = (msg.latitude, msg.longitude)

    def _rel_alt_cb(self, msg: Float64):
        self.rel_alt = float(msg.data)

    def _heading_cb(self, msg: Float64):
        self.heading_deg = float(msg.data)

    def _local_pose_cb(self, msg: PoseStamped):
        self.local_xy = (msg.pose.position.x, msg.pose.position.y)

    def _local_vel_cb(self, msg: TwistStamped):
        self.ground_speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)

    def _battery_cb(self, msg: BatteryState):
        self.battery_voltage = msg.voltage
        if 0.0 < msg.voltage < self.cfg.battery_critical_v and self._airborne:
            self._request_abort(
                "battery",
                f"battery critical {msg.voltage:.2f} V < {self.cfg.battery_critical_v} V")

    def _rc_in_cb(self, msg: RCIn):
        if not self.cfg.require_rc_switch:
            return          # RC CH7 not used: neither starts nor aborts the mission
        idx = int(self.cfg.rc_auto_channel) - 1      # physical CHn -> index n-1
        if not 0 <= idx < len(msg.channels):
            return
        pwm = msg.channels[idx]
        was_auto = self._rc_auto_active
        self._rc_auto_active = pwm > self.cfg.rc_auto_pwm_thresh
        if was_auto and not self._rc_auto_active:
            self._request_abort(
                "pilot", f"RC CH{idx + 1}={pwm} < {self.cfg.rc_auto_pwm_thresh}")

    def _request_abort(self, kind: str, reason: str):
        if self._abort is None:
            self.get_logger().error(f"[mission_node] ABORT REQUESTED ({kind}): {reason}")
            self._abort = (kind, reason)

    # ===================================================================
    # Waiting helpers (mission thread only)
    # ===================================================================

    def _tick(self):
        """Raise MissionAbort if any abort condition holds. Call often."""
        if not rclpy.ok():
            raise MissionAbort("failure", "ROS shutting down")
        if self._abort is not None:
            raise MissionAbort(*self._abort)

    def _mission_left_s(self):
        """Seconds left of mission_time_limit_s since lift-off (None before)."""
        if self.mission_t0 is None:
            return None
        return float(self.cfg.mission_time_limit_s) - (time.time() - self.mission_t0)

    def _log_clock(self, what: str):
        left = self._mission_left_s()
        if left is None:
            return
        limit = float(self.cfg.mission_time_limit_s)
        text = (f"[mission_node] CLOCK {what}: {(limit - left) / 60.0:.1f} min since lift-off, "
                f"{left:+.0f} s left of {limit / 60.0:.0f} min")
        (self.get_logger().warn if left < 0 else self.get_logger().info)(text)

    def _arena_deadline(self):
        """time.time() by which the arena sweep must end so that the drop and
        the exit corridor + home still fit in the mission time (None: clock
        not running)."""
        left = self._mission_left_s()
        if left is None:
            return None
        budget = left - float(self.cfg.mission_drop_reserve_s) \
            - float(self.cfg.mission_exit_reserve_s)
        if budget <= 0.0:
            self.get_logger().error(
                f"[mission_node] CLOCK: no time left for the arena search ({left:.0f} s left, "
                "drop + exit reserves need more) — skipping it")
        else:
            self.get_logger().info(f"[mission_node] CLOCK: arena search gets {budget:.0f} s")
        return time.time() + budget

    def _sleep(self, secs: float):
        end = time.time() + secs
        while time.time() < end:
            self._tick()
            time.sleep(0.1)

    def _wait_for(self, cond, timeout: float, what: str) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._tick()
            if cond():
                return True
            time.sleep(0.1)
        self.get_logger().error(f"[mission_node] Timeout waiting for {what} ({timeout:.0f}s)")
        return False

    def _should_abort(self) -> bool:
        try:
            self._tick()
            return False
        except MissionAbort:
            return True

    def _armed(self) -> bool:
        return bool(self.mavros_state is not None and self.mavros_state.armed)

    def _goto_and_wait(self, lat, lon, alt, tolerance, timeout, speed, face_target=True,
                       hold_yaw_enu=None) -> bool:
        """GUIDED goto with arrival check.

        Heading: face_target turns the nose toward the target; otherwise
        hold_yaw_enu (ENU rad) is held. With neither, yaw is left to the FC —
        which in GUIDED turns the nose toward the target anyway
        (WP_YAW_BEHAVIOR), so pass hold_yaw_enu to keep a heading.

        Speed: the first position target after another GUIDED submode (e.g.
        takeoff) resets the limit to the FC's WP_SPD, so hold first, then
        DO_CHANGE_SPEED, then the real target."""
        if self.gps is not None:
            mav.goto(self, self.gps[0], self.gps[1], alt, publish_count=3, yaw_enu=hold_yaw_enu)
        mav.change_speed(self, speed)
        yaw = hold_yaw_enu
        if face_target and self.gps is not None:
            yaw = mav.yaw_enu_towards(self.gps[0], self.gps[1], lat, lon)
        mav.goto(self, lat, lon, alt, publish_count=self.cfg.goto_publish_count, yaw_enu=yaw)
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._tick()
            if self.gps is not None and mav.haversine(*self.gps, lat, lon) <= tolerance:
                return True
            time.sleep(0.2)
        self.get_logger().error(f"[mission_node] Arrival timeout at ({lat:.7f}, {lon:.7f})")
        return False

    def _goto_wp(self, label, latlon, alt, speed, tolerance, yaw_enu=None,
                 required=False) -> bool:
        """Fly to a mission waypoint at alt, nose held on yaw_enu (None: toward
        it). Timeout from the distance. Not reached: required -> MissionAbort
        "failure" (phase failsafe), else logged and False."""
        lat, lon = latlon
        dist = 0.0 if self.gps is None else mav.haversine(*self.gps, lat, lon)
        self.get_logger().info(
            f"[mission_node] WAYPOINT {label}: ({lat:.7f}, {lon:.7f}), {dist:.1f} m away, "
            f"at {alt:.2f} m, {speed:.1f} m/s")
        ok = self._goto_and_wait(lat, lon, alt, tolerance,
                                 max(30.0, 4.0 * dist / max(speed, 0.1)), speed,
                                 face_target=yaw_enu is None, hold_yaw_enu=yaw_enu)
        if ok:
            self.get_logger().info(f"[mission_node] WAYPOINT {label}: reached")
        elif required:
            raise MissionAbort("failure", f"waypoint {label} not reached")
        else:
            self.get_logger().warn(f"[mission_node] WAYPOINT {label}: not reached — carrying on")
        return ok

    def _wp(self, name):
        """(lat, lon) of a mission waypoint, anchored on today's home fix:
        "home", "qr", "entry_mouth", "entry_end", "exit_mouth", "exit_end",
        "entry_start" / "exit_start" (entry_start_standoff_m /
        exit_search_standoff_m in front of the mouth), "entry_clear" /
        "exit_clear" (corridor_clear_m past the end)."""
        s = self.survey
        if name == "home":
            return self.home
        if name == "qr":
            return s.gps(s.qr)
        corridor, _, where = name.partition("_")
        c = getattr(s, corridor)
        if where == "start":
            standoff = (self.cfg.entry_start_standoff_m if corridor == "entry"
                        else self.cfg.exit_search_standoff_m)
            return s.gps(c.world(-float(standoff)))
        if where == "clear":
            return s.gps(c.world(c.length + float(self.cfg.corridor_clear_m)))
        return s.gps(c.mouth if where == "mouth" else c.end)

    def _phase(self, label: str, fn, *args):
        """PHASE FAILSAFE: run one phase; a failure in it (MissionAbort
        "failure" or a crash) moves the mission on to the next phase: the
        planner is stopped, the drone holds where it is and the result is
        None. Pilot, FC, battery and sensor (red camera) aborts still raise."""
        try:
            return fn(*args)
        except MissionAbort as a:
            if a.kind != "failure":
                raise
            reason = a.reason
        except Exception as e:
            import traceback
            self.get_logger().error(traceback.format_exc())
            reason = f"crash: {e}"
        self.get_logger().error(
            f"[mission_node] PHASE FAILSAFE — {label} failed ({reason}): "
            "moving on to the next phase")
        if self.gps is not None and self.rel_alt is not None:
            mav.goto(self, self.gps[0], self.gps[1], self.rel_alt,
                     publish_count=self.cfg.goto_publish_count)
        return None

    # ===================================================================
    # Mission phases
    # ===================================================================

    def _wait_for_start(self):
        self.get_logger().info("[mission_node] Waiting for MAVROS connection...")
        while self.mavros_state is None or not self.mavros_state.connected:
            self._tick()
            time.sleep(0.5)

        # ArduPilot only streams what is requested (SITL --no-mavproxy) or
        # what SRx_ params enable (hardware). Ask explicitly for what we use.
        mav.request_message_intervals(self, rate_hz=self.cfg.stream_rate_hz)

        if self.cfg.require_rc_switch:
            self.get_logger().info(
                f"[mission_node] Waiting for pilot: RC CH{self.cfg.rc_auto_channel} "
                f"> {self.cfg.rc_auto_pwm_thresh} to start the mission")
            while not self._rc_auto_active:
                self._tick()
                time.sleep(0.2)
            self.get_logger().info("[mission_node] RC auto switch ON — starting mission")

    def _reference_qr(self):
        """Phase 1 (in the air, over the launch point): find, centre over and
        read the reference QR; qr_scanner_node stores it. Returns its text
        ("" = it holds no text and is matched by picture), or None if it was
        not read — the mission then carries on without a target."""
        text = ReferenceQr(self).run()
        if text is None and self.cfg.allow_qr_fallback:
            self.get_logger().warn(
                f"[mission_node] QR scan failed — using fallback ID '{self.cfg.qr_fallback_id}' "
                "(allow_qr_fallback=true, SITL only)")
            return self.cfg.qr_fallback_id
        return text

    def _preflight(self):
        if not self._wait_for(lambda: self.gps is not None, 60.0, "GPS fix"):
            raise MissionAbort("failure", "no GPS fix")
        self.home = self.gps
        self.get_logger().info(f"[mission_node] Home: ({self.home[0]:.7f}, {self.home[1]:.7f})")
        self.stages.start("down_cam", self.cfg.stage_up_timeout_s)
        if self.cfg.staged_launch and "launch_camera:=true" in self.cfg.stage_launch_args \
                and not self._wait_down_frame(self.cfg.down_cam_up_timeout_s):
            # No reference QR, no red zones (every arena leg needs them): stay down
            raise MissionAbort("failure", "downward camera sent no frame — not taking off")

    def _wait_down_frame(self, timeout: float) -> bool:
        """True once a /down_frame image arrives (the camera can start and
        then die, so a publisher alone proves nothing). The subscription only
        lives for this wait."""
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        got = threading.Event()
        sub = self.create_subscription(Image, "/down_frame", lambda _m: got.set(),
                                       qos_profile_sensor_data)
        try:
            return self._wait_for(got.is_set, timeout, "downward camera frame (/down_frame)")
        finally:
            self.destroy_subscription(sub)

    def _check_ground_drift(self, ref, secs: float, when: str):
        """Wait secs, aborting if the EKF position moves more than
        takeoff_max_ground_drift_m from ref while the drone sits still."""
        limit = self.cfg.takeoff_max_ground_drift_m
        end = time.time() + secs
        while time.time() < end:
            self._tick()
            drift = math.hypot(self.local_xy[0] - ref[0], self.local_xy[1] - ref[1])
            if drift > limit:
                raise MissionAbort(
                    "failure", f"position estimate drifted {drift:.2f} m {when} "
                    f"(> {limit:.2f} m: vibration? weak GPS/flow?) — not taking off")
            time.sleep(0.1)

    def _takeoff(self, alt=None):
        """Gradual takeoff to alt (default cruise_altitude_m), as
        testing/waypoint_mission.py:
          1. the position estimate must hold still (takeoff_steady_time_s)
          2. GUIDED, arm, sit armed ARM_WAIT_S (motors spool; drift still checked)
          3. lift off to takeoff_liftoff_alt_m (MAV_CMD_NAV_TAKEOFF)
          4. slide the altitude target up to alt at takeoff_climb_rate_ms
             over the lift-off point, holding the heading the drone was
             placed with"""
        cfg = self.cfg
        alt = cfg.cruise_altitude_m if alt is None else alt
        first = min(cfg.takeoff_liftoff_alt_m, alt)
        # A cold FC has no local position until the EKF origin is set (~30-60 s).
        if not self._wait_for(lambda: self.local_xy is not None and self.heading_deg is not None,
                              self.cfg.arm_timeout_s, "local position + compass heading"):
            raise MissionAbort("failure", "no local position / heading (is the EKF origin set?)")
        self.get_logger().info(
            f"[mission_node] Checking the position estimate is steady "
            f"({cfg.takeoff_steady_time_s:.0f} s)")
        self._check_ground_drift(self.local_xy, cfg.takeoff_steady_time_s,
                                 "while disarmed on the ground")
        self.placed_heading_deg = self.heading_deg
        self.get_logger().info(f"[mission_node] Placed heading {self.placed_heading_deg:.1f} deg")

        if not mav.set_mode(self, "GUIDED"):
            raise MissionAbort("failure", "could not set GUIDED")
        # A cold FC rejects arming for ~30-60 s while the EKF settles: retry.
        deadline = time.time() + self.cfg.arm_timeout_s
        while not self._armed():
            self._tick()
            if time.time() > deadline:
                raise MissionAbort("failure", "arming timed out (check pre-arm messages)")
            if mav.arm(self):
                self._wait_for(self._armed, 5.0, "armed state")
            if not self._armed():
                self._sleep(2.0)

        self.get_logger().info(f"[mission_node] Armed — taking off in {ARM_WAIT_S:.0f} s")
        # Drift measured from arming, not from the steady check: the GPS
        # estimate wanders slowly while waiting, which is no reason to abort.
        self._check_ground_drift(self.local_xy, ARM_WAIT_S, "on the ground after arming")
        if not self._armed():
            raise MissionAbort("failure", "disarmed during the pre-takeoff wait")
        # Re-anchor home to where the estimate says we are right now, so the
        # climb and later limits are relative to the actual lift-off point.
        if self.gps is not None:
            self.home = self.gps
        self._anchor_survey()

        self.get_logger().info(f"[mission_node] Lift-off to {first:.1f} m")
        if not mav.takeoff(self, first):
            raise MissionAbort("failure", "takeoff command rejected")
        if self.mission_t0 is None:
            self.mission_t0 = time.time()
        self._airborne = True
        if not self._wait_for(lambda: self.rel_alt is not None and self.rel_alt >= 0.8 * first,
                              self.cfg.altitude_timeout_s, f"lift-off to {first} m"):
            raise MissionAbort("failure", "did not lift off")

        if alt - first > 0.05:
            self._climb_gradually(alt)
        self.get_logger().info(f"[mission_node] At {self.rel_alt:.2f} m")
        self._sleep(1.0)

    def _anchor_survey(self):
        """Place the mission waypoints (mission_common/survey.py) on today's
        home fix and tell path_planner (/mission/survey_offset). Not near
        wp_home (a phase test flight elsewhere) -> none: the QR is expected
        ahead of takeoff."""
        survey = site_survey.load(self.cfg)
        self.survey, how = site_survey.anchor(survey, self.home, self.cfg)
        log = self.get_logger()
        (log.info if self.survey is not None else log.warn)(f"[mission_node] Waypoints: {how}")
        if self.survey is not None:
            for name in ("entry", "exit"):
                c = getattr(self.survey, name)
                if c is not None:
                    log.info(f"[mission_node] Waypoints: {name} corridor {c.length:.2f} m long, "
                             f"heading {(90.0 - math.degrees(c.axis_yaw)) % 360.0:.0f} deg")
        self.survey_pub.publish(String(data="off" if self.survey is None else
                                       f"{self.survey.shift[0]:.3f},{self.survey.shift[1]:.3f}"))

    def _climb_gradually(self, alt: float):
        """Slide the altitude target from here to alt at takeoff_climb_rate_ms
        over home, nose on the placed heading; done once within
        altitude_tolerance_m of alt."""
        cfg = self.cfg
        rate = cfg.takeoff_climb_rate_ms
        start = self.rel_alt
        ramp = max(0.0, alt - start) / rate
        # compass (deg CW from North) -> ENU yaw (rad CCW from East)
        yaw = self.placed_yaw_enu()
        lat, lon = self.home
        self.get_logger().info(
            f"[mission_node] Climbing {start:.1f} -> {alt:.1f} m at {rate:.2f} m/s")
        t0 = time.time()
        while True:
            self._tick()
            el = time.time() - t0
            mav.goto(self, lat, lon, min(alt, start + rate * el),
                     publish_count=1, yaw_enu=yaw, log=False)       # ~10 Hz
            if el >= ramp and abs(self.rel_alt - alt) <= cfg.altitude_tolerance_m:
                break
            if el > ramp + cfg.altitude_timeout_s:
                raise MissionAbort("failure", f"did not reach takeoff altitude {alt} m")
        mav.goto(self, lat, lon, alt, publish_count=cfg.goto_publish_count, yaw_enu=yaw)

    def _reference_qr_phase(self):
        """Phase 1, after the takeoff over wp_home: to wp_qr at
        cruise_altitude_m (nose on the placed heading), then find, centre over
        and read the reference QR there."""
        if self.survey is not None and self.survey.qr is not None:
            self._goto_wp("reference QR", self._wp("qr"), float(self.cfg.cruise_altitude_m),
                          float(self.cfg.qr_move_speed_ms), float(self.cfg.qr_move_tolerance_m),
                          yaw_enu=self.placed_yaw_enu())
        return self._reference_qr()

    def _entry_corridor_phase(self):
        """Phase 2, plain waypoints THROUGH the entry corridor (GPS only, no
        obstacle sensor), nose down it: at cruise_altitude_m to
        entry_start_standoff_m in front of wp_entry_mouth, straight down there
        to corridor_start_alt_m (half the 8 ft walls, under the banner), then
        wp_entry_mouth -> wp_entry_end -> corridor_clear_m past it, and
        straight up to post_entry_alt_m — the arena sweep starts from there.

        Not at corridor_start_alt_m in front of it -> it does not go in. Any
        failure ("failure" abort or crash) -> straight up to post_entry_alt_m
        where it is (the corridor has no roof), then MissionAbort("failure")
        (phase failsafe: the sweep starts from there)."""
        cfg, c = self.cfg, self.survey.entry
        try:
            self._through_entry_corridor(c)
        except Exception as e:
            if isinstance(e, MissionAbort) and e.kind != "failure":
                raise
            self.get_logger().error(
                f"[mission_node] Entry corridor failed ({e}): straight up to "
                f"{cfg.post_entry_alt_m:.2f} m here")
            self._climb_in_place(float(cfg.post_entry_alt_m), c.axis_yaw)
            raise MissionAbort("failure", f"entry corridor: {e}")
        self._climb_in_place(float(cfg.post_entry_alt_m), c.axis_yaw)
        return True

    def _through_entry_corridor(self, c):
        cfg = self.cfg
        alt = float(cfg.corridor_start_alt_m)
        speed, tol = float(cfg.corridor_speed_ms), float(cfg.corridor_tolerance_m)
        self._goto_wp("entry_start", self._wp("entry_start"), float(cfg.cruise_altitude_m),
                      speed, tol, yaw_enu=c.axis_yaw, required=True)
        self._climb_in_place(alt, c.axis_yaw)
        if self.rel_alt is None or abs(self.rel_alt - alt) > cfg.altitude_tolerance_m:
            raise MissionAbort("failure", f"{alt:.2f} m not reached in front of the "
                                          "entry corridor — not going in")
        # In the corridor a missed arrival is logged, not a failure: the FC
        # keeps flying to the target either way (GPS is only good to ~1 m).
        for label in ("entry_mouth", "entry_end", "entry_clear"):
            self._goto_wp(label, self._wp(label), alt, speed, tol, yaw_enu=c.axis_yaw)

    def _exit_corridor_phase(self):
        """Phase 5: to the exit corridor's start point (exit_search_standoff_m
        in front of wp_exit_mouth) at arena_alt_m around the red zones, down
        to exit_overfly_alt_m facing down the corridor, then OVER it:
        wp_exit_mouth -> wp_exit_end (plain waypoints).

        True: over, at wp_exit_end. False: it failed and the EXIT CORRIDOR
        FAILSAFE has run (_exit_failsafe: straight up above the walls, home,
        landed). Only this phase has that failsafe: any failure in it — a
        waypoint not reached, a companion "failure" abort (e.g. red camera
        silent on the way there) or a crash — runs it; battery / pilot / FC
        aborts still raise MissionAbort as everywhere."""
        try:
            reason = self._fly_exit_corridor()
        except MissionAbort as a:
            if a.kind != "failure":
                raise
            reason = a.reason
        except Exception as e:
            import traceback
            self.get_logger().error(traceback.format_exc())
            reason = f"crash: {e}"
        if reason is None:
            return True
        self._exit_failsafe(reason)
        return False

    def _exit_failsafe(self, reason: str):
        """Exit corridor failed: climb straight up where the vehicle is to
        exit_failsafe_alt_m (above the corridor walls), then fly home and land — fully autonomous. If the altitude is not
        confirmed the vehicle lands in place instead: flying home lower could
        cross a wall.

        Home: still over the arena (red_zone_node running, the start point not
        reached yet) -> around the red zones at arena_alt_m with the red
        camera watching (ArenaSearch.go_to) — the red mat must never be flown
        over, at any height. No route / red camera dead -> MissionAbort:
        lands where it is (never over red). At / in the corridor: straight
        home at exit_failsafe_alt_m (away from the arena)."""
        alt = float(self.cfg.exit_failsafe_alt_m)
        self.get_logger().error(
            f"[mission_node] EXIT CORRIDOR FAILSAFE — {reason}: climbing straight up to "
            f"{alt:.1f} m (above the corridor walls), then return to launch and land")
        self._home_and_land(alt, routed=self._red_watch and self.arena.frame is not None,
                            why="exit corridor failsafe")
        self.get_logger().warn(
            "[mission_node] Mission ended by the exit corridor failsafe — landed at home")

    def _home_and_land(self, alt: float, routed: bool, why: str):
        """Phase 6 (and the exit corridor failsafe's end): straight up where
        the vehicle is to alt — above the corridor walls — then home, land.

        alt not confirmed after two tries -> MissionAbort (lands in place):
        flying home lower could cross a corridor wall or banner. routed: home
        around the red zones at arena_alt_m (ArenaSearch.go_to; the vehicle
        is on the arena side); else a straight line at alt (launch side).
        State: CORRIDOR_2_TRANSIT -> HOMING -> LANDING -> LANDED."""
        yaw = None if self.heading_deg is None else math.radians(90.0 - self.heading_deg)
        for _ in range(2):
            self._climb_in_place(alt, yaw)
            if self.rel_alt is not None and self.rel_alt >= alt - self.cfg.altitude_tolerance_m:
                break
        else:
            raise MissionAbort("failure", f"{why}: {alt:.1f} m not reached "
                                          "— landing in place rather than fly home low")
        self.sm.c2_complete()
        self._log_clock("homing")
        if routed:
            if not self.arena.go_to(*self.home, f"home ({why})"):
                raise MissionAbort("failure", f"{why}: no route home clear of the red zones "
                                              "— landing here")
        elif not self._goto_and_wait(*self.home, alt, self.cfg.arrival_tolerance_m,
                                     self.cfg.arrival_timeout_s, self.cfg.transit_speed_ms):
            self.get_logger().warn("[mission_node] Home not reached in time — landing here")
        self.sm.home_reached()
        self._land_and_wait()
        self.sm.touchdown()

    def _fly_exit_corridor(self):
        """The exit corridor itself (see _exit_corridor_phase). None if
        over it, else why it failed."""
        if self.home is None:
            return "no home position"
        if self.survey is None or self.survey.exit is None:
            return "exit corridor waypoints not set"
        cfg, c = self.cfg, self.survey.exit
        lat, lon = self._wp("exit_start")
        self.get_logger().info(
            f"[mission_node] Exit corridor: to its start point "
            f"{cfg.exit_search_standoff_m:.1f} m in front of wp_exit_mouth "
            f"({lat:.7f}, {lon:.7f}) at {cfg.arena_alt_m:.2f} m, around red")
        if not self.arena.go_to(lat, lon, "exit corridor start"):
            return "no route to the exit corridor start point clear of the red zones"
        # Out of the arena from here: the red camera's job is done.
        self.stages.stop("arena")
        self._red_watch = False
        alt = float(cfg.exit_overfly_alt_m)
        self._climb_in_place(alt, c.axis_yaw)
        if self.rel_alt is None or self.rel_alt < alt - cfg.altitude_tolerance_m:
            return f"{alt:.2f} m not confirmed — not flying over the walls lower"
        speed, tol = float(cfg.corridor_speed_ms), float(cfg.corridor_tolerance_m)
        for label in ("exit_mouth", "exit_end"):
            if not self._goto_wp(label, self._wp(label), alt, speed, tol, yaw_enu=c.axis_yaw):
                return f"waypoint {label} not reached"
        return None

    def placed_yaw_enu(self):
        """The heading the drone was placed with, as an ENU yaw (rad), or None.
        Phase 1 holds it."""
        if self.placed_heading_deg is None:
            return None
        return math.radians(90.0 - self.placed_heading_deg)

    def _focus_camera(self, distance_m: float):
        """Fixed focus of the downward Cam3 (camera_ros node 'camera') for
        the ground distance_m below: LensPosition = 1 / distance (dioptres).
        Autofocus cannot work on this camera stack (libcamera: "PDAF data in
        unsupported format"), so each phase sets the focus for its altitude.
        No camera node (SITL) -> logged, carries on."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        lens = 1.0 / max(float(distance_m), 0.1)
        client = self.create_client(SetParameters, "/camera/set_parameters")
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f"[mission_node] No camera node — focus for {distance_m:.1f} m not set")
                return
            req = SetParameters.Request(parameters=[Parameter(
                name="LensPosition",
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=lens))])
            future = client.call_async(req)
            end = time.time() + 3.0
            while not future.done() and time.time() < end:
                time.sleep(0.05)
            res = future.result() if future.done() else None
            if res is not None and res.results and res.results[0].successful:
                self.get_logger().info(
                    f"[mission_node] Camera focus set for {distance_m:.1f} m "
                    f"(LensPosition {lens:.2f})")
            else:
                reason = res.results[0].reason if res is not None and res.results else "no reply"
                self.get_logger().warn(
                    f"[mission_node] Camera focus for {distance_m:.1f} m NOT set: {reason}")
        finally:
            self.destroy_client(client)

    def _climb_in_place(self, alt: float, yaw_enu=None):
        """Climb (or descend) straight to alt where the vehicle is (the entry
        corridor ends holding at its measured altitude just past the walls;
        never climb diagonally back across a wall). yaw_enu: heading to hold
        (None: left to the FC)."""
        if self.gps is not None:
            mav.goto(self, self.gps[0], self.gps[1], alt,
                     publish_count=self.cfg.goto_publish_count, yaw_enu=yaw_enu)
        if not self._wait_for(
                lambda: self.rel_alt is not None
                and abs(self.rel_alt - alt) <= self.cfg.altitude_tolerance_m,
                self.cfg.altitude_timeout_s, f"altitude {alt} m"):
            self.get_logger().warn(f"[mission_node] {alt} m not reached — continuing")

    def _drop_payload(self, qr_lat, qr_lon):
        """Phase 4 (mission_manager/payload_drop.py): over the QR around the
        red zones, straight down, camera-centred, release, back up."""
        return PayloadDrop(self).run(
            qr_lat, qr_lon,
            approach=lambda lat, lon: self.arena.go_to(lat, lon, "over the drop QR"),
            clear=self.arena.clear_of_red)

    def _land_and_wait(self):
        self._landing = True
        if not mav.land(self):
            mav.land(self)          # one retry; LAND is the end of the line
        if not self._wait_for(lambda: not self._armed(), self.cfg.land_timeout_s,
                              "touchdown + auto-disarm"):
            raise MissionAbort("failure", "landing not confirmed")
        self._airborne = False

    def _check_waypoints(self):
        """Full mission only, on the ground: every waypoint set and the drone
        near wp_home (else the yaml is for another field) — or no takeoff."""
        course = site_survey.load(self.cfg)
        missing = [n for n, v in (("wp_home", course), ("wp_qr", course and course.qr),
                                  ("wp_entry_mouth/_end", course and course.entry),
                                  ("wp_exit_mouth/_end", course and course.exit)) if not v]
        if missing:
            raise MissionAbort("failure", f"waypoints not set in mission_params.yaml "
                                          f"({', '.join(missing)}) — not taking off")
        if not (any(self.cfg.arena_corners_lat) and any(self.cfg.arena_corners_lon)):
            raise MissionAbort("failure", "arena_corners_lat/lon not set in "
                                          "mission_params.yaml — not taking off")
        placed, how = site_survey.anchor(course, self.home, self.cfg)
        if placed is None:
            raise MissionAbort("failure", f"{how} — wrong field or wrong waypoints, "
                                          "not taking off")
        self.survey = placed            # re-anchored at takeoff (_anchor_survey)
        self.get_logger().info(f"[mission_node] Waypoints OK: {how}")

    def _run_mission(self):
        plog = self.plog
        try:
            self._wait_for_start()

            # ---- Phase 0: preflight (on the ground) ----
            plog.begin(0, "preflight")
            self.sm.auto_activate()
            self._preflight()               # GPS fix, downward camera up
            self._check_waypoints()
            self._focus_camera(float(self.cfg.cruise_altitude_m))   # reference QR height

            # ---- Phase 1: takeoff over home, to the QR, read it ----
            plog.begin(1, "takeoff_qr", self.home, self._wp_or_none("qr"))
            self._takeoff()                 # anchors the waypoints on today's home fix
            # From here every phase is run by _phase: a failure in one moves
            # on to the next (PHASE FAILSAFE).
            self.sm.airborne()
            self._log_clock("phase 1, reference QR")
            self.sm.mission_target_qr = self._phase("phase 1 (reference QR)",
                                                    self._reference_qr_phase)
            if self.sm.mission_target_qr is None:
                self.get_logger().warn(
                    "[mission_node] Reference QR NOT read — continuing without a target "
                    "(the arena sweep finds no match: the payload goes on a random QR)")
                plog.end("QR NOT read")
            else:
                shown = self.sm.mission_target_qr or "(no text — matched by picture)"
                self.get_logger().info(f"[mission_node] Target locked: '{shown}'")
                plog.end(f"QR read: {shown}")

            # ---- Phase 2: through the entry corridor (waypoints), up to 9.25 m ----
            self.sm.qr_scanned()
            plog.begin(2, "entry_corridor", self._wp("entry_mouth"), self._wp("entry_end"))
            self._log_clock("phase 2, entry corridor")
            done = self._phase("phase 2 (entry corridor)", self._entry_corridor_phase) is True
            plog.end("through the entry corridor" if done else "FAILED — arena sweep from here")

            # ---- Phase 3: arena search (red zone + BCD sweep) ----
            self.sm.entry_complete()
            plog.begin(3, "arena_search")
            self._log_clock("phase 3, arena search")
            # red_zone_node from here to the exit corridor's start point:
            # every leg over the arena (sweep, drop, to the exit) routes around red.
            self.stages.start("arena", self.cfg.stage_up_timeout_s)
            self._red_watch = True
            target = None
            until = self._arena_deadline()
            if until is None or until > time.time():
                target = self._phase("phase 3 (arena search)", self.arena.run, None, until)
            if target is None:
                target = self.arena.random_qr()
                if target is not None:
                    self.get_logger().warn(
                        "[mission_node] Target QR not found — dropping on a RANDOM QR seen "
                        f"in the sweep ({target[0]:.7f}, {target[1]:.7f})")
                plog.end("no match" + (" — random QR for the drop" if target else
                                       ", no QR seen"))
            else:
                plog.end(f"MATCH at ({target[0]:.7f}, {target[1]:.7f})")

            # ---- Phase 4: payload drop ----
            if target is not None:
                self.sm.qr_matched()
                plog.begin(4, "payload_drop", None, target)
                self._log_clock("phase 4, payload drop")
                self._phase("phase 4 (payload drop)", self._drop_payload, *target)
                if self.sm.state_id == "DROPPING" and self.sm.drop_confirmed:
                    self.sm.drop_complete()
                    plog.end("payload released")
                else:
                    self.get_logger().warn(
                        "[mission_node] Payload NOT dropped properly — moving on to the "
                        "exit corridor")
                    self.sm.drop_failed()
                    plog.end("payload NOT released")
            else:
                self.get_logger().warn(
                    "[mission_node] No matching QR and no QR seen — keeping payload, continuing")
                self.sm.search_exhausted()

            # ---- Phase 5: over the exit corridor (waypoints) ----
            # Failed -> its failsafe has already climbed above the walls,
            # flown home and landed.
            plog.begin(5, "exit_corridor", self._wp("exit_start"), self._wp("exit_end"))
            self._log_clock("phase 5, exit corridor")
            through = self._exit_corridor_phase()
            self.stages.stop("arena")
            self._red_watch = False
            if not through:
                plog.end("FAILED — exit failsafe flew home and landed")
                self._log_clock("landed (exit corridor failsafe)")
                return
            plog.end("over the exit corridor")

            # ---- Phase 6: 5 m, home, land ----
            plog.begin(6, "home_land", None, self.home)
            self._home_and_land(float(self.cfg.cruise_altitude_m), routed=False,
                                why="homing")
            self._mission_complete = True
            self._log_clock("landed")
            plog.end("landed")
            self.get_logger().info("[mission_node] Mission complete!")

        except MissionAbort as a:
            self._handle_abort(a)
            plog.end(f"MISSION ABORTED — {a}")
        except Exception as e:
            import traceback
            self.get_logger().error(f"[mission_node] Mission thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))
            plog.end(f"MISSION CRASHED — {e}")

    def _wp_or_none(self, name):
        try:
            return self._wp(name)
        except Exception:
            return None

    def _handle_abort(self, a: MissionAbort):
        self.get_logger().error(f"[mission_node] MISSION ABORTED — {a}")
        self._landing = True     # mode changes from here on are ours or the FC's
        try:
            if a.kind == "pilot":
                if self.sm.state_id not in ("LANDED", "RC_MODE", "ABORTED"):
                    self.sm.abort()
                if self._armed():
                    mav.set_mode(self, "LOITER")     # pilot flies from here
            else:
                if self.sm.state_id not in ("LANDED", "RC_MODE", "ABORTED"):
                    self.sm.failsafe()
                if a.kind == "battery":
                    # ArduPilot's battery failsafe acts first if it is set (it
                    # leaves GUIDED). Still in GUIDED = it did nothing (seen on
                    # the drone, BATT_FS_*_ACT 0): land here rather than hover
                    # on a draining pack.
                    st = self.mavros_state
                    if self._armed() and self._airborne and st is not None \
                            and st.mode == "GUIDED":
                        self.get_logger().error(
                            "[mission_node] Battery critical — LANDING here (ArduPilot "
                            "battery failsafe did not take over)")
                        if not mav.land(self):
                            mav.land(self)
                    else:
                        self.get_logger().error(
                            "[mission_node] Battery critical — ArduPilot failsafe in control "
                            f"({st.mode if st else '?'})")
                elif a.kind == "fc":
                    self.get_logger().error(
                        "[mission_node] FC is in control — companion stopped commanding")
                elif a.kind == "sensor" and self._armed() and self._airborne:
                    self._rtl()
                elif self._armed() and self._airborne:
                    mav.land(self)
                elif self._armed():
                    mav.disarm(self)
        except Exception as e:
            self.get_logger().error(f"[mission_node] abort handling error: {e}")

    def _rtl(self):
        """Sensor-dead (red camera) failsafe: RTL (FC climbs to RTL_ALT, flies home,
        lands). RTL refused or not engaged within RTL_CONFIRM_S (no home /
        EKF / GPS) -> LAND here. In RTL the FC's own failsafes still apply
        (FS_EKF_ACTION, BATT_FS_CRT_ACT -> LAND)."""
        self.get_logger().error("[mission_node] Sensor dead — RTL")
        if mav.set_mode(self, "RTL") or mav.set_mode(self, "RTL"):
            end = time.time() + RTL_CONFIRM_S
            while time.time() < end:
                st = self.mavros_state
                if st is not None and st.mode in ("RTL", "LAND"):
                    self.get_logger().error(f"[mission_node] FC in {st.mode} — returning home")
                    return
                time.sleep(0.1)
        self.get_logger().error("[mission_node] RTL NOT engaged — LANDING here")
        if not mav.land(self):
            mav.land(self)

    # ===================================================================
    # Telemetry (main thread timer)
    # ===================================================================

    def _publish_telemetry(self):
        state_id = self.sm.state_id
        self.status_pub.publish(String(data=state_id))
        st = self.mavros_state
        data = {
            "state": state_id,
            "battery_v": round(self.battery_voltage, 2) if self.battery_voltage else None,
            "rel_alt": round(self.rel_alt, 2) if self.rel_alt is not None else None,
            "target_qr": self.sm.mission_target_qr,
            "drop_confirmed": self.sm.drop_confirmed,
            "rc_auto": self._rc_auto_active,
            "fcu_connected": st.connected if st else False,
            "fcu_armed": st.armed if st else False,
            "fcu_mode": st.mode if st else "UNKNOWN",
            "abort": None if self._abort is None else f"{self._abort[0]}: {self._abort[1]}",
            "mission_complete": self._mission_complete,
            "mission_left_s": None if self.mission_t0 is None
            else round(self._mission_left_s(), 1),
        }
        self.telemetry_pub.publish(String(data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[mission_node] Shutting down")
    finally:
        node.stages.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
