# mission_node.py — ROS2 node that owns and drives the Mission 2 sequence
#
# Architecture:
#   - rclpy.spin() runs in the main thread. Every ROS callback here only
#     records data / sets flags and returns immediately — no service calls,
#     no state transitions that do work. (A blocking callback would freeze the
#     single-threaded executor, and with it every service response.)
#   - The mission runs linearly in a worker thread (_run_mission). It calls
#     services and sleep-polls their futures while the main thread spins.
#   - Abort conditions (RC switch low, FC left GUIDED, battery critical,
#     obstacle sensor / red camera dead -> RTL)
#     are checked continuously by _tick() inside every wait, so the mission can
#     be stopped mid-leg, not only between phases.
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
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State as MavrosState, RCIn
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus

from mission_common import mavros_utils as mav
from mission_common import srv_helpers as srv
from mission_common.config_loader import load_mission_config
from mission_manager.arena_search import ArenaSearch
from mission_manager.payload_drop import PayloadDrop
from mission_manager.reference_qr import ReferenceQr
from mission_manager.state_machine import MissionSM
from path_planner.exit_corridor import search_points

# Sit armed on the ground this long before every takeoff command (precaution).
# Keep it under ArduPilot's DISARM_DELAY (default 10 s) or the FC disarms on
# its own first.
ARM_WAIT_S = 7.0
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
      'sensor'  — a sensor stayed dead after the hold-and-retry (obstacle
                  sensor, red-zone camera, corridor SENSOR_TIMEOUT): RTL;
                  RTL refused / not engaged -> LAND.
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
        self.c1_exit = None             # (lat, lon, axis ENU rad): where phase 2 left
                                        # corridor 1's walls — phase 5 mirrors it
        self.mission_t0 = None          # time.time() at lift-off: the mission clock
        self._rc_auto_active = False
        self._sensor_live = False       # coverage_guard defaults are fail-safe False
        self._coverage_ok = False
        self._sensor_bad_since = None
        self._sensor_check = True       # False: the obstacle sensor is not needed / not ours
                                        # to time out (exit corridor leg, exit failsafe)

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
        # coverage_guard publishes these RELIABLE at 10 Hz.
        self.create_subscription(Bool, "/avoidance/sensor_live", self._sensor_live_cb, 10)
        self.create_subscription(Bool, "/avoidance/coverage_ok", self._coverage_ok_cb, 10)

        self.create_timer(1.0 / self.cfg.tick_rate_hz, self._publish_telemetry)
        self.arena = ArenaSearch(self)          # phase 3 (subscribes now)

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
        idx = int(self.cfg.rc_auto_channel) - 1      # physical CHn -> index n-1
        if not 0 <= idx < len(msg.channels):
            return
        pwm = msg.channels[idx]
        was_auto = self._rc_auto_active
        self._rc_auto_active = pwm > self.cfg.rc_auto_pwm_thresh
        if was_auto and not self._rc_auto_active:
            self._request_abort(
                "pilot", f"RC CH{idx + 1}={pwm} < {self.cfg.rc_auto_pwm_thresh}")

    def _sensor_live_cb(self, msg: Bool):
        self._sensor_live = bool(msg.data)

    def _coverage_ok_cb(self, msg: Bool):
        self._coverage_ok = bool(msg.data)

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
        if self._airborne and self._sensor_required():
            if self._sensor_live:
                self._sensor_bad_since = None
            else:
                now = time.time()
                if self._sensor_bad_since is None:
                    self._sensor_bad_since = now
                    self.get_logger().warn("[mission_node] Obstacle sensor NOT live")
                elif now - self._sensor_bad_since > self.cfg.sensor_fail_timeout_s:
                    raise MissionAbort(
                        "sensor",
                        f"obstacle sensor dead > {self.cfg.sensor_fail_timeout_s:.0f}s")

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

    def _sensor_required(self) -> bool:
        return self._sensor_check and bool(self.cfg.require_avoidance_sensor)

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
        """GUIDED goto with arrival check. Holds while the obstacle sensor is
        not live and re-sends the target once it recovers (the 10 s sensor
        timeout in _tick() turns a long outage into an abort).

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

        def send():
            yaw = hold_yaw_enu
            if face_target and self.gps is not None:
                yaw = mav.yaw_enu_towards(self.gps[0], self.gps[1], lat, lon)
            mav.goto(self, lat, lon, alt, publish_count=self.cfg.goto_publish_count, yaw_enu=yaw)

        send()
        holding = False
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._tick()
            if self._sensor_required() and not self._sensor_live:
                if not holding and self.gps is not None:
                    self.get_logger().warn("[mission_node] Sensor not live — holding position")
                    mav.goto(self, self.gps[0], self.gps[1], alt, publish_count=5,
                             yaw_enu=hold_yaw_enu)
                    holding = True
                    deadline += self.cfg.sensor_fail_timeout_s
            elif holding:
                self.get_logger().info("[mission_node] Sensor recovered — resuming goto")
                holding = False
                send()
            elif self.gps is not None and mav.haversine(*self.gps, lat, lon) <= tolerance:
                return True
            time.sleep(0.2)
        self.get_logger().error(f"[mission_node] Arrival timeout at ({lat:.7f}, {lon:.7f})")
        return False

    def _call_leg(self, srv_name: str, timeout: float):
        """Call a blocking path_planner leg service; cancels it on abort."""
        try:
            return srv.call_trigger(self, srv_name, call_timeout=timeout,
                                    should_abort=self._should_abort)
        except srv.CallAborted:
            self._stop_planner()
            self._tick()          # re-raise the MissionAbort that caused it
            raise

    def _stop_planner(self):
        try:
            srv.call_trigger(self, srv.SRV_PLANNER_STOP, call_timeout=3.0, wait_timeout=2.0)
        except Exception as e:      # best effort
            self.get_logger().warn(f"[mission_node] planner stop failed: {e}")

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
        if self.cfg.require_avoidance_sensor and not self._wait_for(
                lambda: self._sensor_live, self.cfg.sensor_preflight_timeout_s,
                "obstacle sensor live (/avoidance/sensor_live)"):
            raise MissionAbort("failure", "obstacle sensor not live before takeoff")

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
        # CH7 LOW during the wait still aborts.
        self._check_ground_drift(self.local_xy, ARM_WAIT_S, "on the ground after arming")
        if not self._armed():
            raise MissionAbort("failure", "disarmed during the pre-takeoff wait")
        # Re-anchor home to where the estimate says we are right now, so the
        # climb and later limits are relative to the actual lift-off point.
        if self.gps is not None:
            self.home = self.gps

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

    def _fly_leg(self, srv_name: str, timeout: float, label: str):
        res = self._call_leg(srv_name, timeout)
        if res is None:
            raise MissionAbort("failure", f"{label}: no response from path_planner")
        if not res.success:
            kind = "sensor" if res.message.startswith("SENSOR_TIMEOUT") else "failure"
            raise MissionAbort(kind, f"{label} failed: {res.message}")

    def _entry_corridor_phase(self):
        """Phase 2, shared by the mission and its test flight
        (entry_corridor_mission): back over the takeoff point at this altitude
        (finding / centring on the QR moved the drone), down to
        corridor_start_alt_m on the placed heading (the banner search is
        centred on it), then /path_planner/entry_corridor — +-90 deg banner
        search, line-up on the walls under it, corridor flown until the walls
        end — and straight up to post_entry_alt_m."""
        self._return_over_home()
        self._climb_in_place(self.cfg.corridor_start_alt_m, self.placed_yaw_enu())
        # In the leg path_planner owns the obstacle sensor timeout (as in the
        # exit corridor): its SENSOR_TIMEOUT first backs out from under the
        # banner, so the RTL climb that follows cannot hit it.
        self._sensor_check = False
        try:
            self._fly_leg(srv.SRV_ENTRY_CORRIDOR, self.cfg.entry_corridor_timeout_s,
                          "entry corridor")
        finally:
            self._sensor_check = True
            self._sensor_bad_since = None
        # The planner ends just past the walls, nose down the corridor:
        # corridor 1's far end and direction, for phase 5.
        if self.gps is not None and self.heading_deg is not None:
            self.c1_exit = (self.gps[0], self.gps[1], math.radians(90.0 - self.heading_deg))
            self.get_logger().info(
                f"[mission_node] Corridor 1 left at ({self.gps[0]:.7f}, {self.gps[1]:.7f}), "
                f"heading {self.heading_deg:.0f} deg")
        self._climb_in_place(self.cfg.post_entry_alt_m)

    def _exit_corridor_phase(self):
        """Phase 5: to where the exit corridor should be (corridor 1 mirrored
        across the takeoff point, path_planner/exit_corridor.py) at
        arena_alt_m around the red zones, face back down the corridors, down
        to corridor_start_alt_m, then /path_planner/exit_corridor — +-90 deg
        blue banner search, line-up, through with obstacles passed beside.
        Banner not found -> the next search point.

        True: through, just past the exit corridor's walls. False: it failed
        and the EXIT CORRIDOR FAILSAFE has run (_exit_failsafe: straight up
        above the walls, home, landed). Only this phase has that failsafe:
        any failure in it — a planner reply, a companion "failure" abort
        (e.g. red camera silent on the way there) or a crash — runs it;
        battery / pilot / FC aborts still raise MissionAbort as everywhere."""
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
        exit_failsafe_alt_m (above the corridor walls; path_planner has
        already backed out from under the banner if it failed at the mouth),
        then fly home and land — fully autonomous. If the altitude is not
        confirmed the vehicle lands in place instead: flying home lower could
        cross a wall.

        Home: once the arena has been flown (phases 3-5 plan in its frame),
        around the red zones at arena_alt_m with the red camera watching
        (ArenaSearch.go_to) — the red mat must never be flown over, at any
        height, and a straight line home from the arena side can cross it.
        No route / red camera dead -> MissionAbort: lands where it is (never
        over red). Arena never flown: straight home at exit_failsafe_alt_m.

        The obstacle sensor is not needed above the walls: its timeout is
        off from here on (a dead OAK-D may be why the corridor failed)."""
        alt = float(self.cfg.exit_failsafe_alt_m)
        self.get_logger().error(
            f"[mission_node] EXIT CORRIDOR FAILSAFE — {reason}: climbing straight up to "
            f"{alt:.1f} m (above the corridor walls), then return to launch and land")
        self._sensor_check = False
        self._stop_planner()
        for _ in range(2):
            self._climb_in_place(alt)
            if self.rel_alt is not None and self.rel_alt >= alt - self.cfg.altitude_tolerance_m:
                break
        else:
            raise MissionAbort("failure", f"exit corridor failsafe: {alt:.1f} m not reached "
                                          "— landing in place rather than fly home low")
        self.sm.c2_complete()
        if self.arena.frame is not None:
            if not self.arena.go_to(*self.home, "home (exit corridor failsafe)"):
                raise MissionAbort("failure", "exit corridor failsafe: no route home clear "
                                              "of the red zones — landing here")
        elif not self._goto_and_wait(*self.home, alt, self.cfg.arrival_tolerance_m,
                                     self.cfg.arrival_timeout_s, self.cfg.transit_speed_ms):
            self.get_logger().warn("[mission_node] Home not reached in time — landing here")
        self.sm.home_reached()
        self._land_and_wait()
        self.sm.touchdown()
        self.get_logger().warn(
            "[mission_node] Mission ended by the exit corridor failsafe — landed at home")

    def _fly_exit_corridor(self):
        """The exit corridor itself (see _exit_corridor_phase). None if
        through, else why it failed."""
        if self.c1_exit is None or self.home is None:
            return "corridor 1's exit was not recorded"
        lat1, lon1, axis = self.c1_exit
        d_n, d_e = mav.ne_offset(self.home[0], self.home[1], lat1, lon1)
        pts = search_points((d_e, d_n), axis, float(self.cfg.exit_search_standoff_m),
                            float(self.cfg.exit_search_shift_m))
        for i, ((e, n), facing) in enumerate(pts):
            label = f"exit corridor search point {i + 1}/{len(pts)}"
            lat, lon = mav.offset_gps(self.home[0], self.home[1], n, e)
            self.get_logger().info(
                f"[mission_node] {label}: ({e:+.1f} E, {n:+.1f} N) m from takeoff, "
                f"facing {math.degrees(facing):.0f} deg ENU")
            if not self.arena.go_to(lat, lon, label):
                continue
            self._climb_in_place(self.cfg.corridor_start_alt_m, facing)
            # In the leg path_planner owns the obstacle sensor timeout
            # (SENSOR_TIMEOUT, same sensor_fail_timeout_s): failing the leg
            # itself, it first backs out from under the banner — cancelled by
            # our own timeout it would not, and the failsafe's climb could
            # hit the banner.
            self._sensor_check = False
            try:
                res = self._call_leg(srv.SRV_EXIT_CORRIDOR, self.cfg.exit_corridor_timeout_s)
            finally:
                self._sensor_check = True
                self._sensor_bad_since = None
            if res is None:
                return "no response from path_planner"
            if res.success:
                return None
            if not res.message.startswith("NO_BANNER"):
                return res.message
            self.get_logger().warn(f"[mission_node] {label}: {res.message}")
        return "blue banner not found at any search point"

    def _return_over_home(self):
        """Fly level back over the takeoff point (the reference QR search and
        centring move the drone up to qr_max_radius_m), at the current
        altitude, at the QR phase's move speed. A missed arrival is logged,
        not an abort (GPS is only good to ~1 m)."""
        if self.gps is None or self.home is None or self.rel_alt is None:
            self.get_logger().warn("[mission_node] No GPS / home — banner search starts here")
            return
        dist = mav.haversine(*self.gps, *self.home)
        speed = float(self.cfg.qr_move_speed_ms)
        self.get_logger().info(
            f"[mission_node] Back over the takeoff point ({dist:.1f} m) at {self.rel_alt:.1f} m")
        if not self._goto_and_wait(
                *self.home, self.rel_alt, float(self.cfg.qr_move_tolerance_m),
                max(15.0, 4.0 * dist / max(speed, 0.1)), speed, face_target=False,
                hold_yaw_enu=self.placed_yaw_enu()):
            self.get_logger().warn("[mission_node] Takeoff point not reached — searching from here")

    def placed_yaw_enu(self):
        """The heading the drone was placed with, as an ENU yaw (rad), or None.
        Phases 1 and 2 hold it: the banner search is centred on it."""
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

    def _run_mission(self):
        try:
            self._wait_for_start()

            # Competition day: the arena's corners must be set (all zero =
            # a placeholder rectangle somewhere else entirely).
            if self.cfg.require_arena_corners and not (
                    any(self.cfg.arena_corners_lat) and any(self.cfg.arena_corners_lon)):
                raise MissionAbort("failure", "arena_corners_lat/lon not set in "
                                              "mission_params.yaml — not taking off")

            # Phase 1: gradual takeoff, then the reference QR read in the air
            # over the launch point and stored.
            self.sm.auto_activate()
            self._preflight()
            self._focus_camera(float(self.cfg.cruise_altitude_m))   # reference QR height
            self._takeoff()

            self.sm.airborne()
            self._log_clock("phase 1, reference QR")
            self.sm.mission_target_qr = self._reference_qr()
            if self.sm.mission_target_qr is None:
                self.get_logger().warn(
                    "[mission_node] Reference QR NOT read — continuing without a target "
                    "(no arena search, payload kept)")
            else:
                shown = self.sm.mission_target_qr or "(no text — matched by picture)"
                self.get_logger().info(f"[mission_node] Target locked: '{shown}'")

            # Phase 2: the entry corridor.
            self.sm.qr_scanned()
            self._log_clock("phase 2, entry corridor")
            self._entry_corridor_phase()

            self.sm.entry_complete()
            # Phase 3: the arena search (no reference QR -> nothing to match),
            # ended early by the mission clock so phases 4-5 still fit.
            target = None
            if self.sm.mission_target_qr is not None:
                self._log_clock("phase 3, arena search")
                until = self._arena_deadline()
                if until is None or until > time.time():
                    target = self.arena.run(until=until)
            if target is not None:
                self.sm.qr_matched()
                self._log_clock("phase 4, payload drop")
                self._drop_payload(*target)
                self.sm.drop_complete()
            else:
                self.get_logger().warn(
                    "[mission_node] Target QR not found — keeping payload, continuing")
                self.sm.search_exhausted()

            # Phase 5: the exit corridor. Failed -> its failsafe has already
            # climbed above the walls, flown home and landed.
            self._log_clock("phase 5, exit corridor")
            if not self._exit_corridor_phase():
                self._log_clock("landed (exit corridor failsafe)")
                return

            self.sm.c2_complete()
            self._log_clock("homing")
            self._climb_in_place(self.cfg.cruise_altitude_m)
            if not self._goto_and_wait(*self.home, self.cfg.cruise_altitude_m,
                                       self.cfg.arrival_tolerance_m, self.cfg.arrival_timeout_s,
                                       self.cfg.transit_speed_ms):
                self.get_logger().warn("[mission_node] Home not reached in time — landing here")

            self.sm.home_reached()
            self._land_and_wait()
            self.sm.touchdown()
            self._mission_complete = True
            self._log_clock("landed")
            self.get_logger().info("[mission_node] Mission complete!")

        except MissionAbort as a:
            self._handle_abort(a)
        except Exception as e:
            import traceback
            self.get_logger().error(f"[mission_node] Mission thread crashed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._handle_abort(MissionAbort("failure", f"crash: {e}"))

    def _handle_abort(self, a: MissionAbort):
        self.get_logger().error(f"[mission_node] MISSION ABORTED — {a}")
        self._landing = True     # mode changes from here on are ours or the FC's
        self._stop_planner()
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
        """Sensor-dead failsafe: RTL (FC climbs to RTL_ALT, flies home,
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
            "sensor_live": self._sensor_live,
            "coverage_ok": self._coverage_ok,
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
