# path_planner_node.py — ROS2 node that flies the mission legs via MAVROS
#
# Trigger services consumed by mission_manager (each blocks until the leg is
# done, cancelled, or failed):
#   /path_planner/fly_corridor_1   — corridor 1 (clean)
#   /path_planner/fly_corridor_2   — corridor 2 (static obstacles)
#   /path_planner/fly_corridor_here — corridor test: from HERE along the
#                                    current heading, until the walls end
#                                    (no surveyed waypoints, no set length)
#   /path_planner/entry_corridor   — mission phase 2: find the GREEN banner,
#                                    line up on the corridor under it, fly
#                                    through at the middle of the opening
#                                    (entry_corridor.py; width, length and
#                                    altitude all measured, none configured)
#   /path_planner/exit_corridor    — mission phase 5: the same with the BLUE
#                                    banner (/corridor/banner_blue), obstacles
#                                    ON: passed beside only with
#                                    exit_pass_side_clearance_m each side, else
#                                    flown OVER (a centred obstacle), climbing
#                                    only once past the banner; neither -> beside
#                                    at side_clearance_m; no gap -> BLOCKED.
#                                    The caller places the drone in front of
#                                    the exit corridor facing it first.
#   /path_planner/run_lawnmower    — BCD lawnmower search (arena_speed_ms)
#   /path_planner/stop             — cancel the active leg and hold position
#
# Response message is a machine-readable token first:
#   OK | CANCELLED | BUSY | NO_GPS | BLOCKED | WP_TIMEOUT ... |
#   SENSOR_TIMEOUT ... | LEG_TIMEOUT ...
#
# Navigation: GUIDED GlobalPositionTarget setpoints (mavros_utils.goto) with
# the plain position controller (FC: GUID_OPTIONS bit 6 clear, OA_TYPE 0), so
# speed follows MAV_CMD_DO_CHANGE_SPEED. Each leg first sends a hold target,
# then sets the speed (entering a GUIDED submode resets the speed limit).
#
# CORRIDORS (11.5 ft = 3.5 m wide). All corridors are flown the same way:
#   1. get to the corridor's line-up point (surveyed legs: the first
#      waypoint, a few metres before the mouth, at cruise altitude around red
#      zones; banner legs: straight out in front of the banner)
#   2. face along the corridor and change to the corridor altitude there
#   3. follow the corridor's GPS CENTRELINE (corridor_follow.GpsCorridorTrack:
#      the surveyed waypoints or the banner fix), corrected by the walls
#      whenever /corridor/perception sees one or both — gated against GPS,
#      at most corridor_fuse_max_correction_m. No walls in view is normal in
#      an 11.5 ft corridor (OAK-D bench: both walls in 81 of ~2000 frames) and
#      does NOT stop the vehicle. At corridor_rate_hz a target goes
#      lookahead_m ahead along that line, shifted across to the planned
#      lateral position (centre, or the free side of an obstacle) and at the
#      planned altitude (nominal, or over an obstacle). Logic:
#      corridor_follow.py. Obstacles come from perception with or without a
#      wall fit.
#   4. exit, whichever comes first:
#        - blue ground marker (/corridor/ground_marker) past half the corridor
#          length, then exit_clearance_m more
#        - the walls' end, measured ahead by perception, + exit_clearance_m
#        - surveyed legs: the full length + exit_clearance_m
#        - banner legs: the expected length (the entry corridor's measured
#          one, else corridor_nominal_length_m) + exit_clearance_m with no
#          obstacle still to pass; hard stop at + max_overrun_m
#   5. surveyed legs: climb vertically to cruise altitude where the leg ended.
# The green ground marker (entrance) is logged as a cross-check.
# BendyRuler is NOT used (docs/OA_EVALUATION.md: in SITL it left the corridor
# in every run).
#
# Sensor gating:
#   lawnmower/transit: sensor_live False -> hold; resume when live again
#   corridor: GPS lost, sensor_live False, or (obstacle legs) no perception
#             message for corridor_perception_stale_s -> hold; after
#             coverage_backoff_after_s back off coverage_backoff_m along the
#             corridor; resume when it recovers. Walls out of view is NOT a
#             fault.
#   unhealthy > sensor_fail_timeout_s -> fail the leg with SENSOR_TIMEOUT.
#   An obstacle that can be neither passed nor climbed holds the vehicle;
#   after sensor_fail_timeout_s -> BLOCKED. The exit corridor failing (any
#   reason) -> mission_node's exit failsafe: straight up above the walls,
#   home, land (this node first backs out from under the banner if it failed
#   within exit_banner_clear_m of the mouth).
#
# Usage:
#   ros2 run path_planner path_planner_node

import math
import threading
import time
from dataclasses import replace

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Float32MultiArray, Float64, String
from sensor_msgs.msg import NavSatFix, NavSatStatus

from mission_common import banner_msg
from mission_common import corridor_msg
from mission_common import mavros_utils as mav
from mission_common.config_loader import load_mission_config
from path_planner.corridor_follow import (
    FollowParams,
    FuseParams,
    GpsCorridorTrack,
    ObstacleMemory,
    plan,
)
from path_planner.entry_corridor import (
    BannerFix,
    EntryParams,
    approach_command,
    banner_view,
    point_out,
    search_altitudes,
    sweep_angle,
    sweep_duration,
    sweep_points,
    target_altitude,
)
from path_planner.waypoint_gen import (
    build_transit_space,
    generate_mission_waypoints,
    gps_to_local,
    local_to_gps,
    plan_path,
)

_DOUBLE_ARRAY = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)


NAN_F = float("nan")


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")
        self.cfg = load_mission_config(self)

        # ----- Course geometry (local ENU metres about origin) -----
        # PLACEHOLDERS — set the real corridors on competition day (yaml).
        self._corridor1_local = self._xy_param(
            "corridor1_waypoints", [-3.0, 13.0], [15.0, 15.0])
        self._corridor2_local = self._xy_param(
            "corridor2_waypoints", [13.0, -3.0], [-5.0, -5.0])

        defaults = {
            1: ([24.0, 36.0, 36.0, 24.0], [12.0, 12.0, 22.0, 22.0]),
            2: ([10.0, 15.0, 12.5], [5.0, 5.0, 10.0]),
            3: ([5.0, 10.0, 10.0, 7.0, 7.0, 5.0], [20.0, 20.0, 22.0, 22.0, 25.0, 25.0]),
        }
        self._red_zones = []
        for i, (dx, dy) in defaults.items():
            zone = self._xy_param(f"red_zone_{i}", dx, dy, suffix_x="_x", suffix_y="_y")
            if len(zone) >= 3:
                self._red_zones.append(zone)

        edge_margin = float(self.cfg.arena_edge_margin_m)   # declared by load_mission_config
        self._declare_corridor_params()

        # ----- Lawnmower (routed around red zones) -----
        wps = generate_mission_waypoints(
            origin_lat=self.cfg.origin_lat,
            origin_lon=self.cfg.origin_lon,
            arena_width=self.cfg.arena_width_m,
            arena_height=self.cfg.arena_height_m,
            altitude=self.cfg.cruise_altitude_m,
            hfov_deg=self.cfg.down_cam_hfov_deg,
            overlap=self.cfg.strip_overlap,
            red_zones_vertices=self._red_zones,
            red_zone_buffer=self.cfg.red_zone_buffer_m,
            entry_x=self._corridor1_local[-1][0],
            entry_y=self._corridor1_local[-1][1],
            speed_ms=self.cfg.arena_speed_ms,
            edge_margin=edge_margin,
        )
        self._lawnmower_local = wps["local_waypoints"]
        self.get_logger().info(
            f"[path_planner] {len(self._lawnmower_local)} lawnmower waypoints, "
            f"strip={wps['strip_width']:.1f} m, est. {wps['estimated_dist']:.0f} m / "
            f"{wps['estimated_time'] / 60:.1f} min")

        # Free space for transit hops (corridor ends, drop point -> corridor 2).
        self._transit_space = build_transit_space(
            self._corridor1_local + self._corridor2_local + self._lawnmower_local,
            self._red_zones, self.cfg.red_zone_buffer_m)

        # ----- Live data -----
        self._gps = None
        self._rel_alt = None
        self._yaw_enu = None            # rad, from compass heading
        self._sensor_live = False
        self._perception = None         # (dict, arrival time) newest, unconsumed
        # (dict, arrival time) newest message per banner colour; the active
        # banner leg reads one of them through self._banner
        self._banners = {"green": None, "blue": None}
        self._banner_colour = "green"
        # Length (mouth -> wall end) of the last corridor flown with both
        # measured — the entry corridor. The exit corridor is the same size:
        # where its own wall end cannot be measured (obstacles hide it, the
        # last ~2.5 m of a wide corridor has no wall in view) this is used.
        self._measured_len = None
        self._marker = "none"

        # ----- Run control -----
        self._run_lock = threading.Lock()       # one leg at a time
        self._cancel = threading.Event()
        self._leg_deadline = 0.0

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._gps_cb, mavros_qos)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt", self._alt_cb, mavros_qos)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg", self._hdg_cb, mavros_qos)
        self.create_subscription(Bool, "/avoidance/sensor_live", self._live_cb, 10)
        self.create_subscription(Float32MultiArray, "/corridor/perception", self._perception_cb, 10)
        for colour, topic in (("green", banner_msg.TOPIC_GREEN), ("blue", banner_msg.TOPIC_BLUE)):
            self.create_subscription(Float32MultiArray, topic,
                                     lambda m, c=colour: self._banner_cb(m, c), 10)
        self.create_subscription(String, "/corridor/ground_marker", self._marker_cb, 10)

        # Leg services block for the whole leg: reentrant group + multithreaded
        # executor so /stop and the subscriptions keep running meanwhile.
        cb = ReentrantCallbackGroup()
        self.create_service(Trigger, "/path_planner/fly_corridor_1",
                            self._handle_corridor_1, callback_group=cb)
        self.create_service(Trigger, "/path_planner/fly_corridor_2",
                            self._handle_corridor_2, callback_group=cb)
        self.create_service(Trigger, "/path_planner/fly_corridor_here",
                            self._handle_corridor_here, callback_group=cb)
        self.create_service(Trigger, "/path_planner/entry_corridor",
                            self._handle_entry_corridor, callback_group=cb)
        self.create_service(Trigger, "/path_planner/exit_corridor",
                            self._handle_exit_corridor, callback_group=cb)
        self.create_service(Trigger, "/path_planner/run_lawnmower",
                            self._handle_run_lawnmower, callback_group=cb)
        self.create_service(Trigger, "/path_planner/stop",
                            self._handle_stop, callback_group=cb)

        self.get_logger().info(
            "[path_planner] Ready — fly_corridor_1, fly_corridor_2, fly_corridor_here, "
            "entry_corridor, exit_corridor, run_lawnmower, stop")

    # ===================================================================
    # Params / callbacks
    # ===================================================================

    def _xy_param(self, name, default_x, default_y, suffix_x="_x", suffix_y="_y"):
        nx, ny = name + suffix_x, name + suffix_y
        for n, d in ((nx, default_x), (ny, default_y)):
            if not self.has_parameter(n):
                self.declare_parameter(n, [float(v) for v in d], _DOUBLE_ARRAY)
        xs = list(self.get_parameter(nx).value)
        ys = list(self.get_parameter(ny).value)
        if len(xs) != len(ys):
            raise ValueError(f"{nx} and {ny} must have the same length")
        return list(zip(xs, ys))

    def _declare_corridor_params(self):
        d = FollowParams()
        decl = {
            "corridor_altitude_m": 1.5,
            "corridor_follow_speed_ms": 0.5,
            "corridor_rate_hz": 5.0,
            "corridor_exit_clearance_m": 1.0,
            "corridor_max_overrun_m": 3.0,
            "corridor_exit_window_m": 3.0,      # a wall end must be this near the surveyed end
            "require_corridor_perception": True,
            "corridor_width_m": 3.5052,         # known width (the GPS line's walls)
            "corridor_nominal_length_m": 6.7056,  # banner legs: end here (+ exit
                                                # clearance) if no wall end is
                                                # measured; 0 = only a measured end
            "corridor_end_visible_m": 3.5,      # unknown length: a wall end this
                                                # near must be in view
            "corridor_perception_stale_s": 1.0,  # obstacle legs: no perception
                                                # message this long = sensor lost
            "exit_banner_clear_m": 1.0,         # exit corridor failure this near the
                                                # mouth (under the banner): back out
                                                # this far before handing back
            "exit_pass_side_clearance_m": 0.40,  # exit corridor: pass beside only with
                                                # this much air each side, else fly over
            "body_above_camera_m": 0.20,        # PLACEHOLDER: camera to the drone's top
                                                # (props / GPS mast): banner ceiling
            "banner_climb_after_m": 0.7,        # no climb until this far past the mouth
                                                # (half the drone's length + GPS error)
            "drone_width_m": d.drone_width_m,
            "side_clearance_m": d.side_clearance_m,
            "corridor_lookahead_m": d.lookahead_m,
            "obstacle_stop_dist_m": d.stop_dist_m,
            "obstacle_length_m": d.obstacle_length_m,
            "climb_clearance_m": d.climb_clearance_m,
            "body_below_camera_m": d.body_below_camera_m,
            "corridor_max_altitude_m": d.max_altitude_m,
            "corridor_centre_slow_m": d.centre_slow_m,
            "corridor_centre_min_advance": d.centre_min_advance,
        }
        for name, default in decl.items():
            self.declare_parameter(name, default)
        gp = lambda n: self.get_parameter(n).value          # noqa: E731
        self._follow = FollowParams(
            drone_width_m=float(gp("drone_width_m")),
            side_clearance_m=float(gp("side_clearance_m")),
            lookahead_m=float(gp("corridor_lookahead_m")),
            stop_dist_m=float(gp("obstacle_stop_dist_m")),
            obstacle_length_m=float(gp("obstacle_length_m")),
            climb_clearance_m=float(gp("climb_clearance_m")),
            body_below_camera_m=float(gp("body_below_camera_m")),
            max_altitude_m=float(gp("corridor_max_altitude_m")),
            centre_slow_m=float(gp("corridor_centre_slow_m")),
            centre_min_advance=float(gp("corridor_centre_min_advance")),
        )
        self._c_alt = float(gp("corridor_altitude_m"))
        self._c_speed = float(gp("corridor_follow_speed_ms"))
        self._c_period = 1.0 / float(gp("corridor_rate_hz"))
        self._c_exit_clear = float(gp("corridor_exit_clearance_m"))
        self._c_overrun = float(gp("corridor_max_overrun_m"))
        self._c_exit_window_m = float(gp("corridor_exit_window_m"))
        self._c_need_perception = bool(gp("require_corridor_perception"))
        self._c_width = float(gp("corridor_width_m"))
        self._c_end_visible_m = float(gp("corridor_end_visible_m"))
        self._c_nominal_len = float(gp("corridor_nominal_length_m"))
        self._c_perc_stale = float(gp("corridor_perception_stale_s"))
        self._exit_banner_clear = float(gp("exit_banner_clear_m"))
        self._exit_pass_side = float(gp("exit_pass_side_clearance_m"))
        self._body_above_cam = float(gp("body_above_camera_m"))
        self._banner_climb_after = float(gp("banner_climb_after_m"))
        self._banner_bottom = None      # bottom edge height of the banner last fixed (m)
        f = FuseParams()
        for k, v in vars(f).items():
            self.declare_parameter(f"corridor_fuse_{k}", v)
        self._fuse = FuseParams(**{k: float(gp(f"corridor_fuse_{k}")) for k in vars(f)})
        e = EntryParams()
        entry = {f"entry_{k}": v for k, v in vars(e).items()}
        for name, default in entry.items():
            self.declare_parameter(name, default)
        self._entry = EntryParams(**{
            k: type(v)(gp(f"entry_{k}")) for k, v in vars(e).items()})
        # Equal is allowed: a hard altitude cap. Climbing over is then
        # impossible, so an obstacle with no side gap holds -> BLOCKED.
        if self._c_alt > self._follow.max_altitude_m:
            raise ValueError("corridor_altitude_m must not exceed corridor_max_altitude_m")
        self.get_logger().info(
            f"[path_planner] corridor: alt {self._c_alt:.2f} m (max "
            f"{self._follow.max_altitude_m:.2f}), {self._c_speed:.2f} m/s, drone "
            f"width {self._follow.drone_width_m:.2f} m + {self._follow.side_clearance_m:.2f} m "
            f"clearance, perception {'REQUIRED' if self._c_need_perception else 'OFF (GPS centreline)'}; "
            f"GPS centreline primary, walls correct it by <= {self._fuse.max_correction_m:.1f} m")

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self._gps = (msg.latitude, msg.longitude)

    def _alt_cb(self, msg: Float64):
        self._rel_alt = float(msg.data)

    def _hdg_cb(self, msg: Float64):
        # compass_hdg: degrees clockwise from North -> ENU yaw (CCW from East)
        self._yaw_enu = _wrap(math.radians(90.0 - float(msg.data)))

    def _live_cb(self, msg: Bool):
        self._sensor_live = bool(msg.data)

    def _perception_cb(self, msg: Float32MultiArray):
        m = corridor_msg.decode(msg)
        if m is None:
            self.get_logger().error(
                "[path_planner] /corridor/perception layout mismatch — rebuild "
                "obstacle_avoidance and path_planner together", throttle_duration_sec=5.0)
            return
        self._perception = (m, time.time())

    def _banner_cb(self, msg: Float32MultiArray, colour: str):
        m = banner_msg.decode(msg)
        if m is None:
            self.get_logger().error(
                f"[path_planner] {colour} banner layout mismatch — rebuild "
                "obstacle_avoidance and path_planner together", throttle_duration_sec=5.0)
            return
        self._banners[colour] = (m, time.time())

    @property
    def _banner(self):
        """Newest message of the active leg's banner (_banner_colour)."""
        return self._banners[self._banner_colour]

    @_banner.setter
    def _banner(self, value):
        self._banners[self._banner_colour] = value

    def _marker_cb(self, msg: String):
        self._marker = msg.data

    # ===================================================================
    # Shared helpers
    # ===================================================================

    def _to_gps(self, xy):
        return local_to_gps(xy[0], xy[1], self.cfg.origin_lat, self.cfg.origin_lon)

    def _to_local(self, latlon):
        return gps_to_local(latlon[0], latlon[1], self.cfg.origin_lat, self.cfg.origin_lon)

    def _sensor_ok(self):
        return not self.cfg.require_avoidance_sensor or self._sensor_live

    def _hold(self, alt, yaw=None):
        if self._gps is not None:
            mav.goto(self, self._gps[0], self._gps[1], alt, publish_count=5, yaw_enu=yaw)

    def _set_speed(self, speed, alt):
        self._hold(alt)                 # enter the submode (resets speed limits)...
        mav.change_speed(self, speed)   # ...then set the speed

    def _wait_alt(self, alt, timeout, tol=None):
        tol = self.cfg.altitude_tolerance_m if tol is None else tol
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._cancel.is_set() or not rclpy.ok():
                return False
            if self._rel_alt is not None and abs(self._rel_alt - alt) <= tol:
                return True
            time.sleep(0.2)
        return False

    def _fly_to(self, latlon, alt, speed, leg):
        """Fly one waypoint, nose along the direction of travel.
        Returns 'OK', 'CANCELLED', 'WP_TIMEOUT', 'SENSOR_TIMEOUT', 'LEG_TIMEOUT'."""
        dist = mav.haversine(*self._gps, *latlon)
        wp_deadline = time.time() + 10.0 + 2.5 * dist / max(speed, 0.1)

        def send():
            yaw = mav.yaw_enu_towards(self._gps[0], self._gps[1], latlon[0], latlon[1])
            mav.goto(self, latlon[0], latlon[1], alt,
                     publish_count=self.cfg.goto_publish_count, yaw_enu=yaw)

        send()
        bad_since = None
        while True:
            if self._cancel.is_set() or not rclpy.ok():
                return "CANCELLED"
            now = time.time()
            if not self._sensor_ok():
                if bad_since is None:
                    bad_since = now
                    self.get_logger().warn(f"[path_planner] {leg}: obstacle sensor not live — holding")
                    self._hold(alt)
                if now - bad_since > self.cfg.sensor_fail_timeout_s:
                    return "SENSOR_TIMEOUT"
                # Pausing must not eat the waypoint / leg budget.
                wp_deadline += 0.2
                self._leg_deadline += 0.2
                time.sleep(0.2)
                continue
            if bad_since is not None:
                self.get_logger().info(f"[path_planner] {leg}: sensor OK — resuming")
                bad_since = None
                send()

            if mav.haversine(*self._gps, *latlon) <= self.cfg.mavros_arrival_tolerance_m:
                return "OK"
            if now > self._leg_deadline:
                return "LEG_TIMEOUT"
            if now > wp_deadline:
                return "WP_TIMEOUT"
            time.sleep(0.2)

    def _transit(self, leg, target_local, alt, speed, skip_timed_out=False, extra=()):
        """Route from here to target_local (+ extra waypoints) around red zones."""
        start = self._to_local(self._gps)
        approach = plan_path(start, target_local, self._transit_space)[1:-1]
        route = [self._to_gps(p) for p in approach + [target_local] + list(extra)]
        for i, wp in enumerate(route):
            self.get_logger().info(
                f"[path_planner] {leg} WP{i + 1}/{len(route)}: ({wp[0]:.7f}, {wp[1]:.7f})")
            result = self._fly_to(wp, alt, speed, leg)
            if result == "WP_TIMEOUT" and skip_timed_out:
                self.get_logger().warn(f"[path_planner] {leg}: WP{i + 1} timeout, skipping")
                continue
            if result != "OK":
                return f"{result} {leg} at WP{i + 1}/{len(route)}"
        return None

    def _run(self, leg, timeout, body):
        """One leg at a time, with a deadline; body() returns (ok, message)."""
        if not self._run_lock.acquire(blocking=False):
            return False, "BUSY another leg is active"
        try:
            self._cancel.clear()
            if self._gps is None:
                return False, "NO_GPS"
            self._leg_deadline = time.time() + timeout
            return body()
        except Exception as e:      # a bug must fail the leg, not kill the node
            import traceback
            self.get_logger().error(f"[path_planner] {leg} crashed:\n{traceback.format_exc()}")
            self._hold(self._rel_alt if self._rel_alt is not None else self.cfg.cruise_altitude_m)
            return False, f"ERROR {leg}: {type(e).__name__}: {e}"
        finally:
            self._run_lock.release()

    # ===================================================================
    # Lawnmower
    # ===================================================================

    def _lawnmower(self):
        alt = self.cfg.cruise_altitude_m
        self._set_speed(self.cfg.arena_speed_ms, alt)
        wps = self._lawnmower_local
        err = self._transit("lawnmower", wps[0], alt, self.cfg.arena_speed_ms,
                            skip_timed_out=True, extra=wps[1:])
        return (False, err) if err else (True, "OK lawnmower complete")

    # ===================================================================
    # Corridor
    # ===================================================================

    def _corridor(self, leg, local_wps):
        cruise = self.cfg.cruise_altitude_m
        entry, end = self._to_gps(local_wps[0]), self._to_gps(local_wps[-1])
        d_n, d_e = mav.ne_offset(entry[0], entry[1], end[0], end[1])
        length = math.hypot(d_n, d_e)
        if length < 2.0:
            return False, f"CONFIG {leg}: first and last waypoint {length:.1f} m apart"
        axis = (d_n / length, d_e / length)
        axis_yaw = math.atan2(d_n, d_e)

        # 1. transit at cruise altitude to the point before the mouth
        self._set_speed(self.cfg.transit_speed_ms, cruise)
        err = self._transit(leg, local_wps[0], cruise, self.cfg.transit_speed_ms)
        if err:
            return False, err

        # 2. face along the corridor, change to corridor altitude
        self.get_logger().info(
            f"[path_planner] {leg}: at entry, {length:.1f} m corridor heading "
            f"{math.degrees(axis_yaw):.0f} deg ENU — to {self._c_alt:.2f} m")
        mav.goto(self, entry[0], entry[1], self._c_alt, publish_count=10, yaw_enu=axis_yaw)
        if not self._wait_alt(self._c_alt, 30.0, tol=2 * self._follow.alt_tol_m):
            return False, f"{'CANCELLED' if self._cancel.is_set() else 'WP_TIMEOUT'} {leg} corridor altitude"
        mav.change_speed(self, self._c_speed)

        # 3-5. follow
        result = self._follow_corridor(leg, entry, axis, axis_yaw, length)

        # 6. climb straight up where we are (never diagonally across a wall)
        if not result[1].startswith("CANCELLED"):
            self._hold(cruise)
            self._wait_alt(cruise, 30.0)
        return result

    def _corridor_here(self, leg):
        """Corridor test leg: entry = here, axis = current heading (the caller
        has pointed the nose down the corridor). No length: it ends where the
        walls are seen to end. Same follower as the surveyed legs; ends holding
        where it finished at corridor altitude (no climb to cruise — test
        sites have trees)."""
        if self._yaw_enu is None:
            return False, f"NO_HEADING {leg}"
        if not self._c_need_perception:
            return False, f"CONFIG {leg}: no set length, so the exit must be seen — " \
                          "needs require_corridor_perception"
        entry, axis_yaw = self._gps, self._yaw_enu
        axis = (math.sin(axis_yaw), math.cos(axis_yaw))         # (north, east)
        self.get_logger().info(
            f"[path_planner] {leg}: from here, heading {math.degrees(axis_yaw):.0f} deg ENU, "
            f"until the walls end — to {self._c_alt:.2f} m")
        mav.goto(self, entry[0], entry[1], self._c_alt, publish_count=10, yaw_enu=axis_yaw)
        if not self._wait_alt(self._c_alt, 30.0, tol=2 * self._follow.alt_tol_m):
            return False, f"{'CANCELLED' if self._cancel.is_set() else 'WP_TIMEOUT'} {leg} corridor altitude"
        mav.change_speed(self, self._c_speed)
        result = self._follow_corridor(leg, entry, axis, axis_yaw, None)
        if not result[1].startswith("CANCELLED"):
            self._hold(self._rel_alt if self._rel_alt is not None else self._c_alt)
        return result

    # ===================================================================
    # Banner corridors: entry (phase 2, green) and exit (phase 5, blue) —
    # logic and rationale in entry_corridor.py
    # ===================================================================

    def _banner_corridor(self, leg, colour, obstacles, climb):
        """Find the `colour` banner, line up on the walls under it, fly
        through at the middle of the opening until the walls end. Ends
        holding where it finished (the next phase decides what follows).
        obstacles: pass obstacles (False: a clean corridor). climb False: never
        above the corridor altitude — obstacles are only passed beside."""
        if not self._c_need_perception:
            return False, f"CONFIG {leg}: needs corridor perception (require_corridor_perception)"
        self._banner_colour = colour
        topic = banner_msg.TOPIC_GREEN if colour == "green" else banner_msg.TOPIC_BLUE
        deadline = time.time() + 5.0
        while self._banner is None or time.time() - self._banner[1] > 1.0:
            if self._cancel.is_set() or time.time() > deadline:
                return False, (f"NO_BANNER_FEED {leg}: no {topic} — is the {colour} banner_node "
                               "running and the OAK colour stream on (i_pipeline_type RGBD)?")
            time.sleep(0.1)
        alt0 = self._rel_alt if self._rel_alt is not None else self.cfg.cruise_altitude_m
        self._set_speed(self._c_speed, alt0)

        ok, msg = self._banner_search(leg)
        if not ok:
            return False, msg
        ok, msg, nominal, track, entry = self._banner_approach(leg)
        if not ok:
            return False, msg

        self._leg_deadline = time.time() + self.cfg.corridor_timeout_s
        axis_yaw = track.axis_yaw
        axis = (math.sin(axis_yaw), math.cos(axis_yaw))         # (north, east)
        # Where it ends if no wall end is measured: the entry corridor's
        # measured length (exit corridor), else the nominal (22 ft) length.
        expected = (self._measured_len if obstacles else None) or \
            (self._c_nominal_len if self._c_nominal_len > 0.0 else None)
        self.get_logger().info(
            f"[path_planner] {leg}: expected length "
            + (f"{expected:.1f} m" if expected else "unknown — the wall end must be seen")
            + (" (the entry corridor's)" if obstacles and self._measured_len else ""))
        result = self._follow_corridor(leg, entry, axis, axis_yaw, None, nominal=nominal,
                                       track=track, obstacles=obstacles, alt_gate=True,
                                       climb=climb, expected_len=expected,
                                       mouth_at=self._entry.preentry_m,
                                       wall_guard=obstacles, banner_bottom=self._banner_bottom,
                                       pass_side_clearance=self._exit_pass_side
                                       if obstacles else None)
        if not result[0] and not result[1].startswith("CANCELLED"):
            self._clear_banner(leg, entry, axis, nominal)
        if not result[1].startswith("CANCELLED"):
            self._hold(self._rel_alt if self._rel_alt is not None else nominal)
        return result

    def _clear_banner(self, leg, entry, axis, alt):
        """A corridor leg failed: mission_node climbs straight up from where
        the vehicle is (exit failsafe, or RTL on a dead sensor). Within exit_banner_clear_m of the
        mouth that climb would go into the banner across its top: back out
        along the way in (open arena) first."""
        s, mouth = self._last_s, self._last_mouth
        if s is None or mouth is None or abs(s - mouth) > self._exit_banner_clear or \
                self._gps is None:
            return
        back = s - (mouth - self._exit_banner_clear - 0.3)
        lat, lon = mav.offset_gps(self._gps[0], self._gps[1], -axis[0] * back, -axis[1] * back)
        self.get_logger().warn(
            f"[path_planner] {leg}: failed under the banner (s={s:.1f} m, mouth {mouth:.1f} m) "
            f"— backing out {back:.1f} m before the climb")
        mav.goto(self, lat, lon, alt, publish_count=5,
                 yaw_enu=math.atan2(axis[0], axis[1]))
        deadline = time.time() + 10.0 + 4.0 * back / max(self._c_speed, 0.1)
        while time.time() < deadline and not self._cancel.is_set():
            if mav.haversine(*self._gps, lat, lon) < 0.3:
                break
            time.sleep(0.2)

    def _banner_fresh(self, max_age=1.0):
        """Newest banner message if it is recent and has a banner, else None."""
        b = self._banner
        if b is None or time.time() - b[1] > max_age or b[0]["state"] < banner_msg.HOLD:
            return None
        return b

    def _banner_yaw(self, b):
        """World direction (ENU rad) to the banner centre."""
        yaw = b["cam_yaw"] if not math.isnan(b["cam_yaw"]) else (self._yaw_enu or 0.0)
        return _wrap(yaw + b["bearing"])

    def _banner_locked(self):
        """The newest banner message if it is a fresh LOCK (not coasting), else None."""
        b = self._banner_fresh(0.5)
        return b[0] if b is not None and b[0]["state"] == banner_msg.LOCK else None

    def _sweep(self, leg, centre, pts, lat, lon, alt):
        """Turn the nose through pts (angles from centre, ENU rad; see
        sweep_points) at (lat, lon, alt) until the banner locks. Returns
        (banner, None), (None, None) if not seen, or (None, error)."""
        rate = math.radians(self._entry.search_yaw_rate_deg_s)
        t_sweep, sweep_s = time.time(), sweep_duration(pts, rate)
        found = self._banner_locked()
        while found is None and time.time() - t_sweep < sweep_s:
            t0 = time.time()
            if self._cancel.is_set() or not rclpy.ok():
                return None, f"CANCELLED {leg}"
            if t0 > self._leg_deadline:
                return None, f"LEG_TIMEOUT {leg} banner search"
            yaw = _wrap(centre + sweep_angle(pts, rate, t0 - t_sweep))
            mav.goto(self, lat, lon, alt, publish_count=1, yaw_enu=yaw, log=False)
            self._sleep_rest(t0)
            found = self._banner_locked()
        return found, None

    def _banner_search(self, leg):
        """Sweep the nose entry_search_half_angle_deg each side of the start
        heading (left first) at each search altitude until the banner locks."""
        ep = self._entry
        start = self._gps
        alts = search_altitudes(self._rel_alt if self._rel_alt is not None else ep.search_alt_min_m, ep)
        rate = math.radians(ep.search_yaw_rate_deg_s)
        sweeps = [sweep_points(i, ep) for i in range(len(alts))]
        self._leg_deadline = time.time() + sum(sweep_duration(s, rate) + 35.0 for s in sweeps)
        # The drone was placed facing the corridors and phase 1 held that
        # heading: the sweep is centred on it.
        centre = self._yaw_enu or 0.0
        span = ("full turn" if ep.search_half_angle_deg >= 180.0
                else f"+-{ep.search_half_angle_deg:.0f} deg")

        found = self._banner_locked()
        for i, a in enumerate(alts):
            if found is not None:
                break
            if i > 0:
                self.get_logger().info(
                    f"[path_planner] {leg}: banner not seen — searching at {a:.1f} m")
                mav.goto(self, start[0], start[1], a, publish_count=5, yaw_enu=self._yaw_enu)
                reached = self._wait_alt(a, 30.0, tol=2 * self._follow.alt_tol_m)
                found = self._banner_locked()
                if not reached and found is None:
                    if self._cancel.is_set():
                        return False, f"CANCELLED {leg}"
                    return False, f"WP_TIMEOUT {leg}: search altitude {a:.1f} m not reached"
            else:
                self.get_logger().info(
                    f"[path_planner] {leg}: searching for the {self._banner_colour} banner at {a:.1f} m "
                    f"({span} around {math.degrees(centre):.0f} deg ENU, "
                    f"{ep.search_yaw_rate_deg_s:.0f} deg/s)")
            if found is None:
                found, err = self._sweep(leg, centre, sweeps[i], start[0], start[1], a)
                if err is not None:
                    return False, err
        if found is None:
            return False, (f"NO_BANNER {leg}: not found sweeping {span} at "
                           + ", ".join(f"{a:.1f}" for a in alts) + " m")
        yaw = self._banner_yaw(found)
        alt = self._rel_alt if self._rel_alt is not None else alts[0]
        self.get_logger().info(
            f"[path_planner] {leg}: BANNER found at {math.degrees(yaw):.0f} deg ENU "
            f"({alt:.1f} m) — turning to it")
        mav.goto(self, self._gps[0], self._gps[1], alt, publish_count=5, yaw_enu=yaw)
        return True, "OK"

    def _banner_approach(self, leg):
        """Fly toward the banner until it is fixed (BannerFix: its shape read
        frame by frame, its height from frames at different places or
        heights), then to the line-up point straight out in front of the
        entrance, nose on the banner. There the corridor's GPS line is the
        banner fix (its middle, the corridor straight back from it); walls
        seen during entry_lineup_settle_s correct it (GpsCorridorTrack —
        gated, so another corridor's walls are rejected) but are not needed:
        the OAK-D rarely sees an 11.5 ft corridor's walls from out there.
        Returns (ok, message, corridor altitude, track, entry gps) — the
        track is in the frame of `entry` (the line-up point) along the
        corridor. Steps and rationale: entry_corridor.py header."""
        ep, fp = self._entry, self._follow
        self._leg_deadline = time.time() + ep.approach_timeout_s
        origin = self._gps                  # local (east, north) frame

        def here():
            d_n, d_e = mav.ne_offset(origin[0], origin[1], self._gps[0], self._gps[1])
            return d_e, d_n

        track = None                        # GpsCorridorTrack, from the line-up point
        entry = None                        # its frame origin (gps)
        lineup_t = None
        fix = BannerFix(ep)
        place = None                        # (middle, corridor direction) once fixed
        hold_alt = self._rel_alt if self._rel_alt is not None else ep.search_alt_min_m
        heading = self._banner_yaw(self._banner[0]) if self._banner_fresh() else self._yaw_enu
        frame_t = None                      # message time of the newest banner frame
        backed = 0.0
        at_lineup = False
        last_mode, bad_since = None, None
        lost_since = None                   # banner fully lost (before it is fixed)
        self._perception = None

        def fail(text):
            return False, text, None, None, None

        while True:
            t0 = time.time()
            if self._cancel.is_set() or not rclpy.ok():
                return fail(f"CANCELLED {leg}")
            if t0 > self._leg_deadline:
                return fail(f"LEG_TIMEOUT {leg} approach: " + (
                    "line-up point not reached" if place is not None else
                    f"banner not fixed ({len(fix.frames)} frames, height "
                    f"{fix.height:.2f} +- {fix.height_se:.2f} m)"))
            alt = self._rel_alt if self._rel_alt is not None else hold_alt
            yaw = self._yaw_enu if self._yaw_enu is not None else heading
            pos = here()

            # --- banner: each new frame read on its own ---
            bm = self._banner_fresh()
            b = bm[0] if bm is not None else None
            if b is not None and b["state"] == banner_msg.LOCK and bm[1] != frame_t:
                frame_t = bm[1]
                view = banner_view(b)
                if view is not None:
                    fix.add(pos, alt if math.isnan(b["cam_alt"]) else b["cam_alt"],
                            yaw if math.isnan(b["cam_yaw"]) else b["cam_yaw"], view)
                    if fix.ready():
                        loc = fix.locate()
                        if loc is not None:
                            if place is None:
                                q = point_out(*loc, ep.preentry_m)
                                self.get_logger().info(
                                    f"[path_planner] {leg}: banner FIXED at "
                                    f"({mav.offset_gps(origin[0], origin[1], loc[0][1], loc[0][0])[0]:.7f}, "
                                    f"{mav.offset_gps(origin[0], origin[1], loc[0][1], loc[0][0])[1]:.7f}): "
                                    f"{math.hypot(loc[0][0] - pos[0], loc[0][1] - pos[1]):.1f} m "
                                    f"away, corridor heading {math.degrees(loc[1]):.0f} deg ENU, "
                                    f"bottom edge {fix.height:.2f} m -> corridor altitude "
                                    f"{target_altitude(fix.height, ep):.2f} m; to the line-up "
                                    f"point {ep.preentry_m:.1f} m out "
                                    f"({math.hypot(q[0] - pos[0], q[1] - pos[1]):.1f} m from here)")
                            if not at_lineup:       # frozen once there
                                place = loc

            # --- at the line-up point: walls (if seen) correct the banner's line ---
            pm, self._perception = self._perception, None
            if track is not None:
                d_n, d_e = mav.ne_offset(entry[0], entry[1], self._gps[0], self._gps[1])
                a = track.axis_yaw
                track.set_pose(d_n * math.sin(a) + d_e * math.cos(a),
                               d_n * math.cos(a) - d_e * math.sin(a), t0)
                if pm is not None:
                    track.update(pm[0], pm[1],
                                 yaw if math.isnan(pm[0]["cam_yaw"]) else pm[0]["cam_yaw"])

            # --- obstacle sensor (depth) health ---
            if not self._sensor_ok():
                if bad_since is None:
                    bad_since = t0
                    self.get_logger().warn(f"[path_planner] {leg}: obstacle sensor not live — holding")
                    self._hold(alt, heading)
                if t0 - bad_since > self.cfg.sensor_fail_timeout_s:
                    return fail(f"SENSOR_TIMEOUT {leg} approach")
                self._sleep_rest(t0)
                continue
            bad_since = None

            if place is not None:
                mid, axis = place
                nominal = target_altitude(fix.height, ep)
                # --- at the line-up point, settled: hand over ---
                if at_lineup and t0 - lineup_t >= ep.lineup_settle_s:
                    self.get_logger().info(
                        f"[path_planner] {leg}: lined up on the banner's GPS line "
                        f"(offset {-track.centre:+.2f} m; walls: {track.accepted} frames "
                        f"agreed, {track.rejected} rejected, line corrected "
                        f"{track.correction:+.2f} m) -> corridor altitude {nominal:.2f} m")
                    self._banner_bottom = fix.height
                    return True, "OK", nominal, track, entry
                # --- to the line-up point, nose on the banner (down the corridor there) ---
                q = point_out(mid, axis, ep.preentry_m)
                if not at_lineup and math.hypot(q[0] - pos[0], q[1] - pos[1]) <= 0.3 \
                        and abs(alt - nominal) <= 2 * fp.alt_tol_m:
                    at_lineup, lineup_t = True, t0
                    # The corridor's GPS line: through the banner's middle,
                    # along its direction; the frame starts here.
                    entry = self._gps
                    d_e, d_n = mid[0] - pos[0], mid[1] - pos[1]
                    track = GpsCorridorTrack(
                        fp, self._fuse, axis, self._c_width,
                        centre0=d_n * math.cos(axis) - d_e * math.sin(axis),
                        pivot=d_e * math.cos(axis) + d_n * math.sin(axis),
                        heading_sd_deg=self._fuse.banner_heading_sd_deg,
                        max_heading_dev_deg=self._fuse.banner_max_heading_dev_deg,
                        lateral_sd_m=self._fuse.banner_lateral_sd_m,
                        max_correction_m=self._fuse.banner_max_correction_m)
                heading = axis if at_lineup else math.atan2(mid[1] - pos[1], mid[0] - pos[0])
                mode = "line up" if at_lineup else "to the line-up point"
                if mode != last_mode:
                    self.get_logger().info(f"[path_planner] {leg}: {mode} at {alt:.2f} m")
                    last_mode = mode
                lat, lon = mav.offset_gps(self._gps[0], self._gps[1], q[1] - pos[1], q[0] - pos[0])
                mav.goto(self, lat, lon, nominal, publish_count=1, yaw_enu=heading, log=False)
                self._sleep_rest(t0)
                continue

            # --- not fixed yet: toward the banner ---
            dist = float("nan")             # rough (3 x the tolerance) is enough here
            if not math.isnan(fix.height) and fix.height_se <= 3.0 * ep.height_tol_m:
                loc = fix.locate()
                if loc is not None:
                    dist = math.hypot(loc[0][0] - pos[0], loc[0][1] - pos[1])
            # Only a fresh detection (LOCK) steers. While coasting (HOLD) the
            # tracker repeats the last pixel position, which banner_node turns
            # into angles with the CURRENT heading: steering on it while
            # turning chases the banner out of view. Wait on the last good
            # heading instead.
            lock = b if b is not None and b["state"] == banner_msg.LOCK else None
            if b is not None and lock is None:
                lost_since = None
                if last_mode != "coast":
                    self.get_logger().info(
                        f"[path_planner] {leg}: banner coasting — holding at {alt:.2f} m")
                    last_mode = "coast"
                self._hold(hold_alt, heading)
                self._sleep_rest(t0)
                continue
            if lock is None:
                lost_since = lost_since or t0
                if t0 - lost_since > ep.research_after_s:
                    # Lost for good: one sweep around where it was last seen.
                    pts = sweep_points(0, ep)
                    self._leg_deadline += sweep_duration(pts, math.radians(ep.search_yaw_rate_deg_s))
                    self.get_logger().warn(
                        f"[path_planner] {leg}: banner lost {t0 - lost_since:.0f} s — sweeping "
                        f"+-{ep.search_half_angle_deg:.0f} deg around "
                        f"{math.degrees(heading):.0f} deg ENU at {hold_alt:.2f} m")
                    found, err = self._sweep(leg, heading, pts, self._gps[0], self._gps[1], hold_alt)
                    if err is not None:
                        return fail(err)
                    if found is None:
                        return fail(f"NO_BANNER {leg}: lost during the approach and not found "
                                    f"again")
                    heading, lost_since, last_mode = self._banner_yaw(found), None, None
                    self.get_logger().info(
                        f"[path_planner] {leg}: BANNER found again at "
                        f"{math.degrees(heading):.0f} deg ENU")
                    self._sleep_rest(t0)
                    continue
            else:
                lost_since = None
                heading = self._banner_yaw(lock)
            cmd = approach_command(lock is not None,
                                   lock["elev_bottom"] if lock else float("nan"),
                                   dist, alt, hold_alt, backed, ep, fp.lookahead_m, fp.alt_tol_m)
            if cmd.mode == "descend":
                hold_alt = cmd.alt
            if cmd.advance < 0.0:
                backed -= cmd.advance
            if cmd.mode != last_mode:
                self.get_logger().info(
                    f"[path_planner] {leg}: banner {cmd.mode} at {alt:.2f} m"
                    + ("" if b is None else
                       f" (bearing {math.degrees(b['bearing']):+.0f} deg, bottom edge "
                       f"{math.degrees(b['elev_bottom']):+.0f} deg up)")
                    + (f", ~{dist:.1f} m away" if not math.isnan(dist) else ""))
                last_mode = cmd.mode
            fe, fn = math.cos(heading), math.sin(heading)
            lat, lon = mav.offset_gps(self._gps[0], self._gps[1],
                                      fn * cmd.advance, fe * cmd.advance)
            mav.goto(self, lat, lon, cmd.alt, publish_count=1, yaw_enu=heading, log=False)
            self._sleep_rest(t0)

    def _progress(self, entry, axis):
        """(along, left) metres of the vehicle relative to the entry point."""
        d_n, d_e = mav.ne_offset(entry[0], entry[1], self._gps[0], self._gps[1])
        return d_n * axis[0] + d_e * axis[1], d_n * axis[1] - d_e * axis[0]

    def _follow_corridor(self, leg, entry, axis, axis_yaw, length, nominal=None,
                         track=None, obstacles=True, alt_gate=False, climb=True,
                         expected_len=None, mouth_at=None, wall_guard=False,
                         banner_bottom=None, pass_side_clearance=None):
        """Follow the corridor from `entry` along `axis` (see the header).

        Steering reference: the GPS centreline (GpsCorridorTrack), corrected
        by the walls whenever perception sees them — no walls in view is
        normal in an 11.5 ft corridor and does not stop the vehicle. Obstacle
        legs need perception messages (the depth camera) to keep coming.

        length: surveyed length (m), or None = unknown: a measured wall end,
        else expected_len, finishes the leg. nominal: corridor altitude
        (default corridor_altitude_m). track: a GpsCorridorTrack in THIS leg's
        frame (entry, axis_yaw) — banner legs: built at the line-up point from
        the banner fix; None = the line through entry along axis_yaw.
        obstacles False: no pass/climb (clean corridor). alt_gate: no forward
        progress until the vehicle has first come within 2 x alt_tol of
        nominal. climb False: the climb ceiling is nominal — obstacles are
        passed beside or block — and the wall guard is on. wall_guard: never
        step nearer a wall than the clearance (exit corridor).
        banner_bottom: the banner's bottom edge height (banner legs): under
        it, and banner_climb_after_m past the mouth, the altitude stays
        body_above_camera_m + climb_clearance_m below it.
        pass_side_clearance: pass beside only with this much air each side,
        else climb over if possible (None: side_clearance_m).
        expected_len: mouth -> wall end (m) to assume while no wall end is
        measured (length None only). mouth_at: progress of the mouth when
        known otherwise (banner legs: the line-up point is entry_preentry_m in
        front of it) — a measured mouth wins. A leg that ends on a measured
        wall end records its length in _measured_len."""
        fp = self._follow
        nominal = self._c_alt if nominal is None else nominal
        if not climb:
            # never above nominal, and the wall guard on
            fp = replace(fp, max_altitude_m=nominal, wall_guard=True)
        if wall_guard:
            fp = replace(fp, wall_guard=True)
        if pass_side_clearance is not None:
            fp = replace(fp, pass_side_clearance_m=pass_side_clearance)
        if banner_bottom is not None:
            ceiling = banner_bottom - self._body_above_cam - fp.climb_clearance_m
            fp = replace(fp, banner_ceiling_m=ceiling, banner_clear_m=self._banner_climb_after)
            self.get_logger().info(
                f"[path_planner] {leg}: banner bottom {banner_bottom:.2f} m -> no higher than "
                f"{ceiling:.2f} m until {self._banner_climb_after:.1f} m past the mouth"
                + ("" if pass_side_clearance is None else
                   f"; obstacles passed beside only with {pass_side_clearance:.2f} m each "
                   "side, else flown over"))
        track = track if track is not None else \
            GpsCorridorTrack(fp, self._fuse, axis_yaw, self._c_width)
        memory = ObstacleMemory(fp)
        alt_ready = not alt_gate
        entrance_seen = False
        exit_s = None
        walls_end_s = None          # progress where the walls were seen to end
        mouth_s = float("nan")      # progress of the mouth, while it is ahead
        bad_since = None
        backed_off = False
        blocked_since = None
        last_mode = None
        last_walls = None           # wall correction state last logged
        path = (NAN_F, 0.0)         # (path_dist, time): clear distance ahead of the nose
        self._last_s = None         # progress / mouth for the caller (exit failsafe)
        self._last_mouth = None

        def path_cap(step):
            """Exit corridor (wall_guard): never step to within
            min_stop_dist_m of anything straight ahead (corridor_msg
            path_dist — seen with or without a wall fit)."""
            d, t_p = path
            if not fp.wall_guard or math.isnan(d) or time.time() - t_p > 0.5:
                return step
            return min(step, max(0.0, d - fp.min_stop_dist_m))

        self._perception = None
        perc_t = time.time()        # newest perception frame (grace from the start)

        while True:
            t0 = time.time()
            if self._cancel.is_set() or not rclpy.ok():
                return False, f"CANCELLED {leg}"
            if t0 > self._leg_deadline:
                return False, f"LEG_TIMEOUT {leg}"
            s, lateral = self._progress(entry, axis)
            track.set_pose(s, lateral, t0)
            alt = self._rel_alt if self._rel_alt is not None else nominal
            yaw = self._yaw_enu if self._yaw_enu is not None else axis_yaw
            self._last_s = s

            # --- measurements: walls correct the GPS line, obstacles to memory ---
            pm = None
            if self._c_need_perception:
                pm, self._perception = self._perception, None
            if pm is not None:
                m = pm[0]
                perc_t = pm[1]
                path = (m.get("path_dist", NAN_F), pm[1])
                cam_yaw = yaw if math.isnan(m["cam_yaw"]) else m["cam_yaw"]
                cam_alt = alt if math.isnan(m["cam_alt"]) else m["cam_alt"]
                walls_ok = track.update(m, pm[1], cam_yaw)
                if walls_ok and not math.isnan(m["mouth_dist"]):
                    mouth_s = s + m["mouth_dist"]
                if walls_ok and not math.isnan(m["walls_end"]):
                    # Only a wall end near the known end can be the exit
                    # (surveyed length, or mouth + expected length: white
                    # walls give depth only on the logos, and a wall's depth
                    # stopping where a logo stops looks like its end).
                    # Neither known: the latest measured.
                    m_ref = mouth_s if not math.isnan(mouth_s) else mouth_at
                    end_at = length if length is not None else (
                        m_ref + expected_len if expected_len and m_ref is not None else None)
                    if end_at is None or s + m["walls_end"] >= end_at - self._c_exit_window_m:
                        walls_end_s = s + m["walls_end"]
                elif length is None and walls_end_s is not None and walls_ok and \
                        int(m["walls"]) == 2 and walls_end_s - s < self._c_end_visible_m:
                    self.get_logger().info(
                        f"[path_planner] {leg}: walls go on past the end measured at "
                        f"{walls_end_s:.1f} m — not the exit")
                    walls_end_s = None
                if obstacles and not math.isnan(m["obs_dist"]):
                    # Fitted or not (perception reports obstacles without a
                    # wall fit too), measured from the vehicle: placed with
                    # the GPS line's centre.
                    memory.update(m, track.centre, s, cam_alt, track.width)
                elif obstacles and int(m["walls"]) > 0:
                    memory.unseen(s, cam_alt, m["slice_lo"],
                                  climbing=alt > nominal + fp.alt_tol_m)
            c_axis = track.axis
            if memory.passed(s):
                self.get_logger().info(f"[path_planner] {leg}: obstacle passed at s={s:.1f} m")
            walls_state = track.wall_age(t0) <= 1.0
            if walls_state != last_walls:
                self.get_logger().info(
                    f"[path_planner] {leg}: " + (
                        f"walls in view — correcting the GPS line ({track.correction:+.2f} m "
                        f"so far)" if walls_state else
                        f"no walls in view — on the GPS line (corrected "
                        f"{track.correction:+.2f} m) at s={s:.1f} m"))
                last_walls = walls_state

            # --- entrance / exit ---
            if self._marker == "green" and not entrance_seen:
                entrance_seen = True
                self.get_logger().info(f"[path_planner] {leg}: ENTRANCE marker (green) at s={s:.1f} m")
            if self._marker == "blue" and exit_s is None and length is not None \
                    and s > 0.5 * length:
                exit_s = s
                self.get_logger().info(f"[path_planner] {leg}: EXIT marker (blue) at s={s:.1f} m")
            if exit_s is not None and s >= exit_s + self._c_exit_clear:
                return True, f"OK {leg} complete (exit marker at {exit_s:.1f} m)"
            mouth_ref = mouth_s if not math.isnan(mouth_s) else mouth_at
            self._last_mouth = mouth_ref
            if walls_end_s is not None and s >= walls_end_s + self._c_exit_clear:
                if mouth_ref is not None and length is None:
                    self._measured_len = walls_end_s - mouth_ref
                    self.get_logger().info(
                        f"[path_planner] {leg}: corridor {self._measured_len:.1f} m long")
                return True, f"OK {leg} complete (walls ended at {walls_end_s:.1f} m)"
            expected_s = (mouth_ref + expected_len) if expected_len and mouth_ref is not None \
                else None
            if walls_end_s is None and expected_s is not None and \
                    s >= expected_s + self._c_exit_clear and memory.active is None:
                return True, (f"OK {leg} complete (expected end at {expected_s:.1f} m, "
                              "no wall end measured)")
            if expected_s is not None and s >= expected_s + self._c_overrun:
                return True, f"OK {leg} complete (overrun {s:.1f} m)"
            if length is not None and s >= length + self._c_exit_clear:
                if self._c_need_perception:
                    self.get_logger().warn(f"[path_planner] {leg}: no exit marker — exit by distance")
                return True, f"OK {leg} complete (by distance {s:.1f} m)"

            # --- health: GPS fresh, obstacle sensor live, perception coming ---
            perc_ok = not (self._c_need_perception and obstacles) or \
                t0 - perc_t <= self._c_perc_stale
            healthy = track.valid(t0) and self._sensor_ok() and perc_ok
            if not healthy:
                if bad_since is None:
                    bad_since = t0
                    self.get_logger().warn(
                        f"[path_planner] {leg}: " + ("no GPS" if not track.valid(t0) else
                                                     "depth / perception lost") + " — holding")
                    self._hold(alt, c_axis)
                elif not backed_off and t0 - bad_since >= self.cfg.coverage_backoff_after_s:
                    back = self.cfg.coverage_backoff_m
                    lat, lon = mav.offset_gps(self._gps[0], self._gps[1],
                                              -axis[0] * back, -axis[1] * back)
                    self.get_logger().warn(f"[path_planner] {leg}: backing off {back:.1f} m")
                    mav.goto(self, lat, lon, alt, publish_count=5, yaw_enu=c_axis)
                    backed_off = True
                if t0 - bad_since > self.cfg.sensor_fail_timeout_s:
                    return False, f"SENSOR_TIMEOUT {leg} at s={s:.1f} m"
                # The leg deadline is NOT extended here: lose/recover cycles
                # must not keep the vehicle in the corridor indefinitely.
                self._sleep_rest(t0)
                continue
            if bad_since is not None:
                self.get_logger().info(f"[path_planner] {leg}: sensors back — resuming")
                bad_since, backed_off = None, False

            # --- plan and command ---
            # Not into the mouth before lined up on the current best line:
            # the measured mouth, else the known one (banner legs).
            cmd = plan(track, memory.confirmed(), s, alt, nominal, fp,
                       mouth_s=mouth_s if not math.isnan(mouth_s) else
                       (mouth_at if mouth_at is not None else NAN_F))
            cmd.advance = path_cap(cmd.advance)
            if not alt_ready:
                if abs(alt - nominal) <= 2 * fp.alt_tol_m:
                    alt_ready = True
                    self.get_logger().info(
                        f"[path_planner] {leg}: at corridor altitude {alt:.2f} m — going in")
                else:
                    cmd.advance = 0.0           # line up and change height first
            if cmd.mode != last_mode:
                o = memory.active
                self.get_logger().info(
                    f"[path_planner] {leg}: {cmd.mode} at s={s:.1f} m"
                    + ("" if o is None else
                       f" (obstacle at {o.s_front:.1f} m, [{o.right:+.2f},{o.left:+.2f}] m"
                       f", top {o.top:.2f} m)"))
                last_mode = cmd.mode
            if cmd.mode == "blocked":
                blocked_since = blocked_since or t0
                if t0 - blocked_since > self.cfg.sensor_fail_timeout_s:
                    return False, (f"BLOCKED {leg} at s={s:.1f} m: no side gap and "
                                   + ("too tall to climb" if climb else "climbing is off"))
            else:
                blocked_since = None

            heading = c_axis                            # nose: the corrected corridor
            self.get_logger().info(
                f"[path_planner] {leg}: follow s={s:.1f} m, offset {-track.centre:+.2f} m "
                f"(GPS line corrected {track.correction:+.2f} m; walls {track.walls}, "
                f"{min(track.wall_age(t0), 99.0):.1f} s ago; {track.accepted} accepted / "
                f"{track.rejected} rejected), step {cmd.lateral:+.2f} m across / "
                f"{cmd.advance:.2f} m on, axis {math.degrees(c_axis):.0f} deg",
                throttle_duration_sec=0.5)
            # The step in the frame s is measured in (the leg's axis): along
            # the corrected axis instead, every sideways step also moved s —
            # at an obstacle's stop point that pushed s past it, the plan
            # backed off, and the two fought for ever (crossing to the other
            # side behind a passed obstacle, slightly turned line).
            fe, fn = axis[1], axis[0]
            d_e = fe * cmd.advance - fn * cmd.lateral
            d_n = fn * cmd.advance + fe * cmd.lateral
            lat, lon = mav.offset_gps(self._gps[0], self._gps[1], d_n, d_e)
            mav.goto(self, lat, lon, cmd.alt, publish_count=1, yaw_enu=heading, log=False)
            self._sleep_rest(t0)

    def _sleep_rest(self, t0):
        time.sleep(max(0.0, self._c_period - (time.time() - t0)))

    # ===================================================================
    # Service handlers
    # ===================================================================

    def _respond(self, response, ok_msg):
        ok, msg = ok_msg
        response.success, response.message = ok, msg
        # Separate call sites: rclpy rejects one call site logging at two
        # different severities.
        if ok:
            self.get_logger().info(f"[path_planner] -> {msg}")
        else:
            self.get_logger().warn(f"[path_planner] -> {msg}")
        return response

    def _handle_corridor_1(self, request, response):
        return self._respond(response, self._run(
            "corridor_1", self.cfg.corridor_timeout_s,
            lambda: self._corridor("corridor_1", self._corridor1_local)))

    def _handle_corridor_2(self, request, response):
        return self._respond(response, self._run(
            "corridor_2", self.cfg.corridor_timeout_s,
            lambda: self._corridor("corridor_2", self._corridor2_local)))

    def _handle_corridor_here(self, request, response):
        return self._respond(response, self._run(
            "corridor_test", self.cfg.corridor_timeout_s,
            lambda: self._corridor_here("corridor_test")))

    def _handle_entry_corridor(self, request, response):
        # Each phase (search, approach, corridor) sets its own deadline.
        return self._respond(response, self._run(
            "entry_corridor", self.cfg.corridor_timeout_s,
            lambda: self._banner_corridor("entry_corridor", "green",
                                          obstacles=False, climb=True)))

    def _handle_exit_corridor(self, request, response):
        return self._respond(response, self._run(
            "exit_corridor", self.cfg.corridor_timeout_s,
            lambda: self._banner_corridor("exit_corridor", "blue",
                                          obstacles=True, climb=True)))

    def _handle_run_lawnmower(self, request, response):
        return self._respond(response, self._run(
            "lawnmower", self.cfg.lawnmower_timeout_s, self._lawnmower))

    def _handle_stop(self, request, response):
        self._cancel.set()
        # Hold here before replying so the active leg's last target is
        # replaced; the caller's next goto then overrides this hold.
        self._hold(self._rel_alt if self._rel_alt is not None else self.cfg.cruise_altitude_m)
        response.success = True
        response.message = "OK stopped, holding position"
        self.get_logger().info("[path_planner] STOP — holding position")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[path_planner] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
