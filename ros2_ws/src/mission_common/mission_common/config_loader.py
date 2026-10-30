# config_loader.py — Central parameter declaration for all mission nodes
#
# Instead of scattering constants across source files, every node calls
#   cfg = load_mission_config(node)
# and then reads cfg.arena_width_m, cfg.cruise_altitude_m, etc.
#
# Default values are compiled here so nodes work even when launched
# without a YAML parameter file (e.g. in unit tests or demos).
#
# Usage:
#   from mission_common.config_loader import load_mission_config
#   cfg = load_mission_config(self)   # inside a Node.__init__
#   print(cfg.cruise_altitude_m)


class MissionConfig:
    """Simple attribute-access namespace for mission parameters."""

    def __repr__(self):
        attrs = ", ".join(f"{k}={v!r}" for k, v in self.__dict__.items())
        return f"MissionConfig({attrs})"


def _declare(node, name: str, default):
    """Declare a ROS2 parameter (skip if already declared) and return its value."""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def load_mission_config(node) -> MissionConfig:
    """
    Declare all mission parameters on *node* and return a populated
    MissionConfig object.

    Parameters come from the YAML loaded via the launch file.  If a
    parameter is not in the YAML, the compiled default is used.
    """
    cfg = MissionConfig()

    # ── Arena & Course Geometry ──────────────────────────────────
    cfg.arena_width_m         = _declare(node, "arena_width_m", 40.0)
    cfg.arena_height_m        = _declare(node, "arena_height_m", 30.0)

    # ── Altitudes (metres above home) ────────────────────────────
    cfg.drop_altitude_m       = _declare(node, "drop_altitude_m", 5.0)
    cfg.cruise_altitude_m     = _declare(node, "cruise_altitude_m", 5.0)

    # ── Speeds (sent to ArduPilot with DO_CHANGE_SPEED per leg) ──
    # transit: open-air hops (to a corridor's entry point, homing). Inside a
    # corridor path_planner uses its own corridor_follow_speed_ms.
    cfg.transit_speed_ms      = _declare(node, "transit_speed_ms", 2.0)
    cfg.arena_speed_ms        = _declare(node, "arena_speed_ms", 1.5)

    # ── GPS Origin of the local arena frame ──────────────────────
    cfg.origin_lat            = _declare(node, "origin_lat", 12.9716)
    cfg.origin_lon            = _declare(node, "origin_lon", 77.5946)

    # ── Downward camera (RPi Camera Module 1.3) ──────────────────
    # 53.5 deg = Camera Module 1.3; 66 = Camera Module 3, 102 its Wide variant.
    cfg.down_cam_hfov_deg     = _declare(node, "down_cam_hfov_deg", 53.5)
    # Fraction of the camera footprint shared by adjacent strips.
    cfg.strip_overlap         = _declare(node, "strip_overlap", 0.20)

    # ── Battery (4S, under load). ArduPilot BATT_FS_* is the primary
    #    failsafe; the companion only stops commanding the mission. ──
    cfg.battery_critical_v    = _declare(node, "battery_critical_v", 13.6)

    # ── Timeouts & Tolerances ────────────────────────────────────
    cfg.service_timeout_s     = _declare(node, "service_timeout_s", 10.0)
    cfg.altitude_timeout_s    = _declare(node, "altitude_timeout_s", 30.0)
    cfg.altitude_tolerance_m  = _declare(node, "altitude_tolerance_m", 0.5)
    cfg.arrival_tolerance_m   = _declare(node, "arrival_tolerance_m", 1.5)
    cfg.arrival_timeout_s     = _declare(node, "arrival_timeout_s", 60.0)
    cfg.goto_publish_count    = _declare(node, "goto_publish_count", 20)
    cfg.arm_timeout_s         = _declare(node, "arm_timeout_s", 90.0)
    cfg.land_timeout_s        = _declare(node, "land_timeout_s", 120.0)
    cfg.stream_rate_hz        = _declare(node, "stream_rate_hz", 10.0)

    # ── Mission start / reference QR ─────────────────────────────
    cfg.tick_rate_hz          = _declare(node, "tick_rate_hz", 2.0)
    # Mission waits for the pilot's auto switch before doing anything.
    cfg.require_rc_switch     = _declare(node, "require_rc_switch", False)
    # PHYSICAL RC channel number, 1-based (CH7 -> RCIn.channels[6]).
    cfg.rc_auto_channel       = _declare(node, "rc_auto_channel", 7)
    cfg.rc_auto_pwm_thresh    = _declare(node, "rc_auto_pwm_thresh", 1700)
    cfg.qr_scan_attempts      = _declare(node, "qr_scan_attempts", 3)
    cfg.qr_scan_timeout_s     = _declare(node, "qr_scan_timeout_s", 60.0)
    # Only for SITL (no downward camera): fly with qr_fallback_id when the
    # reference scan fails. On hardware a failed scan means no target: the
    # mission carries on without the arena search.
    cfg.allow_qr_fallback     = _declare(node, "allow_qr_fallback", False)
    cfg.qr_fallback_id        = _declare(node, "qr_fallback_id", "QR_TARGET_DEMO_ID")
    # Reference QR in the air (mission_manager/reference_qr.py), at
    # cruise_altitude_m over the launch point; the QR is at most ~1 m ahead.
    # It looks from over the launch point first; none in view: fly
    # qr_initial_advance_m ahead, then hover at these along-nose offsets
    # (m, + = forward) from there, in order — never sideways
    cfg.qr_search_offsets_m   = _declare(node, "qr_search_offsets_m", [0.3, -0.3, 0.6, -0.6])
    cfg.qr_center_tol_m       = _declare(node, "qr_center_tol_m", 0.3)    # close enough = no move
    cfg.qr_center_passes      = _declare(node, "qr_center_passes", 4)     # locate -> move, repeated
    cfg.qr_initial_advance_m  = _declare(node, "qr_initial_advance_m", 1.0)  # the QR lies this far ahead
    # Only QRs within this of that expected spot are used (GPS drift; not clutter)
    cfg.qr_expected_radius_m  = _declare(node, "qr_expected_radius_m", 2.5)
    cfg.qr_max_radius_m       = _declare(node, "qr_max_radius_m", 4.0)    # never further from takeoff
    cfg.qr_max_jump_m         = _declare(node, "qr_max_jump_m", 3.0)      # longest single correction
    cfg.qr_move_speed_ms      = _declare(node, "qr_move_speed_ms", 0.5)
    cfg.qr_move_tolerance_m   = _declare(node, "qr_move_tolerance_m", 0.7)  # GPS noise ~1 m
    # after each move, before the camera looks: speed below qr_settle_speed_ms
    # for qr_settle_hold_s, waiting at most qr_settle_timeout_s
    cfg.qr_settle_speed_ms    = _declare(node, "qr_settle_speed_ms", 0.1)
    cfg.qr_settle_hold_s      = _declare(node, "qr_settle_hold_s", 1.0)
    cfg.qr_settle_timeout_s   = _declare(node, "qr_settle_timeout_s", 8.0)
    cfg.qr_locate_timeout_s   = _declare(node, "qr_locate_timeout_s", 8.0)  # > scanner locate_timeout_s

    # ── Gradual takeoff (as testing/waypoint_mission.py) ─────────
    # Position estimate watched this long before arming, and must not drift
    # more than takeoff_max_ground_drift_m then or in the armed wait; lift
    # off to takeoff_liftoff_alt_m, then climb at takeoff_climb_rate_ms to
    # cruise_altitude_m holding the heading the drone was placed with.
    cfg.takeoff_steady_time_s      = _declare(node, "takeoff_steady_time_s", 3.0)
    cfg.takeoff_max_ground_drift_m = _declare(node, "takeoff_max_ground_drift_m", 0.8)
    cfg.takeoff_liftoff_alt_m      = _declare(node, "takeoff_liftoff_alt_m", 0.5)
    cfg.takeoff_climb_rate_ms      = _declare(node, "takeoff_climb_rate_ms", 0.5)

    # ── Entry corridor start ─────────────────────────────────────
    # After the reference QR, descend to this before the banner search (the
    # search's first full turn is at the altitude it starts at).
    cfg.corridor_start_alt_m  = _declare(node, "corridor_start_alt_m", 1.2192)
    # Surveyed entry corridor: flown OVER at cruise_altitude_m, mouth centre
    # -> midpoint of its two exit corners, at this speed / arrival tolerance.
    cfg.entry_overfly_speed_ms    = _declare(node, "entry_overfly_speed_ms", 0.7)
    cfg.entry_overfly_tolerance_m = _declare(node, "entry_overfly_tolerance_m", 0.7)
    # Straight up to this at the end of the entry corridor (the arena sweep
    # starts from there; the later phases set their own altitudes).
    cfg.post_entry_alt_m      = _declare(node, "post_entry_alt_m", 9.25)

    # ── Arena search, phase 3 (mission_manager/arena_search.py) ──
    # The arena's 4 GPS corners in order around it (competition day). All
    # zero = not set: the arena_width_m x arena_height_m rectangle at
    # origin_lat/lon is used instead.
    cfg.arena_corners_lat     = _declare(node, "arena_corners_lat", [0.0, 0.0, 0.0, 0.0])
    cfg.arena_corners_lon     = _declare(node, "arena_corners_lon", [0.0, 0.0, 0.0, 0.0])
    cfg.arena_alt_m           = _declare(node, "arena_alt_m", 9.25)
    cfg.arena_sweep_speed_ms  = _declare(node, "arena_sweep_speed_ms", 0.5)
    cfg.arena_red_margin_m    = _declare(node, "arena_red_margin_m", 2.0)   # from the red mat
    cfg.arena_edge_margin_m   = _declare(node, "arena_edge_margin_m", 1.0)  # strips inside the edge
    cfg.arena_qr_size_m       = _declare(node, "arena_qr_size_m", 1.0)      # sets the strip spacing
    cfg.arena_match_passes    = _declare(node, "arena_match_passes", 2)     # fly over + rescan
    cfg.arena_match_tol_m     = _declare(node, "arena_match_tol_m", 1.0)    # match this close = over it
    cfg.arena_rescan_timeout_s = _declare(node, "arena_rescan_timeout_s", 15.0)
    # Competition day: corners all zero -> the full mission refuses to take
    # off (the placeholder rectangle at origin_lat/lon is for tests only).
    cfg.require_arena_corners = _declare(node, "require_arena_corners", True)

    # ── Mission clock (mission_node) ─────────────────────────────
    # Runs from lift-off. The arena sweep ends early so that, after it,
    # mission_drop_reserve_s (phase 4) + mission_exit_reserve_s (phase 5,
    # home, landing) are still left of mission_time_limit_s.
    cfg.mission_time_limit_s   = _declare(node, "mission_time_limit_s", 900.0)
    cfg.mission_drop_reserve_s = _declare(node, "mission_drop_reserve_s", 90.0)
    cfg.mission_exit_reserve_s = _declare(node, "mission_exit_reserve_s", 300.0)

    # ── Exit corridor, phase 5 (path_planner/exit_corridor.py) ───
    # Corridor 1's far end mirrored across the line through the takeoff
    # point = the exit corridor's arena-side entrance; the blue banner is
    # searched for from exit_search_standoff_m further out in the arena.
    # Not found -> the point shifted exit_search_shift_m each way along the
    # corridor entrances (0 = no shifted searches); still not -> the exit
    # corridor failsafe (below).
    cfg.exit_search_standoff_m  = _declare(node, "exit_search_standoff_m", 6.0)
    cfg.exit_search_shift_m     = _declare(node, "exit_search_shift_m", 3.0)
    cfg.exit_corridor_timeout_s = _declare(node, "exit_corridor_timeout_s", 600.0)
    # Exit corridor failsafe (ONLY the exit corridor): straight up to this,
    # home, land. Must clear the corridor walls (8 ft = 2.44 m) with margin.
    cfg.exit_failsafe_alt_m = _declare(node, "exit_failsafe_alt_m", 5.0)
    if cfg.exit_failsafe_alt_m < 3.5:
        raise ValueError("exit_failsafe_alt_m must clear the 2.44 m corridor walls "
                         "with margin (>= 3.5 m)")
    # Hard ceiling (mavros_utils.MAX_ALT_M, 9.5 m): refuse to start rather
    # than have goto() silently cap a phase's altitude in flight.
    from mission_common.mavros_utils import MAX_ALT_M
    for name in ("cruise_altitude_m", "drop_altitude_m", "post_entry_alt_m",
                 "arena_alt_m", "exit_failsafe_alt_m", "corridor_start_alt_m"):
        if float(getattr(cfg, name)) > MAX_ALT_M:
            raise ValueError(f"{name} {getattr(cfg, name)} m is above the "
                             f"{MAX_ALT_M} m altitude cap")

    # ── Site survey (mission_common/survey.py; yaml /** block) ───
    # [lat, lon] points measured on site; [0, 0] = not measured. Corridor
    # corners are named from the drone's view while flying THROUGH that
    # corridor (entry: home -> arena, exit: arena -> home); mouth = the
    # banner side. With survey_home set and the drone taking off within
    # survey_max_home_dist_m of it: phase 1 expects the QR at survey_qr, and
    # both corridors are flown on their surveyed centrelines (banner seen:
    # it corrects the line; not seen: in anyway).
    cfg.survey_home            = _declare(node, "survey_home", [0.0, 0.0])
    cfg.survey_qr              = _declare(node, "survey_qr", [0.0, 0.0])
    for which in ("entry", "exit"):
        for k in ("mouth_left", "mouth_right", "end_left", "end_right"):
            setattr(cfg, f"survey_{which}_{k}",
                    _declare(node, f"survey_{which}_{k}", [0.0, 0.0]))
    cfg.survey_corridor_width_m = _declare(node, "survey_corridor_width_m", 3.5052)
    cfg.survey_max_home_dist_m  = _declare(node, "survey_max_home_dist_m", 100.0)
    cfg.survey_anchor_to_home   = _declare(node, "survey_anchor_to_home", True)
    cfg.survey_max_home_shift_m = _declare(node, "survey_max_home_shift_m", 3.0)

    # ── Payload drop ─────────────────────────────────────────────
    cfg.drop_arrival_tolerance_m = _declare(node, "drop_arrival_tolerance_m", 1.0)
    cfg.drop_arrival_timeout_s   = _declare(node, "drop_arrival_timeout_s", 30.0)
    cfg.drop_settle_s            = _declare(node, "drop_settle_s", 3.0)
    # Camera centring at drop_altitude_m before the release
    cfg.drop_center_tol_m        = _declare(node, "drop_center_tol_m", 0.3)
    cfg.drop_center_passes       = _declare(node, "drop_center_passes", 3)
    cfg.drop_center_max_offset_m = _declare(node, "drop_center_max_offset_m", 2.0)
    # Wait for /payload/release (winch lower + jerk + restart, held in place)
    cfg.drop_release_timeout_s   = _declare(node, "drop_release_timeout_s", 60.0)

    # ── Obstacle sensor health (obstacle_avoidance coverage_guard) ─
    # sensor_live False -> hold; still False after sensor_fail_timeout_s -> LAND.
    # In a corridor, no valid corridor fit -> hold; after
    # coverage_backoff_after_s back off coverage_backoff_m along the corridor;
    # still lost after sensor_fail_timeout_s -> LAND.
    cfg.require_avoidance_sensor  = _declare(node, "require_avoidance_sensor", True)
    cfg.sensor_preflight_timeout_s = _declare(node, "sensor_preflight_timeout_s", 30.0)
    cfg.sensor_fail_timeout_s     = _declare(node, "sensor_fail_timeout_s", 10.0)
    cfg.coverage_backoff_after_s  = _declare(node, "coverage_backoff_after_s", 3.0)
    cfg.coverage_backoff_m        = _declare(node, "coverage_backoff_m", 1.0)

    # ── Path Planner Specific ────────────────────────────────────
    cfg.corridor_timeout_s          = _declare(node, "corridor_timeout_s", 120.0)
    # mission_node's wait for the whole entry corridor leg (search + approach
    # + corridor); the planner enforces each part's own deadline.
    cfg.entry_corridor_timeout_s    = _declare(node, "entry_corridor_timeout_s", 600.0)
    cfg.lawnmower_timeout_s        = _declare(node, "lawnmower_timeout_s", 600.0)
    cfg.mavros_arrival_tolerance_m  = _declare(node, "mavros_arrival_tolerance_m", 2.0)
    cfg.red_zone_buffer_m           = _declare(node, "red_zone_buffer_m", 0.5)

    # ── Payload Specific ─────────────────────────────────────────
    # Winch (spool): spinning = payload held, stopped = it unwinds and drops
    # (motor driver on two FC relays, 0 = RELAY1; IN = in ON, OUT = out ON)
    cfg.winch_relay_in      = _declare(node, "winch_relay_in", 0)
    cfg.winch_relay_out     = _declare(node, "winch_relay_out", 1)
    cfg.winch_unwind_s      = _declare(node, "winch_unwind_s", 11.0)
    cfg.winch_jerks         = _declare(node, "winch_jerks", 1)
    cfg.winch_jerk_s        = _declare(node, "winch_jerk_s", 0.5)

    return cfg
