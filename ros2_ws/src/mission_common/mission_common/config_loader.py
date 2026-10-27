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

    # ── Downward camera (RPi Camera Module 3) ────────────────────
    # 66 deg = standard Camera Module 3; use 102 for the Wide variant.
    cfg.down_cam_hfov_deg     = _declare(node, "down_cam_hfov_deg", 66.0)
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
    cfg.require_rc_switch     = _declare(node, "require_rc_switch", True)
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
    cfg.qr_step_m             = _declare(node, "qr_step_m", 0.5)          # search step, none in view
    cfg.qr_max_steps          = _declare(node, "qr_max_steps", 4)         # search steps before giving up
    cfg.qr_center_tol_m       = _declare(node, "qr_center_tol_m", 0.3)    # close enough = no move
    cfg.qr_center_passes      = _declare(node, "qr_center_passes", 2)     # locate -> move, repeated
    cfg.qr_max_radius_m       = _declare(node, "qr_max_radius_m", 4.0)    # never further from takeoff
    cfg.qr_max_jump_m         = _declare(node, "qr_max_jump_m", 3.0)      # longest single correction
    cfg.qr_move_speed_ms      = _declare(node, "qr_move_speed_ms", 0.5)
    cfg.qr_move_tolerance_m   = _declare(node, "qr_move_tolerance_m", 0.7)  # GPS noise ~1 m
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
    cfg.corridor_start_alt_m  = _declare(node, "corridor_start_alt_m", 3.0)
    # Straight up to this once past the entry corridor's walls (the later
    # phases set their own altitudes).
    cfg.post_entry_alt_m      = _declare(node, "post_entry_alt_m", 10.0)

    # ── Arena search, phase 3 (mission_manager/arena_search.py) ──
    # The arena's 4 GPS corners in order around it (competition day). All
    # zero = not set: the arena_width_m x arena_height_m rectangle at
    # origin_lat/lon is used instead.
    cfg.arena_corners_lat     = _declare(node, "arena_corners_lat", [0.0, 0.0, 0.0, 0.0])
    cfg.arena_corners_lon     = _declare(node, "arena_corners_lon", [0.0, 0.0, 0.0, 0.0])
    cfg.arena_alt_m           = _declare(node, "arena_alt_m", 10.0)
    cfg.arena_sweep_speed_ms  = _declare(node, "arena_sweep_speed_ms", 0.5)
    cfg.arena_red_margin_m    = _declare(node, "arena_red_margin_m", 2.0)   # from the red mat
    cfg.arena_edge_margin_m   = _declare(node, "arena_edge_margin_m", 1.0)  # strips inside the edge
    cfg.arena_qr_size_m       = _declare(node, "arena_qr_size_m", 1.0)      # sets the strip spacing
    cfg.arena_match_passes    = _declare(node, "arena_match_passes", 2)     # fly over + rescan
    cfg.arena_match_tol_m     = _declare(node, "arena_match_tol_m", 1.0)    # match this close = over it
    cfg.arena_rescan_timeout_s = _declare(node, "arena_rescan_timeout_s", 15.0)

    # ── Payload drop ─────────────────────────────────────────────
    cfg.drop_arrival_tolerance_m = _declare(node, "drop_arrival_tolerance_m", 1.0)
    cfg.drop_arrival_timeout_s   = _declare(node, "drop_arrival_timeout_s", 30.0)
    cfg.drop_settle_s            = _declare(node, "drop_settle_s", 3.0)

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
    cfg.release_channel     = _declare(node, "release_channel", 10)
    cfg.release_pwm_open    = _declare(node, "release_pwm_open", 1100)
    cfg.release_pause_s     = _declare(node, "release_pause_s", 1.0)

    return cfg
