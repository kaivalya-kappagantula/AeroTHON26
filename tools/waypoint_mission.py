#!/usr/bin/env python3
"""
waypoint_mission.py — Autonomous takeoff -> N waypoints -> land (ArduCopter, GUIDED).

Waypoints are offsets RELATIVE TO THE TAKEOFF POINT, in metres:
    --frame body (default): forward, right, altitude  (rotated by heading at takeoff)
    --frame ned           : north,   east,  altitude
Altitude is height above the takeoff point (positive up).

Works with GPS or optical-flow positioning, since it only uses LOCAL_POSITION_NED.

Safety behaviour:
  * If the mode is switched away from GUIDED (safety pilot takes over), the script
    STOPS sending commands immediately and exits. It never fights the pilot.
  * Ctrl+C, low battery, or a timeout -> RTL (fly home and land). If RTL is refused,
    or the position estimate itself was lost, it lands in place instead.
    Pressing Ctrl+C again during RTL lands in place.
    Below 1 m (e.g. a props-off bench test) an abort always lands, never RTLs.
  * Waypoints farther than --max-radius from takeoff are rejected before arming.
  * Won't arm / take off if the position estimate drifts more than
    --max-ground-drift while the drone is sitting on the ground.
  * Gradual takeoff: motors spool for --spool-wait s, lift off to --liftoff-alt,
    then climb slowly to --takeoff-alt at --climb-rate.
  * Heading is held at the takeoff heading for the whole flight.
  * The setpoint slides toward each waypoint at --speed (no setpoint jumps),
    and a waypoint only counts as reached once the drone has slowed down.

Examples:
  # SITL first (always):
  python3 waypoint_mission.py --conn tcp:127.0.0.1:5762 --min-voltage 0

  # Real drone, Pi <-> Pixhawk TELEM2 (defaults: /dev/ttyAMA2 @ 921600):
  python3 waypoint_mission.py

  # Custom waypoints (forward, right, alt):
  python3 waypoint_mission.py --wp 2,0,1 --wp 2,2,1 --wp 0,2,1 --wp 0,0,1
"""

import argparse
import csv
import math
import sys
import time

from pymavlink import mavutil

# ---------------------------------------------------------------- config ---

DEFAULT_WAYPOINTS = [  # 2 m square at 1 m, forward/right of takeoff
    (3.5, 0.0, 1.0),
    (3.5, 3.5, 1.0),
    (0.0, 3.5, 1.0),
    (0.0, 0.0, 1.0),
]

# Motor output ceiling, written before every arm. MOT_SPIN_MAX caps output in
# every mode; was 0.95. Same value as mission_common.mavros_utils.THROTTLE_CAP.
THROTTLE_CAP_PARAM = "MOT_SPIN_MAX"
THROTTLE_CAP = 0.80

STREAMS_HZ = {  # requested explicitly (no MAVProxy to do it for us)
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
}

# Position + velocity feed-forward + fixed yaw: ignore accel and yaw-rate.
# (If yaw were ignored, ArduCopter would turn the nose toward each waypoint.)
POS_VEL_YAW_MASK = 0b0000_1001_1100_0000

SETTLED_SPEED = 0.3   # m/s — a waypoint only counts as reached below this speed
RTL_MIN_HEIGHT = 1.0  # m — abort below this height lands in place instead of RTL


class PilotOverride(Exception):
    """Mode left GUIDED — a human has control. Stop commanding."""


class MissionAbort(Exception):
    """Something went wrong — land."""


# ----------------------------------------------------------------- state ---

class Vehicle:
    def __init__(self, conn, baud, log_path):
        print(f"[link] connecting to {conn} ...")
        self.m = mavutil.mavlink_connection(conn, baud=baud, source_system=255)
        self.m.wait_heartbeat(timeout=30)
        print(f"[link] heartbeat from sys {self.m.target_system} comp {self.m.target_component}")

        self.mode = None
        self.armed = False
        self.params = {}
        self.pos = None          # (n, e, d) local NED, metres
        self.vel = (0.0, 0.0, 0.0)  # (vn, ve, vd) m/s
        self.yaw = None          # radians
        self.voltage = None      # volts
        self.lat = self.lon = None   # degrees (EKF global position)
        self.rel_alt = None          # m above home (EKF)
        self.gps_fix = self.gps_sats = self.gps_hdop = None
        self.last_pos_time = 0.0
        self.statustext = []

        self._log_f = open(log_path, "w", newline="")
        self._log = csv.writer(self._log_f)
        self._log.writerow(["t", "mode", "armed", "n", "e", "d", "yaw_deg", "volt",
                            "lat", "lon", "sats", "hdop", "event"])
        self.t0 = time.time()

        self._request_streams()

    def _request_streams(self):
        for msg_id, hz in STREAMS_HZ.items():
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def pump(self, duration=0.0):
        """Read all pending messages (for at least `duration` seconds)."""
        end = time.time() + duration
        while True:
            msg = self.m.recv_match(blocking=duration > 0, timeout=0.05)
            if msg is not None:
                self._handle(msg)
            elif time.time() >= end:
                break

    def _handle(self, msg):
        t = msg.get_type()
        if t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system \
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            self.mode = mavutil.mode_string_v10(msg)
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == "LOCAL_POSITION_NED":
            self.pos = (msg.x, msg.y, msg.z)
            self.vel = (msg.vx, msg.vy, msg.vz)
            self.last_pos_time = time.time()
        elif t == "ATTITUDE":
            self.yaw = msg.yaw
        elif t == "SYS_STATUS":
            if msg.voltage_battery not in (0, 65535):
                self.voltage = msg.voltage_battery / 1000.0
        elif t == "GLOBAL_POSITION_INT":
            if msg.lat != 0 or msg.lon != 0:
                self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
                self.rel_alt = msg.relative_alt / 1000.0
        elif t == "GPS_RAW_INT":
            self.gps_fix = msg.fix_type
            self.gps_sats = msg.satellites_visible
            self.gps_hdop = msg.eph / 100.0 if msg.eph != 65535 else None
        elif t == "PARAM_VALUE":
            self.params[msg.param_id.strip("\x00")] = msg.param_value
        elif t == "STATUSTEXT":
            print(f"[fc] {msg.text}")

    def log(self, event=""):
        n, e, d = self.pos if self.pos else (None, None, None)
        yaw = math.degrees(self.yaw) if self.yaw is not None else None
        self._log.writerow([round(time.time() - self.t0, 2), self.mode, self.armed,
                            n, e, d, yaw, self.voltage,
                            self.lat, self.lon, self.gps_sats, self.gps_hdop, event])
        self._log_f.flush()

    # ------------------------------------------------------------ commands --

    def set_mode(self, name, timeout=5):
        mode_id = self.m.mode_mapping()[name]
        end = time.time() + timeout
        while time.time() < end:
            self.m.set_mode(mode_id)
            self.pump(0.5)
            if self.mode == name:
                self.log(f"mode={name}")
                return
        raise MissionAbort(f"could not enter {name} (current: {self.mode})")

    def apply_throttle_cap(self, timeout=5):
        """Write THROTTLE_CAP to MOT_SPIN_MAX and wait for the FC to echo it."""
        self.params.pop(THROTTLE_CAP_PARAM, None)
        end = time.time() + timeout
        while time.time() < end:
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                      THROTTLE_CAP_PARAM.encode(), THROTTLE_CAP,
                                      mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            self.pump(1.0)
            got = self.params.get(THROTTLE_CAP_PARAM)
            if got is not None and abs(got - THROTTLE_CAP) < 1e-3:
                self.log(f"throttle cap {THROTTLE_CAP_PARAM}={THROTTLE_CAP:.2f}")
                return
        raise MissionAbort(f"throttle cap not confirmed ({THROTTLE_CAP_PARAM} = "
                           f"{self.params.get(THROTTLE_CAP_PARAM)}) — refusing to fly")

    def arm(self, timeout=10):
        end = time.time() + timeout
        while time.time() < end:
            self.m.arducopter_arm()
            self.pump(1.0)
            if self.armed:
                self.log("armed")
                return
        raise MissionAbort("arming failed — check [fc] PreArm messages above")

    def takeoff_cmd(self, alt):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

    def send_setpoint(self, pos, vel, yaw):
        """Position target + velocity feed-forward + fixed heading."""
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, POS_VEL_YAW_MASK,
            pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], 0, 0, 0, yaw, 0)

    def speed(self):
        return math.hypot(*self.vel)

    def gps_text(self):
        """One-line GPS summary for printing."""
        fix_names = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS",
                     5: "RTK float", 6: "RTK fixed"}
        fix = fix_names.get(self.gps_fix, str(self.gps_fix))
        hdop = f"{self.gps_hdop:.2f}" if self.gps_hdop is not None else "?"
        if self.lat is None:
            return f"lat/lon unknown  (fix={fix}, sats={self.gps_sats}, hdop={hdop})"
        return (f"lat={self.lat:.7f} lon={self.lon:.7f} alt={self.rel_alt:.2f} m  "
                f"(fix={fix}, sats={self.gps_sats}, hdop={hdop})")


# --------------------------------------------------------------- mission ---

class Mission:
    def __init__(self, v: Vehicle, args):
        self.v = v
        self.a = args
        self.home = None   # (n, e, d) at takeoff
        self.home_yaw = None

    def guard(self):
        """Called in every wait loop."""
        v = self.v
        if self.a.require_guided and v.mode != "GUIDED":
            raise PilotOverride(f"mode changed to {v.mode}")
        if time.time() - v.last_pos_time > 1.5:
            raise MissionAbort("lost LOCAL_POSITION_NED (EKF/position estimate stale)")
        if v.voltage is not None and v.voltage < self.a.min_voltage:
            raise MissionAbort(f"battery low: {v.voltage:.2f} V < {self.a.min_voltage} V")

    def height(self):
        """Height above the takeoff point, or None if unknown."""
        if self.home is None or self.v.pos is None:
            return None
        return self.home[2] - self.v.pos[2]

    def to_ned(self, wp):
        """Convert a waypoint (relative to home) into an absolute local-NED target."""
        a, b, alt = wp
        if self.a.frame == "body":
            c, s = math.cos(self.home_yaw), math.sin(self.home_yaw)
            dn, de = a * c - b * s, a * s + b * c   # forward/right -> north/east
        else:
            dn, de = a, b
        hn, he, hd = self.home
        return hn + dn, he + de, hd - alt

    def preflight(self):
        v = self.v
        for wp in self.a.waypoints:
            r = math.hypot(wp[0], wp[1])
            if r > self.a.max_radius or not (1.0 <= wp[2] <= self.a.max_alt):
                raise MissionAbort(f"waypoint {wp} outside limits "
                                   f"(radius<={self.a.max_radius}, 1<=alt<={self.a.max_alt})")
        print("[pre] waiting for local position + attitude ...")
        end = time.time() + 30
        while (v.pos is None or v.yaw is None) and time.time() < end:
            v.pump(0.5)
        if v.pos is None or v.yaw is None:
            raise MissionAbort("no LOCAL_POSITION_NED/ATTITUDE — is the EKF origin set? (GPS lock or flow + set origin)")

        # The drone is sitting still, so the position estimate should be too.
        print(f"[pre] checking position estimate is steady ({self.a.steady_time:.0f} s) ...")
        p0 = v.pos
        end = time.time() + self.a.steady_time
        while time.time() < end:
            v.pump(0.2)
            drift = math.hypot(v.pos[0] - p0[0], v.pos[1] - p0[1])
            if drift > self.a.max_ground_drift:
                raise MissionAbort(f"position estimate drifted {drift:.2f} m while disarmed on the ground "
                                   f"— EKF/GPS/flow not good enough to fly. Not arming.")
        self.home = v.pos
        self.home_yaw = v.yaw
        print(f"[pre] home NED={tuple(round(x, 2) for x in self.home)}  "
              f"heading={math.degrees(self.home_yaw):.1f}°  battery={v.voltage} V")
        print(f"[pre] GPS: {v.gps_text()}")
        print("[pre] plan:")
        for i, wp in enumerate(self.a.waypoints, 1):
            n, e, d = self.to_ned(wp)
            print(f"   WP{i}: {wp}  ->  N={n:.2f} E={e:.2f} alt={-d + self.home[2]:.2f}")
        v.log("preflight_ok")

    def takeoff(self):
        """Gradual takeoff:
          1. arm and let the motors spool up for --spool-wait seconds,
          2. lift off to a small height (--liftoff-alt) with MAV_CMD_NAV_TAKEOFF,
          3. slide the altitude target up to --takeoff-alt at --climb-rate."""
        v, alt = self.v, self.a.takeoff_alt
        first = min(self.a.liftoff_alt, alt)
        v.apply_throttle_cap()
        v.set_mode("GUIDED")
        v.arm()

        print(f"[takeoff] motors spooling for {self.a.spool_wait:.1f} s")
        # Measure drift from the moment of arming, not from preflight: the GPS
        # estimate wanders slowly while you wait at the ENTER prompt, and that
        # is not a reason to abort.
        ref = v.pos
        end = time.time() + self.a.spool_wait
        while time.time() < end:
            v.pump(0.2)
            self.guard()
            v.log()
            drift = math.hypot(v.pos[0] - ref[0], v.pos[1] - ref[1])
            if drift > self.a.max_ground_drift:
                raise MissionAbort(f"position estimate drifted {drift:.2f} m on the ground after arming "
                                   f"(vibration? weak GPS/flow?) — not taking off")
        if not v.armed:
            raise MissionAbort("disarmed during spool-up")
        # Re-anchor home to where the estimate says we are right now, so later
        # waypoints are relative to the actual lift-off point.
        self.home = v.pos

        print(f"[takeoff] lift-off to {first:.1f} m")
        v.takeoff_cmd(first)
        v.log("takeoff_cmd")
        end = time.time() + self.a.leg_timeout
        while True:
            if time.time() > end:
                raise MissionAbort("takeoff timeout")
            v.pump(0.2)
            self.guard()
            v.log()
            if self.height() >= first * 0.8:
                break

        if alt - first > 0.05:
            print(f"[takeoff] climbing to {alt:.1f} m at {self.a.climb_rate} m/s")
            self.goto("climb", (0.0, 0.0, alt), speed=self.a.climb_rate)
        else:
            v.pump(self.a.hover)
        print(f"[takeoff] at {self.height():.2f} m")

    def goto(self, tag, wp, speed=None):
        """Slide the setpoint from the current position to the waypoint at `speed`
        (default --speed), then hold until the drone is inside --tol AND has slowed
        below SETTLED_SPEED."""
        v, spd = self.v, (speed or self.a.speed)
        target = self.to_ned(wp)
        start = v.pos
        delta = [t - s for t, s in zip(target, start)]
        dist = math.sqrt(sum(x * x for x in delta))
        unit = [x / dist for x in delta] if dist > 1e-3 else [0.0, 0.0, 0.0]
        ramp_time = dist / spd if dist > 1e-3 else 0.0

        print(f"[{tag}] -> {wp}  ({dist:.1f} m at {spd} m/s)")
        v.log(f"{tag}_start")
        t0 = time.time()
        deadline = t0 + ramp_time + self.a.leg_timeout
        while time.time() < deadline:
            el = time.time() - t0
            if el < ramp_time:                       # moving target
                s = spd * el
                sp = [st + u * s for st, u in zip(start, unit)]
                ff = [u * spd for u in unit]
            else:                                    # parked on the waypoint
                sp, ff = list(target), [0.0, 0.0, 0.0]
            v.send_setpoint(sp, ff, self.home_yaw)
            v.pump(0.1)
            self.guard()
            v.log()

            if el >= ramp_time:
                err_h = math.hypot(target[0] - v.pos[0], target[1] - v.pos[1])
                err_v = abs(target[2] - v.pos[2])
                if err_h < self.a.tol and err_v < self.a.tol and v.speed() < SETTLED_SPEED:
                    print(f"[{tag}] reached (h_err={err_h:.2f} m, v_err={err_v:.2f} m, "
                          f"speed={v.speed():.2f} m/s), hovering {self.a.hover}s")
                    print(f"[{tag}] GPS: {v.gps_text()}")
                    v.log(f"{tag}_reached")
                    hold_end = time.time() + self.a.hover
                    while time.time() < hold_end:
                        v.send_setpoint(target, (0.0, 0.0, 0.0), self.home_yaw)
                        v.pump(0.2)
                        self.guard()
                        v.log()
                    return
        raise MissionAbort(f"{tag} timeout")

    def _wait_disarm(self, mode, timeout, tag):
        """Wait for touchdown + disarm while ArduCopter flies `mode` by itself.
        If the pilot switches to another mode, stop watching — they have control."""
        v = self.v
        end = time.time() + timeout
        while time.time() < end:
            v.pump(0.5)
            v.log()
            if not v.armed:
                print(f"[{tag}] disarmed — on the ground")
                v.log("disarmed")
                return
            if v.mode not in (mode, "LAND"):   # RTL switches to LAND near home
                raise PilotOverride(f"mode changed to {v.mode} during {mode}")
        print(f"[{tag}] WARNING: still armed after {timeout:.0f} s — take manual control")

    def land(self):
        print("[land] LAND mode — landing here")
        self.v.set_mode("LAND")
        self._wait_disarm("LAND", 90, "land")

    def return_home(self):
        """ArduCopter RTL: climb to RTL_ALT (if lower), fly to home, land, disarm."""
        print("[rtl] RTL mode — returning to takeoff point")
        self.v.set_mode("RTL")
        self._wait_disarm("RTL", self.a.rtl_timeout, "rtl")

    def run(self):
        self.preflight()
        if not self.a.yes:
            input("\n>>> Props clear? Safety pilot ready on RC? Press ENTER to arm, Ctrl+C to cancel ")
        self.takeoff()
        for i, wp in enumerate(self.a.waypoints, 1):
            self.goto(f"wp{i}", wp)
        self.land()


# ------------------------------------------------------------------ main ---

def parse_wp(s):
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("waypoint must be a,b,alt")
    return tuple(parts)


def main():
    p = argparse.ArgumentParser(description="Autonomous takeoff -> waypoints -> land")
    p.add_argument("--conn", default="/dev/ttyAMA2",
                   help="MAVLink connection (Pi: /dev/ttyAMA2, SITL: tcp:127.0.0.1:5762)")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--wp", dest="waypoints", type=parse_wp, action="append",
                   help="waypoint 'a,b,alt' (repeat). Default: 2 m square at 1 m")
    p.add_argument("--frame", choices=["body", "ned"], default="body")
    p.add_argument("--takeoff-alt", type=float, default=2.0)
    p.add_argument("--spool-wait", type=float, default=7.0,
                   help="seconds to let motors spin up after arming, before lift-off")
    p.add_argument("--liftoff-alt", type=float, default=0.5,
                   help="initial lift-off height (m) before the slow climb")
    p.add_argument("--climb-rate", type=float, default=0.5,
                   help="climb speed from lift-off to --takeoff-alt (m/s)")
    p.add_argument("--tol", type=float, default=0.3, help="arrival tolerance (m)")
    p.add_argument("--hover", type=float, default=3.0, help="hover time at each wp (s)")
    p.add_argument("--speed", type=float, default=0.25, help="travel speed between waypoints (m/s)")
    p.add_argument("--leg-timeout", type=float, default=40.0)
    p.add_argument("--max-ground-drift", type=float, default=0.8,
                   help="abort if the position estimate moves more than this (m) while on the ground")
    p.add_argument("--steady-time", type=float, default=3.0,
                   help="seconds to watch the position estimate before arming")
    p.add_argument("--max-radius", type=float, default=15.0)
    p.add_argument("--max-alt", type=float, default=10.0)
    p.add_argument("--min-voltage", type=float, default=14.0, help="4S abort voltage")
    p.add_argument("--abort-action", choices=["rtl", "land"], default="rtl",
                   help="on abort: fly home and land (rtl) or land in place (land)")
    p.add_argument("--rtl-timeout", type=float, default=180.0,
                   help="max time to wait for RTL to finish (s)")
    p.add_argument("--no-require-guided", dest="require_guided", action="store_false",
                   help="(don't) treat leaving GUIDED as pilot override")
    p.add_argument("--log", default=f"wp_mission_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    p.add_argument("-y", "--yes", action="store_true", help="skip the arm confirmation prompt")
    args = p.parse_args()
    args.waypoints = args.waypoints or DEFAULT_WAYPOINTS

    v = Vehicle(args.conn, args.baud, args.log)
    mission = Mission(v, args)
    try:
        mission.run()
    except PilotOverride as e:
        print(f"\n[override] {e} — script stopped sending commands. Pilot has control.")
        v.log("pilot_override")
    except (MissionAbort, KeyboardInterrupt) as e:
        reason = str(e) or "Ctrl+C"
        v.log(f"abort:{reason}")
        if not v.armed:
            print(f"\n[abort] {reason} — not armed, nothing to do")
        else:
            # RTL needs a position estimate; if that is what failed, land in place.
            pos_lost = "LOCAL_POSITION_NED" in reason
            # Below RTL_MIN_HEIGHT (still on/near the ground) RTL would climb to
            # RTL_ALT first — on a props-off bench that means motors spin at full
            # climb forever. Just land: ArduCopter detects touchdown and disarms.
            height = mission.height()
            low = height is None or height < RTL_MIN_HEIGHT
            action = "land" if (pos_lost or low) else args.abort_action
            h_txt = "unknown" if height is None else f"{height:.1f} m"
            print(f"\n[abort] {reason} — {action.upper()} (height {h_txt})")
            safe_abort(mission, action)
    finally:
        print(f"[log] {args.log}")


def safe_abort(mission, action):
    """Execute the abort action. RTL refused -> LAND. Ctrl+C during RTL -> LAND."""
    try:
        if action == "rtl":
            try:
                mission.return_home()
                return
            except MissionAbort as e:
                print(f"[abort] RTL not accepted ({e}) — LAND instead")
            except KeyboardInterrupt:
                print("\n[abort] Ctrl+C during RTL — LAND here instead")
        mission.land()
    except PilotOverride as e:
        print(f"[override] {e} — pilot has control.")
    except KeyboardInterrupt:
        print("\n[abort] script stopped — vehicle stays in its current mode. TAKE MANUAL CONTROL")
    except MissionAbort:
        print("[abort] could not set LAND — TAKE MANUAL CONTROL")


if __name__ == "__main__":
    sys.exit(main())
