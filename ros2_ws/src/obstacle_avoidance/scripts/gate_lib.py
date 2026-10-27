#!/usr/bin/env python3
"""gate_lib.py — shared SITL/MAVLink helpers for the Stage 1 and Stage 2 gates.

Deliberately observation-only where it matters: the flight steps record WHAT
happened (which sweep window was active, timestamped in the FC's own boot clock)
and the analysis prints WHAT the log shows. Neither step encodes the expected
sector mapping — that is the human's to judge, because a test whose expectation
comes from the same geometry.py it is checking would pass on mirrored data.

Control link: a pymavlink connection (default tcp:127.0.0.1:5760) used for
arm/mode/RC/param/reboot. The obstacle path (synthetic node -> MAVROS -> FC) is
separate and lives in the bash orchestrators. Window boundaries are captured in
GLOBAL_POSITION_INT.time_boot_ms, which shares the FC boot clock with the
dataflash PRX.TimeUS, so correlation is exact and needs no wall-clock/GPS sync.

Subcommands: load-params, reboot, stage1-flight, stage2-flight, analyze-prx.
"""

import argparse
import csv
import math
import subprocess
import sys
import time

from pymavlink import mavutil

# --- message ids we stream ---
MSG_RATES = {30: 10, 32: 10, 33: 10, 24: 2, 74: 5, 193: 2, 2: 1}
#            ATT   LPN  GPI   GPS  VFR  EKF  SYSTIME


def log(msg):
    print(f"[gate_lib {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def die(msg, code=2):
    print(f"[gate_lib FATAL] {msg}", file=sys.stderr, flush=True)
    sys.exit(code)


# --------------------------------------------------------------------------- #
# connection / streams
# --------------------------------------------------------------------------- #
def connect(url, timeout=60):
    log(f"connecting {url} ...")
    m = mavutil.mavlink_connection(url, source_system=250, source_component=1)
    # Bounded, backed-off wait: a dead/closed TCP socket EOFs on every recv,
    # and a tight loop over that prints "EOF on TCP socket" fast enough to write
    # hundreds of MB. The sleep caps that and gives the port time to come up.
    t0 = time.time()
    hb = None
    while time.time() - t0 < timeout:
        hb = m.recv_match(type='HEARTBEAT', blocking=False)
        if hb is not None:
            break
        time.sleep(0.2)
    if hb is None:
        die(f"no heartbeat from {url} within {timeout}s")
    log(f"heartbeat: sys={m.target_system} comp={m.target_component}")
    request_streams(m)
    return m


def request_streams(m):
    for msgid, hz in MSG_RATES.items():
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msgid, int(1e6 / hz), 0, 0, 0, 0, 0)
        time.sleep(0.05)


def boot_ms(m, timeout=2.0):
    """Latest FC boot time in ms (shares the clock with PRX.TimeUS)."""
    t0 = time.time()
    val = None
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'],
                           blocking=True, timeout=timeout)
        if msg is not None:
            val = msg.time_boot_ms
            break
    return val


def rel_alt(m, timeout=2.0):
    msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True,
                       timeout=timeout)
    return None if msg is None else msg.relative_alt / 1000.0


def heading(m, timeout=2.0):
    msg = m.recv_match(type='VFR_HUD', blocking=True, timeout=timeout)
    return None if msg is None else msg.heading


def local_ned(m, timeout=2.0):
    msg = m.recv_match(type='LOCAL_POSITION_NED', blocking=True,
                       timeout=timeout)
    return None if msg is None else (msg.x, msg.y, msg.z)


# --------------------------------------------------------------------------- #
# EKF / arm / mode / RC
# --------------------------------------------------------------------------- #
EKF_OK_BITS = (mavutil.mavlink.EKF_ATTITUDE
               | mavutil.mavlink.EKF_VELOCITY_HORIZ
               | mavutil.mavlink.EKF_POS_HORIZ_ABS
               | mavutil.mavlink.EKF_PRED_POS_HORIZ_ABS)


def wait_ekf(m, timeout=150):
    log("waiting for EKF/GPS to settle ...")
    t0 = time.time()
    gps_ok = ekf_ok = False
    while time.time() - t0 < timeout:
        msg = m.recv_match(
            type=['EKF_STATUS_REPORT', 'GPS_RAW_INT'], blocking=True,
            timeout=2)
        if msg is None:
            continue
        if msg.get_type() == 'GPS_RAW_INT':
            gps_ok = msg.fix_type >= 3
        elif msg.get_type() == 'EKF_STATUS_REPORT':
            ekf_ok = (msg.flags & EKF_OK_BITS) == EKF_OK_BITS
        if gps_ok and ekf_ok:
            log(f"EKF ready after {time.time()-t0:.0f}s")
            return True
    die(f"EKF not ready within {timeout}s (gps_ok={gps_ok} ekf_ok={ekf_ok})")


def set_mode(m, name, timeout=15):
    mapping = m.mode_mapping()
    if name not in mapping:
        die(f"mode {name} not in mapping {list(mapping)}")
    mode_id = mapping[name]
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.mav.set_mode_send(
            m.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            log(f"mode = {name}")
            return True
    die(f"could not enter mode {name} within {timeout}s")


def arm(m, timeout=90):
    log("arming (cold SITL may reject for ~30-60s) ...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log(f"armed after {time.time()-t0:.0f}s")
            return True
    die(f"failed to arm within {timeout}s")


def disarm(m, force=False, timeout=30):
    log("disarming ...")
    p2 = 21196 if force else 0
    t0 = time.time()
    while time.time() - t0 < timeout:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, p2, 0, 0, 0, 0, 0)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("disarmed")
            return True
    log("WARN: still armed after disarm attempts")
    return False


def rc(m, ch1=0, ch2=0, ch3=0, ch4=0):
    """Override RC. 0 = release that channel to its default."""
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        ch1, ch2, ch3, ch4, 0, 0, 0, 0)


def hold(m, seconds, keep_rc=None):
    """Idle for `seconds`, draining messages and refreshing RC override."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        if keep_rc is not None:
            rc(m, **keep_rc)
        m.recv_match(blocking=True, timeout=0.4)


def climb_to(m, alt_m, keep_rc, timeout=30):
    log(f"climbing to {alt_m} m ...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        rc(m, **{**keep_rc, 'ch3': 1700})       # throttle up
        a = rel_alt(m)
        if a is not None and a >= alt_m:
            log(f"reached {a:.1f} m")
            rc(m, **{**keep_rc, 'ch3': 1500})    # neutral throttle = hold
            return True
    log(f"WARN: climb timeout at {rel_alt(m)} m")
    rc(m, **{**keep_rc, 'ch3': 1500})
    return False


# --------------------------------------------------------------------------- #
# params
# --------------------------------------------------------------------------- #
def _read_param(m, name, timeout=3):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        pv = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
        if pv and pv.param_id.strip('\x00') == name:
            return pv.param_value
    return None


def load_params(m, pairs, tol=1e-3):
    """Set each (name, value) and read it back. Returns list of result dicts.

    Verifies the write actually took — an unknown param name is silently
    dropped by the FC (as PRX1_YAW_CORR was), and this is where that shows up.
    """
    results = []
    for name, value in pairs:
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.2)
        got = _read_param(m, name)
        if got is None:
            status = 'UNKNOWN/NO-ACK'
        elif abs(got - float(value)) > tol:
            status = f'MISMATCH(got={got:g})'
        else:
            status = 'OK'
        results.append({'name': name, 'want': float(value),
                        'got': got, 'status': status})
        log(f"  param {name} -> want {value} got {got} : {status}")
    return results


def parse_param_file(path):
    pairs = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            parts = ln.split()
            if len(parts) >= 2:
                pairs.append((parts[0], float(parts[1])))
    return pairs


def reboot(m, url, timeout=90):
    log("rebooting FC ...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
        1, 0, 0, 0, 0, 0, 0)
    m.close()
    time.sleep(5)
    m2 = connect(url, timeout=timeout)
    return m2


def set_ros_param(node, name, value):
    """Set a ROS param on the running synthetic node (the sweep mechanism)."""
    r = subprocess.run(['ros2', 'param', 'set', node, name, str(value)],
                       capture_output=True, text=True, timeout=15)
    ok = r.returncode == 0 and 'Set parameter successful' in r.stdout
    if not ok:
        log(f"WARN ros2 param set {node} {name}={value}: "
            f"{r.stdout.strip()} {r.stderr.strip()}")
    return ok


# --------------------------------------------------------------------------- #
# subcommands
# --------------------------------------------------------------------------- #
def cmd_load_params(a):
    m = connect(a.url)
    pairs = parse_param_file(a.file) if a.file else []
    for kv in (a.set or []):
        name, val = kv.split('=')
        pairs.append((name, float(val)))
    results = load_params(m, pairs)
    bad = [r for r in results if r['status'] != 'OK']
    print("\n=== param verification ===")
    for r in results:
        print(f"  {r['name']:<16} want={r['want']:<8g} "
              f"got={r['got']} {r['status']}")
    if bad and a.strict:
        die(f"{len(bad)} param(s) did not take: "
            + ', '.join(r['name'] for r in bad))


def cmd_reboot(a):
    m = connect(a.url)
    reboot(m, a.url)
    log("reboot complete, FC back")


def _open_windows_writer(path):
    f = open(path, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['label', 'bearing_deg', 'boot_ms_start', 'boot_ms_end',
                'wall_start', 'wall_end'])
    return f, w


def cmd_stage1_flight(a):
    m = connect(a.url)
    wait_ekf(m)
    keep = {'ch1': 1500, 'ch2': 1500, 'ch4': 1500}      # roll/pitch/yaw neutral
    set_mode(m, 'LOITER')
    arm(m)
    climb_to(m, a.alt, keep)

    # fix obstacle range/width so the analysis knows the target distance
    set_ros_param(a.node, 'obstacle_range_m', a.range)
    set_ros_param(a.node, 'obstacle_width_deg', a.width)

    fh, w = _open_windows_writer(a.windows_out)
    bearings = [float(x) for x in a.bearings.split(',')]

    def window(label, bearing, do):
        set_ros_param(a.node, 'obstacle_bearing_deg', bearing)
        time.sleep(1.0)                       # let the new scan propagate
        b0, t0 = boot_ms(m), time.time()
        print(f">>> WINDOW {label}: bearing={bearing:+.1f}  "
              f"wall={time.strftime('%H:%M:%S')}  boot_ms={b0}", flush=True)
        do()
        b1, t1 = boot_ms(m), time.time()
        w.writerow([label, bearing, b0, b1, f"{t0:.3f}", f"{t1:.3f}"])
        fh.flush()

    # Gate 1A/1B: sweep bearings, hold each
    for b in bearings:
        window(f"sweep_{b:+.0f}", b, lambda: hold(m, a.hold, keep))

    # Gate 1C: obstacle back to forward, yaw the body ~90 deg
    def yaw90():
        h0 = heading(m)
        rc(m, **{**keep, 'ch4': 1700})        # yaw right
        t0 = time.time()
        while time.time() - t0 < 20:
            h = heading(m)
            if h is not None and abs((h - h0 + 180) % 360 - 180) >= 88:
                break
            m.recv_match(blocking=True, timeout=0.3)
        rc(m, **{**keep, 'ch4': 1500})
        log(f"yawed from {h0} to {heading(m)} deg")
        hold(m, a.hold, keep)
    window("yaw90_bearing_0", 0.0, yaw90)

    fh.close()
    set_mode(m, 'LAND')
    disarm(m, timeout=40) or disarm(m, force=True)
    log(f"stage1 flight done; windows -> {a.windows_out}")


def cmd_stage2_flight(a):
    m = connect(a.url)
    wait_ekf(m)
    keep = {'ch1': 1500, 'ch2': 1500, 'ch4': 1500}
    set_ros_param(a.node, 'obstacle_bearing_deg', 0.0)
    set_ros_param(a.node, 'obstacle_range_m', a.range)
    set_ros_param(a.node, 'obstacle_width_deg', a.width)
    set_mode(m, 'LOITER')
    arm(m)
    climb_to(m, a.alt, keep)
    hold(m, 3, keep)

    # Snapshot the wall's world position: obstacle_range ahead along heading.
    # NOTE the caveat (see script header): the synthetic obstacle is BODY-fixed
    # at constant range, so it recedes as the drone advances. Closest approach
    # is measured against this frozen snapshot so the number is still meaningful.
    x0, y0, _ = local_ned(m)
    hdg = math.radians(heading(m))
    fwd = (math.cos(hdg), math.sin(hdg))       # NED unit forward
    log(f"pitch-start NED=({x0:.2f},{y0:.2f}) hdg={math.degrees(hdg):.0f} "
        f"wall {a.range} m ahead")

    log("pitching forward (rc2=1400) ...")
    closest = a.range
    t0 = time.time()
    while time.time() - t0 < a.push_s:
        rc(m, **{**keep, 'ch2': 1400})         # pitch forward = move forward
        p = local_ned(m)
        if p is not None:
            dfwd = (p[0] - x0) * fwd[0] + (p[1] - y0) * fwd[1]
            approach = a.range - dfwd           # remaining distance to wall
            closest = min(closest, approach)
        m.recv_match(blocking=True, timeout=0.2)
    rc(m, **keep)

    print("\n=== STAGE 2 RESULT ===")
    print(f"closest_approach_m = {closest:.2f}   (judge against AVOID_MARGIN "
          f"1.2 m; drone should refuse to close within it)")
    with open(a.out, 'w') as f:
        f.write(f"closest_approach_m,{closest:.3f}\n")

    set_mode(m, 'LAND')
    disarm(m, timeout=40) or disarm(m, force=True)


def cmd_analyze_prx(a):
    """Read PRX.D0/D45/D315 from the .BIN, slice by windows, print OBSERVED."""
    windows = []
    with open(a.windows) as f:
        for row in csv.DictReader(f):
            windows.append(row)

    mlog = mavutil.mavlink_connection(a.bin)
    rows = []                       # (t_ms, layer, D0, D45, D315)
    fields_seen = None
    while True:
        msg = mlog.recv_match(type='PRX', blocking=False)
        if msg is None:
            break
        d = msg.to_dict()
        if fields_seen is None:
            fields_seen = list(d.keys())
        rows.append((d.get('TimeUS', 0) / 1000.0, d.get('Layer'),
                     d.get('D0'), d.get('D45'), d.get('D315')))

    if not rows:
        die("PRX is defined but has no rows — was the synthetic node feeding "
            "/mavros/obstacle/send, PRX1_TYPE=2, and logging on during flight?")
    print(f"PRX fields present: {fields_seen}")

    with open(a.csv_out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['boot_ms', 'Layer', 'D0', 'D45', 'D315'])
        w.writerows(rows)
    print(f"raw PRX CSV -> {a.csv_out}  ({len(rows)} rows)\n")

    def median(vs):
        vs = sorted(vs)
        return vs[len(vs) // 2] if vs else float('nan')

    near = a.range          # "sees the obstacle" == reading within 0.5 m of this
    print("=== OBSERVED per window (NOT asserted — you judge the mapping) ===")
    print(f"cell = median_of_valid(>0)  [n_valid, n_near~{near:.1f}m]")
    hdr = (f"{'window':<18}{'bearing':>8} |{'D0':>17}{'D45':>17}"
           f"{'D315':>17} | {'nearest(min med)':>16}")
    print(hdr)
    print("-" * len(hdr))
    for win in windows:
        s, e = float(win['boot_ms_start']), float(win['boot_ms_end'])
        seg = [r for r in rows if s <= r[0] <= e]
        cells, meds = {}, {}
        for idx, k in ((2, 'D0'), (3, 'D45'), (4, 'D315')):
            valid = [r[idx] for r in seg if r[idx] is not None and r[idx] > 0]
            n_near = sum(1 for v in valid if abs(v - near) <= 0.5)
            m = median(valid)
            meds[k] = m
            cells[k] = (f"{m:6.2f} [{len(valid):>3},{n_near:>3}]"
                        if valid else f"{'--':>6} [  0,  0]")
        finite = {k: v for k, v in meds.items() if v == v}
        nearest = min(finite, key=finite.get) if finite else '--'
        print(f"{win['label']:<18}{float(win['bearing_deg']):>+8.0f} |"
              f"{cells['D0']:>17}{cells['D45']:>17}{cells['D315']:>17} | "
              f"{nearest:>16}")
    print(f"\nRead it yourself: which sector holds ~{near:.1f} m in each window?")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default='tcp:127.0.0.1:5760')
    sub = p.add_subparsers(dest='cmd', required=True)

    lp = sub.add_parser('load-params')
    lp.add_argument('--file')
    lp.add_argument('--set', action='append', help='NAME=VALUE, repeatable')
    lp.add_argument('--strict', action='store_true')
    lp.set_defaults(func=cmd_load_params)

    rb = sub.add_parser('reboot')
    rb.set_defaults(func=cmd_reboot)

    s1 = sub.add_parser('stage1-flight')
    s1.add_argument('--node', default='/synthetic_sector_node')
    s1.add_argument('--alt', type=float, default=5.0)
    s1.add_argument('--bearings', default='0.0,-30.0,30.0')
    s1.add_argument('--hold', type=float, default=10.0)
    s1.add_argument('--range', type=float, default=3.0)
    s1.add_argument('--width', type=float, default=10.0)
    s1.add_argument('--windows-out', default='/tmp/stage1_windows.csv')
    s1.set_defaults(func=cmd_stage1_flight)

    s2 = sub.add_parser('stage2-flight')
    s2.add_argument('--node', default='/synthetic_sector_node')
    s2.add_argument('--alt', type=float, default=5.0)
    s2.add_argument('--range', type=float, default=3.0)
    s2.add_argument('--width', type=float, default=10.0)
    s2.add_argument('--push-s', type=float, default=15.0)
    s2.add_argument('--out', default='/tmp/stage2_result.csv')
    s2.set_defaults(func=cmd_stage2_flight)

    an = sub.add_parser('analyze-prx')
    an.add_argument('--bin', required=True)
    an.add_argument('--windows', required=True)
    an.add_argument('--range', type=float, default=3.0)
    an.add_argument('--csv-out', default='/tmp/prx_raw.csv')
    an.set_defaults(func=cmd_analyze_prx)

    a = p.parse_args()
    a.func(a)


if __name__ == '__main__':
    main()
