#!/usr/bin/env python3
"""pi_check.py — on the drone, before flying: are the OAK-D and the RPi Camera
Module 1.3 feeds working, and can the Pi 5 run everything at once?

    ./tools/pi_check.sh                 # on the Pi (props OFF; FC may be connected)
    ./tools/pi_check.sh --duration 300  # longer full-load soak

Runs the mission's OWN stage launches (mission_stage.launch.py), not a copy:

  1. system   Pi model, RAM, 5 V input (PMIC), USB current limit, temperature,
              throttling / under-voltage flags, OAK-D on USB (USB3 or USB2),
              Pi camera listed by rpicam
  2. OAK-D    oak stage alone: colour + depth rate, obstacle sensor live,
              live view http://<pi>:8081/ answers, one snapshot saved
  3. Cam 1.3  + down_cam stage: /down_frame rate, live view :8080, snapshot
  4. LOAD     + red_zone_node + corridor_perception + path_planner +
              qr_scanner_node scanning, ALL AT ONCE (more than any real phase:
              in flight the red zone and the exit corridor never overlap) for
              --duration s: CPU, RAM, temperature, throttling, and the camera
              rates must hold

MAVROS is started on /dev/ttyAMA2 if it exists (--fc none to skip): the red-zone
node only processes frames with FC telemetry, so without the FC the load is lower
than in flight (reported). Nothing is armed or commanded.

Report + snapshots: ~/aerothon_logs/pi_check_<time>/ (report.txt, oak.jpg,
down.jpg, launch output). Exit code 0 = all PASS, 1 = a FAIL.
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

OAK_RGB, OAK_DEPTH, DOWN = "/oak/rgb/image_raw", "/oak/stereo/image_raw", "/down_frame"
# Pass limits
MIN_HZ = {OAK_RGB: 10.0, OAK_DEPTH: 10.0, DOWN: 10.0}   # OAK runs 15 fps, Cam 1.3 ~30
LOAD_KEEP = 0.75            # under full load each feed keeps >= 75 % of its alone rate
MAX_CPU = 85.0              # % of all cores, average over the load test
MIN_MEM_MB = 500.0          # MemAvailable, minimum
MAX_TEMP_C = 80.0           # Pi 5 throttles at 85 C
MIN_5V = 4.85               # PMIC EXT5V_V under load


# ───────────────────────── report ─────────────────────────
class Report:
    def __init__(self, path):
        self.f = open(path, "w", buffering=1)
        self.fails = 0
        self.warns = 0

    def line(self, text=""):
        print(text, flush=True)
        self.f.write(text + "\n")

    def check(self, ok, what, detail="", warn_only=False):
        tag = "PASS" if ok else ("WARN" if warn_only else "FAIL")
        if not ok:
            if warn_only:
                self.warns += 1
            else:
                self.fails += 1
        self.line(f"  [{tag}] {what}" + (f" — {detail}" if detail and not ok else ""))
        return ok


def sh(cmd, timeout=10.0):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return (r.stdout + r.stderr).strip()
    except Exception as e:      # noqa: BLE001
        return f"({e})"


def read(path, default=""):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return default


# ───────────────────────── system probes ─────────────────────────
def temp_c():
    t = read("/sys/class/thermal/thermal_zone0/temp")
    return int(t) / 1000.0 if t.isdigit() else None


def throttled():
    """vcgencmd get_throttled as an int (None off the Pi)."""
    out = sh("vcgencmd get_throttled 2>/dev/null")
    if "=" not in out:
        return None
    try:
        return int(out.split("=")[1], 16)
    except ValueError:
        return None


def throttle_text(v):
    bits = {0: "UNDER-VOLTAGE now", 1: "freq capped now", 2: "THROTTLED now",
            3: "soft temp limit now", 16: "under-voltage happened",
            17: "freq cap happened", 18: "throttling happened",
            19: "soft temp limit happened"}
    return ", ".join(t for b, t in bits.items() if v & (1 << b)) or "none"


def ext5v():
    """Pi 5 input voltage from the PMIC (None if not available)."""
    for line in sh("vcgencmd pmic_read_adc EXT5V_V 2>/dev/null").splitlines():
        if "EXT5V_V" in line and "=" in line:
            try:
                return float(line.split("=")[1].rstrip("V"))
            except ValueError:
                pass
    return None


def oak_usb():
    """(found, speed Mb/s) of a Luxonis/Movidius device (USB vendor 03e7)."""
    base = "/sys/bus/usb/devices"
    if not os.path.isdir(base):
        return False, None
    for d in os.listdir(base):
        if read(os.path.join(base, d, "idVendor")) == "03e7":
            sp = read(os.path.join(base, d, "speed"))
            return True, float(sp) if sp else None
    return False, None


class CpuSampler:
    """Whole-system CPU % and the top processes, from /proc (no psutil)."""

    def __init__(self):
        self.ncpu = os.cpu_count() or 1
        self.hz = os.sysconf("SC_CLK_TCK")
        self._total = self._proc = None
        self._t = None

    @staticmethod
    def _sys():
        f = [int(x) for x in read("/proc/stat").splitlines()[0].split()[1:]]
        idle = f[3] + f[4]
        return sum(f), idle

    def _procs(self):
        out = {}
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            st = read(f"/proc/{pid}/stat")
            if not st:
                continue
            rest = st.rsplit(")", 1)[-1].split()
            ticks = int(rest[11]) + int(rest[12])
            cmd = read(f"/proc/{pid}/cmdline").replace("\0", " ")
            name = st[st.find("(") + 1:st.rfind(")")]
            for part in cmd.split():
                if part.startswith("__node:="):
                    name = part[8:]
                    break
            out[pid] = (name, ticks)
        return out

    def sample(self):
        """(total % of all cores, [(name, % of one core), ...] top 8) since last call."""
        total, idle = self._sys()
        procs, now = self._procs(), time.monotonic()
        res = (None, [])
        if self._total is not None:
            dt, di = total - self._total[0], idle - self._total[1]
            cpu = 100.0 * (dt - di) / dt if dt > 0 else 0.0
            el = now - self._t
            per = {}
            for pid, (name, ticks) in procs.items():
                if pid in self._proc:
                    d = ticks - self._proc[pid][1]
                    if d > 0:
                        per[name] = per.get(name, 0.0) + 100.0 * d / self.hz / el
            res = (cpu, sorted(per.items(), key=lambda kv: -kv[1])[:8])
        self._total, self._proc, self._t = (total, idle), procs, now
        return res


def mem_available_mb():
    for line in read("/proc/meminfo").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) / 1024.0
    return None


# ───────────────────────── ROS measuring ─────────────────────────
class Probe(Node):
    def __init__(self):
        super().__init__("pi_check")
        self.live = []                  # /avoidance/sensor_live values
        self.create_subscription(Bool, "/avoidance/sensor_live",
                                 lambda m: self.live.append(bool(m.data)), 10)

    def rate(self, topic, secs):
        """Messages / s on an image topic over secs (subscribed only meanwhile:
        a Python subscriber copying 2.7 MB frames is load of its own)."""
        n = [0]
        sub = self.create_subscription(Image, topic, lambda _m: n.__setitem__(0, n[0] + 1),
                                       qos_profile_sensor_data)
        time.sleep(secs)
        self.destroy_subscription(sub)
        return n[0] / secs

    def wait_topic(self, topic, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if self.count_publishers(topic) > 0 and self.rate(topic, 1.0) > 0:
                return True
            time.sleep(1.0)
        return False

    def call(self, srv, timeout=5.0):
        cli = self.create_client(Trigger, srv)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            fut = cli.call_async(Trigger.Request())
            end = time.time() + timeout
            while not fut.done() and time.time() < end:
                time.sleep(0.05)
            return fut.result() if fut.done() else None
        finally:
            self.destroy_client(cli)


def snapshot(port, path, timeout=8.0):
    """One JPEG from the live view's MJPEG stream (proves the feed a browser
    sees). True if saved."""
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/stream.mjpg",
                                    timeout=timeout) as r:
            buf, end = b"", time.time() + timeout
            while time.time() < end and len(buf) < 4_000_000:
                buf += r.read(4096)
                a = buf.find(b"\xff\xd8")
                b = buf.find(b"\xff\xd9", a + 2) if a >= 0 else -1
                if a >= 0 and b > a:
                    with open(path, "wb") as f:
                        f.write(buf[a:b + 2])
                    return True
    except Exception:           # noqa: BLE001
        pass
    return False


# ───────────────────────── processes ─────────────────────────
class Procs:
    def __init__(self, outdir):
        self.outdir = outdir
        self.procs = []

    def start(self, name, cmd):
        out = open(os.path.join(self.outdir, f"{name}.out"), "wb")
        p = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        self.procs.append((name, p))
        return p

    def stage(self, stage, sim, outdir):
        return self.start(f"stage_{stage}", [
            "ros2", "launch", "mission_bringup", "mission_stage.launch.py",
            f"stage:={stage}", f"sim:={'true' if sim else 'false'}", "launch_camera:=true",
            "live_view:=true", "record_video:=false", f"log_dir:={outdir}"])

    def stop_all(self):
        for _, p in reversed(self.procs):
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        end = time.time() + 15.0
        for _, p in self.procs:
            try:
                p.wait(timeout=max(0.1, end - time.time()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


# ───────────────────────── main ─────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--duration", type=float, default=120.0, help="full-load test (s)")
    ap.add_argument("--fc", default="auto",
                    help="auto (MAVROS on /dev/ttyAMA2 if present) | none | an fcu_url")
    ap.add_argument("--sim", action="store_true",
                    help="laptop: simulated cameras (needs SITL + MAVROS running)")
    args = ap.parse_args()

    outdir = os.path.expanduser(time.strftime("~/aerothon_logs/pi_check_%Y%m%d_%H%M%S"))
    os.makedirs(outdir, exist_ok=True)
    rep = Report(os.path.join(outdir, "report.txt"))
    procs = Procs(outdir)
    if args.sim:                        # the corridor sim renders depth at 5 Hz
        MIN_HZ[OAK_RGB] = MIN_HZ[OAK_DEPTH] = 4.0
    rclpy.init()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    cpu = CpuSampler()
    on_pi = os.path.exists("/proc/device-tree/model")
    alone = {}

    try:
        # ── 1. system ──
        rep.line(f"pi_check {time.strftime('%Y-%m-%d %H:%M:%S')}  ->  {outdir}")
        rep.line("\n1. SYSTEM")
        model = read("/proc/device-tree/model").rstrip("\0") or sh("uname -m")
        rep.line(f"  model: {model}, {os.cpu_count()} cores, "
                 f"RAM available {mem_available_mb():.0f} MB, temp {temp_c()} C")
        th = throttled()
        if th is not None:
            rep.check(th & 0x1 == 0 and th & 0x10000 == 0, "no under-voltage since boot",
                      f"get_throttled=0x{th:x} ({throttle_text(th)})")
        v = ext5v()
        if v is not None:
            rep.check(v >= MIN_5V, f"5 V input {v:.2f} V at idle", f"need >= {MIN_5V} V")
        if on_pi:
            lim = sh("vcgencmd get_config usb_max_current_enable 2>/dev/null")
            rep.check("=1" in lim, "USB current limit lifted (usb_max_current_enable=1)",
                      f"{lim or '?'}: without a 5 A supply the Pi 5 limits ALL USB to 600 mA "
                      "— the OAK-D can brown out. Add usb_max_current_enable=1 to "
                      "/boot/firmware/config.txt only if the supply really gives 5 A",
                      warn_only=True)
        if not args.sim:
            found, speed = oak_usb()
            rep.check(found, "OAK-D on USB (vendor 03e7)")
            if found:
                rep.check(speed is not None and speed >= 5000, "OAK-D on USB3",
                          f"{speed:.0f} Mb/s" + ("" if speed and speed >= 5000 else
                                                 " — use a blue USB3 port and a USB3 cable"),
                          warn_only=True)
            cams = sh("rpicam-hello --list-cameras 2>&1 || libcamera-hello --list-cameras 2>&1",
                      timeout=15.0)
            rep.check("ov5647" in cams.lower(), "Pi Camera Module 1.3 (ov5647) listed",
                      cams.splitlines()[-1] if cams and "ov5647" not in cams.lower() else "")

        # FC telemetry for red_zone_node (nothing is commanded)
        fc = None if args.fc == "none" else \
            ("/dev/ttyAMA2:921600" if args.fc == "auto" and os.path.exists("/dev/ttyAMA2")
             else (None if args.fc == "auto" else args.fc))
        if fc and not args.sim:
            procs.start("mavros", ["ros2", "launch", "mavros", "apm.launch", f"fcu_url:={fc}"])
            rep.line(f"  MAVROS started on {fc} (telemetry for the red-zone node)")
        elif not args.sim:
            rep.line("  no FC: red_zone_node will not process frames — load below flight level")

        # ── 2. OAK-D ──
        rep.line("\n2. OAK-D (oak stage: driver + sector_builder + coverage_guard + live view)")
        procs.stage("oak", args.sim, outdir)
        ok = probe.wait_topic(OAK_DEPTH, 60.0)
        rep.check(ok, "OAK-D depth stream up", "" if ok else "nothing on " + OAK_DEPTH
                  + " in 60 s — see stage_oak.out")
        if ok:
            time.sleep(5.0)
            for t in (OAK_RGB, OAK_DEPTH):
                alone[t] = probe.rate(t, 5.0)
                rep.check(alone[t] >= MIN_HZ[t], f"{t} {alone[t]:.1f} Hz", f"need >= {MIN_HZ[t]}")
            probe.live.clear()
            time.sleep(5.0)
            frac = sum(probe.live) / len(probe.live) if probe.live else 0.0
            rep.check(frac >= 0.95, f"obstacle sensor live {100 * frac:.0f} % of the time",
                      "coverage_guard /avoidance/sensor_live")
            rep.check(snapshot(8081, os.path.join(outdir, "oak.jpg")),
                      "OAK-D live view http://<pi>:8081/ (snapshot oak.jpg)")
        cpu.sample()
        time.sleep(5.0)
        c, top = cpu.sample()
        rep.line(f"  CPU with the OAK-D: {c:.0f} % of all cores; top: "
                 + ", ".join(f"{n} {p:.0f}%" for n, p in top[:4]))

        # ── 3. Cam 1.3 ──
        rep.line("\n3. Pi Camera Module 1.3 (down_cam stage: camera_ros + live view)")
        procs.stage("down_cam", args.sim, outdir)
        ok = probe.wait_topic(DOWN, 45.0)
        rep.check(ok, "downward camera stream up", "" if ok else "nothing on /down_frame in "
                  "45 s — see stage_down_cam.out (ribbon cable? camera enabled?)")
        if ok:
            time.sleep(3.0)
            alone[DOWN] = probe.rate(DOWN, 5.0)
            rep.check(alone[DOWN] >= MIN_HZ[DOWN], f"{DOWN} {alone[DOWN]:.1f} Hz",
                      f"need >= {MIN_HZ[DOWN]}")
            rep.check(snapshot(8080, os.path.join(outdir, "down.jpg")),
                      "Cam 1.3 live view http://<pi>:8080/ (snapshot down.jpg)")
            for t in (OAK_RGB, OAK_DEPTH):          # OAK still fine with both cameras
                if t in alone:
                    r = probe.rate(t, 3.0)
                    rep.check(r >= LOAD_KEEP * alone[t], f"{t} with both cameras {r:.1f} Hz")

        # ── 4. full load ──
        rep.line(f"\n4. FULL LOAD for {args.duration:.0f} s: + red_zone_node + "
                 "corridor_perception + path_planner + qr_scanner_node scanning")
        procs.stage("arena", args.sim, outdir)
        procs.stage("exit_corridor", args.sim, outdir)
        procs.start("qr_scanner", ["ros2", "run", "qr_detection", "qr_scanner_node",
                                   "--ros-args", "--params-file", os.path.join(
                                       sh("ros2 pkg prefix mission_bringup"), "share",
                                       "mission_bringup", "config", "mission_params.yaml")])
        time.sleep(15.0)
        res = probe.call("/perception/start_qr_scan", 15.0)
        rep.line(f"  qr_scanner arena scan: {'ON' if res and res.success else 'NOT started'}")
        need = ["red_zone_node", "corridor_perception_node", "path_planner_node",
                "qr_scanner_node"]
        missing = [n for n in need if n not in probe.get_node_names()]
        rep.check(not missing, "all mission nodes up", "missing " + ", ".join(missing))

        cpus, mems, temps, v5s, tops = [], [], [], [], {}
        rates = {t: [] for t in alone}
        th0 = throttled()
        cpu.sample()
        t_end = time.time() + args.duration
        next_rate = time.time() + 10.0
        while time.time() < t_end:
            time.sleep(2.0)
            c, top = cpu.sample()
            if c is not None:
                cpus.append(c)
                for n, p in top:
                    tops[n] = max(tops.get(n, 0.0), p)
            mems.append(mem_available_mb())
            temps.append(temp_c() or 0.0)
            v = ext5v()
            if v is not None:
                v5s.append(v)
            if time.time() >= next_rate:            # a few rate windows through the test
                for t in rates:
                    rates[t].append(probe.rate(t, 3.0))
                next_rate = time.time() + 20.0
            left = t_end - time.time()
            print(f"\r  ... {left:4.0f} s left  CPU {c or 0:3.0f} %  RAM free "
                  f"{mems[-1]:5.0f} MB  {temps[-1]:4.1f} C", end="", flush=True)
        print()
        avg = sum(cpus) / len(cpus) if cpus else 0.0
        rep.check(avg <= MAX_CPU, f"CPU average {avg:.0f} %, peak {max(cpus or [0]):.0f} % "
                  f"of {os.cpu_count()} cores", f"need average <= {MAX_CPU:.0f} %")
        rep.check(min(mems) >= MIN_MEM_MB, f"RAM available never below {min(mems):.0f} MB",
                  f"need >= {MIN_MEM_MB:.0f} MB")
        if on_pi:
            rep.check(max(temps) <= MAX_TEMP_C, f"temperature peak {max(temps):.1f} C",
                      f"need <= {MAX_TEMP_C:.0f} C (fan / heatsink?)")
        if v5s:
            rep.check(min(v5s) >= MIN_5V, f"5 V input never below {min(v5s):.2f} V",
                      f"need >= {MIN_5V} V — supply / BEC too weak for the load")
        th1 = throttled()
        if th1 is not None:
            new = th1 & ~(th0 or 0)
            rep.check(th1 & 0xF == 0 and new & 0xF0000 == 0,
                      "no under-voltage / throttling during the load",
                      f"get_throttled=0x{th1:x} ({throttle_text(th1)})")
        for t, rs in rates.items():
            if rs:
                worst = min(rs)
                rep.check(worst >= LOAD_KEEP * alone[t] and worst >= MIN_HZ[t],
                          f"{t} under load {worst:.1f} Hz (alone {alone[t]:.1f})",
                          f"need >= {100 * LOAD_KEEP:.0f} % of alone and >= {MIN_HZ[t]}")
        dead = [n for n, p in procs.procs if p.poll() is not None]
        rep.check(not dead, "no launch died during the test",
                  "died: " + ", ".join(dead) + " (see their .out files)")
        rep.line("  heaviest processes (peak % of ONE core): "
                 + ", ".join(f"{n} {p:.0f}%" for n, p in
                             sorted(tops.items(), key=lambda kv: -kv[1])[:8]))
        if not fc and not args.sim:
            rep.line("  NOTE: no FC connected — red_zone_node idled; flight load is higher.")
    except KeyboardInterrupt:
        rep.line("\ninterrupted")
        rep.fails += 1
    finally:
        procs.stop_all()
        executor.shutdown(timeout_sec=2.0)     # stop spinning before tearing down
        spin.join(timeout=3.0)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    verdict = "ALL PASS" if rep.fails == 0 else f"{rep.fails} FAIL"
    rep.line(f"\nRESULT: {verdict}" + (f", {rep.warns} WARN" if rep.warns else "")
             + f"  (report + snapshots in {outdir})")
    sys.exit(0 if rep.fails == 0 else 1)


if __name__ == "__main__":
    main()
