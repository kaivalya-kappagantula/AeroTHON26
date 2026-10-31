# phase_log.py — simple per-phase text logs for mission_node
#
# One folder per flight (log_dir, set by mission.launch.py:
# ~/aerothon_logs/mission_<time>/), one plain-text file per phase:
#
#   0_preflight.log  1_takeoff_qr.log  2_entry_corridor.log  3_arena_search.log
#   4_payload_drop.log  5_exit_corridor.log  6_home_land.log
#   summary.log      one line per phase: start, end, result, time taken
#   stage_<name>.out console output of each phase's nodes (crash tracebacks)
#
# Each phase file holds every node's log messages (/rosout: mission_node,
# path_planner, qr_scanner, red_zone, the FC's own STATUSTEXT through MAVROS,
# ...) while that phase runs, and once a second a TELEM line (FC mode,
# armed, altitude, position, distance to the phase's end waypoint, battery,
# obstacle sensor). Nothing is recorded between phases or after landing.
#
#   less ~/aerothon_logs/<flight>/3_arena_search.log
#   grep -h "WARN\|ERROR" ~/aerothon_logs/<flight>/*.log

import os
import threading
import time

from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mission_common import mavros_utils as mav

LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def _stamp(t=None):
    t = time.time() if t is None else t
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t * 10) % 10}"


class PhaseLog:
    def __init__(self, node, log_dir: str):
        self.node = node
        self.dir = os.path.expanduser(log_dir) if log_dir else ""
        self._lock = threading.Lock()
        self._file = None
        self._phase = None              # (label, t0, end waypoint (lat, lon) or None)
        self._summary = None
        if not self.dir:
            return
        os.makedirs(self.dir, exist_ok=True)
        self._summary = open(os.path.join(self.dir, "summary.log"), "a", buffering=1)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=200)
        node.create_subscription(Log, "/rosout", self._rosout_cb, qos)
        node.create_timer(1.0, self._telemetry)
        node.get_logger().info(f"[mission_node] Phase logs in {self.dir}")

    def _write(self, line: str):
        with self._lock:
            if self._file is not None:
                self._file.write(line + "\n")

    def _rosout_cb(self, msg: Log):
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self._write(f"{_stamp(t)} {LEVELS.get(msg.level, msg.level):5s} {msg.name}: {msg.msg}")

    def _telemetry(self):
        if self._phase is None:
            return
        n = self.node
        st = n.mavros_state
        gps, alt = n.gps, n.rel_alt
        to_end = ""
        end = self._phase[2]
        if gps is not None and end is not None:
            to_end = f" to_end={mav.haversine(*gps, *end):.1f}m"
        self._write(
            f"{_stamp()} TELEM mode={st.mode if st else '?'} armed={int(bool(st and st.armed))}"
            f" alt={'?' if alt is None else f'{alt:.2f}'}"
            f" pos={'?' if gps is None else f'{gps[0]:.7f},{gps[1]:.7f}'}{to_end}"
            f" hdg={'?' if n.heading_deg is None else f'{n.heading_deg:.0f}'}"
            f" batt={'?' if not n.battery_voltage else f'{n.battery_voltage:.2f}V'}"
            f" oak={int(bool(n._sensor_live))}")

    def begin(self, number: int, name: str, start=None, end=None):
        """Close the running phase's file and open this one's. start / end:
        the phase's begin and end waypoints (lat, lon), logged and used for
        the TELEM distance; None = where the drone is / decided in flight."""
        self.end("next phase started")
        where = lambda p: "here" if p is None else f"({p[0]:.7f}, {p[1]:.7f})"  # noqa: E731
        head = f"PHASE {number} {name}: from {where(start)} to {where(end)}"
        self.node.get_logger().info(f"[mission_node] ===== {head} =====")
        if not self.dir:
            self._phase = (f"{number} {name}", time.time(), end)
            return
        path = os.path.join(self.dir, f"{number}_{name.replace(' ', '_')}.log")
        with self._lock:
            self._file = open(path, "a", buffering=1)
            self._file.write(f"{_stamp()} ===== {head} =====\n")
        self._phase = (f"{number} {name}", time.time(), end)
        self._summary.write(f"{_stamp()} START {number} {name}  ({where(start)} -> {where(end)})\n")

    def end(self, result: str):
        """Close the running phase (if any) with its result."""
        if self._phase is None:
            return
        label, t0, _ = self._phase
        self._phase = None
        took = time.time() - t0
        self.node.get_logger().info(
            f"[mission_node] ===== PHASE {label} END: {result} ({took:.0f} s) =====")
        if not self.dir:
            return
        with self._lock:
            if self._file is not None:
                self._file.write(f"{_stamp()} ===== END: {result} ({took:.0f} s) =====\n")
                self._file.close()
                self._file = None
        self._summary.write(f"{_stamp()} END   {label}: {result} ({took:.0f} s)\n")
