#!/usr/bin/env python3
# flight_primitives.py — Phase 1: flight primitives from our own code.
#
# One standalone node that commands the drone through the four flight verbs —
# no FSM, no perception. It exposes:
#
#     arm()          -> set GUIDED + arm the vehicle
#     takeoff(alt)   -> guided takeoff to `alt` metres, then hold
#     goto(x, y, z)  -> fly to local ENU offset (x East, y North) at altitude z
#     land()         -> land and confirm disarm
#
# and a Gate-1 self-test that runs  arm -> takeoff -> goto A -> goto B -> land
# `reps` times (default 5) in a row, with zero manual intervention. Any timeout
# aborts the run and reports which rep/step failed — flakiness is a failure.
#
# Waypoints use the *fixed* mavros_utils.goto helper (persistent streaming
# publisher), which replaces the raw GlobalPositionTarget burst that used to
# time out.
#
# Usage:
#   ros2 run mission_manager flight_primitives
#   ros2 run mission_manager flight_primitives --ros-args -p reps:=1
#   ros2 run mission_manager flight_primitives --ros-args -p altitude:=6.0 \
#       -p wp_a:="[10.0, 0.0]" -p wp_b:="[10.0, 10.0]"

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

from mission_common import mavros_utils as mav


class FlightPrimitives(Node):
    def __init__(self):
        super().__init__("flight_primitives")

        # ── Parameters ────────────────────────────────────────────────────
        self.reps = int(self.declare_parameter("reps", 5).value)
        self.altitude = float(self.declare_parameter("altitude", 5.0).value)
        wp_a = list(self.declare_parameter("wp_a", [15.0, 0.0]).value)
        wp_b = list(self.declare_parameter("wp_b", [15.0, 15.0]).value)
        self.wp_a = (float(wp_a[0]), float(wp_a[1]))
        self.wp_b = (float(wp_b[0]), float(wp_b[1]))
        self.arrival_tol = float(self.declare_parameter("arrival_tolerance_m", 2.0).value)
        self.arrival_timeout = float(self.declare_parameter("arrival_timeout_s", 60.0).value)
        self.altitude_tol = float(self.declare_parameter("altitude_tolerance_m", 0.5).value)
        self.altitude_timeout = float(self.declare_parameter("altitude_timeout_s", 40.0).value)
        self.land_timeout = float(self.declare_parameter("land_timeout_s", 60.0).value)
        self.goto_stream_count = int(self.declare_parameter("goto_stream_count", 20).value)

        # ── State (updated by subscription callbacks) ────────────────────
        self._state: State | None = None
        self._gps: tuple[float, float] | None = None
        self._alt: float | None = None
        self._home: tuple[float, float] | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(State, "/mavros/state", self._state_cb, qos)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._gps_cb, qos)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, qos)

        # Run the Gate-1 sequence off-thread; main thread spins the node so
        # subscription callbacks and service futures keep flowing.
        self._result: bool | None = None
        threading.Thread(target=self._run_gate, daemon=True).start()

    # ── Callbacks ────────────────────────────────────────────────────────
    def _state_cb(self, msg: State):
        self._state = msg

    def _gps_cb(self, msg: NavSatFix):
        self._gps = (msg.latitude, msg.longitude)

    def _pose_cb(self, msg: PoseStamped):
        self._alt = msg.pose.position.z

    # ── Internal blocking waiters (sleep-poll; main thread does the spin) ──
    def _sleep(self, secs: float):
        """Interruptible sleep that bails out on shutdown."""
        end = time.time() + secs
        while time.time() < end:
            if not rclpy.ok():
                raise KeyboardInterrupt
            time.sleep(min(0.1, end - time.time()))

    def _wait_ready(self, timeout: float = 60.0) -> bool:
        """Block until FCU is connected and in STANDBY (pre-arm passed)."""
        self.get_logger().info("Waiting for FCU ready (connected + standby)...")
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            s = self._state
            # system_status 3 == MAV_STATE_STANDBY (ready to arm)
            if s is not None and s.connected and s.system_status == 3:
                return True
            time.sleep(0.25)
        self.get_logger().error("FCU not ready (timeout)")
        return False

    def _wait_altitude(self, target: float) -> bool:
        deadline = time.time() + self.altitude_timeout
        while time.time() < deadline and rclpy.ok():
            if self._alt is not None and abs(self._alt - target) <= self.altitude_tol:
                self.get_logger().info(f"  reached altitude {self._alt:.2f} m")
                return True
            time.sleep(0.2)
        cur = f"{self._alt:.2f}" if self._alt is not None else "n/a"
        self.get_logger().error(
            f"  ALTITUDE TIMEOUT (at {cur} m, target {target:.1f} m)")
        return False

    def _wait_arrival(self, lat: float, lon: float) -> bool:
        deadline = time.time() + self.arrival_timeout
        while time.time() < deadline and rclpy.ok():
            if self._gps is not None:
                d = mav.haversine(self._gps[0], self._gps[1], lat, lon)
                if d <= self.arrival_tol:
                    self.get_logger().info(f"  arrived ({d:.2f} m from target)")
                    return True
            time.sleep(0.25)
        self.get_logger().error("  ARRIVAL TIMEOUT")
        return False

    def _wait_disarm(self) -> bool:
        deadline = time.time() + self.land_timeout
        while time.time() < deadline and rclpy.ok():
            if self._state is not None and not self._state.armed:
                self.get_logger().info("  disarmed (touchdown confirmed)")
                return True
            time.sleep(0.25)
        self.get_logger().error("  LAND/DISARM TIMEOUT (still armed)")
        return False

    # ── Flight verbs ─────────────────────────────────────────────────────
    def arm(self, timeout: float = 90.0) -> bool:
        """Set GUIDED mode and arm the vehicle.

        Patient by design: a freshly booted FCU transiently rejects arming with
        'Gyros inconsistent' / 'Need Position Estimate' / 'waiting for home'
        while the EKF converges (~30-60 s). We retry until it takes. Only the
        first rep ever pays this — later reps start already settled.
        """
        if not self._wait_ready():
            return False
        if not mav.set_mode(self, "GUIDED"):
            return False
        self._sleep(1.0)
        deadline = time.time() + timeout
        attempt = 0
        while time.time() < deadline and rclpy.ok():
            attempt += 1
            if mav.arm(self):
                # Confirm the FCU actually reports armed.
                t = time.time() + 5.0
                while time.time() < t and rclpy.ok():
                    if self._state is not None and self._state.armed:
                        return True
                    time.sleep(0.1)
            self.get_logger().warn(
                f"  arm attempt {attempt} not confirmed (FCU settling), retrying")
            self._sleep(2.0)
        self.get_logger().error("  ARM FAILED (timeout)")
        return False

    def takeoff(self, alt: float) -> bool:
        """Guided takeoff to `alt` metres, then hold."""
        if not mav.takeoff(self, alt):
            return False
        if not self._wait_altitude(alt):
            return False
        self._sleep(2.0)  # hold / settle
        return True

    def goto(self, x: float, y: float, z: float) -> bool:
        """Fly to local ENU offset (x East, y North) at altitude z metres."""
        if self._home is None:
            self.get_logger().error("  goto called before home fix acquired")
            return False
        lat, lon = mav.offset_gps(self._home[0], self._home[1], y, x)
        self.get_logger().info(
            f"  goto local=({x:.1f},{y:.1f},{z:.1f}) -> ({lat:.7f},{lon:.7f})")
        mav.goto(self, lat, lon, z, publish_count=self.goto_stream_count)
        return self._wait_arrival(lat, lon)

    def land(self) -> bool:
        """Land and confirm disarm."""
        if not mav.land(self):
            return False
        if not self._wait_disarm():
            # Best-effort explicit disarm if auto-disarm didn't fire.
            mav.disarm(self)
            return False
        return True

    def _safe_disarm_land(self):
        """Best-effort cleanup after a failed step: bring the vehicle down so it
        is not stranded armed/airborne (which would leave system_status ACTIVE
        and break the next run's readiness check). LAND first, then disarm."""
        try:
            if self._state is None or not self._state.armed:
                return
            self.get_logger().warn("cleanup: LAND + disarm to recover ground state")
            mav.set_mode(self, "LAND")
            t = time.time() + 45.0
            while time.time() < t and rclpy.ok():
                if self._state is not None and not self._state.armed:
                    self.get_logger().info("cleanup: disarmed")
                    return
                time.sleep(0.5)
            mav.disarm(self)  # force-disarm fallback
        except Exception as e:  # never let cleanup mask the original failure
            self.get_logger().warn(f"cleanup error: {e}")

    # ── Gate 1 sequence ──────────────────────────────────────────────────
    def _acquire_home(self, timeout: float = 30.0) -> bool:
        self.get_logger().info("Acquiring home GPS fix...")
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            if self._gps is not None:
                self._home = self._gps
                self.get_logger().info(
                    f"Home fix: lat={self._home[0]:.7f}, lon={self._home[1]:.7f}")
                return True
            time.sleep(0.25)
        self.get_logger().error("No home GPS fix (timeout)")
        return False

    def _run_gate(self):
        try:
            if not self._wait_ready():
                self._finish(False)
                return
            # --no-mavproxy: we must request our own data streams.
            mav.request_message_intervals(self, (33, 32))  # GLOBAL_POSITION_INT, LOCAL_POSITION_NED
            if not self._acquire_home():
                self._finish(False)
                return

            ax, ay = self.wp_a
            bx, by = self.wp_b
            alt = self.altitude

            for rep in range(1, self.reps + 1):
                self.get_logger().info(
                    f"═══ REP {rep}/{self.reps}: arm → takeoff → A → B → land ═══")
                steps = [
                    ("arm",         lambda: self.arm()),
                    ("takeoff",     lambda: self.takeoff(alt)),
                    ("goto A",      lambda: self.goto(ax, ay, alt)),
                    ("goto B",      lambda: self.goto(bx, by, alt)),
                    ("land",        lambda: self.land()),
                ]
                for name, fn in steps:
                    self.get_logger().info(f"[rep {rep}] {name} ...")
                    if not fn():
                        self.get_logger().error(
                            f"✗ GATE 1 FAILED at rep {rep}, step '{name}'")
                        self._safe_disarm_land()
                        self._finish(False)
                        return
                self.get_logger().info(f"✓ rep {rep}/{self.reps} complete")
                self._sleep(2.0)  # settle between reps

            self.get_logger().info(
                f"✓✓✓ GATE 1 PASSED: {self.reps}/{self.reps} reps, zero timeouts")
            self._finish(True)
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted")
            self._finish(False)

    def _finish(self, ok: bool):
        # Signal main(); it polls this and stops spinning. We deliberately do
        # NOT call rclpy.shutdown() from this worker thread — tearing down the
        # context under the spinning main thread can raise.
        self._result = ok


def main():
    rclpy.init()
    node = FlightPrimitives()
    try:
        # Spin until the mission thread reports a result.
        while rclpy.ok() and node._result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        result = node._result
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    # Non-zero exit on failure so scripts/CI can detect flakiness.
    if result is False:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
