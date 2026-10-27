# coverage_guard_node.py — Section 4.5 (Stage 4).
#
# Replaces the deleted corridor controller. The corridor is flown with waypoints
# + BendyRuler, so NOTHING in this package commands velocity. What the package
# still owes the mission is an honest answer to: is the forward sensor
# trustworthy right now? This node publishes Bools and commands the vehicle in
# NO way. The mission FSM (a different package) owns every mode and setpoint
# decision, so there is exactly one place in the system that can move the drone.
# It is NOT a speed clamp and MUST NOT publish setpoints.
#
# TWO independent signals, both at guard_rate_hz, both latched, both starting
# False:
#
#   /avoidance/sensor_live  (Bool) — HARD, PHASE-INDEPENDENT stop.
#       False if no /avoidance/scan arrived within depth_stale_timeout_s.
#       The mission FSM acts on this in EVERY phase: no scan means the pipeline
#       is dead and the vehicle must hold regardless of where it is.
#
#   /avoidance/coverage_ok  (Bool) — ADVISORY, corridor-only.
#       False if coverage < coverage_min_valid.
#       Only meaningful where returns are EXPECTED. In the open outdoor arena
#       low coverage and a mostly-empty scan are the NORMAL state (nothing is
#       within sensor range during the BCD search), so the FSM consults this
#       ONLY during the corridor leg. Acting on it in the arena would hold the
#       mission almost continuously.
#
# NOTE: there is deliberately NO "scan all-inf" condition. An all-inf scan is
# the normal state in open terrain, and a genuinely blind camera is already
# caught by coverage (which counts valid PIXELS, not obstacle bins). Testing
# all-inf here false-positived in the arena and would hold the search.
#
# Latch: once a signal goes False it stays False until its raw condition has
# been clear continuously for coverage_recover_hold_s. Without the latch a
# marginal wall produces a true/false stream and the mission FSM stutters.
#
# FSM timer note (implemented in the mission package, stated here so the two
# timers stay consistent): "sensor_live False for > 10 s -> escalate to
# failsafe" must only run during an ACTIVE FLIGHT LEG. This guard starts False
# by design and needs coverage_recover_hold_s of healthy data to clear, so an
# escalation timer started at boot would trip on the ground before arming.

import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from mavros_msgs.msg import State

# Minimum separation between the guard's stale cutoff and the builder's scan
# watchdog. 0.10 s is a single frame at 10 Hz — not enough; require 0.15 s.
GUARD_TIMEOUT_MARGIN_S = 0.15


def check_timeout_ordering(stale_s, watchdog_s, margin_s=GUARD_TIMEOUT_MARGIN_S):
    """Fail loudly if the guard's cutoff is not clearly ahead of the builder's.

    Raises ValueError unless
        depth_stale_timeout_s + margin_s <= depth_watchdog_timeout_s.
    Fix a violation by RAISING depth_watchdog_timeout_s, never by lowering the
    guard's stale timeout. Called at node startup so a bad config fails
    immediately, not at the first depth dropout in flight.
    """
    if stale_s + margin_s > watchdog_s:
        raise ValueError(
            f'timeout ordering violated: depth_stale_timeout_s ({stale_s}) + '
            f'{margin_s} = {stale_s + margin_s:.2f} > depth_watchdog_timeout_s '
            f'({watchdog_s}). Raise depth_watchdog_timeout_s to >= '
            f'{stale_s + margin_s:.2f}; do not lower the guard.')


class _Latch:
    """Fail-safe latched Boolean.

    Goes False the instant its raw condition fails; returns to True only after
    the raw condition has held continuously for hold_s. Starts False.
    """

    def __init__(self, hold_s):
        self.hold_s = float(hold_s)
        self.value = False
        self._clear_since = None

    def update(self, raw_ok, now_s):
        if not raw_ok:
            self._clear_since = None
            self.value = False
        elif not self.value:
            if self._clear_since is None:
                self._clear_since = now_s
            if now_s - self._clear_since >= self.hold_s:
                self.value = True
        return self.value


class CoverageGuardNode(Node):

    def __init__(self):
        super().__init__('coverage_guard_node')

        self.declare_parameter('guard_rate_hz', 10.0)
        self.declare_parameter('coverage_min_valid', 0.40)
        self.declare_parameter('depth_stale_timeout_s', 0.40)
        self.declare_parameter('depth_watchdog_timeout_s', 0.60)
        self.declare_parameter('coverage_recover_hold_s', 1.0)

        gp = self.get_parameter
        self.guard_rate_hz = float(gp('guard_rate_hz').value)
        self.coverage_min_valid = float(gp('coverage_min_valid').value)
        self.stale_timeout_s = float(gp('depth_stale_timeout_s').value)
        self.watchdog_timeout_s = float(gp('depth_watchdog_timeout_s').value)
        self.recover_hold_s = float(gp('coverage_recover_hold_s').value)

        # Ordering invariant (fails startup on a bad config): the guard's stale
        # cutoff must sit clearly ahead of the builder's scan watchdog.
        check_timeout_ordering(self.stale_timeout_s, self.watchdog_timeout_s)

        # --- inputs ---
        self._coverage = None       # latest /avoidance/coverage
        self._last_scan_s = None    # arrival time (node clock) of last scan
        self._state = None          # latest /mavros/state, for log context only

        # --- latches (fail-safe default False, independent recovery timers) ---
        self._live = _Latch(self.recover_hold_s)
        self._cov = _Latch(self.recover_hold_s)
        self._published_once = False

        # Reliable, low-rate command signals (not sensor data): the mission FSM
        # must not miss a transition.
        self.pub_live = self.create_publisher(
            Bool, '/avoidance/sensor_live', 10)
        self.pub_cov = self.create_publisher(
            Bool, '/avoidance/coverage_ok', 10)

        # /avoidance/scan is published RELIABLE (to match MAVROS), so subscribe
        # RELIABLE to align with the publisher. Coverage is still BEST_EFFORT
        # (sensor-style publisher) — a RELIABLE sub would get nothing from it
        # (Trap #7). /mavros/state RELIABLE offer satisfies either request.
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            LaserScan, '/avoidance/scan', self._on_scan, scan_qos)
        self.create_subscription(
            Float32, '/avoidance/coverage', self._on_cov,
            qos_profile_sensor_data)
        self.create_subscription(
            State, '/mavros/state', self._on_state, qos_profile_sensor_data)

        self.timer = self.create_timer(
            1.0 / self.guard_rate_hz, self._on_timer)

        self.get_logger().info(
            f'coverage_guard_node up @ {self.guard_rate_hz:.0f} Hz. '
            f'sensor_live: scan stale > {self.stale_timeout_s:.2f}s -> False '
            f'(hard, every phase). coverage_ok: coverage < '
            f'{self.coverage_min_valid:.2f} -> False (advisory, corridor only). '
            f'recover hold {self.recover_hold_s:.1f}s. Publishes Bools only.')

    # ------------------------------------------------------------ callbacks
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg):
        # Only arrival matters for liveness; scan contents are not inspected.
        self._last_scan_s = self._now()

    def _on_cov(self, msg):
        self._coverage = float(msg.data)

    def _on_state(self, msg):
        self._state = msg

    # -------------------------------------------------------------- logic
    def _on_timer(self):
        now = self._now()

        scan_fresh = (self._last_scan_s is not None and
                      (now - self._last_scan_s) <= self.stale_timeout_s)
        coverage_raw = (self._coverage is not None and
                        self._coverage >= self.coverage_min_valid)

        prev_live, prev_cov = self._live.value, self._cov.value
        live = self._live.update(scan_fresh, now)
        cov = self._cov.update(coverage_raw, now)

        self.pub_live.publish(Bool(data=live))
        self.pub_cov.publish(Bool(data=cov))

        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                f'guard active; initial sensor_live={live}, coverage_ok={cov}')

        if live != prev_live:
            self._log_live(live, now)
        if cov != prev_cov:
            self._log_cov(cov)

    # -------------------------------------------------------------- logging
    def _ctx(self):
        if self._state is not None:
            return f'mode={self._state.mode} armed={self._state.armed}'
        return 'mode=UNKNOWN'

    # Transition logs are machine-greppable tokens, not only prose: during
    # flight testing (Gates 4A-4C, 6C, 8C, 8D) these get grepped from ROS logs
    # against dataflash timestamps to confirm the right condition fired.
    #   GUARD sensor_live=False reason=SCAN_STALE age=0.43s limit=0.40s mode=...
    #   GUARD coverage_ok=False reason=LOW_COVERAGE value=0.12 limit=0.40 mode=...
    def _log_live(self, live, now):
        if not live:
            if self._last_scan_s is None:
                detail = 'reason=NO_SCAN'
            else:
                detail = (f'reason=SCAN_STALE age={now - self._last_scan_s:.2f}s '
                          f'limit={self.stale_timeout_s:.2f}s')
            self.get_logger().warn(
                f'GUARD sensor_live=False {detail} {self._ctx()}')
        else:
            self.get_logger().warn(
                f'GUARD sensor_live=True reason=RECOVERED '
                f'hold={self.recover_hold_s:.1f}s {self._ctx()}')

    def _log_cov(self, cov):
        if not cov:
            if self._coverage is None:
                detail = 'reason=NO_COVERAGE'
            else:
                detail = (f'reason=LOW_COVERAGE value={self._coverage:.2f} '
                          f'limit={self.coverage_min_valid:.2f}')
            self.get_logger().warn(
                f'GUARD coverage_ok=False {detail} {self._ctx()}')
        else:
            self.get_logger().warn(
                f'GUARD coverage_ok=True reason=RECOVERED '
                f'hold={self.recover_hold_s:.1f}s {self._ctx()}')


def main(args=None):
    rclpy.init(args=args)
    node = CoverageGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
