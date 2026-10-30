# payload_node.py — ROS2 node for the payload winch (thread on a spool)
#
# The payload (~100 g) hangs on a 5 m thread wound on a motor-driven spool.
# It is heavy enough to unwind the spool by itself, so the motor is kept
# reeling in (full speed) for the whole flight to hold it up:
#
#   node start          -> IN (retried until MAVROS accepts it)
#   /payload/release    -> OUT: lower the payload on the thread for
#                          winch_unwind_s (time it for ~4.7 m of the 5 m:
#                          too long and the spool runs off the end and winds
#                          the thread back on the other way)
#                       -> IN at once: the jerk throws the payload off its
#                          hook; winch_jerks > 1 repeats OUT winch_jerk_s / IN
#                       -> stays IN: reels the thread up, until landing
#   landed              -> OFF: disarmed after flying (above FLEW_ALT_M while
#                          armed), whoever landed it (mission, FC failsafe,
#                          pilot). Armed again -> IN.
#
# Hardware interface: the motor driver's two direction inputs are on FC
# relays (on/off, so no speed setting: on = full speed), switched with
# MAV_CMD_DO_SET_RELAY via MAVROS /mavros/cmd/command. Relay numbers are the
# command's, 0-based: 0 = RELAY1, 1 = RELAY2.
#   IN  = winch_relay_in ON,  winch_relay_out OFF   (reel up / hold)
#   OUT = winch_relay_out ON, winch_relay_in OFF    (lower)
#   OFF = both OFF (never both ON)
# Spins the wrong way on the bench -> swap winch_relay_in / winch_relay_out.
#
# FC setup (do this in Mission Planner, the node cannot do it for you):
#   RELAY1_PIN 50, RELAY2_PIN 51 (AUX1 / AUX2 = SERVO9 / SERVO10)
#   SERVO9_FUNCTION, SERVO10_FUNCTION = -1 (GPIO), or the pins stay PWM outputs
#   RELAY1_FUNCTION, RELAY2_FUNCTION = 1 (Relay); DO_SET_RELAY is ignored
#                                       otherwise (ArduPilot 4.5+)
#   RELAY1_DEFAULT, RELAY2_DEFAULT = 0 (off at boot: motor still until this
#                                       node starts)
#
# Usage:
#   ros2 run payload payload_node

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong

from mission_common.config_loader import load_mission_config

MAV_CMD_DO_SET_RELAY = 181
IN, OUT, OFF = "IN", "OUT", "OFF"   # winch modes
SPIN_RETRY_S = 2.0              # start-up spin command retried this often
RETRY_FOR_S = 15.0              # reel-in / landing stop retried this long
FLEW_ALT_M = 1.0                # armed above this = a flight; its disarm = landed


class PayloadNode(Node):
    def __init__(self):
        super().__init__("payload_node")
        self.cfg = load_mission_config(self)

        self.relay_in   = int(self.cfg.winch_relay_in)
        self.relay_out  = int(self.cfg.winch_relay_out)
        self.unwind_s   = float(self.cfg.winch_unwind_s)
        self.jerks      = max(1, int(self.cfg.winch_jerks))
        self.jerk_s     = float(self.cfg.winch_jerk_s)

        self.is_released = False
        self._lock = threading.Lock()   # one motor command sequence at a time
        self._winch_on = False
        self._armed = None              # None until the first /mavros/state
        self._flew = False

        # Service handler, timer and client share a reentrant group so the
        # client's response can be processed by another executor thread while
        # the handler waits (no nested spinning).
        self._cb_group = ReentrantCallbackGroup()
        self.cmd_client = self.create_client(
            CommandLong, "/mavros/cmd/command", callback_group=self._cb_group)
        self.create_service(Trigger, "/payload/release", self._handle_release,
                            callback_group=self._cb_group)
        # Spin from the start: the payload would unwind the spool otherwise.
        self._spin_timer = self.create_timer(SPIN_RETRY_S, self._start_spin,
                                             callback_group=self._cb_group)

        # MAVROS publishes BEST_EFFORT; a RELIABLE subscriber gets nothing.
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(State, "/mavros/state", self._state_cb, mavros_qos,
                                 callback_group=self._cb_group)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                 self._rel_alt_cb, mavros_qos,
                                 callback_group=self._cb_group)

        self.get_logger().info(
            f"[payload] Ready — /payload/release (reel-in relay {self.relay_in}, "
            f"out relay {self.relay_out}, unwind {self.unwind_s:.1f} s, "
            f"{self.jerks} jerk(s))")

    # -----------------------------------------------------------------------
    # Relay helper — sends MAVLink DO_SET_RELAY via MAVROS CommandLong
    # -----------------------------------------------------------------------
    def _set_relay(self, relay: int, on: bool, wait_s: float = 5.0) -> bool:
        """param1 = relay instance (0 = RELAY1), param2 = 1 on / 0 off."""
        if not self.cmd_client.wait_for_service(timeout_sec=wait_s):
            self.get_logger().warn("[payload] /mavros/cmd/command not available yet",
                                   throttle_duration_sec=10.0)
            return False

        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_DO_SET_RELAY
        req.param1 = float(relay)
        req.param2 = 1.0 if on else 0.0

        future = self.cmd_client.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        state = "ON" if on else "OFF"
        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info(f"[payload] Relay {relay} {state}")
            return True

        self.get_logger().error(f"[payload] Failed to set relay {relay} {state}")
        return False

    def _apply(self, mode: str) -> bool:
        """One attempt at IN, OUT or OFF. The relay being switched off goes
        first, so both are never on together."""
        if mode == IN:
            ok = (self._set_relay(self.relay_out, False)
                  and self._set_relay(self.relay_in, True))
        elif mode == OUT:
            ok = (self._set_relay(self.relay_in, False)
                  and self._set_relay(self.relay_out, True))
        else:
            ok = (self._set_relay(self.relay_in, False)
                  and self._set_relay(self.relay_out, False))
        if ok:
            self._winch_on = mode == IN
        return ok

    def _set_winch(self, mode: str) -> bool:
        """_apply, retried for RETRY_FOR_S. Caller holds self._lock."""
        deadline = time.time() + RETRY_FOR_S
        while time.time() < deadline:
            if self._apply(mode):
                return True
        return False

    # -----------------------------------------------------------------------
    # Start-up: winch on, full speed, until the FC accepts it
    # -----------------------------------------------------------------------
    def _start_spin(self):
        if not self._lock.acquire(blocking=False):
            return                      # a release is running, it owns the motor
        try:
            if self._spin_timer.is_canceled():
                return
            if self._apply(IN):
                self._spin_timer.cancel()
                self.get_logger().info("[payload] Winch spinning — holding the payload")
        finally:
            self._lock.release()

    # -----------------------------------------------------------------------
    # Landing: winch off once disarmed after a flight; on again when re-armed
    # -----------------------------------------------------------------------
    def _rel_alt_cb(self, msg):
        if self._armed and msg.data > FLEW_ALT_M:
            self._flew = True

    def _state_cb(self, msg):
        was, self._armed = self._armed, bool(msg.armed)
        if was is None or was == self._armed:
            return
        if self._armed:
            self._flew = False
            with self._lock:
                if not self._winch_on and self._spin_timer.is_canceled():
                    self.get_logger().info("[payload] Armed — winch on")
                    self._set_winch(IN)
        elif self._flew:
            with self._lock:
                self._spin_timer.cancel()   # landed before the start-up spin took
                self.get_logger().info("[payload] Landed (disarmed) — winch off")
                if not self._set_winch(OFF):
                    self.get_logger().error("[payload] Winch NOT stopped after landing")

    # -----------------------------------------------------------------------
    # /payload/release — lower the payload, jerk it off, reel the thread up
    # -----------------------------------------------------------------------
    def _handle_release(self, request, response):
        self.get_logger().info("[payload] RELEASE requested")

        with self._lock:
            self._spin_timer.cancel()

            if self.is_released:
                response.success = True
                response.message = "Payload already released"
                return response

            if not self._apply(OUT):
                # Back to holding so the payload is not left unheld.
                self._set_winch(IN)
                response.success = False
                response.message = "Failed to reverse the winch"
                return response

            self.get_logger().info(f"[payload] Winch OUT — lowering {self.unwind_s:.1f} s")
            time.sleep(self.unwind_s)

            # The jerk: straight back to IN, no pause. Further jerks: a short
            # OUT, then IN again.
            for i in range(self.jerks):
                if i > 0:
                    self._apply(OUT)
                    time.sleep(self.jerk_s)
                self.get_logger().info(f"[payload] Jerk {i + 1}/{self.jerks} — winch IN")
                self._apply(IN)
            self.is_released = True

            # Reel the thread up (stays IN until landing). Retried: a dangling
            # thread on the way out is worse than a late reply.
            reeling = self._winch_on or self._set_winch(IN)

        response.success = True
        response.message = ("Payload jerked off, winch reeling in" if reeling else
                            "Payload lowered, winch NOT reeling in (thread hanging)")
        self.get_logger().info(f"[payload] {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PayloadNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[payload] Shutting down")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
