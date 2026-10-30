# payload_node.py — ROS2 node for the payload winch (thread on a spool)
#
# The payload (~100 g) hangs on a thread wound on a motor-driven spool. It is
# heavy enough to unwind the spool by itself, so the motor is kept spinning
# (reeling in, at full speed) for the whole flight to hold it up:
#
#   node start          -> winch_pwm_spin (retried until MAVROS accepts it)
#   /payload/release    -> winch_pwm_stop: the motor lets go, gravity unwinds
#                          the thread and the payload drops, for winch_unwind_s
#                       -> winch_pwm_spin again: pull the thread back in,
#                          winch_reel_wait_s before the reply (drone holds)
#   landed              -> winch_pwm_stop: disarmed after flying (above
#                          FLEW_ALT_M while armed), whoever landed it
#                          (mission, FC failsafe, pilot). Armed again -> spin.
#
# Hardware interface:
#   MAVLink MAV_CMD_DO_SET_SERVO via MAVROS /mavros/cmd/command.
#
# FC setup (do this in Mission Planner, the node cannot do it for you):
#   SERVO<release_channel>_FUNCTION = 0   (DO_SET_SERVO is ignored on an
#                                          output assigned to any function)
#   SERVO<release_channel>_TRIM     = winch_pwm_stop. The FC outputs trim on
#                                     unassigned channels from boot, so the
#                                     motor is still until this node starts.
#   Check that output <release_channel> physically exists on the Pixhawk 6C
#   Mini, or move release_channel to an output that does.
# Bench check: winch_pwm_spin must REEL IN (wrong way -> the other end of the
# PWM range), winch_pwm_stop must leave the spool free to turn.
#
# Usage:
#   ros2 run payload payload_node
#   ros2 run payload payload_node --ros-args -p release_channel:=10

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

MAV_CMD_DO_SET_SERVO = 183
SPIN_RETRY_S = 2.0              # start-up spin command retried this often
RETRY_FOR_S = 15.0              # reel-in / landing stop retried this long
FLEW_ALT_M = 1.0                # armed above this = a flight; its disarm = landed


class PayloadNode(Node):
    def __init__(self):
        super().__init__("payload_node")
        self.cfg = load_mission_config(self)

        self.channel      = int(self.cfg.release_channel)
        self.pwm_spin     = int(self.cfg.winch_pwm_spin)
        self.pwm_stop     = int(self.cfg.winch_pwm_stop)
        self.unwind_s     = float(self.cfg.winch_unwind_s)
        self.reel_wait_s  = float(self.cfg.winch_reel_wait_s)

        self.is_released = False
        self._lock = threading.Lock()   # one motor command sequence at a time
        self._winch_on = False
        self._armed = None              # None until the first /mavros/state
        self._flew = False

        # Service handler, timer and client share a reentrant group so the
        # client's response can be processed by another executor thread while
        # the handler waits (no nested spinning).
        self._cb_group = ReentrantCallbackGroup()
        self.servo_client = self.create_client(
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
            f"[payload] Ready — /payload/release (ch={self.channel}, "
            f"spin={self.pwm_spin}, stop={self.pwm_stop}, "
            f"unwind {self.unwind_s:.1f} s)")

    # -----------------------------------------------------------------------
    # Servo helper — sends MAVLink DO_SET_SERVO via MAVROS CommandLong
    # -----------------------------------------------------------------------
    def _set_servo(self, channel: int, pwm: int, wait_s: float = 5.0) -> bool:
        """param1 = servo output number, param2 = PWM (us)."""
        if not self.servo_client.wait_for_service(timeout_sec=wait_s):
            self.get_logger().warn("[payload] /mavros/cmd/command not available yet",
                                   throttle_duration_sec=10.0)
            return False

        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_DO_SET_SERVO
        req.param1 = float(channel)
        req.param2 = float(pwm)

        future = self.servo_client.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info(f"[payload] Servo ch={channel} set to PWM={pwm}")
            return True

        self.get_logger().error(f"[payload] Failed to set servo ch={channel} to PWM={pwm}")
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
            if self._set_servo(self.channel, self.pwm_spin):
                self._spin_timer.cancel()
                self._winch_on = True
                self.get_logger().info("[payload] Winch spinning — holding the payload")
        finally:
            self._lock.release()

    def _set_winch(self, on: bool) -> bool:
        """Spin or stop, retried for RETRY_FOR_S. Caller holds self._lock."""
        pwm = self.pwm_spin if on else self.pwm_stop
        deadline = time.time() + RETRY_FOR_S
        while time.time() < deadline:
            if self._set_servo(self.channel, pwm):
                self._winch_on = on
                return True
        return False

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
                    self._set_winch(True)
        elif self._flew:
            with self._lock:
                self._spin_timer.cancel()   # landed before the start-up spin took
                self.get_logger().info("[payload] Landed (disarmed) — winch off")
                if not self._set_winch(False):
                    self.get_logger().error("[payload] Winch NOT stopped after landing")

    # -----------------------------------------------------------------------
    # /payload/release — stop the winch, let the payload drop, reel back in
    # -----------------------------------------------------------------------
    def _handle_release(self, request, response):
        self.get_logger().info("[payload] RELEASE requested")

        with self._lock:
            self._spin_timer.cancel()

            if self.is_released:
                response.success = True
                response.message = "Payload already released"
                return response

            if not self._set_servo(self.channel, self.pwm_stop):
                # Retry the spin so the payload is not left unheld.
                self._set_winch(True)
                response.success = False
                response.message = "Failed to stop the winch"
                return response

            self.get_logger().info(f"[payload] Winch off — unwinding {self.unwind_s:.1f} s")
            self._winch_on = False
            time.sleep(self.unwind_s)   # gravity unwinds the thread, payload drops
            self.is_released = True

            # Reel the thread back in. Retried: a dangling thread on the way out
            # is worse than a late reply.
            reeling = self._set_winch(True)
            if reeling:
                time.sleep(self.reel_wait_s)

        response.success = True
        response.message = ("Payload released, thread reeled in" if reeling else
                            "Payload released, winch NOT restarted (thread hanging)")
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
