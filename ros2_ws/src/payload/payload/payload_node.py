# payload_node.py — ROS2 node for payload delivery mechanism
#
# Provides one Trigger service consumed by the mission_manager:
#   /payload/release       — release the payload from the mechanism
#
# Hardware interface:
#   MAVLink MAV_CMD_DO_SET_SERVO via MAVROS /mavros/cmd/command.
#
# FC setup (do this in Mission Planner, the node cannot do it for you):
#   SERVO<release_channel>_FUNCTION = 0   (DO_SET_SERVO is ignored on an
#                                          output assigned to any function)
#   SERVO<release_channel>_TRIM     = the LOCK PWM. The FC outputs trim on
#                                     unassigned channels from boot, so it
#                                     holds the mechanism locked; this node
#                                     never sends a lock command.
#   Check that output <release_channel> physically exists on the Pixhawk 6C
#   Mini, or move release_channel to an output that does.
#
# Usage:
#   ros2 run payload payload_node
#   ros2 run payload payload_node --ros-args -p release_channel:=10

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandLong

from mission_common.config_loader import load_mission_config

MAV_CMD_DO_SET_SERVO = 183


class PayloadNode(Node):
    def __init__(self):
        super().__init__("payload_node")
        self.cfg = load_mission_config(self)

        self.release_channel  = int(self.cfg.release_channel)
        self.release_pwm_open = int(self.cfg.release_pwm_open)
        self.release_pause    = float(self.cfg.release_pause_s)

        self.is_released = False

        # Service handler and client share a reentrant group so the client's
        # response can be processed by another executor thread while the
        # handler waits (no nested spinning).
        self._cb_group = ReentrantCallbackGroup()
        self.servo_client = self.create_client(
            CommandLong, "/mavros/cmd/command", callback_group=self._cb_group)
        self.create_service(Trigger, "/payload/release", self._handle_release,
                            callback_group=self._cb_group)

        self.get_logger().info(
            f"[payload] Ready — /payload/release (ch={self.release_channel}, "
            f"open={self.release_pwm_open})")

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
    # /payload/release — detach the payload from the mechanism
    # -----------------------------------------------------------------------
    def _handle_release(self, request, response):
        self.get_logger().info("[payload] RELEASE requested")

        if self.is_released:
            response.success = True
            response.message = "Payload already released"
            return response

        if not self._set_servo(self.release_channel, self.release_pwm_open):
            response.success = False
            response.message = "Failed to activate release servo"
            return response

        time.sleep(self.release_pause)   # let the mechanism fully open

        self.is_released = True
        response.success = True
        response.message = "Payload released"
        self.get_logger().info("[payload] Payload released successfully")
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
