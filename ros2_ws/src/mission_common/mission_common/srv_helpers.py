# srv_helpers.py — ROS2 service-call wrappers for path_planner, qr_detection, payload
#
# All services are std_srvs/Trigger. Leg parameters (speed, corridor geometry)
# live in the path_planner's own config, selected by WHICH service is called.
#
# THREADING CONTRACT: call these from the mission worker thread only, never
# from inside a ROS callback. They sleep-poll the response future, which only
# completes while another thread spins the node.
#
# Usage:
#   from mission_common.srv_helpers import call_trigger
#   res = call_trigger(node, "/path_planner/fly_corridor_1",
#                      call_timeout=130.0, should_abort=lambda: False)

import time

from rclpy.node import Node
from std_srvs.srv import Trigger

# Service names (one source of truth for mission_node and tests)
SRV_SCAN_QR = "/perception/scan_qr"
SRV_START_QR_SCAN = "/perception/start_qr_scan"
SRV_FLY_CORRIDOR_1 = "/path_planner/fly_corridor_1"
SRV_FLY_CORRIDOR_2 = "/path_planner/fly_corridor_2"
SRV_FLY_CORRIDOR_HERE = "/path_planner/fly_corridor_here"
SRV_ENTRY_CORRIDOR = "/path_planner/entry_corridor"
SRV_EXIT_CORRIDOR = "/path_planner/exit_corridor"
SRV_RUN_LAWNMOWER = "/path_planner/run_lawnmower"
SRV_PLANNER_STOP = "/path_planner/stop"
SRV_PAYLOAD_RELEASE = "/payload/release"


class CallAborted(Exception):
    """Raised by call_trigger when should_abort() turns True mid-call."""


def call_trigger(node: Node, srv_name: str, call_timeout: float,
                 wait_timeout: float = 10.0, should_abort=None):
    """Call a Trigger service and return its response, or None on failure.

    wait_timeout : how long to wait for the server to appear.
    call_timeout : how long to wait for the RESPONSE. For services that block
                   until a flight leg completes this must cover the whole leg,
                   not a generic 10 s — otherwise the mission moves on while
                   the vehicle is still flying the previous leg.
    should_abort : optional callable polled while waiting; when it returns
                   True the wait stops and CallAborted is raised (the request
                   is already in flight; the caller is responsible for
                   cancelling it, e.g. via /path_planner/stop).
    """
    client = node.create_client(Trigger, srv_name)
    try:
        if not client.wait_for_service(timeout_sec=wait_timeout):
            node.get_logger().error(
                f"[srv_helpers] Service {srv_name} not available after {wait_timeout}s")
            return None

        future = client.call_async(Trigger.Request())
        deadline = time.time() + call_timeout
        while not future.done():
            if should_abort is not None and should_abort():
                raise CallAborted(srv_name)
            if time.time() > deadline:
                node.get_logger().error(
                    f"[srv_helpers] {srv_name} gave no response within {call_timeout:.0f}s")
                return None
            time.sleep(0.05)

        result = future.result()
        if result is None:
            node.get_logger().error(f"[srv_helpers] Call to {srv_name} returned no result")
            return None
        node.get_logger().info(
            f"[srv_helpers] {srv_name} -> success={result.success}, "
            f"message='{result.message}'")
        return result
    finally:
        node.destroy_client(client)
