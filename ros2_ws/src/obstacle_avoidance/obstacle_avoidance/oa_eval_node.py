# oa_eval_node.py — SITL only. Flies ONE corridor pass with the chosen
# planner and scores it with oa_eval.Judge against sitl_corridor_sim's world.
# Evaluation harness for "is BendyRuler better than path_planner?"
# (docs/OA_EVALUATION.md); it changes nothing in either planner.
#
#   planner:=bendy    ArduPilot BendyRuler. The FC must have been started with
#                     params/bendyruler_eval.param (OA_TYPE 1, GUID_OPTIONS 64
#                     so GUIDED position targets go through the OA-aware
#                     waypoint navigator). This node only takes off, flies to
#                     the corridor's first waypoint and then sends ONE target:
#                     the last waypoint. Everything in between is ArduPilot.
#   planner:=current  path_planner_node's /path_planner/fly_corridor_2 (the
#                     mission's own corridor leg, corridor_follow.py). The FC
#                     runs params/stage5_6_corridor.param.
#
# Both see the same depth: sitl_corridor_sim -> sector_builder ->
# obstacle_publisher -> OBSTACLE_DISTANCE (BendyRuler), and -> corridor_perception
# (path_planner). Truth is the SITL position in the sim's corridor frame, with
# the sim's constant gps_offset_m (gps_drift_m must be 0: its phase is the
# sim's, not ours).
#
# Result: one JSON line appended to result_file (verdict + metrics + run
# tags), and, if trace_file is set, the trajectory as CSV (t, x, y, z).
# Then it lands and exits.

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from mission_common import mavros_utils as mav
from obstacle_avoidance.oa_eval import Course, EvalParams, Judge, boxes_from_flat

_DA = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
M_PER_DEG = 111_320.0


class OaEvalNode(Node):
    def __init__(self):
        super().__init__('oa_eval_node')
        dp = self.declare_parameter
        dp('planner', 'bendy')
        dp('origin_lat', 12.9716)
        dp('origin_lon', 77.5946)
        dp('corridor_waypoints_x', [0.0, 0.0], _DA)
        dp('corridor_waypoints_y', [2.0, 12.2], _DA)
        dp('corridor_width_m', 3.5052)
        dp('corridor_length_m', 6.7056)
        dp('corridor_wall_height_m', 2.4384)
        dp('mouth_offset_m', 2.5)
        dp('obstacles', [0.0], _DA)
        dp('obstacle_depth_m', 0.4)
        dp('gps_offset_m', 0.0)
        dp('altitude_m', 1.5)
        dp('drone_width_m', 0.55)
        dp('body_below_m', 0.15)
        dp('require_rejoin', True)
        dp('stall_s', 8.0)
        dp('timeout_s', 120.0)
        dp('result_file', '/tmp/oa_eval.jsonl')
        dp('trace_file', '')
        dp('tags', '{}')                  # JSON: scenario, spacing, dropout, ...
        gp = lambda n: self.get_parameter(n).value          # noqa: E731

        self.planner = gp('planner')
        if self.planner not in ('bendy', 'current'):
            raise ValueError(f"planner must be bendy or current, not {self.planner!r}")
        lat0, lon0 = float(gp('origin_lat')), float(gp('origin_lon'))

        def to_gps(x, y):
            return (lat0 + y / M_PER_DEG,
                    lon0 + x / (M_PER_DEG * math.cos(math.radians(lat0))))

        xs, ys = gp('corridor_waypoints_x'), gp('corridor_waypoints_y')
        self.entry, self.exit = to_gps(xs[0], ys[0]), to_gps(xs[-1], ys[-1])
        d_n, d_e = mav.ne_offset(*self.entry, *self.exit)
        n = math.hypot(d_n, d_e)
        self.axis = (d_n / n, d_e / n)
        self.axis_yaw = math.atan2(d_n, d_e)               # ENU
        self.mouth = float(gp('mouth_offset_m'))
        self.offset = float(gp('gps_offset_m'))
        self.alt = float(gp('altitude_m'))
        self.timeout = float(gp('timeout_s'))
        self.result_file, self.trace_file = gp('result_file'), gp('trace_file')
        self.tags = json.loads(gp('tags'))
        course = Course(width=float(gp('corridor_width_m')),
                        length=float(gp('corridor_length_m')),
                        wall_height=float(gp('corridor_wall_height_m')),
                        boxes=boxes_from_flat(list(gp('obstacles')),
                                              float(gp('obstacle_depth_m'))))
        self.judge = Judge(course, EvalParams(
            drone_radius_m=0.5 * float(gp('drone_width_m')),
            body_below_m=float(gp('body_below_m')),
            require_rejoin=bool(gp('require_rejoin')), stall_s=float(gp('stall_s'))))
        self.get_logger().info(
            f'[oa_eval] planner={self.planner}: corridor {course.width:.2f} x '
            f'{course.length:.2f} m, boxes {course.boxes}, flying at {self.alt:.2f} m')

        self._state = self._gps = self._rel_alt = None
        self._trace = []
        self._lock = threading.Lock()
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(State, '/mavros/state',
                                 lambda m: setattr(self, '_state', m), 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 lambda m: setattr(self, '_gps', (m.latitude, m.longitude)),
                                 best)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 lambda m: setattr(self, '_rel_alt', m.data), best)
        self._judging = False
        self.create_timer(0.1, self._tick)
        self.done = False
        threading.Thread(target=self._run, daemon=True).start()

    # ── truth ───────────────────────────────────────────────────────────
    def _corridor_xy(self):
        d_n, d_e = mav.ne_offset(*self.entry, *self._gps)
        a_n, a_e = self.axis
        return d_n * a_n + d_e * a_e - self.mouth, d_n * a_e - d_e * a_n + self.offset

    def _tick(self):
        if not self._judging or self._gps is None or self._rel_alt is None:
            return
        x, y = self._corridor_xy()
        t = time.monotonic()
        with self._lock:
            self.judge.update(t, x, y, self._rel_alt)
            self._trace.append((t, x, y, self._rel_alt))

    # ── flight ──────────────────────────────────────────────────────────
    def _wait(self, cond, timeout, period=0.2):
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            if cond():
                return True
            time.sleep(period)
        return False

    def _dist(self, target):
        return mav.haversine(*self._gps, *target) if self._gps else float('inf')

    def _run(self):
        outcome = 'ok'
        try:
            outcome = self._fly()
        except Exception as e:                       # record, never hang the batch
            outcome = f'crash: {e}'
            self.get_logger().error(f'[oa_eval] {outcome}')
        self._judging = False
        self._write(outcome)
        if self._state is not None and self._state.armed:
            mav.land(self)
            self._wait(lambda: not self._state.armed, 60.0)
        self.done = True

    def _fly(self):
        log = self.get_logger().info
        if not self._wait(lambda: self._state is not None and self._state.connected, 120.0):
            return 'no_mavros'
        # Nothing else asks for the streams here (mission_node does in the
        # mission), and sitl_corridor_sim renders nothing without the GPS.
        mav.request_message_intervals(self, rate_hz=10.0)
        if not self._wait(lambda: self._gps is not None and self._rel_alt is not None, 120.0):
            return 'no_gps'
        # EKF/GPS settle in SITL: keep trying to arm until pre-arm passes.
        armed = False
        end = time.monotonic() + 120.0
        while not armed and time.monotonic() < end:
            mav.set_mode(self, 'GUIDED')
            armed = mav.arm(self) and self._wait(lambda: self._state.armed, 3.0)
            if not armed:
                time.sleep(3.0)
        if not armed:
            return 'arm_failed'
        if not mav.takeoff(self, self.alt):
            return 'takeoff_rejected'
        if not self._wait(lambda: self._rel_alt > self.alt - 0.15, 40.0):
            return 'takeoff_timeout'
        self._judging = True

        if self.planner == 'current':
            return self._fly_current()

        log('[oa_eval] bendy: to the corridor entry point')
        mav.goto(self, *self.entry, self.alt, publish_count=10, yaw_enu=self.axis_yaw)
        if not self._wait(lambda: self._dist(self.entry) < 0.4, 60.0):
            return 'entry_timeout'
        time.sleep(2.0)                              # settle, let the OA database fill
        log('[oa_eval] bendy: ONE target, the corridor exit — BendyRuler has it now')
        mav.goto(self, *self.exit, self.alt, publish_count=10, yaw_enu=self.axis_yaw)
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.timeout and rclpy.ok():
            if self.judge.exited:
                time.sleep(1.0)
                return 'ok'
            time.sleep(0.2)
        return 'timeout'

    def _fly_current(self):
        cli = self.create_client(Trigger, '/path_planner/fly_corridor_2')
        if not cli.wait_for_service(timeout_sec=30.0):
            return 'no_path_planner'
        self.get_logger().info('[oa_eval] current: /path_planner/fly_corridor_2')
        fut = cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.timeout and rclpy.ok() and not fut.done():
            time.sleep(0.2)
        if not fut.done():
            stop = self.create_client(Trigger, '/path_planner/stop')
            if stop.wait_for_service(timeout_sec=2.0):
                stop.call_async(Trigger.Request())
            return 'timeout'
        r = fut.result()
        return 'ok' if r.success else f'leg_failed: {r.message}'

    # ── result ──────────────────────────────────────────────────────────
    def _write(self, outcome):
        with self._lock:
            r = self.judge.result()
            trace = list(self._trace)
        if outcome != 'ok' and outcome not in r['reasons']:
            r['clean'] = False
            r['reasons'].append(outcome.split(':')[0])
        r.update(planner=self.planner, outcome=outcome, tags=self.tags,
                 stamp=time.strftime('%Y-%m-%d %H:%M:%S'))
        with open(self.result_file, 'a') as f:
            f.write(json.dumps(r) + '\n')
        if self.trace_file:
            with open(self.trace_file, 'w') as f:
                f.write('t,x,y,z\n')
                t0 = trace[0][0] if trace else 0.0
                for t, x, y, z in trace:
                    f.write(f'{t - t0:.2f},{x:.3f},{y:.3f},{z:.3f}\n')
        self.get_logger().info(
            f"[oa_eval] RESULT {'CLEAN' if r['clean'] else 'FAIL ' + ','.join(r['reasons'])}: "
            f"min clearance wall {r['min_wall_clearance_m']} m, obstacle "
            f"{r['min_obstacle_clearance_m']} m, passes {r['passes']}, flips "
            f"{r['side_flips']}, {r['time_s']} s -> {self.result_file}")


def main(args=None):
    rclpy.init(args=args)
    node = OaEvalNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
