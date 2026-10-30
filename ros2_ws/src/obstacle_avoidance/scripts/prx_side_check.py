#!/usr/bin/env python3
"""Test 1 of docs/OA_EVALUATION.md: does ArduPilot see the obstacle on the
SAME side as the camera, and is its proximity sensor healthy?

Reads both ends of the chain at once, live, on hardware or SITL:
  ours   /avoidance/scan (sector_builder, REP-103: + bearing = LEFT)
  FC     what ArduPilot itself reports back, through MAVROS's raw MAVLink
         stream (/uas1/mavlink_source): SYS_STATUS proximity present /
         enabled / healthy bits, and the 8 proximity sectors it sends as
         DISTANCE_SENSOR (orientation 0 = forward, 1 = 45 deg RIGHT ... 7 =
         45 deg LEFT — ArduPilot's own convention, not geometry.py's)

Procedure: point the vehicle at open space, then hold a box ~1.5 m off to
the LEFT of the nose for 10 s, then to the RIGHT. Each line shows both sides;
"AGREE" when the FC's nearest sector is on the same side as our nearest bin.

    python3 scripts/prx_side_check.py                 # live
    python3 scripts/prx_side_check.py --expect left   # PASS/FAIL after --secs
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       qos_profile_sensor_data)
from mavros_msgs.msg import Mavlink
from mavros_msgs.srv import MessageInterval
from mavros.mavlink import convert_to_bytes
from pymavlink.dialects.v20 import ardupilotmega as apm
from sensor_msgs.msg import LaserScan

PRX_BIT = apm.MAV_SYS_STATUS_SENSOR_PROXIMITY
SECTOR_NAME = ['fwd', 'fwd-R', 'R', 'back-R', 'back', 'back-L', 'L', 'fwd-L']
SECTOR_SIDE = [0, -1, -1, -1, 0, 1, 1, 1]       # +1 left, -1 right (like ours)
NEAR_M = 3.0                                     # only judge sides inside this
STALE_S = 1.5
ROUTER_QOS = QoSProfile(depth=1000, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE)


def _side(deg):
    return 0 if abs(deg) < 7.5 else (1 if deg > 0 else -1)


def _side_name(s):
    return {1: 'LEFT', -1: 'RIGHT', 0: 'ahead'}[s]


class PrxSideCheck(Node):
    def __init__(self, args):
        super().__init__('prx_side_check')
        self.args = args
        self.parser = apm.MAVLink(None)
        self.parser.robust_parsing = True
        self.sectors = {}            # orientation -> (metres, t)
        self.prx = None              # (present, enabled, healthy, t)
        self.ours = None             # (range, bearing_deg, t)
        self.tally = {'agree': 0, 'disagree': 0, 'fc_side': {}, 'unhealthy': 0}
        self.create_subscription(Mavlink, '/uas1/mavlink_source', self._on_mav, ROUTER_QOS)
        self.create_subscription(LaserScan, '/avoidance/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self._report)
        self._ask_streams()
        self.t0 = time.monotonic()

    def _ask_streams(self):
        cli = self.create_client(MessageInterval, '/mavros/set_message_interval')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('no /mavros/set_message_interval: relying on default streams')
            return
        for mid in (apm.MAVLINK_MSG_ID_DISTANCE_SENSOR, apm.MAVLINK_MSG_ID_SYS_STATUS):
            cli.call_async(MessageInterval.Request(message_id=mid, message_rate=5.0))

    def _on_mav(self, m):
        if m.msgid not in (apm.MAVLINK_MSG_ID_DISTANCE_SENSOR, apm.MAVLINK_MSG_ID_SYS_STATUS):
            return
        try:
            msgs = self.parser.parse_buffer(bytes(convert_to_bytes(m))) or []
        except Exception:
            return
        now = time.monotonic()
        for msg in msgs:
            if msg.get_type() == 'SYS_STATUS':
                self.prx = (bool(msg.onboard_control_sensors_present & PRX_BIT),
                            bool(msg.onboard_control_sensors_enabled & PRX_BIT),
                            bool(msg.onboard_control_sensors_health & PRX_BIT), now)
            elif msg.get_type() == 'DISTANCE_SENSOR' and msg.orientation < 8:
                self.sectors[msg.orientation] = (msg.current_distance / 100.0, now)

    def _on_scan(self, s):
        best = (math.inf, 0.0)
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and r < best[0]:
                best = (r, math.degrees(s.angle_min + i * s.angle_increment))
        self.ours = (*best, time.monotonic())

    def _report(self):
        now = time.monotonic()
        if self.prx is None or now - self.prx[3] > STALE_S:
            health = 'FC proximity: no SYS_STATUS'
            healthy = False
        else:
            present, enabled, healthy = self.prx[:3]
            health = (f"FC proximity present={present} enabled={enabled} "
                      f"{'HEALTHY' if healthy else 'UNHEALTHY'}")
        live = {o: d for o, (d, t) in self.sectors.items() if now - t < STALE_S}
        fc = ' '.join(f"{SECTOR_NAME[o]}={d:.2f}" for o, d in sorted(live.items())) or 'no sectors'
        fc_near = min(live.items(), key=lambda kv: kv[1]) if live else None
        if self.ours is None or now - self.ours[2] > STALE_S:
            ours, our_side = 'ours: no /avoidance/scan', None
        elif math.isfinite(self.ours[0]):
            our_side = _side(self.ours[1])
            ours = f"ours: {self.ours[0]:.2f} m at {self.ours[1]:+.0f} deg ({_side_name(our_side)})"
        else:
            ours, our_side = 'ours: clear', None

        verdict = ''
        if fc_near and fc_near[1] < NEAR_M and our_side is not None and self.ours[0] < NEAR_M:
            fc_side = SECTOR_SIDE[fc_near[0]]
            ok = fc_side == our_side
            verdict = f"  FC nearest {SECTOR_NAME[fc_near[0]]} -> {'AGREE' if ok else 'DISAGREE'}"
            self.tally['agree' if ok else 'disagree'] += 1
            self.tally['fc_side'][fc_side] = self.tally['fc_side'].get(fc_side, 0) + 1
        if not healthy:
            self.tally['unhealthy'] += 1
        print(f"[{now - self.t0:5.1f}s] {health} | {ours} | FC: {fc}{verdict}", flush=True)

    def summary(self):
        t = self.tally
        print(f"\nAGREE {t['agree']}  DISAGREE {t['disagree']}  "
              f"seconds unhealthy {t['unhealthy']}  FC side counts "
              f"{ {_side_name(k): v for k, v in t['fc_side'].items()} }")
        if self.args.expect:
            want = 1 if self.args.expect == 'left' else -1
            fc_ok = t['fc_side'].get(want, 0) > 0.8 * max(1, sum(t['fc_side'].values()))
            ok = fc_ok and t['disagree'] == 0 and t['unhealthy'] == 0 and t['agree'] > 0
            print(f"TEST 1 ({self.args.expect}): {'PASS' if ok else 'FAIL'}")
            return ok
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--expect', choices=['left', 'right'])
    ap.add_argument('--secs', type=float, default=10.0)
    args = ap.parse_args()
    rclpy.init()
    node = PrxSideCheck(args)
    try:
        end = time.monotonic() + args.secs if args.expect else math.inf
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = node.summary()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
