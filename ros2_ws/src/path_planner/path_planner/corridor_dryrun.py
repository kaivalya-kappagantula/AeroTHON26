# corridor_dryrun.py — hardware check of corridor following WITHOUT flying.
#
# Runs the flight code's corridor decisions on the real sensors and prints /
# logs what it WOULD command. It sends NOTHING to the flight controller (no
# MAVROS service, no setpoint): the FC may be powered for GPS and compass, or
# even armed-disabled; the drone is CARRIED through the corridor.
#
#   in : /corridor/perception (corridor_perception_node on the OAK-D),
#        /mavros/global_position/global, /mavros/global_position/compass_hdg
#   out: a line per frame on screen + CSV (csv_file)
#
# The same objects as path_planner_node._follow_corridor: GpsCorridorTrack
# (the GPS centreline, corrected by the walls) and ObstacleMemory + plan()
# from corridor_follow.py, with mission_params.yaml's values. The GPS line is
# the one fly_corridor_here uses: from where the node starts, along the
# heading it starts with — so START it with the drone on the corridor's
# centreline, nose down the corridor, a few metres before the mouth
# (start_offset_m: if you start off the centreline, how far LEFT of it you
# are — the line is then shifted back).
#
# What to look at (docs/CORRIDOR_HARDWARE_CHECK.md):
#   walls     0/1/2 per frame, accepted or rejected by the track
#   offset    where the track thinks the drone is from the centre (+ left);
#             compare with a tape measure at marks along the corridor
#   corr      how far the walls have moved the GPS line (stays < 1 m)
#   plan      the mode (centre / pass_left / pass_right / blocked) and the
#             step it would send — around a real obstacle it must pick the
#             open side before reaching the stop distance
#
#   ros2 run path_planner corridor_dryrun --ros-args \
#       --params-file <mission_params.yaml> -p csv_file:=/tmp/dryrun.csv

import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32MultiArray, Float64

from mission_common import corridor_msg
from mission_common import mavros_utils as mav
from path_planner.corridor_follow import (FollowParams, FuseParams, GpsCorridorTrack,
                                          ObstacleMemory, plan)


class CorridorDryRun(Node):
    def __init__(self):
        super().__init__('path_planner_node')       # reads the planner's yaml section
        f, d = FollowParams(), FuseParams()
        dp = self.declare_parameter
        dp('csv_file', '/tmp/corridor_dryrun.csv')
        dp('start_offset_m', 0.0)
        dp('obstacles', True)                       # exit corridor: true
        dp('corridor_width_m', 3.5052)
        dp('corridor_altitude_m', 1.5)
        dp('drone_width_m', f.drone_width_m)
        dp('side_clearance_m', f.side_clearance_m)
        dp('corridor_lookahead_m', f.lookahead_m)
        dp('obstacle_stop_dist_m', f.stop_dist_m)
        dp('obstacle_length_m', f.obstacle_length_m)
        for k, v in vars(d).items():
            dp(f'corridor_fuse_{k}', v)
        gp = lambda n: self.get_parameter(n).value          # noqa: E731
        self.fp = FollowParams(
            drone_width_m=float(gp('drone_width_m')),
            side_clearance_m=float(gp('side_clearance_m')),
            lookahead_m=float(gp('corridor_lookahead_m')),
            stop_dist_m=float(gp('obstacle_stop_dist_m')),
            obstacle_length_m=float(gp('obstacle_length_m')),
            max_altitude_m=float(gp('corridor_altitude_m')),    # never climbs here
        )
        self.fuse = FuseParams(**{k: float(gp(f'corridor_fuse_{k}')) for k in vars(d)})
        self.width = float(gp('corridor_width_m'))
        self.alt = float(gp('corridor_altitude_m'))
        self.offset0 = float(gp('start_offset_m'))
        self.obstacles = bool(gp('obstacles'))

        self._gps = self._yaw = None
        self.entry = self.axis = self.track = None
        self.memory = ObstacleMemory(self.fp)
        self.frames = 0
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self._gps_cb, best)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self._hdg_cb, best)
        self.create_subscription(Float32MultiArray, '/corridor/perception', self._perc_cb, 10)
        self._csv = open(gp('csv_file'), 'w', newline='')
        self._w = csv.writer(self._csv)
        self._w.writerow(['t', 's', 'gps_left', 'walls', 'accepted', 'offset', 'correction',
                          'axis_dev_deg', 'obs_dist', 'obs_right', 'obs_left', 'mode',
                          'step_across', 'step_on', 'target'])
        self.t0 = time.time()
        self.get_logger().info(
            '[dryrun] NOTHING is sent to the FC. Waiting for GPS + compass; the GPS line '
            'starts HERE along the CURRENT heading — stand on the centreline '
            f'(start_offset_m {self.offset0:+.2f}), nose down the corridor.')

    def _gps_cb(self, m):
        if m.status.status >= NavSatStatus.STATUS_FIX:
            self._gps = (m.latitude, m.longitude)

    def _hdg_cb(self, m):
        self._yaw = math.atan2(math.sin(math.radians(90.0 - m.data)),
                               math.cos(math.radians(90.0 - m.data)))

    def _perc_cb(self, msg):
        m = corridor_msg.decode(msg)
        if m is None or self._gps is None or self._yaw is None:
            return
        now = time.time()
        if self.track is None:
            self.entry, a = self._gps, self._yaw
            self.axis = (math.sin(a), math.cos(a))
            self.track = GpsCorridorTrack(self.fp, self.fuse, a, self.width, -self.offset0)
            self.get_logger().info(
                f'[dryrun] GPS line set: heading {math.degrees(a):.0f} deg ENU, '
                f'corridor {self.width:.2f} m')
        d_n, d_e = mav.ne_offset(*self.entry, *self._gps)
        s = d_n * self.axis[0] + d_e * self.axis[1]
        left = d_n * self.axis[1] - d_e * self.axis[0]
        self.track.set_pose(s, left, now)
        cam_yaw = self._yaw if math.isnan(m['cam_yaw']) else m['cam_yaw']
        ok = self.track.update(m, now, cam_yaw)
        if self.obstacles and not math.isnan(m['obs_dist']):
            self.memory.update(m, self.track.centre, s, self.alt, self.track.width)
        self.memory.passed(s)
        cmd = plan(self.track, self.memory.confirmed() if self.obstacles else [],
                   s, self.alt, self.alt, self.fp)
        self.frames += 1
        dev = math.degrees(math.atan2(math.sin(self.track.axis - self.track.axis_yaw),
                                      math.cos(self.track.axis - self.track.axis_yaw)))
        row = [round(now - self.t0, 2), round(s, 2), round(left, 2), int(m['walls']), int(ok),
               round(-self.track.centre, 2), round(self.track.correction, 2), round(dev, 1),
               round(m['obs_dist'], 2), round(m['obs_right'], 2), round(m['obs_left'], 2),
               cmd.mode, round(cmd.lateral, 2), round(cmd.advance, 2), round(cmd.target, 2)]
        self._w.writerow(row)
        if self.frames % 3 == 0:
            self._csv.flush()
            o = self.memory.active
            self.get_logger().info(
                f"[dryrun] s={s:4.1f} m  walls={int(m['walls'])} "
                f"{'ACCEPTED' if ok else ('rejected' if int(m['walls']) else '        ')}  "
                f"offset={-self.track.centre:+.2f} m (+left)  corr={self.track.correction:+.2f} m "
                f"axis {dev:+.1f} deg  |  {cmd.mode:10s} step {cmd.lateral:+.2f} across "
                f"{cmd.advance:.2f} on"
                + ('' if o is None else
                   f"  |  obstacle {o.s_front - s:.1f} m ahead [{o.right:+.2f},{o.left:+.2f}]"))

    def close(self):
        self._csv.close()
        if self.track is not None:
            self.get_logger().info(
                f'[dryrun] {self.frames} frames, walls accepted {self.track.accepted}, '
                f'rejected {self.track.rejected}, final correction '
                f'{self.track.correction:+.2f} m -> {self._csv.name}')


def main(args=None):
    rclpy.init(args=args)
    node = CorridorDryRun()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
