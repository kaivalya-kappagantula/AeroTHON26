# Closed-loop test of path_planner_node._follow_corridor (the real loop code)
# with a fake clock, a kinematic vehicle and a perception oracle that models
# the camera's limits: walls fitted only while >= 1.8 m of them is ahead,
# obstacles reported only in range and inside the depth slice, measurements
# taken at the pose of the previous cycle (latency). GPS carries a fixed
# lateral error the centring must remove.
import math
import types

import pytest
import rclpy

from mission_common import mavros_utils as mav
import path_planner.path_planner_node as ppn

W = 1.524
W2 = W / 2
MOUTH = 2.5              # entry waypoint -> mouth
WALLS = 12.5             # walls length
LENGTH = MOUTH + WALLS + 1.0
DRONE_R = 0.55 / 2
ENTRY = (12.9716, 77.5946)
AXIS = (0.0, 1.0)        # corridor runs East
SLICE_LO, SLICE_HI = -0.35, 0.50


class Clock:
    def __init__(self):
        self.t = 1000.0
        self.on_sleep = None        # called once per loop cycle

    def time(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 0.0)
        if self.on_sleep is not None:
            self.on_sleep()


class World:
    """Truth: corridor frame x (from the mouth), y (+left), alt, yaw."""

    def __init__(self, boxes, gps_error, y0):
        self.boxes = boxes            # (x0, x1, y0, y1, top)
        self.err = gps_error          # EKF lateral = truth - err
        self.x, self.y, self.alt, self.yaw = -MOUTH, y0, 1.5, 0.0
        self.track = []

    def ekf_gps(self):
        return mav.offset_gps(ENTRY[0], ENTRY[1], self.y - self.err, self.x + MOUTH)

    def perception(self):
        nan = float("nan")
        m = dict(walls=0, heading_err=nan, wall_left=nan, wall_right=nan,
                 mouth_dist=nan, walls_end=nan, obs_dist=nan, obs_left=nan,
                 obs_right=nan, obs_top=nan, slice_lo=SLICE_LO, cam_alt=self.alt,
                 cam_yaw=self.yaw, n_points=0)
        if not (-6.0 < self.x < WALLS - 1.8):
            return m
        m.update(walls=2, heading_err=-self.yaw, wall_left=W2 - self.y,
                 wall_right=-W2 - self.y)
        if self.x < -1.0:
            m["mouth_dist"] = -self.x
        if WALLS - self.x < 4.4:
            m["walls_end"] = WALLS - self.x
        ahead = [b for b in self.boxes if b[0] - self.x > 0.3 and b[0] - self.x < 3.5
                 and b[4] - self.alt > SLICE_LO]
        if ahead:
            b = min(ahead, key=lambda b: b[0])
            d = b[0] - self.x
            # Top visible only below the top of the view at that distance
            # (OAK-D Lite vertical half-FOV ~27.7 deg), else NaN — as the
            # real perception reports it.
            top = b[4] - self.alt
            m.update(obs_dist=d, obs_left=b[3] - self.y, obs_right=b[2] - self.y,
                     obs_top=top if top < d * math.tan(math.radians(27.7)) - 0.1 else nan)
        return m


@pytest.fixture
def node(monkeypatch):
    rclpy.init(args=["--ros-args", "-p", "require_avoidance_sensor:=false",
                     "-p", "require_corridor_perception:=true"])
    clock = Clock()
    monkeypatch.setattr(ppn, "time", types.SimpleNamespace(time=clock.time, sleep=clock.sleep))
    n = ppn.PathPlannerNode()
    yield n, clock
    n.destroy_node()
    rclpy.shutdown()


def fly(node_clock, world, monkeypatch):
    n, clock = node_clock

    def sync():
        n._gps = world.ekf_gps()
        n._rel_alt = world.alt
        n._yaw_enu = world.yaw

    def perceive():
        # Perception of the pose BEFORE this cycle's move (latency).
        n._perception = (world.perception(), clock.time())

    def goto(node, lat, lon, alt, publish_count=20, yaw_enu=None, log=True):
        d_n, d_e = mav.ne_offset(*n._gps, lat, lon)
        dist = math.hypot(d_n, d_e)
        step = min(dist, 0.5 * 0.2)                 # 0.5 m/s, one 0.2 s cycle
        if dist > 1e-9:
            world.x += step * d_e / dist
            world.y += step * d_n / dist
        world.alt += max(-0.2, min(0.2, alt - world.alt))
        if yaw_enu is not None:
            world.yaw = yaw_enu
        world.track.append((world.x, world.y, world.alt))
        sync()
        return True

    monkeypatch.setattr(mav, "goto", goto)
    clock.on_sleep = perceive
    sync()
    n._marker = "none"
    n._leg_deadline = clock.time() + 300.0
    return n._follow_corridor("t", ENTRY, AXIS, 0.0, LENGTH)


def clearance(world):
    wall = min(W2 - abs(y) - DRONE_R for x, y, a in world.track if 0 <= x <= WALLS)
    obst = []
    for (x0, x1, y0, y1, top) in world.boxes:
        best = 9.0
        for x, y, a in world.track:
            horiz = math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)) - DRONE_R
            best = min(best, max(horiz, (a - 0.15) - top))
        obst.append(best)
    return wall, obst


def test_clean_corridor_with_gps_error(node, monkeypatch):
    world = World([], gps_error=0.6, y0=0.6)
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    wall, _ = clearance(world)
    assert wall > 0.35                               # centred despite 0.6 m GPS error


def test_obstacles_pass_both_sides_and_climb(node, monkeypatch):
    boxes = [(2.5, 2.9, -W2, -W2 + 0.533, 2.2),     # 35 % right wall -> pass left
             (6.0, 6.4, W2 - 0.61, W2, 2.2),        # 40 % left wall  -> pass right
             (9.0, 9.4, -0.15, 0.35, 1.8)]          # no side gap     -> climb
    world = World(boxes, gps_error=-0.8, y0=-0.8)
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    wall, obst = clearance(world)
    assert wall > 0.08, wall
    assert all(c > 0.10 for c in obst), obst
    assert max(a for x, y, a in world.track) > 1.8 + 0.15 + 0.2   # went over #3


def test_unpassable_obstacle_blocks(node, monkeypatch):
    boxes = [(4.0, 4.4, -0.3, 0.3, 5.0)]            # centred and too tall
    world = World(boxes, gps_error=0.0, y0=0.0)
    ok, msg = fly(node, world, monkeypatch)
    assert not ok and msg.startswith("BLOCKED"), msg
    _, obst = clearance(world)
    assert obst[0] > 0.3                             # stopped well short
