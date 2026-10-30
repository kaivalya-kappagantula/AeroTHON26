# Entry corridor (mission phase 2): the pure logic in entry_corridor.py, and
# the whole leg closed-loop — path_planner_node._banner_corridor (the real
# code: search, approach, height, line-up, corridor follower, exit) against a
# fake clock, a kinematic vehicle and oracles for the banner and the walls
# that model the cameras' limits. RealWorld replaces the wall oracle with
# obstacle_avoidance's real corridor fit on ray-cast depth, for corridor sizes
# the oracle cannot judge — and for the exit corridor (phase 5, blue banner)
# with its obstacles, which the same leg passes beside.
import collections
import math
import statistics
import types

import numpy as np
from dataclasses import replace

import pytest
import rclpy

from mission_common import mavros_utils as mav
from obstacle_avoidance import corridor_geometry as cg
from obstacle_avoidance.corridor_render import Camera, Scene, render
import path_planner.path_planner_node as ppn
from path_planner.entry_corridor import (BannerFix, EntryParams, approach_command, banner_view,
                                         lateral_offset, point_out, search_altitudes,
                                         sweep_angle, sweep_duration, sweep_points,
                                         target_altitude)

P = EntryParams()
NAN = float("nan")
HFOV2, VFOV2 = math.radians(34.5), math.radians(21.2)     # OAK-D Lite colour 640x360
BANNER_KEYS = ("bearing", "bearing_left", "bearing_right", "elev_bottom",
               "elev_bottom_left", "elev_bottom_right")


def banner_frame(pos, alt, yaw, banner_h, half, yc=0.0):
    """What banner_node reports from `pos` (east, north) for a banner across
    x = 0 at y = yc +- half, bottom edge banner_h: bearings of its centre and
    ends, the bottom edge's elevation at each end and at its lowest image
    point (the bounding-box bottom, taken at the centre column); NaN where
    out of the view. None if its centre is out of the view or beyond 15 m."""
    def ray(y):
        dx, dy = -pos[0], y - pos[1]
        return _wrap(math.atan2(dy, dx) - yaw), math.hypot(dx, dy)
    b, d = ray(yc)
    if pos[0] > -0.3 or d > 15.0 or abs(b) > HFOV2 \
            or abs(math.atan2(banner_h - alt, d)) > VFOV2:
        return None
    ends = [ray(yc + half), ray(yc - half)]
    elevs = [math.atan2(banner_h - alt, de) for _, de in ends]
    # image row of a point ~ tan(elevation) / cos(bearing): the lowest is the bbox bottom
    k = min(math.tan(e) / math.cos(be) for e, (be, _) in zip(elevs, ends))
    elev = math.atan(k * math.cos(b))
    cut = [abs(be) > HFOV2 or abs(e) > VFOV2 for e, (be, _) in zip(elevs, ends)]
    return dict(bearing=b,
                bearing_left=NAN if abs(ends[0][0]) > HFOV2 else ends[0][0],
                bearing_right=NAN if abs(ends[1][0]) > HFOV2 else ends[1][0],
                elev_bottom=elev if abs(elev) <= VFOV2 else NAN,
                elev_bottom_left=NAN if cut[0] else elevs[0],
                elev_bottom_right=NAN if cut[1] else elevs[1])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# --- pure logic --------------------------------------------------------------

def test_search_altitudes_current_first_then_nearest():
    assert search_altitudes(2.0, P) == [2.0, 1.5, 2.5, 1.0, 3.0]
    assert search_altitudes(5.0, P) == [5.0, 3.0, 2.5, 2.0, 1.5, 1.0]
    assert search_altitudes(1.1, P) == [1.1, 1.5, 2.0, 2.5, 3.0]


def test_middle_of_the_opening():
    assert target_altitude(3.0, P) == pytest.approx(1.5)
    assert target_altitude(0.6, P) == pytest.approx(P.min_altitude_m)


@pytest.mark.parametrize("alt", [1.4, 4.0])                 # below and above the banner
@pytest.mark.parametrize("pos,yaw_deg", [((-8.0, 0.0), 0.0), ((-7.0, -5.0), 20.0),
                                          ((-5.0, -5.0), 60.0)])
def test_banner_view_reads_the_corridor_direction_from_one_frame(pos, yaw_deg, alt):
    # 11.5 ft banner on 8 ft walls, corridor running east (ENU 0); the nose
    # anywhere that keeps the banner in view. No size, no compass.
    yaw = math.radians(yaw_deg)
    v = banner_view(banner_frame(pos, alt, yaw, 2.4384, 1.7526))
    assert v.above == (alt < 2.4384)
    assert _wrap(yaw + v.axis) == pytest.approx(0.0, abs=1e-6)
    k = abs(2.4384 - alt)                                  # scale: metres per unit
    fwd, left = v.centre
    mid = (pos[0] + k * (fwd * math.cos(yaw) - left * math.sin(yaw)),
           pos[1] + k * (fwd * math.sin(yaw) + left * math.cos(yaw)))
    assert mid == pytest.approx((0.0, 0.0), abs=1e-6)


def test_banner_view_needs_both_ends_and_some_elevation():
    b = banner_frame((-8.0, 0.0), 1.4, 0.0, 2.4384, 1.7526)
    assert banner_view(dict(b, bearing_left=NAN)) is None
    level = banner_frame((-8.0, 0.0), 2.43, 0.0, 2.4384, 1.7526)   # camera level with it
    assert banner_view(level) is None


def _fly(path, banner_h, half, yaw_bias=0.0):
    """BannerFix fed from banner frames along path [(east, north, alt)], nose
    on the banner's middle, the compass yaw_bias (rad) off."""
    fix = BannerFix(P)
    for x, y, a in path:
        yaw = math.atan2(-y, -x)
        b = banner_frame((x, y), a, yaw, banner_h, half)
        v = banner_view(b) if b is not None else None
        if v is not None:
            fix.add((x, y), a, yaw + yaw_bias, v)
    return fix


def test_height_from_the_descent_alone():
    # Spotted 10 m out at 5 m, descending in place to 1.4 m: fixed there.
    fix = _fly([(-10.0, 0.0, 5.0 - 0.2 * i) for i in range(19)], 2.4384, 1.7526)
    assert fix.ready() and fix.height == pytest.approx(2.4384, abs=0.01)


def test_height_from_closing_in_at_one_altitude():
    fix = _fly([(-12.0 + 0.25 * i, -0.1 * i, 1.4) for i in range(16)], 2.4384, 0.762)
    assert fix.ready() and fix.height == pytest.approx(2.4384, abs=0.01)


@pytest.mark.parametrize("bias_deg", [0.0, 3.0, -3.0])
def test_line_up_point_with_compass_error(bias_deg):
    # 35 deg off to the side, 6 m out: the compass error only turns the
    # newest frame's result about the vehicle, so the line-up point is off
    # by ~6 m x the error at most, and the height not at all.
    path = [(-10.0, -7.0, 5.0 - 0.2 * i) for i in range(19)]
    path += [(-10.0 + 0.25 * i, -7.0 + 0.175 * i, 1.4) for i in range(17)]
    fix = _fly(path, 2.4384, 1.7526, math.radians(bias_deg))
    assert fix.ready() and fix.height == pytest.approx(2.4384, abs=0.03)
    mid, axis = fix.locate()
    d = math.hypot(*path[-BannerFix.PLACE_FRAMES][:2])     # farthest frame used
    err = d * math.radians(abs(bias_deg)) + 0.02
    assert math.hypot(*mid) < err
    assert _wrap(axis) == pytest.approx(math.radians(bias_deg), abs=1e-3)
    assert math.hypot(*point_out(mid, axis, 2.5)) == pytest.approx(2.5, abs=err)


def test_not_fixed_from_a_single_place_and_height():
    fix = _fly([(-10.0, 0.0, 1.4)] * 10, 2.4384, 1.7526)
    assert not fix.ready()


def test_lateral_offset_flags_the_neighbouring_corridor():
    # Banner over the corridor 3 m to the left of the walls being tracked.
    assert lateral_offset((0.0, 3.0), (-4.0, 0.0), 0.0, 0.0) == pytest.approx(3.0)


def test_approach_gets_below_the_banner_first():
    c = approach_command(True, math.radians(-3), 6.0, 2.5, 2.5, 0.0, P, 0.8, 0.15)
    assert c.mode == "descend" and c.advance == 0.0 and c.alt < 2.5
    # Its bottom edge below the view: also above it.
    c = approach_command(True, NAN, NAN, 2.5, 2.5, 0.0, P, 0.8, 0.15)
    assert c.mode == "descend"


def test_approach_closes_in_from_the_lowest_search_altitude():
    # Far away a low banner can stay under 5 deg up even at the lowest
    # search altitude: closing in raises it (holding there never would).
    c = approach_command(True, math.radians(3), NAN, 1.0, 1.0, 0.0, P, 0.8, 0.15)
    assert c.mode == "approach" and c.advance > 0.0


def test_approach_stops_at_standoff():
    c = approach_command(True, math.radians(20), 1.8, 1.5, 1.5, 0.0, P, 0.8, 0.15)
    assert c.mode == "standoff" and c.advance == 0.0


def test_approach_backs_off_when_banner_lost_unfixed():
    c = approach_command(False, NAN, NAN, 1.5, 1.5, 0.0, P, 0.8, 0.15)
    assert c.mode == "backoff" and c.advance < 0.0
    c = approach_command(False, NAN, NAN, 1.5, 1.5, P.max_backoff_m, P, 0.8, 0.15)
    assert c.mode == "hold"


# --- closed loop -------------------------------------------------------------

REF = (12.9716, 77.5946)      # world origin: centre of the banner corridor's mouth
SPEED, CLIMB, YAW_RATE = 0.5, 0.5, math.radians(90)


class Clock:
    def __init__(self):
        self.t = 1000.0
        self.on_sleep = None

    def time(self):
        return self.t

    def sleep(self, dt):
        dt = max(dt, 0.0)
        self.t += dt
        if self.on_sleep is not None:
            self.on_sleep(dt)


class World:
    """Corridors run East (+x) from x = 0; y = North. `corridors` are
    (centre y, width); the first one carries the green banner, bottom edge
    at banner_h, across its entrance from wall to wall."""

    def __init__(self, corridors, length, banner_h, pose):
        self.corridors, self.length, self.banner_h = corridors, length, banner_h
        self.x, self.y, self.alt, self.yaw = pose
        self.target = None            # (x, y, alt, yaw)
        self.track = []

    def gps(self):
        return mav.offset_gps(REF[0], REF[1], self.y, self.x)

    def step(self, dt):
        if self.target is not None:
            tx, ty, ta, tyaw = self.target
            dx, dy = tx - self.x, ty - self.y
            d = math.hypot(dx, dy)
            k = min(1.0, SPEED * dt / d) if d > 1e-9 else 0.0
            self.x, self.y = self.x + k * dx, self.y + k * dy
            self.alt += max(-CLIMB * dt, min(CLIMB * dt, ta - self.alt))
            if tyaw is not None:
                e = math.atan2(math.sin(tyaw - self.yaw), math.cos(tyaw - self.yaw))
                self.yaw += max(-YAW_RATE * dt, min(YAW_RATE * dt, e))
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.track.append((self.x, self.y, self.alt))

    def banner(self):
        """/corridor/banner as banner_node would publish it (banner_frame()),
        plain cloth: no stereo distance."""
        yc, w = self.corridors[0]
        out = dict(state=0, bearing=NAN, bearing_left=NAN, bearing_right=NAN,
                   elev_bottom=NAN, elev_top=NAN, dist=NAN, area_frac=NAN,
                   cam_alt=self.alt, cam_yaw=self.yaw, elev_bottom_left=NAN,
                   elev_bottom_right=NAN)
        f = banner_frame((self.x, self.y), self.alt, self.yaw, self.banner_h, 0.5 * w, yc)
        if f is not None:
            out.update(state=2, **f)
        return out

    def perception(self):
        """/corridor/perception: the corridor nearest the vehicle laterally,
        walls fitted only while facing roughly along them and >= 1.8 m of
        wall is ahead; mouth beyond 1 m, end within 4.4 m."""
        m = dict(walls=0, heading_err=NAN, wall_left=NAN, wall_right=NAN,
                 mouth_dist=NAN, walls_end=NAN, obs_dist=NAN, obs_left=NAN,
                 obs_right=NAN, obs_top=NAN, slice_lo=-0.35, cam_alt=self.alt,
                 cam_yaw=self.yaw, n_points=0)
        yc, w = min(self.corridors, key=lambda c: abs(c[0] - self.y))
        if abs(self.yaw) > math.radians(30) or not (-6.0 < self.x < self.length - 1.8):
            return m
        m.update(walls=2, heading_err=-self.yaw, wall_left=yc + w / 2 - self.y,
                 wall_right=yc - w / 2 - self.y)
        if self.x < -1.0:
            m["mouth_dist"] = -self.x
        if self.length - self.x < 4.4:
            m["walls_end"] = self.length - self.x
        return m


class RealWorld(World):
    """World whose /corridor/perception is the REAL corridor fit on depth
    ray-cast from the vehicle pose, as corridor_perception_node runs it in
    SITL (half-resolution OAK-D depth, stride 2, width from recent wall
    pairs, else width_prior). One corridor; wall_height is its walls' height.

    As on the drone (wall_depth_max): wall pixels beyond this give no depth
    (0 = none at all, None = perfect walls) — the OAK-D on the real 11.5 ft
    corridor saw one wall in most frames, both in ~4 %, none in ~8 %, which
    wall_depth_max 3.5 reproduces. gps_drift_per_m: GPS lateral error growing
    with the distance flown east of the start (the banner fix is in the GPS
    frame, so only the CHANGE since it hurts)."""
    c = Camera()
    CAM = Camera(fx=c.fx / 2, fy=c.fy / 2, cx=c.cx / 2, cy=c.cy / 2,
                 width=c.width // 2, height=c.height // 2)
    FIT = cg.CorridorParams()

    def __init__(self, corridor, length, wall_height, banner_h, pose, boxes=(),
                 wall_depth_max=None, width_prior=None, gps_drift_per_m=0.0):
        super().__init__([corridor], length, banner_h, pose)
        self.scene = Scene(width=corridor[1], x_start=0.0, x_end=length, wall_height=wall_height,
                           boxes=list(boxes))
        self.widths = collections.deque(maxlen=15)
        self.wall_depth_max = wall_depth_max
        self.width_prior = width_prior
        self.drift = gps_drift_per_m
        self.x0 = self.x
        if width_prior is not None:
            self.FIT = replace(cg.CorridorParams(), max_single_offset_m=1.5)

    def gps_error(self):
        """GPS lateral (north) error now, m."""
        return self.drift * (self.x - self.x0)

    def gps(self):
        return mav.offset_gps(REF[0], REF[1], self.y + self.gps_error(), self.x)

    def perception(self):
        cam = self.CAM
        pos = (self.x, self.y - self.corridors[0][0], self.alt)
        z = render(self.scene, cam, pos, self.yaw)
        if self.wall_depth_max is not None:
            sc = self.scene
            walls = np.nan_to_num(render(Scene(width=sc.width, x_start=sc.x_start,
                                               x_end=sc.x_end, wall_height=sc.wall_height),
                                         cam, pos, self.yaw), nan=np.inf)
            ground = np.nan_to_num(render(Scene(x_start=1e3, x_end=1e3 + 1.0), cam, pos,
                                          self.yaw), nan=np.inf)
            zz = np.nan_to_num(z, nan=np.inf)
            with np.errstate(invalid="ignore"):
                wall = np.isfinite(zz) & (np.abs(zz - walls) < 1e-3) & (walls < ground - 1e-3)
            z[wall & (zz > self.wall_depth_max)] = np.nan
        x, y, h = cg.level_points(z, cam.fx, cam.fy, cam.cx, cam.cy, 0.0, 2)
        fov = cg.view_limits(cam.fx, cam.fy, cam.cx, cam.cy, cam.width)
        width = statistics.median(self.widths) if self.widths else self.width_prior
        r = cg.fit_corridor(x, y, h, self.FIT, fov=fov, height_above_ground=self.alt,
                            width=width)
        if r["walls"] == 2:
            self.widths.append(r["wall_left"] - r["wall_right"])
        elif r["walls"] == 0:
            cg.find_obstacle_unfitted(r, x, y, h, self.FIT, width, 0.0, fov, self.alt)
        r.update(slice_lo=self.FIT.slice_lo_m, cam_alt=self.alt, cam_yaw=self.yaw,
                 path_dist=cg.path_distance(x, y, h, self.FIT, self.alt))
        return r


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


def fly(node_clock, world, monkeypatch, exit_corridor=False):
    """The entry corridor leg (green banner, clean), or with exit_corridor
    the exit corridor leg (blue banner, obstacles, no climbing)."""
    n, clock = node_clock
    n._banner_colour = "blue" if exit_corridor else "green"

    def sync():
        n._gps = world.gps()
        n._rel_alt = world.alt
        n._yaw_enu = world.yaw
        n._perception = (world.perception(), clock.time())
        n._banner = (world.banner(), clock.time())

    def goto(node, lat, lon, alt, publish_count=20, yaw_enu=None, log=True):
        d_n, d_e = mav.ne_offset(REF[0], REF[1], lat, lon)
        # The FC flies in the GPS frame the target was computed in.
        d_n -= world.gps_error() if hasattr(world, "gps_error") else 0.0
        world.target = (d_e, d_n, alt, yaw_enu)
        return True

    def on_sleep(dt):
        world.step(dt)
        sync()

    monkeypatch.setattr(mav, "goto", goto)
    monkeypatch.setattr(mav, "change_speed", lambda node, speed, timeout=5.0: True)
    clock.on_sleep = on_sleep
    world.target = (world.x, world.y, world.alt, world.yaw)
    sync()
    n._leg_deadline = clock.time() + 600.0
    if exit_corridor:
        return n._banner_corridor("t", "blue", obstacles=True, climb=False)
    return n._banner_corridor("t", "green", obstacles=False, climb=True)


def inside(world):
    return [(x, y, a) for x, y, a in world.track if 0.0 <= x <= world.length]


def test_search_approach_and_through(node, monkeypatch):
    # Starts 8 m out, 2 m to the side, nose turned 70 deg left of the
    # corridor: the banner is ~84 deg to the right, found on the right half
    # of the +-90 deg sweep (after the left half).
    world = World([(0.0, 3.5)], length=6.0, banner_h=3.0,
                  pose=(-8.0, 2.0, 2.0, math.radians(70.0)))
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    assert "walls ended" in msg
    pts = inside(world)
    assert pts and world.x >= world.length                  # went through to the end
    assert max(abs(y) for x, y, a in pts) < 0.3             # centred between the walls
    assert all(a == pytest.approx(1.5, abs=0.2) for x, y, a in pts)   # middle of 3.0 m
    assert max(a for x, y, a in pts) < 3.0 - 1.2            # well under the banner


def test_banner_behind_the_drone_is_not_searched(node, monkeypatch):
    # The drone is always placed facing the corridors, so the search only
    # sweeps the front half: facing straight away, the banner is never seen.
    world = World([(0.0, 3.5)], length=6.0, banner_h=3.0, pose=(-8.0, 2.0, 2.0, math.pi))
    ok, msg = fly(node, world, monkeypatch)
    assert not ok and msg.startswith("NO_BANNER")


def test_height_measured_without_depth_on_the_banner(node, monkeypatch):
    world = World([(0.0, 3.5)], length=6.0, banner_h=2.6, pose=(-9.0, 0.5, 2.0, 0.0))
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    pts = inside(world)
    assert all(a == pytest.approx(1.3, abs=0.2) for x, y, a in pts)


def test_banner_out_of_view_from_start_altitude(node, monkeypatch):
    # From 6 m the banner (bottom 2.5 m) is below the view: the sweep
    # there finds nothing, the next altitude (3.0 m) does; then it gets
    # below the banner before approaching.
    world = World([(0.0, 2.0)], length=5.0, banner_h=2.5, pose=(-8.0, 0.0, 6.0, 0.0))
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    approach = [(x, a) for x, y, a in world.track if -6.5 < x < -1.0]
    assert approach and max(a for x, a in approach) < 2.5   # never level with it near
    assert all(a == pytest.approx(1.25, abs=0.2) for x, y, a in inside(world))


def test_enters_the_corridor_with_the_banner_not_its_neighbour(node, monkeypatch):
    # Two corridors; the vehicle starts in line with the one WITHOUT the banner.
    world = World([(0.0, 2.0), (2.6, 2.0)], length=6.0, banner_h=3.0,
                  pose=(-8.0, 2.6, 1.8, 0.0))
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    pts = inside(world)
    assert pts and max(abs(y) for x, y, a in pts) < 0.4     # the banner corridor (y = 0)


FT = 0.3048


@pytest.mark.parametrize("width_ft,ahead,left", [
    (11.5, 10.0, 0.0),          # competition corridor, entrance straight ahead
    (5.0, 12.0, 6.0),           # narrow corridor, entrance 27 deg off to the left
])
def test_real_corridor_fit_through(node, monkeypatch, width_ft, ahead, left):
    # 8 ft walls, 22 ft long, the banner sitting on top of them (bottom edge
    # 8 ft), plain cloth (no stereo depth on it); climb to 5 m first. The
    # corridor width is configured (corridor_width_m); length and altitude
    # are measured.
    node[0]._c_width = width_ft * FT
    world = RealWorld((0.0, width_ft * FT), length=22 * FT, wall_height=8 * FT,
                      banner_h=8 * FT, pose=(-ahead, -left, 5.0, 0.0))
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    assert "walls ended" in msg
    pts = inside(world)
    assert pts and world.x >= world.length
    assert max(abs(y) for x, y, a in pts) < 0.15
    assert all(a == pytest.approx(4 * FT, abs=0.2) for x, y, a in pts)   # half of 8 ft


OBSTACLE_W, OBSTACLE_TOP, OBSTACLE_D = 3.5 * FT, 6 * FT, 0.4


def boxes_at(*spec):
    """(s_front, lateral centre + left) -> Scene boxes, 3.5 x 6 ft obstacles."""
    h = OBSTACLE_W / 2
    return [(s, s + OBSTACLE_D, c - h, c + h, OBSTACLE_TOP) for s, c in spec]


@pytest.mark.parametrize("spec,depth", [
    # middle, then both walls: KNOWN LIMIT — after the first obstacle the wall
    # fit's lateral estimate is ~0.2 m off, the path check sees obstacle 2
    # inside the drone's width and holds (safe), the leg times out
    pytest.param(((1.5, 0.0), (3.6, 1.1), (5.6, -1.1)), None,
                 marks=pytest.mark.xfail(strict=True, reason="lateral estimate near obstacles")),
    (((0.6, 1.0), (3.2, -0.9), (5.8, 0.6)), None),   # alternating, 2.6 m apart
    # Opposite walls only 2.2 m apart with no lane clearing both: only
    # possible when the planner knows they are 0.4 m deep (not 1.0 m).
    (((1.2, 1.0), (3.4, -0.9), (5.4, 0.1)), OBSTACLE_D),
])
def test_exit_corridor_passes_obstacles_beside(node, monkeypatch, spec, depth):
    # The exit corridor: competition size, three 3.5 x 6 ft obstacles. 6 ft
    # in an 8 ft corridor: the leg must go around each one (climbing is off)
    # without touching it or a wall, and come out the far end.
    if depth is not None:
        node[0]._follow = replace(node[0]._follow, obstacle_length_m=depth)
    node[0]._measured_len = 22 * FT     # as phase 2 measures the entry corridor
    boxes = boxes_at(*spec)
    half_w = 11.5 * FT / 2
    world = RealWorld((0.0, 11.5 * FT), length=22 * FT, wall_height=8 * FT,
                      banner_h=8 * FT, pose=(-10.0, 0.0, 3.0, 0.0), boxes=boxes)
    ok, msg = fly(node, world, monkeypatch, exit_corridor=True)
    assert ok, msg
    assert "walls ended" in msg or "expected end" in msg
    pts = inside(world)
    assert pts and world.x >= world.length
    assert max(a for x, y, a in pts) < 4 * FT + 0.25            # never climbed
    hw = node[0]._follow.drone_width_m / 2
    assert max(abs(y) for x, y, a in pts) < half_w - hw        # off the walls
    for x0, x1, y0, y1, _ in boxes:
        beside = [y for x, y, a in pts if x0 - hw <= x <= x1 + hw]
        assert beside, "no track beside an obstacle"
        gap = min(max(y0 - y, y - y1) for y in beside)
        assert gap > hw, f"within {gap:.2f} m of the obstacle at {x0:.1f} m"


def _random_layouts(n, seed):
    """n layouts of three obstacles anywhere across the corridor, fronts at
    least 1.6 m apart (seeded: the same every run)."""
    import random
    rng = random.Random(seed)
    out = []
    while len(out) < n:
        fronts = sorted(rng.uniform(0.6, 5.6) for _ in range(3))
        if min(b - a for a, b in zip(fronts, fronts[1:])) >= 1.6:
            out.append(tuple((round(f, 2), round(rng.uniform(-1.2, 1.2), 2)) for f in fronts))
    return out


@pytest.mark.parametrize("spec", _random_layouts(8, seed=7))
def test_exit_corridor_never_near_an_obstacle(node, monkeypatch, spec):
    # Random layouts: whether the leg gets through or fails safe (BLOCKED /
    # timeout -> the mission lands), the drone never comes within half its
    # width of an obstacle — not beside it, not across its face.
    node[0]._measured_len = 22 * FT
    boxes = boxes_at(*spec)
    world = RealWorld((0.0, 11.5 * FT), length=22 * FT, wall_height=8 * FT,
                      banner_h=8 * FT, pose=(-10.0, 0.0, 3.0, 0.0), boxes=boxes)
    ok, msg = fly(node, world, monkeypatch, exit_corridor=True)
    assert ok or msg.startswith(("BLOCKED", "LEG_TIMEOUT", "SENSOR_TIMEOUT")), msg
    hw = node[0]._follow.drone_width_m / 2
    for x0, x1, y0, y1, _ in boxes:
        near = [y for x, y, a in world.track if x0 - hw <= x <= x1 + hw]
        assert all(max(y0 - y, y - y1) > hw for y in near), f"{msg}: too close at {x0:.1f} m"


# ── As on the drone: poor wall depth, drifting GPS ─────────────────────────

@pytest.mark.parametrize("wall_depth_max", [3.5, 3.0, 0.0])
def test_entry_corridor_with_drone_like_wall_depth(node, monkeypatch, wall_depth_max):
    # 3.5: like the bench (one wall most frames); 3.0: worse (half the frames
    # no wall); 0.0: the walls give NO depth at all (white walls) — the
    # banner's GPS line alone. GPS drifts 0.3 m sideways over the run.
    world = RealWorld((0.0, 11.5 * FT), length=22 * FT, wall_height=8 * FT,
                      banner_h=8 * FT, pose=(-10.0, 0.0, 5.0, 0.0),
                      wall_depth_max=wall_depth_max, width_prior=11.5 * FT,
                      gps_drift_per_m=0.3 / 17.0)
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    pts = inside(world)
    assert pts and world.x >= world.length
    hw = node[0]._follow.drone_width_m / 2
    worst = max(abs(y) for x, y, a in pts)
    assert worst < 0.45, f"{worst:.2f} m off centre"
    assert 11.5 * FT / 2 - worst - hw > 0.5               # half a metre of air to the walls


@pytest.mark.parametrize("shift,err_deg", [(0.8, 8.0), (-0.8, -8.0), (1.0, 0.0)])
def test_entry_corridor_banner_position_wrong(node, monkeypatch, shift, err_deg):
    # SITL with drone-like walls and drifting GPS: the banner fix was ~0.8 m
    # to the side and 7 deg off; walls at the mouth disagreed with it, were
    # rejected (prior too confident), and the drone entered 1.6 m off centre.
    orig = ppn.BannerFix.locate

    def locate(self):
        loc = orig(self)
        if loc is None:
            return None
        (e, n), a = loc
        return (e - shift * math.sin(a), n + shift * math.cos(a)), a + math.radians(err_deg)
    monkeypatch.setattr(ppn.BannerFix, "locate", locate)
    world = RealWorld((0.0, 11.5 * FT), length=22 * FT, wall_height=8 * FT,
                      banner_h=8 * FT, pose=(-10.0, 0.0, 5.0, 0.0),
                      wall_depth_max=3.5, width_prior=11.5 * FT, gps_drift_per_m=0.3 / 17.0)
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    pts = inside(world)
    assert pts and world.x >= world.length
    hw = node[0]._follow.drone_width_m / 2
    worst = max(abs(y) for x, y, a in pts)
    assert 11.5 * FT / 2 - worst - hw > 0.25, f"{worst:.2f} m off centre"


@pytest.mark.parametrize("err_deg", [15.0, -20.0])
def test_entry_corridor_banner_direction_wrong(node, monkeypatch, err_deg):
    # SITL with drone-like walls: the banner fix's corridor direction was
    # 21 deg off and no wall was in view from the line-up point. The line
    # pivots under the banner (its position is good) and the walls seen
    # inside turn it: the drone must stay well inside.
    orig = ppn.BannerFix.locate

    def locate(self):
        loc = orig(self)
        return None if loc is None else (loc[0], loc[1] + math.radians(err_deg))
    monkeypatch.setattr(ppn.BannerFix, "locate", locate)
    world = RealWorld((0.0, 11.5 * FT), length=22 * FT, wall_height=8 * FT,
                      banner_h=8 * FT, pose=(-10.0, 0.0, 5.0, 0.0),
                      wall_depth_max=3.5, width_prior=11.5 * FT)
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    pts = inside(world)
    assert pts and world.x >= world.length
    hw = node[0]._follow.drone_width_m / 2
    worst = max(abs(y) for x, y, a in pts)
    assert 11.5 * FT / 2 - worst - hw > 0.25, f"{worst:.2f} m off centre"


@pytest.mark.parametrize("spec", [
    ((0.6, 1.0), (3.2, -0.9), (5.8, 0.6)),       # alternating, 2.6 m apart
    ((1.5, 0.0),),                               # one, in the middle
    ((1.6, 0.75), (4.1, -0.75)),                 # staggered 2.5 m (failed in SITL before)
])
def test_exit_corridor_with_drone_like_wall_depth(node, monkeypatch, spec):
    node[0]._measured_len = 22 * FT
    boxes = boxes_at(*spec)
    world = RealWorld((0.0, 11.5 * FT), length=22 * FT, wall_height=8 * FT,
                      banner_h=8 * FT, pose=(-10.0, 0.0, 3.0, 0.0), boxes=boxes,
                      wall_depth_max=3.5, width_prior=11.5 * FT, gps_drift_per_m=0.2 / 17.0)
    ok, msg = fly(node, world, monkeypatch, exit_corridor=True)
    assert ok, msg
    pts = inside(world)
    assert pts and world.x >= world.length
    hw = node[0]._follow.drone_width_m / 2
    assert max(abs(y) for x, y, a in pts) < 11.5 * FT / 2 - hw    # off the walls
    for x0, x1, y0, y1, _ in boxes:
        beside = [y for x, y, a in world.track if x0 - hw <= x <= x1 + hw]
        assert beside, "no track beside an obstacle"
        gap = min(max(y0 - y, y - y1) for y in beside)
        assert gap > hw, f"within {gap:.2f} m of the obstacle at {x0:.1f} m"


def test_exit_corridor_blocked_obstacle_is_not_climbed(node, monkeypatch):
    # A wide obstacle leaves no gap either side: the leg holds (no climbing
    # over) and fails BLOCKED rather than flying into it.
    boxes = [(2.0, 2.4, -1.4, 1.4, OBSTACLE_TOP)]
    world = RealWorld((0.0, 11.5 * FT), length=22 * FT, wall_height=8 * FT,
                      banner_h=8 * FT, pose=(-10.0, 0.0, 3.0, 0.0), boxes=boxes)
    ok, msg = fly(node, world, monkeypatch, exit_corridor=True)
    assert not ok and msg.startswith("BLOCKED"), msg
    assert max(x for x, y, a in world.track) < 2.0               # stopped short
    assert max(a for x, y, a in world.track if x > -2.5) < 4 * FT + 0.25


def test_blue_floor_marker_does_not_end_a_leg_with_no_length(node, monkeypatch):
    # A blue exit marker under the vehicle used to crash the leg (no length
    # to compare with); with no known length only the wall end counts.
    node[0]._marker = "blue"
    world = World([(0.0, 3.5)], length=6.0, banner_h=3.0, pose=(-8.0, 0.0, 2.0, 0.0))
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    assert "walls ended" in msg


def test_no_banner_anywhere_fails(node, monkeypatch):
    world = World([(0.0, 2.0)], length=6.0, banner_h=3.0, pose=(-8.0, 0.0, 2.0, 0.0))
    world.banner = lambda: dict(state=0, bearing=NAN, bearing_left=NAN, bearing_right=NAN,
                                elev_bottom=NAN, elev_top=NAN, dist=NAN, area_frac=NAN,
                                cam_alt=world.alt, cam_yaw=world.yaw, elev_bottom_left=NAN,
                                elev_bottom_right=NAN)
    ok, msg = fly(node, world, monkeypatch)
    assert not ok and msg.startswith("NO_BANNER"), msg
    alts = {round(a, 1) for x, y, a in world.track}
    assert {1.0, 1.5, 2.5, 3.0} <= alts                     # tried every search altitude


# ── Banner search sweep (+-half angle around the placed heading) ──────────

def test_sweep_goes_left_first_then_right_then_alternates():
    p = EntryParams(search_half_angle_deg=90.0)
    h = math.pi / 2
    assert sweep_points(0, p) == [0.0, h, -h]
    assert sweep_points(1, p) == [-h, h]          # crosses back from the right edge
    assert sweep_points(2, p) == [h, -h]
    assert sweep_points(3, p) == [-h, h]


def test_sweep_never_looks_behind():
    p = EntryParams(search_half_angle_deg=90.0)
    rate = math.radians(20.0)
    for i in range(4):
        pts = sweep_points(i, p)
        dur = sweep_duration(pts, rate)
        for k in range(0, 101):
            a = sweep_angle(pts, rate, dur * k / 100)
            assert abs(a) <= math.pi / 2 + 1e-9


def test_sweep_timing_and_path():
    p = EntryParams(search_half_angle_deg=90.0)
    rate = math.radians(20.0)
    first = sweep_points(0, p)
    assert sweep_duration(first, rate) == pytest.approx(270.0 / 20.0)
    assert sweep_duration(sweep_points(1, p), rate) == pytest.approx(180.0 / 20.0)
    assert math.degrees(sweep_angle(first, rate, 0.0)) == pytest.approx(0.0)
    assert math.degrees(sweep_angle(first, rate, 2.0)) == pytest.approx(40.0)    # turning left
    assert math.degrees(sweep_angle(first, rate, 4.5)) == pytest.approx(90.0)    # left edge
    assert math.degrees(sweep_angle(first, rate, 9.0)) == pytest.approx(0.0)     # back through centre
    assert math.degrees(sweep_angle(first, rate, 13.5)) == pytest.approx(-90.0)  # right edge
    assert math.degrees(sweep_angle(first, rate, 99.0)) == pytest.approx(-90.0)  # stays there


def test_sweep_of_180_or_more_is_a_full_turn():
    p = EntryParams(search_half_angle_deg=180.0)
    assert sweep_points(0, p) == [0.0, 2 * math.pi]
    assert sweep_points(3, p) == [0.0, 2 * math.pi]


# ── Banner dropouts during the approach (as banner_node's tracker does) ───

class DropoutWorld(World):
    """World whose banner drops out after its first lock, for `window`
    seconds (clock time) starting `after` s later. HOLD = banner_node's
    tracker coasting: the LAST lock's pixel position (so its bearing and
    elevation) re-sent with the CURRENT heading. NONE = not seen at all.
    window None: never comes back."""

    def __init__(self, clock, mode, after, window, *args):
        super().__init__(*args)
        self.clock, self.mode, self.after, self.window = clock, mode, after, window
        self.first_lock = self.last = None
        self.hold_yaws = []                   # commanded yaws during HOLD

    def banner(self):
        out = super().banner()
        t = self.clock.time()
        if out["state"] == 2 and self.first_lock is None:
            self.first_lock = t
        if self.first_lock is not None:
            start = self.first_lock + self.after
            if t >= start and (self.window is None or t < start + self.window):
                if self.mode == "hold" and self.last is not None:
                    stale = dict(self.last, state=1, cam_alt=self.alt, cam_yaw=self.yaw)
                    if self.target is not None:
                        self.hold_yaws.append(self.target[3])
                    return stale
                return dict(out, state=0, bearing=NAN, elev_bottom=NAN,
                            bearing_left=NAN, bearing_right=NAN,
                            elev_bottom_left=NAN, elev_bottom_right=NAN)
        if out["state"] == 2:
            self.last = dict(out)
        return out


def test_coasting_banner_does_not_steer_the_approach(node, monkeypatch):
    # Banner ~25 deg to the right: the drone turns to it, and while turning
    # the tracker coasts (HOLD) for 1.5 s with stale angles. Steering on
    # them chased the banner out of view in SITL; now the heading is held.
    n, clock = node
    world = DropoutWorld(clock, "hold", 0.2, 1.5, [(0.0, 3.5)], 6.0, 3.0,
                         (-8.0, 2.0, 2.0, math.radians(-10.0)))
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    assert world.hold_yaws and max(world.hold_yaws) - min(world.hold_yaws) < 1e-9
    assert inside(world) and world.x >= world.length


def test_banner_lost_for_a_while_is_searched_for_again(node, monkeypatch):
    # Not seen at all for 6 s (> research_after_s 5 s) after the first lock:
    # one sweep around where it was finds it, and the approach carries on.
    n, clock = node
    world = DropoutWorld(clock, "none", 0.2, 6.0, [(0.0, 3.5)], 6.0, 3.0,
                         (-9.0, 0.5, 2.0, 0.0))
    ok, msg = fly(node, world, monkeypatch)
    assert ok, msg
    assert inside(world) and world.x >= world.length


def test_banner_lost_for_good_fails_instead_of_hovering(node, monkeypatch):
    n, clock = node
    world = DropoutWorld(clock, "none", 0.2, None, [(0.0, 3.5)], 6.0, 3.0,
                         (-9.0, 0.5, 2.0, 0.0))
    t0 = clock.time()
    ok, msg = fly(node, world, monkeypatch)
    assert not ok and "lost during the approach" in msg
    assert clock.time() - t0 < 60.0          # well before the 90 s approach timeout
