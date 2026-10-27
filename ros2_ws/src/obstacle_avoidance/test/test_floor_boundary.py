# RGB wall bases (floor_boundary.py) on a synthetic corridor with plain white
# walls — the case stereo depth cannot see. Same synthetic image as
# corridor_bench.py --sim (mud/grass ground, white walls, sky).
import math

import numpy as np
import pytest

from obstacle_avoidance import corridor_geometry as cg
from obstacle_avoidance import floor_boundary as fb
from obstacle_avoidance.corridor_render import Camera, Scene, render

CAM = Camera()
P = cg.CorridorParams()
FOV = cg.view_limits(CAM.fx, CAM.fy, CAM.cx, CAM.cy, CAM.width)
K = (CAM.fx, CAM.fy, CAM.cx, CAM.cy, CAM.width)


def colour(z, pos, yaw, seed=0, wall_bgr=(228, 228, 232)):
    """BGR image of the rendered scene (corridor_bench SimSource._rgb)."""
    rng = np.random.default_rng(seed)
    u, v = np.meshgrid(np.arange(CAM.width), np.arange(CAM.height))
    xc, yc = (u - CAM.cx) / CAM.fx, (v - CAM.cy) / CAM.fy
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    wx = pos[0] + z * (cy_ + xc * sy_)
    wy = pos[1] + z * (sy_ - xc * cy_)
    wz = pos[2] - z * yc
    n = lambda s: rng.normal(0, s, z.shape)[..., None]      # noqa: E731
    img = np.empty((CAM.height, CAM.width, 3), np.float32)
    img[:] = (235, 206, 160)                                # sky
    ground = (np.isfinite(z) & (wz < 0.02)) | (~np.isfinite(z) & (v > CAM.cy))
    gx, gy = np.nan_to_num(wx), np.nan_to_num(wy)
    grass = (np.sin(3.1 * gx) * np.sin(2.3 * gy + 1) + 0.5 * np.sin(7 * gx + 3 * gy)) > 0.8
    mud = np.array((55, 78, 100), np.float32) + n(18)
    img[ground] = mud[ground]
    g = ground & grass
    img[g] = (np.array((45, 115, 70), np.float32) + n(20))[g]
    wall = np.isfinite(z) & ~ground
    shade = (1.0 - 0.08 * np.clip(wz, 0, 3) / 3)[..., None]
    wimg = np.array(wall_bgr, np.float32) * shade + n(4)
    img[wall] = wimg[wall]
    return np.clip(img, 0, 255).astype(np.uint8)


def fit(width, y0, yaw_deg, cam_height=1.2, known_width=None):
    pos, yaw = (0.0, y0, cam_height), math.radians(yaw_deg)
    z = render(Scene(width=width, x_start=-1.0, x_end=5.7), CAM, pos, yaw)
    r, _, _ = fb.fit_rgb(fb.FloorBoundary(K), colour(z, pos, yaw), P, FOV, 0.0, 0.0,
                         cam_height, width=known_width)
    return r


@pytest.mark.parametrize("width,y0,yaw_deg", [(3.5, 0.3, 0.0), (3.5, -0.4, 8.0),
                                              (1.524, 0.2, 0.0), (2.5, 0.0, -6.0)])
def test_white_walls_found_and_width_measured(width, y0, yaw_deg):
    r = fit(width, y0, yaw_deg)
    assert r["walls"] == 2
    assert r["wall_left"] - r["wall_right"] == pytest.approx(width, rel=0.06)
    assert 0.5 * (r["wall_left"] + r["wall_right"]) == pytest.approx(-y0, abs=0.08)
    assert r["heading_err"] == pytest.approx(-math.radians(yaw_deg), abs=math.radians(2))
    assert all(math.isnan(r[k]) for k in fb.OBS_FIELDS)     # obstacles come from depth


def test_open_ground_is_not_a_corridor():
    pos = (0.0, 0.0, 1.2)
    z = render(Scene(x_start=50.0, x_end=60.0), CAM, pos, 0.0)
    r, _, _ = fb.fit_rgb(fb.FloorBoundary(K), colour(z, pos, 0.0), P, FOV, 0.0, 0.0, 1.2)
    assert r["walls"] == 0
