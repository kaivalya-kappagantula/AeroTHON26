# corridor_render.py — ray-cast depth images of a walled corridor, and colour
# images of its green entrance banner.
#
# Test/SITL helper (pure NumPy): renders what a forward depth camera sees in a
# straight corridor with box obstacles, so corridor_geometry can be checked
# against known ground truth and the planner can be flown in SITL without
# Gazebo. render_colour gives the colour camera's view for banner_node: the
# banner green, everything else grey (the corridor's own colours are not
# modelled). Not used on the vehicle.
#
# World frame: x along the corridor, y LEFT, z up, ground at z = 0. The corridor
# spans x in [x_start, x_end] with walls at y = +-width/2.

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Scene:
    width: float = 1.524
    x_start: float = 2.0
    x_end: float = 12.0
    wall_height: float = 3.0
    # boxes: (x_min, x_max, y_min, y_max, z_top)
    boxes: list = field(default_factory=list)


@dataclass
class Camera:
    fx: float = 457.4
    fy: float = 457.4
    cx: float = 327.3
    cy: float = 240.3
    width: int = 640
    height: int = 480
    max_range: float = 15.0


@dataclass
class Banner:
    """Flat banner in the plane x = x (seen from both sides), spanning y in
    [y_min, y_max] and z in [z_bottom, z_top]. Default: the competition's
    11.5 x 1.5 ft banner sitting on top of 8 ft walls at the mouth."""
    x: float = 0.0
    y_min: float = -1.7526
    y_max: float = 1.7526
    z_bottom: float = 2.4384
    z_top: float = 2.8956


# OAK-D Lite colour camera at 640x360 (HFOV 69 deg, VFOV 42.4 deg). The long
# max_range only sets how far the scene still hides the banner.
COLOUR_CAMERA = Camera(fx=465.0, fy=465.0, cx=319.5, cy=179.5, width=640, height=360,
                       max_range=60.0)

SKY_BGR = (200, 185, 170)
SURFACE_BGR = (120, 125, 125)       # walls, ground, boxes
BANNER_BGR = (60, 170, 40)          # cloth green (as test_banner.py)


def _rays(cam: Camera, yaw, pitch, roll):
    """Ray per pixel in the world frame, scaled so its optical-axis component
    is 1 (a ray parameter is then the depth along the optical axis)."""
    u, v = np.meshgrid(np.arange(cam.width), np.arange(cam.height))
    xc = (u - cam.cx) / cam.fx
    yc = (v - cam.cy) / cam.fy
    cp, sp, cy_, sy_ = np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    fwd = np.array([cp * cy_, cp * sy_, sp])
    left = np.array([-sy_, cy_, 0.0])
    up = np.array([-sp * cy_, -sp * sy_, cp])
    cr, sr = np.cos(roll), np.sin(roll)
    left, up = cr * left + sr * up, -sr * left + cr * up
    return fwd[None, None, :] - xc[..., None] * left - yc[..., None] * up


def banner_depth(banner: Banner, cam: Camera, pos, yaw, pitch=0.0, roll=0.0):
    """Depth to the banner along the optical axis per pixel, inf where the
    pixel does not see it (the scene is not checked: see render_colour)."""
    d = _rays(cam, yaw, pitch, roll)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (banner.x - pos[0]) / d[..., 0]
        y = pos[1] + t * d[..., 1]
        z = pos[2] + t * d[..., 2]
        ok = np.isfinite(t) & (t > 1e-6) & (y >= banner.y_min) & (y <= banner.y_max) & \
            (z >= banner.z_bottom) & (z <= banner.z_top)
    return np.where(ok, t, np.inf)


def render_colour(scene: Scene, banner: Banner, cam: Camera, pos, yaw, pitch=0.0, roll=0.0):
    """Colour image (H x W x 3 uint8, BGR): the banner where nothing in the
    scene is nearer, grey surfaces, pale sky where the scene has no return."""
    z = render(scene, cam, pos, yaw, pitch, roll)
    tb = banner_depth(banner, cam, pos, yaw, pitch, roll)
    img = np.empty(z.shape + (3,), np.uint8)
    img[:] = SURFACE_BGR
    img[np.isnan(z)] = SKY_BGR
    img[tb < np.nan_to_num(z, nan=np.inf)] = BANNER_BGR
    return img


def render(scene: Scene, cam: Camera, pos, yaw, pitch=0.0, roll=0.0):
    """Depth image (metres along the optical axis, NaN = no return).

    pos (x, y, z) camera position in the world, yaw CCW from +x, pitch
    nose-up, roll right-wing-down.
    """
    d = _rays(cam, yaw, pitch, roll)
    px, py, pz = pos
    t = np.full(d.shape[:2], np.inf)

    def hit_plane(axis, value, lo, hi):
        """Intersect with the plane coord[axis] = value, bounded on the other
        two axes by lo/hi dicts {axis: (min, max)}."""
        nonlocal t
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (value - pos[axis]) / d[..., axis]
        ok = np.isfinite(tt) & (tt > 1e-6)
        for ax, (a, b) in lo.items():
            c = pos[ax] + tt * d[..., ax]
            ok &= (c >= a) & (c <= b)
        t = np.where(ok & (tt < t), tt, t)

    W2 = scene.width / 2
    for wy in (W2, -W2):
        hit_plane(1, wy, {0: (scene.x_start, scene.x_end),
                          2: (0.0, scene.wall_height)}, None)
    hit_plane(2, 0.0, {}, None)                                  # ground
    for (x0, x1, y0, y1, ztop) in scene.boxes:
        for xv in (x0, x1):
            hit_plane(0, xv, {1: (y0, y1), 2: (0.0, ztop)}, None)
        for yv in (y0, y1):
            hit_plane(1, yv, {0: (x0, x1), 2: (0.0, ztop)}, None)
        hit_plane(2, ztop, {0: (x0, x1), 1: (y0, y1)}, None)

    z = t                                    # d has optical-z component 1
    z[(z > cam.max_range)] = np.nan
    z[~np.isfinite(z)] = np.nan
    return z.astype(np.float32)
