# Green banner detector/tracker (banner.py, green_detect.py's logic) on
# synthetic frames, the colour-ray geometry banner_node uses, and the colour
# decoder.
import math
import types

import numpy as np
import pytest

from obstacle_avoidance import corridor_geometry as cg
from obstacle_avoidance.banner import BannerParams, Detector, Tracker
from obstacle_avoidance.corridor_render import BANNER_BGR as SIM_BANNER_BGR
from obstacle_avoidance.corridor_render import (
    COLOUR_CAMERA,
    Banner,
    Scene,
    banner_depth,
    render_colour,
)
from obstacle_avoidance.depth_image import DepthDecodeError, decode_bgr

W, H = 640, 360
BANNER_BGR = (60, 170, 40)          # saturated cloth green


def frame(seed=0):
    """Grey concrete-ish background with mild noise."""
    rng = np.random.default_rng(seed)
    img = np.full((H, W, 3), 128, np.float32) + rng.normal(0, 6, (H, W, 1))
    return np.clip(img, 0, 255).astype(np.uint8)


def with_banner(img, x0, y0, x1, y1, seed=1):
    rng = np.random.default_rng(seed)
    patch = np.array(BANNER_BGR, np.float32) + rng.normal(0, 3, (y1 - y0, x1 - x0, 1))
    img[y0:y1, x0:x1] = np.clip(patch, 0, 255).astype(np.uint8)
    return img


def with_foliage(img, cx, cy, r, seed=2):
    """Green blob with leaf-like light/shadow texture."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:H, 0:W]
    blob = (xx - cx) ** 2 + (yy - cy) ** 2 < r * r
    shade = rng.uniform(0.35, 1.0, (H, W, 1))
    leaves = np.array((40, 150, 70), np.float32) * shade
    img[blob] = np.clip(leaves[blob], 0, 255).astype(np.uint8)
    return img


def test_flat_banner_detected_and_locked():
    p = BannerParams()
    det, trk = Detector(p), Tracker(p)
    results = []
    for i in range(p.confirm):
        cands, _, _ = det.detect(with_banner(frame(i), 250, 60, 450, 120))
        results.append(trk.update(cands, W, H))
    assert all(r is None for r in results[:-1])          # confirm frames first
    r = results[-1]
    assert r["state"] == "LOCK"
    assert r["cx"] == pytest.approx(350, abs=3)
    x, y, bw, bh = r["bbox"]
    assert y + bh == pytest.approx(120, abs=3)            # bottom edge


def blue_params():
    """banner_blue.yaml's values on top of the defaults."""
    return BannerParams(h_min=100, h_max=130, s_min=80, a_min=0, b_min=15, allow_top=False)


def test_blue_banner_found_by_the_blue_detector_only():
    img = frame()
    img[150:200, 120:520] = (170, 90, 30)               # blue cloth (BGR)
    blue, green = Detector(blue_params()), Detector(BannerParams())
    assert len(blue.detect(img)[0]) == 1
    assert green.detect(img)[0] == []
    green_img = with_banner(frame(), 120, 150, 520, 200)
    assert blue.detect(green_img)[0] == []


def test_blue_sky_on_the_top_edge_is_not_acquired():
    img = frame()
    img[0:90, :] = (230, 160, 90)                       # clear blue sky, top of the view
    tr = Tracker(blue_params())
    det = Detector(blue_params())
    for _ in range(5):
        out = tr.update(det.detect(img)[0], W, H)
    assert out is None
    img[180:230, 120:520] = (170, 90, 30)               # the banner below it is
    for _ in range(5):
        out = tr.update(det.detect(img)[0], W, H)
    assert out is not None and out["state"] == "LOCK"
    assert 180 <= out["cy"] <= 230


def test_foliage_rejected():
    p = BannerParams()
    cands, rejected, _ = Detector(p).detect(with_foliage(frame(), 320, 180, 60))
    assert cands == []
    assert rejected


def test_tracker_coasts_then_drops():
    p = BannerParams()
    det, trk = Detector(p), Tracker(p)
    for i in range(p.confirm):
        trk.update(det.detect(with_banner(frame(i), 250, 60, 450, 120))[0], W, H)
    assert trk.update([], W, H)["state"] == "HOLD"
    for _ in range(p.hold):
        trk.update([], W, H)
    assert trk.update([], W, H) is None


def test_level_rays_centre_and_pitch():
    fx, fy, cx, cy = 500.0, 500.0, 320.0, 180.0
    f, l, u = cg.level_rays([cx], [cy], fx, fy, cx, cy, 0.0)
    assert math.atan2(l[0], f[0]) == pytest.approx(0.0)
    assert math.atan2(u[0], f[0]) == pytest.approx(0.0)
    # Nose up 5 deg: the image centre looks 5 deg above the horizon.
    f, l, u = cg.level_rays([cx], [cy], fx, fy, cx, cy, math.radians(5))
    assert math.atan2(u[0], f[0]) == pytest.approx(math.radians(5))
    # A pixel left of centre has a positive (left) bearing.
    f, l, u = cg.level_rays([cx - fx * math.tan(0.2)], [cy], fx, fy, cx, cy, 0.0)
    assert math.atan2(l[0], f[0]) == pytest.approx(0.2)


# SITL banner: what sitl_corridor_sim draws (full 640x360 colour), through the
# real detector and banner_node's ray geometry.
SIM_CAM = COLOUR_CAMERA
SIM_SCENE = Scene(width=1.524, x_start=0.0, x_end=13.0, wall_height=2.4384)   # 8 ft


def test_sim_banner_detected_with_true_angles():
    banner = Banner()                        # 11.5 x 1.5 ft on top of the 8 ft walls
    pos = (-10.0, 0.0, 2.0)                  # 10 m in front of the mouth, on the centreline
    img = render_colour(SIM_SCENE, banner, SIM_CAM, pos, 0.0)
    p = BannerParams()
    det, trk = Detector(p), Tracker(p)
    for _ in range(p.confirm):
        r = trk.update(det.detect(img)[0], SIM_CAM.width, SIM_CAM.height)
    assert r is not None and r["state"] == "LOCK"
    x0, y0, bw, bh = r["bbox"]
    f, l, u = cg.level_rays([r["cx"], r["cx"]], [r["cy"], y0 + bh], SIM_CAM.fx, SIM_CAM.fy,
                            SIM_CAM.cx, SIM_CAM.cy, 0.0)
    assert math.atan2(l[0], f[0]) == pytest.approx(0.0, abs=math.radians(0.5))
    elev_bottom = math.atan2(u[1], math.hypot(f[1], l[1]))
    assert elev_bottom == pytest.approx(math.atan2(banner.z_bottom - pos[2], 10.0),
                                        abs=math.radians(0.6))


def test_sim_banner_depth_and_wall_occlusion():
    banner = Banner()
    mid = 0.5 * (banner.z_bottom + banner.z_top)
    v, u = int(SIM_CAM.cy), int(SIM_CAM.cx)
    tb = banner_depth(banner, SIM_CAM, (-10.0, 0.0, mid), 0.0)
    assert tb[v, u] == pytest.approx(10.0)
    # A banner hung inside the opening, seen from outside the right wall and
    # aimed at its centre: the wall hides the middle, the part overhanging
    # that wall stays in view.
    banner = Banner(z_bottom=1.9812, z_top=2.4384)
    mid = 0.5 * (banner.z_bottom + banner.z_top)
    pos = (3.0, -5.0, mid)
    yaw = math.atan2(0.0 - pos[1], 0.0 - pos[0])
    tb = banner_depth(banner, SIM_CAM, pos, yaw)
    img = render_colour(SIM_SCENE, banner, SIM_CAM, pos, yaw)
    green = (img == np.array(SIM_BANNER_BGR, np.uint8)).all(axis=2)
    assert np.isfinite(tb[v, u]) and not green[v, u]
    assert green.any()


def test_sim_banner_bottom_corners_seen_from_an_angle():
    # 30 deg off to the side, below the banner: the near (right) end's bottom
    # corner sits higher in the image; both corners' elevations come out
    # within ~0.3 deg (what banner_node publishes as elev_bottom_left/right).
    banner = Banner()
    a = math.radians(30)
    pos = (-7.0 * math.cos(a), -7.0 * math.sin(a), 1.4)
    img = render_colour(SIM_SCENE, banner, SIM_CAM, pos, a)
    p = BannerParams()
    det, trk = Detector(p), Tracker(p)
    for _ in range(p.confirm):
        r = trk.update(det.detect(img)[0], SIM_CAM.width, SIM_CAM.height)
    assert r is not None and not (r["cut_left"] or r["cut_right"] or r["edge"])
    (ul, vl), (ur, vr) = r["bottom_left"], r["bottom_right"]
    f, l, u = cg.level_rays([ul, ur], [vl, vr], SIM_CAM.fx, SIM_CAM.fy, SIM_CAM.cx,
                            SIM_CAM.cy, 0.0)
    elev = [math.atan2(u[i], math.hypot(f[i], l[i])) for i in range(2)]
    true = [math.atan2(banner.z_bottom - pos[2], math.hypot(-pos[0], y - pos[1]))
            for y in (banner.y_max, banner.y_min)]
    assert elev == pytest.approx(true, abs=math.radians(0.3))
    assert elev[1] > elev[0]                                # right end nearer


def _img_msg(arr, encoding, pad=0):
    h, w, ch = arr.shape
    rows = np.zeros((h, w * ch + pad), np.uint8)
    rows[:, :w * ch] = arr.reshape(h, w * ch)
    return types.SimpleNamespace(height=h, width=w, encoding=encoding,
                                 step=w * ch + pad, data=rows.tobytes())


def test_decode_bgr_orders_and_padding():
    bgr = np.random.default_rng(0).integers(0, 255, (4, 5, 3), dtype=np.uint8)
    assert np.array_equal(decode_bgr(_img_msg(bgr, "bgr8", pad=3)), bgr)
    assert np.array_equal(decode_bgr(_img_msg(bgr[..., ::-1].copy(), "rgb8")), bgr)
    with pytest.raises(DepthDecodeError):
        decode_bgr(_img_msg(bgr, "yuv422"))
