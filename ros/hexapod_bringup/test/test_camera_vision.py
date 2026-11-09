"""camera_vision's geometry without the models: a floor pixel's position and the free-floor sectors."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import camera_vision as cv  # noqa: E402

K = np.array([[287.9, 0, 160.7], [0, 285.8, 131.7], [0, 0, 1]])
D = np.zeros(5)


def test_ground_point_below_the_centre():
    # level camera 0.16 m up: the row fy * tan(a) below the centre sees the floor at h / tan(a)
    a = math.radians(10)
    g = cv.ground_point(160.7, 131.7 + 285.8 * math.tan(a), K, D, 0.16, 0.0, cam_x=0.05)
    assert g[0] == pytest.approx(0.05 + 0.16 / math.tan(a), rel=1e-6) and g[1] == pytest.approx(0, abs=1e-9)


def test_ground_point_pitch_and_side():
    # looking down 10 deg, the image centre is that same floor point; a pixel right of centre is to the right
    g = cv.ground_point(160.7, 131.7, K, D, 0.16, math.radians(10))
    assert g[0] == pytest.approx(0.16 / math.tan(math.radians(10)), rel=1e-6)
    assert cv.ground_point(250, 200, K, D, 0.16, 0.0)[1] < 0
    assert cv.ground_point(160.7, 100, K, D, 0.16, 0.0) is None  # above the horizon


def test_free_floor_sectors():
    wall = np.array([[0.6, y] for y in np.linspace(-0.1, 0.1, 30)])        # ahead, in the corridor
    side = np.array([[0.5, 0.5 + d] for d in np.linspace(0, 0.05, 30)])    # 45 deg left
    f = cv.free_floor(np.vstack([wall, side]))
    assert f["centre"] == pytest.approx(0.6, abs=1e-6)
    assert f["left"] == pytest.approx(math.hypot(0.5, 0.5), abs=0.01)
    assert f["right"] == 2.5
    assert cv.free_floor(np.zeros((0, 2))) == {"left": 2.5, "centre": 2.5, "right": 2.5}


def test_obstacle_scan_nearest_per_bearing_and_free_elsewhere():
    half = math.radians(30)
    # a slipper 0.6 m dead ahead (30 points), a lone stray pixel at 0.3 m off to the left
    ahead = np.column_stack([np.tile([0.60, 0.61, 0.62], 10), np.linspace(-0.02, 0.02, 30)])  # depth not tied to bearing
    stray = np.array([[0.3, 0.12]])
    r = cv.obstacle_scan(np.vstack([ahead, stray]), half, bins=32)
    mid = r[15:17]
    assert np.all(np.isfinite(mid)) and np.all(np.abs(mid - 0.6) < 0.02)
    assert np.isinf(r[0]) and np.isinf(r[-1])          # free where nothing was seen
    assert np.sum(np.isfinite(r)) <= 3                   # the stray pixel is not an obstacle


def test_obstacle_scan_ignores_beyond_range_and_behind():
    half = math.radians(30)
    far = np.column_stack([np.full(40, 2.0), np.zeros(40)])
    behind = np.column_stack([np.full(40, -0.5), np.zeros(40)])
    assert np.all(np.isinf(cv.obstacle_scan(np.vstack([far, behind]), half, max_range=1.5)))
    assert np.all(np.isinf(cv.obstacle_scan(np.zeros((0, 2)), half)))


def test_nms_per_label_then_across():
    a = ("door", 0.8, (0, 0, 100, 200))
    b = ("door", 0.5, (5, 5, 100, 200))      # the same door, weaker: dropped
    c = ("cabinet", 0.6, (2, 2, 100, 200))   # the same box, another label: kept per label, dropped across
    d = ("towel", 0.3, (150, 10, 180, 30))   # elsewhere: kept
    assert cv.nms([a, b, c, d], 0.5) == [a, c, d]
    assert cv.nms(cv.nms([a, b, c, d], 0.5), 0.85, same_label=False) == [a, d]


def test_label_name():
    assert cv.label_name("Computer Monitor") == "computer_monitor"
    assert cv.label_name("slipper.") == "slipper"
    assert all(cv.label_name(w) == cv.label_name(cv.label_name(w)) for w in cv.HOUSEHOLD.split(","))


def test_implausible_refuses_surfaces_and_wrong_sizes():
    import camera_vision as cv
    W, H, F = 320, 240, 280.0
    # a white wall at 0.27 m filling the frame, named bathtub: a surface, not an object
    assert cv.implausible("bathtub", (0, 0, 319, 239), W, H, F, F, 0.27) == "fills the view"
    # the floor's lower half, the box stopped 4 px short of the left edge (run 10's "bathtub"): still three edges
    assert cv.implausible("bathtub", (4, 144, 319, 239), W, H, F, F, 1.0) == "fills the view"
    # a "refrigerator" 2.5 m wide at 2 m (a white wall), and a "bathtub" 0.3 m long at 0.6 m (a bin)
    assert "at most" in cv.implausible("refrigerator", (10, 0, 10 + 350, 200), W, H, F, F, 2.0)
    assert "at least" in cv.implausible("bathtub", (100, 100, 100 + 140, 200), W, H, F, F, 0.6)
    # a real fridge 0.7 m wide at 2 m, its top cut by the low camera (height only a lower bound): kept
    assert cv.implausible("refrigerator", (100, 0, 100 + 98, 230), W, H, F, F, 2.0) is None
    # a door at 2 m cut top and bottom is two edges, not three; 0.8 m wide: kept
    assert cv.implausible("door", (50, 0, 50 + 112, 239), W, H, F, F, 2.0) is None
    # a slipper 0.25 m long at 0.8 m: kept; a "slipper" 1 m long: refused
    assert cv.implausible("slipper", (120, 180, 120 + 88, 200), W, H, F, F, 0.8) is None
    assert cv.implausible("slipper", (20, 150, 20 + 280, 200), W, H, F, F, 0.8) is not None
    # labels without a size, or no distance: not judged by size
    assert cv.implausible("lamp", (10, 10, 300, 200), W, H, F, F, 1.0) is None
    assert cv.implausible("bathtub", (100, 100, 140, 120), W, H, F, F, None) is None


def test_on_the_floor_refuses_a_tall_label_on_bare_floor_only():
    # run 10's "bathtub": the tiles at the bottom of the frame, nothing raised in the box
    assert cv.on_the_floor("bathtub", 0.92, 0.0).startswith("the floor")
    assert cv.on_the_floor("bed", 0.70, 0.05) is not None
    # a real bathtub or bed side stands in its box; a chair's box is floor between raised legs and a seat
    assert cv.on_the_floor("bathtub", 0.30, 0.55) is None
    assert cv.on_the_floor("bed", 0.65, 0.20) is None
    # low things lie on the floor (a slipper is 2 cm), and labels without a size are not judged
    assert cv.on_the_floor("slipper", 0.95, 0.0) is None
    assert cv.on_the_floor("rug", 0.95, 0.0) is None


def test_obstacles_marks_floor_and_raised_pixels():
    """Depth.obstacles on a synthetic scene (no model): the floor, and a 0.3 m tall, 0.2 m wide board 0.8 m
    ahead. The board's pixels are raised, the floor's are floor, and a box on each reads that way."""
    w, h, height, pitch, cam_x = 320, 240, 0.1575, math.radians(3.0), 0.065
    d = cv.Depth.__new__(cv.Depth)
    d.rise, d.rays, d.key = 0.045, None, None
    cp, sp = math.cos(pitch), math.sin(pitch)
    r = d.camera_rays(K, D, w, h) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]).T
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(r[..., 2] < -1e-3, height / -r[..., 2], 50.0)  # the floor, or far away above the horizon
        tb = (0.8 - cam_x) / r[..., 0]
        zb, yb = height + tb * r[..., 2], tb * r[..., 1]
    board = (tb < t) & (zb >= 0) & (zb <= 0.3) & (np.abs(yb) <= 0.1)
    t = np.where(board, tb, t)
    a, b = 1.0, 0.1
    d.disparity = lambda bgr: a / t + b
    pts, obst, X, Y, fit, floor = d.obstacles(np.zeros((h, w, 3), np.uint8), K, D, height, pitch, 0.0, cam_x)
    tall = board & (zb > 0.05)  # the board's lowest 4.5 cm is climb height: floor, rightly
    assert fit and obst[tall].all() and not floor[tall].any()
    # a box over the bottom-right floor (run 10's corner "bathtub"), and a box around the board
    corner = (slice(200, 240), slice(230, 320))
    assert floor[corner].mean() > 0.95 and obst[corner].mean() == 0
    assert cv.on_the_floor("bathtub", floor[corner].mean(), obst[corner].mean()) is not None
    v, u = np.nonzero(board)
    around = (slice(v.min(), v.max() + 1), slice(u.min(), u.max() + 1))
    assert cv.on_the_floor("bathtub", floor[around].mean(), obst[around].mean()) is None
