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
