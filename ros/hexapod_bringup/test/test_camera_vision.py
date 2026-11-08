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
