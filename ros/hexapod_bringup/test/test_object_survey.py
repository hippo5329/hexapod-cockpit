"""object_survey's note-keeping without ROS: merging sightings, confirming objects, when to survey."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import object_survey as osv  # noqa: E402


def test_same_label_near_merges_far_does_not():
    om = osv.ObjectMap(merge_m=0.5)
    a, _ = om.add("bottle", 0.5, 1.0, 1.0, 0, "walk")
    b, _ = om.add("bottle", 0.5, 1.3, 1.0, 1, "survey 1")
    assert a is b and a["sightings"] == 2 and a["x"] == pytest.approx(1.15)
    c, _ = om.add("bottle", 0.5, 2.0, 1.0, 2, "survey 1")  # 0.85 m from the mean: another bottle
    d, _ = om.add("cup", 0.5, 1.15, 1.0, 3, "survey 1")    # another label at the same spot: another object
    assert len({id(o) for o in (a, c, d)}) == 3 and len(om.objs) == 3


def test_furniture_merges_wider():
    om = osv.ObjectMap(merge_m=0.5, merge_large_m=0.9)
    a, _ = om.add("couch", 0.6, 0.0, 0.0, 0, "walk")
    b, _ = om.add("couch", 0.6, 0.8, 0.0, 1, "walk")
    assert a is b


def test_position_is_score_weighted_and_best_kept():
    om = osv.ObjectMap()
    o, top = om.add("chair", 0.9, 0.0, 0.0, 0, "walk", robot=(1, 2, 3), dist=1.5)
    assert top
    o, top = om.add("chair", 0.3, 0.4, 0.0, 1, "walk", robot=(4, 5, 6), dist=0.9)
    assert not top and o["x"] == pytest.approx(0.1) and o["best_robot"] == (1, 2, 3) and o["nearest"] == 0.9


def test_confirmed_by_views_frames_or_score():
    om = osv.ObjectMap()
    one, _ = om.add("book", 0.5, 0, 0, 0, "walk")
    assert not om.confirmed(one)
    om.add("book", 0.5, 0, 0, 1, "survey 1")
    assert om.confirmed(one)  # two views
    three, _ = om.add("cup", 0.5, 5, 5, 0, "walk")
    om.add("cup", 0.5, 5, 5, 0, "walk")
    assert not om.confirmed(three)
    om.add("cup", 0.5, 5, 5, 0, "walk")
    assert om.confirmed(three)  # three frames
    sure, _ = om.add("tv", 0.75, 9, 9, 0, "walk")
    assert om.confirmed(sure) and [o["label"] for o in om.confirmed_objects()] == ["book", "cup", "tv"]


def test_schedule_first_then_by_travel_and_gap():
    s = osv.Schedule(every_m=1.5, min_gap_s=60)
    assert s.due(0)
    s.done(0)
    for i in range(20):
        s.moved(0.1 * i, 0)  # 1.9 m
    assert not s.due(30)     # too soon
    assert s.due(61)
    s.done(61)
    assert not s.due(200)    # no travel since
    assert not osv.Schedule(every_m=0).due(0)


def test_keep_sighting():
    ok = {"label": "cup", "score": 0.6, "x": 1.0, "y": 0.0, "dist": 1.0, "clipped": False, "from": "depth"}
    assert osv.keep_sighting(ok)
    assert not osv.keep_sighting({**ok, "score": 0.41})
    assert not osv.keep_sighting({**ok, "dist": 3.5})
    assert not osv.keep_sighting({k: v for k, v in ok.items() if k != "x"})
    assert not osv.keep_sighting({**ok, "clipped": True, "from": "floor contact"})
    assert osv.keep_sighting({**ok, "clipped": True})  # by depth: its own pixels place it


def test_markdown_lists_confirmed_and_surveys():
    om = osv.ObjectMap()
    om.add("chair", 0.8, 1.234, -0.5, 0, "survey 1", dist=1.1)
    om.add("cup", 0.5, 3, 3, 0, "walk")
    md = osv.markdown(om, [{"t": 0, "x": 0.0, "y": 0.0, "steps": 8, "headings": 8, "seen": {"chair": 3}, "note": ""}],
                      "Objects", 0, 4.2)
    assert "| 1 | chair | - | +1.23 | -0.50 | 1 | 1 | 0.80 | 1.10 m |" in md
    assert "1 object(s) confirmed, 1 unconfirmed" in md and "8 of 8 headings: chair x3" in md and "cup |" not in md


def test_rooms_in_the_home_frame():
    # home at map (2, 1) facing +y: the home frame's +x is the map's +y
    import math
    rooms = osv.Rooms([{"name": "study", "polygon": [[-1, -1], [1, -1], [1, 1], [-1, 1]]},
                       {"name": "gym", "polygon": [[2, -1], [4, -1], [4, 1], [2, 1]]}], (2.0, 1.0, math.pi / 2))
    assert osv.to_home(2.0, 4.0, rooms.home) == pytest.approx((3.0, 0.0))
    assert osv.from_home(3.0, 0.0, rooms.home) == pytest.approx((2.0, 4.0))
    assert rooms.of(2.2, 1.3) == "study" and rooms.of(2.0, 4.0) == "gym" and rooms.of(6.0, 1.0) == ""


def test_markdown_by_room():
    import math
    om = osv.ObjectMap()
    om.add("chair", 0.8, 0.5, 0.0, 0, "survey 1")
    om.add("tv", 0.8, 3.0, 0.0, 0, "survey 1")
    om.add("cup", 0.8, 9.0, 9.0, 0, "survey 1")
    rooms = osv.Rooms([{"name": "study", "polygon": [[-1, -1], [1, -1], [1, 1], [-1, 1]]},
                       {"name": "gym", "polygon": [[2, -1], [4, -1], [4, 1], [2, 1]]}])
    md = osv.markdown(om, [{"t": 0, "x": 3.0, "y": 0.0, "steps": 8, "headings": 8, "seen": {}, "note": ""}], "O", 0, 0,
                      rooms)
    assert "| chair | study |" in md and "| cup | - |" in md
    assert "- **study**: chair" in md and "- **gym**: tv" in md and "- **elsewhere**: cup" in md
    assert "in the gym, 8 of 8" in md


def test_frame_pose_exact_then_latest_never_far():
    held = {10.0: (1.0, 2.0, 0.5, 10.0)}
    exact = lambda st: held.get(st)
    latest = lambda: (3.0, 4.0, 0.1, 9.85)
    assert osv.pose_for_frame(exact, latest, 10.0) == (1.0, 2.0, 0.5, 10.0)
    # newer than the buffer (the usual case: odom TF lags the camera): the latest, if close
    assert osv.pose_for_frame(exact, latest, 10.07) == (3.0, 4.0, 0.1, 9.85)
    assert osv.pose_for_frame(exact, latest, 10.5) is None
    assert osv.pose_for_frame(exact, lambda: None, 10.07) is None
