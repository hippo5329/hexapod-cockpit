#!/usr/bin/env python3
"""While exploring, turn around now and then to label what the camera sees, and keep a note of it.

The camera looks ahead only (about 58 degrees across), and walking it sees what the frontiers point at. So
every survey_every_m of travel (and once at the start) the survey pauses the explorer (explore/resume false: it
cancels its goal), turns a full circle in `steps` turns of Nav2's Spin behaviour (it checks the costmap for
room to turn), and at each heading stands still for `dwell_s` while camera_vision labels the frame
(NanoDet-Plus, COCO's 80 classes, the distance from depth). Then it resumes the explorer, facing the way it
was going. It also takes what the camera labels while walking.

Each sighting is put on the map (camera/vision's position in base_footprint, through TF at the frame's
stamp) and merged by place: with the nearest object it has been named as within merge_m (merge_large_m for
furniture), else with the nearest object of any name within merge_any_m. An open-vocabulary detector names
one thing differently frame to frame (a white bin: bathtub, basket, trash can), so an object's label is the
name with the most score summed over its sightings, the others kept as "also seen as". People and pets
(MOBILE) are not objects. An object is confirmed once seen in two surveys, or at two headings of the
surveys with confirm_score or more; sightings while walking (every frame, blurred) place and name it but
never confirm it alone (2026-11-08: frames confirmed 1104 "objects" in one run). The note, rewritten after
every survey and at the end:
  <note_prefix>.md     the confirmed objects (label, also seen as, room, map x y, sightings, views, best score,
                       nearest seen),
                       then each survey (when, where, what it saw), then the unconfirmed count
  <note_prefix>.json   every object, confirmed or not, and the surveys
  <note_prefix>.png    the map with the confirmed objects and the robot's path
  <note_prefix>_img/   the labelled camera frame of each object's best sighting, and one frame of every survey
                       heading (survey<N>_h<k>.jpg: what the detector named there, to judge its names by)
and objects/markers (visualization_msgs/MarkerArray, map frame) for RViz.

  camera/vision                std_msgs/String, camera_vision's JSON       in
  camera/vision/image/compressed   its labelled frame                       in
  map                          nav_msgs/OccupancyGrid (for the picture)    in
  explore/resume               std_msgs/Bool                               out
  spin                         nav2_msgs/action/Spin                       client
  objects/markers              visualization_msgs/MarkerArray              out

Parameters:
  note_prefix     /tmp/objects, where the note goes (no extension)
  survey_every_m  1.5 m of travel on the map between surveys (0: no surveys, labels while walking only)
  min_gap_s       60 s, the least time between surveys
  start_delay_s   15 s before the first (the explorer subscribed, the stack settled)
  steps           8: a full circle in 45-degree turns (the camera's view overlaps the next)
  dwell_s         2.0 s at each heading, the first settle_s of it not counted (the gait stands, the frame clears)
  settle_s        0.8 s
  max_range       2.0 m: farther sightings are not kept (the depth reads short past ~1.5 m)
  min_score       0.45, the least score kept (NanoDet's scale; OWLv2's runs lower: 0.2)
  confirm_score   0.7: seen this sure at two headings of the surveys confirms an object (OWLv2: 0.5)
  merge_m         0.5 m; merge_large_m 0.9 m (bed, couch, table, chair and the like); merge_any_m 0.35 m,
                  a sighting of another name joins an object this near
  rooms_file      "", JSON {"rooms": [{"name", "polygon": [[x, y], ...]}]} in the home frame: each object and
                  survey is named by its room, and the note lists the objects room by room
  home_pose       "0,0,0", home on this run's map: x, y (m), yaw (deg), the home frame's origin and heading
"""
import json
import math
import time

LARGE = {"bed", "couch", "sofa", "dining_table", "table", "desk", "chair", "bench", "refrigerator", "tv", "television",
         "toilet", "sink", "oven", "potted_plant", "suitcase", "microwave", "cabinet", "shelf", "bookshelf", "door",
         "window", "curtain", "bathtub", "treadmill", "exercise_bike", "rug", "yoga_mat", "bicycle"}
MOBILE = {"person", "cat", "dog"}  # they move: not a note's objects


class ObjectMap:
    """Sightings in the map frame, merged into objects by place; each object's label by score vote."""

    def __init__(self, merge_m=0.5, merge_large_m=0.9, confirm_score=0.7, merge_any_m=0.35):
        self.merge_m, self.merge_large_m, self.confirm_score = merge_m, merge_large_m, confirm_score
        self.merge_any_m = merge_any_m
        self.objs = []

    def add(self, label, score, x, y, t, view, robot=None, dist=None, heading=None):
        """One sighting (`view` "walk" or "survey N", `heading` the survey's heading index); returns the object (a
        dict) it joined or started, and whether it is its best yet."""
        best, bd = None, math.inf
        for o in self.objs:
            d = math.hypot(o["x"] - x, o["y"] - y)
            if label in o["labels"]:
                r = self.merge_large_m if label in LARGE else self.merge_m
            else:
                r = self.merge_any_m
            if d <= r and d < bd:
                best, bd = o, d
        if best is None:
            best = {"id": len(self.objs) + 1, "label": label, "labels": {}, "x": x, "y": y, "w": 0.0, "sightings": 0,
                    "views": [], "headings": [], "best_score": 0.0, "nearest": None, "first_t": t, "last_t": t,
                    "best_robot": None}
            self.objs.append(best)
        o = best
        w = max(score, 1e-3)  # the position: a score-weighted mean
        o["x"] = (o["x"] * o["w"] + x * w) / (o["w"] + w)
        o["y"] = (o["y"] * o["w"] + y * w) / (o["w"] + w)
        o["w"] += w
        o["labels"][label] = o["labels"].get(label, 0.0) + w
        o["label"] = max(o["labels"], key=o["labels"].get)
        o["sightings"] += 1
        if view not in o["views"]:
            o["views"].append(view)
        if heading is not None and view.startswith("survey"):
            h = f"{view.split()[-1]}/{heading}"
            if h not in o["headings"]:
                o["headings"].append(h)
        o["last_t"] = t
        if dist is not None:
            o["nearest"] = dist if o["nearest"] is None else min(o["nearest"], dist)
        top = score > o["best_score"]
        if top:
            o["best_score"], o["best_robot"] = score, robot
        return o, top

    def confirmed(self, o):
        surveys = {v for v in o["views"] if v.startswith("survey")}
        return len(surveys) >= 2 or (len(o["headings"]) >= 2 and o["best_score"] >= self.confirm_score)

    def confirmed_objects(self):
        return sorted((o for o in self.objs if self.confirmed(o)), key=lambda o: (o["label"], o["x"], o["y"]))


def also_seen(o, top=3):
    """The object's other names, most score first (at most `top`)."""
    rest = sorted((k for k in o.get("labels", {}) if k != o["label"]), key=lambda k: -o["labels"][k])
    return ", ".join(rest[:top]) or "-"


def to_home(x, y, home):
    """A map point in the home frame (home = x, y, yaw in rad on this map: its origin and heading)."""
    dx, dy = x - home[0], y - home[1]
    c, s = math.cos(home[2]), math.sin(home[2])
    return c * dx + s * dy, -s * dx + c * dy


def from_home(x, y, home):
    """A home-frame point on the map (to_home's inverse)."""
    c, s = math.cos(home[2]), math.sin(home[2])
    return home[0] + c * x - s * y, home[1] + s * x + c * y


def inside(poly, x, y):
    """Point in polygon (even-odd)."""
    n, hit = len(poly), False
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


class Rooms:
    """Named rooms as polygons in the home frame (rooms_file, JSON {"rooms": [{"name", "polygon": [[x, y], ...]}]}):
    a map made fresh at home each run moves its frame, home stays. home is the run's home pose on its map."""

    def __init__(self, rooms=(), home=(0.0, 0.0, 0.0)):
        self.rooms, self.home = list(rooms), home

    def of(self, x, y):
        hx, hy = to_home(x, y, self.home)
        for r in self.rooms:
            if inside(r["polygon"], hx, hy):
                return r["name"]
        return ""


class Schedule:
    """When to survey: at the start, then after every_m of travel and min_gap_s since the last."""

    def __init__(self, every_m=1.5, min_gap_s=60.0):
        self.every_m, self.min_gap_s = every_m, min_gap_s
        self.travel, self.last_xy, self.last_t = 0.0, None, None

    def moved(self, x, y):
        if self.last_xy is not None:
            self.travel += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

    def due(self, now):
        if self.every_m <= 0:
            return False
        if self.last_t is None:
            return True
        return self.travel >= self.every_m and now - self.last_t >= self.min_gap_s

    def done(self, now):
        self.travel, self.last_t = 0.0, now


def keep_sighting(o, max_range=2.0, min_score=0.45):
    """camera_vision's object: has a position, near enough, sure enough, not a person or pet; a box cut by the
    frame's bottom edge placed by floor contact is not placed at all (its contact is out of the frame)."""
    if "x" not in o or o.get("score", 0) < min_score or o.get("dist", 1e9) > max_range or o.get("label") in MOBILE:
        return False
    return not (o.get("clipped") and o.get("from") == "floor contact")


def pose_for_frame(exact, latest, stamp, slack=0.3):
    """The robot's (x, y, yaw, t) for a camera frame stamped `stamp`: `exact(stamp)` if the TF buffer holds it,
    else `latest()` if that is within `slack` s of the frame. Never waits: the frame (stamped at receipt, ~70 ms
    old) is newer than the newest odom TF (~165 ms old), and waiting for one in the node's only executor thread
    blocks the very TF it waits for -- no survey saw anything (2026-11-08). Standing in a dwell, the latest
    pose is the frame's. None if neither."""
    p = exact(stamp)
    if p is not None:
        return p
    q = latest()
    return q if q is not None and abs(q[3] - stamp) <= slack else None


def fmt_t(t):
    return time.strftime("%H:%M:%S", time.gmtime(t))


def markdown(om, surveys, title, started, path_m, rooms=None):
    rooms = rooms or Rooms()
    conf = om.confirmed_objects()
    room = lambda o: rooms.of(o["x"], o["y"]) or "-"
    out = [f"# {title}", "",
           f"Started {time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(started))} UTC; {len(surveys)} survey(s); "
           f"{path_m:.1f} m walked; {len(conf)} object(s) confirmed, {len(om.objs) - len(conf)} unconfirmed.", "",
           "## Objects", "", "| # | label | also seen as | room | map x | map y | sightings | views | best score | nearest seen | "
           "first seen |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for o in conf:
        near = f"{o['nearest']:.2f} m" if o["nearest"] is not None else "-"
        out.append(f"| {o['id']} | {o['label']} | {also_seen(o)} | {room(o)} | {o['x']:+.2f} | {o['y']:+.2f} | {o['sightings']} | {len(o['views'])} | "
                   f"{o['best_score']:.2f} | {near} | {fmt_t(o['first_t'])} |")
    if rooms.rooms:
        out += ["", "## By room", ""]
        names = [r["name"] for r in rooms.rooms] + ["-"]
        for name in names:
            here = [o for o in conf if room(o) == name]
            if here:
                labels = {}
                for o in here:
                    labels[o["label"]] = labels.get(o["label"], 0) + 1
                out.append(f"- **{'elsewhere' if name == '-' else name}**: " +
                           ", ".join(f"{k}" + (f" x{v}" if v > 1 else "") for k, v in sorted(labels.items())))
    out += ["", "## Surveys", ""]
    for s in surveys:
        seen = ", ".join(f"{k} x{v}" for k, v in sorted(s["seen"].items())) or "nothing"
        where = rooms.of(s["x"], s["y"])
        out.append(f"- {fmt_t(s['t'])} at map ({s['x']:+.2f}, {s['y']:+.2f}){' in the ' + where if where else ''}, {s['headings']} of {s['steps']} headings"
                   f"{' (' + s['note'] + ')' if s.get('note') else ''}: {seen}")
    return "\n".join(out) + "\n"


def draw_map(grid, om, path, scale=4, rooms=None):
    """The map (grid: width, height, resolution, origin x y, data) with the rooms, the confirmed objects and the
    path; BGR."""
    import cv2
    import numpy as np
    w, h, res, ox, oy, data = grid
    a = np.array(data, np.int16).reshape(h, w)
    img = np.full((h, w), 205, np.uint8)
    img[(a >= 0) & (a < 25)] = 255
    img[a > 65] = 0
    img[(a >= 25) & (a <= 65)] = 128
    img = cv2.resize(np.flipud(img), (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    px = lambda x, y: (int((x - ox) / res * scale), int((h - (y - oy) / res) * scale))
    for r in (rooms.rooms if rooms else ()):
        pts = np.array([px(*from_home(x, y, rooms.home)) for x, y in r["polygon"]], np.int32)
        cv2.polylines(img, [pts], True, (60, 170, 60), 1, cv2.LINE_AA)
        cx, cy = pts.mean(axis=0)
        cv2.putText(img, r["name"], (int(cx) - 4 * len(r["name"]), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 130, 40), 1,
                    cv2.LINE_AA)
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        cv2.line(img, px(x0, y0), px(x1, y1), (200, 120, 0), 1, cv2.LINE_AA)
    for o in om.confirmed_objects():
        p = px(o["x"], o["y"])
        cv2.circle(img, p, 5, (0, 0, 220), -1, cv2.LINE_AA)
        cv2.putText(img, f"{o['id']} {o['label']}", (p[0] + 7, p[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 160), 1,
                    cv2.LINE_AA)
    return img


def main():
    import os
    import signal
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.time import Time
    import tf2_ros
    from nav2_msgs.action import Spin
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
    from visualization_msgs.msg import Marker, MarkerArray

    class Survey(Node):
        def __init__(self):
            super().__init__("object_survey")
            p = lambda n, d: self.declare_parameter(n, d).value
            self.prefix = p("note_prefix", "/tmp/objects")
            self.sched = Schedule(float(p("survey_every_m", 1.5)), float(p("min_gap_s", 60.0)))
            self.start_delay_s = float(p("start_delay_s", 15.0))
            rooms_file, home = p("rooms_file", ""), [float(v) for v in str(p("home_pose", "0,0,0")).split(",")]
            self.rooms = Rooms(json.load(open(rooms_file))["rooms"] if rooms_file else (),
                               (home[0], home[1], math.radians(home[2])))
            self.steps, self.dwell_s, self.settle_s = int(p("steps", 8)), float(p("dwell_s", 2.0)), float(p("settle_s", 0.8))
            self.max_range, self.min_score = float(p("max_range", 2.0)), float(p("min_score", 0.45))
            self.om = ObjectMap(float(p("merge_m", 0.5)), float(p("merge_large_m", 0.9)), float(p("confirm_score", 0.7)),
                                float(p("merge_any_m", 0.35)))
            self.started, self.surveys, self.path, self.path_m = time.time(), [], [], 0.0
            self.grid, self.frames = None, {}  # stamp -> labelled jpeg (the last few)
            self.pending = {}  # stamp -> objects whose best sighting is that frame, not yet received
            self.state, self.cur = "walk", None  # walk | pause | dwell | turn | resume
            self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
            tf2_ros.TransformListener(self.tf, self)
            self.resume_pub = self.create_publisher(Bool, "explore/resume", 10)
            self.marker_pub = self.create_publisher(MarkerArray, "objects/markers", 10)
            self.spin = ActionClient(self, Spin, "spin")
            self.goal = None
            self.create_subscription(String, "camera/vision", self.on_vision, 10)
            self.create_subscription(CompressedImage, "camera/vision/image/compressed", self.on_frame, qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid, "map", self.on_map,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            os.makedirs(self.prefix + "_img", exist_ok=True)
            self.create_timer(0.2, self.tick)
            self.get_logger().info(f"object survey: every {self.sched.every_m} m, {self.steps} headings, note {self.prefix}.md")

        # ---------------------------------------------------------------- inputs
        def tf_pose(self, at):
            """map -> base_footprint at `at` (a Time; Time() the latest) as (x, y, yaw, t); None if not held."""
            try:
                t = self.tf.lookup_transform("map", "base_footprint", at)
            except tf2_ros.TransformException:
                return None
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            st = t.header.stamp
            return t.transform.translation.x, t.transform.translation.y, yaw, st.sec + st.nanosec * 1e-9

        def robot_xy(self):
            p = self.tf_pose(Time())
            return p[:3] if p else None

        def on_map(self, m):
            i = m.info
            self.grid = (i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y, m.data)

        def on_frame(self, m):
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.frames[stamp] = bytes(m.data)
            while len(self.frames) > 12:
                del self.frames[min(self.frames)]
            # one labelled frame of each survey heading, once it has settled
            if self.state == "dwell" and time.time() >= self.cur["dwell_from"] and not self.cur.get("framed"):
                self.cur["framed"] = True
                name = f"survey{len(self.surveys) + 1}_h{self.cur['step']}.jpg"
                with open(os.path.join(self.prefix + "_img", name), "wb") as f:
                    f.write(self.frames[stamp])
            # camera_vision publishes a frame just after its JSON: the sightings it carried wait for it here
            for k in [k for k in self.pending if abs(k - stamp) < 0.005]:
                for obj in self.pending.pop(k):
                    if abs(obj.get("best_stamp", -1.0) - stamp) < 0.005:  # still its best
                        self.save_frame(obj, stamp)

        def save_frame(self, obj, stamp):
            obj["img"] = f"{obj['label']}_{obj['id']}.jpg"
            with open(os.path.join(self.prefix + "_img", obj["img"]), "wb") as f:
                f.write(self.frames[stamp])

        def on_vision(self, m):
            try:
                v = json.loads(m.data)
            except ValueError:
                return
            if self.state == "dwell":
                if time.time() < self.cur["dwell_from"]:
                    return  # still settling: the frame may be blurred, the pose still moving
                view = f"survey {len(self.surveys) + 1}"
            elif self.state == "walk":
                view = "walk"
            else:
                return  # turning: blurred
            stamp = v.get("stamp", 0.0)
            objs = [o for o in v.get("objects", []) if keep_sighting(o, self.max_range, self.min_score)]
            if not objs:
                return
            pose = pose_for_frame(lambda st: self.tf_pose(Time(nanoseconds=int(st * 1e9))),
                                  lambda: self.tf_pose(Time()), stamp)
            if pose is None:
                return
            tx, ty, yaw = pose[:3]
            c, s = math.cos(yaw), math.sin(yaw)
            for o in objs:
                x, y = tx + c * o["x"] - s * o["y"], ty + s * o["x"] + c * o["y"]
                heading = self.cur["step"] if self.state == "dwell" else None
                obj, top = self.om.add(o["label"], o["score"], x, y, time.time(), view, (tx, ty, yaw), o.get("dist"),
                                       heading)
                if self.state == "dwell":
                    self.cur["seen"][o["label"]] = self.cur["seen"].get(o["label"], 0) + 1
                if top:
                    obj["best_stamp"] = stamp
                    near = min(self.frames, key=lambda k: abs(k - stamp), default=None)
                    if near is not None and abs(near - stamp) < 0.005:  # the same frame (floats: not ==)
                        self.save_frame(obj, near)
                    else:
                        self.pending.setdefault(stamp, []).append(obj)
            while len(self.pending) > 12:
                del self.pending[min(self.pending)]

        # ---------------------------------------------------------------- the survey
        def explore(self, on):
            for _ in range(3):  # best effort somewhere on the way: say it three times (idempotent)
                self.resume_pub.publish(Bool(data=on))

        def tick(self):
            now = time.time()
            pose = self.robot_xy()
            # Travel in 5 cm steps: summed every tick, the SLAM pose's millimetres of jitter would add up to a
            # survey's worth of travel in minutes of standing.
            if pose and self.state == "walk":
                d = math.hypot(pose[0] - self.path[-1][0], pose[1] - self.path[-1][1]) if self.path else 1.0
                if d > 0.05:
                    if self.path:
                        self.path_m += d
                    self.path.append((pose[0], pose[1]))
                    self.sched.moved(pose[0], pose[1])
            if self.state == "walk" and pose and now - self.started >= self.start_delay_s and self.sched.due(now):
                self.get_logger().info(f"survey {len(self.surveys) + 1}: pausing explore at ({pose[0]:+.2f}, {pose[1]:+.2f})")
                self.explore(False)
                self.cur = {"t": now, "x": pose[0], "y": pose[1], "steps": self.steps, "headings": 0, "seen": {},
                            "fails": 0, "step": 0, "until": now + 2.0, "note": ""}
                self.state = "pause"  # the explorer's goal cancelled, the gait stands
            elif self.state == "pause" and now >= self.cur["until"]:
                self.dwell(now)
            elif self.state == "dwell" and now >= self.cur["until"]:
                self.cur["headings"] += 1
                self.cur["step"] += 1
                if self.cur["step"] >= self.steps:  # all headings seen: the last turn faces the way it came
                    self.turn()
                    self.cur["last"] = True
                else:
                    self.turn()
            elif self.state == "resume":
                self.finish(now)

        def dwell(self, now):
            self.explore(False)  # again: an explorer that started after the pause was sent must not walk off
            self.state = "dwell"
            self.cur["dwell_from"], self.cur["until"] = now + self.settle_s, now + self.dwell_s
            self.cur["framed"] = False

        def turn(self):
            if not self.spin.wait_for_server(timeout_sec=1.0):
                self.cur["note"] = "no spin server"
                self.state = "resume"
                return
            g = Spin.Goal()
            g.target_yaw = 2 * math.pi / self.steps
            g.time_allowance.sec = 15
            self.state = "turn"
            self.spin.send_goal_async(g).add_done_callback(self.on_goal)

        def on_goal(self, fut):
            self.goal = fut.result()
            if not self.goal.accepted:
                self.turned(False)
                return
            self.goal.get_result_async().add_done_callback(lambda f: self.turned(f.result().status == 4))

        def turned(self, ok):
            self.goal = None
            if not ok:
                self.cur["fails"] += 1
                self.get_logger().warning(f"survey {len(self.surveys) + 1}: turn {self.cur['step']} refused or failed")
            last = self.cur.pop("last", False)
            if self.cur["fails"] >= 2:
                self.cur["note"] = "stopped: no room to turn"
                self.state = "resume"
            elif last:
                self.state = "resume"
            else:
                self.dwell(time.time())

        def finish(self, now):
            self.cur = {k: v for k, v in self.cur.items() if k in ("t", "x", "y", "steps", "headings", "seen", "note")}
            self.surveys.append(self.cur)
            seen = ", ".join(sorted(self.cur["seen"])) or "nothing"
            where = self.rooms.of(self.cur["x"], self.cur["y"])
            self.get_logger().info(f"survey {len(self.surveys)} done{' in the ' + where if where else ''}: "
                                   f"{self.cur['headings']} headings, {seen}; resuming explore")
            self.cur = None
            self.sched.done(now)
            self.state = "walk"
            self.explore(True)
            self.write()

        # ---------------------------------------------------------------- the note
        def write(self):
            import cv2
            conf = self.om.confirmed_objects()
            with open(self.prefix + ".md", "w") as f:
                f.write(markdown(self.om, self.surveys, "Objects seen while exploring", self.started, self.path_m, self.rooms))
            for o in self.om.objs:
                o["room"] = self.rooms.of(o["x"], o["y"])
            with open(self.prefix + ".json", "w") as f:
                json.dump({"started": self.started, "path_m": self.path_m, "objects": self.om.objs,
                           "confirmed": [o["id"] for o in conf], "surveys": self.surveys}, f, indent=1)
            if self.grid:
                cv2.imwrite(self.prefix + ".png", draw_map(self.grid, self.om, self.path, rooms=self.rooms))
            arr = MarkerArray()
            arr.markers.append(Marker(action=Marker.DELETEALL))
            for o in conf:
                for kind, mid in ((Marker.SPHERE, 2 * o["id"]), (Marker.TEXT_VIEW_FACING, 2 * o["id"] + 1)):
                    mk = Marker(type=kind, id=mid, ns="objects")
                    mk.header.frame_id = "map"
                    mk.pose.position.x, mk.pose.position.y = o["x"], o["y"]
                    mk.pose.position.z = 0.05 if kind == Marker.SPHERE else 0.25
                    mk.pose.orientation.w = 1.0
                    mk.scale.x = mk.scale.y = mk.scale.z = 0.1 if kind == Marker.SPHERE else 0.12
                    mk.color.r, mk.color.a = 1.0, 1.0
                    if kind == Marker.TEXT_VIEW_FACING:
                        mk.text, mk.color.g, mk.color.b = f"{o['id']} {o['label']}", 1.0, 1.0
                    arr.markers.append(mk)
            self.marker_pub.publish(arr)

        def stop(self):
            if self.goal is not None:
                self.goal.cancel_goal_async()  # stopped mid-turn: no turn left running
            if self.cur is not None:
                self.cur["note"] = "cut short: the survey was stopped"
                self.cur = {k: v for k, v in self.cur.items() if k in ("t", "x", "y", "steps", "headings", "seen", "note")}
                self.surveys.append(self.cur)
            self.write()

    rclpy.init()
    node = Survey()
    def on_term(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)  # the explore watcher stops it with a TERM
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        end = time.time() + 1.0
        try:
            while time.time() < end and rclpy.ok():  # the cancel and the markers out
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:  # a SIGINT has shut the context down already
            pass


if __name__ == "__main__":
    main()
