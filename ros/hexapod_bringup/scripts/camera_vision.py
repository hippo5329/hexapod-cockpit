#!/usr/bin/env python3
"""What the camera sees and how far: objects (NanoDet-Plus, COCO's 80 classes) and the obstacles ahead.

Distances come from depth made metric by the floor. Depth Anything V2 Small gives inverse depth per
pixel up to a scale and an offset; a floor pixel's true depth is known from the geometry (the
calibration's K and distortion, the camera's height, the body's tilt from the IMU plus the camera's own
tilt on the body), so the two are fitted on the floor and every pixel gets a metric depth. A point more
than 3.5 cm above the floor is an obstacle: walls, doors and furniture of no class as well as objects.
An object's position is its own obstacle pixels' nearest quarter (or, with no depth, where its box meets
the floor). Checked against the checkerboard at 0.5 and 1.0 m (solvePnP: 0.501, 0.998): within 5 %.

  camera/image_raw/compressed, camera/camera_info, imu/data   in
  camera/detections        vision_msgs/Detection2DArray (class name in hypothesis.class_id, score;
                           the position in base_footprint in results[0].pose.pose.position)
  camera/vision            std_msgs/String, JSON: objects [{label, score, box, x, y, dist, bearing,
                           clipped, from}], free {left, centre, right} (m to the nearest obstacle: ahead
                           in the body's corridor, 15-50 deg either side; 2.5 = none seen; null: no
                           floor fit), pitch_deg, ms
  camera/vision/image/compressed   the frame with boxes, distances and the obstacles tinted red

Parameters:
  model_path        the NanoDet-Plus ONNX (opencv_zoo object_detection_nanodet_2022nov.onnx, 416x416)
  depth_model_path  Depth Anything V2 Small ONNX (onnx-community/depth-anything-v2-small model.onnx);
                    empty: no depth (objects by floor contact, no obstacles); needs onnxruntime
  score             0.40, the lowest class score kept
  cam_height        0.1575 m, the camera above the floor while standing (URDF: base_link 0.060 + 0.0975)
  cam_x             0.065 m, the camera ahead of the body centre (URDF camera_joint)
  cam_y             -0.005 m, the camera left of the body centre (- = right)
  mount_pitch_deg   2.3, the camera's own tilt on the body (+ looks down); fitted on the checkerboard
                    frames with the IMU's +0.4 deg (3 deg total)
  rate              5.0 Hz, the most frames processed per second (the newest is taken, others dropped)
  threads           4, onnxruntime's threads (set them: in a cell it cannot pin its default count)
"""
import json
import math
import threading
import time

import cv2
import numpy as np

COCO = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant stop_sign "
        "parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe backpack umbrella "
        "handbag tie suitcase frisbee skis snowboard sports_ball kite baseball_bat baseball_glove skateboard "
        "surfboard tennis_racket bottle wine_glass cup fork knife spoon bowl banana apple sandwich orange "
        "broccoli carrot hot_dog pizza donut cake chair couch potted_plant bed dining_table toilet tv laptop "
        "mouse remote keyboard cell_phone microwave oven toaster sink refrigerator book clock vase scissors "
        "teddy_bear hair_drier toothbrush").split()


class NanoDet:
    """NanoDet-Plus-m 416 (opencv_zoo): BGR in, normalised; four strides, GFL boxes (8-bin distances)."""

    SIZE, STRIDES, REG_MAX = 416, (8, 16, 32, 64), 7
    MEAN = np.array([103.53, 116.28, 123.675], np.float32)
    STD = np.array([57.375, 57.12, 58.395], np.float32)

    def __init__(self, path, score=0.40, iou=0.6):
        self.net = cv2.dnn.readNet(path)
        self.score, self.iou = score, iou
        self.centres = []
        for s in self.STRIDES:
            n = math.ceil(self.SIZE / s)
            xv, yv = np.meshgrid(np.arange(n) * s, np.arange(n) * s)
            self.centres.append(np.column_stack((xv.ravel(), yv.ravel())).astype(np.float32) + 0.5 * (s - 1))

    def __call__(self, bgr):
        """[(class index, score, (x1, y1, x2, y2) in the image's pixels)], best first."""
        h, w = bgr.shape[:2]
        k = self.SIZE / max(h, w)  # letterbox: scale to fit, pad right and bottom
        canvas = np.zeros((self.SIZE, self.SIZE, 3), np.uint8)
        canvas[:round(h * k), :round(w * k)] = cv2.resize(bgr, (round(w * k), round(h * k)))
        self.net.setInput(cv2.dnn.blobFromImage((canvas.astype(np.float32) - self.MEAN) / self.STD))
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        boxes, scores = [], []
        bins = np.arange(self.REG_MAX + 1, dtype=np.float32)
        for s, cls, reg, c in zip(self.STRIDES, outs[::2], outs[1::2], self.centres):
            cls, reg = cls.reshape(-1, len(COCO)), reg.reshape(-1, 4, self.REG_MAX + 1)
            e = np.exp(reg - reg.max(axis=2, keepdims=True))
            d = (e / e.sum(axis=2, keepdims=True)) @ bins * s
            boxes.append(np.column_stack((c[:, 0] - d[:, 0], c[:, 1] - d[:, 1], c[:, 0] + d[:, 2], c[:, 1] + d[:, 3])))
            scores.append(cls)
        boxes, scores = np.concatenate(boxes) / k, np.concatenate(scores)
        ids, best = scores.argmax(axis=1), scores.max(axis=1)
        keep = best >= self.score
        boxes, ids, best = boxes[keep], ids[keep], best[keep]
        if not len(best):
            return []
        xywh = [[float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])] for b in boxes]
        idx = []  # per class (NMSBoxesBatched is OpenCV 4.7+; jazzy's is 4.6)
        for c in set(ids.tolist()):
            sel = [i for i in range(len(ids)) if ids[i] == c]
            kept = cv2.dnn.NMSBoxes([xywh[i] for i in sel], [float(best[i]) for i in sel], self.score, self.iou)
            idx += [sel[int(j)] for j in np.array(kept).ravel()]
        out = []
        for i in idx:
            x1, y1, x2, y2 = np.clip(boxes[i], 0, [w - 1, h - 1, w - 1, h - 1])
            out.append((int(ids[i]), float(best[i]), (float(x1), float(y1), float(x2), float(y2))))
        return sorted(out, key=lambda o: -o[1])


def ground_point(u, v, K, D, height, pitch, roll=0.0, cam_x=0.0, cam_y=0.0):
    """The floor point (x ahead, y left, in m from the body centre's ground projection) that pixel (u, v)
    sees, or None at or above the horizon. pitch + looks down (ROS: nose down), roll + right side down."""
    n = cv2.undistortPoints(np.array([[[u, v]]], np.float64), K, D).ravel()
    r = np.array([1.0, -n[0], -n[1]])  # camera_link: x ahead, y left, z up
    cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    r = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]) @ r  # roll about x
    r = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ r  # pitch about y, + = nose down
    if r[2] > -1e-3:
        return None
    t = height / -r[2]
    return cam_x + t * r[0], cam_y + t * r[1]


class Depth:
    """Depth Anything V2 Small (Apache-2.0; onnx-community/depth-anything-v2-small): relative inverse depth
    per pixel, made metric by the floor. Where a pixel is floor, its depth is known from the geometry
    (height, tilt, the ray); the model's output is inverse depth up to a scale and an offset, fitted on
    the floor in front of the feet and refitted on every pixel that agrees. A point that then stands
    more than `rise` above the floor is an obstacle: walls and doors as well as objects of no class."""

    SIZE = 266  # 19 x 14, the ViT's patch

    def __init__(self, path, threads=4, rise=0.035):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads, so.inter_op_num_threads = threads, 1  # a cell's cpuset: ort cannot pin more
        so.log_severity_level = 3
        self.sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.rise, self.rays, self.key = rise, None, None

    def disparity(self, bgr):
        h, w = bgr.shape[:2]
        x = cv2.resize(bgr, (self.SIZE, self.SIZE), interpolation=cv2.INTER_AREA)[..., ::-1].astype(np.float32) / 255
        x = (x - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
        out = self.sess.run(None, {self.inp: x.transpose(2, 0, 1)[None].astype(np.float32)})[0]
        return cv2.resize(out.reshape(self.SIZE, self.SIZE), (w, h), interpolation=cv2.INTER_LINEAR)

    def camera_rays(self, K, D, w, h):
        """Per pixel, the undistorted ray in camera_link (x ahead = optical z = 1, y left, z up)."""
        if self.key != (K.tobytes(), D.tobytes(), w, h):
            uv = np.stack(np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64)), -1)
            n = cv2.undistortPoints(uv.reshape(-1, 1, 2), K, D).reshape(h, w, 2)
            self.rays = np.stack([np.ones((h, w)), -n[..., 0], -n[..., 1]], -1)
            self.key = (K.tobytes(), D.tobytes(), w, h)
        return self.rays

    def obstacles(self, bgr, K, D, height, pitch, roll, cam_x, cam_y=0.0):
        """(obstacle points (N, 2) x ahead, y left in m; obstacle pixel mask; per pixel x, y (m); fit ok)."""
        h, w = bgr.shape[:2]
        disp = self.disparity(bgr)
        cp, sp, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
        R = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        r = self.camera_rays(K, D, w, h) @ R.T
        below = r[..., 2] < -0.02
        inv_floor = np.where(below, -r[..., 2] / height, 0.0)  # 1 / floor depth (along the optical axis)
        band = np.zeros((h, w), bool)
        band[int(h * 0.85):, int(w * 0.2):int(w * 0.8)] = True
        sel = band & below
        ok, a, b = False, 0.0, 0.0
        for _ in range(3):
            if sel.sum() < 200:
                break
            A = np.column_stack([inv_floor[sel], np.ones(sel.sum())])
            a, b = np.linalg.lstsq(A, disp[sel], rcond=None)[0]
            if a <= 0:
                break
            pred = a * inv_floor + b
            sel = below & (np.abs(disp - pred) < 0.08 * np.abs(pred))
            ok = True
        if not ok:
            return np.zeros((0, 2)), np.zeros((h, w), bool), None, None, False
        with np.errstate(divide="ignore", invalid="ignore"):
            z = np.where(disp - b > 1e-6, a / (disp - b), np.inf)
        up = height + z * r[..., 2]  # the point's height above the floor
        obst = np.isfinite(z) & (z < 2.5) & (up > self.rise) & (up < 0.6)
        X, Y = cam_x + z * r[..., 0], cam_y + z * r[..., 1]
        return np.column_stack([X[obst], Y[obst]]), obst, X, Y, True


def free_floor(pts, half_width=0.12):
    """Free floor in m: ahead in the body's own corridor, and to the left and right (15-50 deg)."""
    if not len(pts):
        return {"left": 2.5, "centre": 2.5, "right": 2.5}
    x, y = pts[:, 0], pts[:, 1]
    bearing, dist = np.degrees(np.arctan2(y, x)), np.hypot(x, y)
    sectors = {"centre": (x > 0) & (np.abs(y) < half_width), "left": (bearing > 15) & (bearing < 50),
               "right": (bearing < -15) & (bearing > -50)}
    out = {}
    for k, m in sectors.items():
        d = (x[m] if k == "centre" else dist[m])
        # the 2nd percentile: a few stray pixels are not a wall
        out[k] = round(float(np.percentile(d, 2)), 3) if m.sum() > 15 else 2.5
    return out


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage, Imu
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    class Vision(Node):
        def __init__(self):
            super().__init__("camera_vision")
            p = lambda n, d: self.declare_parameter(n, d).value
            path = p("model_path", "")
            if not path:
                raise SystemExit("camera_vision: set model_path (the NanoDet-Plus ONNX)")
            self.det = NanoDet(path, float(p("score", 0.40)))
            self.height, self.cam_x = float(p("cam_height", 0.1575)), float(p("cam_x", 0.065))
            self.cam_y = float(p("cam_y", -0.005))
            self.mount = math.radians(float(p("mount_pitch_deg", 2.3)))
            self.period = 1.0 / float(p("rate", 5.0))
            dpath = p("depth_model_path", "")
            self.depth = Depth(dpath, int(p("threads", 4))) if dpath else None
            self.K = self.D = None
            self.pitch = self.roll = 0.0
            self.frame, self.lock, self.wake = None, threading.Lock(), threading.Event()
            self.create_subscription(CameraInfo, "camera/camera_info", self.on_info, qos_profile_sensor_data)
            self.create_subscription(Imu, "imu/data", self.on_imu, qos_profile_sensor_data)
            self.create_subscription(CompressedImage, "camera/image_raw/compressed", self.on_image, qos_profile_sensor_data)
            self.det_pub = self.create_publisher(Detection2DArray, "camera/detections", 10)
            self.json_pub = self.create_publisher(String, "camera/vision", 10)
            self.img_pub = self.create_publisher(CompressedImage, "camera/vision/image/compressed", qos_profile_sensor_data)
            threading.Thread(target=self.work, daemon=True).start()

        def on_info(self, m):
            self.K, self.D = np.array(m.k, np.float64).reshape(3, 3), np.array(m.d, np.float64)

        def on_imu(self, m):
            a = m.linear_acceleration
            pitch, roll = math.atan2(-a.x, math.hypot(a.y, a.z)), math.atan2(a.y, a.z)
            self.pitch += 0.1 * (pitch - self.pitch)  # the gait's jolts are fast, the tilt is slow
            self.roll += 0.1 * (roll - self.roll)

        def on_image(self, m):
            with self.lock:
                self.frame = m
            self.wake.set()

        def work(self):
            while rclpy.ok():
                self.wake.wait(1.0)
                self.wake.clear()
                with self.lock:
                    m, self.frame = self.frame, None
                if m is None or self.K is None:
                    continue
                t0 = time.time()
                try:
                    self.process(m)
                except Exception as e:  # one bad frame is not the node's end
                    self.get_logger().warning(f"frame dropped: {e}")
                time.sleep(max(0.0, self.period - (time.time() - t0)))

        def process(self, m):
            t0 = time.time()
            bgr = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return
            h, w = bgr.shape[:2]
            pitch = self.pitch + self.mount
            gp = lambda u, v: ground_point(u, v, self.K, self.D, self.height, pitch, self.roll, self.cam_x, self.cam_y)
            t1 = time.time()
            pts, obst, X, Y, fit = (self.depth.obstacles(bgr, self.K, self.D, self.height, pitch, self.roll, self.cam_x, self.cam_y)
                                    if self.depth else (np.zeros((0, 2)), None, None, None, False))
            t2 = time.time()
            objs, arr = [], Detection2DArray()
            arr.header = m.header
            for cid, score, (x1, y1, x2, y2) in self.det(bgr):
                g, how = gp((x1 + x2) / 2, y2), "floor contact"
                if fit:  # the box's own obstacle pixels, nearest quarter: holds for a box cut by the frame too
                    sub = obst[int(y1):int(y2) + 1, int(x1):int(x2) + 1]
                    if sub.sum() >= 20:
                        xs = X[int(y1):int(y2) + 1, int(x1):int(x2) + 1][sub]
                        ys = Y[int(y1):int(y2) + 1, int(x1):int(x2) + 1][sub]
                        near = xs <= np.percentile(xs, 25)
                        g, how = (float(np.median(xs[near])), float(np.median(ys[near]))), "depth"
                o = {"label": COCO[cid], "score": round(score, 2), "box": [round(x1), round(y1), round(x2), round(y2)],
                     "clipped": y2 >= h - 3, "from": how}
                if g:
                    o.update(x=round(g[0], 3), y=round(g[1], 3), dist=round(math.hypot(*g), 3),
                             bearing=round(math.degrees(math.atan2(g[1], g[0])), 1))
                objs.append(o)
                d = Detection2D()
                d.header = m.header
                d.bbox.center.position.x, d.bbox.center.position.y = (x1 + x2) / 2, (y1 + y2) / 2
                d.bbox.size_x, d.bbox.size_y = x2 - x1, y2 - y1
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id, hyp.hypothesis.score = COCO[cid], score
                if g:
                    hyp.pose.pose.position.x, hyp.pose.pose.position.y = g
                d.results.append(hyp)
                arr.detections.append(d)
            free = free_floor(pts) if fit else None  # None: no floor fit, nothing known
            ms = round((time.time() - t0) * 1000)
            self.json_pub.publish(String(data=json.dumps({"stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                                          "objects": objs, "free": free,
                                                          "pitch_deg": round(math.degrees(pitch), 2), "ms": ms,
                                                          "ms_depth": round((t2 - t1) * 1000)})))
            self.det_pub.publish(arr)
            out = bgr.copy()
            if fit:
                out[obst] = (0.5 * out[obst] + [0, 0, 128]).astype(np.uint8)  # obstacles tinted red
            for o in objs:
                x1, y1, x2, y2 = o["box"]
                cv2.rectangle(out, (x1, y1), (x2, y2), (0, 160, 255), 1)
                txt = f"{o['label']} {o['dist']:.2f}m" if "dist" in o else o["label"]
                cv2.putText(out, txt + ("<" if o["clipped"] else ""), (x1, max(10, y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, (0, 160, 255), 1, cv2.LINE_AA)
            cv2.putText(out, ("free L %.2f C %.2f R %.2f" % (free["left"], free["centre"], free["right"])) if free
                        else "no floor fit",
                        (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 0), 1, cv2.LINE_AA)
            img = CompressedImage()
            img.header, img.format = m.header, "jpeg"
            img.data = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            self.img_pub.publish(img)

    rclpy.init()
    node = Vision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
