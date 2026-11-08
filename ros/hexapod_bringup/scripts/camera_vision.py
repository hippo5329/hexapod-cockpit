#!/usr/bin/env python3
"""What the camera sees and how far: objects and the obstacles ahead.

Objects: NanoDet-Plus on the CPU (COCO's 80 classes; the default), or on a GPU an open-vocabulary detector that
labels from a list of phrases (HOUSEHOLD: slippers, cables, mats, doors, shelves and the like, which COCO has no
class for): OWLv2 (detector owlv2, 32 ms a frame on an RTX 5090) or Grounding DINO (grounding_dino, 46 ms).

Distances come from depth made metric by the floor. Depth Anything V2 Small gives inverse depth per
pixel up to a scale and an offset; a floor pixel's true depth is known from the geometry (the
calibration's K and distortion, the camera's height, the body's tilt from the IMU plus the camera's own
tilt on the body), so the two are fitted on the floor and every pixel gets a metric depth. A point more
than climb_height (4.5 cm) above the floor is an obstacle: walls, doors and furniture of no class as well as
objects; lower is floor the robot climbs (up to 4 cm, user 2026-11-08; the half centimetre is the depth's error).
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
  camera/obstacle_scan     sensor_msgs/LaserScan in base_footprint across the camera's view (obstacle_scan()):
                           per bearing the nearest obstacle up to 1.5 m, inf where the floor is free; only with a
                           floor fit. A costmap layer of its own reads it: what the LiDAR's plane (14.5 cm) passes
                           over -- a slipper -- must not be cleared by the LiDAR's rays. A mat (up to 4 cm) is
                           floor: the robot climbs it.

Parameters:
  detector          nanodet (CPU) | owlv2 | grounding_dino (GPU, transformers)
  model_path        nanodet: the NanoDet-Plus ONNX (opencv_zoo object_detection_nanodet_2022nov.onnx, 416x416);
                    owlv2 / grounding_dino: a Hugging Face model id (default google/owlv2-base-patch16-ensemble,
                    IDEA-Research/grounding-dino-base; both Apache-2.0)
  labels            the open-vocabulary detectors' phrases, comma separated (default HOUSEHOLD)
  depth_model_path  Depth Anything V2 Small ONNX (onnx-community/depth-anything-v2-small model.onnx; CPU, needs
                    onnxruntime), or hf:<model id> on the GPU (depth-anything/Depth-Anything-V2-Large-hf: 38 ms,
                    the most accurate at 1 m; Base and Large are CC-BY-NC-4.0, Small Apache-2.0);
                    empty: no depth (objects by floor contact, no obstacles)
  device            cuda, for the GPU models
  score             the lowest score kept: 0.40 (NanoDet), 0.20 (OWLv2: its scores run lower; a slipper 0.22),
                    0.35 (Grounding DINO)
  text_threshold    0.30, Grounding DINO's phrase score
  cam_height        0.1575 m, the camera above the floor while standing (URDF: base_link 0.060 + 0.0975)
  cam_x             0.065 m, the camera ahead of the body centre (URDF camera_joint)
  cam_y             -0.005 m, the camera left of the body centre (- = right)
  mount_pitch_deg   2.3, the camera's own tilt on the body (+ looks down); fitted on the checkerboard
                    frames with the IMU's +0.4 deg (3 deg total)
  rate              5.0 Hz, the most frames processed per second (the newest is taken, others dropped)
  threads           4, onnxruntime's threads (set them: in a cell it cannot pin its default count)
  climb_height      0.045 m: higher than this above the floor is an obstacle, lower is climbed
  scan_range        1.5 m, how far camera/obstacle_scan reports
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
        """[(label, score, (x1, y1, x2, y2) in the image's pixels)], best first."""
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
            out.append((COCO[int(ids[i])], float(best[i]), (float(x1), float(y1), float(x2), float(y2))))
        return sorted(out, key=lambda o: -o[1])


# What a house holds, for an open-vocabulary detector: COCO's indoor classes and what COCO lacks (slippers,
# cables, mats, doors, shelves, boxes, gym things). One phrase each, singular; the note uses them as labels.
HOUSEHOLD = ("person, cat, dog, chair, sofa, bed, table, desk, cabinet, shelf, bookshelf, door, window, curtain, "
             "refrigerator, oven, microwave, sink, toilet, bathtub, television, computer monitor, laptop, keyboard, "
             "lamp, potted plant, box, bag, backpack, suitcase, basket, laundry basket, trash can, bottle, cup, bowl, "
             "book, clock, vase, toy, ball, shoe, slipper, sock, cable, power strip, rug, mat, yoga mat, pillow, "
             "blanket, towel, fan, heater, dumbbell, treadmill, exercise bike, bicycle, robot, charger, umbrella")


def nms(dets, iou=0.6, same_label=True):
    """Greedy NMS over [(label, score, box)] (best kept); same_label False: across labels (one box, one label)."""
    keep = []
    for d in sorted(dets, key=lambda o: -o[1]):
        x1, y1, x2, y2 = d[2]
        ok = True
        for k in keep:
            if same_label and k[0] != d[0]:
                continue
            a1, b1, a2, b2 = k[2]
            iw, ih = max(0.0, min(x2, a2) - max(x1, a1)), max(0.0, min(y2, b2) - max(y1, b1))
            inter = iw * ih
            union = (x2 - x1) * (y2 - y1) + (a2 - a1) * (b2 - b1) - inter
            if union > 0 and inter / union > iou:
                ok = False
                break
        if ok:
            keep.append(d)
    return keep


class OWLv2:
    """An open-vocabulary detector on the GPU (OWLv2, google/owlv2-base-patch16-ensemble, Apache-2.0): every label
    is a text query, encoded once at start; a frame costs one pass of the image encoder however many labels.
    The frame is padded to a square (grey) and scaled to the model's 960 px on the GPU."""

    MEAN, STD = (0.48145466, 0.4578275, 0.40821073), (0.26862954, 0.26130258, 0.27577711)

    def __init__(self, model_id, labels, threshold=0.20, device="cuda"):
        import torch
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.torch, self.device = torch, device
        proc = Owlv2Processor.from_pretrained(model_id)
        self.model = Owlv2ForObjectDetection.from_pretrained(model_id).to(device).eval()
        self.labels = [w.strip() for w in labels.split(",") if w.strip()]
        self.size = proc.image_processor.size["height"]
        t = proc(text=[[f"a photo of a {w}" for w in self.labels]], return_tensors="pt").to(device)
        with torch.inference_mode():
            q = self.model.owlv2.get_text_features(input_ids=t.input_ids, attention_mask=t.attention_mask)
        self.query = q[None]  # (1, labels, d)
        self.qmask = torch.ones(self.query.shape[:2], dtype=torch.bool, device=device)
        self.mean = torch.tensor(self.MEAN, device=device).view(1, 3, 1, 1)
        self.std = torch.tensor(self.STD, device=device).view(1, 3, 1, 1)
        self.threshold = threshold

    def __call__(self, bgr):
        torch = self.torch
        h, w = bgr.shape[:2]
        s = max(h, w)
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16, enabled=self.device != "cpu"):
            x = torch.from_numpy(np.ascontiguousarray(bgr[..., ::-1])).to(self.device).permute(2, 0, 1)[None].float() / 255
            sq = torch.full((1, 3, s, s), 0.5, device=self.device)
            sq[..., :h, :w] = x
            x = (torch.nn.functional.interpolate(sq, size=(self.size, self.size), mode="bilinear", align_corners=False)
                 - self.mean) / self.std
            fmap = self.model.image_embedder(pixel_values=x)[0]
            b, gh, gw, d = fmap.shape
            feats = fmap.reshape(b, gh * gw, d)
            logits = self.model.class_predictor(feats, self.query, self.qmask)[0]
            boxes = self.model.box_predictor(feats, fmap)
        scores = torch.sigmoid(logits[0].float())  # (cells, labels)
        best, ids = scores.max(dim=1)
        keep = best >= self.threshold
        best, ids, boxes = best[keep].cpu().numpy(), ids[keep].cpu().numpy(), boxes[0][keep].float().cpu().numpy() * s
        dets = []
        for (cx, cy, bw, bh), sc, i in zip(boxes, best, ids):
            x1, y1 = float(np.clip(cx - bw / 2, 0, w - 1)), float(np.clip(cy - bh / 2, 0, h - 1))
            x2, y2 = float(np.clip(cx + bw / 2, 0, w - 1)), float(np.clip(cy + bh / 2, 0, h - 1))
            if x2 - x1 < 2 or y2 - y1 < 2 or (x2 - x1) * (y2 - y1) > 0.9 * w * h:
                continue
            dets.append((label_name(self.labels[int(i)]), float(sc), (x1, y1, x2, y2)))
        return nms(nms(dets, 0.5), 0.85, same_label=False)


def label_name(phrase):
    """An open-vocabulary phrase as a label: lower case, words joined by '_' (COCO's style)."""
    return "_".join(phrase.lower().replace(".", " ").split())


class GroundingDINO:
    """An open-vocabulary detector on the GPU (transformers' zero-shot object detection: Grounding DINO,
    IDEA-Research/grounding-dino-base, Apache-2.0): labels from a list of phrases, so it names what COCO has
    no class for. The phrases go in chunks (the text encoder takes 256 tokens), one batch per frame."""

    def __init__(self, model_id, labels, threshold=0.35, text_threshold=0.30, device="cuda", chunk=64, size=480):
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        self.torch, self.device = torch, device
        self.proc = AutoProcessor.from_pretrained(model_id)
        # A QVGA frame needs no 800 px (the processor's default): twice the camera's, and the time goes with the pixels.
        self.proc.image_processor.size = {"shortest_edge": size, "longest_edge": int(size * 4 / 3) + 1}
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(device).eval()
        words = [w.strip() for w in labels.split(",") if w.strip()]
        self.prompts = [". ".join(words[i:i + chunk]) + "." for i in range(0, len(words), chunk)]
        self.threshold, self.text_threshold = threshold, text_threshold

    def __call__(self, bgr):
        from PIL import Image
        h, w = bgr.shape[:2]
        img = Image.fromarray(bgr[..., ::-1])
        inp = self.proc(images=[img] * len(self.prompts), text=self.prompts, padding=True, return_tensors="pt").to(self.device)
        with self.torch.inference_mode(), self.torch.autocast("cuda", dtype=self.torch.float16, enabled=self.device != "cpu"):
            out = self.model(**inp)
        out.logits, out.pred_boxes = out.logits.float(), out.pred_boxes.float()
        res = self.proc.post_process_grounded_object_detection(
            out, inp.input_ids, threshold=self.threshold, text_threshold=self.text_threshold,
            target_sizes=[(h, w)] * len(self.prompts))
        dets = []
        for r in res:
            for box, score, lab in zip(r["boxes"].tolist(), r["scores"].tolist(), r.get("text_labels", r["labels"])):
                if not lab:
                    continue  # no phrase above text_threshold
                x1, y1, x2, y2 = (float(np.clip(box[0], 0, w - 1)), float(np.clip(box[1], 0, h - 1)),
                                  float(np.clip(box[2], 0, w - 1)), float(np.clip(box[3], 0, h - 1)))
                if (x2 - x1) * (y2 - y1) > 0.9 * w * h:
                    continue  # "the whole frame is a floor/door/room": not an object
                dets.append((label_name(lab), float(score), (x1, y1, x2, y2)))
        return nms(nms(dets, 0.5), 0.85, same_label=False)


class _OnnxDisparity:
    """Depth Anything V2 Small, ONNX on the CPU (266 px)."""

    SIZE = 266  # 19 x 14, the ViT's patch

    def __init__(self, path, threads=4):
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads, so.inter_op_num_threads = threads, 1  # a cell's cpuset: ort cannot pin more
        so.log_severity_level = 3
        self.sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name

    def __call__(self, bgr):
        h, w = bgr.shape[:2]
        x = cv2.resize(bgr, (self.SIZE, self.SIZE), interpolation=cv2.INTER_AREA)[..., ::-1].astype(np.float32) / 255
        x = (x - [0.485, 0.456, 0.406]) / [0.229, 0.224, 0.225]
        out = self.sess.run(None, {self.inp: x.transpose(2, 0, 1)[None].astype(np.float32)})[0]
        return cv2.resize(out.reshape(self.SIZE, self.SIZE), (w, h), interpolation=cv2.INTER_LINEAR)


class _HFDisparity:
    """Depth Anything V2 (any size) through transformers on the GPU, at the model's own 518 px."""

    def __init__(self, model_id, device="cuda"):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.torch, self.device = torch, device
        self.proc = AutoImageProcessor.from_pretrained(model_id)
        self.model = AutoModelForDepthEstimation.from_pretrained(model_id).to(device).eval()

    def __call__(self, bgr):
        from PIL import Image
        h, w = bgr.shape[:2]
        inp = self.proc(images=Image.fromarray(bgr[..., ::-1]), return_tensors="pt").to(self.device)
        with self.torch.inference_mode():
            d = self.model(**inp).predicted_depth  # relative inverse depth, as the ONNX model's
            d = self.torch.nn.functional.interpolate(d[:, None], size=(h, w), mode="bilinear", align_corners=False)
        return d[0, 0].float().cpu().numpy()


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
    """Depth Anything V2: relative inverse depth per pixel, made metric by the floor. `path` an ONNX file runs
    Small on the CPU (Apache-2.0; onnx-community/depth-anything-v2-small); `hf:<model id>` runs that model through
    transformers on `device` (depth-anything/Depth-Anything-V2-Small-hf, Apache-2.0; -Base-hf and -Large-hf are
    CC-BY-NC-4.0: non-commercial use only). Where a pixel is floor, its depth is known from the geometry
    (height, tilt, the ray); the model's output is inverse depth up to a scale and an offset, fitted on
    the floor in front of the feet and refitted on every pixel that agrees. A point that then stands
    more than `rise` above the floor is an obstacle: walls and doors as well as objects of no class."""

    def __init__(self, path, threads=4, rise=0.045, device="cuda"):
        self.disparity = _HFDisparity(path[3:], device) if path.startswith("hf:") else _OnnxDisparity(path, threads)
        self.rise, self.rays, self.key = rise, None, None

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


def obstacle_scan(pts, half_fov, bins=32, max_range=1.5, min_points=8):
    """Obstacle points (N, 2; x ahead, y left, m, about the body centre) as a scan: `bins` bearings across
    +-half_fov (rad); each the 5th-percentile distance of its points within max_range (a few stray pixels are
    not a wall), inf where fewer than min_points (free as far as the camera judges). Returns the ranges."""
    ranges = np.full(bins, np.inf)
    if len(pts):
        x, y = pts[:, 0], pts[:, 1]
        bearing, dist = np.arctan2(y, x), np.hypot(x, y)
        k = np.floor((bearing + half_fov) / (2 * half_fov) * bins).astype(int)
        keep = (k >= 0) & (k < bins) & (dist <= max_range) & (x > 0)
        for b in np.unique(k[keep]):
            d = dist[keep & (k == b)]
            if len(d) >= min_points:
                ranges[b] = float(np.percentile(d, 5))
    return ranges


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, LaserScan
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    class Vision(Node):
        def __init__(self):
            super().__init__("camera_vision")
            p = lambda n, d: self.declare_parameter(n, d).value
            path, detector = p("model_path", ""), p("detector", "nanodet")
            device = p("device", "cuda")
            if detector == "owlv2":
                self.det = OWLv2(path or "google/owlv2-base-patch16-ensemble", p("labels", HOUSEHOLD),
                                 float(p("score", 0.20)), device)
            elif detector == "grounding_dino":
                self.det = GroundingDINO(path or "IDEA-Research/grounding-dino-base", p("labels", HOUSEHOLD),
                                         float(p("score", 0.35)), float(p("text_threshold", 0.30)), device)
            elif path:
                self.det = NanoDet(path, float(p("score", 0.40)))
            else:
                raise SystemExit("camera_vision: set model_path (the NanoDet-Plus ONNX), or detector:=owlv2|grounding_dino")
            self.height, self.cam_x = float(p("cam_height", 0.1575)), float(p("cam_x", 0.065))
            self.cam_y = float(p("cam_y", -0.005))
            self.mount = math.radians(float(p("mount_pitch_deg", 2.3)))
            self.period = 1.0 / float(p("rate", 5.0))
            dpath = p("depth_model_path", "")
            self.depth = Depth(dpath, int(p("threads", 4)), float(p("climb_height", 0.045)), device) if dpath else None
            self.K = self.D = None
            self.pitch = self.roll = 0.0
            self.frame, self.lock, self.wake = None, threading.Lock(), threading.Event()
            self.create_subscription(CameraInfo, "camera/camera_info", self.on_info, qos_profile_sensor_data)
            self.create_subscription(Imu, "imu/data", self.on_imu, qos_profile_sensor_data)
            self.create_subscription(CompressedImage, "camera/image_raw/compressed", self.on_image, qos_profile_sensor_data)
            self.det_pub = self.create_publisher(Detection2DArray, "camera/detections", 10)
            self.json_pub = self.create_publisher(String, "camera/vision", 10)
            self.img_pub = self.create_publisher(CompressedImage, "camera/vision/image/compressed", qos_profile_sensor_data)
            self.scan_pub = self.create_publisher(LaserScan, "camera/obstacle_scan", qos_profile_sensor_data)
            self.scan_range = float(p("scan_range", 1.5))
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
            for label, score, (x1, y1, x2, y2) in self.det(bgr):
                g, how = gp((x1 + x2) / 2, y2), "floor contact"
                if fit:  # the box's own obstacle pixels, nearest quarter: holds for a box cut by the frame too
                    sub = obst[int(y1):int(y2) + 1, int(x1):int(x2) + 1]
                    if sub.sum() >= 20:
                        xs = X[int(y1):int(y2) + 1, int(x1):int(x2) + 1][sub]
                        ys = Y[int(y1):int(y2) + 1, int(x1):int(x2) + 1][sub]
                        near = xs <= np.percentile(xs, 25)
                        g, how = (float(np.median(xs[near])), float(np.median(ys[near]))), "depth"
                o = {"label": label, "score": round(score, 2), "box": [round(x1), round(y1), round(x2), round(y2)],
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
                hyp.hypothesis.class_id, hyp.hypothesis.score = label, score
                if g:
                    hyp.pose.pose.position.x, hyp.pose.pose.position.y = g
                d.results.append(hyp)
                arr.detections.append(d)
            free = free_floor(pts) if fit else None  # None: no floor fit, nothing known
            if fit:  # no floor fit: nothing is known, so nothing is said (the costmap keeps what it had)
                half = math.atan2(w / 2.0, self.K[0, 0])
                sc = LaserScan()
                sc.header.stamp, sc.header.frame_id = m.header.stamp, "base_footprint"
                rng = obstacle_scan(pts, half, max_range=self.scan_range)
                sc.angle_min, sc.angle_max = -half, half
                sc.angle_increment = 2 * half / len(rng)
                sc.range_min, sc.range_max = 0.05, self.scan_range
                sc.ranges = [float(r) for r in rng]
                self.scan_pub.publish(sc)
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
