#!/usr/bin/env python3
"""Calibrate the camera without a screen: camera_calibration's GUI needs a display, this needs only OpenCV.

    calib_collect.py http://<camera ip>/capture <out dir> [seconds 600] [square m 0.025] [name hexacam1]

Polls /capture, finds the board (8x6 inner corners, or 7x6 with one column hidden), keeps a view when it
differs from every kept one in centre, size or tilt (camera_calibration's measure), and logs each kept
view with the ranges covered so far: move the board until x and y span ~0.1-0.9, size ~0.15-0.6 and
tilts reach ~0.4. At 40 views or the time limit it calibrates (plumb_bob, k3 fixed) and writes
<out>/<name>_<w>x<h>.yaml in camera_calibration's format, the bridge's camera_info_file.
"""
import sys, time, os, urllib.request, numpy as np, cv2
URL, OUT = sys.argv[1], sys.argv[2]
MAX_S = float(sys.argv[3]) if len(sys.argv) > 3 else 600
SQ = float(sys.argv[4]) if len(sys.argv) > 4 else 0.025
NAME = sys.argv[5] if len(sys.argv) > 5 else "hexacam1"
WANT = 40
os.makedirs(OUT, exist_ok=True)
log = open(f"{OUT}/log.txt", "a", buffering=1)
def say(*a):
    s = time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a); print(s, flush=True); log.write(s + "\n")
def find(g):
    for n in ((8, 6), (7, 6)):
        ok, c = cv2.findChessboardCornersSB(g, n, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok: return n, c.reshape(-1, 2)
    return None, None
def params(n, c, w, h):
    # like camera_calibration: x, y of the centre, size (area share), skew (corner angle)
    nx, ny = n; p = c.reshape(ny, nx, 2)
    q = [p[0, 0], p[0, -1], p[-1, -1], p[-1, 0]]
    area = abs(cv2.contourArea(np.array(q, np.float32)))
    a, b = q[1] - q[0], q[3] - q[0]
    skew = abs(np.pi / 2 - np.arccos(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b)))
    return np.array([c[:, 0].mean() / w, c[:, 1].mean() / h, np.sqrt(area / (w * h)), min(1.0, 2 * skew)])
views, kept, t0, last = [], [], time.time(), 0
size = None
while time.time() - t0 < MAX_S and len(kept) < WANT:
    try:
        jpg = urllib.request.urlopen(URL, timeout=3).read()
    except Exception as e:
        say("capture failed:", e); time.sleep(1); continue
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
    size = img.shape[::-1]
    n, c = find(img)
    if n is None:
        time.sleep(0.2); continue
    p = params(n, c, *size)
    d = min((np.abs(p - k[2]).sum() for k in kept), default=9)
    if d > 0.2:
        kept.append((n, c, p)); cv2.imwrite(f"{OUT}/v{len(kept):02d}.png", img)
        X = np.array([k[2] for k in kept])
        say(f"view {len(kept)} kept {n[0]}x{n[1]}: x {p[0]:.2f} y {p[1]:.2f} size {p[2]:.2f} skew {p[3]:.2f}"
            f" | range x {X[:,0].min():.2f}-{X[:,0].max():.2f} y {X[:,1].min():.2f}-{X[:,1].max():.2f}"
            f" size {X[:,2].min():.2f}-{X[:,2].max():.2f} skew max {X[:,3].max():.2f}")
    time.sleep(0.3)
say(f"collected {len(kept)} views in {time.time()-t0:.0f} s")
if len(kept) < 10: say("too few views; no calibration"); sys.exit(1)
obj, img_pts = [], []
for n, c, _ in kept:
    o = np.zeros((n[0] * n[1], 3), np.float32); o[:, :2] = np.mgrid[0:n[0], 0:n[1]].T.reshape(-1, 2) * SQ
    obj.append(o); img_pts.append(c.astype(np.float32))
rms, K, D, rv, tv = cv2.calibrateCamera(obj, img_pts, size, None, None, flags=cv2.CALIB_FIX_K3)
per = [float(np.sqrt(np.mean(np.sum((cv2.projectPoints(o, r, t, K, D)[0].reshape(-1, 2) - ip) ** 2, 1))))
       for o, ip, r, t in zip(obj, img_pts, rv, tv)]
w, h = size
say(f"rms {rms:.3f} px; fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}; D {np.round(D.ravel(), 4).tolist()}")
say("worst views:", sorted(((round(e, 2), i + 1) for i, e in enumerate(per)), reverse=True)[:5])
P = np.zeros((3, 4)); P[:3, :3] = K
f = lambda a: "[" + ", ".join(f"{v:.6f}" for v in np.asarray(a).ravel()) + "]"
yml = f"""image_width: {w}
image_height: {h}
camera_name: {NAME}
camera_matrix:
  rows: 3
  cols: 3
  data: {f(K)}
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: {f(D.ravel()[:5])}
rectification_matrix:
  rows: 3
  cols: 3
  data: {f(np.eye(3))}
projection_matrix:
  rows: 3
  cols: 4
  data: {f(P)}
"""
path = f"{OUT}/{NAME}_{w}x{h}.yaml"
open(path, "w").write(yml)
say("wrote", path)
