#!/usr/bin/env python3
"""A vector checkerboard PDF (US Letter landscape) for ROS camera_calibration: 9 x 7 squares of 25 mm
(8 x 6 inner corners) and a 100 mm bar to check the print scale. No dependencies. Args: <out.pdf>"""
import sys
MM = 72 / 25.4
W, H = 279.4, 215.9            # Letter landscape, mm
COLS, ROWS, SQ = 9, 7, 25.0    # squares -> 8x6 inner corners, 25 mm
ox, oy = (W - COLS * SQ) / 2, 22.0
ops = ["0 g"]
for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 0:
            ops.append(f"{(ox + c*SQ)*MM:.3f} {(oy + r*SQ)*MM:.3f} {SQ*MM:.3f} {SQ*MM:.3f} re f")
# 100 mm scale bar under the board, to check the print scale
bx, by = ox, 11.0
ops.append(f"{bx*MM:.3f} {by*MM:.3f} {100*MM:.3f} {1.2*MM:.3f} re f")
for t in (0, 50, 100):
    ops.append(f"{(bx+t)*MM - 0.5:.3f} {(by-2)*MM:.3f} 1 {5*MM:.3f} re f")
def text(x, y, size, s):
    s = s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    ops.append(f"BT /F1 {size} Tf {x*MM:.3f} {y*MM:.3f} Td ({s}) Tj ET")
text(bx + 104, by - 0.5, 8, "100 mm: measure this bar; print at 100 % / Actual size, no fit-to-page")
top = oy + ROWS * SQ + 3
text(ox, top + 8, 11, "miniHexa camera calibration target: 9 x 7 squares = 8 x 6 inner corners, 25.0 mm squares")
text(ox, top + 2, 8, "ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args "
     "-r image:=/camera/image_raw -r camera:=/camera   (square = your measured size in m)")
text(ox, 4, 7, "Glue flat on a rigid board, matte paper. White margin around the board is part of the target.")
content = "\n".join(ops).encode()
objs = [b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {W*MM:.2f} {H*MM:.2f}] /Contents 4 0 R "
        f"/Resources << /Font << /F1 5 0 R >> >> >>".encode(),
        b"<< /Length %d >>\nstream\n" % len(content) + content + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"]
out, offs = bytearray(b"%PDF-1.4\n"), []
for i, o in enumerate(objs, 1):
    offs.append(len(out)); out += b"%d 0 obj\n" % i + o + b"\nendobj\n"
x = len(out)
out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1) + b"".join(b"%010d 00000 n \n" % o for o in offs)
out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objs) + 1, x)
open(sys.argv[1], "wb").write(out)
