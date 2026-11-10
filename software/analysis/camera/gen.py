import cv2, io, numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader

D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
SIDE_PX = 320

# --- 自检：ID 0 的模块结构（每 40 px 一个模块 = 8x8）---
img0 = cv2.aruco.drawMarker(D, 0, 320)
small = img0[::40, ::40]
print("=== ID 0 结构（8x8 模块，含 1 模块黑边）===")
for row in small:
    print("  " + "".join("#" if v < 128 else "." for v in row))

def png(i):
    im = cv2.aruco.drawMarker(D, i, SIDE_PX)
    ok, buf = cv2.imencode(".png", im)
    return io.BytesIO(buf.tobytes())

TAG, PITCH = 10.0, 14.0
COLS, ROWS = 14, 19
N = COLS * ROWS
x0 = (210 - COLS * PITCH) / 2.0
y_top = 289.0

c = canvas.Canvas("apriltag36h11_10mm_A4.pdf", pagesize=A4)
for r in range(ROWS):
    for k in range(COLS):
        tid = r * COLS + k
        cx = x0 + k * PITCH + (PITCH - TAG) / 2.0
        cy = y_top - r * PITCH - (PITCH - TAG) / 2.0 - TAG
        c.drawImage(ImageReader(png(tid)), cx * mm, cy * mm, TAG * mm, TAG * mm,
                    preserveAspectRatio=False, mask=None)
# 标尺
c.setLineWidth(0.4)
c.setFont("Helvetica", 6)
by = 12.0
c.line(7 * mm, by * mm, 107 * mm, by * mm)
for i in range(11):
    x = (7 + i * 10) * mm
    c.line(x, by * mm, x, (by + (3 if i % 5 == 0 else 1.8)) * mm)
c.drawString(7 * mm, (by - 3.5) * mm, "0")
c.drawString(56.5 * mm, (by - 3.5) * mm, "50")
c.drawString(104 * mm, (by - 3.5) * mm, "100 mm")
c.setFont("Helvetica-Bold", 7)
c.drawString(112 * mm, (by - 1) * mm, "PRINT 100% (no fit-to-page)")
c.setFont("Helvetica", 5.5)
c.drawString(112 * mm, (by + 2.6) * mm, "AprilTag 36h11  ID 0-%d" % (N - 1))
c.drawString(112 * mm, (by + 5.4) * mm, "black square 10.0mm = 8 modules (1.25mm)")
c.drawString(112 * mm, (by + 8.2) * mm, "pitch 14.0mm  grid %dx%d" % (COLS, ROWS))
c.drawString(7 * mm, (by + 8.2) * mm, "IDs run left->right, top->bottom: id = row*%d + col" % COLS)
c.showPage()
c.save()
print("tags:", N, "grid:", COLS, "x", ROWS, "-> apriltag36h11_10mm_A4.pdf")
import os
print("size", os.path.getsize("apriltag36h11_10mm_A4.pdf"), "bytes")
