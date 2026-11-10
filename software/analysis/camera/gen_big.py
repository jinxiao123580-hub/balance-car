import cv2, io
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader

D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
def png(i):
    im = cv2.aruco.drawMarker(D, i, 800)
    return io.BytesIO(cv2.imencode(".png", im)[1].tobytes())

# (尺寸mm, ID, 底边y mm)  页面 210x297
items = [(100.0, 0, 168.0), (70.0, 1, 86.0), (50.0, 2, 26.0)]
c = canvas.Canvas("apriltag36h11_big_A4.pdf", pagesize=A4)
c.setFont("Helvetica-Bold", 11)
c.drawString(15*mm, 286*mm, "AprilTag 36h11 - large tags for the car")
c.setFont("Helvetica", 7)
c.drawString(15*mm, 281*mm, "PRINT 100% (no fit-to-page). black square = tag size; keep >=1 module white around it.")
c.drawString(15*mm, 276*mm, "module = size/8.  larger tag = works from farther away.")

for size, tid, y in items:
    x = 55.0                                  # 左对齐，右侧写标签
    c.drawImage(ImageReader(png(tid)), x*mm, y*mm, size*mm, size*mm,
                preserveAspectRatio=False, mask=None)
    c.setFont("Helvetica", 7)
    c.drawString((x+size+5)*mm, (y+size-3)*mm, "ID %d" % tid)
    c.drawString((x+size+5)*mm, (y+size-8)*mm, "%.0f mm" % size)
    c.drawString((x+size+5)*mm, (y+size-13)*mm, "module %.1f mm" % (size/8))
    c.setLineWidth(0.3); c.setDash(1,1)
    c.rect((x-2)*mm, (y-2)*mm, (size+4)*mm, (size+4)*mm)
    c.setDash()

c.setLineWidth(0.4)
c.line(10*mm, 10*mm, 110*mm, 10*mm)
for i in range(11):
    xx=(10+i*10)*mm
    c.line(xx, 10*mm, xx, (10+(3 if i%5==0 else 1.8))*mm)
c.setFont("Helvetica", 5.5)
c.drawString(10*mm, 6.0*mm, "0")
c.drawString(56*mm, 6.0*mm, "50")
c.drawString(104*mm, 6.0*mm, "100 mm - verify with a ruler")
c.showPage(); c.save()
print("ok")
