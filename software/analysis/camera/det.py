import cv2, numpy as np, time
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
P = cv2.aruco.DetectorParameters_create()
P.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
cap = cv2.VideoCapture(1)
for _ in range(15): cap.read()
ok, f = cap.read(); cap.release()
if not ok: raise SystemExit("no frame")
print("frame", f.shape)
gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
corners, ids, _ = cv2.aruco.detectMarkers(gray, D, parameters=P)
if ids is None:
    print("DETECTED 0 tags")
else:
    print("DETECTED %d tags" % len(ids))
    rows=[]
    for c, i in zip(corners, ids.flatten()):
        pts = c.reshape(4,2)
        w = float(np.linalg.norm(pts[0]-pts[1]))
        h = float(np.linalg.norm(pts[1]-pts[2]))
        cx, cy = pts.mean(axis=0)
        rows.append((int(i), w, h, cx, cy))
    rows.sort(key=lambda r: -r[1])
    for r in rows[:12]:
        print("  id=%3d  side=%.1f x %.1f px  center=(%.0f, %.0f)  module~%.2f px" % (r[0], r[1], r[2], r[3], r[4], r[1]/8))
    cv2.aruco.drawDetectedMarkers(f, corners, ids)
cv2.imwrite("/tmp/tags/det.jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 88])
