import cv2, csv, time, numpy as np, sys
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
P = cv2.aruco.DetectorParameters_create()
P.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
secs = float(sys.argv[1]) if len(sys.argv)>1 else 20.0
out  = sys.argv[2] if len(sys.argv)>2 else "/tmp/tags/trk.csv"
TAG_MM = 50.0
cap = cv2.VideoCapture(1)
for _ in range(15): cap.read()
rows=[]; t0=time.monotonic(); lost=0
while time.monotonic()-t0 < secs:
    ok, f = cap.read()
    if not ok: continue
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    c, i, _ = cv2.aruco.detectMarkers(g, D, parameters=P)
    t = round(time.monotonic()-t0, 3)
    if i is None:
        lost += 1; rows.append((t, round(time.time(),3), "","","","",0)); continue
    k = int(np.argmax([np.linalg.norm(cc.reshape(4,2)[0]-cc.reshape(4,2)[1]) for cc in c]))
    pts = c[k].reshape(4,2)
    side = float(np.linalg.norm(pts[0]-pts[1]))
    cx, cy = pts.mean(axis=0)
    rows.append((t, round(time.time(),3), round(cx,2), round(cy,2), round(side,2), round(TAG_MM/side,4), int(i.flatten()[k])))
cap.release()
with open(out,"w",newline="") as fh:
    w=csv.writer(fh); w.writerow(["t_s","wall","cx","cy","side_px","mm_per_px","id"]); w.writerows(rows)
g=[r for r in rows if r[2]!=""]
if not g: print("NO TAG"); raise SystemExit(3)
cx=[r[1] for r in g]; side=[r[4] for r in g]; mpp=float(np.median([r[5] for r in g]))
t=[r[0] for r in g]
mt=sum(t)/len(t); mx=sum(cx)/len(cx)
b=sum((a-mt)*(q-mx) for a,q in zip(t,cx))/sum((a-mt)**2 for a in t)
print("frames=%d detected=%d lost=%d fps=%.1f" % (len(rows), len(g), lost, len(rows)/max(secs,1e-6)))
print("id=%s  side=%.1f..%.1f px  -> %.3f mm/px" % (g[0][5], min(side), max(side), mpp))
print("cx %.1f -> %.1f   net %+.1f px = %+.1f mm" % (cx[0], cx[-1], cx[-1]-cx[0], (cx[-1]-cx[0])*mpp))
print("drift slope %+.2f px/s = %+.2f mm/s" % (b, b*mpp))
print("span %.1f px = %.1f mm" % (max(cx)-min(cx), (max(cx)-min(cx))*mpp))
