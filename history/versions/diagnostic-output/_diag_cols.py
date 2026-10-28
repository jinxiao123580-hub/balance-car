# -*- coding: utf-8 -*-
import glob, os

for f in sorted(glob.glob(r"software/ble-panel/ble_tune_log_*.csv")):
    hdr = None
    nrows = 0
    lens = set()
    marks = 0
    for line in open(f, encoding="utf-8", errors="replace"):
        s = line.rstrip("\n").rstrip("\r")
        if not s.strip():
            continue
        if s.startswith("#"):
            marks += 1
            continue
        if s.startswith("time_s"):
            hdr = s.split(",")
            continue
        if hdr is None:
            continue
        nrows += 1
        lens.add(len(s.split(",")))
    print("%-52s hdr=%3s rows=%4d datalens=%s marks=%d" % (
        os.path.basename(f), len(hdr) if hdr else "?", nrows, sorted(lens), marks))
    if hdr:
        print("      hdr[25:30] =", hdr[25:30] if len(hdr) > 30 else hdr[-6:])
        print("      hdr[-3:]   =", hdr[-3:])
