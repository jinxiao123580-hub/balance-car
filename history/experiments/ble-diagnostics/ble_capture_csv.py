# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""Headless BLE capture to CSV for JDY-23 (FFE0/FFE1 notify)."""
import asyncio, sys, time, csv

FFE0_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
FFE1_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
KNOWN_MAC = _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
OUT = sys.argv[2] if len(sys.argv) > 2 else str(_PROJECT_ROOT / "docs" / "validation" / "recordings" / "diag_drive_lock.csv")

buf = b""
t0 = time.time()
count = 0
f = open(OUT, "w", newline="", encoding="utf-8")
w = csv.writer(f)
w.writerow(["raw"])

def on_received(_c, data):
    global buf, count
    buf += data
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        txt = line.decode("utf-8", errors="ignore").strip()
        if txt:
            w.writerow([txt])
            count += 1

async def main():
    from bleak import BleakScanner, BleakClient
    device = None
    print("scanning...", flush=True)
    devs = await BleakScanner.discover(timeout=8.0)
    for d in devs:
        uuid_str = str(getattr(d, "metadata", {}).get("uuids", ""))
        if d.address == KNOWN_MAC or "ffe0" in uuid_str.lower():
            device = d; print(f"found {d.address} {d.name!r}", flush=True); break
    if device is None:
        for d in devs:
            if "jdy" in (d.name or "").lower():
                device = d; print(f"fallback {d.address} {d.name!r}", flush=True); break
    if device is None:
        print("NOT_FOUND", flush=True); return
    for at in ("public", "random"):
        try:
            client = BleakClient(device, services=[FFE0_SERVICE_UUID], timeout=30.0, pair=False)
            print(f"connect {at} ...", flush=True)
            await asyncio.wait_for(client.connect(), timeout=20.0)
            if client.is_connected: print("CONNECTED", flush=True); break
        except Exception as e:
            print(f"  {at} fail {type(e).__name__} {str(e)[:80]}", flush=True)
    else:
        print("CONNECT_FAILED", flush=True); return
    target = None
    for svc in client.services:
        for ch in svc.characteristics:
            if ch.uuid.lower() == FFE1_CHAR_UUID: target = ch; break
        if target: break
    if target is None:
        print("NO_FFE1", flush=True); return
    print("subscribing, capturing...", flush=True)
    await client.start_notify(target.uuid, on_received)
    await asyncio.sleep(DURATION)
    try: await client.stop_notify(target.uuid)
    except Exception: pass
    await client.disconnect()
    f.close()
    print(f"DONE {count} lines -> {OUT}", flush=True)

asyncio.run(main())
