# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""Connect to a BLE UART module by explicit MAC; capture notify lines to CSV."""
import asyncio, sys, time, csv

MAC = sys.argv[1] if len(sys.argv) > 1 else _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
OUT = sys.argv[3] if len(sys.argv) > 3 else str(_PROJECT_ROOT / "docs" / "validation" / "recordings" / "bluetooth_test.csv")

TA_MATCH = ("0000ffe0", "fff0", "ffe0", "49535343", "6e40")  # service uuid substrings to try

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
    from bleak import BleakClient, BleakScanner
    dev = None
    print("scanning...", flush=True)
    try:
        devs = await BleakScanner.discover(timeout=8.0)
        for d in devs:
            if d.address and d.address.upper() == MAC.upper():
                dev = d; print(f"found by MAC {d.address} {d.name!r}", flush=True); break
    except Exception as e:
        print("scan err", type(e).__name__, str(e)[:80], flush=True)
    if dev is None:
        # fallback: build device from address
        from bleak.backends.device import BLEDevice
        dev = BLEDevice(MAC, "ble", {}, rssi=0)
        print("using raw address", flush=True)

    # Try connect with explicit address type for each attempt.
    client = None
    addr_type_map = {"public": 0, "random": 1}
    for at in ("public", "random"):
        try:
            print(f"connect {at} ...", flush=True)
            from bleak import BleakClient
            # Force address_type on this attempt (holds the candidate address+type)
            dev._address_type = addr_type_map[at]
            c = BleakClient(dev, timeout=60.0)
            tstart = time.time()
            await asyncio.wait_for(c.connect(), timeout=55.0)
            print(f"  connect {at} took {time.time()-tstart:.1f}s", flush=True)
            if c.is_connected:
                client = c; print(f"CONNECTED ({at})", flush=True); break
        except Exception as e:
            print(f"  {at} fail {type(e).__name__} {str(e)[:120]}", flush=True)
    if client is None:
        print("CONNECT_FAILED", flush=True); f.close(); return

    # list services & find a writable+notify characteristic pair
    print("services:", flush=True)
    pairs = []
    for svc in client.services:
        print(f"  svc {svc.uuid}", flush=True)
        for ch in svc.characteristics:
            props = str(ch.properties)
            print(f"    ch {ch.uuid} props={props} notif={ch.notify}", flush=True)
    # pick notify char
    target = None
    for svc in client.services:
        for ch in svc.characteristics:
            if "notify" in ch.properties:
                target = ch; print(f"USE notify {ch.uuid}", flush=True); break
        if target: break
    if target is None:
        print("NO_NOTIFY_CHAR", flush=True)
        try: await client.disconnect()
        except Exception: pass
        f.close(); return
    print("subscribing...", flush=True)
    try:
        await client.start_notify(target.uuid, on_received)
        print("subscribed, capturing...", flush=True)
        await asyncio.sleep(DURATION)
        try: await client.stop_notify(target.uuid)
        except Exception: pass
    except Exception as e:
        print("notify err", type(e).__name__, str(e)[:90], flush=True)
    try: await client.disconnect()
    except Exception: pass
    f.close()
    print(f"DONE {count} lines -> {OUT}", flush=True)

asyncio.run(main())
