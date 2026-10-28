# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""Quick BLE reader for JDY-23 (FFE0/FFE1 notify). Prints raw lines for N seconds."""
import asyncio, sys, time

FFE0_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
FFE1_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
KNOWN_MAC = _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0

buf = b""
t0 = time.time()

def on_received(_c, data):
    global buf
    buf += data
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        txt = line.decode("utf-8", errors="ignore").strip()
        if txt:
            print(f"[{time.time()-t0:7.1f}s] {txt}", flush=True)

async def main():
    from bleak import BleakScanner, BleakClient
    device = None
    print("scanning for JDY-23 / FFE0 ...", flush=True)
    devs = await BleakScanner.discover(timeout=8.0)
    for d in devs:
        uuid_str = str(getattr(d, "metadata", {}).get("uuids", ""))
        if d.address == KNOWN_MAC or "ffe0" in uuid_str.lower() or "FFE0" in uuid_str:
            device = d
            print(f"  found: {d.address} name={d.name!r} uuids={uuid_str}", flush=True)
            break
    if device is None:
        # fallback: pick any device whose metadata mentions FFE0 or name contains JDY
        for d in devs:
            nm = (d.name or "").lower()
            if "jdy" in nm or "ble" in nm or "spp" in nm:
                device = d
                print(f"  fallback: {d.address} name={d.name!r}", flush=True)
                break
    if device is None:
        print("NOT_FOUND", flush=True)
        print("all devices:", [(d.address, d.name) for d in devs], flush=True)
        return

    for at in ("public", "random"):
        try:
            client = BleakClient(device, services=[FFE0_SERVICE_UUID], timeout=30.0, pair=False)
            print(f"connecting address_type={at} ...", flush=True)
            await asyncio.wait_for(client.connect(), timeout=20.0)
            if client.is_connected:
                print("CONNECTED", flush=True)
                break
        except Exception as e:
            print(f"  {at} fail: {type(e).__name__} {str(e)[:100]}", flush=True)
    else:
        print("CONNECT_FAILED", flush=True)
        return

    target = None
    for svc in client.services:
        for ch in svc.characteristics:
            if ch.uuid.lower() == FFE1_CHAR_UUID:
                target = ch
                break
        if target: break
    if target is None:
        print("NO_FFE1", flush=True)
        await client.disconnect()
        return
    print(f"subscribing {target.uuid} ...", flush=True)
    await client.start_notify(target.uuid, on_received)
    await asyncio.sleep(DURATION)
    try:
        await client.stop_notify(target.uuid)
    except Exception:
        pass
    await client.disconnect()
    print("DONE", flush=True)

asyncio.run(main())
