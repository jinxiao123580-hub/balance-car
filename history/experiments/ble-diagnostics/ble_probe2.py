# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""Second-chance connect: capture YahBoom_BL's raw adv details, then try connect
several ways bleak supports on Windows: pre-scanned device object (auto address),
explicit public/random, and connect-by-filter callback.
"""
import asyncio, sys
from bleak import BleakScanner, BleakClient

TARGET_ADDR = _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")
TARGET_NAME = "YahBoom_BL"
SVC = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHAR = "0000ffe1-0000-1000-8000-00805f9b34fb"

async def capture_details():
    print("== capture raw adv details for target ==", flush=True)
    found = await BleakScanner.discover(timeout=10.0, return_adv=True)
    for addr, item in found.items():
        if isinstance(item, tuple):
            d, adv = item
        else:
            adv = item
            d = getattr(adv, "device", None)
        name = (d.name if d else "") or (getattr(adv, "local_name", "") or "")
        if addr.upper() == TARGET_ADDR.upper() or TARGET_NAME.lower() in name.lower():
            print(f"ADDR={addr} name={name!r} rssi={getattr(adv,'rssi',None)}", flush=True)
            print(f"  details type={type(d.details).__name__} details={d.details!r}", flush=True)
            try:
                print(f"  metadata={d.metadata!r}", flush=True)
            except AttributeError:
                print(f"  metadata=<none>", flush=True)
            print(f"  adv: {adv!r}", flush=True)
            dt = d.details
            for a in ("address_type","path","service_uuids","rssi","advertisement"):
                try:
                    print(f"  .{a} = {getattr(dt,a)}", flush=True)
                except Exception as e:
                    print(f"  .{a} err {type(e).__name__}", flush=True)
    return

async def try_one(label, factory):
    print(f"\n== try: {label} ==", flush=True)
    try:
        client = factory()
        await asyncio.wait_for(client.connect(), timeout=15.0)
        if client.is_connected:
            print(f"  CONNECTED [{label}] ✓", flush=True)
            for s in client.services:
                print(f"    svc {s.uuid}", flush=True)
                for c in s.characteristics:
                    print(f"      {c.uuid} props={c.properties}", flush=True)
            await client.disconnect()
            return True
        print(f"  not connected [{label}]", flush=True)
    except Exception as e:
        print(f"  FAIL [{label}]: {type(e).__name__} {str(e)[:160]}", flush=True)
    return False

async def main():
    await capture_details()
    # Attempt 1: pre-scanned device object (let bleak pick address type)
    print("\n== pre-scan to get live device object ==", flush=True)
    devs = await BleakScanner.discover(timeout=10.0)
    dev = None
    for d in devs:
        if d.address and d.address.upper() == TARGET_ADDR.upper():
            dev = d; break
    if dev is None:
        for d in devs:
            if TARGET_NAME.lower() in (d.name or "").lower():
                dev = d; break
    if dev is None:
        print("  target not in scan", flush=True)
    else:
        print(f"  live device: {dev.address} {dev.name!r}", flush=True)
        await try_one("live-device-autoaddr", lambda: BleakClient(dev, timeout=20.0, pair=False))
        await try_one("live-device-public", lambda: BleakClient(dev, timeout=20.0, pair=False, winrt={"address_type":"public"}))
        await try_one("live-device-random", lambda: BleakClient(dev, timeout=20.0, pair=False, winrt={"address_type":"random"}))
    # Attempt 2: connect by address with services filter (what main scripts use)
    await try_one("by-addr-public-svc", lambda: BleakClient(TARGET_ADDR, services=[SVC], timeout=20.0, pair=False, winrt={"address_type":"public"}))
    await try_one("by-addr-random-svc", lambda: BleakClient(TARGET_ADDR, services=[SVC], timeout=20.0, pair=False, winrt={"address_type":"random"}))
    await try_one("by-addr-random-nosvc", lambda: BleakClient(TARGET_ADDR, timeout=20.0, pair=False, winrt={"address_type":"random"}))

asyncio.run(main())
