# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""Minimal: scan for a named BLE device, connect, print services, disconnect.
Usage: python ble_ping.py [MAC] [mode]
  mode=scan  : scan and list matching devices
  mode=info  : connect and print GATT services/chars (default)
"""
import asyncio, sys, time

MAC = sys.argv[1] if len(sys.argv) > 1 else _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")
MODE = sys.argv[2] if len(sys.argv) > 2 else "info"

async def scan(needle=None):
    from bleak import BleakScanner
    print("scanning...", flush=True)
    devs = await BleakScanner.discover(timeout=8.0)
    for d in devs:
        nm = d.name or ""
        do_print = (needle is None) or (needle.lower() in (""+d.address).lower()) or (needle.lower() in nm.lower())
        if do_print:
            rssi = getattr(d, "metadata", {}).get("rssi", "?")
            print(f"  {d.address}  NAME={nm!r}  RSSI={rssi}", flush=True)
    return devs

async def show_info(dev):
    from bleak import BleakClient
    print(f"connecting to {dev.address} {dev.name!r} ...", flush=True)
    t0 = time.time()
    try:
        c = BleakClient(dev, timeout=60.0)
        await asyncio.wait_for(c.connect(), timeout=50.0)
        print(f"  CONNECTED in {time.time()-t0:.1f}s", flush=True)
        print("  services:", flush=True)
        for s in c.services:
            print(f"    {s.uuid}", flush=True)
            for ch in s.characteristics:
                print(f"      {ch.uuid}  props={ch.properties}", flush=True)
        try: await c.disconnect()
        except Exception: pass
    except Exception as e:
        print(f"  FAIL {type(e).__name__} {str(e)[:150]}", flush=True)

async def main():
    devs = await scan(MAC)
    if MODE == "scan":
        return
    dev = None
    for d in devs:
        if d.address and d.address.replace(":","").upper().endswith(MAC.replace(":","").upper()[-6:]):
            dev = d; break
    if dev is None and devs:
        # fallback by name
        for d in devs:
            if "yahboom" in (d.name or "").lower():
                dev = d; break
    if dev is None:
        print("NOT_AT_SCAN", flush=True)
        return
    await show_info(dev)

asyncio.run(main())
