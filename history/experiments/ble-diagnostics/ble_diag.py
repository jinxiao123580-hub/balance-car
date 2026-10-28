# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""Diagnose: print device details / address_type, try connect with explicit types."""
import asyncio, sys
from bleak import BleakScanner, BleakClient

MAC = sys.argv[1] if len(sys.argv) > 1 else _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")

async def main():
    print("scan...", flush=True)
    devs = await BleakScanner.discover(timeout=8.0)
    dev = None
    for d in devs:
        if d.address and d.address.upper() == MAC.upper():
            dev = d
            break
    if dev is None:
        print("NOT_FOUND", flush=True)
        return
    print("device:", dev.address, repr(dev.name), flush=True)
    dt = getattr(dev, 'details', None)
    print("details type:", type(dt).__name__, flush=True)
    print("details attrs:", [a for a in dir(dt) if not a.startswith('_')], flush=True)
    for attr in ("address_type", "path", "service_uuids", "rssi"):
        try:
            v = getattr(dt, attr)
            print(f"  {attr} = {v}", flush=True)
        except Exception as e:
            print(f"  {attr} err {type(e).__name__} {e}", flush=True)
    print("details repr:", repr(dt)[:200], flush=True)
    # Try connect; if device.details has address_type, bleak uses it automatically.

if __name__ == "__main__":
    asyncio.run(main())
