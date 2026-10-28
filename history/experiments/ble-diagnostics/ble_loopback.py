# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

"""BLE loopback: connect a module, write '?' cmd, capture any echo for N sec.
Usage: python ble_loopback.py [seconds] [MAC]  (MAC default = old JDY-23)
"""
import sys, asyncio, bleak

MAC = sys.argv[2] if len(sys.argv) > 2 else _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")
SVC = "0000ffe0-0000-1000-8000-00805f9b34fb"
WR  = "0000ffe1-0000-1000-8000-00805f9b34fb"
NOT = "0000ffe1-0000-1000-8000-00805f9b34fb"

buf = []
def on_recv(_h, data):
    buf.append(data)
    try:
        print("RX:", data.decode('utf-8','replace').rstrip())
    except Exception:
        print("RX(raw):", data)

async def main(sec):
    for addr_type in ('public','random'):
        try:
            cl = bleak.BleakClient(MAC, timeout=10)
            await cl.connect(address_type=addr_type)
            print("CONNECTED", addr_type)
            await cl.start_notify(NOT, on_recv)
            # ask for params
            await cl.write_gatt_char(WR, b'?\n')
            print("WRITE '?'")
            end = asyncio.get_event_loop().time() + sec
            while asyncio.get_event_loop().time() < end:
                await asyncio.sleep(0.5)
            await cl.disconnect()
            print(f"TOTAL RX bytes: {sum(len(b) for b in buf)}")
            return
        except Exception as e:
            print("fail", addr_type, repr(e))
    print("NO CONNECT")

asyncio.run(main(int(sys.argv[1]) if len(sys.argv)>1 else 12))
