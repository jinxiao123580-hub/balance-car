# -*- coding: utf-8 -*-
"""Scan and print address_type + rssi + services for BLE devices."""
import asyncio
from bleak import BleakScanner

async def main():
    print("scanning...", flush=True)
    devs = await BleakScanner.discover(timeout=9.0, return_adv=True)
    for addr, adv in devs.items():
        d = adv[0]
        ad = adv[1]
        nm = d.name or "(none)"
        rssi = ad.rssi
        atype = None
        try:
            if hasattr(d, 'details'):
                dt = d.details
                if isinstance(dt, dict):
                    atype = dt.get('address_type')
                else:
                    atype = getattr(dt, 'address_type', None)
        except Exception:
            atype = None
        servs = list(ad.service_uuids) if hasattr(ad, 'service_uuids') else []
        # only show strong / named devices to reduce noise
        if rssi and rssi > -80 or nm != "(none)":
            print(f"{addr}  NAME={nm!r}  RSSI={rssi}  ADDRTYPE={atype}  services={servs}", flush=True)

asyncio.run(main())