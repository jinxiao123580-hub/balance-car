import asyncio
from bleak import BleakScanner

async def main():
    print("Scanning for 8 seconds...")
    devices = await BleakScanner.discover(timeout=8.0, return_adv=True)
    if not devices:
        print("No devices found.")
        return
    for addr, adv in devices.items():
        d = adv[0]
        name = d.name if d.name else "(no name)"
        rssi = adv[1].rssi
        print(f"MAC={addr}  NAME='{name}'  RSSI={rssi}")
        # service UUIDs in adv data
        try:
            serv = adv[1].service_uuids if hasattr(adv[1], 'service_uuids') else []
            if serv:
                print(f"    services={list(serv)}")
        except Exception as e:
            pass

asyncio.run(main())