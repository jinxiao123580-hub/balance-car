# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""Dead-or-occupied? definitive probe for a BLE serial module.

Usage:
  python ble_probe.py [MAC]                 # scan first, then try connect
  python ble_probe.py --name YahBoom 20     # match by name substring, 20s capture

For each gorund truth: re-scan fresh before EVERY connect attempt (bleak caches
the device), try public then random address types, and if connected:
  - dump all services/characteristics (look for FFE1 notify/write)
  - subscribe FFE1 notify for N sec
  - write '?' to FFE1 to ask STM32 firmware for its tune params (full-chain test)
"""
import asyncio, sys, time

DEFAULT_MAC = _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")
SVC = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHAR = "0000ffe1-0000-1000-8000-00805f9b34fb"

async def scan_fresh(needle=None):
    from bleak import BleakScanner
    print("scanning (fresh)...", flush=True)
    devs = await BleakScanner.discover(timeout=8.0, return_adv=True)
    found = []
    for addr, item in devs.items():
        if isinstance(item, tuple):
            d, adv = item
        else:
            adv = item
            d = getattr(adv, "device", None) or _fake(addr)
        name = (d.name or "") if d.name else (getattr(adv, "local_name", "") or "")
        uuids = list(getattr(adv, "service_uuids", []) or [])
        entry = (addr, name, uuids, getattr(adv, "rssi", 0), d)
        if needle is None or needle.lower() in (addr + " " + name).lower():
            found.append(entry)
            print(f"  {addr}  {name!r}  rssi={entry[3]}  svc={uuids}", flush=True)
    return found

def _fake(addr):
    class _F:
        name = ""
        address = addr
    return _F()

async def try_connect(d, atype):
    from bleak import BleakClient
    cl = BleakClient(d, timeout=20.0, pair=False,
                     winrt={"address_type": atype, "use_cached_services": True})
    try:
        await asyncio.wait_for(cl.connect(), timeout=18.0)
        if cl.is_connected:
            print(f"  CONNECTED ({atype}) ✓", flush=True)
            return cl
    except Exception as e:
        print(f"  fail {atype}: {type(e).__name__} {str(e)[:120]}", flush=True)
    try:
        await cl.disconnect()
    except Exception:
        pass
    return None

async def try_connect_nosvc(d, atype):
    """Same, but no services= filter (some modules mishandle service filtering on Windows)."""
    from bleak import BleakClient
    print(f"[connect-nosvc] address_type={atype} ...", flush=True)
    cl = BleakClient(d, timeout=20.0, pair=False,
                     winrt={"address_type": atype})
    try:
        await asyncio.wait_for(cl.connect(), timeout=18.0)
        if cl.is_connected:
            print(f"  CONNECTED-nosvc ({atype}) ✓", flush=True)
            return cl
    except Exception as e:
        print(f"  fail nosvc/{atype}: {type(e).__name__} {str(e)[:150]}", flush=True)
    try:
        await cl.disconnect()
    except Exception:
        pass
    return None

async def probe(needle_mac, name_hint, capture_sec):
    from bleak import BleakScanner
    found = await scan_fresh()
    dev = None
    dev_obj = None
    # match MAC first, then name, then ffe0 service
    for addr, name, uuids, rssi, dobj in found:
        if needle_mac and addr.lower() == needle_mac.lower():
            dev = (addr, name, uuids); dev_obj = dobj; break
    if dev is None and name_hint:
        for addr, name, uuids, rssi, dobj in found:
            if name_hint.lower() in name.lower():
                dev = (addr, name, uuids); dev_obj = dobj; break
    if dev is None:
        for addr, name, uuids, rssi, dobj in found:
            if any("ffe0" in u.lower() or "ffe" in u.lower() for u in uuids):
                dev = (addr, name, uuids); dev_obj = dobj; break
    if dev is None:
        print("NOT_AT_SCAN — 目标模块根本没在空口广播（断电/烧毁/未连线/被手机单连接占用后不再广播）", flush=True)
        return

    addr, name, uuids = dev
    print(f"\n目标: {addr}  {name!r}  svc={uuids}", flush=True)

    got = None
    for atype in ("public", "random"):
        ok = False
        for attempt in range(1, 4):
            # fresh re-scan each attempt; target is inter. broadcast, so retry until it shows
            fresh = await scan_fresh()
            d = None
            for a, n, u, r, dobj in fresh:
                if a.lower() == addr.lower():
                    d = dobj; break
            if d is None:
                print(f"  {atype} attempt {attempt}: 目标未出现在广播，重扫 ...", flush=True)
                continue
            got = await try_connect(d, atype)
            if got:
                ok = True
                break
        if ok:
            break
    if got is None:
        # retry once WITHOUT services= filter, which often fixes Windows winrt timeouts
        print("\n[retry] 去掉 services= 过滤再试 ...", flush=True)
        for atype in ("public", "random"):
            ok = False
            for attempt in range(1, 4):
                fresh = await scan_fresh()
                d = None
                for a, n, u, r, dobj in fresh:
                    if a.lower() == addr.lower():
                        d = dobj; break
                if d is None:
                    print(f"  nosvc/{atype} attempt {attempt}: 目标未出现，重扫 ...", flush=True)
                    continue
                got = await try_connect_nosvc(d, atype)
                if got:
                    ok = True
                    break
            if ok:
                break
    if got is None:
        print("\n结论: 广播间歇可见但所有地址类型尝试都连不上 → 高度怀疑被别的蓝牙单连接占用，或需断电重启重新广播", flush=True)
        return

    # dump services
    print("\nservices:", flush=True)
    has_ffe1 = False
    for s in got.services:
        print(f"  {s.uuid}", flush=True)
        for ch in s.characteristics:
            prop = ch.properties
            print(f"    {ch.uuid}  props={prop}", flush=True)
            if ch.uuid.lower() == CHAR.lower():
                has_ffe1 = True
                wr_props = prop

    if not has_ffe1:
        print("\n结论: 没有 FFE1 → 不是标准 FFE0/FFE1 串口模块，脚本需改 UUID", flush=True)
        await got.disconnect()
        return

    rx = []
    def on_recv(_h, data):
        rx.append(data)
        try:
            print(f"  RX: {data.decode('utf-8','replace')!r}", flush=True)
        except Exception:
            pass

    print(f"\n订阅 FFE1 notify，写入 '?'，采集 {capture_sec}s ...", flush=True)
    try:
        await got.start_notify(CHAR, on_recv)
        await got.write_gatt_char(CHAR, b'?\n')
    except Exception as e:
        print(f"  notify/write 失败: {type(e).__name__} {str(e)[:120]}", flush=True)

    end = asyncio.get_event_loop().time() + capture_sec
    while asyncio.get_event_loop().time() < end:
        await asyncio.sleep(0.5)

    try:
        await got.stop_notify(CHAR)
    except Exception:
        pass
    try:
        await got.disconnect()
    except Exception:
        pass

    total = sum(len(b) for b in rx)
    text = b"".join(rx).decode("utf-8", "replace").strip()
    print(f"\n总收 {len(rx)} 帧 / {total} 字节", flush=True)
    if total == 0:
        print("结论: 连上了但空口零字节 → 最可能 STM32↔模块 UART 波特率不匹配（固件9600 vs 亚博115200），或板子没电/没烧固件", flush=True)
    else:
        print("结论: 收到回显 → 全链路通！模块 RF 正常、波特率也匹配", flush=True)

def _mk_bleak(addr, name):
    from bleak import BLEDevice
    d = BLEDevice(addr, name, {}, rssi=0)  # bleak>=0.21: BLEDevice(address, name, details, *, rssi)
    return d

def _main():
    args = sys.argv[1:]
    mac = DEFAULT_MAC
    name_hint = ""
    sec = 12
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--name":
            name_hint = args[i+1]; i += 2
        elif a == "--mac":
            mac = args[i+1]; i += 2
        else:
            if a.isdigit():
                sec = int(a); i += 1
            elif ":" in a and mac == DEFAULT_MAC:
                mac = a; i += 1
            else:
                mac = a; i += 1
    asyncio.run(probe(mac, name_hint, sec))

if __name__ == "__main__":
    _main()
