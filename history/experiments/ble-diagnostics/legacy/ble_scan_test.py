# -*- coding: utf-8 -*-
# ============================================================
# BLE 连接诊断脚本
#
# 用法：
#   1. 手机蓝牙完全关闭，模块断电重启（拔3.3V几秒再上电）
#   2. 运行：python ble_scan_test.py
#   3. 把打印结果发回来
#
# 功能：
#   - 列出电脑能扫到的所有 BLE 设备（名称/地址/地址类型/RSSI）
#   - 自动尝试用 public / random 两种地址类型连接 YahBoom_BL
# ============================================================

import asyncio
import sys

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "YahBoom_BL"
FFE0_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
FFE1_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"


async def scan_all():
    print("=" * 60)
    print("第一步：扫描电脑能看到的所有 BLE 设备")
    print("=" * 60)

    devices = await BleakScanner.discover(timeout=10.0)

    if not devices:
        print("什么都没扫到！")
        print("  检查：Windows 蓝牙是否开启？模块是否上电并处于广播状态？")
        print("  检查：手机蓝牙是否已完全关闭（模块被手机占着连接时不会广播）？")
        return []

    for dev in devices:
        details = getattr(dev, "details", {}) or {}
        addr_type = details.get("address_type", "?")
        rssi = getattr(dev, "rssi", "?")
        print(
            f"  名称={repr(dev.name)}  地址={dev.address}  "
            f"类型={addr_type}  RSSI={rssi}"
        )

    return devices


def pick_target(devices):
    exact = [d for d in devices if (d.name or "").strip().lower() == DEVICE_NAME.lower()]
    if exact:
        return exact[0]

    fuzzy = [d for d in devices if d.name and DEVICE_NAME.lower() in (d.name or "").lower()]
    if fuzzy:
        return fuzzy[0]

    return None


async def try_connect(device, addr_type):
    print(f"\n尝试连接：{repr(device.name)} {device.address}  (address_type={addr_type})")

    client = BleakClient(
        device,
        services=[FFE0_SERVICE_UUID],
        timeout=15.0,
        pair=False,
        winrt={"address_type": addr_type, "use_cached_services": True},
    )

    try:
        await asyncio.wait_for(client.connect(), timeout=20.0)
    except Exception as exc:
        print(f"  连接失败：{type(exc).__name__}: {exc}")
        return False

    if not client.is_connected:
        print("  连接状态异常（is_connected=False）")
        return False

    print("  连接成功！")

    # 列出服务/特征
    for service in client.services:
        print(f"    SERVICE {service.uuid}")
        for char in service.characteristics:
            print(f"      CHAR {char.uuid} props={char.properties}")
            if char.uuid.lower() == FFE1_CHAR_UUID:
                props = [p.lower() for p in char.properties]
                if "notify" in props or "indicate" in props:
                    print("      -> FFE1 支持 notify，可以用来收数据！")

    try:
        await client.disconnect()
    except Exception:
        pass

    return True


async def main():
    devices = await scan_all()

    if not devices:
        return

    target = pick_target(devices)

    if target is None:
        print("\n没找到名称含 YahBoom_BL 的设备。")
        print("上面的列表里有你的模块吗？名字可能不一样，把列表发回来确认。")
        return

    details = getattr(target, "details", {}) or {}
    detected_type = details.get("address_type", None)

    print(f"\n目标设备：{repr(target.name)} {target.address}")
    print(f"扫描报告的类型：{detected_type}")

    # 优先用扫描到的类型，public / random 都试一遍
    types_to_try = []
    if detected_type:
        types_to_try.append(detected_type)
    for t in ("public", "random"):
        if t not in types_to_try:
            types_to_try.append(t)

    success = False
    for t in types_to_try:
        ok = await try_connect(target, t)
        if ok:
            success = True
            print(f"\n>>> 用 {t} 连接成功了！")
            break

    if not success:
        print("\n两种地址类型都连不上。")
        print("  可能原因：")
        print("   1. 手机还连着模块（再检查一遍手机蓝牙是否完全关闭）")
        print("   2. Windows 缓存了旧的配对信息（设置→蓝牙→删掉再试）")
        print("   3. 模块本身出问题（可考虑换一个，推荐 JDY-23 或正版 HM-10）")
        print("把上面完整输出发回来，我来判断。")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n已取消")
        sys.exit(0)
