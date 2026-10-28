#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
from bleak import BleakClient, BleakScanner


DEVICE_NAME = "JDY-23"
FFE1_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"


# ============================================================
# 收到 JDY-23 BLE 数据
# ============================================================

def notification_handler(sender, data: bytearray):
    print(
        data.decode("utf-8", errors="replace"),
        end="",
        flush=True
    )


# ============================================================
# 扫描 JDY-23
# ============================================================

async def find_jdy():
    print("正在扫描 JDY-23 ...")

    wanted = DEVICE_NAME.lower()

    def matcher(device, adv):
        names = [
            (device.name or "").strip().lower(),
            (adv.local_name or "").strip().lower(),
        ]

        return any(
            name == wanted or wanted in name
            for name in names
            if name
        )

    device = await BleakScanner.find_device_by_filter(
        matcher,
        timeout=15.0
    )

    return device


# ============================================================
# 主程序
# ============================================================

async def main():

    device = await find_jdy()

    if device is None:
        print("没有找到 JDY-23")
        return


    print()
    print(
        f"找到设备: name={device.name!r} "
        f"address={device.address}"
    )

    print()
    print("正在连接 BLE ...")


    # 你之前已经实测 public 可以正常连接
    client = BleakClient(
        device,
        timeout=30.0,
        pair=False,
        winrt={
            "address_type": "public",
            "use_cached_services": True,
        },
    )


    try:

        await client.connect()

        if not client.is_connected:
            print("BLE连接失败")
            return


        print("BLE连接成功")


        # ====================================================
        # 查找 FFE1
        # ====================================================

        target = None

        for service in client.services:

            for char in service.characteristics:

                if char.uuid.lower() == FFE1_UUID:
                    target = char
                    break

            if target is not None:
                break


        if target is None:
            print("没有找到 FFE1")
            return


        print(
            "找到 FFE1:",
            target.uuid
        )


        # ====================================================
        # 订阅 Notify
        # ====================================================

        await client.start_notify(
            target.uuid,
            notification_handler
        )


        print()
        print("========================================")
        print("开始显示 JDY-23 收到的原始 UART 数据")
        print("Ctrl+C 退出")
        print("========================================")
        print()


        while client.is_connected:
            await asyncio.sleep(1.0)


    except KeyboardInterrupt:
        print("\n用户退出")


    except Exception as exc:
        print()
        print(
            "BLE错误:",
            type(exc).__name__,
            str(exc)
        )


    finally:

        if client.is_connected:

            try:
                await client.stop_notify(
                    FFE1_UUID
                )
            except Exception:
                pass

            try:
                await client.disconnect()
            except Exception:
                pass


# ============================================================

if __name__ == "__main__":

    try:
        asyncio.run(main())

    except KeyboardInterrupt:
        print("\n程序结束")