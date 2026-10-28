import asyncio
import csv
import threading
import traceback
from collections import deque
from pathlib import Path

from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ============================================================
# YahBoom_BL + Bleak + Kalman 双曲线
#
# 适用于：
#   Windows
#   YahBoom_BL BLE 串口模块
#
# 关键修改：
#   1. pair=False
#   2. address_type="public"
#   3. 只枚举 FFE0 / FFE1
#   4. use_cached_services=True
#
# 安装：
#   pip install -U bleak matplotlib
#
# 运行：
#   python yahboom_ble_kalman_plot_ffe1.py
# ============================================================


DEVICE_NAME = "YahBoom_BL"

# 亚博/HM-10/CC2541类模块常见UUID
FFE0_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
FFE1_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

WINDOW_SECONDS = 8.0

CSV_PATH = Path("kalman_compare_ble.csv")


# ============================================================
# 数据缓存
# ============================================================

times = deque()
before_values = deque()
after_values = deque()

start_ms = None

receive_buffer = bytearray()

csv_file = None
csv_writer = None

ble_client = None

connection_ok = False


# ============================================================
# BLE扫描
# ============================================================

async def find_device():
    print(f"正在扫描 {DEVICE_NAME} ...")

    def matcher(device, adv):
        name = (device.name or "").strip()
        return name.lower() == DEVICE_NAME.lower()

    device = await BleakScanner.find_device_by_filter(
        matcher,
        timeout=10.0,
    )

    if device is None:
        print("\n没有找到 YahBoom_BL")
        return None

    print("\n找到设备：")
    print("name    =", repr(device.name))
    print("address =", device.address)

    return device


# ============================================================
# 解析STM32 CSV
#
# STM32发送：
# time_ms,acc_pitch,kalman_pitch
#
# 例如：
# 15230,-0.812,-0.216
# ============================================================

def parse_line(text):
    global start_ms

    text = text.strip()

    if not text:
        return

    parts = text.split(",")

    if len(parts) != 3:
        return

    try:
        t_ms = int(parts[0])
        before = float(parts[1])
        after = float(parts[2])
    except ValueError:
        # 跳过表头或乱码
        return

    if start_ms is None:
        start_ms = t_ms

    t = (t_ms - start_ms) / 1000.0

    times.append(t)
    before_values.append(before)
    after_values.append(after)

    if csv_writer is not None:
        csv_writer.writerow([
            f"{t:.4f}",
            f"{before:.4f}",
            f"{after:.4f}",
        ])


# ============================================================
# BLE Notify回调
#
# BLE可能把一整行拆成多个packet，
# 所以先进入buffer，再按换行切割。
# ============================================================

def notification_handler(_sender, data: bytearray):
    global receive_buffer

    receive_buffer.extend(data)

    while b"\n" in receive_buffer:
        raw_line, _, remaining = receive_buffer.partition(b"\n")

        receive_buffer = bytearray(remaining)

        text = raw_line.decode(
            "utf-8",
            errors="ignore",
        )

        parse_line(text)


# ============================================================
# BLE连接
# ============================================================

async def bluetooth_task():
    global ble_client
    global connection_ok

    device = await find_device()

    if device is None:
        return

    print("\n准备连接 BLE ...")

    # 关键：
    # - 不配对
    # - 固定public
    # - 只请求FFE0
    # - 使用Windows缓存
    ble_client = BleakClient(
        device,
        services=[
            FFE0_SERVICE_UUID,
        ],
        timeout=60.0,
        pair=False,
        winrt={
            "address_type": "public",
            "use_cached_services": True,
        },
    )

    try:
        print("调用 connect() ...")

        await asyncio.wait_for(
            ble_client.connect(),
            timeout=65.0,
        )

        print(
            "连接状态：",
            ble_client.is_connected,
        )

        if not ble_client.is_connected:
            print("BLE没有真正连接成功。")
            return

        connection_ok = True

        print("\n检测到的服务：")

        for service in ble_client.services:
            print(
                f"SERVICE {service.uuid}"
            )

            for char in service.characteristics:
                print(
                    f"  CHAR {char.uuid} "
                    f"properties={char.properties}"
                )

        # ----------------------------------------------------
        # 优先直接使用FFE1
        # ----------------------------------------------------

        target_char = None

        for service in ble_client.services:
            for char in service.characteristics:
                if char.uuid.lower() == FFE1_CHAR_UUID:
                    target_char = char
                    break

            if target_char is not None:
                break

        if target_char is None:
            print(
                "\n没有找到 FFE1 Characteristic。"
            )
            print(
                "请把上面打印的 SERVICE / CHAR 发回来。"
            )
            return

        print(
            "\n找到 FFE1：",
            target_char.uuid,
        )

        print(
            "FFE1 properties =",
            target_char.properties,
        )

        props = [
            p.lower()
            for p in target_char.properties
        ]

        if (
            "notify" not in props
            and
            "indicate" not in props
        ):
            print(
                "\nFFE1 不支持 notify/indicate。"
            )
            return

        print("\n正在订阅 FFE1 Notify ...")

        await ble_client.start_notify(
            target_char.uuid,
            notification_handler,
        )

        print("订阅成功。")
        print("等待 STM32 UART5 数据...\n")

        while ble_client.is_connected:
            await asyncio.sleep(0.1)

    except BaseException as exc:
        print("\nBLE错误")
        print(
            "类型：",
            type(exc).__name__,
        )
        print(
            "str：",
            str(exc),
        )
        print(
            "repr：",
            repr(exc),
        )

        print("\n完整 Traceback：")
        traceback.print_exc()

    finally:
        if (
            ble_client is not None
            and
            ble_client.is_connected
        ):
            try:
                await ble_client.stop_notify(
                    FFE1_CHAR_UUID
                )
            except Exception:
                pass

            try:
                await ble_client.disconnect()
            except Exception:
                pass


# ============================================================
# Matplotlib
# ============================================================

fig, ax = plt.subplots()

line_before, = ax.plot(
    [],
    [],
    label="Before Kalman (Accel)",
)

line_after, = ax.plot(
    [],
    [],
    label="After Kalman",
)

ax.set_title(
    "BMI270 Kalman Filter Comparison"
)

ax.set_xlabel(
    "Time (s)"
)

ax.set_ylabel(
    "Pitch angle (deg)"
)

ax.grid(True)
ax.legend()


# ============================================================
# 保留最近8秒
# ============================================================

def trim_data():
    if not times:
        return

    threshold = (
        times[-1]
        -
        WINDOW_SECONDS
    )

    while (
        times
        and
        times[0] < threshold
    ):
        times.popleft()
        before_values.popleft()
        after_values.popleft()


# ============================================================
# 图像更新
# ============================================================

def plot_update(_frame):
    trim_data()

    if not times:
        return (
            line_before,
            line_after,
        )

    x = list(times)

    y_before = list(
        before_values
    )

    y_after = list(
        after_values
    )

    line_before.set_data(
        x,
        y_before,
    )

    line_after.set_data(
        x,
        y_after,
    )

    xmin = x[0]
    xmax = x[-1]

    if xmax <= xmin:
        xmax = xmin + 1.0

    ax.set_xlim(
        xmin,
        xmax,
    )

    all_y = (
        y_before
        +
        y_after
    )

    ymin = min(all_y)
    ymax = max(all_y)

    span = ymax - ymin

    if span < 1.0:
        center = (
            ymin + ymax
        ) / 2.0

        ymin = center - 0.5
        ymax = center + 0.5
    else:
        margin = span * 0.1

        ymin -= margin
        ymax += margin

    ax.set_ylim(
        ymin,
        ymax,
    )

    return (
        line_before,
        line_after,
    )


# ============================================================
# 后台BLE线程
# ============================================================

def start_ble_thread():
    def worker():
        asyncio.run(
            bluetooth_task()
        )

    thread = threading.Thread(
        target=worker,
        daemon=True,
    )

    thread.start()

    return thread


# ============================================================
# main
# ============================================================

def main():
    global csv_file
    global csv_writer

    csv_file = CSV_PATH.open(
        "w",
        newline="",
        encoding="utf-8",
    )

    csv_writer = csv.writer(
        csv_file
    )

    csv_writer.writerow([
        "time_s",
        "before_kalman_deg",
        "after_kalman_deg",
    ])

    start_ble_thread()

    animation = FuncAnimation(
        fig,
        plot_update,
        interval=30,
        cache_frame_data=False,
    )

    # 防止被垃圾回收
    _ = animation

    try:
        plt.show()

    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

        print(
            "\nCSV 已保存到：",
            CSV_PATH.resolve(),
        )


if __name__ == "__main__":
    main()