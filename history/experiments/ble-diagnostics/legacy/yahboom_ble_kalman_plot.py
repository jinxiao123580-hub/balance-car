import asyncio
import csv
import threading
from collections import deque
from pathlib import Path

from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

DEVICE_NAME = "YahBoom_BL"
PREFERRED_UUID_FRAGMENT = "ffe1"
WINDOW_SECONDS = 8.0
CSV_PATH = Path("kalman_compare_ble.csv")

times = deque()
before_values = deque()
after_values = deque()
start_ms = None
receive_buffer = bytearray()
csv_file = None
csv_writer = None
ble_client = None

async def find_device():
    print(f"正在扫描 {DEVICE_NAME} ...")
    devices = await BleakScanner.discover(timeout=8.0)
    for dev in devices:
        raw_name = dev.name or ""
        clean_name = raw_name.strip()
        print(f"发现: {repr(raw_name):22s}  {dev.address}")
        if clean_name.lower() == DEVICE_NAME.lower():
            return dev
    return None

def choose_notify_characteristic(services):
    notify_candidates = []
    print("\n检测到的 BLE Services / Characteristics：")
    for service in services:
        print(f"\nService: {service.uuid}")
        for char in service.characteristics:
            props = [p.lower() for p in char.properties]
            print(f"  Characteristic: {char.uuid} properties={char.properties}")
            if "notify" in props or "indicate" in props:
                notify_candidates.append(char)

    if not notify_candidates:
        return None

    for char in notify_candidates:
        if PREFERRED_UUID_FRAGMENT in char.uuid.lower():
            return char

    return notify_candidates[0]

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
        return

    if start_ms is None:
        start_ms = t_ms

    t = (t_ms - start_ms) / 1000.0

    times.append(t)
    before_values.append(before)
    after_values.append(after)

    if csv_writer is not None:
        csv_writer.writerow([f"{t:.4f}", f"{before:.4f}", f"{after:.4f}"])

def notification_handler(_sender, data: bytearray):
    global receive_buffer
    receive_buffer.extend(data)

    while b"\n" in receive_buffer:
        raw_line, _, remaining = receive_buffer.partition(b"\n")
        receive_buffer = bytearray(remaining)
        text = raw_line.decode("utf-8", errors="ignore")
        parse_line(text)

async def get_services_compat(client):
    services = getattr(client, "services", None)
    if services is not None:
        try:
            list(services)
            return services
        except Exception:
            pass

    get_services = getattr(client, "get_services", None)
    if get_services is not None:
        result = get_services()
        if asyncio.iscoroutine(result):
            result = await result
        return result

    raise RuntimeError("当前 Bleak 版本无法读取 BLE services")

async def bluetooth_task():
    global ble_client

    device = await find_device()
    if device is None:
        print("\n没有找到 YahBoom_BL。确认模块已上电，Windows 蓝牙已开启。")
        return

    print(f"\n找到设备：{repr(device.name)}  {device.address}")
    ble_client = BleakClient(device)

    try:
        print("正在连接 BLE...")
        await ble_client.connect()
        print(f"连接状态：{ble_client.is_connected}")

        services = await get_services_compat(ble_client)
        notify_char = choose_notify_characteristic(services)

        if notify_char is None:
            print("\n没有找到支持 Notify/Indicate 的 Characteristic。")
            return

        print(f"\n使用 Notify Characteristic：{notify_char.uuid}")

        await ble_client.start_notify(
            notify_char.uuid,
            notification_handler,
        )

        print("\n已开始接收数据。")
        print("等待 STM32 的 UART5 数据...\n")

        while ble_client.is_connected:
            await asyncio.sleep(0.1)

    except Exception as exc:
        print(f"\nBLE 连接/接收错误：{exc}")

    finally:
        if ble_client is not None and ble_client.is_connected:
            try:
                await ble_client.disconnect()
            except Exception:
                pass

fig, ax = plt.subplots()

line_before, = ax.plot([], [], label="Before Kalman (Accel)")
line_after, = ax.plot([], [], label="After Kalman")

ax.set_title("BMI270 Kalman Filter Comparison")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Pitch angle (deg)")
ax.grid(True)
ax.legend()

def trim_data():
    if not times:
        return

    threshold = times[-1] - WINDOW_SECONDS

    while times and times[0] < threshold:
        times.popleft()
        before_values.popleft()
        after_values.popleft()

def plot_update(_frame):
    trim_data()

    if not times:
        return line_before, line_after

    x = list(times)
    y_before = list(before_values)
    y_after = list(after_values)

    line_before.set_data(x, y_before)
    line_after.set_data(x, y_after)

    xmin = x[0]
    xmax = x[-1]
    if xmax <= xmin:
        xmax = xmin + 1.0
    ax.set_xlim(xmin, xmax)

    all_y = y_before + y_after
    ymin = min(all_y)
    ymax = max(all_y)
    span = ymax - ymin

    if span < 1.0:
        center = (ymin + ymax) / 2.0
        ymin = center - 0.5
        ymax = center + 0.5
    else:
        margin = span * 0.10
        ymin -= margin
        ymax += margin

    ax.set_ylim(ymin, ymax)
    return line_before, line_after

def start_ble_thread():
    def worker():
        asyncio.run(bluetooth_task())

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return thread

def main():
    global csv_file, csv_writer

    csv_file = CSV_PATH.open("w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["time_s", "before_kalman_deg", "after_kalman_deg"])

    start_ble_thread()

    animation = FuncAnimation(
        fig,
        plot_update,
        interval=30,
        cache_frame_data=False,
    )
    _ = animation

    try:
        plt.show()
    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

        print(f"\nCSV 已保存到：{CSV_PATH.resolve()}")

if __name__ == "__main__":
    main()