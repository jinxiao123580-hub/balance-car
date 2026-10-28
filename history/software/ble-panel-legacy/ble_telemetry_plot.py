#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ble_telemetry_plot.py — STM32 平衡车 JDY-23 蓝牙诊断实时绘图

适配当前固件（正式控制版，2026 版 18 列）：
- JDY-23 / FFE0 + FFE1 Notify
- STM32 UART5 当前使用 9600 baud
- D,<18列>：周期诊断快照（固件只发 D 行，无 # 表头 / E 事件行）

关键修复：
1. 内置 18 列定义，不依赖开机时 #D 表头；BLE 晚连接也能直接解析绘图。
2. 图例、坐标轴、面板标题全部改成中文，并注明每条曲线含义。
3. 标题实时显示已解析行数、收到字节数、运行时间。
"""

import argparse
import asyncio
import csv
import importlib.util
import threading
import traceback
from collections import deque
from pathlib import Path

from bleak import BleakClient, BleakScanner

# ============================================================
# Matplotlib 后端 + 中文字体
# ============================================================
import matplotlib

if (
    importlib.util.find_spec("PyQt5")
    or importlib.util.find_spec("PySide6")
    or importlib.util.find_spec("PySide2")
):
    matplotlib.use("QtAgg")
elif importlib.util.find_spec("tkinter"):
    matplotlib.use("TkAgg")
else:
    print(
        "[warn] 未找到 GUI 后端（PyQt5/PySide6/tkinter），无法弹出实时图窗；"
        "数据仍可写入 CSV。"
    )

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Windows 优先使用微软雅黑；没有则依次回退
plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "Noto Sans CJK SC",
    "Arial Unicode MS",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False

# ============================================================
# BLE 参数
# ============================================================
FFE0_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
FFE1_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
DEFAULT_NAME = "JDY-23"

STOP_REASONS = {
    0: "正常/无停机",
    1: "IMU 连续读取失败",
    2: "倾倒确认停机",
    3: "严重倾倒紧急停机",
}

# ============================================================
# 当前固件固定 18 列（正式控制版）
# 注意：电脑通常在 STM32 已经启动后才建立 BLE 连接，
#       固件只发 D 行、不发 #D 表头，所以必须从一开始就用这份内置列名。
# 格式：D,ms,pitch100,gyro100,target100,balHz,spdAng100,auth1000,
#       spdDirHz,posDirHz,finalHz,speed1000,raw1000,pos100,enc1,enc2,m1Hz,m2Hz
# ============================================================
FIRMWARE_COLS = [
    "ms", "pitch100", "gyro100", "target100", "balHz", "spdAng100", "auth1000",
    "spdDirHz", "posDirHz", "finalHz", "speed1000", "raw1000", "pos100",
    "enc1", "enc2", "m1Hz", "m2Hz",
]


def scale_for(name: str) -> int:
    if name.endswith("1000"):
        return 1000
    if name.endswith("100"):
        return 100
    return 1


def display_name(name: str) -> str:
    """pitch100 -> pitch，speed1000 -> speed，其余保持不变。"""
    if name.endswith("1000"):
        return name[:-4]
    if name.endswith("100"):
        return name[:-3]
    return name


# ============================================================
# 全局状态
# ============================================================
raw_names = list(FIRMWARE_COLS)

# 关键修复：不能初始化为空。
# 如果错过 #D 表头，D 数据仍然可以直接进入这些队列。
col_names = [display_name(n) for n in raw_names]
scales = [scale_for(n) for n in raw_names]
queues = {n: deque() for n in col_names}

events = deque()  # (t, stopReason)
recv_buffer = bytearray()
start_ms = None
csv_writer = None
csv_file = None
ble_client = None
args = None

total_rows = 0
total_bytes = 0
bad_rows = 0
last_row_t = 0.0
fig_ref = None

data_lock = threading.RLock()

# 防止错误行刷屏
_warned_len_mismatch = False


# ============================================================
# 数据解析
# ============================================================
def handle_line(text: str):
    global start_ms, raw_names, col_names, scales
    global total_rows, last_row_t, bad_rows, _warned_len_mismatch

    text = text.strip()
    if not text:
        return

    # 固件注释/表头
    if text.startswith("#"):
        if text.startswith("#D,"):
            new_raw_names = text.split(",")[1:]
            if new_raw_names:
                with data_lock:
                    raw_names = new_raw_names
                    col_names = [display_name(n) for n in raw_names]
                    scales = [scale_for(n) for n in raw_names]
                    for n in col_names:
                        queues.setdefault(n, deque())
                print(f"[info] 收到固件 #D 表头：{len(col_names)} 列，已切换为动态列定义")
        elif text.startswith("#ERR") or text.startswith("#SETUP"):
            print("[STM32]", text)
        return

    if text[0] not in ("D", "E"):
        return

    parts = text.split(",")

    # D/E 后面应该正好对应 raw_names
    if len(parts) != len(raw_names) + 1:
        bad_rows += 1
        if not _warned_len_mismatch:
            _warned_len_mismatch = True
            print(
                f"[warn] 数据列数不匹配：收到 {len(parts)-1} 列，"
                f"当前定义 {len(raw_names)} 列；该行已丢弃。"
            )
        return

    try:
        nums = [float(p) for p in parts[1:]]
    except ValueError:
        bad_rows += 1
        return

    t_ms = int(nums[0])
    if start_ms is None:
        start_ms = t_ms
    t = (t_ms - start_ms) / 1000.0

    with data_lock:
        # 先把本行转成缩放后的显示值
        scaled_map = {}
        for name, scale, v in zip(col_names, scales, nums):
            value = v / scale
            scaled_map[name] = value
            queues.setdefault(name, deque()).append((t, value))
            if len(queues[name]) > 200000:
                queues[name].popleft()

        if text[0] == "E":
            reason = int(scaled_map.get("lastStop", 0))
            events.append((t, reason))
            print(
                f"[事件] t={t:7.1f}s  "
                f"原因={reason}:{STOP_REASONS.get(reason, '未知')}  "
                f"pitch={scaled_map.get('pitch', 0.0):.2f}°  "
                f"finalHz={scaled_map.get('finalHz', 0.0):.0f}"
            )

        total_rows += 1
        last_row_t = t

    if csv_writer is not None:
        try:
            csv_writer.writerow([text[0], f"{t:.4f}"] + [f"{v:.4f}" for v in nums])
        except Exception:
            pass

    if total_rows == 1:
        print(
            f"[data] 开始收到数据！第1行 t={t:.1f}s "
            f"pitch={scaled_map.get('pitch', 0.0):.2f}° "
            f"finalHz={scaled_map.get('finalHz', 0.0):.0f}"
        )
        print("[info] 即使没有收到 #D 表头，也已使用内置 18 列定义直接绘图。")
    elif total_rows % 50 == 0:
        print(
            f"[data] 已收 {total_rows} 行，t={t:.1f}s，"
            f"pitch={scaled_map.get('pitch', 0.0):.2f}°，"
            f"speed={scaled_map.get('speed', 0.0):.3f}"
        )


def on_received(_sender, data: bytearray):
    global recv_buffer, total_bytes

    total_bytes += len(data)
    recv_buffer.extend(data)

    while b"\n" in recv_buffer:
        raw_line, _, remaining = recv_buffer.partition(b"\n")
        recv_buffer = bytearray(remaining)
        handle_line(raw_line.decode("utf-8", errors="ignore"))


# ============================================================
# 数据窗口裁剪
# ============================================================
def trim_old_locked():
    if not col_names:
        return

    # 优先使用 ms/pitch/speed 中第一个有数据的队列确定当前时间
    ref_q = None
    for key in ("pitch", "speed", "ms"):
        q = queues.get(key)
        if q:
            ref_q = q
            break

    if not ref_q:
        return

    threshold = ref_q[-1][0] - args.window

    for q in queues.values():
        while q and q[0][0] < threshold:
            q.popleft()

    while events and events[0][0] < threshold:
        events.popleft()


# ============================================================
# BLE
# ============================================================
async def scan_all(timeout: float = 12.0):
    print(f"扫描 {timeout}s 内所有可见 BLE 设备 ...")
    devices = await BleakScanner.discover(timeout=timeout, return_adv=True)

    if not devices:
        print("没有发现任何 BLE 设备。")
        return

    print(f"发现 {len(devices)} 个设备：")
    for addr, item in devices.items():
        # 兼容不同 bleak 版本：可能是 AdvertisementData 或 (device, adv) 元组
        if isinstance(item, tuple):
            device, adv = item
        else:
            adv = item
            device = getattr(adv, "device", None)
        name = (device.name if device else "") or (adv.local_name or "(无名字)")
        print(f"  {addr}  rssi={adv.rssi:>4}  name={name!r}")


async def find_device(name: str):
    print(f"正在扫描 {name} ...")
    wanted = name.lower()

    def matcher(d, adv):
        names = [
            (d.name or "").strip().lower(),
            (getattr(adv, "local_name", None) or "").strip().lower(),
        ]
        return any(
            n == wanted or (len(wanted) >= 3 and wanted in n)
            for n in names
            if n
        )

    device = await BleakScanner.find_device_by_filter(matcher, timeout=10.0)

    if device is None:
        print(f"\n没有找到 {name}。")
        print("可运行 --scan 检查实际广播名称。")
        return None

    print(f"\n找到设备: name={device.name!r} address={device.address}")
    return device


async def ble_task(name: str, addr_type: str = "auto"):
    global ble_client

    device = await find_device(name)
    if device is None:
        return

    types_to_try = ["public", "random"] if addr_type == "auto" else [addr_type]

    for at in types_to_try:
        print(f"\n准备连接 BLE ... (address_type={at})")
        ble_client = BleakClient(
            device,
            services=[FFE0_SERVICE_UUID],
            timeout=60.0,
            pair=False,
            winrt={"address_type": at, "use_cached_services": True},
        )

        try:
            await asyncio.wait_for(ble_client.connect(), timeout=25.0)
            if ble_client.is_connected:
                print(f"连接成功 (address_type={at})")
                break
            ble_client = None
        except Exception as exc:
            print(f"address_type={at} 连接失败: {type(exc).__name__} {str(exc)[:120]}")
            try:
                await ble_client.disconnect()
            except Exception:
                pass
            ble_client = None

    if ble_client is None or not ble_client.is_connected:
        print("\nBLE 连接失败。")
        return

    try:
        target_char = None

        for service in ble_client.services:
            for char in service.characteristics:
                if char.uuid.lower() == FFE1_CHAR_UUID:
                    target_char = char
                    break
            if target_char is not None:
                break

        if target_char is None:
            print("\n没有找到 FFE1 Characteristic。")
            return

        props = [p.lower() for p in target_char.properties]
        if "notify" not in props and "indicate" not in props:
            print("\nFFE1 不支持 Notify/Indicate。")
            return

        print(f"\n找到 FFE1: {target_char.uuid}，正在订阅 Notify ...")
        await ble_client.start_notify(target_char.uuid, on_received)
        print("订阅成功，等待 STM32 UART5 数据...（关闭图窗即可退出）")

        while ble_client.is_connected:
            await asyncio.sleep(0.1)

    except BaseException as exc:
        print("\nBLE错误:", type(exc).__name__, str(exc))
        traceback.print_exc()
    finally:
        if ble_client is not None and ble_client.is_connected:
            try:
                await ble_client.stop_notify(FFE1_CHAR_UUID)
            except Exception:
                pass
            try:
                await ble_client.disconnect()
            except Exception:
                pass


def start_ble_thread(name: str, addr_type: str = "auto"):
    t = threading.Thread(
        target=lambda: asyncio.run(ble_task(name, addr_type)),
        daemon=True,
    )
    t.start()
    return t


# ============================================================
# 绘图辅助
# ============================================================
def set_axis_from_series(ax, series_dict):
    """根据多个 (x,y) 序列自动设置坐标范围。"""
    valid = [points for points in series_dict.values() if points]
    if not valid:
        return

    xs = [x for points in valid for x, _ in points]
    ys = [y for points in valid for _, y in points]

    if xs:
        xmin, xmax = min(xs), max(xs)
        if xmax <= xmin:
            xmax = xmin + 1.0
        ax.set_xlim(xmin, xmax)

    if ys:
        ymin, ymax = min(ys), max(ys)
        span = ymax - ymin
        if span < 1e-9:
            pad = max(1.0, abs(ymin) * 0.1)
        else:
            pad = span * 0.10
        ax.set_ylim(ymin - pad, ymax + pad)


SCRIPT_VERSION = "V3_18COL_2026"

def build_axes():
    fig, axs = plt.subplots(6, 1, figsize=(14, 13), sharex=True)
    fig.suptitle(f"平衡车实时诊断 V3（18列正式版）— {args.name}", fontsize=15)

    # ---------------- 1. 姿态与目标角 ----------------
    a1 = axs[0]
    l1 = {
        "pitch": a1.plot([], [], label="俯仰角 pitch：Kalman融合后的车体倾角", lw=1.4)[0],
        "target": a1.plot([], [], label="目标平衡角 target：控制器希望达到的倾角", lw=1.2)[0],
    }
    a1.set_title("① 姿态角与目标角", loc="left", fontsize=10)
    a1.set_ylabel("角度 (°)")
    a1.grid(True, alpha=0.35)
    a1.legend(loc="upper right", fontsize=8)

    # ---------------- 2. 角速度 ----------------
    a2 = axs[1]
    l2 = {
        "gyro": a2.plot([], [], label="俯仰角速度 gyro：车体前后旋转速度", lw=1.2)[0],
    }
    a2.set_title("② 车体俯仰角速度", loc="left", fontsize=10)
    a2.set_ylabel("角速度 (°/s)")
    a2.grid(True, alpha=0.35)
    a2.legend(loc="upper right", fontsize=8)

    # ---------------- 3. 融合速度 / 位置 ----------------
    a3 = axs[2]
    l3 = {
        "speed": a3.plot([], [], label="融合速度 speed：IMU预测 + 编码器修正后的最终车速", lw=1.4)[0],
        "raw": a3.plot([], [], label="综合编码器速度 raw：由左右编码器合成的原始车速", lw=1.1)[0],
    }
    a3b = a3.twinx()
    l3p = a3b.plot(
        [],
        [],
        label="累计位置 pos：相对开机位置的位移",
        lw=1.0,
        ls=":",
    )[0]
    a3.set_title("③ 车辆融合速度 / 综合编码器速度 / 位置", loc="left", fontsize=10)
    a3.set_ylabel("速度")
    a3b.set_ylabel("位置 (count)")
    a3.grid(True, alpha=0.35)
    a3.legend(loc="upper left", fontsize=8)
    a3b.legend(loc="upper right", fontsize=8)

    # ---------------- 4. 左右编码器累计位置 ----------------
    a4 = axs[3]
    l4 = {
        "enc1": a4.plot([], [], label="编码器1累计 enc1：1号轮当前位置计数", lw=1.3)[0],
        "enc2": a4.plot([], [], label="编码器2累计 enc2：2号轮当前位置计数", lw=1.3)[0],
    }
    a4.set_title("④ 左右编码器累计位置（确认两轮反馈是否都正常）", loc="left", fontsize=10)
    a4.set_ylabel("count")
    a4.grid(True, alpha=0.35)
    a4.legend(loc="upper right", fontsize=8)

    # ---------------- 5. 控制器输出 ----------------
    a5 = axs[4]
    l5 = {
        "finalHz": a5.plot([], [], label="最终输出 finalHz：送给两电机前的总平衡命令", lw=1.4)[0],
        "balHz": a5.plot([], [], label="直立PD balHz：由倾角误差产生的主控制量", lw=1.2)[0],
        "spdDirHz": a5.plot([], [], label="速度修正 spdDirHz：速度P直接STEP阻尼", lw=1.0)[0],
        "posDirHz": a5.plot([], [], label="位置修正 posDirHz：把车辆拉回起点", lw=1.0)[0],
    }
    a5b = a5.twinx()
    l5s = a5b.plot(
        [],
        [],
        label="速度目标角修正 spdAng",
        lw=0,
        marker=".",
        ms=3,
    )[0]
    a5.set_title("⑤ 控制器各部分 STEP 输出", loc="left", fontsize=10)
    a5.set_ylabel("STEP频率 (Hz)")
    a5b.set_ylabel("角度(°)")
    a5.grid(True, alpha=0.35)
    a5.legend(loc="upper left", fontsize=8)
    a5b.legend(loc="upper right", fontsize=8)

    # ---------------- 6. 电机命令频率 ----------------
    a6 = axs[5]
    l6 = {
        "m1Hz": a6.plot([], [], label="电机1命令 m1Hz：限幅后的STEP频率", lw=1.1)[0],
        "m2Hz": a6.plot([], [], label="电机2命令 m2Hz：限幅后的STEP频率", lw=1.1)[0],
    }
    a6.set_title("⑥ 电机实际命令频率", loc="left", fontsize=10)
    a6.set_ylabel("STEP频率 (Hz)")
    a6.set_xlabel("时间 (s)")
    a6.grid(True, alpha=0.35)
    a6.legend(loc="upper right", fontsize=8)

    # 页脚说明
    fig.text(
        0.01,
        0.004,
        "重点看震荡：若 pitch 在平衡点附近来回摆且 balHz 高频正负交替 → 直立环增益/阻尼配合问题；"
        "若 spdDirHz/posDirHz 与 pitch 相位不对 → 外环速度/位置环与直立环打架。",
        fontsize=8,
    )

    fig.subplots_adjust(top=0.94, bottom=0.055, hspace=0.42, left=0.09, right=0.91)

    return fig, [
        (a1, l1, None, None),
        (a2, l2, None, None),
        (a3, l3, a3b, [(l3p, "pos")]),
        (a4, l4, None, None),
        (a5, l5, a5b, [(l5s, "spdAng")]),
        (a6, l6, None, None),
    ]

def update(_frame, panels):
    global fig_ref

    with data_lock:
        trim_old_locked()

        # 拷贝一份，避免 BLE 线程正在 append 时 Matplotlib 同时遍历
        data_snapshot = {
            name: list(q)
            for name, q in queues.items()
        }
        events_snapshot = list(events)
        rows_snapshot = total_rows
        t_snapshot = last_row_t
        bytes_snapshot = total_bytes
        bad_snapshot = bad_rows

    if fig_ref is not None:
        status = (
            f"数据行={rows_snapshot}  接收={bytes_snapshot}B  "
            f"时间={t_snapshot:.1f}s  保护事件={len(events_snapshot)}  坏行={bad_snapshot}"
        )
        if rows_snapshot == 0:
            status += "  [等待有效 D/E 数据]"
        fig_ref.suptitle(f"平衡车实时诊断 V3 — {args.name}  |  {status}", fontsize=14)

    # 清理并重画保护事件竖线
    for p in panels:
        ax = p[0]
        for vl in list(getattr(ax, "_evlines", [])):
            try:
                vl.remove()
            except Exception:
                pass
        ax._evlines = []
        for ev_t, _reason in events_snapshot[-30:]:
            ax._evlines.append(ax.axvline(ev_t, alpha=0.25, lw=1))

    # 普通面板
    for p in panels:
        ax = p[0]
        lines_map = p[1]

        main_series = {}
        for name, line in lines_map.items():
            pts = data_snapshot.get(name, [])
            main_series[name] = pts
            if pts:
                line.set_data(
                    [x for x, _ in pts],
                    [y for _, y in pts],
                )

        set_axis_from_series(ax, main_series)

        # 右轴 twin 曲线（pos / spdAng 等）
        if p[2] is not None and p[3] is not None:
            twin_series = {}
            for line, key in p[3]:
                pts = data_snapshot.get(key, [])
                twin_series[key] = pts
                if pts:
                    line.set_data(
                        [x for x, _ in pts],
                        [y for _, y in pts],
                    )
            if twin_series:
                set_axis_from_series(p[2], twin_series)

    out = []
    for p in panels:
        out.extend(p[1].values())
        if isinstance(p[2], dict):
            out.extend(p[2].values())
        elif p[2] is not None and p[3] is not None:
            for line, _key in p[3]:
                out.append(line)
    return out


# ============================================================
# main
# ============================================================
def main():
    global args, csv_file, csv_writer, fig_ref

    parser = argparse.ArgumentParser(description="STM32 平衡车 JDY-23 实时诊断绘图")
    parser.add_argument("--name", default=DEFAULT_NAME, help="BLE设备名，默认 JDY-23")
    parser.add_argument("--csv", default="ble_diag_log.csv", help="CSV保存路径")
    parser.add_argument("--window", type=float, default=30.0, help="图中保留最近多少秒，默认30秒")
    parser.add_argument(
        "--addr-type",
        default="auto",
        choices=["auto", "public", "random"],
        help="BLE地址类型；auto=先public再random",
    )
    parser.add_argument("--scan", action="store_true", help="只扫描 BLE，不连接")
    parser.add_argument("--no-save", action="store_true", help="不保存 CSV")
    args = parser.parse_args()

    if args.scan:
        asyncio.run(scan_all())
        return

    if not args.no_save:
        csv_path = Path(args.csv)
        try:
            csv_file = csv_path.open("w", newline="", encoding="utf-8-sig")
        except PermissionError:
            # Excel/WPS 正占用旧日志时，自动换一个文件名继续运行，
            # 不再因为 CSV 被占用而导致整个 BLE 绘图程序退出。
            index = 1
            while True:
                candidate = csv_path.with_name(
                    f"{csv_path.stem}_{index}{csv_path.suffix}"
                )
                try:
                    csv_file = candidate.open("w", newline="", encoding="utf-8-sig")
                    args.csv = str(candidate)
                    print(
                        f"[warn] {csv_path.name} 正被占用，本次自动保存为 {candidate.name}"
                    )
                    break
                except PermissionError:
                    index += 1

        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["type", "time_s"] + raw_names)

    print(f"[版本] {SCRIPT_VERSION} —— 18列正式控制版")
    print(f"[info] 内置固件列定义：{len(raw_names)} 列")
    print("[info] 固件只发 D 行；BLE 晚连接也能直接出图。")

    start_ble_thread(args.name, args.addr_type)

    fig, panels = build_axes()
    fig_ref = fig

    # 保存引用，避免动画对象被垃圾回收
    ani = FuncAnimation(
        fig,
        update,
        fargs=(panels,),
        interval=100,
        cache_frame_data=False,
    )
    fig._ani_ref = ani

    plt.show()

    if csv_file is not None:
        csv_file.flush()
        csv_file.close()
        print("\nCSV 已保存到：", Path(args.csv).resolve())


if __name__ == "__main__":
    main()