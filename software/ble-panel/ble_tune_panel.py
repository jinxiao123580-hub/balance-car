# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[2]

# -*- coding: utf-8 -*-
"""
平衡车蓝牙在线调参面板 + 数据落盘 (JDY-23 / BLE FFE0-FFE1)
==========================================================
- 滑动条 / 数值框实时修改参数（松手即发）
- 收到的每行遥测 + 当时全部参数 -> 实时追加到本地 CSV
- 命令协议见固件 handleBtCommand()：
    P/Kp D/Kd O/平衡点 Z/死区 L/限幅 S/斜坡
    K/speedKp I/speedKi Q/speedDirectKp H/MAX_SPEED_DIRECT_OUTPUT
    N/MAX_SPEED_ANGLE W/SPEED_INTEGRAL_LIMIT F/OUTER_FULL G/OUTER_DISABLE
    N/MAX_SPEED_ANGLE W/SPEED_INTEGRAL_LIMIT F/OUTER_FULL G/OUTER_DISABLE
    T/turnKp U/turnKd R/verticalLoopGain   ?=回显全部
  - 固件已加 WHEEL_TO_HZ_SCALE=4 量纲标定：直投 = Q * speedError * 4（有效增益=Q×4）
  - 面板默认载入当前分阶段调参基线：先用目标角速度环验证低频回正，Q=0 暂停直接制动
用法：python ble_tune_panel.py
"""
import asyncio, sys, csv, os, time, threading, re
from collections import deque

try:
    import matplotlib
    matplotlib.use("Qt5Agg")
    import matplotlib.font_manager as _fm
    _ok = False
    for _f in ("Microsoft YaHei", "SimHei", "SimSun", "PingFang SC", "Noto Sans CJK SC"):
        try:
            if any(_f.lower() in fn.name.lower() for fn in _fm.fontManager.ttflist):
                matplotlib.rcParams["font.sans-serif"] = [_f, "DejaVu Sans"]
                _ok = True
                break
        except Exception:
            pass
    _fm = None
    matplotlib.rcParams["axes.unicode_minus"] = False
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    from matplotlib.figure import Figure
    _MPL_OK = True
except Exception:
    _MPL_OK = False

FFE0_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
FFE1_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
KNOWN_MAC = _project_os.environ.get("BALANCE_CAR_BLE_ADDRESS", "")
# JDY-23 低功耗模式 0 下，串口 10s 无数据会自动再次睡眠。
# keep-alive 间隔须明显小于 10s；发空行(\n)固件直接 return，无回显噪声。
KEEPALIVE_SEC = 3
# 断线后自动重连（BleakClient.connect 正常会抛/置 is_connected=False，靠 guard 轮询触发）
AUTO_RECONNECT = True
# 输出目录（每次开始新录制时生成唯一文件名，避免多实例覆盖同一文件）
OUT_DIR = str(_PROJECT_ROOT / "docs" / "validation" / "recordings")


def make_csv_path():
    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    millis = int((time.time() % 1.0) * 1000.0)
    return os.path.join(
        OUT_DIR, f"ble_tune_log_{stamp}_{millis:03d}_p{os.getpid()}.csv")

# 遥测列（与固件 sendTelemetry 完整控制链一致，整数缩放）+ 参数列
TELEM_COLS = [
    "ms", "pitch100", "gyro100", "target100",
    "balPHz", "balDHz", "balHz", "spdAng100", "auth1000",
    "spdDirRawHz", "spdDirHz", "posSpdTarget1000", "mixHz", "limitedHz",
    "finalHz", "turnHz", "speed1000", "raw1000",
    "speedAccel1000", "predSpeed1000", "predDelta1000", "spdDirUnclampedHz",
    "pos100", "posError100", "enc1", "enc2", "m1Hz", "m2Hz", "accPitch100",
    "kalmanBias1000", "gyroIntegAngle100",
    "balAccelRawHzStep", "balAccelLimitedHzStep", "integratedSpeedHz",
    "speedError1000", "speedP100", "speedI100",
    "speedAngleUnclamped100", "speedAngleLimited100", "balAccelEffectiveLimitHzStep",
    "rawSpeedEncoderMode",
    "dynamicRx1000", "accelMagErrX1000", "fallStopCount",
    "gyroAbsMax100", "accPitchDevMax100", "balHzAbsMax",
    "kalmanBiasMin1000", "kalmanBiasMax1000",
]

# 参数定义: (命令字母, 变量名, 默认值, 最小, 最大, 小数位, 单位, 中文显示名)
# 默认值 = 当前调参基线。连接后的车端回显只显示在右侧，不覆盖这些待下发输入值。
PARAMS = [
    ("P", "balanceKp", 1000.0, -10000.0, 10000.0, 0, "Hz/°", "直立环P"),
    ("D", "balanceKd", 100.0, -10000.0, 10000.0, 1, "Hz·s/°", "直立环D阻尼"),
    ("O", "balanceAngleOffset", 4.0, -10000.0, 10000.0, 2, "°", "平衡目标角"),
    ("Z", "deadband", 0.0, -10000.0, 10000.0, 2, "°", "死区(°内不动)"),
    ("L", "limitHz", 1750.0, -10000.0, 10000.0, 0, "Hz", "输出限幅"),
    ("S", "maxAccelIncrement", 100.0, -10000.0, 10000.0, 0, "Hz/10ms", "常规STEP频率增量限幅"),
    ("B", "highSpeedAccelStartHz", 1000.0, -10000.0, 10000.0, 0, "Hz", "高速继续加速保护起点"),
    ("H", "highSpeedAccelLimitHz", 40.0, -10000.0, 10000.0, 0, "Hz/10ms", "高速继续加速增量上限"),
    ("R", "balanceAccelScale", 0.1, -10000.0, 10000.0, 4, "", "直立PD→频率增量比例"),
    ("K", "speedKp", -0.02, -10000.0, 10000.0, 2, "", "速度环P"),
    ("I", "speedKi", 0.0, -10000.0, 10000.0, 2, "", "速度环I(条件积分)"),
    ("N", "MAX_SPEED_ANGLE", 2.0, -10000.0, 10000.0, 1, "°", "速度修正角上限"),
    ("W", "SPEED_INTEGRAL_LIMIT", 50.0, -10000.0, 10000.0, 1, "", "速度积分限幅"),
    ("J", "speedErrorDeadband", 2.0, -10000.0, 10000.0, 2, "", "速度误差死区"),
    ("E", "speedKd", 0.0, -10000.0, 10000.0, 3, "", "速度环微分阻尼"),
    ("X", "positionKp", 5.0, -10000.0, 10000.0, 3, "", "位置P→目标速度"),
    ("Y", "maxPositionReturnSpeed", 1.0, -10000.0, 10000.0, 2, "count/50ms", "最大回位速度"),
    ("M", "positionDeadband", 0.0, -10000.0, 10000.0, 1, "count", "位置死区"),
    ("F", "OUTER_FULL_ANGLE", 5.0, -10000.0, 10000.0, 1, "°", "外环全权限角"),
    ("G", "OUTER_DISABLE_ANGLE", 10.0, -10000.0, 10000.0, 1, "°", "外环退出角"),
    ("T", "turnKp", 0.0, -10000.0, 10000.0, 1, "", "转向P"),
    ("U", "turnKd", 0.0, -10000.0, 10000.0, 1, "", "转向D"),
]

CSV_HEADERS = (["time_s"] + TELEM_COLS +
               [f"{cmd}_{name}" for cmd, name, *_ in PARAMS])


def helper_pad_telem(fields):
    """把遥测数值字段(parts[1:])补齐/截断到当前遥测列数，缺的补空串。"""
    n = len(TELEM_COLS)
    if len(fields) == n:
        return fields
    out = list(fields)
    if len(out) < n:
        out.extend([""] * (n - len(out)))
    else:
        out = out[:n]
    return out


class BleClient:
    def __init__(self):
        self.loop = None
        self.client = None
        self.handle = None
        self.recv_cb = None   # (rawline)->None
        self.conn_cb = None   # (str)->None
        self._wanted = False      # 用户期望保持连接(连接成功后置True, 主动断开置False)
        self._connecting = False  # 正在连接/重连 (防止并发)
        self._rxbuf = b""     # 跨 notify 帧组装整行

    def start(self):
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.create_task(self._idle())
        self.loop.create_task(self._guard())
        self.loop.run_forever()

    async def _idle(self):
        while True:
            await asyncio.sleep(86400)

    async def _guard(self):
        """保活 + 断线自动重连（与 is_connected 轮询解耦，防止 GATT 卡死）。"""
        while True:
            await asyncio.sleep(KEEPALIVE_SEC)
            if not self._wanted or self._connecting:
                continue
            cur = self.client
            if cur is None:
                continue
            try:
                up = cur.is_connected
            except Exception:
                up = False
            if up and self.handle is not None:
                # keep-alive：JDY-23 低功耗模式 0 串口 10s 无数据会睡眠。
                # 发空行 \n 固件直接 return；同时保持 BLE 链路有流量。
                try:
                    await cur.write_gatt_char(self.handle, b"\n")
                except Exception:
                    pass  # 失败会被下一次轮询的 is_connected 捕获
            elif AUTO_RECONNECT:
                self._reconnect()

    async def connect(self):
        from bleak import BleakScanner, BleakClient
        if self._connecting:
            return False
        self._connecting = True
        try:
            if self.conn_cb: self.conn_cb("扫描中…")
            devs = await BleakScanner.discover(timeout=8.0)
            device = None
            for d in devs:
                u = str(getattr(d, "metadata", {}).get("uuids", ""))
                if d.address == KNOWN_MAC or "ffe0" in u.lower():
                    device = d; break
            if device is None:
                for d in devs:
                    if "jdy" in (d.name or "").lower():
                        device = d; break
            if device is None:
                if self.conn_cb: self.conn_cb("未找到 JDY-23")
                return False
            if self.conn_cb: self.conn_cb(f"连接 {device.name or device.address}…")
            for at in ("public", "random"):
                try:
                    self.client = BleakClient(device, services=[FFE0_SERVICE_UUID], timeout=30.0, pair=False)
                    await asyncio.wait_for(self.client.connect(), timeout=20.0)
                    if self.client.is_connected:
                        if self.conn_cb: self.conn_cb("已连接 ✓")
                        break
                except Exception as e:
                    if self.conn_cb: self.conn_cb(f"{at}失败 {str(e)[:40]}")
            if not (self.client and self.client.is_connected):
                return False
            target = None
            for svc in self.client.services:
                for ch in svc.characteristics:
                    if ch.uuid.lower() == FFE1_CHAR_UUID:
                        target = ch; break
                if target: break
            if target is None:
                if self.conn_cb: self.conn_cb("未找到 FFE1"); return False
            self.handle = target.handle
            await self.client.start_notify(target.uuid, self._on_data)
            if self.conn_cb: self.conn_cb("已连接 ✓ 请求参数回显…")
            # 连接后自动向固件请求一次参数回显，让面板初值 = 车当前实际参数
            try:
                await self.client.write_gatt_char(self.handle, b"?\n")
            except Exception as e:
                if self.conn_cb: self.conn_cb(f"请求回显失败 {str(e)[:40]}")
            if self.conn_cb: self.conn_cb("已连接 ✓ 遥测流开")
            self._wanted = True
            return True
        finally:
            self._connecting = False

    def _reconnect(self):
        """可重入：直接在新线程里跑 connect()，不阻塞 asyncio guard 循环。"""
        def wrapper():
            try:
                ok = asyncio.run_coroutine_threadsafe(
                    self.connect(), self.loop).result(timeout=35)
                if self.conn_cb:
                    if ok:
                        self.conn_cb("自动重连成功 ✓")
                    else:
                        self.conn_cb("自动重连未成功，继续尝试…")
            except Exception as e:
                if self.conn_cb: self.conn_cb(f"自动重连失败 {str(e)[:40]}")
        threading.Thread(target=wrapper, daemon=True).start()

    def _on_data(self, _c, data):
        try:
            self._rxbuf += bytes(data)
        except Exception:
            return
        # 按换行拆成完整行（遥测和 PAR 回显都以 \n 结尾），残留帧留到下个 notify 再拼
        while b"\n" in self._rxbuf:
            line, self._rxbuf = self._rxbuf.split(b"\n", 1)
            try:
                txt = line.decode("utf-8", errors="ignore")
            except Exception:
                continue
            line_s = txt.strip("\r")
            if line_s and self.recv_cb:
                self.recv_cb(line_s)

    async def send(self, text):
        if self.client and self.client.is_connected and self.handle:
            try:
                await self.client.write_gatt_char(self.handle, text.encode("utf-8") + b"\n")
                return True
            except Exception as e:
                if self.conn_cb: self.conn_cb(f"发送失败 {str(e)[:40]}")
        return False


# ---------------- GUI ----------------
from PyQt5.QtWidgets import (QApplication, QWidget, QGridLayout, QLabel, QLineEdit,
                             QSlider, QPushButton, QTextEdit, QVBoxLayout, QHBoxLayout,
                             QGroupBox, QComboBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject


class Bridge(QObject):
    text = pyqtSignal(str)
    msg = pyqtSignal(str)


class TunePanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("平衡车蓝牙调参面板 + 录制")
        self.ble = BleClient()
        self.bridge = Bridge()
        self.param_val = {}   # cmd -> float sent value
        self.log_lock = threading.RLock()
        self.csv_f = None
        self.csv_w = None
        self.csv_path = None
        self.csv_rows = 0
        self.csv_recording = False
        self.csv_error = ""
        # 实时三线图环形缓冲（角度统一换算为度数）。
        # 容量取大（≈ 400s @5Hz），实际显示用按秒的滑动窗口截取，窗口宽度可调。
        self._plot_maxlen = 2000
        self._plot_window_s = 60.0               # 滑动窗口宽度（秒），可用下拉切换
        self._plot_t = deque(maxlen=2000)
        self._plot_pitch = deque(maxlen=2000)   # 滤波后 pitch (pitch100/100)
        self._plot_raw = deque(maxlen=2000)     # 直接测角度 (accPitch100/100)
        self._plot_gyro = deque(maxlen=2000)    # 陀螺仪积分角度（面板本地积分）
        self._plot_target = deque(maxlen=2000)  # 目标角 (target100/100)
        self._gyro_angle = 0.0                 # 陀螺积分累计角 °
        self._gyro_prev_ms = None              # 上一遥测的 ms（用于 dt）
        self._plot_timer = None
        self.build_ui()
        self.bridge.text.connect(self.on_telem)
        self.bridge.msg.connect(self.on_msg)
        self.ble.recv_cb = lambda s: self.bridge.text.emit(s)
        self.ble.conn_cb = lambda s: self.bridge.msg.emit(s)
        self.ble.start()
        self.open_csv()
        self.param_val = {cmd: dflt for cmd, _, dflt, *_ in PARAMS}
        self.car_param_val = dict(self.param_val)

    # ---------- UI ----------
    def build_ui(self):
        outer = QVBoxLayout(self)
        top = QHBoxLayout()
        self.status = QLabel("未连接")
        self.record_status = QLabel("录制：准备中")
        self.record_status.setStyleSheet("color:#ef6c00;")
        self.btn_conn = QPushButton("连接")
        self.btn_conn.clicked.connect(self.do_connect)
        self.btn_ask = QPushButton("? 回显")
        self.btn_ask.clicked.connect(self.do_ask)
        self.btn_all = QPushButton("下发全部")
        self.btn_all.clicked.connect(self.send_all)
        self.btn_capture = QPushButton("当前位置设为回位点")
        self.btn_capture.clicked.connect(self.capture_position)
        self.btn_new_record = QPushButton("开始新录制")
        self.btn_new_record.clicked.connect(self.start_new_recording)
        self.btn_stop_record = QPushButton("停止录制")
        self.btn_stop_record.clicked.connect(self.stop_recording)
        top.addWidget(self.status); top.addStretch()
        top.addWidget(self.record_status)
        top.addWidget(self.btn_new_record); top.addWidget(self.btn_stop_record)
        top.addWidget(self.btn_conn); top.addWidget(self.btn_ask)
        top.addWidget(self.btn_all); top.addWidget(self.btn_capture)
        outer.addLayout(top)

        # 左右分栏：左列=参数+日志，右列=实时三线图
        hsplit = QHBoxLayout()
        outer.addLayout(hsplit)

        left = QVBoxLayout()
        grid = QGridLayout()
        self.ctrl = {}
        for row, (cmd, name, dflt, lo, hi, dec, unit, disp) in enumerate(PARAMS):
            lab = QLabel(f"{cmd} {disp}")
            lab.setToolTip(f"命令 {cmd} {name}  [{lo}~{hi}{unit}]")
            box = QLineEdit(f"{dflt:.{dec}f}")
            box.setFixedWidth(90)
            box.editingFinished.connect(lambda cn=cmd: self.commit_box(cn))
            sl = QSlider(Qt.Horizontal)
            scale = 10 ** dec
            sl.setRange(int(round(lo * scale)), int(round(hi * scale)))
            sl.setValue(int(round(dflt * scale)))
            sl.setSingleStep(1)
            sl.sliderReleased.connect(lambda cn=cmd: self.commit_slider(cn))
            sl.valueChanged.connect(lambda v, cn=cmd: self.preview(cn, v))
            cmt = QLabel("")
            cmt.setStyleSheet("color:gray;")
            car = QLabel("车端: --")
            car.setMinimumWidth(125)
            car.setStyleSheet("color:#1565c0;")
            grid.addWidget(lab, row, 0)
            grid.addWidget(box, row, 1)
            grid.addWidget(sl, row, 2)
            grid.addWidget(car, row, 3)
            grid.addWidget(cmt, row, 4)
            self.ctrl[cmd] = {"box": box, "sl": sl, "dec": dec,
                              "lo": lo, "hi": hi, "cmt": cmt, "car": car,
                              "scale": scale, "dflt": dflt}
        left.addLayout(grid)

        # 批量文本参数：允许直接粘贴
        # P=850 D=10 O=-0.05 ... 或每行一个“P 850”。
        batch_group = QGroupBox("批量粘贴参数（先解析，确认后再点“下发全部”）")
        batch_layout = QHBoxLayout(batch_group)
        self.batch_input = QTextEdit()
        self.batch_input.setAcceptRichText(False)
        self.batch_input.setPlaceholderText(
            "例如：P=850 D=10 O=-0.05 K=0.02 Q=1 H=600 X=0.01 Y=4 M=5"
        )
        self.batch_input.setMaximumHeight(72)
        self.btn_parse_batch = QPushButton("解析到输入框")
        self.btn_parse_batch.clicked.connect(self.apply_pasted_params)
        batch_layout.addWidget(self.batch_input, 1)
        batch_layout.addWidget(self.btn_parse_batch)
        left.addWidget(batch_group)

        lg = QGroupBox("设备回显 / 遥测")
        lv = QVBoxLayout(lg)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setLineWrapMode(QTextEdit.NoWrap)
        self.log.setMaximumHeight(200)
        lv.addWidget(self.log)
        left.addWidget(lg)
        hsplit.addLayout(left, 0)

        # 右侧：三线叠加图（图占满右列，不被参数压扁）
        if _MPL_OK:
            from PyQt5.QtCore import QTimer
            # 右列容器：顶部一行 = 滑动窗口宽度控件，下面 = 画布
            right = QVBoxLayout()
            winrow = QHBoxLayout()
            winrow.addWidget(QLabel("显示窗口"))
            self.win_combo = QComboBox()
            self.win_combo.addItem("15 秒", 15.0)
            self.win_combo.addItem("30 秒", 30.0)
            self.win_combo.addItem("60 秒", 60.0)
            self.win_combo.addItem("120 秒", 120.0)
            self.win_combo.addItem("240 秒", 240.0)
            idx = self.win_combo.findData(self._plot_window_s)
            if idx >= 0:
                self.win_combo.setCurrentIndex(idx)
            self.win_combo.currentIndexChanged.connect(self.on_win_change)
            winrow.addWidget(self.win_combo)
            winrow.addStretch()
            right.addLayout(winrow)

            self.fig = Figure(figsize=(7.0, 5.5), dpi=100)
            self.ax = self.fig.add_subplot(111)
            self.line_target, = self.ax.plot([], [], "r-", lw=1.4, label="target 目标角")
            self.line_pitch, = self.ax.plot([], [], "b-", lw=1.4, label="pitch 滤波后")
            self.line_raw, = self.ax.plot([], [], "g-", lw=.9, alpha=.7, label="accPitch 直接测")
            self.line_gyro, = self.ax.plot([], [], "m-", lw=.9, alpha=.7, label="KF输入角(真实,固件400Hz积分)")
            self.ax.axhline(0, color="k", lw=.5)
            # 鼠标悬停：两组注解，固定在图形两上角，各显示对应线的采样 y 值
            self._hover_annot = self.ax.annotate(
                "", xy=(0.97, 0.97), xycoords="axes fraction",
                ha="right", va="top", fontsize=8,
                bbox=dict(boxstyle="round,pad=0.3", fc="lightyellow",
                           ec="0.5", alpha=0.95), zorder=6)
            self._hover_annot.set_visible(False)
            self._hover_annot2 = self.ax.annotate(
                "", xy=(0.03, 0.97), xycoords="axes fraction",
                ha="left", va="top", fontsize=8,
                bbox=dict(boxstyle="round,pad=0.3", fc="lightyellow",
                           ec="0.5", alpha=0.95), zorder=6)
            self._hover_annot2.set_visible(False)
            self.canvas = FigureCanvasQTAgg(self.fig)
            self._hover_annot.figure.canvas.mpl_connect(
                "motion_notify_event", self.on_plot_hover)
            right.addWidget(self.canvas, 1)
            hsplit.addLayout(right, 1)
            self.ax.set_ylabel("角度 °")
            self.ax.legend(loc="lower left", fontsize=8)
            self.ax.grid(True, alpha=.3)
            self.ax.set_title("目标角 vs 滤波后 vs 直接测 vs 陀螺积分（滑动窗口）")
            self._plot_timer = QTimer()
            self._plot_timer.timeout.connect(self.refresh_plot)
            self._plot_timer.start(100)
        else:
            hsplit.addWidget(QLabel("(未安装 matplotlib，实时图禁用)"))

        self.resize(1180, 760)

    # ---------- CSV ----------
    def open_csv(self):
        """打开唯一的新CSV。调用者无需持有log_lock。"""
        with self.log_lock:
            self._close_csv_locked()
            self.csv_path = make_csv_path()
            self.csv_f = open(self.csv_path, "x", newline="", encoding="utf-8")
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow(CSV_HEADERS)
            self.csv_f.flush()
            self.csv_rows = 0
            self.csv_error = ""
            self.csv_recording = True
        self.update_record_status()

    def start_new_recording(self):
        try:
            self.open_csv()
            self.log.append("开始新录制：" + os.path.basename(self.csv_path))
        except Exception as exc:
            self.csv_recording = False
            self.csv_error = str(exc)
            self.update_record_status()
            self.log.append("CSV创建失败：" + str(exc))

    def stop_recording(self):
        with self.log_lock:
            self.csv_recording = False
            self._close_csv_locked()
        self.update_record_status()
        self.log.append("CSV录制已停止")

    def update_record_status(self):
        name = os.path.basename(self.csv_path) if self.csv_path else "--"
        size = 0
        try:
            if self.csv_path:
                size = os.path.getsize(self.csv_path)
        except OSError:
            pass
        if self.csv_error:
            self.record_status.setText("录制失败：" + self.csv_error[:45])
            self.record_status.setStyleSheet("color:#c62828;font-weight:bold;")
        elif self.csv_recording:
            self.record_status.setText(
                f"● 录制中 {self.csv_rows}行 {size / 1024.0:.1f}KB · {name}")
            self.record_status.setStyleSheet("color:#2e7d32;font-weight:bold;")
        else:
            self.record_status.setText(f"■ 已停止 {self.csv_rows}行 · {name}")
            self.record_status.setStyleSheet("color:#616161;")

    def record_line(self, raw_line):
        # 遥测行 D,... -> 拆成独立列，末尾附当时全部参数值
        if raw_line.startswith("D,"):
            parts = raw_line.split(",")
            # 严格拒绝截断/混入确认文本的D行，避免污染后续CSV统计。
            if len(parts) != len(TELEM_COLS) + 1:
                return
            try:
                for value in parts[1:]:
                    float(value)
            except ValueError:
                return
            extra = helper_pad_telem(parts[1:])
            params_pos = [self.car_param_val.get(c, d) for c, d in
                          [(p[0], p[2]) for p in PARAMS]]
            csv_row = [f"{time.time():.3f}"] + extra + \
                      [f"{x}" for x in params_pos]
            with self.log_lock:
                if not self.csv_recording:
                    return
                try:
                    self.csv_w.writerow(csv_row)
                    self.csv_f.flush()
                    self.csv_rows += 1
                    if self.csv_rows == 1 or self.csv_rows % 20 == 0:
                        self.update_record_status()
                except Exception as exc:
                    # 不再静默吞错：停止当前录制并在界面上明确报警。
                    self.csv_recording = False
                    self.csv_error = str(exc)
                    self._close_csv_locked()
                    self.update_record_status()
                    self.log.append("CSV写入失败，录制已停止：" + str(exc))

    def _close_csv_locked(self):
        try:
            if self.csv_f:
                self.csv_f.flush()
                self.csv_f.close()
        except Exception:
            pass
        self.csv_f = None
        self.csv_w = None

    def close_csv(self):
        with self.log_lock:
            self.csv_recording = False
            self._close_csv_locked()

    # ---------- 回调 ----------
    def push_plot(self, s):
        """D,... 遥测行 -> 追加四线图缓冲（角度统一为 °）。列缺失时降级而不丢整行。
        紫线(真实KF输入角)由固件在400Hz积分后下发，面板不再做5Hz本地积分，避免漂移。"""
        try:
            parts = s.split(",")
            ms = float(parts[1])
            pitch = float(parts[2]) / 100.0   # pitch100 -> ° 滤波后
            target = float(parts[4]) / 100.0  # target100 -> ° 目标角
            # 预测版新增4列；兼容此前27列和更早26列遥测。
            raw = (float(parts[29]) / 100.0) if len(parts) > 31 else ((float(parts[25]) / 100.0) if len(parts) > 27 else ((float(parts[24]) / 100.0) if len(parts) > 26 else pitch))
            gyro_rate = float(parts[3]) / 100.0  # gyro100 -> °/s 陀螺角速度
            gyi = (float(parts[31]) / 100.0) if len(parts) > 31 else ((float(parts[27]) / 100.0) if len(parts) > 27 else ((float(parts[26]) / 100.0) if len(parts) > 26 else None))
        except Exception:
            return
        t = ms / 1000.0
        # 老固件(无gyroIntegAngle列)本地积分兜底：
        if gyi is None:
            if self._gyro_prev_ms is not None:
                dt = (ms - self._gyro_prev_ms) / 1000.0
                if dt > 0:
                    self._gyro_angle += gyro_rate * dt
            self._gyro_prev_ms = ms
            gyi = self._gyro_angle
        self._plot_t.append(t)
        self._plot_pitch.append(pitch)
        self._plot_target.append(target)
        self._plot_raw.append(raw)
        self._plot_gyro.append(gyi)

    def refresh_plot(self):
        if not _MPL_OK or not self._plot_t:
            return
        t = list(self._plot_t)
        # 滑动窗口：只取 [窗口左缘, 最新时间] 内的点，窗口随最新数据向右滑动
        wlen = self._plot_window_s
        xmax = t[-1]
        xmin = xmax - wlen
        # 取窗口内的索引（t 单调递增）
        j = 0
        while j < len(t) and t[j] < xmin:
            j += 1
        ts = t[j:]
        self.line_target.set_data(ts, list(self._plot_target)[j:])
        self.line_pitch.set_data(ts, list(self._plot_pitch)[j:])
        self.line_raw.set_data(ts, list(self._plot_raw)[j:])
        self.line_gyro.set_data(ts, list(self._plot_gyro)[j:])
        # Y 轴只看"滤波后 pitch + 目标角"两条线，忽略 accPitch 直接测/陀螺积分的噪声与尖峰，
        # 避免其他线的瞬时大跳把纵轴撑开、把滤波线压扁。
        focus = list(self._plot_pitch)[j:] + list(self._plot_target)[j:]
        if not focus:
            return
        fmin, fmax = min(focus), max(focus)
        span = (fmax - fmin) or 1.0
        pad = max(0.5, span * 0.15)
        self.ax.set_ylim(fmin - pad, fmax + pad)
        # X 轴固定为窗口宽度，随最新时间滑动；若数据不足窗口宽则贴住左缘
        self.ax.set_xlim(xmax - wlen, xmax)
        try:
            self.ax.relim()
            self.canvas.draw_idle()
        except Exception:
            pass

    def on_win_change(self, idx):
        """滑动窗口宽度下拉切换。"""
        data = self.win_combo.itemData(idx)
        if data is not None:
            self._plot_window_s = float(data)

    def on_plot_hover(self, event):
        """鼠标悬停：显示采样点的 y 值。右侧数据(滤波/陀螺)显示在右上，左侧数据(目标/直接测)显示在左上，各成一行。"""
        try:
            if event.inaxes is not self.ax or not self._plot_t:
                for a in (self._hover_annot, self._hover_annot2):
                    if a.get_visible():
                        a.set_visible(False)
                if not (self._hover_annot.get_visible() or self._hover_annot2.get_visible()):
                    self.canvas.draw_idle()
                return
            t = list(self._plot_t)
            # 找离鼠标最近的 x 索引
            idx = min(range(len(t)), key=lambda i: abs(t[i] - event.xdata))
            xv = t[idx]
            def val(line, dq):
                return round(list(dq)[idx], 3) if idx < len(dq) else float("nan")
            # 右侧数据(显示在右上)：target 目标角 + 真实KF输入角(紫)
            right = (f"target 目标角 : {val(self.line_target, self._plot_target):+.3f}°",
                     f"KF输入角(紫) : {val(self.line_gyro, self._plot_gyro):+.3f}°")
            # 左侧数据(显示在左上)：pitch 滤波后 + accPitch 直接测
            left = (f"pitch 滤波后   : {val(self.line_pitch, self._plot_pitch):+.3f}°",
                    f"accPitch 直接测: {val(self.line_raw, self._plot_raw):+.3f}°")
            self._hover_annot.xy = (0.97, 0.97)
            self._hover_annot.set_text("\n".join(right))
            self._hover_annot.set_visible(True)
            self._hover_annot2.xy = (0.03, 0.97)
            self._hover_annot2.set_text("\n".join(left))
            self._hover_annot2.set_visible(True)
            self.canvas.draw_idle()
        except Exception:
            pass

    def on_msg(self, s):
        self.status.setText(s)
        # 连接成功后清空绘图缓冲：避免旧会话与当前会话的数据线接在一起。
        # 每次成功建立/重连连接都会触发 on_msg("已连接 ✓...") 或 ("自动重连成功 ✓...")
        if ("已连接" in s and "✓" in s) or ("自动重连成功" in s):
            self.clear_plot_after_connect()

    def clear_plot_after_connect(self):
        """清空实时图缓冲并在图上复位，开启一段新的录制会话。"""
        self._plot_t.clear()
        self._plot_pitch.clear()
        self._plot_raw.clear()
        self._plot_gyro.clear()
        self._plot_target.clear()
        self._gyro_angle = 0.0
        self._gyro_prev_ms = None
        # 同一 CSV 文件里分段：在新会话行之前写一行注释做分界，便于事后区分
        with self.log_lock:
            try:
                if self.csv_f:
                    self.csv_f.write("# --- reconnect new session @ %s ---\n" % time.strftime("%H:%M:%S"))
                    self.csv_f.flush()
            except Exception:
                pass
        for ln in (self.line_pitch, self.line_raw, self.line_gyro, self.line_target):
            try:
                ln.set_data([], [])
            except Exception:
                pass
        try:
            self.canvas.draw_idle()
        except Exception:
            pass

    def on_telem(self, s):
        s = s.strip("\r\n")
        if not s:
            return
        # 解析固件 "?" 回显：PAR: P=900.00 D=9.0 O=-2.00 ... -> 同步到面板初值
        if s.startswith("PAR:"):
            self.apply_param_echo(s)
            return
        ack = re.fullmatch(r"([A-Za-z])=([-+]?[0-9]*\.?[0-9]+)\s+OK", s)
        if ack:
            cn = ack.group(1).upper()
            if cn in self.car_param_val:
                self.car_param_val[cn] = float(ack.group(2))
        self.record_line(s)
        # 显示：遥测行太长，截断；回显/确认正常显示
        if s.startswith("D,"):
            tail = s.split(",")
            show = ("遥测 " + tail[1] + "ms pitch=" + (tail[2] if len(tail) > 2 else "?")
                    + " P/D=" + (tail[5] if len(tail) > 5 else "?") + "/" + (tail[6] if len(tail) > 6 else "?")
                    + " direct=" + (tail[11] if len(tail) > 11 else "?")
                    + " mix/lim/final=" + (tail[13] if len(tail) > 13 else "?") + "/"
                    + (tail[14] if len(tail) > 14 else "?") + "/" + (tail[15] if len(tail) > 15 else "?"))
            self.log.append(show)
            self.push_plot(s)
        elif len(s) < 200:
            self.log.append(s)

    def apply_param_echo(self, line):
        """仅在参数旁显示车端当前值；绝不覆盖待下发输入框、滑块或param_val。"""
        body = line[len("PAR:"):]
        found = 0
        import re
        for m in re.finditer(r"([A-Z])=([-+]?[0-9]*\.?[0-9]+)", body):
            cn, vstr = m.group(1), m.group(2)
            if cn not in self.ctrl:
                continue
            try:
                v = float(vstr)
            except ValueError:
                continue
            self.car_param_val[cn] = v
            info = self.ctrl[cn]
            info["car"].setText(f"车端: {v:.{info['dec']}f}")
            found += 1
        if found:
            self.log.append(f"已显示车端当前参数 {found} 项；待下发数值保持不变")
            self.status.setText(f"已连接 · 车端回显已更新 {found} 项 · 输入值未改动")

    # ---------- 操作 ----------
    def do_connect(self):
        self.ble._wanted = True
        self.run_coro(self.ble.connect(), "连接")

    def do_ask(self):
        self.run_coro(self.ble.send("?"), "回显")

    def _preview_val(self, cn, v):
        return v / self.ctrl[cn]["scale"]

    def preview(self, cn, v):
        info = self.ctrl[cn]
        info["box"].setText(f"{self._preview_val(cn, v):.{info['dec']}f}")

    def commit_box(self, cn):
        info = self.ctrl[cn]
        try:
            v = float(info["box"].text())
        except ValueError:
            info["cmt"].setText("无效"); return
        v = max(info["lo"], min(info["hi"], v))
        info["sl"].blockSignals(True)
        info["sl"].setValue(int(round(v * info["scale"])))
        info["sl"].blockSignals(False)
        info["box"].setText(f"{v:.{info['dec']}f}")
        self.send_code(cn)

    def commit_slider(self, cn):
        info = self.ctrl[cn]
        v = self._preview_val(cn, info["sl"].value())
        info["box"].setText(f"{v:.{info['dec']}f}")
        self.send_code(cn)

    def send_code(self, cn):
        info = self.ctrl[cn]
        v = float(info["box"].text())
        self.param_val[cn] = v
        cmd = f"{cn} {v:.{info['dec']}f}"
        info["cmt"].setText(f"发 {cmd}")
        self.run_coro(self.ble.send(cmd), cmd)

    def send_all(self):
        commands = []
        for cn, *_ in PARAMS:
            info = self.ctrl[cn]
            try:
                value = float(info["box"].text())
            except ValueError:
                info["cmt"].setText("无效，全部下发已取消")
                self.status.setText(f"{cn} 不是有效数字，未下发")
                return
            value = max(info["lo"], min(info["hi"], value))
            info["sl"].blockSignals(True)
            info["sl"].setValue(int(round(value * info["scale"])))
            info["sl"].blockSignals(False)
            value = self._preview_val(cn, info["sl"].value())
            info["box"].setText(f"{value:.{info['dec']}f}")
            self.param_val[cn] = value
            command = f"{cn} {value:.{info['dec']}f}"
            info["cmt"].setText(f"待发 {command}")
            commands.append((cn, command))

        async def send_sequence():
            # 单协程顺序写入，避免旧实现多线程并发造成BLE命令乱序或丢失。
            for cn, command in commands:
                ok = await self.ble.send(command)
                if not ok:
                    raise RuntimeError(f"{cn} 下发失败")
                await asyncio.sleep(0.10)
            await self.ble.send("?")
            self.bridge.msg.emit("全部参数已顺序下发并请求回显 ✓")

        self.status.setText(f"正在顺序下发 {len(commands)} 项…")
        self.run_coro(send_sequence(), "顺序下发全部")

    def capture_position(self):
        """让固件把当前编码器位置设为新的回位目标。"""
        self.run_coro(self.ble.send("C 1"), "设置当前位置为回位点")
        self.status.setText("已请求把当前位置设为回位点")

    def apply_pasted_params(self):
        """解析批量粘贴文本并更新待下发输入框；不自动向车辆发送。"""
        text = self.batch_input.toPlainText().strip()
        if not text:
            self.status.setText("批量参数为空")
            return

        # 支持 P=850、P:850、P 850；分隔符可为换行、空格、逗号或分号。
        pattern = re.compile(
            r"(?i)(?<![A-Za-z0-9_])([A-Z])\s*(?:=|:|\s)\s*"
            r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
        )
        matches = pattern.findall(text)
        valid_cmds = set(self.ctrl)
        applied = []
        ignored = []

        for raw_cmd, raw_value in matches:
            cmd = raw_cmd.upper()
            if cmd not in valid_cmds:
                ignored.append(cmd)
                continue
            info = self.ctrl[cmd]
            try:
                value = float(raw_value)
            except ValueError:
                ignored.append(cmd)
                continue
            value = max(info["lo"], min(info["hi"], value))
            info["sl"].blockSignals(True)
            info["sl"].setValue(int(round(value * info["scale"])))
            info["sl"].blockSignals(False)
            # 以滑块实际可表达的精度回写，避免显示值与随后发送值不一致。
            value = self._preview_val(cmd, info["sl"].value())
            info["box"].setText(f"{value:.{info['dec']}f}")
            info["cmt"].setText("已从粘贴文本载入，未发送")
            self.param_val[cmd] = value
            applied.append(cmd)

        if not matches:
            self.status.setText("未识别参数；请使用 P=850 或 P 850 格式")
            self.log.append("批量解析失败：未找到“字母=数字”或“字母 数字”")
            return

        if applied:
            summary = ", ".join(applied)
            self.status.setText(f"已载入 {len(applied)} 项，尚未发送 · 请确认后点“下发全部”")
            self.log.append(f"批量载入（未发送）：{summary}")
        if ignored:
            self.log.append("忽略未知参数：" + ", ".join(ignored))

    # ---------- asyncio 桥 ----------
    def run_coro(self, coro, label):
        def runner():
            try:
                fut = asyncio.run_coroutine_threadsafe(coro, self.ble.loop)
                fut.result(timeout=20)
            except Exception as e:
                self.bridge.msg.emit(f"{label}:{str(e)[:40]}")
        threading.Thread(target=runner, daemon=True).start()

    def closeEvent(self, _e):
        self.close_csv()
        super().closeEvent(_e)


def main():
    app = QApplication(sys.argv)
    w = TunePanel()
    w.show()
    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
