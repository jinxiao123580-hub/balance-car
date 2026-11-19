"""G 帧解析与指令拼装（纯标准库，不依赖 ROS）。

G 帧线上格式（firmware main.cpp:4355 sendFastTelemetry）:
    G,<ms>,<pos*100>,<speed*1000>,<pitch*100>,<target*100>,<output_hz>,<requested*1000>,<btRxBytes>

去掉前缀后的字段下标（踩过坑，写死在这里）:
    0=ms  1=pos*100  2=speed*1000  3=pitch*100  4=target*100  5=hz  6=requested*1000  7=btRx

标度（2026-11-12 悬空实测反推，hz 全程打满 1750）:
    1 step = 0.0625 mm
    1 Hz   = 0.0625 mm/s
    1750 Hz = 109 mm/s = 极速
    pos_mm = pos_raw / 100 / 10.8
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Optional

POS_PER_MM_RAW = 100.0 * 10.8      # pos 原始计数 → mm
MM_PER_STEP = 0.0625               # 1 步 = 0.0625mm
TOP_MM_S = 109.0                   # 1750Hz 对应的极速

# 允许下发的命令前缀白名单（防止误发导致车辆异常）
_ALLOWED_CMDS = ("O ", "^ ", "TF ", "TB ", "EG ", "Z ", "L ", "D ", "P ", "X ", "?", "SAVE")


@dataclass
class CarState:
    """一帧 G 帧解析结果。量的单位已换算成物理量。"""
    stamp_ms: int = 0            # 固件 millis()
    pos_mm: float = 0.0          # 编码器累计位置（负=前进，正=后退）
    speed_mm_s: float = 0.0      # 瞬时速度 mm/s
    pitch_deg: float = 0.0       # 机身俯仰角
    target_deg: float = 0.0      # 平衡环目标角（= O + 外环偏移）
    hz: float = 0.0              # 步进频率指令（±1750 限幅）
    requested: float = 0.0       # manualForward.requested
    bt_rx: int = 0               # 累计接收字节

    def to_dict(self) -> dict:
        return asdict(self)

    @property
    def speed_pred_mm_s(self) -> float:
        """由 hz 反推的期望速度（用于判断轮子是否跟得上指令）。"""
        return self.hz * MM_PER_STEP


def parse_gframe(line: str) -> Optional[CarState]:
    """解析一行 G 帧；不是 G 帧或字段不足时返回 None。"""
    line = line.strip()
    if not line.startswith("G,"):
        return None
    f = line[2:].split(",")
    if len(f) < 6:
        return None
    try:
        vals = [float(x) for x in f[:8]]
    except ValueError:
        return None
    while len(vals) < 8:
        vals.append(0.0)
    return CarState(
        stamp_ms=int(vals[0]),
        pos_mm=vals[1] / POS_PER_MM_RAW,
        speed_mm_s=vals[2] / 1000.0,
        pitch_deg=vals[3] / 100.0,
        target_deg=vals[4] / 100.0,
        hz=vals[5],
        requested=vals[6] / 1000.0,
        bt_rx=int(vals[7]),
    )


def build_lean_command(deg: float) -> str:
    """手动倾角指令：^ <turnHz>,<pitchDeg>。turnHz 恒 0（不做转向）。"""
    return f"^ 0,{deg:.2f}"


def build_o_command(deg: float) -> str:
    """平衡点指令：O <deg>。改 O 前应先发 ^ 0,0 归零（手动态门控）。"""
    return f"O {deg:.2f}"


def build_stop_command() -> str:
    return "^ 0,0"


def sanitize_command(text: str) -> Optional[str]:
    """校验一条待下发命令；不允许的前缀返回 None。

    安全理由：O/^ 会直接让车动，必须限制在白名单内，且不允许换行注入。
    """
    if not text:
        return None
    if "\n" in text or "\r" in text:
        return None
    t = text.strip()
    if not t:
        return None
    upper = t.upper()
    for pfx in _ALLOWED_CMDS:
        if upper.startswith(pfx.upper()):
            return t
    return None
