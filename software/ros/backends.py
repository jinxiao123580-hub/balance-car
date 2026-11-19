"""串口 / BLE 两种后端的接口占位。

约束（2026-11-13 核查结论）：
- 下位机 **没有装 pyserial**，也不允许装包 ⇒ UART 后端必须用标准库 termios 实现。
- 下位机**没有任何无线硬件**（lspci 无 WiFi/BT），当前 JDY-23 链路只存在于 Windows 侧
  ⇒ BLE 后端在下位机上不可用，仅保留接口占位，等以后有 BLE 适配器再实现。

接线要点见 docs/validation/analysis/20261113-ROS-PLAN.md。
"""

from __future__ import annotations

import os
import select
import sys
import time
from typing import Optional

try:  # 只在 Linux 下有意义
    import termios
    import tty
except ImportError:  # Windows 本机做语法检查时会走到这里
    termios = None
    tty = None


class BackendError(RuntimeError):
    pass


class Backend:
    """所有后端的最小接口。"""

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def read_lines(self, timeout: float = 0.0) -> list[str]:
        """读若干整行（不带换行符）。timeout 秒内没有数据就返回空列表。"""
        raise NotImplementedError

    def write_line(self, text: str) -> None:
        raise NotImplementedError

    @property
    def name(self) -> str:
        return type(self).__name__


class UartBackend(Backend):
    """标准库 termios 串口后端（9600 8N1，无流控，raw 模式）。

    用法:
        b = UartBackend("/dev/ttyUSB0")
        b.open()
        for line in b.read_lines(0.2):
            print(line)
        b.write_line("?")
    """

    def __init__(self, device: str, baud: int = 9600, read_chunk: int = 512):
        if termios is None:
            raise BackendError("termios 不可用：本后端只能在 Linux 下运行")
        self.device = device
        self.baud = baud
        self.read_chunk = read_chunk
        self._fd: Optional[int] = None
        self._buf = ""

    def open(self) -> None:
        fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            # 关闭回显/行缓冲，raw 方式；fix 到 8N1
            attrs[0] = 0                                     # iflag
            attrs[1] = 0                                     # oflag
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
            attrs[3] = 0                                     # lflag
            attrs[4] = termios.B9600 if self.baud == 9600 else termios.B115200
            attrs[5] = termios.B9600 if self.baud == 9600 else termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd
        self._buf = ""

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            finally:
                self._fd = None

    def read_lines(self, timeout: float = 0.0) -> list[str]:
        if self._fd is None:
            raise BackendError("串口未打开")
        deadline = time.time() + max(0.0, timeout)
        out: list[str] = []
        while True:
            r, _, _ = select.select([self._fd], [], [], max(0.0, deadline - time.time()))
            if not r:
                break
            try:
                chunk = os.read(self._fd, self.read_chunk)
            except BlockingIOError:
                break
            except OSError as e:
                raise BackendError(f"读串口失败: {e}") from e
            if not chunk:
                break
            self._buf += chunk.decode("utf-8", "replace")
            *done, self._buf = self._buf.replace("\r", "\n").split("\n")
            out.extend(x.strip() for x in done if x.strip())
            if time.time() >= deadline:
                break
        return out

    def write_line(self, text: str) -> None:
        if self._fd is None:
            raise BackendError("串口未打开")
        os.write(self._fd, (text + "\n").encode())


class PyserialBackend(Backend):
    """可选：如果你在**别的机器**上跑（有 pyserial），用这个后端。

    在下位机上不可用（pyserial 未安装且不允许安装），保留接口占位。
    """

    def __init__(self, device: str, baud: int = 9600):
        try:
            import serial  # noqa: F401
        except ImportError as e:
            raise BackendError(
                "pyserial 未安装（下位机不允许安装）。请改用 UartBackend（标准库 termios）。"
            ) from e
        self.device = device
        self.baud = baud
        self._ser = None

    def open(self) -> None:
        import serial
        self._ser = serial.Serial(self.device, self.baud, timeout=0.1)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def read_lines(self, timeout: float = 0.0) -> list[str]:
        if self._ser is None:
            raise BackendError("串口未打开")
        self._ser.timeout = max(0.05, timeout)
        data = self._ser.readline().decode("utf-8", "replace").strip()
        return [data] if data else []

    def write_line(self, text: str) -> None:
        if self._ser is None:
            raise BackendError("串口未打开")
        self._ser.write((text + "\n").encode())


class BleBackend(Backend):
    """BLE 后端接口占位（JDY-23 / FFE1）。

    **当前在下位机上不可实现**：
      - 下位机 lspci 未发现任何无线/蓝牙硬件；
      - bleak 未安装且不允许安装。
    保留接口是为了将来把这条链路接回 Windows 侧或加蓝牙适配器时不用改节点代码。
    """

    FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"

    def __init__(self, address: str = ""):
        self.address = address

    def open(self) -> None:
        raise BackendError(
            "BLE 后端不可用：下位机无蓝牙硬件且未安装 bleak。"
            "当前请使用 UartBackend（杜邦线直连 UART3）。"
        )

    def close(self) -> None:
        return None

    def read_lines(self, timeout: float = 0.0) -> list[str]:
        raise BackendError("BLE 后端不可用")

    def write_line(self, text: str) -> None:
        raise BackendError("BLE 后端不可用")


def make_backend(kind: str, device: str = "", baud: int = 9600) -> Backend:
    """工厂：kind = uart | pyserial | ble。"""
    k = (kind or "uart").lower()
    if k == "uart":
        return UartBackend(device, baud)
    if k == "pyserial":
        return PyserialBackend(device, baud)
    if k == "ble":
        return BleBackend(device)
    raise BackendError(f"未知后端: {kind}")


if __name__ == "__main__":
    # 自检：列出可用后端，不做任何 I/O
    print("platform:", sys.platform, "termios:", termios is not None)
    for k in ("uart", "pyserial", "ble"):
        try:
            b = make_backend(k, "/dev/null")
            print(f"  {k}: {b.name} 可构造")
        except BackendError as e:
            print(f"  {k}: 不可用 -> {e}")
