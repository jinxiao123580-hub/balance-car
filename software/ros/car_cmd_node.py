#!/usr/bin/env python3
"""指令订阅节点：/car/cmd → 串口下发 `^` / `O` 等命令。

话题：/car/cmd  std_msgs/msg/String
    两种写法都接受：
      1) 原始命令行    "O 4.50"   "^ 0,0"   "TF 0.2"   "?"
      2) JSON          {"cmd":"O","value":4.5}  /  {"raw":"^ 0,0"}

安全：只放行白名单前缀（O / ^ / TF / TB / EG / Z / L / D / P / X / ? / SAVE），
      拒绝含换行的注入。涉及运动前请先在实车侧确认。

注意：一个串口不能被两个进程同时打开。若状态节点已经在跑，
      应改用 `car_state_node.py --cmd-topic /car/cmd`（单进程双职责），不要同时开本节点。

用法：
  source /opt/ros/humble/setup.bash
  python3 car_cmd_node.py --device /dev/ttyUSB0
退出码：0 正常，2 打开串口失败。
"""

from __future__ import annotations

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import backends
import gframe


class CarCmdNode(Node):
    def __init__(self, args):
        super().__init__("car_cmd")
        self.args = args
        self.be = backends.make_backend(args.backend, args.device, args.baud)
        self.be.open()
        self.create_subscription(String, args.topic, self.on_cmd, 10)
        self.get_logger().info(
            f"backend={self.be.name} device={args.device} 订阅 {args.topic}")

    def _decode(self, data: str):
        text = (data or "").strip()
        if not text:
            return None
        if text.startswith("{"):
            try:
                obj = json.loads(text)
            except json.JSONDecodeError:
                return None
            if "raw" in obj:
                return str(obj["raw"])
            cmd = str(obj.get("cmd", "")).strip()
            if not cmd:
                return None
            value = obj.get("value")
            if value is None:
                return cmd
            if cmd.upper().startswith("^"):
                return gframe.build_lean_command(float(value))
            if cmd.upper().startswith("O"):
                return gframe.build_o_command(float(value))
            return f"{cmd} {float(value):.3f}"
        return text

    def on_cmd(self, msg: String):
        raw = self._decode(msg.data)
        if raw is None:
            self.get_logger().warn(f"无法解析指令: {msg.data!r}")
            return
        cmd = gframe.sanitize_command(raw)
        if cmd is None:
            self.get_logger().warn(f"拒绝非白名单指令: {raw!r}")
            return
        try:
            self.be.write_line(cmd)
        except backends.BackendError as e:
            self.get_logger().error(str(e))
            return
        self.get_logger().info(f"下发 {cmd}")
        # G 帧 4Hz，等一小会儿把回显/遥测读掉
        try:
            for line in self.be.read_lines(0.15):
                self.get_logger().info(f"回显 {line}")
        except backends.BackendError:
            pass


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="uart", choices=["uart", "pyserial", "ble"])
    ap.add_argument("--device", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--topic", default="/car/cmd")
    a, _ = ap.parse_known_args(argv)

    rclpy.init()
    node = None
    try:
        node = CarCmdNode(a)
        rclpy.spin(node)
    except backends.BackendError as e:
        print(f"[错误] {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.be.close()
            except Exception:
                pass
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
