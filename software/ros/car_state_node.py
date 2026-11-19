#!/usr/bin/env python3
"""状态发布节点：串口 G 帧 → /car/state（JSON 字符串）。

为什么用 std_msgs/String 而不是自定义 msg：
  下位机只装了 ros-humble-* 二进制包，没有 colcon 工作区也不需要编译；
  用 String 装 JSON 可以 `python3 car_state_node.py` 直接跑，零构建、零安装。

话题：/car/state  std_msgs/msg/String
    {"stamp_ms":..,"pos_mm":..,"speed_mm_s":..,"pitch_deg":..,"target_deg":..,"hz":..,
     "requested":..,"bt_rx":..,"speed_pred_mm_s":..}

可选：--cmd-topic /car/cmd 时，同一进程内**同时**订阅指令（一个串口不能被两个进程同时打开，
      所以实机上推荐这种单进程模式）。

用法：
  source /opt/ros/humble/setup.bash
  python3 car_state_node.py --device /dev/ttyUSB0 --baud 9600
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


class CarStateNode(Node):
    def __init__(self, args):
        super().__init__("car_state")
        self.args = args
        self.pub = self.create_publisher(String, args.topic, 10)
        self.cmd_pub_ack = None
        if args.cmd_topic:
            self.create_subscription(String, args.cmd_topic, self.on_cmd, 10)
        self.be = backends.make_backend(args.backend, args.device, args.baud)
        self.be.open()
        self.get_logger().info(f"backend={self.be.name} device={args.device} -> {args.topic}")
        period = 1.0 / max(1.0, float(args.rate))
        self.timer = self.create_timer(period, self.tick)
        self.n_frame = 0

    def tick(self):
        try:
            lines = self.be.read_lines(0.05)
        except backends.BackendError as e:
            self.get_logger().error(str(e))
            return
        for line in lines:
            st = gframe.parse_gframe(line)
            if st is None:
                continue
            d = st.to_dict()
            d["speed_pred_mm_s"] = st.speed_pred_mm_s
            self.pub.publish(String(data=json.dumps(d)))
            self.n_frame += 1
            if self.n_frame % 40 == 1:
                self.get_logger().info(
                    f"pos={st.pos_mm:+.1f}mm pitch={st.pitch_deg:.2f} "
                    f"tgt={st.target_deg:.2f} hz={st.hz:.0f} spd={st.speed_mm_s:+.1f}mm/s")

    def on_cmd(self, msg: String):
        cmd = gframe.sanitize_command(msg.data)
        if cmd is None:
            self.get_logger().warn(f"拒绝非白名单指令: {msg.data!r}")
            return
        try:
            self.be.write_line(cmd)
            self.get_logger().info(f"下发 {cmd}")
        except backends.BackendError as e:
            self.get_logger().error(str(e))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="uart", choices=["uart", "pyserial", "ble"])
    ap.add_argument("--device", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--topic", default="/car/state")
    ap.add_argument("--cmd-topic", default="", help="非空时同进程订阅该指令话题")
    ap.add_argument("--rate", type=float, default=20.0, help="读串口的轮询频率 Hz")
    a, _ = ap.parse_known_args(argv)

    rclpy.init()
    node = None
    try:
        node = CarStateNode(a)
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
