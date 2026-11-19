# software/ros —— 平衡车 ROS 2 桥接骨架

部署步骤与核查依据见 `docs/validation/analysis/20261113-ROS-PLAN.md`。

## 文件

| 文件 | 作用 |
|---|---|
| `gframe.py` | G 帧解析、指令拼装、白名单校验（纯标准库，不依赖 ROS） |
| `backends.py` | 串口/BLE 后端：`UartBackend` 用标准库 `termios`；`PyserialBackend`、`BleBackend` 为占位 |
| `car_state_node.py` | 发布 `/car/state`（`std_msgs/String`，JSON 负载）；`--cmd-topic` 可同进程兼做指令 |
| `car_cmd_node.py` | 订阅 `/car/cmd`，校验后下发 |

## 为什么不用自定义 msg

下位机只有 `ros-humble-*` 二进制包，且不允许装包。用 `std_msgs/String` 装载 JSON，
`python3 xxx.py` 即可运行，**不需要 colcon build**。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42          # 校园网 /15，避开默认 0
python3 car_state_node.py --device /dev/ttyUSB0 --cmd-topic /car/cmd
```

```bash
ros2 topic hz /car/state         # 预期 4Hz 左右
ros2 topic echo /car/state --once
ros2 topic pub --once /car/cmd std_msgs/msg/String "{data: '^ 0,0'}"   # 急停
```

## `/car/state` 负载

```json
{"stamp_ms": 795337, "pos_mm": -979.31, "speed_mm_s": -5.33,
 "pitch_deg": 2.25, "target_deg": 2.40, "hz": -142.0,
 "requested": 0.0, "bt_rx": 950, "speed_pred_mm_s": -8.88}
```

换算（2026-11-12 悬空实测标定）：`pos/100/10.8 = mm`，`speed/1000 = mm/s`，
`pitch/100 = deg`，`1 Hz = 0.0625 mm/s`，`1750 Hz = 109 mm/s`（极速）。

## 已知约束

- **一个串口不能被两个进程同时打开**：实机请用 `car_state_node.py --cmd-topic /car/cmd` 单进程双职责。
- 跨机组播不通（实测超时），必须配 Fast DDS 静态对端，见方案文档 §6.3。
- 运动指令（`O` / `^`）会让车立刻动，联调务必先悬空；`^ 0,0` 是急停。
