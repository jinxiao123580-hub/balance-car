# 平衡车 UART → NVIDIA Jetson → **Windows 本机热点** → 本机 ROS 2：可移植部署方案

日期：2026-11-13　编写：子代理（只读，未连任何机器、未安装任何东西）
替代对象：`20261113-ROS-PLAN.md`（那份针对 x86 slave，**不是** Jetson）
固件：`FW=20261112F`　平衡车：STM32F407，UART 在 **PB10(TX) / PB11(RX)**，3.3V TTL，**9600 8N1**

> **型号未定**（尚未到手）。因此 §1 给的是"拿到机器后按 ①②③ 对号入座"的判定清单，
> 不押注任何一款；凡不能由官方文档固化的，一律标 `[待确认]` 并附现场验证命令。

---

## 0. 三条必须先对齐的前提

1. **Jetson 40-pin 是 3.3V TTL**，与 STM32 的 PB10/PB11 同电平 ⇒ 可直连，**无需电平转换**；但 Jetson 引脚不耐 5V。
2. **JDY-23 现在占着 PB10/PB11**。切到 Jetson 前必须整块拔下，否则两个 TX 同时驱动 PB11 会总线冲突（§4）。
3. **跨机 DDS 组播不通**（此前在同网段实测：组播超时、单播 UDP 通）。**热点链路上同样按"组播不可用"设计**，
   ROS 2 默认发现机制跨机不可依赖，必须配静态对端（§6）。

---

## 1. 到手第一步：对号入座（型号未定，先判定再选路）

### 1.1 三条判定命令

```bash
cat /etc/nv_tegra_release          # 例：R36 (release), REVISION: 3.0   → JetPack 6.0
grep PRETTY_NAME /etc/os-release   # 例：Ubuntu 22.04.4 LTS
uname -r                           # 例：5.15.136-tegra
ls /sys/class/net                  # 有 wlan0 / wlP1p1s0 → 自带 WiFi
ls -l /dev/ttyTHS*                 # 40-pin UART 的设备名
```

### 1.2 版本对应关系（官方）

| JetPack | Jetson Linux (L4T) | 内核 | 根文件系统 | 出处 |
|---|---|---|---|---|
| JetPack 5.1.3 | 35.5.0 | 5.10 | **Ubuntu 20.04** | [NVIDIA JetPack 5.1.3 Release Notes](https://docs.nvidia.com/jetson/jetpack/5.1.3/release-notes/index.html) |
| JetPack 6.0 | 36.3 | 5.15 | **Ubuntu 22.04** | [NVIDIA JetPack 6.0 Release Notes](https://docs.nvidia.com/jetson/jetpack/6.0/release-notes/index.html) |

JetPack 5.1.3 官方支持：AGX Orin / Orin NX / Orin Nano / AGX Xavier / Xavier NX 的模组与开发套件（同上出处）。
JetPack 6 官方支持：**所有 Orin 系列**模组与开发套件（JetPack 6.0 Release Notes）。

| Ubuntu | 对应 ROS 2 | 依据 |
|---|---|---|
| 22.04 (Jammy) | **Humble** | [ROS 2 Humble 安装页](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) |
| 20.04 (Focal) | **Foxy** | [ROS 2 Foxy 安装页](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debs.html) |

### 1.3 对号入座决策表

| ① `nv_tegra_release` | ② Ubuntu | ③ 走哪个 ROS 分支 | ④ 注意 |
|---|---|---|---|
| 含 **R36** | 22.04 | **Humble**（`/opt/ros/humble`） | 首选 |
| 含 **R35** | 20.04 | **Foxy**（`/opt/ros/foxy`） | 骨架需在 Foxy 上重跑 `py_compile` + 逻辑自测 |
| 其他/读不到 | 看 `os-release` | 按上面 ② 判定 | 记录 `uname -r`，回查 NVIDIA 版本表 |

再按两条硬件判定分叉：

| 判定项 | 结果 | 走哪条 |
|---|---|---|
| `ls /sys/class/net` 是否有 `wlan*` | 有 | 主方案：**Windows 热点**（§5.1） |
|  | 无 | 备用：有线直连（§5.2，已压缩） |
| `ls -l /dev/ttyTHS*` | 出现 `ttyTHS1` | 40-pin UART 可直接用（§2） |
|  | 什么都没有 | 回 §2.3 用 Jetson‑IO 启用 uarta |

`[结论]` 若型号可挑：**Orin Nano/NX + JetPack 6.x + Ubuntu 22.04 + Humble** 最省事（与本项目在 x86 上已验证的 Humble 一致，`software/ros/` 骨架可直接复用）。
**但本次不等型号**——清单是判定用的，不是选型建议。

---

## 2. Jetson 40-pin 排针 UART（用 `ttyTHS*`，不用 USB-TTL）

### 2.1 引脚

| 功能 | 40-pin 引脚号 | 接到 |
|---|---|---|
| UART1_TX | **pin 8** | STM32 **PB11 (RX)** |
| UART1_RX | **pin 10** | STM32 **PB10 (TX)** |
| GND | pin 6/9/14/20/25/30/34/39 任选 | STM32 GND（**必须共地**） |
| 3.3V | pin 1/17 | **留空不接**（车板自供电，接上会倒灌） |

出处：排针引脚功能在烧写时静态定义、改配置要用 pinmux 或 **Jetson‑IO** 改 DTB ——
[NVIDIA《Configuring the Jetson Expansion Headers》](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/HR/ConfiguringTheJetsonExpansionHeaders.html)；
另见 [Hardware Setup / Jetson‑IO](https://docs.nvidia.com/jetson/l4t/Tegra%20Linux%20Driver%20Package%20Development%20Guide/hw_setup_jetson_io.html)。

### 2.2 设备名

- Orin 系列：40-pin 的 UART1 对应 **`/dev/ttyTHS1`**。
  依据：NVIDIA 开发者论坛 pins 8/10 → `ttyTHS1` <https://forums.developer.nvidia.com/t/how-to-access-uart1-on-expansion-pins-8-10/328934>；
  同帖提醒**部分引脚图画成 180° 反向**，pin 1 在左上角。
- Xavier NX / Nano：`ttyTHS*` 编号与 console 占用与 Orin 不同，`[待确认]`，用下面命令现场判定。

```bash
ls -l /dev/ttyTHS* /dev/ttyTCU* 2>/dev/null
# 预期（Orin）：crw-rw---- 1 root dialout 240, 1 ... /dev/ttyTHS1
dmesg | grep -iE "ttyTHS|ttyTCU|serial-tegra"
# 预期：3100000.serial: ttyTHS1 at MMIO 0x3100000 ... is a TEGRA_UART
```

### 2.3 启用 40-pin UART

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
# Configure 40-pin Header → pin 8/10 切到 uarta（UART1）
# Save and reboot to reconfigure pins
```

`[注意]` 官方页明确：headless 方式（`config-by-function.py -o dt uarta`）**只改 DTB、不接进启动链**；
只有交互式工具会写 `OVERLAYS` 行。用 headless 方式必须自己确认 `extlinux.conf` 里有对应 overlay。

### 2.4 **经典坑：串口控制台占用**

调试控制台与 40-pin UART 可能落在**同一 UART 控制器**上。控制台还挂着的话，内核日志会从 pin 8 打出去，STM32 收到乱码。

**判定**：

```bash
cat /proc/cmdline | tr ' ' '\n' | grep console
# Orin 典型：console=ttyTCU0,115200     ← 就是要摘掉的那一项
grep -n "console=" /boot/extlinux/extlinux.conf
```

**解法（官方路径，两处都改）**：

1. **已部署的设备**：从 `/boot/extlinux/extlinux.conf` 的 `APPEND` 行删掉 `console=ttyTCU0,115200`，重启。
   依据：NVIDIA 论坛官方回复 <https://forums.developer.nvidia.com/t/how-to-disable-debug-console-on-uart-port-orin-nx/292045>。
2. **烧写前的镜像**：Orin NX / Orin Nano 改 `p3767.conf.common`；AGX Orin 改 `p3701.conf.common`，删同一个 `console=` 项。
   依据：[NVIDIA《Kernel Boot Time Optimization》— Disable Console Printing over UART](https://docs.nvidia.com/jetson/archives/r38.2.1/DeveloperGuide/SD/Kernel/KernelBootTimeOptimization.html)。
3. 若还有 `nvgetty` 挂在调试串口：`sudo systemctl stop nvgetty && sudo systemctl disable nvgetty`。

`[注意]` 摘掉 console 后**串口就看不到启动日志**了；改之前先确认 SSH 可用，否则改坏了没法远程救。

（Xavier NX / Nano 上 console 可能占 `ttyTHS0`，`[待确认]`，用 `cat /proc/cmdline` 判定。）

### 2.5 已知驱动风险（社区报告，非官方文档）

- **JetPack 6.2.2 起 UART DMA 异常**，表现为"端口在、收发不通"；社区解法：从设备树 `serial@3100000` 移除 `dmas` / `dma-names`，强制 PIO。
  出处：<https://forums.developer.nvidia.com/t/ttyths1-uart1-40-pin-header-pins-8-10-tx-pad-does-not-drive-on-jetpack-7-l4t-r39-2-orin-nano-super-devkit/373290>
- 同帖另报 **JetPack 7 / L4T R39.2 + Orin Nano Super** 上 pin 8 TX 不驱动（示波器 ~0.3V、环回无数据）。
  ⇒ 遇到"引脚定义都对、就是没波形"，先怀疑这条，降 JetPack 版本验证硬件。

---

## 3. 权限

```bash
id | tr ',' '\n' | grep -E "dialout|tty"
getent group dialout
ls -l /dev/ttyTHS1
```

- 预期：`crw-rw---- 1 root dialout 240, 1 ... /dev/ttyTHS1` ⇒ 在 `dialout` 组即可读写，**无需 sudo**。
- 若不在组内：`sudo usermod -aG dialout $USER`，**必须重新登录**（组变更不进已开会话）。
- 不要用 `chmod 777` 绕过：重启失效，且掩盖真实问题。

---

## 4. 与 JDY-23 的互斥（切换代价）

### 4.1 摘下前

1. **断电操作**，不要热插拔 PB10/PB11。
2. JDY-23 的 TX 也挂在 **PB11**；不摘掉就会和 Jetson 的 TX 抢同一根线 ⇒ 乱码或完全无数据。
3. **建议整块拔下**，不要只断信号线（避免它上电后意外驱动）。
4. 先记下原接线位置，便于回退。

### 4.2 切换后 BLE 通道失效

`software/analysis/ble_*.py` 全部依赖 JDY-23（且必须 `response=True`）。摘下后这些脚本**全部失效**；
在 Jetson 串口链路跑通前，车上**没有**任何命令通道。⇒ 切换前把 BLE 侧该做的验证做完。

### 4.3 常用命令的串口等价版（同一套文本行协议，`\n` 结尾，9600 8N1）

| 用途 | BLE 版 | 串口版 |
|---|---|---|
| 读参数 | `?` | `printf '?\n' > /dev/ttyTHS1`，再 `timeout 3 cat /dev/ttyTHS1` |
| 急停 | `^ 0,0` | `printf '^ 0,0\n' > /dev/ttyTHS1` |
| 设平衡点 | `O 4.00` | 先 `printf '^ 0,0\n'`，等 ≥1s，再 `printf 'O 4.00\n'` |
| 前进/后退 | `^ 0,-1.2` / `^ 0,1.2` | 同格式 |
| 看遥测 | BLE 通知 | `cat /dev/ttyTHS1`（G 帧每 250ms 一行） |

`[注意]` `cat` 会一直占着串口，正式跑用 `software/ros/` 的节点。
`O` 命令受手动态门控，改 O 前必须先 `^ 0,0` 并等 ≥1s（BLE 上已验证，串口同样适用）。

---

## 5. 网络：**Windows 本机热点为主方案**

### 5.1 热点可行性（本机已实测，结论：可用）

| 项 | 实测值 | 含义 |
|---|---|---|
| OS | Windows 11 专业版 **Build 26200** | — |
| WLAN 网卡 | **Intel Wi-Fi 6 AX200 160MHz**，Status=Up，300 Mbps | 支持 Wi-Fi Direct |
| `netsh wlan show wirelesscapabilities` | **Wi-Fi Direct Device / GO / Client 全部 Supported** | **网卡能当 AP ⇒ 移动热点可用** |
| `icssvc`（ICS） | Status=**Stopped**，StartType=**Manual** | 开热点时按需启动，无需预先启动 |
| `Microsoft Wi-Fi Direct Virtual Adapter` ×2 | 均 Disconnected | 未开热点时的正常状态 |
| 当前网段 | 无 `192.168.137.x` | 热点尚未启用 |

> **⚠️ 陷阱（务必写进排查手册）**
> `netsh wlan show drivers` 里的 `Hosted network supported : No` **不代表不能开热点**。
> 那是 Win7 时代的 `hostednetwork` 接口，**Windows 11 已废弃**，该字段常年为 No。
> **判定网卡能否当 AP，只看 `netsh wlan show wirelesscapabilities` 里的 Wi-Fi Direct GO 是否 Supported。**

### 5.2 Windows 侧：开启与验证

**开启**：设置 → 网络和 Internet → 移动热点 → 打开；记下 SSID 与密码（可点"编辑"改密码）。

```powershell
# 命令行打开设置页
Start-Process "ms-settings:network-mobilehotspot"
# 确认热点已起来、拿到了地址
Get-NetIPAddress -AddressFamily IPv4 | ? {$_.IPAddress -like '192.168.137.*'}
Get-NetAdapter | ? {$_.Name -like '*Wi-Fi Direct*' -or $_.InterfaceDescription -like '*Wi-Fi Direct*'} |
  Select-Object Name,Status,InterfaceDescription
```

- ICS 默认把热点适配器设为 **192.168.137.1/24**，并给客户端 DHCP 分配 `192.168.137.2` 起的地址。
  **以 `Get-NetIPAddress` 的实际输出为准**，不要写死。
- 手机/笔电能连上但 ping 不通时，先查**客户端隔离**与 Windows 防火墙。

**常见失败原因与处置**：

| 现象 | 先查这条 | 处置 |
|---|---|---|
| 移动热点开关是灰的 | `wirelesscapabilities` 里 Wi-Fi Direct **GO** 是否 Supported | 不是 Supported 才真不行；**别用 `Hosted network supported` 判定** |
| 能开，但客户端连不上 | 5GHz/2.4GHz 频段 | 设置页里把热点频段切到 **2.4GHz**（不少嵌入式网卡只支持 2.4G） |
| 客户端连上但无 IP | DHCP/ICS 未生效 | `Get-Service icssvc`；必要时手动 `Start-Service icssvc` 再关开热点 |
| 有 IP，ping 不通 | 客户端隔离 / 防火墙 | 关客户端隔离；放行 ICMP 与 UDP 入站（ROS 也要用） |
| 一开热点 WLAN 掉线 | 网卡被独占 / 驱动为旧版 | 更新 Intel 驱动；或改用 USB 无线网卡专门做 AP |
| 开热点后内网走不通 | VPN 抢占默认路由 | 临时断开 VPN 再开热点 |

### 5.3 Jetson 侧：连接与验证

```bash
# 1) 扫到热点
nmcli device wifi rescan
nmcli device wifi list | grep -i "<你的SSID>"
# 2) 连接
nmcli device wifi connect "<SSID>" password "<密码>"
# 3) 确认拿到 192.168.137.x
ip -brief addr show
# 预期：wlan0  UP  192.168.137.x/24
# 4) 确认默认网关（ICS 下应为 192.168.137.1）
ip route show default
# 预期：default via 192.168.137.1 dev wlan0
```

跨机连通性（Jetson 侧发起）：

```bash
ping -c3 192.168.137.1          # 预期 3 packets transmitted, 3 received
```

Windows 侧反向验证：

```powershell
Test-Connection -TargetName 192.168.137.<Jetson的末位> -Count 3 -Quiet   # 预期 True
```

`[注意]` Windows 自带防火墙对 `192.168.137.0/24`（或"公用网络"）默认可能拦截入站 UDP。
**若本机也要跑 ROS 节点看话题**，需放行 ROS 相关的 UDP 入站；只在 Jetson 上跑节点则可暂时不管。

### 5.4 备用方案：有线直连（压缩版）

仅在 Jetson **没有** `wlan*` 时启用：

```bash
ip -brief addr; ip route show default       # Jetson：看是否已同网段
```

```powershell
Get-NetIPAddress -AddressFamily IPv4 | ? {$_.IPAddress -ne '127.0.0.1'}
Test-Connection -TargetName <Jetson IP> -Count 3 -Quiet
```

要点：Jetson 网线与 Windows 接同一台交换机/AP，或直连网口手工配同网段静态 IP；随后仍要做 §6.3 的 UDP 单播探测。

### 5.5 两种拓扑的共性

- **组播在热点上同样不可靠**（Wi-Fi 桥接/ICS 下尤其常见）⇒ 一律按"组播不可用"设计，走 §6 静态对端。
- 判定顺序固定：**先 ping 通 → 再 TCP → 再 UDP 单播 → 最后才谈 DDS**。前一环不通，后面不用试。

---

## 6. DDS：静态对端（热点拓扑下的地址填法）

### 6.1 文件放哪

推荐 **`/etc/ros/fastdds_peers.xml`**（系统级、重启不丢）；图省事放 `~/fastdds_peers.xml`。
**不要**放在会被 `apt upgrade` 覆盖的路径。

### 6.2 内容（热点拓扑的具体填法）

**Jetson 侧**（对端 = Windows 热点地址 **192.168.137.1**）：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="peer_participant" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_only</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>192.168.137.1</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

**Windows 侧**（若也要跑 ROS 节点）：同一个文件，把 `initialPeersList` 里的地址换成
**Jetson 的 `192.168.137.x`**（用 `ip -brief addr show wlan0` 查）。

> 两端各写对方的地址，**不是**各写自己。地址会随 DHCP 变，Jetson 侧建议在路由器/热点里给它固定分配，
> 或改用 `nmcli` 配静态 IP 固定在 `192.168.137.x`。

### 6.3 先证单播 UDP（不需要 ROS 的部分）

```bash
# Jetson：起接收端
python3 -c "
import socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',7447));s.settimeout(10)
d,a=s.recvfrom(64);print('recv',d,'from',a)"
```

```powershell
# Windows：发一包
$u=New-Object System.Net.Sockets.UdpClient
$b=[Text.Encoding]::ASCII.GetBytes("ping")
$u.Send($b,$b.Length,"192.168.137.<Jetson末位>",7447) | Out-Null
```

预期：Jetson 打印 `recv b'ping' from ('192.168.137.1', <port>)`。**这步不通，后面全白搭。**

### 6.4 环境变量与域

```bash
source /opt/ros/{DISTRO}/setup.bash        # humble 或 foxy
export ROS_DOMAIN_ID=42                    # 避开默认 0
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/fastdds_peers.xml
```

`{DISTRO}` 按 §1.3 换成实际发行版；非交互 shell 里 `ROS_DISTRO` 为空，**每条命令前都要 source**。

### 6.5 验证（不依赖组播）

```bash
# 机器 A
ros2 run demo_nodes_cpp talker
# 机器 B
ros2 run demo_nodes_cpp listener      # 预期持续打印 I heard: [Hello World: N]

ros2 topic list                       # 预期含 /chatter
ros2 topic hz /chatter                # 预期 average rate: ~1.000
```

`[待确认]` Humble 的 `rmw_fastrtps` 是否支持 `ROS_STATIC_PEERS` 环境变量（较新版本才有）。
先用上面的 XML（fastrtps 2.6 的文档化路径）；该变量若可用可省掉 XML。

---

## 7. 上电 → bring-up 清单（按依赖排序）

> 每步都给了**判定用预期输出**。不达标就停下排查，不要往下走。

**步骤 0 · 本机准备**（不需要 Jetson）

```bash
cd <项目>/software/ros
python3 -m py_compile gframe.py backends.py car_state_node.py car_cmd_node.py; echo $?   # 预期 0
```

**步骤 1 · 平台对号入座** → §1.1 三条命令 + §1.3 决策表，记下 JetPack/Ubuntu/ROS 分支。

**步骤 2 · ROS 2 可用**

```bash
ls -d /opt/ros/*; source /opt/ros/humble/setup.bash; python3 -c "import rclpy; print('rclpy OK')"
```

预期：`/opt/ros/humble` + `rclpy OK`。不达标按 §1.2 官方页安装。

**步骤 3 · 40-pin UART 设备出现**

```bash
ls -l /dev/ttyTHS*; dmesg | grep -i ttyTHS
```

预期：`/dev/ttyTHS1` 存在、属主 `root:dialout`。不达标回 §2.3。

**步骤 4 · 摘掉控制台占用**

```bash
cat /proc/cmdline | tr ' ' '\n' | grep console
```

预期：改完重启后**看不到** `console=ttyTCU0`。不达标按 §2.4 处理。**改前确认 SSH 可用。**

**步骤 5 · 断电接线（只有人能做的）**

1. 车与 Jetson **都断电**。
2. **先接 GND**：Jetson pin 6/9/14 任一 ↔ STM32 GND。
3. 交叉：Jetson **pin 8 (TX)** → STM32 **PB11 (RX)**；Jetson **pin 10 (RX)** ← STM32 **PB10 (TX)**。
4. **不接 3.3V/5V**（pin 1/17 留空）。
5. **把 JDY-23 整块拔下**（§4.1）。
6. 上电顺序：先 Jetson，后车。

**步骤 6 · 串口回环自检**（不接车；把 pin 8 与 pin 10 短接）

```bash
python3 - <<'PY'
import termios, os, time
fd = os.open("/dev/ttyTHS1", os.O_RDWR | os.O_NOCTTY)
a = termios.tcgetattr(fd)
a[0]=0; a[1]=0; a[2]=termios.CS8|termios.CREAD|termios.CLOCAL
a[3]=0; a[4]=a[5]=termios.B9600
termios.tcsetattr(fd, termios.TCSANOW, a)
os.write(fd, b"PING\n"); time.sleep(0.3)
print("loopback:", os.read(fd, 64)); os.close(fd)
PY
```

预期：`loopback: b'PING\n'`。不达标：查短接线 → §2.5 驱动/DMA → Jetson‑IO 是否生效。
**测完务必拔掉短接线**，否则接车后收发互串。

**步骤 7 · 接车读参数**

```bash
printf '?\n' > /dev/ttyTHS1 & timeout 3 cat /dev/ttyTHS1
```

预期：`PAR: TURN=2 ... O=4.00 ... FW=20261112F` + 每 250ms 一行 G 帧。
不达标：① TX/RX 是否交叉错 ② 是否共地 ③ console 是否还占着 ④ 波特率是否 9600。

**步骤 8 · 热点连通**（§5.2 开热点 → §5.3 Jetson 连接 → 双向 ping）

**步骤 9 · UDP 单播 + DDS 静态对端**（§6.3 → §6.4 → §6.5）

**步骤 10 · 起本项目节点**

```bash
cd <项目>/software/ros
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros/fastdds_peers.xml
python3 car_state_node.py --device /dev/ttyTHS1 --cmd-topic /car/cmd
```

本机侧：

```bash
ros2 topic hz /car/state            # 预期 average rate: 4.x
ros2 topic echo /car/state --once
```

预期：JSON 里 `pos_mm / speed_mm_s / pitch_deg / target_deg / hz` 有值且随时间变化。
**这就是"本机能看到 /car/state"的完成判据。**

**步骤 11 · 指令下发（先架空）**

```bash
ros2 topic pub --once /car/cmd std_msgs/msg/String "{data: '?'}"
ros2 topic pub --once /car/cmd std_msgs/msg/String "{data: '^ 0,0'}"
```

预期：`?` 有 PAR 回显；`^ 0,0` 后车不动。轮子离地再测任何会让车动的指令（`O`、非零 `^`）。

---

## 8. 未确定项（到手后逐条关闭）

1. `[待确认]` **实际型号** → 决定 Ubuntu/ROS 分支（§1.3）；本文不押注。
2. `[待确认]` **40-pin UART 的 `ttyTHS` 编号**：Orin 有论坛与 dmesg 佐证为 `ttyTHS1`；Xavier NX / Nano 现场确认（§2.2）。
3. `[待确认]` **console 占哪个设备**：Orin 通常 `ttyTCU0`；Xavier/Nano 可能 `ttyTHS0`。用 `cat /proc/cmdline` 判定（§2.4）。
4. `[待确认]` **Jetson 自带 WiFi 的频段能力**：若只支持 2.4GHz，Windows 热点必须切 2.4GHz 频段（§5.2 排障表）。
5. `[待确认]` **是否踩到 §2.5 的 UART DMA bug**：现象是"端口在、发不出"；先降 JetPack 或打 PIO 补丁。
6. `[待确认]` `ROS_STATIC_PEERS` 在目标发行版是否可用（XML 方案可兜住）。

---

## 9. 与前一份（x86 版）方案的差异

| 项 | x86 版 | 本文件（Jetson 版） |
|---|---|---|
| 串口 | USB-TTL → `ttyUSB0`/`ttyACM0` | 40-pin 直连 → **`ttyTHS1`**，**不需要适配器** |
| 特有坑 | 认芯片/设备名 | **console 占用** + pinmux（Jetson‑IO）+ DMA bug |
| ROS 版本 | Humble（已装） | 取决于 JetPack：**Humble(6.x) 或 Foxy(5.x)** |
| 网络 | 有线同网段 | **Windows 热点（主方案）**，有线降为备用 |
| DDS | 静态对端 XML | **同**；热点拓扑对端填 `192.168.137.1` ↔ `192.168.137.x` |
| 协议/G 帧/权限/互斥 | — | 完全一致，可直接复用 |
