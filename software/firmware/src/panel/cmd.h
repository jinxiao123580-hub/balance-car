#pragma once
// ============================================================
// panel/cmd.{h,cpp} —— BLE（JDY-23，USART3@9600）命令解析与分发
//
// 一、解析优先级（顺序就是 handleLine() 里的判断顺序，不可换）：
//   1) 两字母运行时通路：SP / SV / EG / MS
//      —— 必须最先拦，否则会被下面的单字母解析拆成 S+"P…"、M+"S…" 等；
//   2) 整词命令：SAVE / LOAD / DEF
//      —— 同理，否则会被拆成 S+"AVE"、L+"OAD"、D+"EF" 再被数值守卫静默丢弃；
//   3) '?' 参数回显（PAR + JOGCFG 两行）；'!' 无值=诊断信息；
//   4) '!' 带值 = JOGCFG 配置；'~' / '&' = 位置点动帧/心跳；
//      '^' / '@' = 手动俯仰/转向通路；
//   5) 单字母参数（22 项）+ 特殊键 V（转向 Hz）、C（捕获当前位置目标）；
//   6) 其它：值必须以数字/-/. 开头，否则整条静默忽略；未知键回 "ERR:unknown cmd"。
//
// 二、22 项参数（单字母，单位/默认/实测边界见 params.cpp 的逐槽注释）：
//   P D O Z L S R B H K I N W J E X Y M F G T U
//   一一对应 core/state.h 里的全局变量；改 RAM 立即生效，SAVE 才进 flash。
//
// 三、只改 RAM 的命令（断电即回编译默认值，不进 22 项参数表）：
//   SP（speedCmd 目标速度，count/50ms）、MS（MAX_TARGET_FORWARD_SPEED 上限）、
//   EG（^ 通路的 errorGain，deg per count/50ms）。
//   注意 SV 写的是 speedKp —— 它本身是 22 项里的 K，会被 SAVE 保存，
//   但 SV 走两字母通路、不经过 markDirtyIfParam()，所以改 SV 不会把 SAVED 置 0。
//
// 四、单位换算（全项目通用，务必按这个来）：
//   10.8 编码器 count = 1 mm  =>  1 count/50ms = 20 count/s = 1.852 mm/s。
//   遥测 pos 字段 = vehiclePosition × 100，电脑端 /1080 即得 mm
//   （= count/10.8，两个 100 约掉）。
//   注：docs 里有 "×20/1080" 的写法（漏了 pos 字段的 ×100），以本节为准。
//
// 五、调用：panel::poll() 在 main.cpp loop() 每圈开头调用（≥400Hz）；
//   handleLine() 只在收到完整行（\n 或 \r）时由 poll() 调。
//
// 六、已知旧注释不一致（原文保留，以本文件头清单为准）：
//   旧注释写"单字母22项 + SP/MS/SV + SAVE/LOAD/DEF/?/! + ^ / ~ & 转发远控"：
//   ① 漏了 EG、@、C、V 四项（EG 在两字母通路，@ 与 ^ 同格式，C 是位置目标捕获，
//      V 是手动转向 Hz）；
//   ② "~ / &" 是位置点动子协议，不是"^ 转发远控"的一部分。
// ============================================================
// ============================================================
// panel/cmd.{h,cpp} — BLE 命令解析
// 单字母22项 + SP/MS/SV + SAVE/LOAD/DEF/?/! + ^ / ~ & 转发远控
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "panel/params.h"
#include "panel/telemetry.h"

#include "control/balance.h"
#include "control/cascade.h"
#include "control/motor.h"
#include "control/imu.h"
#include "remote/remote_control.h"
#include "remote/manual_pitch.h"
#include "remote/manual_turn.h"
#include "remote/position_jog.h"

namespace panel {

// poll() —— 每圈从 BTSerial 取字节拼成一条命令，遇到换行就交给 handleLine()。
// 参数: 无。返回: void。调用者: main.cpp loop() 开头，每个迭代一次（≥400Hz）。
// 边界: 单次最多读 32 B（9600 波特下绰绰有余）；行缓冲 48 B，超长整行丢弃
//       （btCmdOverflow 标志，直到下一个换行才复位），不会截断后误执行。
// 收行 → 分发（原 pollBtCommand；main 每圈开头调用）
void poll();

// handleLine() —— 处理一条完整命令（不含换行符）。
// 参数: cmd/len = 行内容与长度，允许前导空格与大小写混用。
// 返回: void；所有回显通过局部 QueuedBtReply 在析构时整行入队（非阻塞）。
// 调用者: poll()。频率: 每收到一条命令一次（面板手点级别）。
// 边界: 见文件头"解析优先级"；数值非法会回 ERR，未知键回 ERR:unknown cmd，
//       值不是数字开头的单字母命令会被静默忽略（防 "SAVE"/"DEF" 被误设成 0）。
// 处理一整行命令（原 handleBtCommand）
void handleLine(char * cmd, uint8_t len);

}  // namespace panel
