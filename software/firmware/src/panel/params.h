#pragma once
// ============================================================
// panel/params.{h,cpp} —— 22 项整定参数表 + 三层值语义 + SAVE/LOAD/DEF
//
// 一、三层值语义（调参时最重要的一条）：
//   1) 编译默认值：core/ 里各全局变量的初值（P=1000、D=100、O=2.99、N=0.8 …）；
//      paramsBoot() 开机先抄一份到 paramDefaults[]，DEF 命令就是把它写回 RAM。
//   2) RAM 当前值：命令 P/D/O… 改的就是这一层，掉电即丢。
//   3) flash 已保存值：只有 SAVE 会写；开机 paramsBoot() 若扫到合法记录会自动覆盖 RAM。
//   RAM 与 flash 的差异用 SAVED 标志表达：SAVED=0 表示"改了还没存"。
//
// 二、持久化（实现在 param_store.h / param_flash_f4.h）：
//   记录 104 B，存 F407 扇区 10/11（0x080C0000 / 0x080E0000，各 128KB）双备份，
//   带 FORMAT_VERSION=3、递增 seq、CRC32 与数值有限性校验。
//   写一条 ≈0.5ms（26 字编程），擦一个扇区 ≈1.1s —— 所以写 flash 前必须车静止
//   （paramWriteAllowed()），可能发生的擦除被提前到开机（prepare()）。
//
// 三、22 项参数（键 / 变量 / 单位 / 默认）见 params.cpp 的逐槽注释；
//   其中 O/Z/N 是度，K/I/E 是 deg per (count/50ms)，X 是 (count/50ms)/count，
//   Y 是 count/50ms，M 是 count，P/D/L/S/R/B/H/T/U 是与 STEP 频率相关的量。
//   通用换算：10.8 编码器 count = 1 mm；1 count/50ms = 1.852 mm/s。
//
// 四、不属于这 22 项、不受 SAVE 影响的运行时量：SP（speedCmd）、MS、EG。
//
// 五、关于"取值范围"（这套参数没有统一钳位）：
//   面板单字母通路只对 L/S/R/H/B/X/Y/M 取绝对值，其余项原样写入，不做区间钳位
//   —— 所以语法上的取值范围就是"有限 float"（SAVE 时也只校验有限性，见 param_store.h）。
//   真正"能用"的范围由实测决定：已测出的边界写在各槽注释里；没测过的项标"未验证"，
//   不要按对称/经验外推。改大改小前先看对应槽的实测记录。
// ============================================================
// ============================================================
// panel/params.{h,cpp} — 22 项参数表 + SAVE/LOAD/DEF + 三层值语义
// FORMAT_VERSION 保持 3；参数表顺序与 PAR 回显、面板 22 项一致（一个字未动）。
// （注：旧描述"参数表顺序与 PAR 回显、面板 22 项一致"与当前实现不符，以代码为准：
//   paramSlots 的顺序是 **flash 记录里 values[] 的顺序**；
//   PAR 回显是 **面板显示顺序**，两者键集合相同但顺序不同
//   —— flash 顺序 P D O Z L S R B H K I N W J E X Y M F G T U
//   —— 回显顺序 P D O Z L S R F G T U K I N W J E B H X Y M
//   改 paramSlots 顺序会改变已有 flash 记录的解释，必须同时升 ParamStore::FORMAT_VERSION。）
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "panel/telemetry.h"

namespace panel {

// paramsBoot() —— 开机参数初始化（由 main.cpp setup() 在电机/IMU 初始化后调用一次）。
// 流程: ① 把 22 个全局变量的编译默认值抄进 paramDefaults[]（供 DEF 用）；
//       ② 扫 flash 找 seq 最大且校验通过的记录：有则覆盖 RAM 并打印
//          "PARAMS: loaded from flash seq=N sector=S"，再调 prepare() 预擦备用扇区；
//          没有则打印 "PARAMS: compiled defaults" 并保持编译默认值。
// 参数: 无。返回: void。
// 边界: flash 记录里任何一项不是有限值（NaN/Inf）→ 整条记录作废，回默认值；
//       版本号不匹配 → 同样作废（FORMAT_VERSION 变更的用途）。
// 开机：先抄编译默认值（DEF 用），再尝试用 flash 里的记录覆盖（原 paramStorageBoot）
void paramsBoot();

// handleParamWord() —— 整词命令 SAVE / LOAD / DEF 的入口。
// 参数: cmd/len = 整行（含前导空格），reply = 回显缓冲。
// 返回: true = 命中并已处理（含报错）；false = 不是这三条，交给单字母通路。
// 调用者: panel::handleLine()，位于两字母通路之后、单字母解析之前。
//       必须先拦：否则 SAVE 会被拆成 S+"AVE" 再被数值守卫静默丢弃。
// 边界: 三条命令都要求车静止（paramWriteAllowed()），否则回 "ERR:xxx needs idle vehicle"；
//       SAVE 可能触发扇区擦除（1.1s 级停住取指）—— 这就是必须车静止的原因。
// SAVE / LOAD / DEF 整词命令（返回 true = 已处理）
bool handleParamWord(const char * cmd, uint8_t len, QueuedBtReply & reply);

// paramWriteAllowed() —— 写 flash 的前置条件（三条同时满足）。
// 返回: true = 允许 SAVE/LOAD/DEF。
// 条件: 手动通路不活跃（!remote::isActive()）、位置点动未进行（!positionJog.active）、
//       IMU 无故障（imuFailCount==0）。
// 边界: 只查"是否在动"，不查姿态角 —— 车静止但车身倾斜时仍允许 SAVE。
// 写 flash 会停住取指：必须车静止（原 paramWriteAllowed）
bool paramWriteAllowed();

// markDirtyIfParam() —— 单字母命令改到 22 项之一时，把"未保存"标志置起来。
// 参数: key = 命令里的原始字符（大小写均可，内部转大写查表）。
// 返回: void。边界: 键不在 22 项表里（例如 V/C）→ 什么都不做；
//       SP/SV/EG/MS 走两字母通路，不经过本函数（改 SV 不会把 SAVED 置 0，属既有行为）。
// 单字母参数被改动 → 标记"RAM 值 != flash 记录"（原 paramSlotIndexOf(key)>=0 → paramDirty=true）
void markDirtyIfParam(char key);

// savedFlag() —— PAR 回显的 SAVED 字段。
// 返回: 0 = RAM 与 flash 不一致（有改动未存或 SAVE 失败）；1 = 一致且 flash 有有效记录；
//       若 flash 从来没存过（paramSaved=false）则返回 0。
// 用途: 面板据此显示橙色"未保存"提示。
// PAR 回显里的 SAVED 标志：paramDirty ? 0 : (paramSaved ? 1 : 0)
int savedFlag();

}  // namespace panel
