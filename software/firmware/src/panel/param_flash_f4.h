#pragma once
#include "param_store.h"

// ============================================================
// panel/param_flash_f4.h —— STM32F407ZG 内部 flash 上的双备份参数区
//
// 扇区映射（F407ZGT6 1MB flash）：
//   扇区 0~3 各 16KB，扇区 4 为 64KB，扇区 5~11 各 128KB。
//   本文件取最后两个 128KB 扇区：10 = 0x080C0000，11 = 0x080E0000。
//   => 应用本体必须 < 768KB（0x080C0000 之前）；当前固件 pio 报 Flash 89440 B，
//      余量充足，但每次固件变大都要盯 pio 的 size 是否越过 0x080C0000。
//
// 为什么选整扇区：F407 的擦除粒度就是扇区，128KB 擦除典型 1.1s（最长 2s+），
// 所以参数存储走"扇区内追加写"，只在写满换扇区时才擦，且把擦除提前到开机。
//
// 单位/持久化：本文件只做"字级 flash 编程"，不解释参数含义与单位；
// 一条记录的布局/CRC/seq 语义见 param_store.h。掉电安全性由追加写 + 回读校验保证。
//
// 只在 STM32F4xx 下编译；主机回归不编译本文件。
//
// （注：旧描述与当前实现不符，以代码为准：旧注释写"当前固件约 83KB"，
//   2026-11-20 的 pio 实测已是 Flash 89440 B ≈ 87KB；余量判断仍成立，
//   但每次固件变大都要用 pio 输出的 size 重新核对 0x080C0000 这条界线。）
// ============================================================
// STM32F407ZG 内部 flash 双备份扇区（2026-11-10）
// ---------------------------------------------------------------
// 1MB flash 的扇区划分：0~3 各 16KB、4 为 64KB、5~11 各 128KB。
// 取最后两个 128KB 扇区（10 = 0x080C0000，11 = 0x080E0000）存参数：
//   - 应用本体必须小于 768KB（当前固件约 83KB，余量充足；pio run 的 size 要盯）
//   - 两个扇区互为备份，写满一个才擦另一个
//
// 时序：单条记录 = 26 次字编程，每次约 16us，合计约 0.5ms；两次编程之间代码
// 取指会被 flash 控制器停住（这正是不在 400Hz 环里写的原因）。擦除一整个
// 128KB 扇区典型 1.1s，只在"写满换扇区"时发生，因此调用方必须先确认车已静止。
#ifdef STM32F4xx
#include <stm32f4xx_hal.h>

// Stm32F4FlashPort —— ParamStore::FlashPort 的 F407 实现（闪存扇区 10/11）。
// 所有编程/擦除都遵循 F407 规则：先解锁、清错误标志、操作、上锁，
// 并在临界区用 PRIMASK 关中断，防止 flash 忙时中断向量取指出问题。
class Stm32F4FlashPort : public ParamStore::FlashPort {
public:
    // 两个备份扇区的起始地址（字地址）：0x080C0000 = 扇区10，0x080E0000 = 扇区11。
    static constexpr uint32_t SECTOR_ADDRESS[2] = {0x080C0000u, 0x080E0000u};
    // HAL 的扇区编号参数（FLASH_SECTOR_10 / FLASH_SECTOR_11），与上面地址一一对应。
    static constexpr uint32_t HAL_SECTOR_ID[2]  = {FLASH_SECTOR_10, FLASH_SECTOR_11};
    // 每个扇区的字节数 128KB；除以 104B 得每扇区 1260 个记录槽。
    static constexpr uint32_t SECTOR_BYTES      = 128u * 1024u;

    uint32_t sectorSizeBytes() const override { return SECTOR_BYTES; }
    uint32_t sectorCount() const override { return 2; }

    // readWords(): 把扇区首地址当成 uint32_t 只读视图返回；sector 越界返回 nullptr。
    // 返回的指针直接指向 flash 存储映射区，调用方只读。
    const uint32_t* readWords(uint32_t sector) const override {
        if (sector > 1u) return nullptr;
        return reinterpret_cast<const uint32_t*>(SECTOR_ADDRESS[sector]);
    }

    // eraseSector(): 擦整个 128KB 扇区（典型 1.1s，最长 2s+，掉电会得到半擦状态）。
    // 返回: HAL_OK 且没有坏扇区才 true。
    // 边界: sector>1 直接 false；临界区内 HAL_FLASH_Unlock/Lock 配对，
    //       PRIMASK 保存-恢复，保证返回时中断状态与进入时一致。
    bool eraseSector(uint32_t sector) override {
        if (sector > 1u) return false;
        FLASH_EraseInitTypeDef init = {};
        init.TypeErase    = FLASH_TYPEERASE_SECTORS;
        init.Sector       = HAL_SECTOR_ID[sector];
        init.NbSectors    = 1;
        init.VoltageRange = FLASH_VOLTAGE_RANGE_3;   // 2.7~3.6V，按 32 位字编程
        uint32_t badSector = 0xFFFFFFFFu;
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        HAL_FLASH_Unlock();
        clearErrorFlags();
        HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&init, &badSector);
        HAL_FLASH_Lock();
        __set_PRIMASK(primask);
        return st == HAL_OK && badSector == 0xFFFFFFFFu;
    }

    // programWord(): 向扇区第 wordIndex 个 32 位字写 value（只能 1→0）。
    // 返回: true=成功且回读一致。
    // 边界:
    //   * sector>1 / 地址不合法 → false；
    //   * 目标已是期望值 → 直接 true（省一次 ~16us 编程停顿，擦除态的 0xFFFFFFFF 同理）；
    //   * 目标不是 0xFFFFFFFF 且不等于期望值 → false（拒绝 AND 出脏值）；
    //   * 单字编程约 16us，26 字一条记录 ≈0.5ms。
    bool programWord(uint32_t sector, uint32_t wordIndex, uint32_t value) override {
        if (sector > 1u) return false;
        volatile uint32_t* addr = reinterpret_cast<volatile uint32_t*>(
            SECTOR_ADDRESS[sector] + wordIndex * 4u);
        // 目标已经是期望值就不写（擦除后的 0xFFFFFFFF 同理），省一次编程停顿
        if (*addr == value) return true;
        if (*addr != 0xFFFFFFFFu) return false;   // 已被占用：拒绝"写已有数据"，否则会 AND 出脏值
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        HAL_FLASH_Unlock();
        clearErrorFlags();
        HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                                 reinterpret_cast<uint32_t>(addr), value);
        HAL_FLASH_Lock();
        __set_PRIMASK(primask);
        return st == HAL_OK && *addr == value;
    }

private:
    // clearErrorFlags(): 清掉上一轮可能残留的 flash 错误标志，否则下一次 HAL 调用直接失败。
    static void clearErrorFlags() {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                               FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    }
};
#endif  // STM32F4xx
