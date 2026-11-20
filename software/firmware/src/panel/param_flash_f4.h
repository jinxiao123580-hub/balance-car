#pragma once
#include "param_store.h"

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

class Stm32F4FlashPort : public ParamStore::FlashPort {
public:
    static constexpr uint32_t SECTOR_ADDRESS[2] = {0x080C0000u, 0x080E0000u};
    static constexpr uint32_t HAL_SECTOR_ID[2]  = {FLASH_SECTOR_10, FLASH_SECTOR_11};
    static constexpr uint32_t SECTOR_BYTES      = 128u * 1024u;

    uint32_t sectorSizeBytes() const override { return SECTOR_BYTES; }
    uint32_t sectorCount() const override { return 2; }

    const uint32_t* readWords(uint32_t sector) const override {
        if (sector > 1u) return nullptr;
        return reinterpret_cast<const uint32_t*>(SECTOR_ADDRESS[sector]);
    }

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
    static void clearErrorFlags() {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                               FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    }
};
#endif  // STM32F4xx
