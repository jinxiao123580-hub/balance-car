#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// 参数持久化的纯逻辑层（2026-11-10）
// ---------------------------------------------------------------
// 背景：22 项整定参数此前只能靠 ST-Link 重烧，现场调一次就要占一次线。
// 目标：面板发一条 SAVE，断电重启仍是新值。
//
// 为什么不是"每次都擦扇区"：F407 的 128KB 扇区擦除典型 1.1 s（最长 2 s 以上），
// 远超 400 Hz 控制环能容忍的停顿。所以采用**扇区内追加写**：每条记录 104 B
// （26 个 32 位字，编程约 0.5 ms），一个 128KB 扇区能存 1200+ 条，
// 只有写满时才去擦另一个扇区（≈1200 次保存才一次，且要求车静止）。
//
// 布局（每个扇区分成等长槽位，槽位 = 一条记录）：
//   [slot0][slot1][slot2] ... [空槽 0xFFFFFFFF ...]
// 追加点 = 第一个"首字仍为 0xFFFFFFFF"的槽。断电写了半条记录时，那个槽的首字
// 已经不是 0xFFFFFFFF，于是被跳过而不是被覆盖 —— 半个记录最多浪费一个槽。
//
// 双备份：两个扇区各存一串记录，每条带递增 seq。开机扫两边取 seq 最大且
// CRC 正确者；都不行就回编译默认值。写满后擦的是"较旧"的那个扇区。
namespace ParamStore {

constexpr uint8_t  PARAM_COUNT    = 22;   // 与面板 22 项、PAR 回显顺序一致
constexpr uint16_t FORMAT_VERSION = 2;    // 改参数顺序/语义必须 +1
constexpr uint32_t RECORD_MAGIC   = 0x50434832u;  // "PCH2"
constexpr uint32_t ERASED_WORD    = 0xFFFFFFFFu;

struct Record {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t seq;
    float    values[PARAM_COUNT];
    uint32_t crc;                  // 覆盖前面所有字节（不含本字段）
};

static_assert(sizeof(float) == 4, "记录格式假定 float 为 4 字节");
static_assert(sizeof(Record) == 104, "记录必须是 104 B（4 字节对齐，无填充）");
constexpr uint32_t RECORD_WORDS = sizeof(Record) / 4;   // 26

// CRC32（IEEE 802.3，反射多项式 0xEDB88320）：逐位实现，104 字节量级不心疼。
inline uint32_t crc32(const void* data, size_t len, uint32_t crc = 0xFFFFFFFFu) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) {
            uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// 记录本身 = 除 crc 之外的全部字段
constexpr uint32_t RECORD_PAYLOAD_BYTES = sizeof(Record) - sizeof(uint32_t);

// 平台适配口：F407 上直接映射到 flash 扇区；主机回归用 RAM 假 flash。
class FlashPort {
public:
    virtual ~FlashPort() = default;
    virtual uint32_t sectorSizeBytes() const = 0;
    virtual uint32_t sectorCount() const = 0;
    // 指向扇区首字的只读视图（期望是存储映射或缓冲区，总长 sectorSizeBytes）
    virtual const uint32_t* readWords(uint32_t sector) const = 0;
    virtual bool eraseSector(uint32_t sector) = 0;
    virtual bool programWord(uint32_t sector, uint32_t wordIndex, uint32_t value) = 0;
};

class Store {
public:
    static constexpr uint32_t MAX_SECTORS = 4;

    struct Scan {
        bool     found  = false;      // 是否找到有效记录
        uint32_t seq    = 0;          // 有效记录里的最大 seq
        uint8_t  sector = 0;          // 该记录所在扇区（新记录优先追加到这里）
        uint32_t nextSlot[MAX_SECTORS] = {};  // 每个扇区下一个可写槽
    };

    // 扫描全部扇区，取 seq 最大且 CRC 正确、数值有限的记录。
    // values 可为 nullptr（只想知道追加位置）。
    Scan scan(const FlashPort& port, float* values) const {
        Scan out;
        const uint32_t count = port.sectorCount();
        if (count == 0 || count > MAX_SECTORS) return out;
        for (uint32_t s = 0; s < count; ++s) {
            const uint32_t* base = port.readWords(s);
            const uint32_t slots = port.sectorSizeBytes() / sizeof(Record);
            if (base == nullptr) { out.nextSlot[s] = 0; continue; }
            uint32_t slot = 0;
            for (; slot < slots; ++slot) {
                const uint32_t* w = base + slot * RECORD_WORDS;
                if (w[0] == ERASED_WORD) break;   // 首个空槽，往后的都不必看
                if (recordValid(w)) {
                    uint32_t seq = w[2];          // 字 0=magic 1=version|count 2=seq
                    if (!out.found || seq > out.seq) {
                        out.found  = true;
                        out.seq    = seq;
                        out.sector = static_cast<uint8_t>(s);
                        if (values != nullptr) {
                            std::memcpy(values,
                                        reinterpret_cast<const uint8_t*>(w) + 12,
                                        PARAM_COUNT * sizeof(float));
                        }
                    }
                }
            }
            out.nextSlot[s] = slot;
        }
        return out;
    }

    // 开机预清：当前扇区已写满时，趁开机（电机未使能、没有 400Hz 环）先把
    // 该擦的那个扇区擦好，避免将来 SAVE 时为了腾地方停住取指 1.1s 让车摔倒。
    bool prepare(FlashPort& port, Scan& sc) {
        const uint32_t sectors = port.sectorCount();
        if (!sc.found || sectors < 2 || sectors > MAX_SECTORS) return true;
        const uint32_t slotsPerSector = port.sectorSizeBytes() / sizeof(Record);
        if (sc.nextSlot[sc.sector] < slotsPerSector) return true;
        const uint32_t other = (sc.sector + 1u) % sectors;
        if (!port.eraseSector(other)) return false;
        sc.nextSlot[other] = 0;
        sc.sector = static_cast<uint8_t>(other);   // 下一次追加落到刚擦好的扇区
        return true;
    }

    // 追加一条记录。sc 会就地更新（找到新记录、下一个空槽）。
    // erasedAny=true 表示这次为了腾地方擦了扇区（耗时可到秒级，调用方要提前停车）。
    bool save(FlashPort& port, const float* values, Scan& sc, bool& erasedAny) {
        erasedAny = false;
        const uint32_t sectors = port.sectorCount();
        if (sectors == 0 || sectors > MAX_SECTORS || values == nullptr) return false;

        uint32_t sector;
        uint32_t slot;
        uint32_t seq;
        if (!sc.found) {
            sector = 0;
            slot   = 0;
            seq    = 1;
        } else {
            sector = sc.sector;
            slot   = sc.nextSlot[sector];
            seq    = sc.seq + 1;
            if (slot >= port.sectorSizeBytes() / sizeof(Record)) {
                // 本扇区写满：换到另一个扇区，先擦干净
                uint32_t other = (sector + 1u) % sectors;
                if (!port.eraseSector(other)) return false;
                erasedAny = true;
                sector = other;
                slot   = 0;
                sc.nextSlot[other] = 0;
            }
        }

        Record rec = {};
        rec.magic   = RECORD_MAGIC;
        rec.version = FORMAT_VERSION;
        rec.count   = PARAM_COUNT;
        rec.seq     = seq;
        std::memcpy(rec.values, values, PARAM_COUNT * sizeof(float));
        rec.crc = crc32(&rec, RECORD_PAYLOAD_BYTES);

        const uint32_t* words = reinterpret_cast<const uint32_t*>(&rec);
        for (uint32_t i = 0; i < RECORD_WORDS; ++i) {
            if (!port.programWord(sector, slot * RECORD_WORDS + i, words[i])) return false;
        }

        // 回读校验（写完立刻确认，避免"以为存了"）
        const uint32_t* base = port.readWords(sector) + slot * RECORD_WORDS;
        if (base == nullptr || std::memcmp(base, &rec, sizeof(rec)) != 0) return false;

        sc.found  = true;
        sc.seq    = seq;
        sc.sector = static_cast<uint8_t>(sector);
        sc.nextSlot[sector] = slot + 1;
        return true;
    }

private:
    static bool recordValid(const uint32_t* w) {
        uint32_t crc = crc32(w, RECORD_PAYLOAD_BYTES);
        if (crc != w[RECORD_WORDS - 1]) return false;
        if (w[0] != RECORD_MAGIC) return false;
        // version(低 16 位) | count(高 16 位)
        if (static_cast<uint16_t>(w[1] & 0xFFFFu) != FORMAT_VERSION) return false;
        if (static_cast<uint16_t>(w[1] >> 16) != PARAM_COUNT) return false;
        // 有限性检查：自平衡车吃到 NaN/Inf 参数会直接失控，宁可回默认值
        const float* v = reinterpret_cast<const float*>(reinterpret_cast<const uint8_t*>(w) + 12);
        for (uint8_t i = 0; i < PARAM_COUNT; ++i) {
            if (!(v[i] > -1.0e9f && v[i] < 1.0e9f)) return false;   // 同时挡住 NaN
        }
        return true;
    }
};

}  // namespace ParamStore
