#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// ============================================================
// panel/param_store.h —— 22 项参数的持久化纯逻辑层（与平台无关）
//
// 为什么需要它：22 项整定参数原来只能靠 ST-Link 重烧，现场调一次占一次线；
// 有了它，面板发一条 SAVE 就断电可保。
//
// 设计要点（下面原有注释讲得对的部分保留，这里补单位/边界/持久化语义）：
//   * 记录 Record = 104 B = 26 个 32 位字；参数本体 22 个 float=88 B，
//     头部 magic(4)+version/count(4)+seq(4)+CRC(4) 共 16 B。
//   * flash 只能 1→0，所以"追加写"而不是就地改：一个 128KB 扇区可存 1260 条。
//   * 双扇区互为备份，开机扫两边取 seq 最大且 CRC/版本/有限性都过的记录。
//   * 一条记录 26 次字编程 ≈ 0.5ms；擦一个 128KB 扇区 ≈1.1s（典型）。
//     因此写 flash 必须车静止（见 params.cpp paramWriteAllowed()），
//     并把"可能发生的擦除"提前到开机（prepare()）。
//   * 参数单位由具体变量决定（P/D/R 无量纲或 Hz 比，O/Z/N 是度，
//     K/I 是 deg 每 (count/50ms)，X/Y 是 count/50ms 与 count）——
//     本层只存 float，不解释单位。
//   * 本层可在主机回归里用 RAM 假 flash 跑（FlashPort 是纯虚接口）。
// ============================================================
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

    // 记录里的参数个数：与面板 22 项、PAR 回显的键集合一致（顺序见 params.cpp）。
    // 改它必须同步改 params.cpp 的 paramSlots 并升 FORMAT_VERSION。
constexpr uint8_t  PARAM_COUNT    = 22;   // 与面板 22 项、PAR 回显顺序一致
    // 记录格式版本：改参数顺序/语义必须 +1，否则旧记录会被当成有效值套到新语义上。
    // 2026-11-12 从 2 升到 3，作废旧记录以让新默认值（O=4.0 那一版）生效。
constexpr uint16_t FORMAT_VERSION = 3;    // 改参数顺序/语义必须 +1（2026-11-12：+1 以作废旧记录，让 O=4.0 默认值生效）
    // 记录魔数 "PCH3"（小端字节序列 'P','C','H','3'），用于快速排除空白/垃圾扇区。
constexpr uint32_t RECORD_MAGIC   = 0x50434833u;  // "PCH3"
    // 擦除态字（NOR flash 擦后全 1）。扫描时"首字 == 该值"即视为空槽，后面不再看。
constexpr uint32_t ERASED_WORD    = 0xFFFFFFFFu;

// 一条持久化记录（104 B，4 字节对齐、无填充）。
// 字段顺序即 flash 里的字顺序，recordValid() 与 save() 都依赖这个布局：
//   magic(字0) / version|count(字1) / seq(字2) /
//   values[22](字3..24) / crc(字25)。
// 单位：values[i] 的单位由 params.cpp 第 i 个槽的变量决定；本结构不解释单位。
// 持久化：只有 SAVE 会写它；LOAD 只读。
struct Record {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t seq;
    float    values[PARAM_COUNT];
    uint32_t crc;                  // 覆盖前面所有字节（不含本字段）
};

// 编译期护栏：记录布局必须正好 104 B（=26 字），否则闪存偏移/CRC 覆盖范围都会错。
static_assert(sizeof(float) == 4, "记录格式假定 float 为 4 字节");
static_assert(sizeof(Record) == 104, "记录必须是 104 B（4 字节对齐，无填充）");
constexpr uint32_t RECORD_WORDS = sizeof(Record) / 4;   // 26

// CRC32（IEEE 802.3，反射多项式 0xEDB88320），对该记录除 crc 字段外的 100 B 计算。
// 参数: data/len 为待校验字节；crc 为链式初值（默认 0xFFFFFFFF）。
// 返回: 最终 CRC（含末尾取反）。边界: 逐位实现，不用查表；104B 量级开销可忽略。
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
// CRC 覆盖的载荷长度 = sizeof(Record) - 4 = 100 B（不含最后一个 crc 字）。
constexpr uint32_t RECORD_PAYLOAD_BYTES = sizeof(Record) - sizeof(uint32_t);

// 平台适配口：F407 上直接映射到 flash 扇区；主机回归用 RAM 假 flash。
// FlashPort —— 平台适配接口（纯虚，无状态）。
// F407 实机实现见 param_flash_f4.h（映射到 flash 扇区 10/11）；
// 主机回归用 RAM 假 flash，因此本层不 include 任何 MCU 头。
//   sectorSizeBytes() 每扇区字节数（F407: 128KB）
//   sectorCount()     参与备份的扇区数（当前 2）
//   readWords(s)      返回第 s 扇区第 0 字的只读视图（nullptr = 不可读）
//   eraseSector(s)    擦整扇区，成功 true（耗时可达秒级，掉电会得到半擦状态）
//   programWord(s,i,v) 写第 i 个 32 位字；只允许 1→0，目标非 0xFFFFFFFF 时返回 false
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

// Store —— 参数记录的无状态算法集（scan / prepare / save）。
class Store {
public:
    static constexpr uint32_t MAX_SECTORS = 4;

    // 扫描结果（值语义，可整体拷贝）。
    // found=false 表示两个扇区里都没有合法记录（→ 回编译默认值）。
    // seq 是有效记录里最大的序号，每次 SAVE +1；sector 是该记录所在扇区；
    // nextSlot[i] 是第 i 扇区第一个空槽（= 下次追加位置），单位=槽。
    struct Scan {
        bool     found  = false;      // 是否找到有效记录
        uint32_t seq    = 0;          // 有效记录里的最大 seq
        uint8_t  sector = 0;          // 该记录所在扇区（新记录优先追加到这里）
        uint32_t nextSlot[MAX_SECTORS] = {};  // 每个扇区下一个可写槽
    };

    // 扫描全部扇区，取 seq 最大且 CRC 正确、数值有限的记录。
    // values 可为 nullptr（只想知道追加位置）。
    // scan(): 遍历所有扇区，取 seq 最大且 recordValid() 通过的记录。
    // 参数: port 平台适配口；values 可为 nullptr（只想知道追加位置）。
    // 返回: Scan（found/seq/sector/nextSlot）。边界: count==0 或 >MAX_SECTORS 直接返回空；
    //       readWords()==nullptr 的扇区跳过，nextSlot 记 0；
    //       遇到首字为 0xFFFFFFFF 的空槽即停止该扇区（断电写一半的半条记录会被跳过，
    //       因为它首字已不是 0xFFFFFFFF）。
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
    // prepare(): 开机预清 —— 当前扇区已写满时，趁开机（电机未使能、无 400Hz 环）
    // 先把要换的那个扇区擦好。参数: port / sc（会就地更新 sc.sector）。
    // 返回: true=无需操作或已备好；false=擦除失败。
    // 边界: 至少 2 个扇区才做双备份；只有 nextSlot 等于槽数（写满）才擦另一扇区。
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
    // save(): 追加一条记录（先把 values 打包成 Record，再写 26 个字）。
    // 参数: port / values(=22 个 float，顺序见 params.cpp) / sc(就地更新) / erasedAny(输出)。
    // 返回: true=写入并回读校验通过；false=任何一步失败（擦除/编程/回读不一致）。
    // 边界:
    //   * 没有有效记录时先擦扇区 0（可能有旧 magic/version 的垃圾记录占着，不能覆盖写）；
    //   * 本扇区写满则擦另一扇区再写，erasedAny=true（耗时可达秒级，调用方须停车）；
    //   * 写完立即 memcmp 回读，避免"以为存了"；
    //   * sc.seq 单调 +1，双扇区靠它取新。
    bool save(FlashPort& port, const float* values, Scan& sc, bool& erasedAny) {
        erasedAny = false;
        const uint32_t sectors = port.sectorCount();
        if (sectors == 0 || sectors > MAX_SECTORS || values == nullptr) return false;

        uint32_t sector;
        uint32_t slot;
        uint32_t seq;
        if (!sc.found) {
            // 没有有效记录时，扇区里可能还躺着旧版本的记录（例如 magic/version 变过），
            // flash 只能把 1 写成 0，直接覆盖会失败，所以先擦干净再写。
            if (!port.eraseSector(0)) return false;
            erasedAny = true;
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
    // recordValid(): 单条记录校验（不修改任何状态）。
    // 顺序: CRC → magic → version → count → 22 个 float 的有限性（同时挡 NaN/Inf）。
    // 返回: true=可用于覆盖 RAM 参数。
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
