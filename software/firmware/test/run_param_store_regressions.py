"""参数持久化（param_store.h）主机回归。

覆盖：记录编解码、CRC32 标准向量、追加写、A/B 回退、写满换扇区、
断电写了半条记录的恢复、NaN/Inf 参数拒收。
这些是"烧一次就能省一次 ST-Link"的前提，改 flash 布局前必须先过。
"""
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
src = root / "src"

PROGRAM = r'''
#include "param_store.h"
#include <cassert>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

using namespace ParamStore;

// 模拟 NOR flash：只能把 1 写成 0，擦除恢复成 0xFFFFFFFF
struct FakePort : public FlashPort {
    std::vector<uint8_t> mem;
    uint32_t bytesPerSector;
    uint32_t sectors;
    uint32_t eraseCount = 0;

    FakePort(uint32_t sectorBytes, uint32_t count)
        : mem(static_cast<size_t>(sectorBytes) * count, 0xFF),
          bytesPerSector(sectorBytes), sectors(count) {}

    uint32_t sectorSizeBytes() const override { return bytesPerSector; }
    uint32_t sectorCount() const override { return sectors; }

    const uint32_t* readWords(uint32_t sector) const override {
        return reinterpret_cast<const uint32_t*>(mem.data() + sector * bytesPerSector);
    }
    uint32_t* raw(uint32_t sector) {
        return reinterpret_cast<uint32_t*>(mem.data() + sector * bytesPerSector);
    }
    bool eraseSector(uint32_t sector) override {
        eraseCount++;
        std::memset(mem.data() + sector * bytesPerSector, 0xFF, bytesPerSector);
        return true;
    }
    bool programWord(uint32_t sector, uint32_t wordIndex, uint32_t value) override {
        uint32_t* w = raw(sector);
        if (wordIndex * 4u >= bytesPerSector) return false;
        w[wordIndex] &= value;          // NOR：1 才能写成 0
        return true;
    }
};

static void fill(float* v, float base) {
    for (uint8_t i = 0; i < PARAM_COUNT; ++i) v[i] = base + i * 0.5f;
}

int main() {
    // 0) CRC32 标准向量
    assert(crc32("123456789", 9) == 0xCBF43926u);

    // 1) 空 flash：扫不到；存一条后能读回（= 断电重启）
    {
        FakePort port(128 * 1024, 2);
        Store store;
        float none[PARAM_COUNT];
        Store::Scan sc = store.scan(port, none);
        assert(!sc.found);
        assert(sc.nextSlot[0] == 0 && sc.nextSlot[1] == 0);

        float in[PARAM_COUNT], out[PARAM_COUNT];
        fill(in, 100.0f);
        bool erased = true;
        assert(store.save(port, in, sc, erased));
        assert(!erased);
        assert(sc.found && sc.seq == 1 && sc.sector == 0 && sc.nextSlot[0] == 1);

        memset(out, 0, sizeof(out));
        Store::Scan boot = store.scan(port, out);
        assert(boot.found && boot.seq == 1);
        for (uint8_t i = 0; i < PARAM_COUNT; ++i) assert(out[i] == in[i]);

        // 2) 追加：seq 递增、槽位前进、同扇区
        float in2[PARAM_COUNT];
        fill(in2, -5.0f);
        Store::Scan sc2 = store.scan(port, out);
        assert(store.save(port, in2, sc2, erased));
        assert(sc2.seq == 2 && sc2.sector == 0 && sc2.nextSlot[0] == 2);
        store.scan(port, out);
        for (uint8_t i = 0; i < PARAM_COUNT; ++i) assert(out[i] == in2[i]);

        // 3) 最新一条 CRC 坏掉 -> 回退到上一条
        uint32_t* w = port.raw(0);
        w[RECORD_WORDS] ^= 0x00000001u;          // 破坏 seq=2 那条的 magic
        Store::Scan b3 = store.scan(port, out);
        assert(b3.found && b3.seq == 1);
        for (uint8_t i = 0; i < PARAM_COUNT; ++i) assert(out[i] == in[i]);
    }

    // 4) 写满换扇区：每扇区 4 槽 -> 第 5 条落进扇区 1，且只擦一次
    {
        FakePort port(4 * sizeof(Record), 2);
        Store store;
        float v[PARAM_COUNT], out[PARAM_COUNT];
        Store::Scan sc;
        bool dummyErased = false;
        for (int k = 0; k < 4; ++k) {
            fill(v, static_cast<float>(k));
            assert(store.save(port, v, sc, dummyErased));
        }
        assert(port.eraseCount == 0);
        Store::Scan b = store.scan(port, out);
        assert(b.found && b.seq == 4 && b.sector == 0 && b.nextSlot[0] == 4);

        fill(v, 42.0f);
        bool erased = false;
        assert(store.save(port, v, b, erased));
        assert(erased && port.eraseCount == 1);
        assert(b.seq == 5 && b.sector == 1 && b.nextSlot[1] == 1);

        Store::Scan after = store.scan(port, out);
        assert(after.found && after.seq == 5 && after.sector == 1);
        for (uint8_t i = 0; i < PARAM_COUNT; ++i) assert(out[i] == v[i]);

        // 扇区 1 也写满 -> 回头擦扇区 0，继续接 seq
        for (int k = 0; k < 3; ++k) {
            fill(v, static_cast<float>(100 + k));
            assert(store.save(port, v, after, erased));
        }
        assert(after.seq == 8 && after.sector == 1 && after.nextSlot[1] == 4);
        fill(v, 7.0f);
        assert(store.save(port, v, after, erased));
        assert(erased && after.seq == 9 && after.sector == 0);
        Store::Scan last = store.scan(port, out);
        assert(last.found && last.seq == 9);
        for (uint8_t i = 0; i < PARAM_COUNT; ++i) assert(out[i] == v[i]);
    }

    // 5) 断电写了半条：首字已写、其余还是 0xFFFFFFFF
    {
        FakePort port(4 * sizeof(Record), 2);
        Store store;
        float v[PARAM_COUNT], out[PARAM_COUNT];
        Store::Scan sc;
        bool dummyErased = false;
        fill(v, 1.0f);
        assert(store.save(port, v, sc, dummyErased));        // slot0 有效
        fill(v, 2.0f);
        assert(store.save(port, v, sc, dummyErased));        // slot1 有效

        // 模拟 slot2 写到第 5 个字时掉电
        uint32_t* w = port.raw(0);
        for (int i = 0; i < 5; ++i) w[2 * RECORD_WORDS + i] = 0xDEADBEEFu;

        Store::Scan b = store.scan(port, out);                    // 仍能读到 slot1
        assert(b.found && b.seq == 2 && b.nextSlot[0] == 3);       // 半条占掉一个槽

        fill(v, 3.0f);
        bool erased = false;
        assert(store.save(port, v, b, erased));                   // 写到下一个槽
        assert(!erased && b.seq == 3 && b.nextSlot[0] == 4);
        Store::Scan after = store.scan(port, out);
        assert(after.found && after.seq == 3);
        for (uint8_t i = 0; i < PARAM_COUNT; ++i) assert(out[i] == v[i]);
    }

    // 6) NaN/Inf 参数拒收（宁可回默认值也不能让车吃到脏参数）
    {
        FakePort port(4 * sizeof(Record), 2);
        Store store;
        float v[PARAM_COUNT], out[PARAM_COUNT];
        Store::Scan sc;
        bool dummyErased = false;
        fill(v, 1.0f);
        v[3] = NAN;
        assert(store.save(port, v, sc, dummyErased));         // 写入层不校验
        Store::Scan b = store.scan(port, out);
        assert(!b.found);                                          // 读取层必须拒收
    }

    // 7) 版本号或 magic 不对 -> 拒收
    {
        FakePort port(4 * sizeof(Record), 2);
        Store store;
        float v[PARAM_COUNT], out[PARAM_COUNT];
        Store::Scan sc;
        bool dummyErased = false;
        fill(v, 1.0f);
        assert(store.save(port, v, sc, dummyErased));
        uint32_t* w = port.raw(0);
        uint32_t keep = w[1];
        w[1] = (keep & 0xFFFFu) | (static_cast<uint32_t>(PARAM_COUNT + 1) << 16);
        assert(!store.scan(port, out).found);
        w[1] = keep;
        w[0] = 0x12345678u | (w[0] & 0xFFFFFFFFu);
        assert(!store.scan(port, out).found);
    }

    // 8) 开机预清：写满后 prepare() 先把另一块擦好，运行中 SAVE 不再需要擦
    {
        FakePort port(4 * sizeof(Record), 2);
        Store store;
        float v[PARAM_COUNT], out[PARAM_COUNT];
        Store::Scan sc;
        bool dummyErased = false;
        for (int k = 0; k < 4; ++k) {
            fill(v, static_cast<float>(k));
            assert(store.save(port, v, sc, dummyErased));
        }
        assert(port.eraseCount == 0);
        assert(store.prepare(port, sc));            // 扇区 0 满了 -> 擦扇区 1
        assert(port.eraseCount == 1 && sc.nextSlot[1] == 0);
        fill(v, 5.0f);
        bool erased = true;
        assert(store.save(port, v, sc, erased));
        assert(!erased);                            // 运行中不再擦
        assert(sc.seq == 5 && sc.sector == 1);
        Store::Scan after = store.scan(port, out);
        assert(after.found && after.seq == 5);
        for (uint8_t i = 0; i < PARAM_COUNT; ++i) assert(out[i] == v[i]);
        // 没写满时 prepare 什么都不做
        assert(store.prepare(port, after) && port.eraseCount == 1);
    }

    puts("param_store regressions OK");
    return 0;
}
'''

TUNE_HEAD = r'''
#include "motion_drive.h"
#include <cassert>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

MotionLeanControl motionLean;
struct QueuedBtReply {
    template <class... T> void print(T...) {}
    template <class... T> void println(T...) {}
};
'''

TUNE_CASES = r'''
static QueuedBtReply stub;

static bool run(const char* text) {
    return handleTunableCommand(text, static_cast<uint8_t>(std::strlen(text)), stub);
}

static bool near(float a, float b) { return std::fabs(a - b) <= 1e-4f; }

int main() {
    // 1) 三个量都能下发
    assert(run("TF 0.30"));
    assert(near(motionLean.forwardTrim, 0.30f));
    assert(run("TB -0.25"));
    assert(near(motionLean.backwardTrim, -0.25f));
    assert(run("EG 0.09"));
    assert(near(motionLean.errorGain, 0.09f));

    // 2) 大小写、前后空白都接受
    assert(run("  tb 0.40  "));
    assert(near(motionLean.backwardTrim, 0.40f));
    assert(run("eg 0.05"));
    assert(near(motionLean.errorGain, 0.05f));

    // 3) 越界要夹住，而不是原样写进去
    assert(run("TF 5"));
    assert(near(motionLean.forwardTrim, 1.0f));
    assert(run("EG 0.5"));
    assert(near(motionLean.errorGain, 0.15f));
    assert(run("EG -1"));
    assert(near(motionLean.errorGain, 0.0f));
    motionLean.errorGain = 0.05f;

    // 4) 坏数值不许改状态
    float keep = motionLean.forwardTrim;
    assert(run("TF"));                 // 缺数值
    assert(run("TF abc"));             // 不是数
    assert(run("TF 1,5"));             // 逗号
    assert(near(motionLean.forwardTrim, keep));

    // 5) 不能抢单字母命令和别的词
    assert(!run("P 900"));
    assert(!run("T 0.5"));
    assert(!run("TFX 1"));
    assert(!run(""));
    assert(!run("SAVE"));

    puts("tunable command regressions OK");
    return 0;
}
'''

WORD_HEAD = r'''
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
'''

WORD_CASES = r'''
static int hits(const char* text) {
    return static_cast<int>(paramWordCommand(text, static_cast<uint8_t>(std::strlen(text))));
}

int main() {
    assert(hits("SAVE") == PARAM_WORD_SAVE);
    assert(hits("save") == PARAM_WORD_SAVE);
    assert(hits("  SAVE  ") == PARAM_WORD_SAVE);
    assert(hits("LOAD") == PARAM_WORD_LOAD);
    assert(hits("def") == PARAM_WORD_DEF);
    // 不是"整条命令只有一个词"就不许抢，否则 P/S 这些单字母命令会被吃掉
    assert(hits("SAVEX") == PARAM_WORD_NONE);
    assert(hits("SAVE 1") == PARAM_WORD_NONE);
    assert(hits("P 900") == PARAM_WORD_NONE);
    assert(hits("?") == PARAM_WORD_NONE);
    assert(hits("") == PARAM_WORD_NONE);
    assert(hits("SAVESAVESAVESAVESAVE") == PARAM_WORD_NONE);   // 超长被截断也不能误命中
    puts("param word command regressions OK");
    return 0;
}
'''


def extract_word_command():
    """从 main.cpp 里抓真实实现来测（不复制一份，避免代码漂移）。"""
    text = (src / "main.cpp").read_text(encoding="utf-8-sig")
    start = text.index("enum ParamWordCommand")
    fn = text.index("static ParamWordCommand paramWordCommand")
    end = text.index("\n}\n", fn) + 3
    return text[start:end]


def extract_tunable():
    """从 main.cpp 抓 handleTunableCommand 的真实实现（TF/TB/EG 路由）。"""
    text = (src / "main.cpp").read_text(encoding="utf-8-sig")
    start = text.index("static bool handleTunableCommand")
    end = text.index("\n}\n", start) + 3
    return text[start:end]


def main():
    with tempfile.TemporaryDirectory() as tmp:
        cpp = Path(tmp) / "t.cpp"
        exe = Path(tmp) / ("t.exe" if sys.platform == "win32" else "t")
        cpp.write_text(PROGRAM, encoding="utf-8")
        subprocess.run(
            ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
             f"-I{src}", str(cpp), "-o", str(exe)],
            check=True,
        )
        subprocess.run([str(exe)], check=True)

        cpp2 = Path(tmp) / "word.cpp"
        exe2 = Path(tmp) / ("word.exe" if sys.platform == "win32" else "word")
        cpp2.write_text(WORD_HEAD + extract_word_command() + WORD_CASES, encoding="utf-8")
        subprocess.run(
            ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
             str(cpp2), "-o", str(exe2)],
            check=True,
        )
        subprocess.run([str(exe2)], check=True)

        cpp3 = Path(tmp) / "tune.cpp"
        exe3 = Path(tmp) / ("tune.exe" if sys.platform == "win32" else "tune")
        cpp3.write_text(TUNE_HEAD + extract_tunable() + TUNE_CASES, encoding="utf-8")
        subprocess.run(
            ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
             f"-I{src}", str(cpp3), "-o", str(exe3)],
            check=True,
        )
        subprocess.run([str(exe3)], check=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
