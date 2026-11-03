"""Read-only CSV evidence for the 2026-10-30 jog failure; writes report/plot only."""
from pathlib import Path
import csv,json,hashlib,argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
parser=argparse.ArgumentParser();parser.add_argument("root",type=Path);args=parser.parse_args()
root=args.root.resolve();out=root/"docs/validation/analysis/jog-instability-20261030";out.mkdir(parents=True,exist_ok=True)
sources=[];datasets=[]
for p in sorted((root/"docs/validation/recordings").glob("*20261030_09*.csv")):
    raw=p.read_bytes();allrows=list(csv.DictReader(raw.decode("utf-8-sig").splitlines()))
    rows=[]
    for r in allrows:
        if r.get("jogActive") in (None,"") or r.get("O_balanceAngleOffset") in (None,""):continue
        try:rows.append({k:float(v) for k,v in r.items() if v not in (None,"")})
        except (TypeError,ValueError):continue
    tasks=[]
    for seq in sorted({r["jogSequence"] for r in rows if r["jogActive"]==1}):
        active=[r for r in rows if r["jogSequence"]==seq and r["jogActive"]==1]
        tail=[r for r in rows if r["jogSequence"]==seq]
        tasks.append(dict(sequence=int(seq),first_ms=active[0]["ms"],last_active_ms=active[-1]["ms"],
            speed_min=min(r["speed1000"] for r in active)/1000,speed_max=max(r["speed1000"] for r in active)/1000,
            max_goal_error=max(abs(r["posError100"]) for r in active)/100,last_reason=int(tail[-1]["jogReason"])))
    resets=[]
    for prev,r in zip(rows,rows[1:]):
        if prev["positionSourceMode"]==r["positionSourceMode"]==1:
            residual=(r["pos100"]-prev["pos100"])/100+r["enc1"]-prev["enc1"]
            if abs(residual)>20:resets.append(dict(ms=r["ms"],coordinate_shift=residual))
    sources.append(dict(path=p.relative_to(root).as_posix(),sha256=hashlib.sha256(raw).hexdigest(),rows=len(rows),
        incomplete_rows=len(allrows)-len(rows),tasks=tasks,coordinate_shifts=resets));datasets.append((p,rows))
(out/"evidence.json").write_text(json.dumps(sources,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
p,rows=datasets[-1];t=[r["ms"]/1000 for r in rows]
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout="constrained")
axes[0].plot(t,[r["pos100"]/100 for r in rows],label="position (includes software re-zero)")
axes[0].plot(t,[r["jogTarget100"]/100 if r["jogActive"] else float("nan") for r in rows],label="old stepped target")
axes[0].set_ylabel("encoder counts")
axes[1].plot(t,[r["speed1000"]/1000 for r in rows],label="measured speed")
axes[1].plot(t,[r["jogSpeed1000"]/1000 if r["jogActive"] else float("nan") for r in rows],label="old jog speed request")
axes[1].set_ylabel("count / 50 ms")
axes[2].plot(t,[r["pitch100"]/100 for r in rows],label="pitch")
axes[2].plot(t,[r["target100"]/100 for r in rows],label="target pitch")
axes[2].set_ylabel("degrees");axes[2].set_xlabel("MCU uptime (s)")
for ax in axes:
    ax.legend(loc="upper left",fontsize=8);ax.grid(alpha=.25)
    for event in sources[-1]["coordinate_shifts"]:ax.axvline(event["ms"]/1000,color="red",alpha=.3,ls="--")
fig.suptitle("Recorded old jog firmware: overshoot, oscillation and coordinate resets")
fig.savefig(out/"latest-csv.png",dpi=150);plt.close(fig)
lines=["# 位置点动失稳分析与修正", "", "日期：2026-10-30，Asia/Shanghai。分析对象为用户烧录后的旧点动版本，修正版本尚未烧录。", "", "## 数据证据", ""]
for src in sources:
    name=Path(src["path"]).name
    lines += [f"- [{name}](../../recordings/{name})：{src['rows']}行完整遥测；{src['incomplete_rows']}行缺少点动字段，未用于点动结论。SHA256见evidence.json。"]
lines += ["", "最新09:19:58记录里三次点动（序号2321735576、5577、5578）均以reason=6取消，未出现到位reason=2。来源始终为编码器1（mode=1），编码器2计数为0；本段不是来源切换造成的位置跳变。", "", "第一段终点1019计数：88.903秒时位置996，89.403秒时位置1072，半秒内已越过目标53计数；此后出现正负速度交替，92.403秒记录到姿态异常取消。其他段也有往返振荡，不能将取消后的坐标归零当成全部失稳的起因。", "", "同一段在91.903→92.403秒，pos从1121变为-313，而enc1从-1362变为-1002。按mode=1实际位移约为-360计数，另有约-1074计数的坐标重设。这对应旧captureCurrentPositionTarget清零软件位置，不是原始编码器凭空跳变。2Hz遥测只能定位到相邻采样区间，不能证明单个控制周期的先后因果。", "", "![旧点动记录](latest-csv.png)", "", "## 代码中确认的风险", "", "1. 新命令直接令位置目标=当前位置±距离；只有速度斜坡，没有位置参考轨迹。连续点击又把点动速度、角度、滤波状态重置为0，造成控制切换。", "2. 点动单独使用0.08角度增益，原稳定K绝对值为0.02，相当于提高4倍；同时对已有融合速度再加0.05低通，100Hz下时间常数约195ms，增加相位滞后。这是失稳的强嫌疑，CSV不能单独量化各项贡献。", "3. 到位/停止/异常取消会重置软件里程坐标；旧点动结束还直接清零角度修正。这会造成遥测跳变和控制交接突变。", "", "较早09:18记录在首次点动前也有O参数调整与大姿态偏离，不能将该文件所有异常归因于点动。最新文件的点动段提供更直接的对应证据。fallStopCount始终为0只表示倒地保护未计数，不代表车体没有异常姿态。", "", "## 当前修正", "", "- 原始编码器与累计里程保持连续；驻车捕获只改变驻车参考，不改累计位置。", "- 终点与控制参考分离。参考速度/加速度受限；100Hz下参考位置每周期变化不超过0.6计数，反向与重复设目标不重置轨迹状态。反馈跟不上时按领先距离减速。", "- 点动复用原K/N速度环，去掉0.08私有增益与额外速度低通；点动不使用I。位置前馈与小幅跟踪校正共同生成有界速度目标。", "- 点动与驻车共享目标速度斜坡，停止/到位/取消交接不直接清零角度修正。实际电机停机故障仍允许保护立即停机。", "- 遥测新增终点与参考速度，区分点击终点、缓变参考和真实反馈。单次12秒超时、500ms心跳、5°姿态取消继续提供退出边界。", "", "## 验证边界", "", "真实main.cpp片段回归验证原K速度反馈、参考缓变、反向/停止交接与里程连续性；控制器测试覆盖理想跟踪和编码器停滞。理想跟踪器不是平衡车动力学模型，不能保证实车稳定。编译通过后仍需从短距离、有防倾倒措施的实车测试验证，不能从本次代码修改宣称已解决失稳。", ""]
(out/"report.md").write_text("\n".join(lines),encoding="utf-8")
print("Saved",out.relative_to(root),"sources",len(sources))
