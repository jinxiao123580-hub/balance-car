"""Read-only analysis of the smooth-jog trial; source CSV is never modified."""
from pathlib import Path
import argparse,csv,hashlib,json,statistics,collections
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
parser=argparse.ArgumentParser();parser.add_argument("source",type=Path);parser.add_argument("output",type=Path);args=parser.parse_args()
raw=args.source.read_bytes();allrows=list(csv.DictReader(raw.decode("utf-8-sig").splitlines()));rows=[]
for item in allrows:
    if item.get("jogReferenceSpeed1000") in (None,"") or item.get("O_balanceAngleOffset") in (None,""):continue
    rows.append({k:float(v) for k,v in item.items() if v not in (None,"")})
assert rows
out=args.output;out.mkdir(parents=True,exist_ok=True);tasks=[]
for seq in sorted({r["jogSequence"] for r in rows if r["jogActive"]==1}):
    group=[r for r in rows if r["jogSequence"]==seq];active=[r for r in group if r["jogActive"]==1]
    tasks.append(dict(sequence=int(seq),active_rows=len(active),first_ms=active[0]["ms"],last_ms=active[-1]["ms"],
        final_reason=int(group[-1]["jogReason"]),goal=active[-1]["jogGoal100"]/100,
        first_position=active[0]["pos100"]/100,last_active_position=active[-1]["pos100"]/100,
        mean_speed_request=statistics.mean(r["posSpdTarget1000"]/1000 for r in active),
        mean_speed_feedback=statistics.mean(r["speed1000"]/1000 for r in active),
        mean_request_angle_contribution=statistics.mean(-r["K_speedKp"]*r["posSpdTarget1000"]/1000 for r in active),
        reference_paused_rows=sum(abs(r["jogReferenceSpeed1000"])<=1 for r in active),
        max_abs_angle_correction=max(abs(r["spdAng100"]) for r in active)/100,
        motor_limit_rows=sum(abs(r["m1Hz"])>=1750 or abs(r["m2Hz"])>=1750 for r in active)))
residual=[abs((b["pos100"]-a["pos100"])/100+b["enc1"]-a["enc1"]) for a,b in zip(rows,rows[1:]) if b["ms"]>a["ms"]]
summary=dict(source=args.source.name,sha256=hashlib.sha256(raw).hexdigest(),complete_rows=len(rows),skipped_rows=len(allrows)-len(rows),
    encoder_sources=dict(collections.Counter(int(r["positionSourceMode"]) for r in rows)),coordinate_residual_max=max(residual),
    final_reasons=dict(collections.Counter(t["final_reason"] for t in tasks)),tasks=tasks)
(out/"evidence.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
# Show a complete, quiet-O timeout case instead of the initial protection event.
seq=2323534121;selected=[r for r in rows if r["jogSequence"]==seq];t=[r["ms"]/1000 for r in selected]
fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True,layout="constrained")
axes[0].plot(t,[r["pos100"]/100 for r in selected],label="actual position")
axes[0].plot(t,[r["jogTarget100"]/100 for r in selected],label="smooth reference / parking target")
axes[0].plot(t,[r["jogGoal100"]/100 for r in selected],"--",label="clicked endpoint")
axes[0].set_ylabel("encoder counts")
axes[1].plot(t,[r["speed1000"]/1000 for r in selected],label="measured speed")
axes[1].plot(t,[r["posSpdTarget1000"]/1000 for r in selected],label="jog speed request")
axes[1].set_ylabel("count / 50 ms")
axes[2].plot(t,[r["spdAng100"]/100 for r in selected],label="total speed-loop angle correction")
axes[2].plot(t,[-r["K_speedKp"]*r["posSpdTarget1000"]/1000 for r in selected],label="contribution of jog request")
axes[2].set_ylabel("degrees");axes[2].set_xlabel("MCU uptime (s)")
for ax in axes:ax.legend(loc="upper left",fontsize=8);ax.grid(alpha=.25)
fig.suptitle("Smooth-jog trial: reference pauses, little net tracking, then timeout")
fig.savefig(out/"tracking.png",dpi=150);plt.close(fig)
lines=["# 平滑点动实车复测分析", "", "日期：2026-10-30，Asia/Shanghai。用户反馈：基本没有前后移动，主要晃动。", "", "## 数据与版本", "", f"来源：[09:48:16记录](../../recordings/{args.source.name})，{len(rows)}行完整遥测，{len(allrows)-len(rows)}行重连/不完整记录排除；原始文件SHA256见evidence.json。新增jogGoal100与jogReferenceSpeed1000存在，说明65字段的平滑点动版本已在工作。全部使用编码器1；本次位置变化与原始编码器增量的差额最大2计数、典型为0，上一版大幅软件坐标归零未再出现。", "", "## 结论", "", "平滑修改已经执行，但它没有建立可靠的前后位置跟踪。9个出现活动数据的任务里，6个以12秒超时结束，1个姿态异常取消，1个心跳丢失取消，1个被新任务替换时仍在运行；没有到位reason=2。本次全部活动采样里，电机命令均未撞到1750Hz限制，不能靠放开L解释当前不移动。", "", "代码把参考领先预算限制为10计数，反馈跟不上就把参考速度降到0；同时把跟踪误差钳为±10，乘0.1后只剩±1计数/50ms速度请求。以K=-0.02、speedLoopDirection=-1计算，零速度下该请求只提供±0.02°的倾角修正。位置误差继续增大也不增加这部分作用。", "", "例如任务2323534121，24个活动采样中19个的参考速度为0，平均速度请求约+0.982计数/50ms，附加角度贡献约+0.0196°；实际位置在133～182计数之间晃动，终点272计数没有追上。图中总角度修正的正负变化主要含速度反馈阻尼，不等于面板给出了同等大小的前进倾角。", "", "采样仅2Hz，逐行速度均值与位置差分会有采样偏差；本结论使用任务结果、参考暂停比例和代码限幅共同判断，不用单个瞬时速度判定方向。原地不动的具体物理阈值（静摩擦、机械平衡点、驱动电流等）尚未通过独立试验确认，不能仅凭CSV量化其贡献。", "", "![位置跟踪、速度与倾角贡献](tracking.png)", "", "## 任务统计", "", "| 序号 | 活动采样 | 参考速度为0 | 平均速度请求 | 请求角度贡献 | 最终状态 |", "| --- | ---: | ---: | ---: | ---: | --- |"]
labels={1:"被后续任务替换",2:"到位",3:"停止",4:"心跳超时",5:"来源切换",6:"保护取消",7:"12秒超时"}
for a in tasks:lines.append(f"| {a['sequence']} | {a['active_rows']} | {a['reference_paused_rows']} | {a['mean_speed_request']:.3f} | {a['mean_request_angle_contribution']:.4f}° | {labels[a['final_reason']]} |")
lines += ["", "速度单位为计数/50ms；角度贡献按当前K计算，未将速度阻尼项混入。数据参数列来自面板本地快照，实际速度P贡献也与K=-0.02一致。", "", "## 后续控制调整方向", "", "保留连续位置反馈与平滑参考，重新设计前后命令的控制作用。可用独立、受限的位置误差倾角P通道驱动位移，保留原速度反馈作阻尼，不使用积分；将运动位置作用与稳定用速度K分开调试。其限幅、斜坡和实际有效量需要独立小范围试验，不能仅从现有CSV确定一组保证能动的增益。", "", "到位判定也需要按原地晃动幅度制定：当前±2计数且瞬时速度≤1、保持200ms可能过严。应以一段时间的位置中心和净速度判断到位，原始编码器反馈继续保留。该项不能单独解释大部分任务根本未追上终点，但会影响已接近终点的任务是否完成。", "", "本报告不修改固件、不烧录；当前已知平滑版本仍未通过前后移动验收。", ""]
(out/"report.md").write_text("\n".join(lines),encoding="utf-8")
assert hashlib.sha256(args.source.read_bytes()).hexdigest()==summary["sha256"]
print('Read-only analysis saved:',out)
