"""Analyze manual speed tracking without editing the source CSV or firmware."""
import argparse,csv,hashlib,io,json,statistics
from pathlib import Path
from datetime import datetime
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
parser.add_argument("source",type=Path)
parser.add_argument("output",type=Path)
args=parser.parse_args()
raw=args.source.read_bytes()
lines=[line for line in raw.decode("utf-8-sig").splitlines() if not line.startswith("#")]
reader=csv.DictReader(lines)
rows=[];rejected=0
for item in reader:
    try:
        row={k:float(v) for k,v in item.items() if v not in (None, "")}
        row["t"]=row["ms"]/1000
        row["reference"]=(row["speedError1000"]+row["speed1000"])/1000
        row["speed"]=row["speed1000"]/1000
        row["position_target"]=row["posSpdTarget1000"]/1000
        row["correction"]=row["spdAng100"]/100
        row["relative_pitch"]=row["pitch100"]/100-row["O_balanceAngleOffset"]
        rows.append(row)
    except (ValueError,TypeError,KeyError):rejected+=1
assert rows,"No valid telemetry rows"
args.output.mkdir(parents=True,exist_ok=True)
# Steady target plateaus; exclude turn and ramps. Units: count/50ms.
groups=[];previous=None
for row in rows:
    ref=row["reference"]
    key=(round(ref),row["O_balanceAngleOffset"],row["N_MAX_SPEED_ANGLE"]) if (
        abs(ref)>=3.99 and abs(ref-round(ref))<.01 and
        row["posSpdTarget1000"]==0 and abs(row["turnHz"])<1) else None
    if key!=previous and key is not None:groups.append((key,[]))
    previous=key
    if key is not None:groups[-1][1].append(row)
plateaus=[]
for key,group in groups:
    if len(group)<3:continue
    mean=lambda k:statistics.mean(r[k] for r in group)
    duration=group[-1]["t"]-group[0]["t"]
    enc1=group[-1]["enc1"]-group[0]["enc1"]
    enc2=group[-1]["enc2"]-group[0]["enc2"]
    plateaus.append(dict(target=key[0],O=key[1],N=key[2],start=group[0]["t"],end=group[-1]["t"],samples=len(group),
        mean_speed=mean("speed"),mean_correction_deg=mean("correction"),mean_relative_pitch_deg=mean("relative_pitch"),
        enc1_delta=enc1,enc2_delta=enc2,encoder_net_speed=-.05*(enc1+enc2)/2/duration if duration>0 else None))
summary=dict(source=args.source.name,source_sha256=hashlib.sha256(raw).hexdigest().upper(),source_bytes=len(raw),
    rows=len(rows),rejected_rows=rejected,mcu_start_s=rows[0]["t"],mcu_end_s=rows[-1]["t"],
    reconstructed_reference_range=[min(r["reference"] for r in rows),max(r["reference"] for r in rows)],
    actual_speed_range=[min(r["speed"] for r in rows),max(r["speed"] for r in rows)],
    correction_range_deg=[min(r["correction"] for r in rows),max(r["correction"] for r in rows)],
    outer_authority_range=[min(r["auth1000"] for r in rows)/1000,max(r["auth1000"] for r in rows)/1000],
    fall_stop_range=[min(r["fallStopCount"] for r in rows),max(r["fallStopCount"] for r in rows)],
    speed_K=sorted(set(r["K_speedKp"] for r in rows)),speed_I=sorted(set(r["I_speedKi"] for r in rows)),plateaus=plateaus,
    limitation="Reference reconstructed from speed error + measured speed; CSV does not record raw button state, applied manual speed or timeout/rearm flags.")
(args.output/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
t=[r["t"] for r in rows]
fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True,layout="constrained")
axes[0].plot(t,[r["reference"] for r in rows],label="Speed target (reconstructed)",color="#cc5500",lw=1.7)
axes[0].plot(t,[r["speed"] for r in rows],label="Measured fused speed",color="#2463a6",alpha=.7,lw=.9)
axes[0].set_ylabel("count / 50 ms")
axes[1].plot(t,[r["correction"] for r in rows],label="Speed-loop angle correction",color="#cc5500")
axes[1].plot(t,[r["relative_pitch"] for r in rows],label="Pitch minus O",color="#2463a6",alpha=.75)
axes[1].set_ylabel("degrees")
axes[2].plot(t,[r["turnHz"] for r in rows],label="Applied turn differential",color="#278046")
axes[2].set_ylabel("STEP Hz");axes[2].set_xlabel("MCU uptime (seconds)")
for ax in axes:
    ax.axhline(0,color="grey",lw=.6);ax.grid(alpha=.2);ax.legend(loc="upper right",fontsize=9)
fig.suptitle("Manual motion tracking: "+args.source.name,fontsize=12)
fig.savefig(args.output/"tracking.png",dpi=160)
plt.close(fig)
print(json.dumps(summary,ensure_ascii=False,indent=2))
