"""Read-only analysis with explicit edge trimming. Run from repository root."""
import csv,json,hashlib,datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
SOURCE=Path('docs/validation/recordings/ble_tune_log_20261031_101235_473_p32752.csv')
OUT=Path('docs/validation/analysis/long-balance-20261031-101235')
with SOURCE.open(encoding='utf-8-sig',newline='') as f: raw=list(csv.DictReader(f))
rows=[r for r in raw if None not in r and all(v not in (None,'') for v in r.values())]
origin=float(rows[0]['ms'])
def sec(r):return (float(r['ms'])-origin)/1000

def segment(start,end):
 g=[r for r in rows if start<=sec(r)<=end]
 def a(k,scale=1):return np.array([float(r[k])*scale for r in g])
 def stat(x):return dict(mean=float(x.mean()),std=float(x.std()),min=float(x.min()),max=float(x.max()),rms=float(np.sqrt(np.mean(x*x))),abs_p95=float(np.percentile(abs(x),95)),abs_p99=float(np.percentile(abs(x),99)))
 error=a('pitch100',.01)-a('O_balanceAngleOffset')
 return dict(start_relative_s=sec(g[0]),end_relative_s=sec(g[-1]),duration_s=sec(g[-1])-sec(g[0]),samples=len(g),start_local=datetime.datetime.fromtimestamp(float(g[0]['time_s']),ZoneInfo('Asia/Shanghai')).isoformat(),end_local=datetime.datetime.fromtimestamp(float(g[-1]['time_s']),ZoneInfo('Asia/Shanghai')).isoformat(),max_sample_gap_ms=float(np.diff(a('ms')).max()),pitch_deviation_deg=stat(error),target_tracking_error_deg=stat(a('pitch100',.01)-a('target100',.01)),within_0p5_deg_pct=float(np.mean(abs(error)<=.5)*100),within_1_deg_pct=float(np.mean(abs(error)<=1)*100),final_step_hz=stat(a('finalHz')),final_step_at_limit_samples=int(np.sum(abs(a('finalHz'))>=a('L_limitHz'))),raw_pd_window_peak_hz=stat(a('balHzAbsMax')),raw_pd_over_step_limit_pct=float(np.mean(a('balHzAbsMax')>=a('L_limitHz'))*100),position_first_count=float(a('pos100',.01)[0]),position_last_count=float(a('pos100',.01)[-1]),position_min_count=float(a('pos100',.01).min()),position_max_count=float(a('pos100',.01).max()),parking_target_values=sorted(set(a('jogTarget100',.01))),position_error_counts=stat(a('posError100',.01)),return_speed_at_cap_pct=float(np.mean(abs(a('posSpdTarget1000',.001))>=a('Y_maxPositionReturnSpeed')-.001)*100),speed_counts_per_50ms=stat(a('speed1000',.001)),fall_count_first=int(a('fallStopCount')[0]),fall_count_last=int(a('fallStopCount')[-1]))
OUT.mkdir(parents=True,exist_ok=True)
metrics=dict(source=SOURCE.as_posix(),sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),parsed_records=len(raw),complete_records=len(rows),excluded_malformed_records=len(raw)-len(rows),trim_method='Discard first 60s, and all records from first large terminal tilt at relative6481s. Keep every valid row inside this window, including disturbances and manual tests.',middle=segment(60,6480.5),fixed_parameters_no_manual=segment(802.5,6066),quiet_after_manual=segment(6120,6480.5))
(OUT/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in metrics.items() if k!='source'},ensure_ascii=False,indent=2))
