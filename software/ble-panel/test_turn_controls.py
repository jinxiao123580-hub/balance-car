"""Offline speed-command press/release, steering and stale-send tests. No hardware.

2026-11-06：前后由「固定±2°倾角」改为「速度档位 -> 车端速度闭环」。
旧固件（MOVE=5 但没有 MOTION=2）必须保持禁用，避免把档位当角度用。
"""
import os
os.environ["QT_QPA_PLATFORM"]="offscreen"
import asyncio,time,io,csv
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt,QEvent
from PyQt5.QtTest import QTest
import ble_tune_panel as panel
panel.BleClient.start=lambda self:None;panel.TunePanel.open_csv=lambda self:None
app=QApplication([]);w=panel.TunePanel();w.run_coro=lambda coro,label:coro.close();w.show();app.processEvents()
w.ble.client=type("Connected",(),{"is_connected":True})()
for cap in ("TURN=2 MOVE=3","TURN=2 MOVE=4 JOG=1 JOGRAMP=1 JOGANGLE=1",
            "TURN=2 MOVE=5 SMOOTH=1 TILTMAX=2.00"):
    w.on_telem("PAR: "+cap);assert not w.forward_button.isEnabled(),cap
w.on_telem("PAR: TURN=2 MOVE=5 SMOOTH=1 MOTION=2 TILTMAX=2.00")
assert not w.forward_button.isEnabled() # need a fresh telemetry frame
assert w._motion_closed_loop and w.forward_strength.isEnabled()
values=["0"]*len(panel.TELEM_COLS)
w.car_param_val["O"]=7.5   # 平衡点来自 PAR 回显，不在 D 帧里
for key,value in {"manualControlMode":5,"manualReq1000":2000,"manualApplied1000":125,
                  "manualAngleRate1000":750,"manualResume1000":0,
                  "target100":780,"speed1000":250}.items():
    values[panel.TELEM_COLS.index(key)]=str(value)
w.on_telem("D,"+",".join(values))
assert w.forward_button.isEnabled()
assert "车端倾角 +0.30°" in w.jog_apply_status.text(), w.jog_apply_status.text()
assert "实测速度 0.2" in w.jog_apply_status.text(), w.jog_apply_status.text()
assert w.forward_strength.value()==2 and w.forward_strength.isEnabled()
w.turn_enable.setChecked(True)
QTest.mousePress(w.forward_button,Qt.LeftButton);assert w._forward_value==2 and w.motion_command()=="^ 0,2"
QTest.mouseRelease(w.forward_button,Qt.LeftButton);assert w._forward_value==0 and w.motion_command()=="^ 0,0"
QTest.mousePress(w.backward_button,Qt.LeftButton);assert w.motion_command()=="^ 0,-2"
QTest.mouseRelease(w.backward_button,Qt.LeftButton);assert w._forward_value==0
# 档位不是角度：半档只发 1，车端按 1/2×5=2.5 counts/50ms 解释
w.forward_strength.setValue(1.0)
QTest.mousePress(w.forward_button,Qt.LeftButton);assert w.motion_command()=="^ 0,1"
QTest.mouseRelease(w.forward_button,Qt.LeftButton)
w.forward_strength.setValue(2.0)
w.forward_reverse.setChecked(True);QTest.mousePress(w.forward_button,Qt.LeftButton);assert w._forward_value==-2
QTest.mouseRelease(w.forward_button,Qt.LeftButton)
QTest.mousePress(w.turn_left,Qt.LeftButton);assert w.motion_command()=="^ -150,0"
QTest.mouseRelease(w.turn_left,Qt.LeftButton);assert w.motion_command()=="^ 0,0"
w.start_forward(1);w.eventFilter(app,QEvent(QEvent.ApplicationDeactivate));assert w._forward_value==0 and not w.turn_enable.isChecked()
w.turn_enable.setChecked(True);w._last_telemetry=time.monotonic();w.start_forward(1)
w._last_telemetry=0;w.turn_tick();assert w._forward_value==0 and not w.turn_enable.isChecked()
for count in (49,56,57,63,65,71,74):
    output=io.StringIO();w.csv_f=output;w.csv_w=csv.writer(output);w.csv_recording=True
    w.record_line("D,"+",".join("0" for _ in range(count)))
    row=next(csv.reader(io.StringIO(output.getvalue())));assert len(row)==len(panel.CSV_HEADERS)
    assert row[1:count+1]==["0"]*count
    assert row[1+count:1+len(panel.TELEM_COLS)]==[""]*(len(panel.TELEM_COLS)-count)
    w.csv_recording=False;w.csv_f=None;w.csv_w=None
async def queue_check():
    b=panel.BleClient();sent=[]
    class Client:
        is_connected=True
        async def write_gatt_char(self,h,v,**kw):sent.append(v)
    b.client=Client();b.handle=1;b._send_lock=asyncio.Lock();await b._send_lock.acquire()
    w._forward_value=2;task=asyncio.create_task(b.send(w.motion_command));await asyncio.sleep(0)
    w.stop_motion();b._send_lock.release();assert await task;assert sent==[b"^ 0,0\n"]
asyncio.run(queue_check());w.close()
print("PASS: MOTION=2 gating (old fixed-angle firmware stays disabled), speed gears, press/release, reverse, steering, focus/stale telemetry stop, latest queued stop and 49/56/57/63/65/71/74 CSV fields")

