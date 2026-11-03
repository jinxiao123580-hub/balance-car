"""Offline GUI and latest-command tests; no real BLE or CSV recording."""
import os
os.environ["QT_QPA_PLATFORM"]="offscreen"
import asyncio,time,io,csv
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt,QEvent
from PyQt5.QtTest import QTest
import ble_tune_panel as panel
panel.BleClient.start=lambda self:None
panel.TunePanel.open_csv=lambda self:None
app=QApplication([]);w=panel.TunePanel()
w.run_coro=lambda coro,label:coro.close()
w.show();app.processEvents()
w.ble.client=type("Connected",(),{"is_connected":True})()
assert not w.turn_enable.isEnabled()
for cap in ("TURN=2 MOVE=1","TURN=2 MOVE=2","TURN=2 MOVE=3","TURN=2 MOVE=4 JOG=1","TURN=2 MOVE=4 JOG=1 JOGRAMP=1"):
    w.on_telem(f"PAR: {cap} N=2")
    assert not w.forward_button.isEnabled()
w.on_telem("PAR: TURN=2 MOVE=4 JOG=1 JOGRAMP=1 JOGANGLE=1 N=2")
assert not w.forward_button.isEnabled() and w.forward_strength.value()==50
assert w.forward_strength.minimum()==5 and w.forward_strength.maximum()==200
assert w.jog_kp.value()==.005 and w.jog_limit.value()==.30
assert w.jog_config_command()=="! 0.0050,0.300"
w.on_telem("JOGCFG 0.0080 0.250")
assert "0.0080" in w.jog_config_echo.text() and w.jog_kp.value()==.005
diagnostics=["0"]*len(panel.TELEM_COLS)
for key,value in {"jogSequence":w._jog_seq,"jogLeanApplied1000":125,"jogKp1000000":5000,"jogLimit1000":300}.items():
    diagnostics[panel.TELEM_COLS.index(key)]=str(value)
w.on_telem("D,"+",".join(diagnostics))
assert "+0.125" in w.jog_state_label.text() and "0.0050" in w.jog_config_echo.text()
assert w.forward_button.isEnabled() and "已生效" in w.jog_apply_status.text()
w.turn_enable.setChecked(True);w._last_telemetry=time.monotonic()
w._jog_pending=False
QTest.mousePress(w.forward_button,Qt.LeftButton)
assert not w._jog_pending # pressing alone does not send continuous inclination
QTest.mouseRelease(w.forward_button,Qt.LeftButton)
seq=w._jog_seq
assert w._jog_pending and w.motion_command()==f"~ {seq},50"
w.on_telem(f"JOG {seq} 1 1")
assert w._jog_active and not w._jog_pending and w.motion_command()==f"& {seq}"
QTest.mouseClick(w.backward_button,Qt.LeftButton)
assert w._jog_seq!=seq and w._jog_delta==-50
old=w._jog_seq
QTest.mouseClick(w.turn_stop,Qt.LeftButton)
assert w._jog_delta==0 and w._jog_pending and not w._jog_active
stop=w._jog_seq
assert w.motion_command()==f"~ {stop},0"
w.on_telem(f"JOG {old} 1 1")
assert not w._jog_active # delayed ACK cannot re-enable canceled movement
w.on_telem(f"JOG {stop} 0 3")
assert not w._jog_pending and w.motion_command()=="V 0"
w.forward_reverse.setChecked(True)
QTest.mouseClick(w.forward_button,Qt.LeftButton)
assert w._jog_delta==-50
w.eventFilter(app,QEvent(QEvent.ApplicationDeactivate))
assert w._jog_delta==0 and not w._jog_active and not w.turn_enable.isChecked()
w.on_telem(f"JOG {w._jog_seq} 0 3")
w.turn_enable.setChecked(True);w._last_telemetry=time.monotonic()
QTest.mousePress(w.turn_left,Qt.LeftButton)
assert w._turn_value==-150 and w.motion_command()=="V -150"
QTest.mouseRelease(w.turn_left,Qt.LeftButton)
assert w._turn_value==0
QTest.mouseClick(w.forward_button,Qt.LeftButton)
w._last_telemetry=0;w.turn_tick()
assert w._jog_delta==0 and not w.turn_enable.isChecked()
for count in (49,56,57,63,65,71):
    output=io.StringIO();w.csv_f=output;w.csv_w=csv.writer(output);w.csv_recording=True
    w.record_line("D,"+",".join("0" for _ in range(count)))
    row=next(csv.reader(io.StringIO(output.getvalue())))
    assert len(row)==len(panel.CSV_HEADERS)
    assert row[1:1+count]==["0"]*count
    assert row[1+count:1+len(panel.TELEM_COLS)]==[""]*(len(panel.TELEM_COLS)-count)
    w.csv_recording=False;w.csv_f=None;w.csv_w=None
async def queue_check():
    b=panel.BleClient();sent=[]
    class Client:
        is_connected=True
        async def write_gatt_char(self,h,v):sent.append(v)
    b.client=Client();b.handle=1;b._send_lock=asyncio.Lock()
    await b._send_lock.acquire()
    w._jog_pending=True;w._jog_delta=50
    task=asyncio.create_task(b.send(w.motion_command));await asyncio.sleep(0)
    w.stop_motion();latest=w.motion_command()
    b._send_lock.release();assert await task
    assert sent==[(latest+"\n").encode()] and latest.endswith(",0")
asyncio.run(queue_check())
w.close()
print("PASS: click jog, capability, target replacement, STOP/stale ACK, reverse, steering, focus/telemetry, latest queued stop, 49/56/57/63/65/71 fields, independent config echo")
