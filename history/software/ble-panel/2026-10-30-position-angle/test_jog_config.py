"""Offline regression for dirty inputs, actual car confirmation and stale BLE writes."""
import os
os.environ["QT_QPA_PLATFORM"]="offscreen"
import asyncio,time
from PyQt5.QtWidgets import QApplication
import ble_tune_panel as panel
panel.BleClient.start=lambda self:None
panel.TunePanel.open_csv=lambda self:None
app=QApplication([]);w=panel.TunePanel();coroutines=[]
w.run_coro=lambda coro,label:coroutines.append(coro)
w.ble.client=type("Connected",(),{"is_connected":True})()
w.on_telem("PAR: TURN=2 MOVE=4 JOG=1 JOGRAMP=1 JOGANGLE=1 N=2")
for c in coroutines:c.close()
coroutines.clear()
w._jog_pending=False;w._last_telemetry=time.monotonic();w._cfg_lean=0
w.on_telem("JOGCFG 0.0050 0.300")
assert w.forward_button.isEnabled() and not w.btn_jog_config.isEnabled()
w.jog_limit.setValue(.5);w.jog_kp.setValue(.02)
assert not w.forward_button.isEnabled() and w.btn_jog_config.isEnabled()
assert "未应用" in w.jog_apply_status.text() and "0.30" in w.jog_config_echo.text()
w.start_forward(1);assert not w._jog_pending # stale car values cannot start a new target
w._jog_active=True;w._refresh_jog_config_controls();w.send_jog_config()
assert not w._cfg_pending and not w.btn_jog_config.isEnabled() and "先停止" in w.jog_apply_status.text()
w._jog_active=False;w._cfg_lean=.02;w._refresh_jog_config_controls()
assert not w.btn_jog_config.isEnabled() and "回零" in w.jog_apply_status.text()
w._cfg_lean=0;w._refresh_jog_config_controls()
sent=[]
async def ok_send(text):
    value=text() if callable(text) else text
    if value:sent.append(value)
    return bool(value)
w.ble.send=ok_send
w.send_jog_config();assert w._cfg_pending and not w.forward_button.isEnabled()
asyncio.run(coroutines.pop())
assert sent==["! 0.0200,0.500"] and w._cfg_pending # a GATT write is not car confirmation
assert "等待车端确认" in w.jog_apply_status.text() and "0.30" in w.jog_config_echo.text()
w.on_telem("JOGCFG 0.0200 0.500")
assert not w._cfg_pending and w.forward_button.isEnabled() and "已生效" in w.jog_apply_status.text()
# Timeout keeps the real car echo and allows retry; a late matching echo reconciles state.
w.jog_limit.setValue(.4);w.send_jog_config()
for c in coroutines:c.close()
coroutines.clear();w._cfg_deadline=time.monotonic()-1;w.turn_tick()
assert not w._cfg_pending and "超时" in w.jog_apply_status.text() and "0.50" in w.jog_config_echo.text()
w.on_telem("JOGCFG 0.0200 0.400")
assert w._jog_config_ready() and "已生效" in w.jog_apply_status.text()
# BLE failure and an old firmware diagnostic response must be visible, never marked applied.
w.jog_limit.setValue(.5)
async def fail_send(text):return False
w.ble.send=fail_send;w.send_jog_config();asyncio.run(coroutines.pop())
assert not w._cfg_pending and "写入失败" in w.jog_apply_status.text() and not w.forward_button.isEnabled()
w.ble.send=ok_send;w.send_jog_config()
for c in coroutines:c.close()
coroutines.clear();w.on_telem("TMC STEP/DIR standalone: hardware-selected mode")
assert "旧诊断" in w.jog_apply_status.text() and not w._cfg_pending
# A timed-out config waiting on the serialized BLE lock must not be sent later.
async def stale_queue():
    b=panel.BleClient();packets=[]
    class Client:
        is_connected=True
        async def write_gatt_char(self,h,value):packets.append(value)
    b.client=Client();b.handle=1;b._send_lock=asyncio.Lock();w.ble=b
    w._last_telemetry=time.monotonic();w._cfg_lean=0;w._jog_pending=False
    w.send_jog_config()
    coroutine=coroutines.pop();await b._send_lock.acquire()
    task=asyncio.create_task(coroutine);await asyncio.sleep(0)
    w._cfg_deadline=time.monotonic()-1;w.turn_tick()
    b._send_lock.release();await task
    assert packets==[] and not w._cfg_pending
asyncio.run(stale_queue())
# Reconnection clears old confirmation; the user must get a new car echo and telemetry.
w.on_msg("已连接 ✓")
assert w._cfg_car is None and not w.forward_button.isEnabled()
for c in coroutines:c.close()
coroutines.clear()
w.close()
print("PASS: dirty input interlock, motion/release guard, write vs ACK, matching/late ACK, failure/timeout, legacy firmware and stale queue cancellation")
