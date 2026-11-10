"""参数持久化面板侧回归：SAVED= 提示、按钮启用、SAVE/LOAD/DEF 命令确实发出去。

2026-11-10：车端把 22 项参数存进内部 flash，面板负责"保存到车端/从车端读回/
恢复出厂值"三个按钮与未保存提示。用假 send 验证，不接硬件。
"""
import os
os.environ["QT_QPA_PLATFORM"] = "offscreen"
import asyncio
from PyQt5.QtWidgets import QApplication
import ble_tune_panel as panel

panel.BleClient.start = lambda self: None
panel.TunePanel.open_csv = lambda self: None

app = QApplication([])
w = panel.TunePanel()
sent = []


async def fake_send(text):
    sent.append(text)
    return True


w.ble.client = type("Connected", (), {"is_connected": True})()
w.ble.send = fake_send
w.run_coro = lambda coro, label: asyncio.run(coro)   # 直接跑完，不依赖 BLE 线程

# 1) 旧固件（PAR 里没有 SAVED=）：按钮必须禁用，不能让人以为能存
w.on_telem("PAR: TURN=2 MOVE=5 SMOOTH=1 MOTION=2 FW=20261109 TILTMAX=2.00")
assert not w._param_persist_supported
assert not w.btn_save.isEnabled() and not w.btn_load.isEnabled() and not w.btn_def.isEnabled()
assert "无 SAVE" in w.save_state.text(), w.save_state.text()

# 2) 新固件、参数与 flash 记录一致 -> "已存车端"
w.on_telem("PAR: TURN=2 MOVE=5 SMOOTH=1 MOTION=2 FW=20261110 TILTMAX=2.00 "
           "P=1000.00 D=100.00 O=4.00 M=12.00 SAVED=1")
assert w._param_persist_supported and w._param_saved
assert w.btn_save.isEnabled() and w.btn_load.isEnabled() and w.btn_def.isEnabled()
assert "已存车端" in w.save_state.text(), w.save_state.text()

# 3) 改过没存 -> 橙色提示
w.on_telem("PAR: TURN=2 MOVE=5 SMOOTH=1 MOTION=2 FW=20261110 TILTMAX=2.00 "
           "P=1000.00 D=100.00 O=4.00 M=12.00 SAVED=0")
assert not w._param_saved
assert "未保存" in w.save_state.text(), w.save_state.text()

# 4) SAVED= 不能被当成字母参数塞进 car_param_val
assert "SAVED" not in w.car_param_val
assert w.car_param_val.get("O") == 4.0

# 5) 三个按钮发出整词命令，并在写 flash 之后补问一次 ?
sent.clear()
w.do_save()
assert sent == ["SAVE", "?"], sent
sent.clear()
w.do_load()
assert sent == ["LOAD", "?"], sent
sent.clear()
w.do_def()
assert sent == ["DEF", "?"], sent

w.close()
print("PASS: SAVED= 提示与三按钮命令（旧固件禁用、未保存橙色、SAVE/LOAD/DEF 拼装）")
