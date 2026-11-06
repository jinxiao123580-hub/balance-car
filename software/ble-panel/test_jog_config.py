"""Retired position-config controls must not appear, and motion stays gated on MOTION=2."""
import os
os.environ["QT_QPA_PLATFORM"]="offscreen"
from PyQt5.QtWidgets import QApplication
import ble_tune_panel as panel
panel.BleClient.start=lambda self:None;panel.TunePanel.open_csv=lambda self:None
app=QApplication([]);w=panel.TunePanel();w.run_coro=lambda coro,label:coro.close()
assert not hasattr(w,"jog_kp") and not hasattr(w,"jog_limit") and not hasattr(w,"btn_jog_config")
# 速度档位：0.5~2.0，默认满档 2.0（车端换算 5 counts/50ms）
assert w.forward_strength.minimum()==0.5 and w.forward_strength.maximum()==2.0
assert w.forward_strength.value()==2.0 and not w.forward_strength.isEnabled()
w.on_telem("PAR: TURN=2 MOVE=4 JOG=1 JOGRAMP=1 JOGANGLE=1")
assert not w._move_supported and not w.forward_button.isEnabled()
w.on_telem("PAR: TURN=2 MOVE=5 SMOOTH=1 TILTMAX=2.00")
assert not w._move_supported and not w.forward_button.isEnabled()  # 旧开环固件不得启用
w.on_telem("JOGCFG 0.0200 0.500")
assert not w._move_supported
w.on_telem("PAR: TURN=2 MOVE=5 SMOOTH=1 MOTION=2 TILTMAX=2.00")
assert w._move_supported and w.forward_strength.isEnabled()
w.close();print("PASS: retired position-config controls removed, speed gear range 0.5-2.0, motion gated on MOTION=2")
