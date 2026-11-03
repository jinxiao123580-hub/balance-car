"""Retired position-config controls must not appear in the fixed-angle mode."""
import os
os.environ["QT_QPA_PLATFORM"]="offscreen"
from PyQt5.QtWidgets import QApplication
import ble_tune_panel as panel
panel.BleClient.start=lambda self:None;panel.TunePanel.open_csv=lambda self:None
app=QApplication([]);w=panel.TunePanel();w.run_coro=lambda coro,label:coro.close()
assert not hasattr(w,"jog_kp") and not hasattr(w,"jog_limit") and not hasattr(w,"btn_jog_config")
assert w.forward_strength.minimum()==w.forward_strength.maximum()==2
w.on_telem("PAR: TURN=2 MOVE=4 JOG=1 JOGRAMP=1 JOGANGLE=1")
assert not w._move_supported and not w.forward_button.isEnabled()
w.on_telem("JOGCFG 0.0200 0.500")
assert not w._move_supported
w.close();print("PASS: retired position-config controls removed, legacy JOGCFG cannot enable fixed-angle drive")
