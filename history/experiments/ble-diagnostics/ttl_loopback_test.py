"""USB-TTL 直测 JDY-23 透传桥: 从 COM 口发一串数据, 同时打印 COM 收到的任何回显.
用法: python ttl_loopback_test.py [COM] [baud] [payload]
  默认 COM6 / 9600 / "hello-JDY-TEST"
提示: 配合手机 LightBlue 连 JDY-23(FFE1 notify) 观察是否收到同样数据.
"""
import sys, time, serial

COM   = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD  = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
PAY   = sys.argv[3].encode() if len(sys.argv) > 3 else b"hello-JDY-TEST\n"

print(f"打开 {COM} @ {BAUD} ...")
ser = serial.Serial(COM, BAUD, timeout=0.5)
time.sleep(0.2)

# 1) 先发 ? 看是否回 OK (测是否在 AT 模式)
for probe in (b"AT\r\n", b"AT\r", b"\r\n"):
    print(f"\n>>> 发 {probe!r}")
    ser.write(probe)
    time.sleep(0.8)
    rx = ser.read(ser.in_waiting or 1)
    print(f"<<< 收到 {len(rx)}B: {rx!r}")
    if rx.strip():
        break

# 2) 发透传 payload
print(f"\n>>> 发透传 {PAY!r}")
ser.write(PAY)
t0 = time.time()
got = b""
while time.time() - t0 < 3:
    b = ser.read(64)
    if b:
        got += b
    else:
        time.sleep(0.1)
print(f"<<< 收到 {len(got)}B: {got!r}")

ser.close()
print("\n完成。 手机 LightBlue 若收到同样数据 => 透传桥完好; 否则串口桥不转发。")
