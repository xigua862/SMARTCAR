import serial, time, sys
s = serial.Serial("COM11", 115200, timeout=0.2)
time.sleep(0.3)
s.reset_input_buffer()
s.write(b"Z\r\n")
time.sleep(0.3)
s.write(b"Z\r\n")
time.sleep(0.3)
s.write(b"Z\r\n")
time.sleep(0.6)
s.write(b"O\r\n")
t = time.time()
got = []
while time.time() - t < 2.5:
    line = s.readline().decode(errors="ignore").strip()
    if "ODOM" in line or line.startswith("P "):
        got.append(line)
s.close()
for g in got[:6]:
    print(g)
print("total lines seen =", len(got))
