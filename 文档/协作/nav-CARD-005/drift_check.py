import serial, time
s = serial.Serial("COM11", 115200, timeout=0.2)
time.sleep(0.3)
s.reset_input_buffer()
t = time.time()
rows = []
while time.time() - t < 12:
    line = s.readline().decode(errors="ignore").strip()
    if line.startswith("P "):
        p = line.split()
        rows.append((time.time() - t, p[1], p[2], p[3]))
s.close()
print("samples =", len(rows))
print("first:", rows[0])
print("mid  :", rows[len(rows) // 2])
print("last :", rows[-1])
