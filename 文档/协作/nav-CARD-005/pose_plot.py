# -*- coding: utf-8 -*-
# CARD-005 推车画图：串口收 P 行 -> CSV + PNG
# 用法: python _tmp/pose_plot.py <名字> <秒数>
#   python _tmp/pose_plot.py run1 90   -> _tmp/pose_run1.csv / _tmp/pose_run1.png
import sys, os, math, time, csv, re

import serial

NAME    = sys.argv[1] if len(sys.argv) > 1 else "run1"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
PORT    = "COM11"
BAUD    = 115200

OUT_CSV = os.path.join("_tmp", "pose_%s.csv" % NAME)
OUT_PNG = os.path.join("_tmp", "pose_%s.png" % NAME)

pat = re.compile(r"^P x=(-?\d+) y=(-?\d+) yaw=([+-]?\d+\.\d)")

def main():
    pts = []  # (t_ms, x, y, yaw)
    t0 = time.time()
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    time.sleep(0.3)
    ser.reset_input_buffer()
    print("collecting %s s from %s ..." % (SECONDS, PORT))
    while time.time() - t0 < SECONDS:
        try:
            line = ser.readline().decode("ascii", "ignore").strip()
        except Exception:
            continue
        m = pat.match(line)
        if m:
            t_ms = int((time.time() - t0) * 1000)
            pts.append((t_ms, int(m.group(1)), int(m.group(2)), float(m.group(3))))
    ser.close()

    if len(pts) < 2:
        print("NO P LINES collected (%d)" % len(pts))
        return

    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "x_mm", "y_mm", "yaw_deg"])
        for p in pts:
            w.writerow(p)

    xs = [p[1] for p in pts]
    ys = [p[2] for p in pts]
    # 路径总长
    path_len = 0.0
    for i in range(1, len(pts)):
        path_len += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    # 闭合差 = 起终点直线距离
    closure = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
    ratio = (closure / path_len * 100.0) if path_len > 0 else 0.0
    print("points=%d path=%.0fmm closure=%.0fmm (%.2f%%)" % (len(pts), path_len, closure, ratio))
    print("start=(%d,%d) end=(%d,%d)" % (xs[0], ys[0], xs[-1], ys[-1]))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.figure(figsize=(8, 8))
        plt.plot(xs, ys, "-", lw=1.2, color="tab:blue")
        plt.plot(xs[0], ys[0], marker="*", markersize=16, color="red", linestyle="None", label="start")
        plt.plot(xs[-1], ys[-1], marker="o", markersize=8, color="black", linestyle="None", label="end")
        plt.axis("equal")
        plt.grid(True, alpha=0.3)
        plt.title("%s  pts=%d path=%.0fmm closure=%.0fmm(%.2f%%)" % (NAME, len(pts), path_len, closure, ratio))
        plt.xlabel("x mm (head)"); plt.ylabel("y mm (right)")
        plt.legend()
        plt.savefig(OUT_PNG, dpi=110)
        print("saved %s / %s" % (OUT_CSV, OUT_PNG))
    except Exception as e:
        print("matplotlib failed (%s), CSV only: %s" % (e, OUT_CSV))
        # 终端 ASCII 简图
        W, H = 60, 24
        x0, x1 = min(xs), max(xs); y0, y1 = min(ys), max(ys)
        sx = (W - 1) / max(1, (x1 - x0)); sy = (H - 1) / max(1, (y1 - y0))
        grid = [[" "] * W for _ in range(H)]
        for x, y in zip(xs, ys):
            cx = int((x - x0) * sx); cy = int((y - y0) * sy)
            grid[H - 1 - cy][cx] = "#"
        grid[H - 1 - int((ys[0] - y0) * sy)][int((xs[0] - x0) * sx)] = "S"
        grid[H - 1 - int((ys[-1] - y0) * sy)][int((xs[-1] - x0) * sx)] = "E"
        for row in grid:
            print("".join(row))

if __name__ == "__main__":
    main()
