 #!/usr/bin/env python3
"""
Real-time serial plotter for Movement Tracker firmware.

Parses lines of the form:
    classic x:<v> y:<v>| dx:<v> dy:<v> | raw_mm:(<v>,<v>) | raw_cpi:(<v>,<v>) |
    yaw_gyro:<v> yaw_sflp:<v> [yaw_fused:<v>] | gyro_xyz:(<v>,<v>,<v>)dps
    [| gyro_raw_z:<v>dps]

Also supports older firmware lines that include a kalman/bias block.

Notes:
  - Supports both the older verbose firmware line and the newer compact line.
  - Missing optional fields are carried forward from their previous values.

Usage:
  pip install pyserial matplotlib
  python plot_serial.py
"""

import re
import threading
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation

try:
    import serial as pyserial
except Exception as e:
    pyserial = None
    SERIAL_IMPORT_ERROR = e
else:
    SERIAL_IMPORT_ERROR = None
    if not hasattr(pyserial, "Serial"):
        SERIAL_IMPORT_ERROR = RuntimeError(
            "Imported module named 'serial' is not pyserial."
        )

PORT = "/dev/tty.usbmodem1302"
BAUD = 115200
HISTORY = 500  # number of samples to show

# ── regex ────────────────────────────────────────────────────────────────────
LINE_RE = re.compile(
    r"classic x:(?P<cx>[-\d.]+) y:(?P<cy>[-\d.]+)"
    r"(?:\s*\|\s*kalman x:(?P<kx>[-\d.]+)"
    r"(?: y:(?P<ky>[-\d.]+) vx:(?P<kvx>[-\d.]+) vy:(?P<kvy>[-\d.]+) mm/s)?"
    r"(?:\s*\|\s*bias ax:(?P<bax>[-\d.]+) ay:(?P<bay>[-\d.]+))?"
    r")?"
    r"\s*\|\s*dx:(?P<dx>[-\d]+) dy:(?P<dy>[-\d]+)"
    r"\s*\|\s*raw_mm:\((?P<rxmm>[-\d.]+),(?P<rymm>[-\d.]+)\)"
    r"(?:\s*\|\s*raw_cpi:\((?P<rcx>[-\d]+),(?P<rcy>[-\d]+)\))?"
    r"\s*\|\s*yaw_gyro:(?P<yg>[-\d.]+) yaw_sflp:(?P<ys>[-\d.]+)(?: yaw_fused:(?P<yf>[-\d.]+))?"
    r"(?:\s*\|\s*gyro_xyz:\((?P<gx>[-\d.]+),(?P<gy>[-\d.]+),(?P<gz>[-\d.]+)\)dps)?"
    r"(?:\s*\|\s*gyro_raw_z:(?P<gzr>[-\d.]+)dps)?"
    r"(?:\s*\|\s*imu r\[g:(?P<gr>[-\d]+) a:(?P<ar>[-\d]+)\]"
    r" axy=\((?P<ax>[-\d.]+),(?P<ay>[-\d.]+)\)m/s2)?"
)

# ── shared data ───────────────────────────────────────────────────────────────
lock = threading.Lock()
bufs = {k: collections.deque(maxlen=HISTORY) for k in [
    "cx", "cy", "kx", "ky", "dx", "dy", "rxmm", "rymm",
    "yg", "ys", "yf", "gx", "gy", "gz", "gzr", "rcx", "rcy"
]}

def serial_reader():
    if SERIAL_IMPORT_ERROR is not None:
        print("PySerial is not available or shadowed by the wrong package.")
        print(f"Import detail: {SERIAL_IMPORT_ERROR}")
        print("Fix:")
        print("  python3 -m pip uninstall -y serial")
        print("  python3 -m pip install pyserial")
        print("Then rerun this script.")
        return

    serial_exception = getattr(pyserial, "SerialException", Exception)
    try:
        ser = pyserial.Serial(PORT, BAUD, timeout=1)
    except serial_exception as e:
        print(f"Cannot open {PORT}: {e}")
        return
    print(f"Opened {PORT} @ {BAUD}")
    while True:
        try:
            raw = ser.readline()
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            continue
        m = LINE_RE.search(line)
        if not m:
            if line:
                print("unparsed:", line[:80])
            continue
        with lock:
            cx = float(m.group("cx"))
            cy = float(m.group("cy"))
            kxg = m.group("kx")
            kyg = m.group("ky")
            for k in bufs:
                g = m.group(k)
                if k == "cx":
                    bufs[k].append(cx)
                elif k == "cy":
                    bufs[k].append(cy)
                elif k == "kx":
                    bufs[k].append(float(kxg) if kxg is not None else cx)
                elif k == "ky":
                    bufs[k].append(float(kyg) if kyg is not None else cy)
                elif g is None:
                    bufs[k].append(bufs[k][-1] if bufs[k] else 0.0)
                else:
                    bufs[k].append(float(g))

# ── plot layout ───────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 2, figsize=(13, 9))
fig.suptitle("Movement Tracker – live serial data", fontsize=12)
fig.tight_layout(pad=2.5)

ax_pos    = axes[0][0]  # XY scatter
ax_flow   = axes[0][1]  # raw optical delta
ax_yaw    = axes[1][0]  # yaw gyro vs sflp
ax_gyro   = axes[1][1]  # gyro xyz
ax_cpi    = axes[2][0]  # raw CPI accumulation
ax_xy_t   = axes[2][1]  # x, y vs time

def init_axes():
    ax_pos.set_title("Position (mm)")
    ax_pos.set_xlabel("X"); ax_pos.set_ylabel("Y")
    ax_pos.set_aspect("equal", adjustable="datalim")

    ax_flow.set_title("Raw optical delta (mm/sample)")
    ax_flow.set_ylabel("mm")

    ax_yaw.set_title("Yaw (deg)")
    ax_yaw.set_ylabel("deg")

    ax_gyro.set_title("Gyro XYZ (dps)")
    ax_gyro.set_ylabel("dps")

    ax_cpi.set_title("Raw accumulated CPI")
    ax_cpi.set_ylabel("counts")

    ax_xy_t.set_title("Kalman X/Y vs time (mm)")
    ax_xy_t.set_ylabel("mm")

init_axes()

# colour helpers
C = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

def animate(_frame):
    with lock:
        snap = {k: list(v) for k, v in bufs.items()}

    n = len(snap["kx"])
    if n == 0:
        return

    t = list(range(n))

    for ax in axes.flat:
        ax.cla()
    init_axes()

    # XY scatter – classic vs kalman
    ax_pos.plot(snap["cx"], snap["cy"], "o-", ms=2, lw=0.8, color=C[0], label="classic")
    ax_pos.plot(snap["kx"], snap["ky"], "o-", ms=2, lw=0.8, color=C[1], label="kalman")
    if snap["kx"]:
        ax_pos.plot(snap["kx"][-1], snap["ky"][-1], "x", ms=8, color=C[1])
    if snap["cx"] and snap["cy"]:
        raw_cpi_text = ""
        if snap["rcx"] and snap["rcy"]:
            raw_cpi_text = f"\nRaw accum CPI X: {int(snap['rcx'][-1])}\nRaw accum CPI Y: {int(snap['rcy'][-1])}"
        ax_pos.text(
            0.02,
            0.98,
            f"Compensated X: {snap['cx'][-1]:.1f} mm\\nCompensated Y: {snap['cy'][-1]:.1f} mm{raw_cpi_text}",
            transform=ax_pos.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.75, "edgecolor": "0.7"},
        )
    ax_pos.legend(fontsize=7)

    # optical flow sample delta (already in mm in UART)
    ax_flow.plot(t, snap["rxmm"], color=C[0], lw=0.9, label="raw dx")
    ax_flow.plot(t, snap["rymm"], color=C[1], lw=0.9, label="raw dy")
    ax_flow.axhline(0, color="k", lw=0.4, ls="--")
    ax_flow.legend(fontsize=7)

    # yaw
    ax_yaw.plot(t, snap["yg"], color=C[0], lw=0.9, label="gyro")
    ax_yaw.plot(t, snap["ys"], color=C[1], lw=0.9, label="sflp")
    ax_yaw.plot(t, snap["yf"], color=C[2], lw=0.9, label="fused")
    ax_yaw.legend(fontsize=7)

    # gyro XYZ
    ax_gyro.plot(t, snap["gx"], color=C[0], lw=0.9, label="gx")
    ax_gyro.plot(t, snap["gy"], color=C[1], lw=0.9, label="gy")
    ax_gyro.plot(t, snap["gz"], color=C[2], lw=0.9, label="gz")
    ax_gyro.axhline(0, color="k", lw=0.4, ls="--")
    ax_gyro.legend(fontsize=7)

    # accumulated raw CPI
    ax_cpi.plot(t, snap["rcx"], color=C[0], lw=0.9, label="raw cpi x")
    ax_cpi.plot(t, snap["rcy"], color=C[1], lw=0.9, label="raw cpi y")
    ax_cpi.axhline(0, color="k", lw=0.4, ls="--")
    ax_cpi.legend(fontsize=7)

    # X/Y vs time
    ax_xy_t.plot(t, snap["cx"], color=C[0], lw=0.9, label="X")
    ax_xy_t.plot(t, snap["cy"], color=C[1], lw=0.9, label="Y")
    ax_xy_t.axhline(0, color="k", lw=0.4, ls="--")
    ax_xy_t.legend(fontsize=7)

if __name__ == "__main__":
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    ani = animation.FuncAnimation(fig, animate, interval=50, cache_frame_data=False)
    plt.show()
