#!/usr/bin/env python3
"""
Real-time serial plotter for Movement Tracker firmware.

Parses lines of the form (emitted by runtime.c at DEBUG_PRINT_PERIOD_MS):
    x:<m> y:<m> | vx:<m/s> vy:<m/s> | yaw:<deg> sflp:<deg> | gz:<dps> dps | dx:<cpi> dy:<cpi>

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

PORT    = "/dev/tty.usbmodem11302"
BAUD    = 115200
HISTORY = 500

LINE_RE = re.compile(
    r"x:(?P<x>[-\d.]+)\s+y:(?P<y>[-\d.]+)"
    r"\s*\|\s*vx:(?P<vx>[-\d.]+)\s+vy:(?P<vy>[-\d.]+)"
    r"\s*\|\s*yaw:(?P<yaw>[-\d.]+)\s+sflp:(?P<sflp>[-\d.]+)"
    r"\s*\|\s*gz:(?P<gz>[-\d.]+)\s+dps"
    r"\s*\|\s*dx:(?P<dx>[-\d]+)\s+dy:(?P<dy>[-\d]+)"
    r"(?:\s*\|\s*dmm:(?P<dxmm>[-\d.]+),(?P<dymm>[-\d.]+))?"
)

FIELDS = ["x", "y", "vx", "vy", "yaw", "sflp", "gz", "dx", "dy", "dxmm", "dymm"]

lock = threading.Lock()
bufs = {k: collections.deque(maxlen=HISTORY) for k in FIELDS}


def serial_reader():
    if SERIAL_IMPORT_ERROR is not None:
        print("PySerial is not available or shadowed by the wrong package.")
        print(f"Import detail: {SERIAL_IMPORT_ERROR}")
        print("Fix:")
        print("  python3 -m pip uninstall -y serial")
        print("  python3 -m pip install pyserial")
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
            raw  = ser.readline()
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            continue
        m = LINE_RE.search(line)
        if not m:
            if line:
                print("unparsed:", line[:120])
            continue
        with lock:
            for k in FIELDS:
                g = m.group(k)
                bufs[k].append(float(g) if g is not None else (bufs[k][-1] if bufs[k] else 0.0))


# ── plot layout ───────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 2, figsize=(13, 9))
fig.suptitle("Movement Tracker – live serial data", fontsize=12)
fig.tight_layout(pad=2.5)

ax_pos   = axes[0][0]   # XY scatter
ax_xy_t  = axes[0][1]   # X/Y vs time
ax_vel   = axes[1][0]   # vx/vy vs time
ax_yaw   = axes[1][1]   # yaw kalman vs sflp
ax_gz    = axes[2][0]   # gyro Z
ax_paa   = axes[2][1]   # raw PAA delta CPI

C = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]


def init_axes():
    ax_pos.set_title("Position (m)")
    ax_pos.set_xlabel("X (m)"); ax_pos.set_ylabel("Y (m)")
    ax_pos.set_aspect("equal", adjustable="datalim")

    ax_xy_t.set_title("X / Y vs time (m)")
    ax_xy_t.set_ylabel("m")

    ax_vel.set_title("Velocity (m/s)")
    ax_vel.set_ylabel("m/s")

    ax_yaw.set_title("Yaw (deg)")
    ax_yaw.set_ylabel("deg")

    ax_gz.set_title("Gyro Z (dps)")
    ax_gz.set_ylabel("dps")

    ax_paa.set_title("Optical flow cumulative position (mm)")
    ax_paa.set_ylabel("mm")


init_axes()


def animate(_frame):
    with lock:
        snap = {k: list(v) for k, v in bufs.items()}

    n = len(snap["x"])
    if n == 0:
        return

    t = list(range(n))

    for ax in axes.flat:
        ax.cla()
    init_axes()

    # XY scatter
    ax_pos.plot(snap["x"], snap["y"], "o-", ms=2, lw=0.8, color=C[0])
    if snap["x"]:
        ax_pos.plot(snap["x"][-1], snap["y"][-1], "x", ms=8, color=C[0])
        ax_pos.text(
            0.02, 0.98,
            f"X: {snap['x'][-1]:.3f} m\nY: {snap['y'][-1]:.3f} m",
            transform=ax_pos.transAxes, va="top", ha="left", fontsize=8,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.75, "edgecolor": "0.7"},
        )

    # X/Y vs time
    ax_xy_t.plot(t, snap["x"], color=C[0], lw=0.9, label="X")
    ax_xy_t.plot(t, snap["y"], color=C[1], lw=0.9, label="Y")
    ax_xy_t.axhline(0, color="k", lw=0.4, ls="--")
    ax_xy_t.legend(fontsize=7)

    # velocity
    ax_vel.plot(t, snap["vx"], color=C[0], lw=0.9, label="vx")
    ax_vel.plot(t, snap["vy"], color=C[1], lw=0.9, label="vy")
    ax_vel.axhline(0, color="k", lw=0.4, ls="--")
    ax_vel.legend(fontsize=7)

    # yaw
    ax_yaw.plot(t, snap["yaw"],  color=C[0], lw=0.9, label="Kalman")
    ax_yaw.plot(t, snap["sflp"], color=C[1], lw=0.9, label="SFLP")
    ax_yaw.legend(fontsize=7)

    # gyro Z
    ax_gz.plot(t, snap["gz"], color=C[2], lw=0.9)
    ax_gz.axhline(0, color="k", lw=0.4, ls="--")

    # Optical flow cumulative position
    cum_x = [sum(snap["dxmm"][:i+1]) for i in range(n)]
    cum_y = [sum(snap["dymm"][:i+1]) for i in range(n)]
    ax_paa.plot(t, cum_x, color=C[0], lw=0.9, label="X (mm)")
    ax_paa.plot(t, cum_y, color=C[1], lw=0.9, label="Y (mm)")
    ax_paa.axhline(0, color="k", lw=0.4, ls="--")
    ax_paa.legend(fontsize=7)


if __name__ == "__main__":
    th = threading.Thread(target=serial_reader, daemon=True)
    th.start()
    ani = animation.FuncAnimation(fig, animate, interval=50, cache_frame_data=False)
    plt.show()
