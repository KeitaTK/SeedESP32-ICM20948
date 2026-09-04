#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XIAO ICM-20948 9-axis realtime gauge monitor
=============================================
Reads the NED-converted 9-axis values
`seq,ax,ay,az,gx,gy,gz,mx,my,mz` sent by the XIAO over USB serial (115200 baud)
and shows them in realtime as 3x3 analog-tachometer-style gauges.

  rows = sensors : accel [g] / gyro [rad/s] / mag [uT]
  cols = axes    : X / Y / Z

Usage (run inside visualizer/):
    uv run gauge_monitor.py                      # default: COM9 / dummy fixed values
    uv run gauge_monitor.py --live --port COM10  # read live USB serial
    uv run gauge_monitor.py --selftest           # headless draw check (no GUI)

Notes:
- This tool assumes the XIAO already transmits NED-converted values.
  When level (component side up), az is displayed around -1.0.
- Lines starting with '#' (e.g. "#init OK") are diagnostic and are skipped.
"""

import argparse
import math
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Circle

# ======================================================================
# Configuration (edit here)
# ======================================================================
DUMMY_DATA = True   # True: show fixed dummy values without USB serial
                    #      (set False to read live data from COM_PORT)
DUMMY_VALUES = [    # fixed dummy values (NED, level-equivalent)
    0.05,  -0.10,  -1.00,   # ax, ay, az [g]       az=-1 (down positive)
    0.010, -0.020,  0.030,  # gx, gy, gz [rad/s]   ~0 at rest
    28.0,   5.0,   35.0,    # mx, my, mz [uT]      north mx>0 / down mz>0
]

COM_PORT = "COM9"
BAUD     = 115200

# --- gauge display ranges (derived from each sensor full scale) ---
ACCEL_FS_G  = 2.0     # accel  +-2g     (firmware: setAccRange(2G))
GYRO_FS_DPS = 250.0   # gyro   +-250dps (firmware: setGyrRange(250))
                      #   display unit is rad/s, so +-250*pi/180 = +-4.36 rad/s
MAG_FS_UT   = 100.0   # mag    +-100uT (AK09916 native FS is +-4912uT, but set
                      #   to match real geomagnetic field ~20-70uT and the
                      #   firmware sanity range 10-100uT)

REFRESH_MS  = 100     # refresh interval [ms] (=10Hz drawing)

ROW_UNITS  = ["g", "rad/s", "uT"]
ROW_NAMES  = ["accel", "gyro", "mag"]
ROW_DIGITS = [3, 3, 1]              # decimal digits for numeric readout
# ======================================================================


class SensorReader(threading.Thread):
    """Background thread that reads the 9-axis CSV from the USB serial port."""

    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.latest = None      # [ax,ay,az,gx,gy,gz,mx,my,mz]
        self.seq = -1
        self.connected = False
        self.error = ""
        self._stop = threading.Event()

    def run(self):
        import serial  # pyserial (declared in the uv project)
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    self.connected = True
                    self.error = ""
                    while not self._stop.is_set():
                        raw = ser.readline()
                        line = raw.decode("utf-8", errors="ignore").strip()
                        if not line or not line[0].isdigit():
                            continue
                        parts = line.split(",")
                        if len(parts) < 10:
                            continue
                        try:
                            seq = int(parts[0])
                            vals = [float(x) for x in parts[1:10]]
                        except ValueError:
                            continue
                        self.seq = seq
                        self.latest = vals
            except Exception as exc:  # noqa: BLE001
                self.connected = False
                self.error = str(exc)
                time.sleep(1.0)

    def stop(self):
        self._stop.set()


# ======================================================================
# Gauge drawing
# ======================================================================
def build_gauge(ax, vmax, name, unit, digits):
    """Draw an analog-tachometer-style gauge with a +-vmax scale and return
    an updater that moves only the needle and the numeric readout."""
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-1.3, 1.3)
    ax.set_ylim(-0.6, 1.35)
    ax.set_title(f"{name} [{unit}]", fontsize=11)

    # background rail (upper semicircle)
    ax.add_patch(Arc((0, 0), 2.0, 2.0, theta1=0.0, theta2=180.0,
                     edgecolor="#999999", lw=1.5))
    # colored zones (green center / red edges)
    def _zone(f1, f2, color):
        t1 = 90.0 - 90.0 * f2   # Arc is drawn CCW (theta1 < theta2)
        t2 = 90.0 - 90.0 * f1
        ax.add_patch(Arc((0, 0), 1.72, 1.72, theta1=t1, theta2=t2,
                         edgecolor=color, lw=7))
    _zone(-0.80, 0.80, "#2ca02c")
    _zone(-1.00, -0.80, "#d62728")
    _zone(0.80, 1.00, "#d62728")

    # tick marks
    for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
        rad = math.radians(90.0 - 90.0 * frac)
        ax.plot([0.82 * math.cos(rad), 0.92 * math.cos(rad)],
                [0.82 * math.sin(rad), 0.92 * math.sin(rad)],
                color="#333333", lw=1.2, zorder=3)
    # scale labels (-max / 0 / +max)
    for frac, txt in ((-1.0, f"-{vmax:g}"), (0.0, "0"), (1.0, f"+{vmax:g}")):
        rad = math.radians(90.0 - 90.0 * frac)
        ax.text(1.06 * math.cos(rad), 1.06 * math.sin(rad), txt,
                ha="center", va="center", fontsize=8, color="#333333")

    # --- dynamic (per-frame) elements ---
    (needle,) = ax.plot([0, 0], [0, 0.78], color="#d62728", lw=2.6, zorder=4)
    ax.add_patch(Circle((0, 0), 0.05, facecolor="#333333", zorder=5))
    txt_val = ax.text(0, -0.30, "----", ha="center", va="center",
                      fontsize=13, fontweight="bold", color="#111111")

    def update(value):
        f = max(-1.0, min(1.0, value / vmax)) if vmax else 0.0
        rad = math.radians(90.0 - 90.0 * f)   # -max=left / 0=top / +max=right
        needle.set_data([0, 0.78 * math.cos(rad)],
                        [0, 0.78 * math.sin(rad)])
        txt_val.set_text(f"{value:+.{digits}f}")
        return needle, txt_val

    return update


# ======================================================================
# Headless self-test: draw all 9 gauges once and exit
# ======================================================================
def selftest():
    import matplotlib
    matplotlib.use("Agg")
    fig = plt.figure(figsize=(12, 6.5))
    axes = fig.subplots(3, 3)
    gauge_max = [ACCEL_FS_G, GYRO_FS_DPS * math.pi / 180.0, MAG_FS_UT]
    for r in range(3):
        for c in range(3):
            upd = build_gauge(axes[r][c], gauge_max[r],
                              f"{ROW_NAMES[r]}{'XYZ'[c]}",
                              ROW_UNITS[r], ROW_DIGITS[r])
            upd(DUMMY_VALUES[r * 3 + c])
    fig.suptitle("selftest (dummy)", fontsize=13)
    fig.canvas.draw()
    plt.close(fig)
    print("selftest OK: 9 gauges drawn.")


# ======================================================================
# main
# ======================================================================
def main():
    p = argparse.ArgumentParser(description="XIAO ICM-20948 9-axis gauge monitor")
    p.add_argument("--port", default=COM_PORT, help=f"serial COM port (default: {COM_PORT})")
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--refresh", type=int, default=REFRESH_MS,
                   help="refresh interval in ms (default: %d)" % REFRESH_MS)
    grp = p.add_mutually_exclusive_group()
    grp.add_argument("--live", action="store_true",
                     help="read live USB serial (disables dummy mode)")
    grp.add_argument("--dummy", action="store_true", help="force dummy display")
    p.add_argument("--selftest", action="store_true",
                   help="draw headlessly once and exit (no GUI)")
    args = p.parse_args()

    if args.selftest:
        selftest()
        return

    dummy = DUMMY_DATA or args.dummy
    if args.live:
        dummy = False

    reader = None
    if not dummy:
        reader = SensorReader(args.port, args.baud)
        reader.start()
        print(f"[live ] reading {args.port} @ {args.baud} baud ...")
        print("        needles stay at 0 until data arrives")
    else:
        print("[dummy] showing fixed dummy values")
        print("        (NED level-equivalent: az=-1.0 / mz=+35uT)")

    gauge_max = [ACCEL_FS_G, GYRO_FS_DPS * math.pi / 180.0, MAG_FS_UT]
    fig = plt.figure(figsize=(12, 6.5))
    axes = fig.subplots(3, 3)
    updaters = []
    for r in range(3):
        for c in range(3):
            updaters.append(
                build_gauge(axes[r][c], gauge_max[r],
                            f"{ROW_NAMES[r]}{'XYZ'[c]}",
                            ROW_UNITS[r], ROW_DIGITS[r]))
    status = fig.suptitle("", fontsize=11)

    def animate(_frame):
        if dummy:
            vals = DUMMY_VALUES
            mode = "DUMMY (fixed values)"
            extra = ""
        else:
            vals = reader.latest if reader.latest is not None else [0.0] * 9
            mode = f"LIVE {args.port} @ {args.baud}"
            if reader.connected:
                extra = "connected" + (f" / seq={reader.seq}" if reader.seq >= 0 else "")
            else:
                extra = f"connection error ({reader.error})"
        for i, upd in enumerate(updaters):
            upd(vals[i])
        status.set_text(f"XIAO ICM-20948 9-axis monitor (NED)   |   {mode}   |   {extra}")
        return ()

    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, animate, interval=args.refresh,
                         cache_frame_data=False)
    try:
        plt.show()
    finally:
        if reader is not None:
            reader.stop()
        plt.close(fig)


if __name__ == "__main__":
    main()

