#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XIAO ESP32-C3 + ICM-20948  回転方向リアルタイム可視化ツール
=============================================================

USBシリアル(COMポート)から受信した roll / pitch / heading / 磁界値を使い、
基板の3次元姿勢(回転方向)をリアルタイムにアニメーション表示します。

使い方 (uvプロジェクト内から実行):
    uv run rotation_visualizer.py                # デフォルト COM9 / top視点
    uv run rotation_visualizer.py --port COM10   # ポート指定
    uv run rotation_visualizer.py --port COM9 --interval 50
    uv run rotation_visualizer.py --view 3d      # 斜め3D視点で見る

視点は --view {top,3d} で切替可能 (既定: top = 真上から見下ろすコンパス視点)。
実機の向きとアニメーションが合わない場合は
--flip-roll / --flip-pitch / --flip-heading で各軸の符号を反転できます。

※ ファームウェア (シリアル出力版) が以下形式で出力している必要があります:
   ms,mx_uT,my_uT,mz_uT,total_uT,roll_deg,pitch_deg,heading_deg
"""

import argparse
import math
import threading
import time

import numpy as np
import serial

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ---------------------------------------------------------------------------
# センサー読み取りスレッド
# ---------------------------------------------------------------------------
class SensorReader(threading.Thread):
    """COMポートを監視し、最新の計測値を保持するスレッド。"""

    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.latest = {
            "ms": 0, "mx": 0.0, "my": 0.0, "mz": 0.0,
            "total": 0.0, "roll": 0.0, "pitch": 0.0, "heading": 0.0,
        }
        self.connected = False
        self.error = ""
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    self.connected = True
                    self.error = ""
                    while not self._stop.is_set():
                        raw = ser.readline()
                        line = raw.decode("utf-8", errors="ignore").strip()
                        if not line:
                            continue
                        parts = line.split(",")
                        if len(parts) < 8 or parts[0] == "ms":
                            continue
                        try:
                            vals = [float(v) for v in parts]
                        except ValueError:
                            continue
                        self.latest = {
                            "ms": int(vals[0]),
                            "mx": vals[1], "my": vals[2], "mz": vals[3],
                            "total": vals[4],
                            "roll": vals[5], "pitch": vals[6], "heading": vals[7],
                        }
            except serial.SerialException as exc:
                self.connected = False
                self.error = str(exc)
                time.sleep(1.0)
            except Exception as exc:
                self.error = str(exc)
                time.sleep(1.0)

    def stop(self):
        self._stop.set()


# ---------------------------------------------------------------------------
# 回転行列
# ---------------------------------------------------------------------------
def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]])


def attitude_matrix(roll_deg, pitch_deg, heading_deg):
    """roll/pitch/heading(deg) から body→world の回転行列 R を返す。"""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(heading_deg)
    # 3-2-1 (yaw→pitch→roll) 列：ワールド座標でのボディ軸 = Rz(yaw)@Ry(pitch)@Rx(roll)
    return rot_z(y) @ rot_y(p) @ rot_x(r)


# ---------------------------------------------------------------------------
# 描画ヘルパー
# ---------------------------------------------------------------------------
def draw_axes(ax, R, length=1.0):
    """基板のローカル3軸(X赤/Y緑/Z青)をワールド座標に描く。"""
    colors = ["tab:red", "tab:green", "tab:blue"]
    labels = ["X", "Y", "Z"]
    for i in range(3):
        v = R[:, i] * length
        ax.quiver(0, 0, 0, v[0], v[1], v[2],
                  color=colors[i], arrow_length_ratio=0.12, linewidth=2.5)
        ax.text(v[0], v[1], v[2], labels[i],
                color=colors[i], fontsize=13, fontweight="bold")


def draw_board_plane(ax, R):
    """基板のXY平面(薄い板)をワールド座標に描く。"""
    hx, hy = 0.55, 0.35
    corners_body = np.array([
        [-hx, -hy, 0.0], [hx, -hy, 0.0],
        [hx, hy, 0.0], [-hx, hy, 0.0],
    ])
    corners = np.array([R @ c for c in corners_body])
    for i in range(4):
        p0, p1 = corners[i], corners[(i + 1) % 4]
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color="0.55", lw=1.2)
    poly = Poly3DCollection([corners], alpha=0.18, facecolor="cyan", edgecolor="none")
    ax.add_collection3d(poly)


# 軸符号フラグ(コマンドラインで反転可能)
SIGN_ROLL = 1.0
SIGN_PITCH = 1.0
SIGN_HEADING = 1.0

# 視点: "top"=真上から見下ろす(既定) / "3d"=斜め視点
VIEW = "top"

reader = None
ax = None


def update(_frame):
    global ax
    d = reader.latest

    R = attitude_matrix(
        d["roll"] * SIGN_ROLL,
        d["pitch"] * SIGN_PITCH,
        d["heading"] * SIGN_HEADING,
    )

    ax.clear()
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-1.6, 1.6)
    ax.set_zlim(-1.6, 1.6)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect((1, 1, 1))
    if VIEW == "top":
        ax.view_init(elev=90, azim=-90)
    else:
        ax.view_init(elev=25, azim=-60)

    # 地面グリッド (z=0)
    lim = 1.3
    grid = np.linspace(-lim, lim, 7)
    for x in grid:
        ax.plot([x, x], [-lim, lim], [0, 0], color="0.8", lw=0.5, alpha=0.6)
    for yy in grid:
        ax.plot([-lim, lim], [yy, yy], [0, 0], color="0.8", lw=0.5, alpha=0.6)

    # ワールドZ軸(上方向)。真上視点では視線と重なるため省略。
    if VIEW != "top":
        ax.quiver(0, 0, 0, 0, 0, 1.5, color="0.4", linewidth=1.0, arrow_length_ratio=0.06)
        ax.text(0, 0, 1.55, "UP", color="0.6", fontsize=9)

    draw_board_plane(ax, R)
    draw_axes(ax, R)

    conn = "CONNECTED" if reader.connected else "DISCONNECTED (%s)" % reader.error
    ax.set_title(
        "ICM-20948 orientation | " + conn + "\n"
        "roll=%7.1f   pitch=%7.1f   heading=%7.1f deg\n"
        "|B|=%6.2f uT   (mx=%6.2f  my=%6.2f  mz=%6.2f) uT"
        % (d["roll"], d["pitch"], d["heading"],
           d["total"], d["mx"], d["my"], d["mz"]),
        fontsize=10,
    )
    return []


def main():
    global reader, ax, SIGN_ROLL, SIGN_PITCH, SIGN_HEADING, VIEW
    parser = argparse.ArgumentParser(description="ICM-20948 回転方向リアルタイム可視化")
    parser.add_argument("--port", default="COM9", help="シリアルポート (既定: COM9)")
    parser.add_argument("--baud", type=int, default=115200, help="ボーレート (既定: 115200)")
    parser.add_argument("--interval", type=int, default=50, help="描画更新間隔 ms (既定: 50)")
    parser.add_argument("--view", default="top", choices=["top", "3d"],
                        help="視点 (既定: top = 真上から見下ろす)")
    parser.add_argument("--flip-roll", action="store_true", help="roll の符号を反転")
    parser.add_argument("--flip-pitch", action="store_true", help="pitch の符号を反転")
    parser.add_argument("--flip-heading", action="store_true", help="heading の符号を反転")
    args = parser.parse_args()

    VIEW = args.view
    if args.flip_roll:
        SIGN_ROLL = -1.0
    if args.flip_pitch:
        SIGN_PITCH = -1.0
    if args.flip_heading:
        SIGN_HEADING = -1.0

    reader = SensorReader(args.port, args.baud)
    reader.start()

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(8.5, 7.5))
    ax = fig.add_subplot(111, projection="3d")

    anim = FuncAnimation(fig, update, interval=args.interval, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        print("\n終了しました")


if __name__ == "__main__":
    main()