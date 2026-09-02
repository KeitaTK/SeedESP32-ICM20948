#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
axis_capture.py — ICM-20948 軸マッピング実験用キャプチャツール
=================================================================

既存 rotation_visualizer.py の SensorReader を流用し、USBシリアル(COM)から
100Hz の9軸生データ(seq,ax,ay,az,gx,gy,gz,mx,my,mz)を読み取り、定義済みの
実験シーケンスに沿って Enter でスナップショット(平均/最小/最大)を記録します。

使い方:
    uv run axis_capture.py                        # 既定 COM9 / 115200
    uv run axis_capture.py --port COM10
    uv run axis_capture.py --duration 1.5         # 静止記録の秒数
    uv run axis_capture.py --rotate-duration 3    # 回転記録の秒数
    uv run axis_capture.py --list                 # シーケンス一覧のみ表示

操作:
    1) 指示が表示される → 基板をその姿勢にする
    2) Enter = 記録開始 / r = 直前をやり直し / q = 中断して保存
    3) 平均/最小/最大を表示・CSVへ追記 → 次のステップへ

出力:
    capture_axis_summary.csv … 各ステップの 平均/最小/最大 (ラベル付き)
    capture_axis_raw.csv     … 全サンプル (ラベル付き・再解析用)
"""

import argparse
import csv
import os
import threading
import time
from dataclasses import dataclass

import serial

AXIS_NAMES = ["ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz"]


@dataclass
class Step:
    label: str
    instruction: str
    kind: str = "static"  # "static"=静止(平均を見る) / "rotate"=回転(最大・最小の符号を見る)


SEQUENCE = [
    Step("accel_level",    "【加速度】基板を水平(部品面が上)に置く", "static"),
    Step("accel_x_down",   "【加速度】基板の印字Xを真下に向ける(縦に立てる)", "static"),
    Step("accel_x_up",     "【加速度】基板の印字Xを真上に向ける", "static"),
    Step("accel_y_down",   "【加速度】基板の印字Yを真下に向ける(縦に立てる)", "static"),
    Step("accel_y_up",     "【加速度】基板の印字Yを真上に向ける", "static"),
    Step("accel_flip",     "【加速度】基板を裏返す(部品面が下)", "static"),
    Step("gyro_yaw_cw",    "【ジャイロ】水平のまま上から見て時計回りに1回転(記録中に回す)", "rotate"),
    Step("gyro_yaw_ccw",   "【ジャイロ】水平のまま上から見て反時計回りに1回転(記録中に回す)", "rotate"),
    Step("gyro_roll_pos",  "【ジャイロ】基板X軸まわりに右ねじ正方向へ回す(記録中に回す)", "rotate"),
    Step("gyro_pitch_pos", "【ジャイロ】基板Y軸まわりに右ねじ正方向へ回す(記録中に回す)", "rotate"),
    Step("mag_north",      "【地磁気】水平のまま印字Xを磁北に向ける", "static"),
    Step("mag_east",       "【地磁気】水平のまま右に90°回す(東)", "static"),
    Step("mag_south",      "【地磁気】水平のままさらに90°(南)", "static"),
    Step("mag_west",       "【地磁気】水平のままさらに90°(西)", "static"),
]


class SensorReader(threading.Thread):
    """COMポートを監視し、最新の9軸値(seq付き)を保持するスレッド。"""

    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.latest_seq = -1
        self.latest_vals = None
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
                        if not line or not line[0].isdigit():
                            continue
                        parts = line.split(",")
                        if len(parts) < 10:
                            continue
                        try:
                            seq = int(parts[0])
                            vals = [float(v) for v in parts[1:10]]
                        except ValueError:
                            continue
                        self.latest_seq = seq
                        self.latest_vals = vals
            except serial.SerialException as exc:
                self.connected = False
                self.error = str(exc)
                time.sleep(1.0)
            except Exception as exc:
                self.error = str(exc)
                time.sleep(1.0)

    def stop(self):
        self._stop.set()



def collect_burst(reader, duration):
    """duration秒ぶんサンプリングし、[(seq, [9値]), ...] を返す。"""
    samples = []
    seen = set()
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration:
        if reader.latest_vals is not None and reader.latest_seq not in seen:
            seen.add(reader.latest_seq)
            samples.append((reader.latest_seq, list(reader.latest_vals)))
        time.sleep(0.004)
    return samples


def print_result(step, samples, duration):
    n = len(samples)
    print("\n--- 結果: %s (%d点 / %.1f秒) ---" % (step.label, n, duration))
    if n == 0:
        print("    (データなし: 接続/ポートを確認してください)")
        return None
    vals = [v for _, v in samples]
    print("     " + " ".join("%9s" % a for a in AXIS_NAMES))
    means, mins, maxs = [], [], []
    for j in range(9):
        col = [v[j] for v in vals]
        means.append(sum(col) / len(col))
        mins.append(min(col))
        maxs.append(max(col))
    print("mean " + " ".join("%9.4f" % v for v in means))
    print("min  " + " ".join("%9.4f" % v for v in mins))
    print("max  " + " ".join("%9.4f" % v for v in maxs))
    if step.kind == "rotate":
        # 最も振れ幅(max-min)が大きい軸を主軸とする(定常バイアスの影響を排除)
        swing_idx = max(range(9), key=lambda j: maxs[j] - mins[j])
        swing = maxs[swing_idx] - mins[swing_idx]
        if abs(maxs[swing_idx]) >= abs(mins[swing_idx]):
            dir_val = maxs[swing_idx]
            dir_s = "正(右ねじ正方向)"
        else:
            dir_val = mins[swing_idx]
            dir_s = "負(右ねじ逆方向)"
        print("    >> 回転の主軸: %s (振れ幅=%+.4f) ピーク=%+.4f (%s)"
              % (AXIS_NAMES[swing_idx], swing, dir_val, dir_s))
    return {"label": step.label, "n": n, "mean": means, "min": mins, "max": maxs}



def main():
    p = argparse.ArgumentParser(description="ICM-20948 軸マッピング実験キャプチャ")
    p.add_argument("--port", default="COM9", help="シリアルポート (既定: COM9)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duration", type=float, default=1.5, help="静止記録の秒数")
    p.add_argument("--rotate-duration", type=float, default=3.0, help="回転記録の秒数")
    p.add_argument("--out", default=".", help="出力先ディレクトリ")
    p.add_argument("--list", action="store_true", help="シーケンス一覧のみ表示")
    args = p.parse_args()

    if args.list:
        for i, s in enumerate(SEQUENCE, 1):
            print("%2d. [%s] %-16s : %s" % (i, s.kind, s.label, s.instruction))
        return

    reader = SensorReader(args.port, args.baud)
    reader.start()

    summary_path = os.path.join(args.out, "capture_axis_summary.csv")
    raw_path = os.path.join(args.out, "capture_axis_raw.csv")

    raw_file = open(raw_path, "w", newline="", encoding="utf-8")
    raw_writer = csv.writer(raw_file)
    raw_writer.writerow(["label", "seq"] + AXIS_NAMES)

    results = {}  # label -> dict
    print("=== ICM-20948 軸マッピング実験キャプチャ ===")
    print("ポート: %s" % args.port)

    i = 0
    while i < len(SEQUENCE):
        step = SEQUENCE[i]
        conn = "接続:OK" if reader.connected else "接続:NG(%s)" % reader.error
        dur = args.duration if step.kind == "static" else args.rotate_duration
        print("\n[%d/%d] %s" % (i + 1, len(SEQUENCE), conn))
        print("  " + step.instruction)
        if step.kind == "rotate":
            print("  → Enterを押したらすぐ回し始めてください (記録%.0f秒)" % dur)
        else:
            print("  → 姿勢を固定して Enter (記録%.0f秒)  [r=やり直し / q=中断]" % dur)
        if reader.latest_vals is not None:
            print("  現在値: " + " ".join("%7.3f" % v for v in reader.latest_vals))

        try:
            cmd = input("  > ").strip().lower()
        except EOFError:
            cmd = "q"

        if cmd == "q":
            print("\n中断。ここまでを保存します。")
            break
        if cmd == "r":
            if i > 0:
                i -= 1
            continue

        print("  記録開始(%.0f秒)..." % dur)
        samples = collect_burst(reader, dur)

        for seq, v in samples:
            raw_writer.writerow([step.label, seq] + v)
        raw_file.flush()

        res = print_result(step, samples, dur)
        if res is not None:
            results[step.label] = res
        i += 1

    raw_file.close()
    reader.stop()

    with open(summary_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        header = ["label", "samples", "duration_s"]
        for a in AXIS_NAMES:
            header += ["%s_mean" % a, "%s_min" % a, "%s_max" % a]
        w.writerow(header)
        for step in SEQUENCE:
            if step.label not in results:
                continue
            r = results[step.label]
            dur = args.duration if step.kind == "static" else args.rotate_duration
            row = [step.label, r["n"], dur]
            for j in range(9):
                row += [r["mean"][j], r["min"][j], r["max"][j]]
            w.writerow(row)

    print("\n=== 完了 ===")
    print("サマリ: %s" % summary_path)
    print("生データ: %s" % raw_path)


if __name__ == "__main__":
    main()

