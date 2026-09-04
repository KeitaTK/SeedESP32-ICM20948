# -*- coding: utf-8 -*-
"""
旧ファーム(PC表示当時)の軸定義・変換を再現し、今回の生データキャプチャで
ヘディングが正しく回転するかを検証する。

旧ファームの変換(センサ座標系, accel[g], mag[uT], z-up静止):
  roll  r = atan2(ay, az)
  pitch p = atan2(-ax, sqrt(ay^2+az^2))
  傾斜補正: bx = mx*cos(p)+mz*sin(p)
            by = mx*sin(r)*sin(p)+my*cos(r)-mz*sin(r)*cos(p)
  heading   = atan2(-by, bx) [deg], <0 なら +360
"""
import math

def parse(path):
    rows = []
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for raw in f:
            line = raw.strip()
            if not line or not line[0].isdigit():
                continue
            parts = line.split(',')
            if len(parts) < 10:
                continue
            try:
                v = [float(x) for x in parts[:10]]
            except ValueError:
                continue
            rows.append(v)
    return rows

def tilt_heading(v):
    # v = [seq, ax, ay, az, gx_rad, gy_rad, gz_rad, mx, my, mz]
    seq, ax, ay, az, gx, gy, gz, mx, my, mz = v
    r = math.atan2(ay, az)
    p = math.atan2(-ax, math.hypot(ay, az))
    bxc = mx*math.cos(p) + mz*math.sin(p)
    byc = mx*math.sin(r)*math.sin(p) + my*math.cos(r) - mz*math.sin(r)*math.cos(p)
    h = math.degrees(math.atan2(-byc, bxc))
    if h < 0:
        h += 360.0
    return h

def raw_heading(v):
    return math.degrees(math.atan2(v[8], v[7]))  # atan2(my, mx)

def unwrap_stats(headings):
    d0 = 0.0
    path = 0.0
    net = 0.0
    mn = mx2 = headings[0]
    jumps = 0
    for i in range(1, len(headings)):
        d = headings[i] - headings[i-1]
        # 0..360 なので ±180 に正規化
        while d > 180: d -= 360
        while d < -180: d += 360
        path += abs(d)
        net += d
        if abs(d) > 45.0:
            jumps += 1
        cur = headings[0] + net
        mn = min(mn, cur)
        mx2 = max(mx2, cur)
    return path, net, mn, mx2, jumps

rows = parse('capture_rotate.txt')
print('data_samples=%d' % len(rows))

th = [tilt_heading(r) for r in rows]
rh = [raw_heading(r) for r in rows]

for name, h in (('tilt_comp(旧ファーム式)', th), ('atan2(my,mx) raw', rh)):
    path, net, mn, mx2, jumps = unwrap_stats(h)
    print('--- %s ---' % name)
    print('  total_abs_path_deg = %.1f' % path)
    print('  net_change_deg     = %.1f' % net)
    print('  unwrap range deg   = [%.1f, %.1f]' % (mn, mx2))
    print('  jumps(>45deg/sample)= %d' % jumps)
    print('  first=%.1f  last=%.1f' % (h[0], h[-1]))
# 2.5s間隔のスナップショット (tilt_comp)
print('--- tilt_comp unwrapped trace every 2.5s (0..50s) ---')
net = 0.0
prev = th[0]
for i in range(len(th)):
    if i == 0:
        continue
    d = th[i] - th[i-1]
    while d > 180: d -= 360
    while d < -180: d += 360
    net += d
    if i % 250 == 0:
        print('t=%5.1fs h_unwrap=%8.1f' % (i/100.0, th[0]+net))
