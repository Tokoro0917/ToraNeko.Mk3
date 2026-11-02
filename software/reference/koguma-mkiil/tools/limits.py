#!/usr/bin/env python3
"""logs/ のログから、ノイズと性能の限界を数字で出す(docs/noise_limits.md)。

    python3 tools/limits.py                 # logs/ の全部
    python3 tools/limits.py logs/2026102*   # 選んだファイルだけ

numpy だけを使う。出すもの:
  1. 止まったときのセンサのノイズ(NOISE、PWMに合わせて読む行)を mm に直したもの
  2. 走っているときの横センサのノイズ(1msおきの走行ログの直線)
  3. 電池の内部抵抗(配線込み)と、モーターの電圧の張り付き
  4. ターンごとの、必要な電圧と電池の電圧の差(マイナスなら張り付く)
  5. 電圧から決まる直線の加速の上限(「3」で内部抵抗が出れば、その値でも出す)
  6. 直線で実際に出た加速(張り付いているところ = 今の電池での上限)を速度ごとに。
     モード6 No.0(加速 40000 で 6000 まで)のように、目標の加速が上限を超える直進のログで測る

モーターのモデル(1輪あたり、電圧[V]):
  並進  V = C + KE*v[m/s] + KA*a[m/s^2]
  回転  左右の差の半分 = KW*w[rad/s] + KJ*wdot[rad/s^2]
並進の係数はログからは決めにくい(定速の区間の速度が偏っている)ので、
FF(motor.c の Aff・Bff・FF_offset)と定速の区間の電圧に合う値に決めてある。
回転の係数は毎回ログから求め直し、決めてある値と並べて出す。
"""
import glob
import sys

import numpy as np

# 並進(1輪): FF の Bff*KE = 1.47 V/(m/s)、Aff*R*T/KT = 0.35 V/(m/s^2)、FF_offset 15/200*16 = 1.2 V と、
# 定速の区間の電圧(1000mm/s で約2.6V、2000mm/s で約4.2V)から
C, KE, KA = 1.25, 1.5, 0.35
# 回転(左右の差の半分): 2026-10-18〜20 のログ全部から求めた値
KW, KJ = 0.063, 0.0060
# 電流の計算用(Define.h): 巻線抵抗、KT から出した逆起電力[V/(m/s)、車輪の速度]
RM, KEMF = 5.4, 0.389
TREAD = 0.055
SAT = 199  # volt_L/R(±200 が上限)がこれ以上なら張り付き

# 横・前センサの表(Core/Src/WallDistance.c)。並びは g_sensor と同じ FL, R, L, FR
TABLE = {
    'FL': ([60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180, 192, 204],
           [1366, 902, 645, 470, 356, 295, 230, 195, 167, 146, 132, 123, 112]),
    'R': ([37, 49, 61, 73, 85, 97, 109, 121, 133],
          [780, 522, 356, 262, 201, 160, 131, 110, 93]),
    'L': ([37, 49, 61, 73, 85, 97, 109, 121, 133],
          [910, 603, 415, 308, 237, 183, 150, 125, 105]),
    'FR': ([60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180, 192, 204],
           [1377, 940, 627, 461, 358, 281, 248, 204, 175, 155, 129, 120, 112]),
}


def slope_at(sensor, value):
    """センサ値 value のところの、1mm あたりの値の変化(表の直線補間の傾き)"""
    mm, v = TABLE[sensor]
    for i in range(len(v) - 1):
        if v[i] >= value >= v[i + 1]:
            return (v[i] - v[i + 1]) / (mm[i + 1] - mm[i])
    return (v[-2] - v[-1]) / (mm[-1] - mm[-2])


def mm_at(sensor, value):
    mm, v = TABLE[sensor]
    return float(np.interp(-value, [-x for x in v], mm))


def load_log(fn):
    meta = {}
    hdr = None
    rows = []
    for line in open(fn, encoding='utf-8'):
        line = line.strip()
        if line.startswith('#BEGIN LOG'):
            for kv in line.split()[2:]:
                k, v = kv.split('=')
                meta[k] = float(v)
        elif line.startswith('t,'):
            hdr = line.split(',')
        elif hdr and line and (line[0].isdigit() or line[0] == '-'):
            try:
                rows.append([float(x) for x in line.split(',')])
            except ValueError:
                pass
    if not hdr or not rows:
        return None, None
    a = np.array(rows)
    return meta, {k: a[:, i] for i, k in enumerate(hdr)}


def smooth(x, k):
    return np.convolve(x, np.ones(k) / k, 'same')


def medfilt(x, k):
    h = k // 2
    xp = np.pad(x, h, mode='edge')
    return np.array([np.median(xp[i:i + k]) for i in range(len(x))])


def segments(mask):
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    return np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)


def report_noise(files):
    print('== 1. 止まったときのセンサのノイズ(NOISE、PWMに合わせて読む) ==')
    for fn in files:
        stats = {}
        for line in open(fn, encoding='utf-8'):
            p = line.strip().split(',')
            # stat,読み方,待ち,取り込み,吸引,センサ,on|off|diff,平均,標準偏差,最小,最大[,PWMに合わせるか]
            if p[0] == 'stat' and p[6] == 'diff' and len(p) >= 12 and p[11] == '1':
                stats[(int(p[4]), p[5])] = (float(p[7]), float(p[8]))
        if not stats:
            continue
        print(fn)
        print('  吸引 センサ  値の平均  標準偏差  距離[mm]  1mmあたり  標準偏差[mm]')
        for (suc, s), (mean, sd) in sorted(stats.items()):
            k = slope_at(s, mean)
            print('  %3d  %-3s  %7.1f  %6.2f  %7.1f  %7.1f  %8.2f' % (suc, s, mean, sd, mm_at(s, mean), k, sd / k))
    print('  前の左右を境目で見るとき(前壁まで約174mm、値は約140)の 1mm あたり: FL %.1f / FR %.1f'
          % (slope_at('FL', 140), slope_at('FR', 140)))
    print()


def report_running_noise(logs):
    print('== 2. 走っているときの横センサのノイズ(1msおきのログ、直線、壁ありの側) ==')
    print('  ログ              速度  左:値 / mm   右:値 / mm   左の跳ね(40以上)')
    for fn, meta, d in logs:
        if meta.get('div', 1) != 1:
            continue
        st = (d['phase'] == 1) & (abs(d['w']) < 80) & (d['v'] > 300) & (abs(d['v'] - d['v_target']) < 150)
        out = []
        for sk, dk in (('sensor_L', 'dist_L'), ('sensor_R', 'dist_R')):
            s = st & (d[dk] < 100)
            if s.sum() < 100:
                out = None
                break
            hs = d[sk] - medfilt(d[sk], 9)
            hd = d[dk] - medfilt(d[dk], 9)
            out.append((hs[s].std(), hd[s].std(), (abs(hs[s]) >= 40).sum()))
        if out:
            v = np.median(d['v'][st])
            print('  %s %5.0f  %5.1f / %.2f  %5.1f / %.2f   %d' % (fn[-19:-8], v, out[0][0], out[0][1],
                                                              out[1][0], out[1][1], out[0][2]))
    print()


def report_battery(logs):
    print('== 3. 電池の内部抵抗(配線込み)とモーターの張り付き ==')
    print('  ログ           E0[V]  内部抵抗[Ω]  R^2  電流の最大[A]  電池の最低[V]  張り付き[ms](直線/ターン/そのほか)')
    rs = []
    e0s = []
    for fn, meta, d in logs:
        dt = meta['dt']
        w = np.radians(d['w'])
        v = d['v'] / 1000
        b = d['batt']
        cur = 0
        for key, vw in (('volt_L', v - w * TREAD / 2), ('volt_R', v + w * TREAD / 2)):
            vm = d[key] / 200 * b
            cur = cur + d[key] / 200 * (vm - KEMF * vw) / RM
        s = (d['phase'] > 0) & (d['phase'] < 9)
        if s.sum() < 50:
            continue
        a = np.c_[np.ones(s.sum()), cur[s]]
        c = np.linalg.lstsq(a, b[s], rcond=None)[0]
        r = b[s] - a @ c
        r2 = 1 - r.var() / b[s].var()
        sat = (abs(d['volt_L']) >= SAT) | (abs(d['volt_R']) >= SAT)
        ph = d['phase']
        st = (sat & np.isin(ph, (1, 2))).sum() * dt * 1000
        tu = (sat & (ph == 4)).sum() * dt * 1000
        ot = max(0.0, sat.sum() * dt * 1000 - st - tu)
        print('  %s  %5.2f  %6.2f  %5.2f  %6.2f  %7.2f   %4.0f / %4.0f / %4.0f'
              % (fn[-19:-8], c[0], -c[1], r2, cur[s].max(), b[s].min(), st, tu, ot))
        if cur[s].max() > 3 and r2 > 0.85:
            rs.append(-c[1])
            e0s.append(c[0])
    print()
    if rs:
        print('  電流が 3A を超えるログ(%d本)の内部抵抗: 中央値 %.2fΩ(%.2f〜%.2f)' % (len(rs), np.median(rs), min(rs), max(rs)))
        print()
        return float(np.median(e0s)), float(np.median(rs))
    return None


def fit_rotation(logs):
    a = []
    y = []
    for fn, meta, d in logs:
        dt = meta['dt']
        w = np.radians(d['w'])
        wd = np.gradient(smooth(w, max(1, int(round(0.004 / dt)))), dt)
        dv = (d['volt_R'] - d['volt_L']) / 400 * d['batt']
        ok = (abs(d['volt_L']) < 195) & (abs(d['volt_R']) < 195) & (d['v'] > 100)
        a.append(np.c_[w, wd][ok])
        y.append(dv[ok])
    a = np.vstack(a)
    y = np.concatenate(y)
    if len(y) < 100 or np.degrees(abs(a[:, 0])).max() < 500:
        return None, 0  # ターンのないログ(直進だけ)では求まらない
    c = np.linalg.lstsq(a, y, rcond=None)[0]
    return c, (y - a @ c).std()


def report_turns(logs):
    c, sd = fit_rotation(logs)
    print('== 4. ターンごとの電圧の余裕 ==')
    if c is None:
        print('  回転の係数: このログにはターンがないので求め直さない(決めてある値 KW %.4f、KJ %.5f)' % (KW, KJ))
    else:
        print('  回転の係数(このログで求め直した値): KW %.4f V/(rad/s)、KJ %.5f V/(rad/s^2)(決めてある値 %.4f、%.5f)、残差 %.2fV'
              % (c[0], c[1], KW, KJ, sd))
    print('  必要な電圧 = %.2f + %.2f v + %.3f w + %.4f wdot(外側の車輪、目標の w から)' % (C, KE, KW, KJ))
    for fn, meta, d in logs:
        dt = meta['dt']
        ph = d['phase']
        gs = segments(ph == 4)
        if not gs or d['v'].max() < 1500:
            continue
        print(fn)
        print('    角度  速度  w目標[dps]  角加速度[deg/s^2]  必要[V]  電池[V]  余裕[V]  張り付き[ms]  wの遅れ[dps]  横G[m/s^2]')
        for g in gs:
            g = np.arange(max(0, g[0] - 2), min(len(ph), g[-1] + 3))
            wt = np.radians(abs(d['w_target'][g]))
            v = d['v'][g] / 1000
            wtd = np.gradient(wt, dt)
            need = C + KE * v + KW * wt + KJ * wtd
            k = int(np.argmax(need))
            ang = abs(np.sum(d['w'][g]) * dt)
            sat = ((abs(d['volt_L'][g]) >= SAT) | (abs(d['volt_R'][g]) >= SAT)).sum() * dt * 1000
            lag = (abs(d['w_target'][g]) - abs(d['w'][g])).max()
            ay = (v * np.radians(abs(d['w'][g]))).max()
            print('    %4.0f  %4.0f  %6.0f  %9.0f  %7.1f  %6.2f  %+6.1f  %5.0f  %6.0f  %6.1f'
                  % (ang, v[0] * 1000, np.degrees(wt.max()), np.degrees(wtd.max()), need[k], d['batt'][g][k],
                     d['batt'][g][k] - need[k], sat, lag, ay))
    print()


def vb_full(v, e0, r, ifan=0.5):
    """両輪とも上限のときの電池の電圧: Vb = E0 - r*(2*(Vb - KEMF*v)/RM + 吸引の電流)"""
    return (e0 + r * (2 * KEMF * v / RM - ifan)) / (1 + 2 * r / RM)


def accel_max(v, e0, r, margin=1.0):
    """電圧から決まる直線の加速の上限[m/s^2]。margin は速度・横壁制御のFBに残す電圧"""
    return max(0.0, (vb_full(v, e0, r) - margin - C - KE * v) / KA)


def report_accel(bat=None):
    print('== 5. 電圧から決まる直線の加速の上限(FBに1V残す) ==')
    if bat is None:
        print('  速度[m/s]  今の電池(0.68Ω)  0.3Ω  内部抵抗0(電圧が下がらない)')
        for v in range(0, 7):
            print('  %d   %5.1f  %5.1f  %5.1f   (電池 %.1fV)' % (v, accel_max(v, 15.8, 0.68), accel_max(v, 15.8, 0.3),
                                                       accel_max(v, 15.8, 1e-4), vb_full(v, 15.8, 0.68)))
    else:
        e0, r = bat
        print('  速度[m/s]  このログの電池(E0 %.2fV、%.2fΩ)  以前の電池(15.8V、0.68Ω)  内部抵抗0' % (e0, r))
        for v in range(0, 7):
            print('  %d   %5.1f (電池 %.1fV)  %5.1f  %5.1f' % (v, accel_max(v, e0, r), vb_full(v, e0, r),
                                                    accel_max(v, 15.8, 0.68), accel_max(v, e0, 1e-4)))
    print()


def report_straight_accel(logs, bat=None):
    """直線(phase 1)で速度が上がっているところの、実際の加速と張り付きを速度 0.5m/s ごとに"""
    rows = []
    for fn, meta, d in logs:
        dt = meta['dt']
        if 'v_target' not in d:
            continue
        v = smooth(d['v'], max(3, int(round(0.009 / dt)))) / 1000
        a = np.gradient(v, dt)
        at = np.gradient(d['v_target'] / 1000, dt)
        sat = (abs(d['volt_L']) >= SAT) & (abs(d['volt_R']) >= SAT)
        s = (d['phase'] == 1) & (at > 1) & (v > 0.2)
        for i in np.flatnonzero(s):
            rows.append((v[i], a[i], at[i], sat[i], d['batt'][i]))
    if not rows:
        return
    print('== 6. 直線で実際に出た加速(速度が上がっているところ。1msおきのログのほうが確か) ==')
    print('  張り付いているところの加速が、今の電池での上限。「5」の上限(FBに1V残す)より少し上に出るはず')
    print('  速度[m/s]  点数  張り付き[%]  目標の加速  実際の加速(張り付き)  電池[V]  「5」の上限')
    r = np.array(rows, dtype=float)
    for lo in np.arange(0, 7, 0.5):
        m = (r[:, 0] >= lo) & (r[:, 0] < lo + 0.5)
        if m.sum() < 5:
            continue
        ms = m & (r[:, 3] > 0)
        act = '%5.1f' % np.median(r[ms, 1]) if ms.sum() >= 3 else '    -'
        lim = '%5.1f' % accel_max(lo + 0.25, *bat) if bat else '    -'
        print('  %3.1f〜%3.1f  %4d  %5.0f  %6.1f  %s  %6.2f  %s'
              % (lo, lo + 0.5, m.sum(), 100 * r[ms, 0].size / m.sum(), np.median(r[m, 2]), act,
                 np.median(r[m, 4]), lim))
    print()


def main():
    args = sys.argv[1:] or sorted(glob.glob('logs/*.txt'))
    noise = [f for f in args if f.endswith('_noise.txt')]
    logs = []
    for f in args:
        if f.endswith('_log.txt'):
            meta, d = load_log(f)
            if d is not None:
                logs.append((f, meta, d))
    if noise:
        report_noise(noise)
    bat = None
    if logs:
        report_running_noise(logs)
        bat = report_battery(logs)
        report_turns(logs)
    report_accel(bat)
    if logs:
        report_straight_accel(logs, bat)


if __name__ == '__main__':
    main()
