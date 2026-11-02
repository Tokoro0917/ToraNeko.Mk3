#!/usr/bin/env python3
"""最短走行の命令列(G_Short_Pass_NANAME)から、走る時間を見積もる(2026-10-26)。

ダイクストラのコストの重みを KOGUMA の実際の速さに合わせるための道具。
機体(Move.c の Short_NANAME_Move2000 / Short_Dijkstra_Move2000)の動きを、次のように数える:
  - 走り出し: 最初が直線なら 0 から V まで 114mm(スタート位置から区画の境目)で加速し、その半区画を走ったことにする。
    最初がターンなら 10mm で V にする(Short_Start / Short_Start_NoFirstTurn)
  - 直線(正の数 n): 90mm × n を V → 最高速 → V。加速 AC、減速 2×AC(Motor_trapezoid_Asymmetric_PID)
  - 斜めの直線(-50 × n): 127.3mm × n を V → 最高速 → V。加速・減速とも AC - 5000(Motor_NANAME_PID)
  - ターン: Turn-Sim と同じ計算で、理想の始まりの点から終わりの点まで速度 V で回る時間。
    角速度は cos 型の加速 → 等角速度 → cos 型の減速、前後の直進は理想の線に乗る長さ(滑りなし)。
    壁切れの位置や前距離・後距離の値には左右されない(ターンの形だけで決まる)
  - 止まる: 最後に V から 30000 で減速して、命令列が境目で終わるなら 270mm、区画の中心で終わるなら 180mm 先で止まる

使い方:
  python3 tools/route_time.py "1 1 -53 -50 -66 ..."          # 2000、最高速 6000、加速 20000(モード1 No.5)
  python3 tools/route_time.py --v 2400 --max 6000 --ac 20000 "..."
  モジュールとして: from route_time import route_time
"""
import math
import sys

D2R = math.pi / 180
C = 180.0           # 区画[mm]
DIAG = 127.3        # 斜め1区間[mm]

# ターンの値 (角度, 最大角速度, 角加速度)。Move.c の G_Turn2000 / G_Turn2400(2026-10-21 に合わせた値)
TURNS = {
	2000: {'big90': (90, 2000, 40000), 'big180': (180, 1280, 45000), 'in45': (45, 1900, 120000),
	       'in135': (135, 1500, 80000), 'out45': (45, 1100, 50000), 'out135': (135, 1420, 80000),
	       'v90': (90, 2000, 130000)},
	2400: {'big90': (90, 1600, 80000), 'big180': (180, 1470, 70000), 'in45': (45, 2500, 160000),
	       'in135': (135, 2000, 55000), 'out45': (45, 1320, 72000), 'out135': (135, 2100, 80000),
	       'v90': (90, 2600, 150000)},
}

CODE = {-4: 'big90', -6: 'big90', -5: 'big180', -7: 'big180', -51: 'in45', -53: 'in45', -52: 'in135',
        -54: 'in135', -61: 'out45', -63: 'out45', -62: 'out135', -64: 'out135', -65: 'v90', -66: 'v90'}

# 理想の始まり(位置と向き)と終わりの点(Turn-Sim の geometry と同じ。左ターン)
_GEOM = {
	'big90': ((0, 0, 90), (-C, C)), 'big180': ((0, 0, 90), (-C, 0)), 'in45': ((0, 0, 90), (-C / 2, C)),
	'in135': ((0, 0, 90), (-C, C / 2)), 'out45': ((C / 2, C, 225), (0, 0)),
	'out135': ((C, C / 2, 135), (0, 0)), 'v90': ((C / 2, 0, 45), (C / 2, C)),
}


def _omega(angle, wmax, wac):
	x13 = math.pi / (2 * wac) * wmax * wmax
	if x13 > angle:
		wmax = math.sqrt(2 * wac * angle / math.pi)
		x13 = angle
	t1 = math.pi * wmax / (2 * wac)
	tc = (angle - x13) / wmax
	return wmax, t1, tc


def _arc_xy(v, angle, wmax, wac, n=4000):
	"""速度 v で cos 型の角速度で angle まで回ったときの、始まりからの位置(前, 左)と時間"""
	wmax, t1, tc = _omega(angle, wmax, wac)
	T = 2 * t1 + tc
	x = y = th = 0.0
	h = T / n
	for i in range(n):
		t = (i + 0.5) * h
		if t < t1:
			w = wmax / 2 * (1 - math.cos(math.pi * t / t1))
		elif t < t1 + tc:
			w = wmax
		else:
			w = wmax / 2 * (1 + math.cos(math.pi * (t - t1 - tc) / t1))
		thm = th + w * D2R * h / 2
		x += v * math.cos(thm) * h
		y += v * math.sin(thm) * h
		th += w * D2R * h
	return x, y, T


_turn_cache = {}


def turn_time(kind, v, table=None):
	"""ターンの理想の始まりの点から終わりの点までの時間[s]。前後の直進は理想の線に乗る長さ"""
	key = (kind, v, table and tuple(sorted(table.items())))
	if key in _turn_cache:
		return _turn_cache[key]
	angle, wmax, wac = (table or TURNS[v])[kind]
	(sx, sy, sa), (ex, ey) = _GEOM[kind]
	a = sa * D2R
	dx, dy = ex - sx, ey - sy
	lx, ly = dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)	# 始まりの向きで見た終点
	ang_end = {'big90': 90, 'big180': 180, 'in45': 45, 'in135': 135, 'out45': 45, 'out135': 135, 'v90': 90}[kind]
	ax, ay, T = _arc_xy(v, angle, wmax, wac)
	ae = ang_end * D2R
	s = math.sin(ae)
	if abs(s) < 1e-6:	# 180°: 前後は平行なので、前の直進を0にして後ろで合わせる
		st, end = 0.0, max(0.0, ax - lx)
	else:
		end = (ly - ay) / s
		st = (lx - ax) - end * math.cos(ae)
	t = (max(st, 0) + max(end, 0)) / v + T
	_turn_cache[key] = t
	return t


def _trap_time(L, v0, v1, vmax, ac, dc):
	"""cos 型の加減速で、距離 L を v0 → (最高 vmax) → v1。加速 ac、減速 dc"""
	def dist(va, vb, a):
		return math.pi * abs(vb * vb - va * va) / (4 * a)

	def tm(va, vb, a):
		return math.pi * abs(vb - va) / (2 * a)
	vp = vmax
	if dist(v0, vp, ac) + dist(vp, v1, dc) > L:
		lo, hi = max(v0, v1), vmax
		for _ in range(60):
			m = (lo + hi) / 2
			if dist(v0, m, ac) + dist(m, v1, dc) > L:
				hi = m
			else:
				lo = m
		vp = lo
	rest = L - dist(v0, vp, ac) - dist(vp, v1, dc)
	return tm(v0, vp, ac) + tm(vp, v1, dc) + max(rest, 0) / max(vp, 1)


def route_time(tokens, v=2000.0, vmax=6000.0, ac=20000.0, ac_naname=None, table=None, detail=False):
	"""命令列(G_Short_Pass_NANAME と同じ形)の時間[s]。detail=True なら (合計, [(命令, 時間)]) を返す"""
	tokens = [t for t in tokens if t != 0]
	acn = ac_naname if ac_naname else ac - 5000
	parts = []
	first = next((t for t in tokens if t != -1), None)
	if first is not None and first >= 1:	# 最初が直線: 114mm で 0 → v、その半区画を走ったことにする
		parts.append(('start', 2 * 114.0 / v))
		i = tokens.index(first)
		tokens = list(tokens)
		tokens[i] = first - 1 if first > 1 else -1
	else:
		parts.append(('start', math.pi * v / (2 * 70000.0)))
	for t in tokens:
		if t == -1:
			continue
		if t > 0:
			parts.append((t, _trap_time(90.0 * t, v, v, vmax, ac, 2 * ac)))
		elif t <= -50 and t % 50 == 0:
			parts.append((t, _trap_time(DIAG * (-t // 50), v, v, vmax, acn, acn)))
		else:
			parts.append((t, turn_time(CODE[t], v, table)))
	last = next((t for t in reversed(tokens) if t != -1), 0)
	goal_x = 270.0 if last > 0 else 180.0
	dec = math.pi * v * v / (4 * 30000.0)
	parts.append(('stop', (goal_x - dec) / v + math.pi * v / (2 * 30000.0)))
	total = sum(p[1] for p in parts)
	return (total, parts) if detail else total


if __name__ == '__main__':
	import argparse
	ap = argparse.ArgumentParser()
	ap.add_argument('tokens')
	ap.add_argument('--v', type=float, default=2000)
	ap.add_argument('--max', type=float, default=6000)
	ap.add_argument('--ac', type=float, default=20000)
	a = ap.parse_args()
	tot, parts = route_time([int(x) for x in a.tokens.split()], a.v, a.max, a.ac, detail=True)
	for k, t in parts:
		print(f'{str(k):>6} {t * 1000:7.1f} ms')
	print(f'合計 {tot:.3f} s')
