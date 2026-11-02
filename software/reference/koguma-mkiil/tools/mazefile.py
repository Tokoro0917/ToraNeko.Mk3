#!/usr/bin/env python3
"""micromouseonline/mazefiles 形式(o---o の文字の迷路)を読み、Maze.c と同じ壁の持ち方にする(2026-10-26)。

  row[y] の bit x: 区画 (x, y-1) と (x, y) の間の横の壁(y = 0..N。0 と N は外周)
  col[x] の bit y: 区画 (x-1, y) と (x, y) の間の縦の壁
戻り値: (N, row, col, goals)。goals は 'G' の区画のリスト
"""


def read_mazefile(path):
	lines = [l.rstrip('\n') for l in open(path, encoding='utf-8', errors='replace')]
	lines = [l for l in lines if l.strip().startswith(('o', '|'))]
	n = (len(lines) - 1) // 2
	row = [0] * (n + 1)
	col = [0] * (n + 1)
	goals = []
	for k in range(n + 1):	# 柱の行: 上から k 本目 = 横の線 n-k
		l = lines[2 * k]
		for x in range(n):
			seg = l[4 * x + 1:4 * x + 4]
			if '-' in seg:
				row[n - k] |= 1 << x
	for k in range(n):	# 区画の行: 上から k 行目 = y = n-1-k
		l = lines[2 * k + 1]
		y = n - 1 - k
		for i in range(n + 1):
			if 4 * i < len(l) and l[4 * i] == '|':
				col[i] |= 1 << y
		for x in range(n):
			if 'G' in l[4 * x + 1:4 * x + 4]:
				goals.append((x, y))
	return n, row, col, goals


def to_dump(n, row, col, routes=None, header=()):
	"""Maze_Debug_Dump と同じ形の文字列(test_shortpath --dump で読める)"""
	out = ['# ' + h for h in header]
	out.append('MAZE_DUMP_BEGIN %d' % n)
	full = (1 << n) - 1
	out += ['ROW %d %08X' % (k, v) for k, v in enumerate(row)]
	out += ['COL %d %08X' % (k, v) for k, v in enumerate(col)]
	out += ['LROW %d %08X' % (k, full) for k in range(n + 1)]
	out += ['LCOL %d %08X' % (k, full) for k in range(n + 1)]
	out.append('MAZE_DUMP_END')
	for name, toks in (routes or {}).items():
		out.append('%s: %s' % (name, ' '.join(str(t) for t in toks)))
	return '\n'.join(out) + '\n'


if __name__ == '__main__':
	import sys
	n, row, col, goals = read_mazefile(sys.argv[1])
	print(n, goals)
	print('ROW', [hex(v) for v in row])
	print('COL', [hex(v) for v in col])
