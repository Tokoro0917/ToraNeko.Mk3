#!/usr/bin/env python3
"""mazefiles 形式の迷路(micromouseonline/mazefiles の classic など)を、test/test_unknown_search.c の
--mazes で読む1行1迷路の形(横壁17個・縦壁17個の16進)にする(2026-10-27)。

16×16 で、ゴールが中央の2×2、スタートの東が壁、スタートからゴールへ行けるものだけ。壁が同じ迷路は1つにする。

使い方:
  git clone https://github.com/micromouseonline/mazefiles /tmp/mazefiles
  python3 tools/mazes_to_lines.py /tmp/mazefiles/classic/*.txt > /tmp/mazes.txt
  ./test_unknown_search --mazes /tmp/mazes.txt
"""
import sys
from mazefile import read_mazefile


def reachable(row, col):
	seen = {(0, 0)}
	st = [(0, 0)]
	while st:
		x, y = st.pop()
		for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0)):
			nx, ny = x + dx, y + dy
			if not (0 <= nx < 16 and 0 <= ny < 16) or (nx, ny) in seen:
				continue
			if (dy == 1 and row[y + 1] >> x & 1) or (dy == -1 and row[y] >> x & 1):
				continue
			if (dx == 1 and col[x + 1] >> y & 1) or (dx == -1 and col[x] >> y & 1):
				continue
			seen.add((nx, ny))
			st.append((nx, ny))
	return any(c in seen for c in ((7, 7), (8, 7), (7, 8), (8, 8)))


done = set()
for path in sys.argv[1:]:
	try:
		n, row, col, goals = read_mazefile(path)
	except Exception:
		continue
	if n != 16 or sorted(goals) != [(7, 7), (7, 8), (8, 7), (8, 8)] or not (col[1] & 1):
		continue
	key = (tuple(row), tuple(col))
	if key in done or not reachable(row, col):
		continue
	done.add(key)
	print(' '.join('%x' % v for v in row) + ' ' + ' '.join('%x' % v for v in col))
