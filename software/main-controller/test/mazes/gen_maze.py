#!/usr/bin/env python3
"""テスト用の迷路（32x32）を作って、テキストファイルに書き出す。

書き出す形式（マイクロマウスでよく使われる形式）:
    o---o---o
    | G     |
    o   o---o
  - 'o' は柱、'---' は横の壁、'|' は縦の壁
  - いちばん上の行が迷路の北側（y が大きい側）、いちばん下の行がスタートのある南側
  - 区画の中の 'S' はスタート、'G' はゴール

使い方:  python3 gen_maze.py   （このフォルダに maze32_*.txt ができる）
乱数の種を固定しているので、何度実行しても同じ迷路ができる。
"""
import random
from collections import deque

SIZE = 32

# (ファイル名, 乱数の種, ゴールの左下 x, y, ゴールの幅, 高さ, ループを作るために壊す壁の割合)
MAZES = [
    ("maze32_center2x2.txt", 1, 15, 15, 2, 2, 0.10),
    ("maze32_offset3x3.txt", 2, 5, 20, 3, 3, 0.15),
]


def make_maze(seed, goal_x, goal_y, goal_w, goal_h, loop_ratio):
    random.seed(seed)
    # north[x][y]: (x,y) の北に壁があるか。east[x][y]: (x,y) の東に壁があるか
    north = [[True] * SIZE for _ in range(SIZE)]
    east = [[True] * SIZE for _ in range(SIZE)]

    def in_goal(x, y):
        return goal_x <= x < goal_x + goal_w and goal_y <= y < goal_y + goal_h

    def open_wall(x, y, d):
        if d == "N":
            north[x][y] = False
        elif d == "S":
            north[x][y - 1] = False
        elif d == "E":
            east[x][y] = False
        elif d == "W":
            east[x - 1][y] = False

    def neighbors(x, y):
        result = []
        if y + 1 < SIZE:
            result.append(("N", x, y + 1))
        if y - 1 >= 0:
            result.append(("S", x, y - 1))
        if x + 1 < SIZE:
            result.append(("E", x + 1, y))
        if x - 1 >= 0:
            result.append(("W", x - 1, y))
        return result

    # 1. 穴掘り法（深さ優先）で、全部の区画がつながった迷路を作る
    visited = [[False] * SIZE for _ in range(SIZE)]
    stack = [(0, 0)]
    visited[0][0] = True
    while stack:
        x, y = stack[-1]
        candidates = [(d, nx, ny) for d, nx, ny in neighbors(x, y) if not visited[nx][ny]]
        if not candidates:
            stack.pop()
            continue
        d, nx, ny = random.choice(candidates)
        open_wall(x, y, d)
        visited[nx][ny] = True
        stack.append((nx, ny))

    # 2. 壁をいくつか壊して、道が何通りもある迷路にする
    for x in range(SIZE):
        for y in range(SIZE):
            if y + 1 < SIZE and north[x][y] and random.random() < loop_ratio:
                north[x][y] = False
            if x + 1 < SIZE and east[x][y] and random.random() < loop_ratio:
                east[x][y] = False

    # 3. ゴールの中の壁をなくす
    for x in range(goal_x, goal_x + goal_w):
        for y in range(goal_y, goal_y + goal_h):
            if in_goal(x, y + 1):
                north[x][y] = False
            if in_goal(x + 1, y):
                east[x][y] = False

    # 4. スタート区画の東は必ず壁、北は必ず開いている（規則。東を閉じても閉じ込められないように北を開ける）
    #    test_maze.c の make_random_true_maze() と同じ
    east[0][0] = True
    north[0][0] = False

    # 5. スタートからゴールにたどり着けるか確かめる
    seen = [[False] * SIZE for _ in range(SIZE)]
    queue = deque([(0, 0)])
    seen[0][0] = True
    reached = False
    while queue:
        x, y = queue.popleft()
        if in_goal(x, y):
            reached = True
        moves = []
        if y + 1 < SIZE and not north[x][y]:
            moves.append((x, y + 1))
        if y - 1 >= 0 and not north[x][y - 1]:
            moves.append((x, y - 1))
        if x + 1 < SIZE and not east[x][y]:
            moves.append((x + 1, y))
        if x - 1 >= 0 and not east[x - 1][y]:
            moves.append((x - 1, y))
        for nx, ny in moves:
            if not seen[nx][ny]:
                seen[nx][ny] = True
                queue.append((nx, ny))
    assert reached, "ゴールにたどり着けない迷路ができた"
    return north, east, in_goal


def to_text(north, east, in_goal):
    lines = []
    for y in range(SIZE - 1, -1, -1):
        # 区画 y の北側の壁の行
        row = "o"
        for x in range(SIZE):
            if north[x][y]:
                row += "---o"
            else:
                row += "   o"
        lines.append(row)
        # 区画 y の行（西の外周 + 各区画の中身と東の壁）
        row = "|"
        for x in range(SIZE):
            if x == 0 and y == 0:
                mark = "S"
            elif in_goal(x, y):
                mark = "G"
            else:
                mark = " "
            row += " " + mark + " "
            if east[x][y]:
                row += "|"
            else:
                row += " "
        lines.append(row)
    lines.append("o" + "---o" * SIZE)  # 南の外周
    return "\n".join(lines) + "\n"


for name, seed, gx, gy, gw, gh, ratio in MAZES:
    north, east, in_goal = make_maze(seed, gx, gy, gw, gh, ratio)
    with open(name, "w") as f:
        f.write(to_text(north, east, in_goal))
    print("wrote", name)
