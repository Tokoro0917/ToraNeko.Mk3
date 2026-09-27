#!/usr/bin/env python3
"""Generate a verified-solvable 16x16 micromouse maze and emit it as C
uint32_t bit arrays matching the exact convention used by G_Maze_Row /
G_Maze_Column in Core/Src/Maze.c:

  G_Maze_Row[y]    bit x set  -> wall on the horizontal boundary between
                                  cell (x, y-1) and cell (x, y)
  G_Maze_Column[x] bit y set  -> wall on the vertical boundary between
                                  cell (x-1, y) and cell (x, y)

Arrays are sized MAZE_SIZE+1 (0..MAZE_SIZE inclusive), same as the real
firmware. This script only produces the *ground-truth* maze used by the
simulator's virtual sensor; it does not touch anything under Core/.
"""
import random

SIZE = 16
GOAL_X, GOAL_Y = 7, 7  # 2x2 goal room occupies (7,7),(8,7),(7,8),(8,8)

random.seed(10)  # fixed seed for a reproducible, pre-verified maze

# wall_row[y][x]    -> True if there's a wall between (x,y-1) and (x,y), y in 0..SIZE
# wall_col[x][y]    -> True if there's a wall between (x-1,y) and (x,y), x in 0..SIZE
wall_row = [[True] * SIZE for _ in range(SIZE + 1)]
wall_col = [[True] * SIZE for _ in range(SIZE + 1)]

def neighbors(x, y):
    result = []
    if x > 0:
        result.append(("W", x - 1, y))
    if x < SIZE - 1:
        result.append(("E", x + 1, y))
    if y > 0:
        result.append(("S", x, y - 1))
    if y < SIZE - 1:
        result.append(("N", x, y + 1))
    return result  # each item is (direction, nx, ny)

def open_wall(x, y, direction, nx, ny):
    if direction == "N":
        wall_row[y + 1][x] = False
    elif direction == "S":
        wall_row[y][x] = False
    elif direction == "E":
        wall_col[x + 1][y] = False
    elif direction == "W":
        wall_col[x][y] = False

# Randomized DFS (recursive backtracker) starting at (0,0) -> spanning tree,
# guarantees every cell is reachable.
visited = [[False] * SIZE for _ in range(SIZE)]
stack = [(0, 0)]
visited[0][0] = True
while stack:
    x, y = stack[-1]
    candidates = [n for n in neighbors(x, y) if not visited[n[2]][n[1]]]
    if not candidates:
        stack.pop()
        continue
    direction, nx, ny = random.choice(candidates)
    open_wall(x, y, direction, nx, ny)
    visited[ny][nx] = True
    stack.append((nx, ny))

assert all(all(row) for row in visited), "spanning tree failed to cover every cell"

# Force the start cell (0,0) to have exactly its north side open, matching
# Maze_Initialization()'s hardcoded assumption in Core/Src/Maze.c.
wall_row[0][0] = True   # south of (0,0): wall (outer boundary anyway)
wall_col[0][0] = True   # west of (0,0): wall (outer boundary anyway)
wall_col[1][0] = True   # east of (0,0): forced wall
wall_row[1][0] = False  # north of (0,0): forced open

# Force the 2x2 goal room to be a single open room (no interior walls),
# matching how a real competition maze's center square is built.
wall_col[GOAL_X + 1][GOAL_Y] = False       # between (7,7)-(8,7)
wall_col[GOAL_X + 1][GOAL_Y + 1] = False   # between (7,8)-(8,8)
wall_row[GOAL_Y + 1][GOAL_X] = False       # between (7,7)-(7,8)
wall_row[GOAL_Y + 1][GOAL_X + 1] = False   # between (8,7)-(8,8)

# Add extra loop-openings (beyond the spanning tree) so there are genuine
# alternative routes for the Dijkstra shuttle search to discover/reject,
# not just one single forced path.
interior_walls = []
for y in range(SIZE):
    for x in range(1, SIZE):
        if wall_col[x][y]:
            interior_walls.append(("col", x, y))
for y in range(1, SIZE):
    for x in range(SIZE):
        if wall_row[y][x]:
            interior_walls.append(("row", y, x))

random.shuffle(interior_walls)
EXTRA_OPENINGS = 22
opened = 0
for kind, a, b in interior_walls:
    if opened >= EXTRA_OPENINGS:
        break
    if kind == "col":
        x, y = a, b
        if wall_col[x][y]:
            wall_col[x][y] = False
            opened += 1
    else:
        y, x = a, b
        if wall_row[y][x]:
            wall_row[y][x] = False
            opened += 1

# Force the outer boundary solid (should already be true, since the DFS
# above never opens an edge leading outside the grid, but keep this
# explicit and authoritative).
for x in range(SIZE):
    wall_row[0][x] = True
    wall_row[SIZE][x] = True
for y in range(SIZE):
    wall_col[0][y] = True
    wall_col[SIZE][y] = True
wall_row[1][0] = False  # re-assert start-cell opening after boundary pass

# --- Verification -----------------------------------------------------
def bfs_reachable(sx, sy):
    seen = [[False] * SIZE for _ in range(SIZE)]
    seen[sy][sx] = True
    q = [(sx, sy)]
    while q:
        x, y = q.pop()
        for direction, nx, ny in neighbors(x, y):
            if seen[ny][nx]:
                continue
            blocked = False
            if direction == "N":
                blocked = wall_row[y + 1][x]
            elif direction == "S":
                blocked = wall_row[y][x]
            elif direction == "E":
                blocked = wall_col[x + 1][y]
            elif direction == "W":
                blocked = wall_col[x][y]
            if not blocked:
                seen[ny][nx] = True
                q.append((nx, ny))
    return seen

reach = bfs_reachable(0, 0)
assert all(all(row) for row in reach), "maze is not fully connected from (0,0) after loop-opening pass"
assert reach[GOAL_Y][GOAL_X], "goal cell not reachable from start"

# Confirm (0,0) really has exactly one open side (north).
open_sides = 0
if not wall_row[1][0]:
    open_sides += 1
if not wall_row[0][0]:
    open_sides += 1
if not wall_col[0][0]:
    open_sides += 1
if not wall_col[1][0]:
    open_sides += 1
assert open_sides == 1, f"start cell should have exactly 1 open side, has {open_sides}"

# --- Emit as C arrays ---------------------------------------------------
def row_bits(y):
    v = 0
    for x in range(SIZE):
        if wall_row[y][x]:
            v |= (1 << x)
    return v

def col_bits(x):
    v = 0
    for y in range(SIZE):
        if wall_col[x][y]:
            v |= (1 << y)
    return v

print("/* Auto-generated by sim/gen_maze.py -- do not hand-edit. */")
print("#ifndef SIM_MAZE_DATA_H_")
print("#define SIM_MAZE_DATA_H_")
print()
print("#include <stdint.h>")
print()
print(f"#define SIM_MAZE_SIZE {SIZE}")
print(f"#define SIM_GOAL_X {GOAL_X}")
print(f"#define SIM_GOAL_Y {GOAL_Y}")
print()
print(f"static const uint32_t TRUE_Maze_Row[{SIZE + 1}] = {{")
print("\t" + ", ".join(str(row_bits(y)) for y in range(SIZE + 1)))
print("};")
print()
print(f"static const uint32_t TRUE_Maze_Column[{SIZE + 1}] = {{")
print("\t" + ", ".join(str(col_bits(x)) for x in range(SIZE + 1)))
print("};")
print()
print("#endif /* SIM_MAZE_DATA_H_ */")
