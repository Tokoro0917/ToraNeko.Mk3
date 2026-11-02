/* フェイルセーフで止まったときに、最後に壁を書いた5区画を戻す(Maze.c の Maze_Undo_Recent_Cells、2026-10-27)のPC上での検証。
 * シミュレータ(sim/sim_main.c の sim_step。Maze.c をそのまま使う)でランダム迷路を探索させ、
 * いろいろな歩数で止めて、
 *   - 戻したあとの壁・見たかどうか・G_MAZE_Explored が、最後の5回(歩数が5より少なければ全部)の
 *     Maze_Wall_Update の前と同じか
 *   - 2回続けて呼んでも、2回目は何も戻さないか
 *   - Maze_Undo_Clear() のあとに書いた壁だけを戻すか(探索を始める前の壁は戻さない)
 * を確かめる。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -DSIM_DEBUG -I Core/Inc -I . -I sim -o /tmp/mu test/test_maze_undo.c \
 *       Core/Src/Maze.c Core/Src/ShortParam.c -lm && /tmp/mu */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* シミュレータの迷路(sim/sim_maze_data.h)の代わりに、ここで作る迷路を使う */
#define SIM_MAZE_DATA_H_
#define SIM_MAZE_SIZE 16
#define SIM_GOAL_X 7
#define SIM_GOAL_Y 7
static uint32_t TRUE_Maze_Row[SIM_MAZE_SIZE + 1], TRUE_Maze_Column[SIM_MAZE_SIZE + 1];

#define main sim_main
#include "sim/sim_main.c"
#undef main

extern uint32_t Maze_Row_Look[], Maze_Column_Look[];

#define S SIM_MAZE_SIZE
#define UNDO_N 5

/* 迷路の状態の写し */
typedef struct {
	uint32_t row[S + 1], col[S + 1], row_look[S + 1], col_look[S + 1];
	uint8_t explored[S][S];
} Snap;

static void snap_take(Snap *s) {
	memcpy(s->row, G_Maze_Row, sizeof(s->row));
	memcpy(s->col, G_Maze_Column, sizeof(s->col));
	memcpy(s->row_look, Maze_Row_Look, sizeof(s->row_look));
	memcpy(s->col_look, Maze_Column_Look, sizeof(s->col_look));
	memcpy(s->explored, G_MAZE_Explored, sizeof(s->explored));
}

static int snap_same(const Snap *a, const Snap *b) {
	return memcmp(a, b, sizeof(Snap)) == 0;
}

/* 迷路の作り方は test_unknown_search.c と同じ */
/* 壁を開ける: 区画(x,y)から向き d(0 北、1 東、2 南、3 西) */
static void open_wall(int x, int y, int d) {
	if (d == 0)
		TRUE_Maze_Row[y + 1] &= ~(1u << x);
	else if (d == 1)
		TRUE_Maze_Column[x + 1] &= ~(1u << y);
	else if (d == 2)
		TRUE_Maze_Row[y] &= ~(1u << x);
	else
		TRUE_Maze_Column[x] &= ~(1u << y);
}

/* 穴掘り法の全域木 + ループ。straight%(0〜100)の割合で前と同じ向きに掘り進む(大会の迷路のような長い通路)。
 * スタートは北だけ開いていて、ゴールの2x2区画の中は壁なし(sim/gen_maze.py と同じ) */
static void make_maze(unsigned seed, int loops, int straight) {
	static const int dx[4] = { 0, 1, 0, -1 }, dy[4] = { 1, 0, -1, 0 };
	static int seen[S][S], stx[S * S], sty[S * S];
	srand(seed);
	for (int k = 0; k <= S; k++) {
		TRUE_Maze_Row[k] = (1u << S) - 1;
		TRUE_Maze_Column[k] = (1u << S) - 1;
	}
	memset(seen, 0, sizeof(seen));
	static int dir_in[S][S];
	int sp = 0;
	stx[sp] = 0;
	sty[sp++] = 0;
	seen[0][0] = 1;
	dir_in[0][0] = 0;
	while (sp > 0) {
		int x = stx[sp - 1], y = sty[sp - 1], cand[4], nc = 0, same = -1;
		for (int d = 0; d < 4; d++) {
			int nx = x + dx[d], ny = y + dy[d];
			if (nx >= 0 && ny >= 0 && nx < S && ny < S && !seen[nx][ny]) {
				if (d == dir_in[x][y])
					same = d;
				cand[nc++] = d;
			}
		}
		if (nc == 0) {
			sp--;
			continue;
		}
		int d = (same >= 0 && rand() % 100 < straight) ? same : cand[rand() % nc];
		open_wall(x, y, d);
		seen[x + dx[d]][y + dy[d]] = 1;
		dir_in[x + dx[d]][y + dy[d]] = d;
		stx[sp] = x + dx[d];
		sty[sp++] = y + dy[d];
	}
	for (int k = 0; k < loops; k++) {	//ループ(外周は開けない)
		int x = rand() % S, y = rand() % S, d = rand() % 4;
		int nx = x + dx[d], ny = y + dy[d];
		if (nx >= 0 && ny >= 0 && nx < S && ny < S)
			open_wall(x, y, d);
	}
	/* ゴールの2x2区画の中は壁なし */
	open_wall(SIM_GOAL_X, SIM_GOAL_Y, 0);
	open_wall(SIM_GOAL_X, SIM_GOAL_Y, 1);
	open_wall(SIM_GOAL_X + 1, SIM_GOAL_Y + 1, 2);
	open_wall(SIM_GOAL_X + 1, SIM_GOAL_Y + 1, 3);
	/* スタート(0,0)は北だけ開いている(Maze_Initialization() と同じ) */
	TRUE_Maze_Column[1] |= 1u;
	open_wall(0, 0, 0);
}

/* スタートから探索を始める(main.c の Maze_Search と同じ: Maze_Undo_Clear のあと、1区画進んで探索) */
static void start_search(void) {
	Maze_Initialization();
	G_Gool_X = SIM_GOAL_X;
	G_Gool_Y = SIM_GOAL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	Maze_Undo_Clear();
}

/* 1区画ぶん探索する(main.c の Maze_Search_Until_Goal と同じ: 動いたあとに G_MAZE_Explored を立てる) */
static void step(void) {
	sim_step();
	G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
}

static int fails = 0;
static void check(int ok, const char *what, unsigned seed, int steps) {
	if (!ok) {
		if (fails < 10) {
			printf("NG: %s (seed %u, %d 歩)\n", what, seed, steps);
		}
		fails++;
	}
}

int main(int argc, char **argv) {
	int n_maze = (argc > 1) ? atoi(argv[1]) : 200;
	static Snap before[400];
	int cases = 0;
	for (unsigned seed = 1; seed <= (unsigned) n_maze; seed++) {
		make_maze(seed, 20, 50);
		int stop_at = 1 + (int) (seed * 7 % 120);	//この歩数で止める(フェイルセーフ)
		start_search();
		int steps = 0;
		while (steps < stop_at) {
			snap_take(&before[steps]);	//steps 回目の Maze_Wall_Update の前
			step();
			steps++;
		}
		Snap after_undo, again;
		int n = Maze_Undo_Recent_Cells();
		snap_take(&after_undo);
		int back = (steps < UNDO_N) ? steps : UNDO_N;
		check(n == back, "戻した回数", seed, steps);
		check(snap_same(&after_undo, &before[steps - back]), "最後の5回の前と同じに戻る", seed, steps);
		check(Maze_Undo_Recent_Cells() == 0, "2回目は何も戻さない", seed, steps);
		snap_take(&again);
		check(snap_same(&after_undo, &again), "2回目で迷路が変わらない", seed, steps);

		/* 続けて探索をやり直す(電源を切らずに): Maze_Undo_Clear のあとの壁だけを戻す */
		Snap kept;
		G_Robot_MAZE_X = 0;
		G_Robot_MAZE_Y = 0;
		G_Robot_Direction = 0;
		Maze_Step_Calculate();
		Maze_Undo_Clear();
		snap_take(&kept);
		step();
		step();
		check(Maze_Undo_Recent_Cells() == 2, "やり直したあとは、その2回だけ戻す", seed, steps);
		snap_take(&again);
		check(snap_same(&kept, &again), "やり直す前の壁は戻さない", seed, steps);
		cases++;
	}
	printf("maze undo: %d cases, %d failed\n", cases, fails);
	return fails ? 1 : 0;
}
