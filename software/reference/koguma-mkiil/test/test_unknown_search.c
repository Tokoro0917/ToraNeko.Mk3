/* ダイクストラ誘導の未知壁探索(main.c モード0 No.6・7・10〜12)のPC上での検証(2026-10-27)。
 * シミュレータ(sim/sim_main.c のシナリオB。main.c と同じ手順で、Maze.c をそのまま使う)を
 * ランダム迷路で走らせ、
 *   - 探索が終わるか(シミュレータの上限を超えると sim_main.c が exit(1) する)
 *   - 迷路の外に出ないか
 *   - 探索で見た壁だけ(見ていない壁は壁)で求めた最短が、G_Short_Run_Set の全部の組で、
 *     本当の迷路の最短と同じ速さか(見ていない区間に、もっと速い経路が残っていないか)
 * を確かめる。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -DSIM_DEBUG -I Core/Inc -I . -I sim -o /tmp/u test/test_unknown_search.c \
 *       Core/Src/Maze.c Core/Src/ShortParam.c -lm && /tmp/u
 *   /tmp/u 1000   迷路の数(既定300)
 *   /tmp/u --mazes FILE   迷路の一覧(1行に1つ、横壁17個・縦壁17個の16進。tools/mazes_to_lines.py で作る)で調べる。
 *     ランダム迷路では起きにくいが大会の迷路で起きたこと(見に行く途中でほかの壁が見つかって目標に行けなくなる、
 *     経路の壁が「壁あり」とわかる)は、micromouseonline/mazefiles の classic の迷路で確かめた(docs/dijkstra_cost.md) */
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

/* 組 set の値で、今の G_Maze_Row/Column の最短のコスト */
static int best_cost(int set) {
	Maze_Dijkstra_Set_Run(&G_Short_Run_Set[set]);
	G_Gool_X = SIM_GOAL_X;
	G_Gool_Y = SIM_GOAL_Y;
	Maze_Dijkstra_Calculation();
	return G_Dijk_Best_Cost;
}

/* 迷路の一覧の次の1行を読む。なければ 0 */
static int read_maze_line(FILE *f) {
	char line[512];
	while (fgets(line, sizeof(line), f)) {
		char *p = line, *end;
		int k;
		for (k = 0; k < 2 * (S + 1); k++) {
			unsigned long v = strtoul(p, &end, 16);
			if (end == p)
				break;
			if (k <= S)
				TRUE_Maze_Row[k] = (uint32_t) v;
			else
				TRUE_Maze_Column[k - S - 1] = (uint32_t) v;
			p = end;
		}
		if (k == 2 * (S + 1))
			return 1;
	}
	return 0;
}

int main(int argc, char **argv) {
	FILE *list = NULL;
	if (argc > 2 && strcmp(argv[1], "--mazes") == 0) {
		list = fopen(argv[2], "r");
		if (list == NULL) {
			printf("cannot open %s\n", argv[2]);
			return 2;
		}
	}
	int n = (list == NULL && argc > 1) ? atoi(argv[1]) : 300;
	int loops_list[] = { 0, S, S * 3, S * 6 }, straight_list[] = { 0, 50, 80 };
	int fails = 0, cases = 0;
	long steps = 0;
	g_trace_file = fopen("/dev/null", "w");
	if (g_trace_file == NULL)
		return 2;
	for (int k = 0; list != NULL || k < n; k++) {
		if (list != NULL) {
			if (!read_maze_line(list))
				break;
		} else {
			make_maze(7000u + (unsigned) k, loops_list[k % 4], straight_list[(k / 4) % 3]);
		}
		long s0 = g_step_counter;
		run_scenario_b();	//探索(往路 → 未知壁 → スタートへ)。終わらなければ sim_main.c が exit(1)
		steps += g_step_counter - s0;
		cases++;
		/* 探索で見た壁だけ(見ていない壁は壁)と、本当の迷路 */
		static uint32_t row[S + 1], col[S + 1];
		Maze_Wall_fill();
		memcpy(row, G_Maze_Row, sizeof(row));
		memcpy(col, G_Maze_Column, sizeof(col));
		for (int set = 0; set < SHORT_RUN_SET_NUM; set++) {
			memcpy(G_Maze_Row, row, sizeof(row));
			memcpy(G_Maze_Column, col, sizeof(col));
			int known = best_cost(set);
			memcpy(G_Maze_Row, TRUE_Maze_Row, sizeof(row));
			memcpy(G_Maze_Column, TRUE_Maze_Column, sizeof(col));
			int truth = best_cost(set);
			if (known != truth) {
				fails++;
				if (fails <= 10)
					printf("  FAIL %s %d set %d: best after search %d, true best %d (0.5ms)\n",
							list ? "maze" : "seed", list ? k + 1 : 7000 + k, set, known, truth);
			}
		}
	}
	printf("unknown-wall search: %d mazes (%.1f steps/maze), %d failed\n", cases,
			(double) steps / cases, fails);
	if (fails == 0)
		printf("ALL PASS\n");
	return fails ? 1 : 0;
}
