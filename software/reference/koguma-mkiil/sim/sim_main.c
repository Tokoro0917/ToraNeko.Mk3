/*
 * sim_main.c
 *
 * PCネイティブ(gccのみ、ARM/HALクロスコンパイル不要)で動くシミュレータハーネス。
 * Core/Src/Maze.c は一切改変せずそのままリンクし、実際の探索・BFS・Dijkstra
 * ロジックを動かして、ブラウザ可視化用のステップ列をJSON(NDJSON、
 * 1行1ステップ)として標準出力に書き出す。
 *
 * モーター・センサーのハードウェア呼び出し(Move.c/motor.c/Wallsensor.c)は
 * 一切使わない。かわりに sim_maze_data.h の「正解」壁ビットマップを参照する
 * 仮想センサー(virtual_wall_search)で G_Wall_data[] を埋め、Move.c の
 * Robot_Maze_Sula_Action()/Robot_Maze_Suction_Action() にある純粋な
 * (モーター呼び出しを含まない)意思決定ロジックだけを sim_step() として移植する。
 *
 * 2つのシナリオを続けて実行する:
 *   A: エセ全面探索 (main.c のメニュー1相当) -- 往路探索 + 全面探索モードでの復路
 *   B: Dijkstra誘導 未知壁探索 (main.c のメニュー7相当) -- 往路探索の後、
 *      スタート→ゴール固定方向のDijkstra経路上の未確認の壁を、現在位置から
 *      直接見に行くことを繰り返し、全て確認できたらスタートに戻って最終走行
 * それぞれ Maze_Initialization() からやり直すので、互いに影響しない独立した実行。
 */

#include <stdio.h>
#include <stdlib.h>

#include "Maze.h"
#include "Define.h"
#include "sim_maze_data.h"

#if MAZE_SIZE != SIM_MAZE_SIZE
#error "sim_maze_data.h was generated for a different MAZE_SIZE"
#endif

/* Wallsensor.h の extern int G_Wall_data[]; を満たすための定義。
 * 実物の Wallsensor.c (HAL依存) はリンクしない */
int G_Wall_data[4] = { 0, 0, 0, 0 };

/* Maze.h に公開されていないが、可視化のために参照したいMaze.c内部グローバル。
 * Maze.c/Maze.h 自体は一切編集していない (Cはexternをどの.cからでも書ける) */
extern int ALL_MODE;
extern unsigned char G_Unknown_Wall_IsRow;
extern short G_Unknown_Wall_I;
extern short G_Unknown_Wall_J;

static long g_step_counter = 0;
/* Maze.c は Maze_Dijkstra_Calculation()/Maze_Shortest_Calculation() 内で
 * デバッグ用のprintf(標準出力)を無条件に呼ぶため、こちらのトレースは
 * 標準出力とは別のファイルに書き出して混ざらないようにする */
static FILE *g_trace_file;

/* ---- 仮想壁センサー: sim_maze_data.h の正解マップを参照する ---- */

/* direction: 0=北,1=東,2=南,3=西。セル(x,y)のその方向に壁があれば1 */
static int wall_present(int direction, int x, int y) {
	switch (direction) {
	case 0:
		return (TRUE_Maze_Row[y + 1] >> x) & 1u;
	case 1:
		return (TRUE_Maze_Column[x + 1] >> y) & 1u;
	case 2:
		return (TRUE_Maze_Row[y] >> x) & 1u;
	default:
		return (TRUE_Maze_Column[x] >> y) & 1u;
	}
}

static void advance_cell(int direction, int *x, int *y) {
	switch (direction % 4) {
	case 0:
		*y += 1;
		break;
	case 1:
		*x += 1;
		break;
	case 2:
		*y -= 1;
		break;
	default:
		*x -= 1;
		break;
	}
}

/* Maze_Wall_Update()はこの直後に「同じ advance」をもう一度自分で行い、
 * その新しいセルを基準にG_Wall_data[]を消費する。実機でもWall_search()は
 * Maze_Wall_Update()より先に呼ばれ、ロボットが物理的に移動した後の
 * センサー値を渡す関係になっている(motor.c: Motor_Sula_before内) */
static void virtual_wall_search(void) {
	int dir = G_Robot_Direction % 4;
	int nx = G_Robot_MAZE_X;
	int ny = G_Robot_MAZE_Y;
	advance_cell(dir, &nx, &ny);

	int left = (dir + 3) % 4;
	int right = (dir + 1) % 4;
	int front_wall = wall_present(dir, nx, ny);
	int left_wall = wall_present(left, nx, ny);
	int right_wall = wall_present(right, nx, ny);

	G_Wall_data[0] = front_wall;
	G_Wall_data[3] = front_wall;
	G_Wall_data[1] = left_wall;
	G_Wall_data[2] = right_wall;
}

/* Core/Src/Move.c の Robot_Maze_Sula_Action()/Robot_Maze_Suction_Action() に
 * ある、モーター呼び出しを除いた純粋な意思決定ロジックの移植(1セル分)。
 * Robot_Maze_Pass_Action()の「既知区間バースト」最適化は再現しない
 * (壁が既知でもMaze_Wall_Update()を毎セル通すだけなので結果は等価、
 * かつ1コマ単位のアニメーションにはこちらの方が都合がよい) */
static void sim_step(void) {
	virtual_wall_search();
	Maze_Wall_Update();

	if ((G_Maze_Flont <= G_Maze_Left) && (G_Maze_Flont <= G_Maze_Right)
			&& (G_Maze_Flont <= G_Maze_Back)) {
		G_Robot_Direction += 0;
	} else if ((G_Maze_Left <= G_Maze_Right) && (G_Maze_Left <= G_Maze_Back)) {
		G_Robot_Direction += 3;
	} else if (G_Maze_Right <= G_Maze_Back) {
		G_Robot_Direction += 1;
	} else {
		G_Robot_Direction += 2;
		G_Just_UTurned = 1;
	}
}

/* ---- トレース出力 (NDJSON: 1行1ステップ) ---- */

static void print_u32_array(const uint32_t *arr, int n) {
	fprintf(g_trace_file,"[");
	for (int i = 0; i < n; i++) {
		fprintf(g_trace_file,"%u%s", arr[i], (i + 1 < n) ? "," : "");
	}
	fprintf(g_trace_file,"]");
}

static void emit_trace(const char *scenario, const char *phase) {
	uint16_t explored_row[MAZE_SIZE];
	for (int y = 0; y < MAZE_SIZE; y++) {
		uint16_t bits = 0;
		for (int x = 0; x < MAZE_SIZE; x++) {
			if (G_MAZE_Explored[x][y]) {
				bits |= (uint16_t) (1u << x);
			}
		}
		explored_row[y] = bits;
	}

	fprintf(g_trace_file,"{\"step\":%ld,\"scenario\":\"%s\",\"phase\":\"%s\",", g_step_counter++,
			scenario, phase);
	fprintf(g_trace_file,"\"x\":%d,\"y\":%d,\"dir\":%d,", G_Robot_MAZE_X, G_Robot_MAZE_Y,
			((G_Robot_Direction % 4) + 4) % 4);
	fprintf(g_trace_file,"\"mode\":%d,", ALL_MODE);
	fprintf(g_trace_file,"\"row\":");
	print_u32_array(G_Maze_Row, MAZE_SIZE + 1);
	fprintf(g_trace_file,",\"col\":");
	print_u32_array(G_Maze_Column, MAZE_SIZE + 1);
	fprintf(g_trace_file,",\"explored\":[");
	for (int y = 0; y < MAZE_SIZE; y++) {
		fprintf(g_trace_file,"%u%s", explored_row[y], (y + 1 < MAZE_SIZE) ? "," : "");
	}
	fprintf(g_trace_file,"]");
	if (ALL_MODE == 2) {
		fprintf(g_trace_file,",\"targetX\":%d,\"targetY\":%d", G_Unknown_Target_X,
				G_Unknown_Target_Y);
		fprintf(g_trace_file,",\"wallIsRow\":%d,\"wallI\":%d,\"wallJ\":%d",
				G_Unknown_Wall_IsRow ? 1 : 0, G_Unknown_Wall_I,
				G_Unknown_Wall_J);
	} else {
		fprintf(g_trace_file,",\"targetX\":null,\"targetY\":null");
		fprintf(g_trace_file,",\"wallIsRow\":null,\"wallI\":null,\"wallJ\":null");
	}
	fprintf(g_trace_file,"}\n");
}

/* 最終ルート確定時に経路全体を一度だけ出力するマーカー行 */
static void emit_final_path(const char *scenario) {
	fprintf(g_trace_file,"{\"step\":%ld,\"scenario\":\"%s\",\"phase\":\"final_path\",",
			g_step_counter++, scenario);
	fprintf(g_trace_file,"\"pathX\":[");
	for (int k = 0; k < G_Dijk_Path_Len; k++) {
		fprintf(g_trace_file,"%d%s", G_Dijk_Path_X[k], (k + 1 < G_Dijk_Path_Len) ? "," : "");
	}
	fprintf(g_trace_file,"],\"pathY\":[");
	for (int k = 0; k < G_Dijk_Path_Len; k++) {
		fprintf(g_trace_file,"%d%s", G_Dijk_Path_Y[k], (k + 1 < G_Dijk_Path_Len) ? "," : "");
	}
	fprintf(g_trace_file,"]}\n");
}

/* main.c と同じ「未到達なら中断」ガードだが、シミュレータ側の異常系
 * フェイルセーフとして反復回数の上限も設ける (実機のFailsafe_Flag()相当) */
#define SIM_STEP_GUARD (MAX_STEP * 8)

static void run_to_cell(const char *scenario, const char *phase, int dest_x,
		int dest_y) {
	long guard = 0;
	while (G_MAZE_Explored[dest_x][dest_y] == 0) {
		if (guard++ > SIM_STEP_GUARD) {
			fprintf(stderr, "[sim] %s/%s: exceeded step guard heading to (%d,%d)\n",
					scenario, phase, dest_x, dest_y);
			exit(1);
		}
		sim_step();
		G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
		emit_trace(scenario, phase);
	}
}

/* main.cのDijkstra誘導 未知壁探索ループでは、狙っている未知壁が実際にまだ
 * 未確認である間だけ移動を続ける (G_MAZE_Explored[target]==0 ではない —
 * 既に別の面から立ったことのあるセルでも、その壁面自体はまだ未確認の
 * ことがあるため)。戻り値: Uターンした、または目標に辿り着けなかった
 * (main.cと同じくステップ上限で打ち切り)場合は1、壁を確認できた場合は0
 * — main.cの「need_recompute」と同じ意味 */
static int run_until_wall_known(const char *scenario, const char *phase) {
	G_Just_UTurned = 0;
	int sub_steps = 0;
	while ((Maze_Unknown_Wall_Still_Unknown() == 1) && (sub_steps < MAX_STEP)) {
		sim_step();
		G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
		emit_trace(scenario, phase);
		sub_steps++;
		if (G_Just_UTurned == 1) {
			break;
		}
	}
	return (G_Just_UTurned == 1) || (sub_steps >= MAX_STEP);
}

/* ---- シナリオA: エセ全面探索 (main.c: Encorder_number_out()==1 相当) ---- */
static void run_scenario_a(void) {
	Maze_Initialization();
	ALL_MODE = 0;

	G_Gool_X = SIM_GOAL_X;
	G_Gool_Y = SIM_GOAL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;

	run_to_cell("A", "outbound", SIM_GOAL_X, SIM_GOAL_Y);

	Maze_Unkown_ALL_ModeSet();
	G_Gool_X = 0;
	G_Gool_Y = 0;
	G_MAZE_Explored[0][0] = 0;
	Maze_Step_Calculate();
	run_to_cell("A", "full_coverage", 0, 0);

	Maze_Unkown_ALL_ModeOFF();
	G_Gool_X = SIM_GOAL_X;
	G_Gool_Y = SIM_GOAL_Y;
	Maze_Wall_fill();
	Maze_Dijkstra_Calculation();
	emit_final_path("A");
}

/* ---- シナリオB: Dijkstra誘導 未知壁探索 (main.c: ==7 相当) ----
 * スタート→ゴール固定方向でDijkstra経路を計算し、経路上の未確認の壁を
 * 現在位置から直接見に行く。Dijkstraの再計算はゴール到達直後とUターン
 * 発生時だけ行い、それ以外は同じ経路データのまま次の未知壁を探す。
 * ゴールやスタートまで戻る「シャトル」構造は使わない(往路の後、
 * 全ての壁が確認できたら一度だけスタートに戻って最終走行する) */
static void run_scenario_b(void) {
	Maze_Initialization();
	ALL_MODE = 0;

	G_Gool_X = SIM_GOAL_X;
	G_Gool_Y = SIM_GOAL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;

	run_to_cell("B", "outbound", SIM_GOAL_X, SIM_GOAL_Y);

	int need_recompute = 1;
	long guard = 0;
	for (;;) {
		if (guard++ > MAX_STEP) {
			fprintf(stderr, "[sim] B/unknown-wall loop: exceeded guard\n");
			exit(1);
		}

		if (need_recompute == 1) {
			G_Gool_X = SIM_GOAL_X;
			G_Gool_Y = SIM_GOAL_Y;
			Maze_Dijkstra_Calculation();
			need_recompute = 0;
		}

		int found = Maze_Unknown_Wall_Scan();
		if (found == 0) {
			break;
		}

		Maze_Unknown_Target_ModeSet(G_Unknown_Target_X, G_Unknown_Target_Y);
		Maze_Step_Calculate();
		need_recompute = run_until_wall_known("B", "check_unknown_wall");
		Maze_Unkown_ALL_ModeOFF();
	}

	G_Gool_X = 0;
	G_Gool_Y = 0;
	G_MAZE_Explored[0][0] = 0;
	Maze_Step_Calculate();
	run_to_cell("B", "return_to_start", 0, 0);

	G_Gool_X = SIM_GOAL_X;
	G_Gool_Y = SIM_GOAL_Y;
	Maze_Wall_fill();
	Maze_Dijkstra_Calculation();
	emit_final_path("B");
}

int main(int argc, char **argv) {
	const char *out_path = (argc > 1) ? argv[1] : "sim/trace.ndjson";
	g_trace_file = fopen(out_path, "w");
	if (g_trace_file == NULL) {
		fprintf(stderr, "[sim] failed to open %s for writing\n", out_path);
		return 1;
	}

	run_scenario_a();
	run_scenario_b();

	fclose(g_trace_file);
	fprintf(stderr, "[sim] wrote trace to %s (%ld records)\n", out_path,
			g_step_counter);
	return 0;
}
