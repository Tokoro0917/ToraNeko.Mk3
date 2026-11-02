/* 既知区間加速の経路(G_Known_Pass と斜めにした Known_Pass_NANAME)のPC上での検証(2026-10-21)。
 * Core/Src/Maze.c をそのままリンクし、探索の途中の「見た区画」と「今の位置」をランダムに作って
 * Known_Pass_Generation() を呼び、できた命令列を迷路の上に「走らせて」、
 *   - 壁を突き抜けないか / 柱を通らないか / 迷路の外に出ないか
 *   - 斜めにした命令列(Known_Pass_NANAME)が、斜めにする前(G_Known_Pass)と同じ位置・向きで終わるか
 *   - 最後は直進の向きで、未探索の区画の手前の境目で終わるか(そこから探索を続けるため)
 *   - Known_Pass_Generation() が動かしたロボットの区画と向きが、走った終わりと合うか
 * を確かめる。走らせ方・座標は test_shortpath.c と同じ(このファイルが #include して使う)。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -ICore/Inc -o /tmp/k test/test_knownpass.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && /tmp/k
 *   /tmp/k -v   失敗した例の命令列も出す
 *
 * 既知区間の命令の意味(Move.c の Robot_Maze_Pass_Action と同じ):
 *   正の数 n: 半区画 n 個まっすぐ(1区画で 2)
 *   -2 / -3: 区画の中で左 / 右に曲がる(境目 → 横の境目。探索と同じ小回り)
 *   -8: 行き止まりのUターン(境目 → 区画の中心で180° → 同じ境目へ戻る)
 *   -1: 何もしない
 *   斜めの命令(-50の倍数、-51〜-54、-61〜-66)は最短走行と同じ */
#define main test_shortpath_main
#include "test_shortpath.c"
#undef main

static int verbose = 0;

/* 既知区間の命令を1つ走らせる。最短走行の命令(exec_token)に、-2/-3/-8 を足したもの */
static int exec_known(int t) {
	if (t == -2 || t == -3) {
		if (ph & 1)
			return snprintf(g_msg, sizeof(g_msg), "small turn %d while diagonal", t), 0;
		int sgn = (t == -2) ? 1 : -1;
		int bx = px, by = py, bh = ph;
		int p[] = { 1, 0, 1, sgn };
		if (!go_points(p, 2, bx, by, bh))
			return 0;
		ph = (bh + 8 - 2 * sgn) % 8;
		return 1;
	}
	if (t == -8) {
		if (ph & 1)
			return snprintf(g_msg, sizeof(g_msg), "U-turn while diagonal"), 0;
		if (!go_local(1, 0))
			return 0;
		ph = (ph + 4) % 8;
		return go_local(1, 0);
	}
	return exec_token(t);
}

static int run_known(const int16_t *pass, int sx, int sy, int sh, int *ex,
		int *ey, int *eh) {
	px = sx;
	py = sy;
	ph = sh;
	for (int i = 0; pass[i] != 0; i++) {
		if (i >= MAX_STEP)
			return snprintf(g_msg, sizeof(g_msg), "no terminator"), 0;
		if (!exec_known(pass[i])) {
			char tmp[320];
			snprintf(tmp, sizeof(tmp), "token[%d]=%d: %s", i, pass[i], g_msg);
			strcpy(g_msg, tmp);
			return 0;
		}
	}
	*ex = px;
	*ey = py;
	*eh = ph;
	return 1;
}

static void dump_known(void) {
	printf("    KNOWN: ");
	for (int i = 0; i < MAX_STEP && G_Known_Pass[i] != 0; i++)
		printf(" %d", G_Known_Pass[i]);
	printf("\n    NANAME:");
	for (int i = 0; i < MAX_STEP && Known_Pass_NANAME[i] != 0; i++)
		printf(" %d", Known_Pass_NANAME[i]);
	printf("\n");
}

/* 見た区画(explored)に接する壁だけ本当の壁を入れ、ほかは未確認(開いている)にする */
static void set_known_walls(void) {
	for (int k = 0; k <= N; k++) {
		G_Maze_Row[k] = 0;
		G_Maze_Column[k] = 0;
	}
	for (int x = 0; x < N; x++) {
		for (int y = 0; y < N; y++) {
			if (!G_MAZE_Explored[x][y])
				continue;
			uint32_t bx = 1u << x, by = 1u << y;
			G_Maze_Row[y + 1] |= true_row[y + 1] & bx;
			G_Maze_Row[y] |= true_row[y] & bx;
			G_Maze_Column[x + 1] |= true_col[x + 1] & by;
			G_Maze_Column[x] |= true_col[x] & by;
		}
	}
	/* 外周はいつも壁 */
	G_Maze_Row[0] = G_Maze_Row[N] = (N >= 32) ? 0xFFFFFFFFu : ((1u << N) - 1);
	G_Maze_Column[0] = G_Maze_Column[N] = (N >= 32) ? 0xFFFFFFFFu : ((1u << N) - 1);
}

/* 1つの場面を作って調べる。戻り値: 1 調べた(成功/失敗は fail_count)、0 場面が作れなかった */
static int stat_runs, stat_naname, stat_uturn, stat_raw_fail;
static int one_case(unsigned seed, int ratio) {
	srand(seed * 7919u + 13u);
	Maze_Initialization();
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++)
			G_MAZE_Explored[x][y] = (rand() % 100) < ratio;
	G_MAZE_Explored[0][0] = 1;
	set_known_walls();
	/* 探索と同じく、目標の区画は未探索にしておく(Maze.c の探索は、行き(ゴール)では
	 * ゴールの区画をまだ見ておらず、帰り(スタート)では G_MAZE_Explored[0][0] = 0 にしてから歩数を作る)。
	 * そうしないと、既知区間がゴールの中で止まらない */
	int back = seed & 1;
	if (back) {
		G_Gool_X = 0;
		G_Gool_Y = 0;
		G_MAZE_Explored[0][0] = 0;
	} else {
		G_Gool_X = MAZE_GOOL_X;
		G_Gool_Y = MAZE_GOOL_Y;
		for (int gx = 0; gx < 2; gx++)
			for (int gy = 0; gy < 2; gy++)
				G_MAZE_Explored[MAZE_GOOL_X + gx][MAZE_GOOL_Y + gy] = 0;
	}
	Maze_Step_Calculate();
	/* ロボット: 見た区画 c から、開いている向き d の隣も見た区画であるところの境目 */
	int cx, cy, d, tries = 0;
	do {
		cx = rand() % N;
		cy = rand() % N;
		d = rand() % 4;
		if (++tries > 2000)
			return 0;
	} while (!G_MAZE_Explored[cx][cy] || !open_dir(cx, cy, d)
			|| !G_MAZE_Explored[cx + DX[d]][cy + DY[d]]);
	G_Robot_MAZE_X = cx;
	G_Robot_MAZE_Y = cy;
	G_Robot_Direction = d;
	int sx = 2 * cx + 1 + DX[d], sy = 2 * cy + 1 + DY[d], sh = 2 * d;

	Known_Pass_Generation();
	stat_runs++;

	int rx, ry, rh, nx, ny, nh;
	int ok = 1;
	char why[400] = "";
	if (!run_known(G_Known_Pass, sx, sy, sh, &rx, &ry, &rh)) {
		snprintf(why, sizeof(why), "KNOWN: %s", g_msg);
		ok = 0;
		stat_raw_fail++;
	} else if (!run_known(Known_Pass_NANAME, sx, sy, sh, &nx, &ny, &nh)) {
		snprintf(why, sizeof(why), "NANAME: %s", g_msg);
		ok = 0;
	} else if (nx != rx || ny != ry || nh != rh) {
		snprintf(why, sizeof(why),
				"NANAME ends at (%d,%d) h%d, KNOWN ends at (%d,%d) h%d", nx, ny,
				nh, rx, ry, rh);
		ok = 0;
	} else if (rh & 1) {
		snprintf(why, sizeof(why), "ends diagonal");
		ok = 0;
	} else {
		/* 終わりは境目で、その先(向きの先)の区画は未探索、手前の区画が G_Robot_MAZE_X/Y */
		int ex = rx - HX[rh], ey = ry - HY[rh];	/* 手前の区画の中心 */
		int fx = rx + HX[rh], fy = ry + HY[rh];	/* 先の区画の中心 */
		if (!((rx & 1) ^ (ry & 1))) {
			snprintf(why, sizeof(why), "ends at (%d,%d), not on a boundary", rx, ry);
			ok = 0;
		} else if (ex / 2 != G_Robot_MAZE_X || ey / 2 != G_Robot_MAZE_Y
				|| rh / 2 != G_Robot_Direction % 4) {
			snprintf(why, sizeof(why),
					"robot state (%d,%d) dir %d, run ends in cell (%d,%d) dir %d",
					G_Robot_MAZE_X, G_Robot_MAZE_Y, G_Robot_Direction % 4, ex / 2,
					ey / 2, rh / 2);
			ok = 0;
		} else if (fx >= 0 && fy >= 0 && fx < 2 * N && fy < 2 * N
				&& G_MAZE_Explored[fx / 2][fy / 2]) {
			snprintf(why, sizeof(why), "next cell (%d,%d) is explored", fx / 2, fy / 2);
			ok = 0;
		}
	}
	for (int i = 0; Known_Pass_NANAME[i] != 0; i++) {
		if (Known_Pass_NANAME[i] <= -50) {
			stat_naname++;
			break;
		}
	}
	for (int i = 0; G_Known_Pass[i] != 0; i++) {
		if (G_Known_Pass[i] == -8) {
			stat_uturn++;
			break;
		}
	}
	if (!ok) {
		fail_count++;
		if (fail_count <= 10 || verbose) {
			printf("  FAIL seed %u ratio %d start cell (%d,%d) dir %d: %s\n", seed,
					ratio, cx, cy, d, why);
			dump_known();
		}
	}
	return 1;
}

int main(int argc, char **argv) {
	(void) test_shortpath_main;
	for (int a = 1; a < argc; a++)
		if (!strcmp(argv[a], "-v"))
			verbose = 1;
	int ratios[] = { 30, 60, 90, 100 };
	for (unsigned m = 0; m < 300; m++) {
		gen_maze(m, (m % 3) * 20);
		for (int r = 0; r < 4; r++)
			for (unsigned k = 0; k < 10; k++)
				one_case(m * 100 + r * 10 + k, ratios[r]);
	}
	printf("known pass: %d cases, %d with diagonal, %d with U-turn, %d failed (%d in G_Known_Pass itself)\n",
			stat_runs, stat_naname, stat_uturn, fail_count, stat_raw_fail);
	if (fail_count == 0)
		printf("ALL PASS\n");
	return fail_count ? 1 : 0;
}
