/* 最短走行の経路(G_Short_Pass_NANAME)のPC上での検証。
 * Core/Src/Maze.c をそのままリンクし、Short_NANAME_Move2000 /
 * Short_Dijkstra_Move2000 と同じ順番で経路を作り、できた命令列を
 * Move.c と同じ解釈で迷路の上に「走らせて」、
 *   - 壁を突き抜けないか / 柱を通らないか / 迷路の外に出ないか
 *   - Move.c が何もしない命令(-2,-3など)が残っていないか
 *   - 最後に直進の向きで終わり、ゴール2x2区画に止まるか
 *   - BFS版は最短歩数どおりのマス数を進んでいるか
 * をランダム迷路で確認する。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -ICore/Inc -o /tmp/t test/test_shortpath.c Core/Src/Maze.c && /tmp/t
 *   gcc -std=c11 -Wall -DMAZE_SIZE=32 -ICore/Inc -o /tmp/t32 test/test_shortpath.c Core/Src/Maze.c && /tmp/t32
 *
 * 実機の迷路の確認:
 *   /tmp/t --dump log.txt
 *     実機のモード3 No.0(Maze_Debug_Dump)の出力を保存したファイルを読み込み、
 *     PCで同じ命令列が作られるか、その命令列が壁に突っ込まないかを調べる。
 *     未確認の壁は最短走行と同じく壁として扱う。
 *   /tmp/t --make-dump 番号 [未確認にする壁の割合%(既定3)]
 *     ランダム迷路で Maze_Debug_Dump と同じ出力を作る(--dump の動作確認用)
 *
 * 座標は半区画単位。マス(x,y)の中心が(2x+1,2y+1)、壁の中点が片方だけ偶数の点、
 * 柱が両方偶数の点。向きは45度単位で 0=北,2=東,4=南,6=西(奇数が斜め)。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Maze.h"
#include "Define.h"

/* Maze.c が参照する外部シンボルのスタブ */
int G_Wall_data[4];
extern uint32_t Maze_Row_Look[];	//Maze.c 内部(見た壁)
extern uint32_t Maze_Column_Look[];
extern uint32_t Maze_Row_Look_Save[];
extern uint32_t Maze_Column_Look_Save[];

#define N MAZE_SIZE

static const int HX[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
static const int HY[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };

static int fail_count = 0;
static char g_msg[256];

/* 真の迷路(検証側)。G_Maze_Row/Column は Maze.c 側の関数が書き換えるので別に持つ */
static uint32_t true_row[N + 1];
static uint32_t true_col[N + 1];

static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };

static void carve(int x, int y, int d) {
	if (d == 0)
		true_row[y + 1] &= ~(1u << x);
	else if (d == 1)
		true_col[x + 1] &= ~(1u << y);
	else if (d == 2)
		true_row[y] &= ~(1u << x);
	else
		true_col[x] &= ~(1u << y);
}

static int open_dir(int x, int y, int d) {
	int nx = x + DX[d], ny = y + DY[d];
	if (nx < 0 || nx >= N || ny < 0 || ny >= N)
		return 0;
	if (d == 0)
		return !(true_row[y + 1] & (1u << x));
	if (d == 1)
		return !(true_col[x + 1] & (1u << y));
	if (d == 2)
		return !(true_row[y] & (1u << x));
	return !(true_col[x] & (1u << y));
}

static int in_goal(int x, int y) {
	return x >= MAZE_GOOL_X && x <= MAZE_GOOL_X + 1 && y >= MAZE_GOOL_Y
			&& y <= MAZE_GOOL_Y + 1;
}

/* 標準ルールに合わせ、ゴール2x2区画の内側の壁は全部抜き、スタートは東が壁 */
static void gen_maze(unsigned seed, int loops) {
	static int visited[N][N];
	static int stx[N * N], sty[N * N];
	int sp = 0;
	srand(seed);
	for (int i = 0; i < N + 1; i++) {
		true_row[i] = 0xFFFFFFFFu;
		true_col[i] = 0xFFFFFFFFu;
	}
	memset(visited, 0, sizeof(visited));
	visited[0][0] = 1;
	stx[sp] = 0;
	sty[sp] = 0;
	sp++;
	while (sp > 0) {
		int cx = stx[sp - 1], cy = sty[sp - 1];
		int cand[4], nc = 0;
		for (int d = 0; d < 4; d++) {
			int nx = cx + DX[d], ny = cy + DY[d];
			if (nx < 0 || nx >= N || ny < 0 || ny >= N || visited[nx][ny])
				continue;
			if (cx == 0 && cy == 0 && d == 1)
				continue;
			cand[nc++] = d;
		}
		if (nc == 0) {
			sp--;
			continue;
		}
		int d = cand[rand() % nc];
		carve(cx, cy, d);
		visited[cx + DX[d]][cy + DY[d]] = 1;
		stx[sp] = cx + DX[d];
		sty[sp] = cy + DY[d];
		sp++;
	}
	for (int k = 0; k < loops; k++) {
		int x = rand() % N, y = rand() % N, d = rand() % 2;
		if (x == 0 && y == 0)
			continue;
		if ((d == 0 && y + 1 < N) || (d == 1 && x + 1 < N))
			carve(x, y, d);
	}
	carve(MAZE_GOOL_X, MAZE_GOOL_Y, 0);
	carve(MAZE_GOOL_X, MAZE_GOOL_Y, 1);
	carve(MAZE_GOOL_X + 1, MAZE_GOOL_Y + 1, 2);
	carve(MAZE_GOOL_X + 1, MAZE_GOOL_Y + 1, 3);
}

/* スタートから2x2ゴールのどれかに入るまでの最短マス数(真の迷路) */
static int true_shortest(void) {
	static int dist[N][N];
	static int qx[N * N], qy[N * N];
	int head = 0, tail = 0;
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++)
			dist[x][y] = -1;
	dist[0][0] = 0;
	qx[tail] = 0;
	qy[tail++] = 0;
	while (head < tail) {
		int cx = qx[head], cy = qy[head++];
		if (in_goal(cx, cy))
			return dist[cx][cy];
		for (int d = 0; d < 4; d++) {
			if (!open_dir(cx, cy, d))
				continue;
			int nx = cx + DX[d], ny = cy + DY[d];
			if (dist[nx][ny] >= 0)
				continue;
			dist[nx][ny] = dist[cx][cy] + 1;
			qx[tail] = nx;
			qy[tail++] = ny;
		}
	}
	return -1;
}

/* ---- 命令列を迷路の上で走らせる --------------------------------------- */

static int px, py, ph;	/* 位置(半区画単位)と向き(45度単位) */

/* 点(x,y)に居てよいか。壁の中点なら壁が無いこと、柱は不可 */
static int point_ok(int x, int y) {
	if (x < 0 || y < 0 || x > 2 * N || y > 2 * N)
		return 0;
	int ox = x & 1, oy = y & 1;
	if (ox && oy)
		return 1;	/* マス中心 */
	if (!ox && !oy)
		return 0;	/* 柱 */
	if (!oy) /* 横壁の上 (x奇数, y偶数): G_Maze_Row[y/2] の bit x/2 */
		return !(true_row[y / 2] & (1u << (x / 2)));
	return !(true_col[x / 2] & (1u << (y / 2)));
}

/* 現在位置から局所座標(前f, 左s)の点へ移動し、その点を検査する */
static int go_local(int f, int s) {
	int fx = HX[ph], fy = HY[ph];
	int lx = HX[(ph + 6) % 8], ly = HY[(ph + 6) % 8];
	int nx = px + f * fx + s * lx;
	int ny = py + f * fy + s * ly;
	if (!point_ok(nx, ny)) {
		snprintf(g_msg, sizeof(g_msg), "invalid point (%d,%d) heading %d", nx,
				ny, ph);
		return 0;
	}
	px = nx;
	py = ny;
	return 1;
}

/* 局所座標の点列を順にたどる(終点の位置はそのまま残る)。曲がる前の向きで解釈 */
static int go_points(const int *fs, int n, int base_x, int base_y, int base_h) {
	for (int k = 0; k < n; k++) {
		px = base_x;
		py = base_y;
		ph = base_h;
		if (!go_local(fs[2 * k], fs[2 * k + 1]))
			return 0;
	}
	return 1;
}

/* 1つの命令を実行。turn: +1=左(反時計回り), -1=右 */
static int exec_token(int t) {
	int bx = px, by = py, bh = ph;
	int diag = ph & 1;
	if (t == -1)
		return 1;
	if (t > 0) {
		if (diag)
			return snprintf(g_msg, sizeof(g_msg), "straight %d while diagonal",
					t), 0;
		for (int k = 0; k < t; k++)
			if (!go_local(1, 0))
				return 0;
		return 1;
	}
	if (t <= -50 && t % 50 == 0) {
		if (!diag)
			return snprintf(g_msg, sizeof(g_msg), "diag %d while orthogonal",
					t), 0;
		for (int k = 0; k < -t / 50; k++) {
			px += HX[ph];
			py += HY[ph];
			if (!point_ok(px, py))
				return snprintf(g_msg, sizeof(g_msg), "diag hits (%d,%d)", px,
						py), 0;
		}
		return 1;
	}
	int sgn;
	switch (t) {
	case -4: /* 大回り90 */
	case -6: {
		if (diag)
			break;
		sgn = (t == -4) ? 1 : -1;
		int p[] = { 1, 0, 2, 0, 2, sgn, 2, 2 * sgn };
		if (!go_points(p, 4, bx, by, bh))
			return 0;
		ph = (bh + 8 - 2 * sgn) % 8;
		return 1;
	}
	case -5: /* 大回り180 */
	case -7: {
		if (diag)
			break;
		sgn = (t == -5) ? 1 : -1;
		int p[] = { 1, 0, 2, 0, 2, sgn, 2, 2 * sgn, 1, 2 * sgn, 0, 2 * sgn };
		if (!go_points(p, 6, bx, by, bh))
			return 0;
		ph = (bh + 4) % 8;
		return 1;
	}
	case -51: /* 斜め入り45 */
	case -53: {
		if (diag)
			break;
		sgn = (t == -51) ? 1 : -1;
		int p[] = { 1, 0, 2, sgn };
		if (!go_points(p, 2, bx, by, bh))
			return 0;
		ph = (bh + 8 - sgn) % 8;
		return 1;
	}
	case -52: /* 斜め入り135 */
	case -54: {
		if (diag)
			break;
		sgn = (t == -52) ? 1 : -1;
		int p[] = { 1, 0, 2, sgn, 1, 2 * sgn };
		if (!go_points(p, 3, bx, by, bh))
			return 0;
		ph = (bh + 8 - 3 * sgn) % 8;
		return 1;
	}
	case -61: /* 斜め出45: 斜めに1つ進んでから新しい向きに半区画 */
	case -63: {
		if (!diag)
			break;
		sgn = (t == -61) ? 1 : -1;
		px += HX[bh];
		py += HY[bh];
		if (!point_ok(px, py))
			return snprintf(g_msg, sizeof(g_msg), "out45 hits (%d,%d)", px,
					py), 0;
		ph = (bh + 8 - sgn) % 8;
		return go_local(1, 0);
	}
	case -62: /* 斜め出135 */
	case -64: {
		if (!diag)
			break;
		sgn = (t == -62) ? 1 : -1;
		px += HX[bh];
		py += HY[bh];
		if (!point_ok(px, py))
			return snprintf(g_msg, sizeof(g_msg), "out135 hits (%d,%d)", px,
					py), 0;
		ph = (bh + 8 - sgn) % 8;	/* ここで一旦直交の向き */
		if (!go_local(1, 0) || !go_local(0, sgn))
			return 0;
		ph = (ph + 8 - 2 * sgn) % 8;
		return go_local(1, 0);
	}
	case -65: /* V90 */
	case -66: {
		if (!diag)
			break;
		sgn = (t == -65) ? 1 : -1;
		px += HX[bh];
		py += HY[bh];
		if (!point_ok(px, py))
			return snprintf(g_msg, sizeof(g_msg), "V90 hits (%d,%d)", px, py),
					0;
		ph = (bh + 8 - 2 * sgn) % 8;
		px += HX[ph];
		py += HY[ph];
		if (!point_ok(px, py))
			return snprintf(g_msg, sizeof(g_msg), "V90 hits (%d,%d)", px, py),
					0;
		return 1;
	}
	default:
		break;
	}
	snprintf(g_msg, sizeof(g_msg), "token %d not executable (heading %d)", t,
			bh);
	return 0;
}

static void dump_tokens(void) {
	printf("    NANAME:");
	for (int i = 0; i < MAX_STEP && G_Short_Pass_NANAME[i] != 0; i++)
		printf(" %d", G_Short_Pass_NANAME[i]);
	printf("\n    PASS:  ");
	for (int i = 0; i < MAX_STEP && G_Short_Pass[i] != 0; i++)
		printf(" %d", G_Short_Pass[i]);
	printf("\n");
}

/* Move.c と同じ解釈で走らせる。成功なら1 */
static int run_tokens(void) {
	px = 1;
	py = 1;
	ph = 0;
	int i0 = 0;
	if (G_Short_Pass_NANAME[0] == 1) {	/* スタート: 区画中心から境界まで */
		if (!go_local(1, 0))
			return 0;
		i0 = 1;
	}
	for (int i = i0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (i >= MAX_STEP)
			return snprintf(g_msg, sizeof(g_msg), "no terminator"), 0;
		if (!exec_token(G_Short_Pass_NANAME[i])) {
			char tmp[320];
			snprintf(tmp, sizeof(tmp), "token[%d]=%d: %s", i,
					G_Short_Pass_NANAME[i], g_msg);
			strcpy(g_msg, tmp);
			return 0;
		}
	}
	/* 最後は Motor_trapezoid_PID(.., 180) で1区画直進して止まる */
	if (ph & 1)
		return snprintf(g_msg, sizeof(g_msg), "ends diagonal"), 0;
	for (int k = 0; k < 2; k++) {
		if (!go_local(1, 0))
			return 0;
		if ((px & 1) && (py & 1)) {
			if (!in_goal(px / 2, py / 2))
				return snprintf(g_msg, sizeof(g_msg),
						"stops at cell (%d,%d), not goal", px / 2, py / 2), 0;
			return 1;
		}
	}
	return snprintf(g_msg, sizeof(g_msg), "no cell center at end"), 0;
}

/* 圧縮前の G_Short_Pass(直進と小回り-2/-3だけ)をそのまま走らせる。
 * 経路探索そのものの誤りと、圧縮の誤りを切り分けるため */
static int run_raw(void) {
	px = 1;
	py = 1;
	ph = 0;
	if (!go_local(1, 0))
		return 0;
	for (int i = 1; G_Short_Pass[i] != 0; i++) {
		int t = G_Short_Pass[i];
		if (t > 0) {
			for (int k = 0; k < t; k++)
				if (!go_local(1, 0))
					return 0;
		} else if (t == -2 || t == -3) {
			int sgn = (t == -2) ? 1 : -1;
			if (!go_local(1, 0) || !go_local(0, sgn))
				return 0;
			ph = (ph + 8 - 2 * sgn) % 8;
		} else {
			return snprintf(g_msg, sizeof(g_msg), "raw token %d", t), 0;
		}
	}
	return 1;
}

/* 既知の壁を真の迷路で埋めた状態で、Move.c と同じ手順で経路を作る */
static void plan(int dijkstra) {
	Maze_Initialization();
	for (int i = 0; i < N + 1; i++) {
		G_Maze_Row[i] = true_row[i];
		G_Maze_Column[i] = true_col[i];
		Maze_Row_Look[i] = 0xFFFFFFFFu;
		Maze_Column_Look[i] = 0xFFFFFFFFu;
	}
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	if (dijkstra)
		Maze_Dijkstra_Calculation();
	else
		Maze_Shortest_Calculation();
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
}

/* 最短走行の走り出し(Move.c の Short_Start)が読む、最初の命令の形を確かめる。
 * 先頭の -1(圧縮で0になった直線)は飛ばし、最初の命令は
 * 直線(>0)、大回り(-4..-7、CP側も同じ番号)、斜め入り(-51..-54)のどれかであること */
static int start_stat[8];	/* 0:直線 1:大回り90 2:大回り180 3:斜め入り45 4:斜め入り135 5:先頭に-1あり */
static int check_start_token(void) {
	int f = 0;
	while (G_Short_Pass_NANAME[f] == -1)
		f++;
	if (f > 0)
		start_stat[5]++;
	int t = G_Short_Pass_NANAME[f];
	int cp = G_Short_Pass_CP[f];
	if (t > 0) {
		start_stat[0]++;
		return 1;
	}
	if (t <= -4 && t > -50 && (cp == -4 || cp == -6)) {
		start_stat[1]++;
		return 1;
	}
	if (t <= -4 && t > -50 && (cp == -5 || cp == -7)) {
		start_stat[2]++;
		return 1;
	}
	if (t == -51 || t == -53) {
		start_stat[3]++;
		return 1;
	}
	if (t == -52 || t == -54) {
		start_stat[4]++;
		return 1;
	}
	return snprintf(g_msg, sizeof(g_msg), "start token [%d]=%d cp=%d", f, t, cp), 0;
}

/* G_Short_Pass(圧縮前)が何マス進むか: 直進2で1マス、小回り1回で1マス */
static int raw_cells(void) {
	int c = 0;
	for (int i = 0; i < MAX_STEP && G_Short_Pass[i] != 0; i++) {
		if (G_Short_Pass[i] > 0)
			c += G_Short_Pass[i] / 2;
		else if (G_Short_Pass[i] == -2 || G_Short_Pass[i] == -3)
			c += 1;
	}
	return c;
}

static int run_suite(const char *name, int dijkstra, int count, int loops,
		unsigned seed0, int show) {
	int fails = 0;
	for (int k = 0; k < count; k++) {
		unsigned seed = seed0 + k;
		gen_maze(seed, loops);
		int sd = true_shortest();
		if (sd < 0)
			continue;
		plan(dijkstra);
		int ok = run_raw();
		if (!ok) {
			char tmp[320];
			snprintf(tmp, sizeof(tmp), "raw path: %s", g_msg);
			strcpy(g_msg, tmp);
		} else {
			ok = run_tokens();
		}
		if (ok)
			ok = check_start_token();
		if (ok && !dijkstra) {
			/* スタートマスから入るので、最初の1マスは数に入らない */
			int rc = raw_cells() + 1;
			if (rc != sd) {
				snprintf(g_msg, sizeof(g_msg),
						"BFS path %d cells, true shortest %d", rc, sd);
				ok = 0;
			}
		}
		if (!ok) {
			fails++;
			if (fails <= show || getenv("ALLFAIL")) {
				printf("  FAIL %s seed=%u: %s\n", name, seed, g_msg);
				dump_tokens();
			}
		}
	}
	printf("%-28s %4d/%d OK\n", name, count - fails, count);
	fail_count += fails;
	return fails;
}

/* ---- 実機の出力を読み込んで確かめる(--dump) ------------------------- */

static int parse_pass(const char *line, int16_t *out) {
	const char *p = strchr(line, ':');
	int k = 0;
	if (!p)
		return 0;
	p++;
	while (k < MAX_STEP - 1) {
		char *end;
		long v = strtol(p, &end, 10);
		if (end == p)
			break;
		out[k++] = (int16_t) v;
		p = end;
	}
	out[k] = 0;
	return 1;
}

static int same_pass(const int16_t *a, const int16_t *b) {
	for (int k = 0; k < MAX_STEP; k++) {
		if (a[k] != b[k])
			return 0;
		if (a[k] == 0)
			return 1;
	}
	return 1;
}

static int check_dump(const char *path) {
	static uint32_t row[N + 1], col[N + 1], lrow[N + 1], lcol[N + 1];
	static int16_t robot_bfs[MAX_STEP], robot_dijk[MAX_STEP];
	int have_bfs = 0, have_dijk = 0, size = -1, nrow = 0;
	char line[4096];
	FILE *f = fopen(path, "r");
	if (!f) {
		printf("cannot open %s\n", path);
		return 2;
	}
	while (fgets(line, sizeof(line), f)) {
		char *p;
		int k;
		unsigned long v;
		if ((p = strstr(line, "MAZE_DUMP_BEGIN")) != NULL) {
			size = atoi(p + 15);
			nrow = 0;
		} else if (sscanf(line, " ROW %d %lx", &k, &v) == 2 && k >= 0 && k <= N) {
			row[k] = (uint32_t) v;
			nrow++;
		} else if (sscanf(line, " COL %d %lx", &k, &v) == 2 && k >= 0 && k <= N) {
			col[k] = (uint32_t) v;
			nrow++;
		} else if (sscanf(line, " LROW %d %lx", &k, &v) == 2 && k >= 0 && k <= N) {
			lrow[k] = (uint32_t) v;
			nrow++;
		} else if (sscanf(line, " LCOL %d %lx", &k, &v) == 2 && k >= 0 && k <= N) {
			lcol[k] = (uint32_t) v;
			nrow++;
		} else if (strstr(line, "BFS_NANAME:")) {
			have_bfs = parse_pass(strstr(line, "BFS_NANAME:"), robot_bfs);
		} else if (strstr(line, "DIJK_NANAME:")) {
			have_dijk = parse_pass(strstr(line, "DIJK_NANAME:"), robot_dijk);
		}
	}
	fclose(f);
	if (size != N) {
		printf("MAZE_DUMP_BEGIN %d not found (this build is MAZE_SIZE=%d)\n",
				size, N);
		return 2;
	}
	if (nrow != 4 * (N + 1)) {
		printf("dump is incomplete: %d of %d lines\n", nrow, 4 * (N + 1));
		return 2;
	}

	/* 最短走行と同じく、未確認の壁は壁として扱う */
	uint32_t mask = (uint32_t) (((uint64_t) 1 << N) - 1);
	int unknown = 0;
	for (int k = 0; k < N + 1; k++) {
		true_row[k] = row[k] | (~lrow[k] & mask);
		true_col[k] = col[k] | (~lcol[k] & mask);
		for (int b = 0; b < N; b++)
			unknown += !((lrow[k] >> b) & 1u) + !((lcol[k] >> b) & 1u);
	}
	printf("MAZE_SIZE=%d, unknown walls=%d\n", N, unknown);

	int bad = 0;
	for (int dijkstra = 0; dijkstra <= 1; dijkstra++) {
		const char *name = dijkstra ? "Dijkstra" : "BFS";
		plan(dijkstra);
		int ok;
		if (G_Short_Pass[0] == 0) {
			ok = 0;
			snprintf(g_msg, sizeof(g_msg),
					"no path (start cannot reach goal; unknown walls count as walls)");
		} else {
			ok = run_raw();
			if (ok)
				ok = run_tokens();
		}
		printf("%-8s path: %s%s\n", name, ok ? "OK" : "NG: ", ok ? "" : g_msg);
		if (!ok) {
			dump_tokens();
			bad++;
		}
		int have = dijkstra ? have_dijk : have_bfs;
		const int16_t *robot = dijkstra ? robot_dijk : robot_bfs;
		if (!have) {
			printf("%-8s robot output: not found\n", name);
		} else if (same_pass(robot, G_Short_Pass_NANAME)) {
			printf("%-8s robot output: same as PC\n", name);
		} else {
			printf("%-8s robot output: DIFFERENT from PC\n", name);
			dump_tokens();
			bad++;
		}
	}
	return bad ? 1 : 0;
}

/* ランダム迷路の一部の壁を未確認にして Maze_Debug_Dump の出力を作る */
static void make_dump(unsigned seed, int ratio) {
	gen_maze(seed, N * 3);
	Maze_Initialization();
	for (int k = 0; k < N + 1; k++) {
		G_Maze_Row_Save[k] = true_row[k];
		G_Maze_Column_Save[k] = true_col[k];
		Maze_Row_Look_Save[k] = 0xFFFFFFFFu;
		Maze_Column_Look_Save[k] = 0xFFFFFFFFu;
	}
	for (int y = 1; y < N; y++) {
		for (int x = 0; x < N; x++) {
			if (rand() % 100 < ratio) {	/* 壁があるかどうかは消して未確認にする */
				Maze_Row_Look_Save[y] &= ~(1u << x);
				G_Maze_Row_Save[y] &= ~(1u << x);
			}
			if (rand() % 100 < ratio) {
				Maze_Column_Look_Save[y] &= ~(1u << x);
				G_Maze_Column_Save[y] &= ~(1u << x);
			}
		}
	}
	Maze_Debug_Dump();
}

int main(int argc, char **argv) {
	if (argc > 2 && strcmp(argv[1], "--dump") == 0)
		return check_dump(argv[2]);
	if (argc > 2 && strcmp(argv[1], "--make-dump") == 0) {
		make_dump((unsigned) atoi(argv[2]), (argc > 3) ? atoi(argv[3]) : 3);
		return 0;
	}
	int n = (argc > 1) ? atoi(argv[1]) : 300;	/* 迷路の数(既定300) */
	printf("MAZE_SIZE=%d\n", N);
	run_suite("BFS  perfect maze", 0, n, 0, 1000, 3);
	run_suite("BFS  maze with loops", 0, n, N * 3, 2000, 3);
	run_suite("Dijkstra perfect maze", 1, n, 0, 1000, 3);
	run_suite("Dijkstra maze with loops", 1, n, N * 3, 2000, 3);
	printf("start token: straight %d, big90 %d, big180 %d, in45 %d, in135 %d (leading -1: %d)\n",
			start_stat[0], start_stat[1], start_stat[2], start_stat[3], start_stat[4], start_stat[5]);
	if (fail_count) {
		printf("FAILED: %d\n", fail_count);
		return 1;
	}
	printf("ALL PASS\n");
	return 0;
}
