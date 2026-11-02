/* 最短走行の経路(G_Short_Pass_NANAME)のPC上での検証。
 * Core/Src/Maze.c をそのままリンクし、Short_NANAME_Move2000 /
 * Short_Dijkstra_Move2000 と同じ順番で経路を作り、できた命令列を
 * Move.c と同じ解釈で迷路の上に「走らせて」、
 *   - 壁を突き抜けないか / 柱を通らないか / 迷路の外に出ないか
 *   - Move.c が何もしない命令(-2,-3など)が残っていないか
 *   - 最後に直進の向きで終わり、ゴール2x2区画に止まるか
 *   - BFS版は最短歩数どおりのマス数を進んでいるか
 *   - ダイクストラ版は一番速いか(下の「ダイクストラの経路が本当に最短か」)
 * をランダム迷路で確認する。
 *   /tmp/t --brute 迷路の数 ループの数
 *     小さい迷路(-DMAZE_SIZE=6 などでビルド)で全部の経路の時間と比べる(2026-10-27)
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -ICore/Inc -o /tmp/t test/test_shortpath.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && /tmp/t
 *   gcc -std=c11 -Wall -DMAZE_SIZE=32 -ICore/Inc -o /tmp/t32 test/test_shortpath.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && /tmp/t32
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
	/* 最後は Motor_trapezoid_PID(.., Short_Goal_X()) で止まる(Move.c、2026-10-20)。
	 * 命令列が境目で終わる(Short_Pass_Ends_At_Boundary)なら 1.5区画、区画の中心で終わるなら 1区画進み、
	 * どちらもゴールの区画の中心で止まる */
	if (ph & 1)
		return snprintf(g_msg, sizeof(g_msg), "ends diagonal"), 0;
	int at_center = (px & 1) && (py & 1);
	if (at_center == Short_Pass_Ends_At_Boundary())
		return snprintf(g_msg, sizeof(g_msg),
				"ends at (%d,%d) but Short_Pass_Ends_At_Boundary()=%d", px, py,
				Short_Pass_Ends_At_Boundary()), 0;
	for (int k = 0; k < (at_center ? 2 : 3); k++)
		if (!go_local(1, 0))
			return 0;
	if (!((px & 1) && (py & 1)))
		return snprintf(g_msg, sizeof(g_msg), "stops at (%d,%d), not a cell center", px, py), 0;
	if (!in_goal(px / 2, py / 2))
		return snprintf(g_msg, sizeof(g_msg),
				"stops at cell (%d,%d), not goal", px / 2, py / 2), 0;
	/* ロボットが経路から出す止まる区画と向き(Maze.c の Short_Pass_End_Cell。最短のあとゴールから帰るのに使う) */
	int ex, ey, ed;
	if (!Short_Pass_End_Cell(&ex, &ey, &ed) || ex != px / 2 || ey != py / 2
			|| ed != ph / 2)
		return snprintf(g_msg, sizeof(g_msg),
				"Short_Pass_End_Cell gives (%d,%d) dir %d, run stops at (%d,%d) dir %d",
				ex, ey, ed, px / 2, py / 2, ph / 2), 0;
	return 1;
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

/* ---- ダイクストラの経路が本当に最短か ---------------------------------
 * (1) 以前のコスト(G_Dijk_Time_Cost = 0): Maze.c のコスト(ST_COST / DIAG_COST / CON_COST)で、経路のコストを数える。
 * 経路は壁の中点から壁の中点へのホップの並び。縦横のホップは半区画2つ、斜めのホップは
 * 半区画1つずつ縦横に進む。各ホップのコストは、ゴール側の次のホップと同じ向きなら CON_COST、
 * 違えば縦横 ST_COST・斜め DIAG_COST(最後のホップはゴールに入る向きと比べる)。
 * 本当の最短は、状態を「壁の中点 + そこに来た向き(8方向)」にして、変化がなくなるまで
 * 全部の状態を見直す素朴な方法で求める(Maze.c とは別に書いた参照実装)。 */

static int hop_cost(int h, int next_h) {
	if (h == next_h)
		return CON_COST;
	return (h % 2 == 0) ? ST_COST : DIAG_COST;
}

/* 圧縮前の G_Short_Pass のコスト */
static int raw_cost(void) {
	static int hops[MAX_STEP * 2];
	int n = 0, h = 0;
	for (int i = 1; i < MAX_STEP && G_Short_Pass[i] != 0; i++) {
		int t = G_Short_Pass[i];
		if (t > 0) {
			for (int k = 0; k < t / 2; k++)
				hops[n++] = h;
		} else if (t == -2) {	//左: 進む向きの左45度の斜めのホップ
			hops[n++] = (h + 7) % 8;
			h = (h + 6) % 8;
		} else if (t == -3) {	//右
			hops[n++] = (h + 1) % 8;
			h = (h + 2) % 8;
		}
	}
	int cost = 0;
	int next_h = h;	//ゴールに入る向き
	for (int i = n - 1; i >= 0; i--) {
		cost += hop_cost(hops[i], next_h);
		next_h = hops[i];
	}
	return cost;
}

/* 半区画単位の座標(cx,cy)の壁の中点が通れるか(外周と真の迷路の壁は通れない) */
static int ref_open(int cx, int cy) {
	if (cx < 0 || cx > 2 * N || cy < 0 || cy > 2 * N)
		return 0;
	if (cx % 2 == 1 && cy % 2 == 0)	//横の壁 G_Maze_Row[cy/2] の bit (cx-1)/2
		return !(true_row[cy / 2] & (1u << ((cx - 1) / 2)));
	if (cx % 2 == 0 && cy % 2 == 1)	//縦の壁 G_Maze_Column[cx/2] の bit (cy-1)/2
		return !(true_col[cx / 2] & (1u << ((cy - 1) / 2)));
	return 0;
}

#define REF_INF 1000000
static int ref_cost[2 * N + 1][2 * N + 1][8];

static int ref_shortest_cost(void) {
	for (int x = 0; x <= 2 * N; x++)
		for (int y = 0; y <= 2 * N; y++)
			for (int h = 0; h < 8; h++)
				ref_cost[x][y][h] = REF_INF;
	/* ゴール2x2区画の外側の境界を、外へ向かう向きでコスト0にする */
	for (int gx = MAZE_GOOL_X; gx <= MAZE_GOOL_X + 1; gx++) {
		for (int gy = MAZE_GOOL_Y; gy <= MAZE_GOOL_Y + 1; gy++) {
			for (int d = 0; d < 4; d++) {
				if (in_goal(gx + DX[d], gy + DY[d]))
					continue;
				int ex = 2 * gx + 1 + DX[d], ey = 2 * gy + 1 + DY[d];
				if (ref_open(ex, ey))
					ref_cost[ex][ey][2 * d] = 0;
			}
		}
	}
	int changed = 1;
	while (changed) {
		changed = 0;
		for (int x = 0; x <= 2 * N; x++) {
			for (int y = 0; y <= 2 * N; y++) {
				for (int h = 0; h < 8; h++) {
					int c = ref_cost[x][y][h];
					if (c >= REF_INF)
						continue;
					/* 通り抜けている向き(横の壁なら北か南、縦の壁なら東か西)の先の区画を通る3本 */
					int fwd;
					if (y % 2 == 0)
						fwd = (HY[h] > 0) ? 0 : 4;
					else
						fwd = (HX[h] > 0) ? 2 : 6;
					for (int k = -1; k <= 1; k++) {
						int nh = (fwd + k + 8) % 8;
						int step = (nh % 2 == 0) ? 2 : 1;
						int nx = x + HX[nh] * step, ny = y + HY[nh] * step;
						if (!ref_open(nx, ny))
							continue;
						int nc = c + hop_cost(nh, h);
						if (nc < ref_cost[nx][ny][nh]) {
							ref_cost[nx][ny][nh] = nc;
							changed = 1;
						}
					}
				}
			}
		}
	}
	/* スタート区画の北の壁(1,2)を、ゴール側から見て南向きに通る状態 */
	int best = REF_INF;
	for (int h = 3; h <= 5; h++)
		if (ref_cost[1][2][h] < best)
			best = ref_cost[1][2][h];
	return best;
}

/* (2) 最短走行の時間(G_Dijk_Time_Cost = 1、2026-10-27): 命令列 G_Short_Pass_NANAME の時間を、
 * tools/route_time.py と同じ数え方で、Maze.c が作った表(G_Dijk_Cost)から数える。
 * ダイクストラが返したコスト(G_Dijk_Best_Cost)がこれと同じか(ターンとホップの並びが圧縮と合っているか)、
 * BFS や以前のコストの経路より遅くないか、を確かめる。本当の最短かは、小さい迷路で全部の経路を
 * 数えて確かめる(--brute)。 */
static int token_time(const int16_t *pass) {
	static int16_t t[MAX_STEP];
	const DijkCost *c = &G_Dijk_Cost;
	int n = 0, total = 0, f = -1, last = 0;
	while (n < MAX_STEP && pass[n] != 0) {
		t[n] = pass[n];
		n++;
	}
	for (int k = 0; k < n; k++)
		if (t[k] != -1) {
			f = k;
			break;
		}
	if (f >= 0 && t[f] >= 1) {	//最初が直線: 走り出しで半区画
		total += c->start_s;
		t[f] = (t[f] > 1) ? t[f] - 1 : -1;
	} else {
		total += c->start_t;
	}
	for (int k = 0; k < n; k++) {
		int v = t[k];
		if (v == -1)
			continue;
		last = v;
		if (v > 0) {
			if (v >= DIJK_TH_NUM)
				return -1;
			total += c->th[v];
		} else if (v <= -50 && v % 50 == 0) {
			if (-v / 50 >= DIJK_TD_NUM)
				return -1;
			total += c->td[-v / 50];
		} else if (v == -4 || v == -6) {
			total += c->turn[T2000_BIG90];
		} else if (v == -5 || v == -7) {
			total += c->turn[T2000_BIG180];
		} else if (v == -51 || v == -53) {
			total += c->turn[T2000_IN45];
		} else if (v == -52 || v == -54) {
			total += c->turn[T2000_IN135];
		} else if (v == -61 || v == -63) {
			total += c->turn[T2000_OUT45];
		} else if (v == -62 || v == -64) {
			total += c->turn[T2000_OUT135];
		} else if (v == -65 || v == -66) {
			total += c->turn[T2000_V90];
		} else {
			return -1;
		}
	}
	return total + ((last > 0) ? c->stop_b : c->stop_c);
}

/* 今の迷路で、BFS と以前のコストのダイクストラの経路の時間のうち速いほう(ダイクストラはこれより遅くてはいけない) */
static int alt_best_time(void) {
	plan(0);
	int best = token_time(G_Short_Pass_NANAME);
	G_Dijk_Time_Cost = 0;
	plan(1);
	G_Dijk_Time_Cost = 1;
	int t = token_time(G_Short_Pass_NANAME);
	if (t >= 0 && (best < 0 || t < best))
		best = t;
	return best;
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
		if (ok && dijkstra && !G_Dijk_Time_Cost) {
			int rc = raw_cost(), best = ref_shortest_cost();
			if (rc != best || G_Dijk_Best_Cost != rc) {
				snprintf(g_msg, sizeof(g_msg),
						"Dijkstra path cost %d (returned %d), optimum %d", rc,
						G_Dijk_Best_Cost, best);
				ok = 0;
			}
		}
		if (ok && dijkstra && G_Dijk_Time_Cost) {
			int tt = token_time(G_Short_Pass_NANAME);
			if (tt != G_Dijk_Best_Cost) {
				snprintf(g_msg, sizeof(g_msg),
						"path time %d, Dijkstra cost %d", tt, G_Dijk_Best_Cost);
				ok = 0;
			} else {
				int alt = alt_best_time();
				if (alt >= 0 && alt < tt) {
					snprintf(g_msg, sizeof(g_msg),
							"Dijkstra path time %d, BFS / old cost %d", tt, alt);
					ok = 0;
				}
				plan(1);	//表示用に戻す
			}
		}
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
	Maze_Dijkstra_Set_Run(&G_Short_Run_Set[0]);
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
	/* 参考: ほかの値のダイクストラの経路が、上(G_Short_Run_Set[0]、2000)と同じか。失敗にはしない */
	static int16_t pass2000[MAX_STEP];
	memcpy(pass2000, G_Short_Pass_NANAME, sizeof(pass2000));
	printf("Dijkstra (2000) estimated time: %.3f s\n", G_Dijk_Best_Cost / (double) DIJK_COST_PER_S);
	for (int m = 0; m < 2; m++) {
		const char *name = m ? "old cost" : "2400";
		if (m) {
			G_Dijk_Time_Cost = 0;
		} else {
			Maze_Dijkstra_Set_Run(&G_Short_Run_Set[1]);
		}
		plan(1);
		printf("Dijkstra (%s): %s\n", name,
				same_pass(pass2000, G_Short_Pass_NANAME) ? "same path" : "different path");
		if (!same_pass(pass2000, G_Short_Pass_NANAME))
			dump_tokens();
	}
	G_Dijk_Time_Cost = 1;
	Maze_Dijkstra_Set_Run(&G_Short_Run_Set[0]);
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

/* ---- 小さい迷路で、全部の経路の時間と比べる(--brute、2026-10-27) ----------------
 * スタートからゴールまでの、同じ区画を2回通らない経路を全部作り、Move.c と同じ手順で
 * 命令列にして時間を数え(token_time)、一番速いものとダイクストラのコストが同じかを確かめる。
 * 経路の数が多いので、小さい迷路(-DMAZE_SIZE=6 など)で使う */
static int bf_x[N * N + 2], bf_y[N * N + 2], bf_seen[N][N], bf_best;
static long bf_paths;

static void bf_eval(int m) {	//区画 0..m(m がゴールの区画)
	for (int k = 0; k < MAX_STEP; k++)
		G_Short_Pass[k] = 0;
	int np = 0;
	G_Short_Pass[0] = -1;
	for (int i = 1; i < m; i++) {
		int din = 0, dout = 0;
		for (int d = 0; d < 4; d++) {
			if (bf_x[i] - bf_x[i - 1] == DX[d] && bf_y[i] - bf_y[i - 1] == DY[d])
				din = d;
			if (bf_x[i + 1] - bf_x[i] == DX[d] && bf_y[i + 1] - bf_y[i] == DY[d])
				dout = d;
		}
		if (din == dout) {
			if (G_Short_Pass[np] < 0)
				np++;
			G_Short_Pass[np] += 2;
		} else {
			np++;
			G_Short_Pass[np] = (dout == (din + 1) % 4) ? -3 : -2;
		}
	}
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	int t = token_time(G_Short_Pass_NANAME);
	bf_paths++;
	if (t >= 0 && (bf_best < 0 || t < bf_best))
		bf_best = t;
}

static void bf_dfs(int m) {
	int x = bf_x[m], y = bf_y[m];
	if (in_goal(x, y)) {
		bf_eval(m);
		return;
	}
	for (int d = 0; d < 4; d++) {
		int nx = x + DX[d], ny = y + DY[d];
		if (!open_dir(x, y, d) || nx < 0 || ny < 0 || nx >= N || ny >= N || bf_seen[nx][ny])
			continue;
		bf_seen[nx][ny] = 1;
		bf_x[m + 1] = nx;
		bf_y[m + 1] = ny;
		bf_dfs(m + 1);
		bf_seen[nx][ny] = 0;
	}
}

static int brute_main(int count, int loops) {
	int fails = 0, done = 0;
	long total_paths = 0;
	printf("MAZE_SIZE=%d, brute force, %d mazes, %d loops\n", N, count, loops);
	for (int set = 0; set < SHORT_RUN_SET_NUM; set++) {
		G_Dijk_Time_Cost = 1;
		Maze_Dijkstra_Set_Run(&G_Short_Run_Set[set]);
		for (int k = 0; k < count; k++) {
			gen_maze(5000 + k, loops);
			if (true_shortest() < 0)
				continue;
			plan(1);
			int dc = G_Dijk_Best_Cost, tt = token_time(G_Short_Pass_NANAME);
			memset(bf_seen, 0, sizeof(bf_seen));
			bf_x[0] = 0;
			bf_y[0] = 0;
			bf_seen[0][0] = 1;
			bf_best = -1;
			bf_paths = 0;
			bf_dfs(0);
			done++;
			total_paths += bf_paths;
			if (dc != bf_best || tt != dc) {
				fails++;
				if (fails <= 5) {
					printf("  FAIL set %d seed %d: Dijkstra %d (path %d), brute force %d (%ld paths)\n",
							set, 5000 + k, dc, tt, bf_best, bf_paths);
					plan(1);
					dump_tokens();
				}
			}
		}
	}
	printf("brute force: %d cases (%ld paths), %d failed\n", done, total_paths, fails);
	if (fails == 0)
		printf("ALL PASS\n");
	return fails ? 1 : 0;
}

int main(int argc, char **argv) {
	if (argc > 2 && strcmp(argv[1], "--dump") == 0)
		return check_dump(argv[2]);
	if (argc > 2 && strcmp(argv[1], "--make-dump") == 0) {
		make_dump((unsigned) atoi(argv[2]), (argc > 3) ? atoi(argv[3]) : 3);
		return 0;
	}
	if (argc > 1 && strcmp(argv[1], "--brute") == 0)
		return brute_main((argc > 2) ? atoi(argv[2]) : 200, (argc > 3) ? atoi(argv[3]) : N);
	int n = (argc > 1) ? atoi(argv[1]) : 300;	/* 迷路の数(既定300) */
	printf("MAZE_SIZE=%d\n", N);
	run_suite("BFS  perfect maze", 0, n, 0, 1000, 3);
	run_suite("BFS  maze with loops", 0, n, N * 3, 2000, 3);
	/* 最短走行の時間(既定。G_Short_Run_Set の 2000 と 2400)と、以前のコスト */
	G_Dijk_Time_Cost = 1;
	Maze_Dijkstra_Set_Run(&G_Short_Run_Set[0]);
	run_suite("Dijkstra perfect maze", 1, n, 0, 1000, 3);
	run_suite("Dijkstra maze with loops", 1, n, N * 3, 2000, 3);
	Maze_Dijkstra_Set_Run(&G_Short_Run_Set[1]);
	run_suite("Dijkstra 2400 perfect maze", 1, n, 0, 1000, 3);
	run_suite("Dijkstra 2400 maze w/ loops", 1, n, N * 3, 2000, 3);
	G_Dijk_Time_Cost = 0;
	run_suite("Dijkstra(old) perfect maze", 1, n, 0, 1000, 3);
	run_suite("Dijkstra(old) maze w/ loops", 1, n, N * 3, 2000, 3);
	G_Dijk_Time_Cost = 1;
	printf("start token: straight %d, big90 %d, big180 %d, in45 %d, in135 %d (leading -1: %d)\n",
			start_stat[0], start_stat[1], start_stat[2], start_stat[3], start_stat[4], start_stat[5]);
	if (fail_count) {
		printf("FAILED: %d\n", fail_count);
		return 1;
	}
	printf("ALL PASS\n");
	return 0;
}
