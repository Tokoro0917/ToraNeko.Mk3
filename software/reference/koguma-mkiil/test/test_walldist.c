/* 壁センサ値 -> 距離変換(WallDist_mm)のPC上での検証。
 * 表を実測値に書き換えたあとも、このテストで表の書き間違いを検出できる。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -ICore/Inc -o /tmp/tw test/test_walldist.c Core/Src/WallDistance.c && /tmp/tw
 */
#include <stdio.h>
#include "WallDistance.h"

static int fail_count = 0;
#define CHECK(cond, ...)                                                      \
	do {                                                                      \
		if (!(cond)) {                                                        \
			fail_count++;                                                     \
			printf("  FAIL: ");                                               \
			printf(__VA_ARGS__);                                              \
			printf("\n");                                                     \
		}                                                                     \
	} while (0)

static const char *name[4] = { "FL", "R", "L", "FR" };

int main(void) {
	/* 表そのものを確かめるので、会場ごとの倍率(WallDist_Scale、WallDistance.c で
	 * 会場の値に書き換える)は1にしておく。倍率の働きは下で別に確かめる */
	float scale_saved[4];
	for (int s = 0; s < 4; s++) {
		scale_saved[s] = WallDist_Scale[s];
		WallDist_Scale[s] = 1.0f;
	}
	/* 書き写した倍率が明らかにおかしくないか(0.5〜2倍) */
	for (int s = 0; s < 4; s++) {
		CHECK(scale_saved[s] > 0.5f && scale_saved[s] < 2.0f,
				"%s: WallDist_Scale = %.2f が範囲外", name[s], scale_saved[s]);
	}

	for (int s = 0; s < 4; s++) {
		/* センサ値が増えるほど距離は縮む(増えない)。表の並び間違いはここで引っかかる */
		float prev = WallDist_mm(s, 0);
		for (int v = 1; v <= 4095; v++) {
			float d = WallDist_mm(s, v);
			CHECK(d <= prev + 1e-4f, "%s: v=%d で距離が増えた (%.2f -> %.2f)",
					name[s], v, prev, d);
			prev = d;
		}
		/* 区画中心(84mm)が表の範囲内に入っていること */
		CHECK(WallDist_mm(s, 4095) < WALLDIST_CENTER_MM
				&& WallDist_mm(s, 0) > WALLDIST_CENTER_MM,
				"%s: 表の範囲が 84mm を含んでいない", name[s]);
	}

	/* 表の点そのものと、その中点での補間 */
	CHECK(WallDist_mm(WALLDIST_L, 237) == 85.0f, "L: 237 -> 85mm");
	CHECK(WallDist_mm(WALLDIST_R, 201) == 85.0f, "R: 201 -> 85mm");
	CHECK(WallDist_mm(WALLDIST_FL, 645) == 84.0f, "FL: 645 -> 84mm");
	CHECK(WallDist_mm(WALLDIST_FR, 627) == 84.0f, "FR: 627 -> 84mm");
	CHECK(WallDist_mm(WALLDIST_FL, 132) == 180.0f, "FL: 132 -> 180mm");
	CHECK(WallDist_mm(WALLDIST_FR, 155) == 168.0f, "FR: 155 -> 168mm");
	/* スラロームの前の直進は、境目(前壁まで174mm)から前壁までの距離を見る。
	 * 174mmより遠くまで表があること(端に張り付くとすぐ打ち切られる) */
	CHECK(WallDist_mm(WALLDIST_FL, 0) > 174.0f && WallDist_mm(WALLDIST_FR, 0) > 174.0f,
			"FL/FR: 表が 174mm より遠くまでない");
	/* 逆引き(距離 -> センサ値)と、会場ごとの倍率 */
	CHECK(WallDist_Value(WALLDIST_FL, 84.0f) == 645.0f, "FL: 84mm -> 645");
	CHECK(WallDist_Value(WALLDIST_L, 85.0f) == 237.0f, "L: 85mm -> 237");
	WallDist_Scale[WALLDIST_FL] = 1.2f;
	CHECK(WallDist_mm(WALLDIST_FL, 774) == 84.0f, "FL: 倍率1.2 で 774 -> 84mm");
	WallDist_Scale[WALLDIST_FL] = 1.0f;
	float mid = WallDist_mm(WALLDIST_L, (237 + 183) / 2);
	CHECK(mid > 90.9f && mid < 91.1f, "L: 210 -> 91mm (got %.2f)", mid);

	if (fail_count == 0) {
		printf("OK\n");
		return 0;
	}
	printf("%d failures\n", fail_count);
	return 1;
}
