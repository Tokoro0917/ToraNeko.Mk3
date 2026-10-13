/*
 * DistHist.c
 *
 *  センサ値を走った距離ごとに残す履歴(説明は DistHist.h)
 */

#include "DistHist.h"
#include <math.h>

void DistHist_Reset(DistHist *h) {
	h->head = 0;
	h->grid = 0;
	h->count = 0;
	h->last_x = 0;
	h->last_v = 0;
	h->started = 0;
}

static void Put(DistHist *h, long g, float v) {
	h->head = (h->head + 1) % DISTHIST_N;
	h->v[h->head] = v;
	h->grid = g;
	if (h->count < DISTHIST_N) h->count++;
}

void DistHist_Push(DistHist *h, float x, float v) {
	if (!h->started) {
		/* 最初のサンプルは、その手前の点にそのまま置く */
		h->count = 0;
		Put(h, (long) floorf(x / DISTHIST_STEP_MM), v);
		h->last_x = x;
		h->last_v = v;
		h->started = 1;
		return;
	}
	if (x <= h->last_x) {
		/* 止まっている・わずかに下がったときは、距離を進めずに値だけ入れ替える */
		h->last_v = v;
		return;
	}
	long g_end = (long) floorf(x / DISTHIST_STEP_MM);
	long g = h->grid + 1;
	/* 長く入れていなかったときは、残せる分(最後の DISTHIST_N 点)だけ埋める */
	if (g_end - g + 1 > DISTHIST_N) {
		g = g_end - DISTHIST_N + 1;
	}
	for (; g <= g_end; g++) {
		float gx = g * DISTHIST_STEP_MM;
		float r = (gx - h->last_x) / (x - h->last_x);
		if (r < 0) r = 0;
		Put(h, g, h->last_v + (v - h->last_v) * r);
	}
	h->last_x = x;
	h->last_v = v;
}

int DistHist_At(const DistHist *h, float x, float *out) {
	if (!h->started) return 0;
	float head_x = h->grid * DISTHIST_STEP_MM;
	if (x >= head_x) {
		/* いちばん新しい点と、最後のサンプルの間 */
		if (x > h->last_x) return 0;
		float span = h->last_x - head_x;
		float r = (span > 0) ? (x - head_x) / span : 1.0f;
		*out = h->v[h->head] + (h->last_v - h->v[h->head]) * r;
		return 1;
	}
	/* 点どうしの間。k 点前と k+1 点前を使う */
	float back = (head_x - x) / DISTHIST_STEP_MM;
	int k = (int) back;
	float r = back - k;
	if (k > h->count - 1) return 0;
	float a = h->v[(h->head - k + DISTHIST_N) % DISTHIST_N];
	if (k == h->count - 1) {
		/* いちばん古い点ちょうどなら、その値 */
		if (r > 0) return 0;
		*out = a;
		return 1;
	}
	float b = h->v[(h->head - k - 1 + DISTHIST_N) % DISTHIST_N];
	*out = a + (b - a) * r;
	return 1;
}

float DistHist_LastX(const DistHist *h) {
	return h->last_x;
}

float DistHist_Max(const DistHist *h, float len) {
	if (!h->started) return 0;
	float m = h->last_v;
	int n = (int) (len / DISTHIST_STEP_MM);
	if (n > h->count) n = h->count;
	for (int k = 0; k < n; k++) {
		float v = h->v[(h->head - k + DISTHIST_N) % DISTHIST_N];
		if (v > m) m = v;
	}
	return m;
}

float DistHist_Length(const DistHist *h) {
	if (!h->started) return 0;
	return h->last_x - (h->grid - (h->count - 1)) * DISTHIST_STEP_MM;
}
