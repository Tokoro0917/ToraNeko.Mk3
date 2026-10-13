/*
 * PillarEdge.c
 *
 *  柱の端の検出(説明は PillarEdge.h と docs/pillar_cut_design.md)
 */

#include "PillarEdge.h"

float Edge_High_Len = 30;
float Edge_Min_High = 90;
float Edge_Rearm_Ratio = 0.75f;
float Edge_Plateau_Wall = 20;
float Edge_D_Back = 12;
float Edge_Min_Hist = 15;

#define EDGE_D_MAX 120.0f	//これより遠い距離は壁とみなさない(横壁制御の WallCtrl_mm_MaxDist と同じ)

void PillarEdge_Reset(PillarEdge *e) {
	DistHist_Reset(&e->v);
	DistHist_Reset(&e->d);
	e->armed = 0;
	e->high_from = 0;
	e->has_prev = 0;
}

int PillarEdge_Update(PillarEdge *e, float x, float v, float d) {
	DistHist_Push(&e->v, x, v);
	DistHist_Push(&e->d, x, d);
	x = DistHist_LastX(&e->v);	//止まっている・わずかに下がったときは進めない
	if (!e->has_prev || x <= e->prev_x) {
		e->prev_x = x;
		e->prev_v = v;
		e->has_prev = 1;
		return 0;
	}
	int found = 0;
	float h = DistHist_Max(&e->v, Edge_High_Len);
	if (h < Edge_Min_High || DistHist_Length(&e->v) < Edge_Min_Hist) {
		e->armed = 0;
	} else if (v >= h * Edge_Rearm_Ratio) {
		if (!e->armed) {
			e->armed = 1;
			e->high_from = x;
		}
	} else if (e->armed && v < h * 0.5f && e->prev_v >= h * 0.5f) {
		/* 半分を切った。前のサンプルとの間を直線で補間する */
		float level = h * 0.5f;
		float r = (e->prev_v - level) / (e->prev_v - v);
		float xc = e->prev_x + (x - e->prev_x) * r;
		e->ev.x = xc;
		e->ev.high = h;
		e->ev.kind = (xc - e->high_from >= Edge_Plateau_Wall) ? EDGE_KIND_WALL : EDGE_KIND_PILLAR;
		float db = -1;
		if (e->ev.kind == EDGE_KIND_WALL && DistHist_At(&e->d, xc - Edge_D_Back, &db)
				&& db > EDGE_D_MAX) {
			db = -1;
		}
		e->ev.d_before = (e->ev.kind == EDGE_KIND_WALL) ? db : -1;
		e->armed = 0;
		found = 1;
	}
	e->prev_x = x;
	e->prev_v = v;
	return found;
}

int PillarEdge_Fuse(int have_r, float c_r, float h_r, int have_l, float c_l, float h_l,
		float tol, float *c) {
	if (have_r && have_l) {
		float diff = c_r - c_l;
		if (diff < 0) diff = -diff;
		if (diff <= tol) {
			*c = (c_r + c_l) * 0.5f;
			return EDGE_SRC_R | EDGE_SRC_L;
		}
		if (h_r >= h_l) {
			*c = c_r;
			return EDGE_SRC_R | EDGE_SRC_DISAGREE;
		}
		*c = c_l;
		return EDGE_SRC_L | EDGE_SRC_DISAGREE;
	}
	if (have_r) {
		*c = c_r;
		return EDGE_SRC_R;
	}
	if (have_l) {
		*c = c_l;
		return EDGE_SRC_L;
	}
	return 0;
}
