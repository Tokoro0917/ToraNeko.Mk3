/*
 * side_edge.h
 * 横のセンサで、壁の終わり・柱の端（値が下がるところ）を見つける（中身は ctrl/side_edge.c）
 */
#ifndef CTRL_SIDE_EDGE_H
#define CTRL_SIDE_EDGE_H

#include <stdbool.h>
#include <stdint.h>

void side_edge_reset(void);
void side_edge_update(float distance_mm, int32_t left_value, int32_t right_value, bool is_straight);

/* 見つけた端の記録（左右それぞれ、いちばん新しいもの）。is_left が true なら左、false なら右。
 *   count       ：これまでに見つけた数（増えたら、新しい端を見つけた）
 *   distance_mm ：見つけた端の、走った距離（エンコーダで測った値）[mm]
 *   is_pillar   ：柱だけ（前も後ろも壁がない）なら true、壁の終わりなら false
 *   boundary_mm ：端の位置から求めた、区画の境目の、走った距離 [mm] */
int side_edge_get_count(bool is_left);
float side_edge_get_distance_mm(bool is_left);
bool side_edge_is_pillar(bool is_left);
float side_edge_get_boundary_mm(bool is_left);

#endif /* CTRL_SIDE_EDGE_H */
