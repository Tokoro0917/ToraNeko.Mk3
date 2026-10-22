/*
 * run.c
 * 機体を動かす（直進・スラローム・止まる・発進など）
 *
 * ■ 今は仮
 * 速度と角度の制御（ステップ7）がまだないので、動きを頼まれたら、走らずにフェイルセーフで止まる。
 * モード0（探索）を選んでスタートすると、すぐに 7 個の LED が全部点滅する（スイッチで解除）。
 * run.h の約束「フェイルセーフで止まったときは、すぐ戻る」は守っている。
 * ステップ7で、本当に走る中身に置き換える。
 */
#include "ctrl/run.h"

#include "sys/failsafe.h"

/* まだ走れないので、フェイルセーフで止まる */
static void cannot_run_yet(void)
{
  failsafe_stop();
}

void run_search_start(void)
{
  cannot_run_yet();
}

void run_search_straight(void)
{
  cannot_run_yet();
}

void run_search_turn_left(void)
{
  cannot_run_yet();
}

void run_search_turn_right(void)
{
  cannot_run_yet();
}

void run_search_stop_at_center(void)
{
  cannot_run_yet();
}

void run_search_go_from_center(run_turn_t turn)
{
  (void)turn;  /* 今は使わない（使わない引数の警告を消すため） */
  cannot_run_yet();
}
