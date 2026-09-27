/*
 * Move.h
 *
 *  Created on: 2023/08/29
 *      Author: akihi
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

extern int G_Pass_before;
extern int G_Pass_after;

void Robot_adjustment();
void Robot_adjustment_180();
void Robot_adjustment_Back();
void Robot_adjustment_Back_Only();

void Robot_Maze_Sula_Action();
void Robot_Maze_Suction_Action();
void Robot_Maze_Pass_Action();

void Sula_Shortest_Move300(int, int);
void Sula_Shortest_Move500(int, int);

void Short_NANAME_Move1000(int, int);
void Short_NANAME_Move2000(int, int);
void Short_NANAME_Move2400(int, int);
void Short_NANAME_Move2700(int, int);

void Short_Dijkstra_Move2000(int, int);

//void Sula_Shortest_Move800();
//void Sula_Shortest_Move1100();
//void Sula_Shortest_Move1000(int, int);
//void Sula_Shortest_Move1200();
//void Shot_NANAME_Move1000(int, int);
//void Adjust_Speed(int);
//void Shot_NANAME_Move1700_FUN(int, int);
//void Shot_NANAME_Move2000_FUN(int, int);

#endif /* INC_MOVE_H_ */
