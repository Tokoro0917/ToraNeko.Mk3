# 迷路（App/maze/）の PC 上でのテスト
#   make        ビルドしてテストを実行する
#   make clean  ビルドしたものを消す

APP_DIR = ../TORANEKO.Mk3/App

CFLAGS = -std=c11 -Wall -Wextra -Werror -g -I$(APP_DIR)

MAZE_SOURCES = \
	$(APP_DIR)/maze/maze_map.c \
	$(APP_DIR)/maze/search.c

test: test_maze
	./test_maze

test_maze: test_maze.c $(MAZE_SOURCES)
	$(CC) $(CFLAGS) -o $@ test_maze.c $(MAZE_SOURCES)

clean:
	rm -f test_maze

.PHONY: test clean
