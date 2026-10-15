# 迷路（App/maze/）の PC 上でのテスト
#   make        ビルドしてテストを実行する
#   make clean  ビルドしたものを消す

APP_DIR = ../TORANEKO.Mk3/App

CFLAGS = -std=c11 -Wall -Wextra -Werror -O2 -g -I$(APP_DIR)

MAZE_SOURCES = \
	$(APP_DIR)/maze/maze_map.c \
	$(APP_DIR)/maze/search.c \
	$(APP_DIR)/maze/path.c \
	$(APP_DIR)/sys/explore.c

test: test_maze
	./test_maze

# dijkstra.c は test_maze.c の中で #include している（テストで中の関数を使うため）
test_maze: test_maze.c $(MAZE_SOURCES) $(APP_DIR)/maze/dijkstra.c
	$(CC) $(CFLAGS) -o $@ test_maze.c $(MAZE_SOURCES)

clean:
	rm -f test_maze

.PHONY: test clean
