# App/ の PC 上でのテスト
#   make        ビルドしてテストを実行する
#   make clean  ビルドしたものを消す
#
#   test_maze  迷路（App/maze/）と探索走行（App/sys/explore.c）
#   test_mode  モード選択と操作（App/sys/mode.c、ui.c、goal.c）
#   test_output  出力と走行ログ（App/sys/output.c、log.c）。./test_output --dump ファイル で実機の出力を確かめる

APP_DIR = ../TORANEKO.Mk3/App

CFLAGS = -std=c11 -Wall -Wextra -Werror -O2 -g -I$(APP_DIR)

MAZE_SOURCES = \
	$(APP_DIR)/maze/maze_map.c \
	$(APP_DIR)/maze/search.c \
	$(APP_DIR)/maze/path.c \
	$(APP_DIR)/sys/explore.c

MODE_SOURCES = \
	$(APP_DIR)/sys/mode.c \
	$(APP_DIR)/sys/ui.c \
	$(APP_DIR)/sys/goal.c \
	$(APP_DIR)/maze/maze_map.c

OUTPUT_SOURCES = \
	$(APP_DIR)/sys/output.c \
	$(APP_DIR)/sys/log.c \
	$(APP_DIR)/maze/maze_map.c \
	$(APP_DIR)/maze/search.c \
	$(APP_DIR)/maze/path.c \
	$(APP_DIR)/maze/dijkstra.c

test: test_maze test_mode test_output
	./test_mode
	./test_output
	./test_maze

# dijkstra.c は test_maze.c の中で #include している（テストで中の関数を使うため）
test_maze: test_maze.c $(MAZE_SOURCES) $(APP_DIR)/maze/dijkstra.c
	$(CC) $(CFLAGS) -o $@ test_maze.c $(MAZE_SOURCES)

test_mode: test_mode.c $(MODE_SOURCES)
	$(CC) $(CFLAGS) -o $@ test_mode.c $(MODE_SOURCES)

test_output: test_output.c $(OUTPUT_SOURCES)
	$(CC) $(CFLAGS) -o $@ test_output.c $(OUTPUT_SOURCES)

clean:
	rm -f test_maze test_mode test_output

.PHONY: test clean
