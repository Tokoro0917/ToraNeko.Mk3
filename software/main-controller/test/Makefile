# App/ の PC 上でのテスト
#   make        ビルドしてテストを実行する
#   make clean  ビルドしたものを消す
#
#   test_maze  迷路（App/maze/）と探索走行（App/sys/explore.c）、最短走行の大回り・斜め（App/sys/shortest.c）
#   test_mode  モード選択と操作（App/sys/mode.c、ui.c、goal.c）
#   test_output  出力と走行ログ（App/sys/output.c、log.c）。./test_output --dump ファイル で実機の出力を確かめる
#   test_flash  迷路とゴールの位置の保存（App/sys/flash.c）。Flash は仮の配列
#   test_trajectory  目標の速さを作る処理（App/ctrl/trajectory.c）
#   test_wall_dist  壁センサの値を距離 [mm] に直す処理（App/ctrl/wall_dist.c）
#   test_turn  最短走行のターンが決めた位置で終わるか（App/config/turn_params.h）。./test_turn --calc で値を計算する

APP_DIR = ../TORANEKO.Mk3/App

CFLAGS = -std=c11 -Wall -Wextra -Werror -O2 -g -I$(APP_DIR)

# App/ のヘッダ全部。どれかを書き換えたら、全部のテストを作り直す（古いテストのまま通ってしまわないように）
APP_HEADERS = $(wildcard $(APP_DIR)/*/*.h)

MAZE_SOURCES = \
	$(APP_DIR)/maze/maze_map.c \
	$(APP_DIR)/maze/search.c \
	$(APP_DIR)/maze/path.c \
	$(APP_DIR)/sys/explore.c \
	$(APP_DIR)/sys/shortest.c

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

FLASH_SOURCES = \
	$(APP_DIR)/sys/flash.c \
	$(APP_DIR)/maze/maze_map.c

test: test_maze test_mode test_output test_flash test_trajectory test_wall_dist test_turn
	./test_mode
	./test_trajectory
	./test_wall_dist
	./test_turn
	./test_output
	./test_flash
	./test_maze

# dijkstra.c は test_maze.c の中で #include している（テストで中の関数を使うため）
test_maze: test_maze.c $(MAZE_SOURCES) $(APP_DIR)/maze/dijkstra.c $(APP_HEADERS)
	$(CC) $(CFLAGS) -o $@ test_maze.c $(MAZE_SOURCES) -lm

test_mode: test_mode.c $(MODE_SOURCES) $(APP_HEADERS)
	$(CC) $(CFLAGS) -o $@ test_mode.c $(MODE_SOURCES)

test_output: test_output.c $(OUTPUT_SOURCES) $(APP_HEADERS)
	$(CC) $(CFLAGS) -o $@ test_output.c $(OUTPUT_SOURCES) -lm

test_trajectory: test_trajectory.c $(APP_DIR)/ctrl/trajectory.c $(APP_HEADERS)
	$(CC) $(CFLAGS) -o $@ test_trajectory.c $(APP_DIR)/ctrl/trajectory.c -lm

test_wall_dist: test_wall_dist.c $(APP_DIR)/ctrl/wall_dist.c $(APP_HEADERS)
	$(CC) $(CFLAGS) -o $@ test_wall_dist.c $(APP_DIR)/ctrl/wall_dist.c -lm

test_turn: test_turn.c $(APP_DIR)/ctrl/trajectory.c $(APP_HEADERS)
	$(CC) $(CFLAGS) -o $@ test_turn.c $(APP_DIR)/ctrl/trajectory.c -lm

test_flash: test_flash.c $(FLASH_SOURCES) $(APP_HEADERS)
	$(CC) $(CFLAGS) -o $@ test_flash.c $(FLASH_SOURCES)

clean:
	rm -f test_maze test_mode test_output test_flash test_trajectory test_wall_dist test_turn

.PHONY: test clean
