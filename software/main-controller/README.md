# main-controller

ToraNeko.Mk3 メイン基板（STM32F411CEU6）のファームウェアです。

KOGUMA.MkIIl のコード（[../reference/koguma-mkiil/](../reference/koguma-mkiil/)）を参照しつつ、機能ごとに内容を確認しながら一つずつ実装していきます。
確認・実装が済んだものだけをこのディレクトリに置きます。

- [PIN_REQUIREMENTS.md](PIN_REQUIREMENTS.md): 確定したピン割り当て（回路図から抽出）と、KOGUMA との違い
- [CUBEMX_SETUP.md](CUBEMX_SETUP.md): CubeMX（`.ioc`）の設定値（ステップ0）
- [KOGUMA_PORTING.md](KOGUMA_PORTING.md): KOGUMA のコードの棚卸し（使っている機能・使っていない機能）と、移植する／しないの判断
- [MODES.md](MODES.md): 機体の操作（選ぶ・決める・スタート）、LED の表し方、モードの割り当て、迷路・ゴールの保存

## 決定事項

| 項目 | 決定 | 理由・補足 |
|---|---|---|
| 書き込み・デバッグ | STLINK-V3MODS（SWD） | SWDIO=PA13, SWCLK=PA14, NRST。SWO は使わない（PB3 は IMU の SPI1_SCK に使っている） |
| PCとの通信・printf | UART → STLINK-V3MODS の仮想COMポート | 書き込みとログがケーブル1本で済む。**USBは使わない**（PA11/PA12 を他用途に回せる） |
| UART | USART1 の **TX だけ**（PA9） | APB2 側なので高速化しやすい。受信はしない。RX に使えるピン（PA10/PB7/PB3）は LED2・モータ・IMU で埋まっているので、CubeMX では Single Wire になる（PA9 は USER CODE 欄でプッシュプルにしている。[CUBEMX_SETUP.md](CUBEMX_SETUP.md) の 7） |
| 開発環境 | **STM32CubeMX（コード生成、Toolchain = CMake）＋ VS Code（STM32Cube for Visual Studio Code 拡張）** | ピン・クロック設定は `.ioc`（CubeMX）で管理。ビルドは CMake + Ninja + arm-none-eabi-gcc なのでコマンドラインだけでビルドでき、CI や Claude の環境でもファームのビルドを確認できる。CubeIDE は使わない |
| ビルド | `cmake --preset Debug && cmake --build --preset Debug` | CubeMX が生成する `CMakePresets.json` をそのまま使う。成果物は `build/Debug/TORANEKO.Mk3.elf` |
| ライブラリ | STM32 HAL | 制御周期の割り込み内で重い箇所は、必要になったら LL/レジスタ直接に置き換える |
| プロジェクト名 | `TORANEKO.Mk3` | KiCad プロジェクトと揃える |
| クロック | HSE 10MHz（セラミック発振子 CSTNE10M0G52Z000R0）→ SYSCLK 100MHz | PLL: M=5, N=200, P=4（VCO 400MHz）。APB1=50MHz、APB2=100MHz、タイマークロックは全て100MHz。USBを使わないので48MHzは不要 |
| 制御周期 | **2kHz**（`config/robot_params.h` の `CTRL_FREQ_HZ` で変更可能にする） | 処理時間の見積りは CPU 使用率 約10〜18%。実機で余裕を測り、余裕があれば 4kHz（壁センサは 2kHz のまま）に上げる。PID ゲイン等は周期から換算する作りにする |
| 制御周期のタイマー | TIM11 | ピンを使わない内部タイマー |
| 壁センサの点灯・測定の順番制御 | TIM3 で ADC を起動し、DMA 完了割り込みで次の LED に進める連鎖方式 | CPU を待たせずに 4 組を毎周期測定できる（2組ずつ同時点灯なので連鎖全体で約 100µs）。詳細はステップ6 |
| 処理時間計測・µs 待ち | TIM5（32bit）を 1MHz でフリーラン | 制御割り込みの開始・終了時刻を記録して CPU 使用率を常に確認できるようにする |
| ms 単位の待ち | SysTick（`HAL_GetTick()`） | KOGUMA の「共通カウンタを 0 に戻す `wait_ms`」はやめる |
| 割り込み優先度 | 壁センサの連鎖（ADC の DMA）> 制御周期（TIM11）> ログ送信（USART の DMA） | 制御処理が長引いてもセンサ測定が遅れないようにする |
| ファイル構成・命名規則 | 下記「ファイル構成」「命名規則」の通り | 生成コードと自作コードを分離し、自作コードは層ごとにフォルダ分け |
| KOGUMA からの移植方針 | 使っていない機能は持ち込まない | 関数・モード単位の棚卸しと判断は [KOGUMA_PORTING.md](KOGUMA_PORTING.md) |
| 操作とモード | タイヤを回して選び、スイッチ SW1 で決定、前センサに手をかざして離すとスタート。モード0 探索（すべて吸引なし）、1 最短走行、2 探索のターン調整、3 最短走行のターン調整（速さはプログラムの変数で選ぶ）、4 実験用（中身は自由に入れ替える）、5 センサ（値の表示と調整）、6 基本調整、7 出力と設定 | 2026-10-14 に決定。詳しくは [MODES.md](MODES.md)。ブザーはないので合図は LED |

## ファイル構成

```
software/main-controller/
├── README.md                     決定事項・ファイル構成・実装ステップ（このファイル）
├── PIN_REQUIREMENTS.md           ピン割り当ての論点
├── CUBEMX_SETUP.md               .ioc の設定値
├── KOGUMA_PORTING.md             KOGUMA コードの棚卸しと移植判断
├── MODES.md                      機体の操作、LED の表し方、モードの割り当て、迷路・ゴールの保存
│
└── TORANEKO.Mk3/                 CubeMX（CMake）プロジェクト
    ├── TORANEKO.Mk3.ioc          CubeMX 設定（ピン・クロック・周辺機能）
    ├── CMakeLists.txt            ビルド設定（初回だけ生成。App/ の追加はここに書く）
    ├── CMakePresets.json         Debug / Release のプリセット（生成）
    ├── cmake/
    │   ├── gcc-arm-none-eabi.cmake   ツールチェーン設定（生成。こちらを使う）
    │   ├── starm-clang.cmake         もう 1 つのツールチェーン設定（生成。使わない）
    │   └── stm32cubemx/CMakeLists.txt  Core/・Drivers/ のソース一覧（再生成のたびに上書き。編集しない）
    ├── startup_stm32f411xe.s     スタートアップ（生成）
    ├── STM32F411xx_FLASH.ld      リンカスクリプト（生成）
    │
    ├── Core/                     ── CubeMX が生成。USER CODE 欄以外は編集しない
    │   ├── Inc/
    │   │   ├── main.h            ピン名の定義（.ioc のラベルから生成）
    │   │   ├── adc.h  dma.h  gpio.h  spi.h  tim.h  usart.h
    │   │   ├── stm32f4xx_it.h
    │   │   └── stm32f4xx_hal_conf.h
    │   ├── Src/
    │   │   ├── main.c            初期化のあと app_init() / app_loop() を呼ぶだけ
    │   │   ├── adc.c  dma.c  gpio.c  spi.c  tim.c  usart.c   周辺機能の初期化
    │   │   ├── stm32f4xx_it.c    割り込みの入口（処理は App 側を呼ぶだけ）
    │   │   ├── stm32f4xx_hal_msp.c
    │   │   ├── system_stm32f4xx.c
    │   │   ├── syscalls.c        システムコール（生成。printf の出力先 _write は App/drv/uart.c にあり、そちらが使われる）
    │   │   └── sysmem.c
    │
    ├── Drivers/                  ── HAL / CMSIS（生成・編集しない。CubeMX なしでビルドできるようにコミットする）
    │
    ├── App/                      ── 自作コードはすべてここ。「（予定）」はまだないファイル、「（.h だけ）」は使う関数の形だけを先に決めたもの
    │   ├── app.c / app.h         app_init()、app_loop()。全体の入口
    │   │
    │   ├── config/               機体の定数
    │   │   ├── robot_params.h    タイヤ径、ギア比、トレッド幅、制御周期など
    │   │   └── ctrl_gains.h      PID ゲイン、フィードフォワード係数（予定）
    │   │
    │   ├── drv/                  ハードに直接触れる層（HAL を呼ぶのは、ここと sys/scheduler.c だけ）
    │   │   ├── encoder.c/.h      AS5047P ×2：角度の読み取り（.h だけ）
    │   │   ├── imu.c/.h          LSM6DSR：角速度・加速度の読み取り（予定）
    │   │   ├── motor.c/.h        MP6551 ×2：PWM の出力（電圧を指定）（予定）
    │   │   ├── wall.c/.h         壁センサ：LED を2組ずつ点灯し、AD 値を読む（予定）
    │   │   ├── battery.c/.h      バッテリ電圧の読み取り（予定）
    │   │   ├── fan.c/.h          吸引ファンの PWM（予定）
    │   │   ├── flash_memory.c/.h 内蔵 Flash の保存場所（セクタ6・7）を消す・書く・読む
    │   │   ├── led.c/.h          UI 用 LED
    │   │   ├── switch.c/.h       ユーザースイッチ SW1 の読み取り
    │   │   ├── spi_bus.c/.h      SPI 転送と CS 制御の共通部分（imu / encoder から使う）（予定）
    │   │   ├── timebase.c/.h     経過時間（ms は SysTick。µs は TIM5 で、ステップ2で追加する）とシステムクロックの取得
    │   │   └── uart.c/.h         ログ送信（DMA 送信・リングバッファ）。printf の出力先（`_write`）もここ
    │   │
    │   ├── ctrl/                 制御
    │   │   ├── odometry.c/.h     エンコーダと IMU から速度・距離・角度を求める（予定）
    │   │   ├── pid.c/.h          汎用の PID 計算（予定）
    │   │   ├── trajectory.c/.h   台形加減速などの目標速度の生成（予定）
    │   │   ├── wall_ctrl.c/.h    壁のあり・なし判定、壁に沿った姿勢の補正（.h だけ）
    │   │   └── run.c/.h          直進・スラローム・斜めなどの動作（.h だけ。run.c で守ることも run.h に書いた）
    │   │
    │   ├── maze/                 迷路（HAL に依存しない。PC でテストできる。ctrl を知らない）
    │   │   ├── maze_map.c/.h     壁の情報の保持・更新、最後に壁を記録した 5 区画ぶんを戻す（保存は sys/flash.c）
    │   │   ├── search.c/.h       足立法・歩数マップ・未知壁探索で見に行く壁（行き止まり潰しは持ち込まない）
    │   │   ├── dijkstra.c/.h     ダイクストラ：スタートからゴールまでの速く走れる道（通る区画の並び）を探す
    │   │   └── path.c/.h         経路（動きの並び）を作る：既知区間の経路（直進・小回り・U ターン。大回りにはしない）、最短経路の大回り・斜めへの変換
    │   │
    │   └── sys/                  全体の管理
    │       ├── scheduler.c/.h    制御周期（2kHz）の割り込みで制御の処理を呼び、処理時間と間隔を記録する（中身の処理はステップ3 以降で足す）
    │       ├── explore.c/.h      探索走行：maze に次の行き先を聞き、run で 1 区画動かす（maze と ctrl をつなぐ唯一の場所）
    │       ├── mode.c/.h         走行モードの選択と実行（モード表）
    │       ├── goal.c/.h         ゴールの位置（機体で入れて Flash に保存する。大きさは 2×2）
    │       ├── ui.c/.h           モード選択の操作、LED での表示と合図（ブザーはない）
    │       ├── log.c/.h          走行ログの記録と吸い出し
    │       ├── output.c/.h       出力（迷路と走行ログを、KOGUMA のビューアで読める形で PC へ出す）
    │       ├── failsafe.c/.h     異常の検知（衝突、電圧低下など）と停止（.h だけ）
    │       └── flash.c/.h        迷路とゴールの位置の Flash への保存と読み込み（何をどの順番で書くか。HAL は使わない）
    │
    └── （build/ はビルド成果物。.gitignore で除外）

software/main-controller/test/    ── PC 上で動かすテスト（CI で実行）。ホストの gcc でビルドするので、ARM 向けの CMake プロジェクトとは分ける
    ├── test_maze.c               迷路探索・最短経路・探索走行のテスト（../TORANEKO.Mk3/App/maze/*.c と sys/explore.c をリンク。探索走行は、本当の迷路の上を動く「仮の機体」で走らせる）。`make` で実行
    ├── test_mode.c               モード選択と操作のテスト（sys/mode.c・ui.c・goal.c をリンク。時刻ごとの入力の予定を与えて、LED と結果を調べる）
    ├── test_output.c             出力と走行ログのテスト（sys/output.c・log.c をリンク。出力を読み直して調べる）。`./test_output --dump ファイル` で、実機の出力（ビューアで保存したファイル）の最短経路が PC と同じか確かめる
    ├── test_flash.c              迷路とゴールの位置の保存のテスト（sys/flash.c をリンク。Flash の代わりに配列を使う。壊れた保存を読まないかも調べる）
    ├── Makefile
    └── mazes/                    テスト用の迷路（32×32 のテキスト。開けば迷路の形が見える）と、それを作る gen_maze.py
```

### 層の依存関係

```
sys（explore / mode / ...）
 ├→ ctrl  →  drv  →  HAL（Core/, Drivers/）
 └→ maze
```

- 呼び出しは上の層から下の層への一方向のみ。`drv` から `ctrl` を呼ぶような逆向きの呼び出しはしない
- `ctrl` と `maze` はお互いを知らない。「迷路で次の行き先を決める → その動作を走らせる → 壁を見て迷路を更新する」の繰り返しは `sys/explore.c` だけが書く（KOGUMA では `Move.c` の `Robot_Maze_*_Action` に迷路判断と動作が混ざっていた）
- `drv/` 内の共通部分（`spi_bus`）は `drv/` 内からだけ使う
- HAL を直接呼ぶのは `drv/` と `sys/scheduler.c`（タイマー割り込みの起動）だけ
- `maze/` は `drv` も HAL も使わない。PC 上でそのままテストできる
- `Core/` 側の生成コードには、USER CODE 欄に `App/` の関数呼び出しを1行書くだけにする（CubeMX で再生成しても自作コードが消えないように）
- `App/` のソースとインクルードパスは `TORANEKO.Mk3/CMakeLists.txt` に追加する（手順は [CUBEMX_SETUP.md](CUBEMX_SETUP.md) の「CMake のビルド設定」）。`cmake/stm32cubemx/` は再生成で上書きされるので触らない

## 命名規則

| 対象 | 規則 | 例 |
|---|---|---|
| ファイル名 | 小文字＋アンダースコア。層はフォルダで表し、ファイル名には付けない | `drv/encoder.c`, `ctrl/pid.c` |
| 関数名 | `モジュール名_動詞_目的語`（小文字＋アンダースコア） | `encoder_read_angle()`, `motor_set_voltage()` |
| 型名 | 小文字＋アンダースコア＋`_t` | `pid_ctrl_t`, `odometry_state_t`（`pid_t` は標準ライブラリと衝突するので使わない） |
| ファイルをまたぐ変数 | `g_` を付ける（なるべく使わず、関数経由で受け渡す） | `g_maze_map` |
| ファイル内だけの変数・関数 | `static` を付ける | `static float s_prev_error;` |
| 定数・マクロ | 大文字＋アンダースコア | `MAZE_SIZE`, `CTRL_PERIOD_S` |
| インクルードガード | `ファイルパスを大文字に_H` | `DRV_ENCODER_H` |

## 実装の進め方

各ステップで「KOGUMA の該当コードを読む → ToraNeko で変える点を決める → 実装 → 実機/PCで確認」を行います。

| # | 内容 | 作るファイル | KOGUMA側の対応ファイル | 状態 |
|---|---|---|---|---|
| 0 | F411 用の空プロジェクト作成（CubeMX → CMake、クロック設定） | `.ioc`, `Core/`, `App/app.c`, `CMakeLists.txt` | `KOGUMA.MkIIl.ioc` | **完了**。CubeMX で生成、`App/app.c` を `main.c` から呼ぶところまで。CI でビルドを確認 |
| 1 | LED点灯とシリアル出力（printf） | `drv/led.c`, `drv/uart.c`, `drv/timebase.c` | `UI.c`, `syscalls.c` | **完了**（2026-10-22 に実機で確認）：LED1→7 の順送り、1 秒ごとに経過時間をシリアルに出力。書き込みは VS Code の F5、シリアルは VS Code の Serial Monitor 拡張で見られた |
| 2 | 2kHz 制御周期割り込み | `sys/scheduler.c` | `PL_timer.c` | 実装済み・実機確認待ち（TIM11 の割り込みで 0.5ms ごとに呼ぶ。TIM5 で µs を数え、1 回の処理時間と割り込みの間隔を 1 秒ごとにシリアルに出す。周期は `CTRL_FREQ_HZ` から決める） |
| 3 | エンコーダ読み取り（AS5047P） | `drv/spi_bus.c`, `drv/encoder.c` | `PL_encoder.c` | 未着手 |
| 4 | IMU 読み取り（LSM6DSR） | `drv/imu.c` | `lsm6dsr.c` | 未着手 |
| 5 | モータ PWM | `drv/motor.c` | `motor.c`（出力部分） | 未着手 |
| 6 | 壁センサ（2組ずつ点灯） | `drv/wall.c`, `ctrl/wall_ctrl.c` | `PL_sensor.c`, `Wallsensor.c` | 未着手 |
| 7 | 速度・角度制御（PID、フィードフォワード） | `ctrl/odometry.c`, `ctrl/pid.c`, `ctrl/trajectory.c`, `ctrl/run.c` | `motor.c`, `Move.c` | 未着手 |
| 8 | 迷路探索と最短経路 | `maze/*`, `../test/` | `Maze.c` | **迷路の処理は完了**（PC テスト済み）：壁の地図・歩数マップ・足立法（`maze_map.c`, `search.c`）、既知区間の経路（直進・小回り・U ターン）と最短経路の大回り・斜め（`path.c`）、ダイクストラ（`dijkstra.c`）、未知壁探索（`search_plan_unknown_walls` / `search_check_unknown_walls`）。2026-10-13 に KOGUMA の最新版（ダイクストラの不具合修正、既知区間は小回りだけ、ダイクストラは止まっているときだけ計算）に合わせた。ダイクストラは、KOGUMA と違い「同じ向きに続けて進む区間」ごとに広げて、いつも本当の最短を返すようにした（2026-10-14）。走りながら呼び出す部分（`sys/explore.c`）はステップ9 |
| 9 | 探索走行、走行モード、ログ、フェイルセーフ、保存 | `sys/explore.c`, `sys/mode.c`, `sys/ui.c`, `sys/goal.c`, `sys/log.c`, `sys/output.c`, `sys/failsafe.c`, `sys/flash.c`, `drv/flash_memory.c`, `drv/switch.c` | `main.c`, `UI.c`, `LOG.c`, `Failsafe.c` | **探索走行（`sys/explore.c`）は作った**（PC テスト済み）：往復・片道・未知壁探索、U ターンと計算のときに止まる、スタート着・片道のゴール着・フェイルセーフで Flash に保存、フェイルセーフでは最後の 5 区画ぶんの壁を戻す（`maze_map_undo_recent_cells`）。機体を動かす部分などは、使う関数の形（`ctrl/run.h`、`ctrl/wall_ctrl.h`、`sys/failsafe.h`）だけを決めてあり、中身はステップ6・7・9 で作る。中身ができるまで、`explore.c` はファームウェアのビルドには入れない（PC テストだけでビルドする）。**モード選択と操作（`sys/mode.c`・`sys/ui.c`・`sys/goal.c`）も作った**（PC テスト済み）：タイヤで選んでスイッチで決定、前センサに手をかざして離すと 1 秒後に始まる、LED の表示と合図、モード0 の探索、モード7 のゴールの位置の入力（No.2）と迷路呼び出し（No.15）。タイヤ・前センサ・フェイルセーフも、使う関数の形（`drv/encoder.h`、`ctrl/wall_ctrl.h`、`sys/failsafe.h`）だけを決めてあり、中身ができるまでファームウェアのビルドには入れない（スイッチの `drv/switch.c` は中身まで作ってビルドに入れた）。**走行ログと出力（`sys/log.c`・`sys/output.c`）も作った**（PC テスト済み。ファームウェアのビルドに入れた）：1000 行、間隔は走らせる処理ごと、8 秒を超えたら記録をやめるだけ（`full=1`）、KOGUMA のビューアで読める形、ログ出力はモード7 No.1。小数の printf のためにリンクに `-u _printf_float` を足した。ログの値を集めて `log_record()` を呼ぶのは、制御（ステップ7）を作るとき。**迷路とゴールの位置の Flash への保存（`sys/flash.c`・`drv/flash_memory.c`）も作った**（PC テスト済み。ファームウェアのビルドに入れた）：ゴールはセクタ6、迷路はセクタ7、目印と確かめの数で壊れた保存を読まない。本物の Flash で確かめるのは、モード7 をファームウェアに入れたとき。ファームウェアのビルドに入れたもの（`log.c`・`output.c`・`flash.c`・`maze/` など）は、まだ `app.c` から呼んでいないので、リンクのときに外れる（RAM の見積り 115KB も、まだビルドでは確かめられない。モード選択をファームウェアに入れたときに確かめる）。そのほか（フェイルセーフ、センサ値の表示）は未着手。操作とモードは [MODES.md](MODES.md)、ログ・出力・フェイルセーフは [KOGUMA_PORTING.md](KOGUMA_PORTING.md) の B・F で決定済み |

### KOGUMA のファイルとの対応

| KOGUMA | ToraNeko |
|---|---|
| `PL_encoder.c` | `drv/encoder.c`、`drv/spi_bus.c`、`sys/ui.c`（タイヤを回してモードを選ぶ部分） |
| `lsm6dsr.c` | `drv/imu.c`、`drv/spi_bus.c` |
| `PL_sensor.c` | `drv/wall.c`（LED点灯・AD読み取り）、`drv/battery.c` |
| `Wallsensor.c` | `ctrl/wall_ctrl.c` |
| `motor.c` | `drv/motor.c`（PWM出力）、`ctrl/pid.c`、`ctrl/odometry.c`、`drv/fan.c` |
| `Move.c` | `ctrl/trajectory.c`、`ctrl/run.c`（動作）、`sys/explore.c`（`Robot_Maze_*_Action` の迷路判断部分） |
| `Maze.c` | `maze/maze_map.c`、`maze/search.c`、`maze/path.c`、`maze/dijkstra.c` |
| `PL_timer.c` | `sys/scheduler.c` |
| `main.c`（モード選択部分） | `sys/mode.c`、`sys/explore.c` |
| `UI.c` | `sys/ui.c`、`drv/led.c`（ブザーの部分は持ち込まない） |
| `LOG.c` | `sys/log.c` |
| `Output_All`（`LOG.c`）、`Maze_Debug_Dump`（`Maze.c`） | `sys/output.c` |
| `Failsafe.c` | `sys/failsafe.c` |
| `Define.h` | `config/robot_params.h`、`config/ctrl_gains.h` |
