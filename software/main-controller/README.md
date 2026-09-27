# main-controller

ToraNeko.Mk3 メイン基板（STM32F411CEU6）のファームウェアです。

KOGUMA.MkIIl のコード（[../reference/koguma-mkiil/](../reference/koguma-mkiil/)）を参照しつつ、機能ごとに内容を確認しながら一つずつ実装していきます。
確認・実装が済んだものだけをこのディレクトリに置きます。

- [PIN_REQUIREMENTS.md](PIN_REQUIREMENTS.md): 確定したピン割り当て（回路図から抽出）と、KOGUMA との違い
- [CUBEMX_SETUP.md](CUBEMX_SETUP.md): CubeMX（`.ioc`）の設定値（ステップ0）

## 決定事項

| 項目 | 決定 | 理由・補足 |
|---|---|---|
| 書き込み・デバッグ | STLINK-V3MODS（SWD） | SWDIO=PA13, SWCLK=PA14, NRST。SWO（PB3）は任意 |
| PCとの通信・printf | UART → STLINK-V3MODS の仮想COMポート | 書き込みとログがケーブル1本で済む。**USBは使わない**（PA11/PA12 を他用途に回せる） |
| UART | USART1（PA9 TX / PA10 RX）を推奨 | APB2 側なので高速化しやすい。PA2/PA3（USART2）は ADC 入力と取り合いになるため避ける |
| 開発環境 | STM32CubeIDE | KOGUMA と同じ。ピン・クロック設定は `.ioc`（CubeMX）で管理 |
| ライブラリ | STM32 HAL | 1kHz 割り込み内で重い箇所は、必要になったら LL/レジスタ直接に置き換える |
| プロジェクト名 | `TORANEKO.Mk3` | KiCad プロジェクトと揃える |
| クロック | HSE 水晶 10MHz → SYSCLK 100MHz | PLL: M=5, N=200, P=4（VCO 400MHz）。APB1=50MHz、APB2=100MHz、タイマークロックは全て100MHz。USBを使わないので48MHzは不要 |
| 制御周期 | **2kHz**（`config/robot_params.h` の `CTRL_FREQ_HZ` で変更可能にする） | 処理時間の見積りは CPU 使用率 約10〜18%。実機で余裕を測り、余裕があれば 4kHz（壁センサは 2kHz のまま）に上げる。PID ゲイン等は周期から換算する作りにする |
| 制御周期のタイマー | TIM11 | ピンを使わない内部タイマー |
| 壁センサの点灯・測定の順番制御 | TIM3 で ADC を起動し、DMA 完了割り込みで次の LED に進める連鎖方式 | CPU を待たせずに 4 組を毎周期測定できる（連鎖全体で約 170〜200µs）。詳細はステップ6 |
| 処理時間計測・µs 待ち | TIM5（32bit）を 1MHz でフリーラン | 制御割り込みの開始・終了時刻を記録して CPU 使用率を常に確認できるようにする |
| ms 単位の待ち | SysTick（`HAL_GetTick()`） | KOGUMA の「共通カウンタを 0 に戻す `wait_ms`」はやめる |
| 割り込み優先度 | 壁センサの連鎖（ADC の DMA）> 制御周期（TIM11）> ログ送信（USART の DMA） | 制御処理が長引いてもセンサ測定が遅れないようにする |
| ファイル構成・命名規則 | 下記「ファイル構成」「命名規則」の通り | 生成コードと自作コードを分離し、自作コードは層ごとにフォルダ分け |

## ファイル構成

```
software/main-controller/
├── README.md                     決定事項・ファイル構成・実装ステップ（このファイル）
├── PIN_REQUIREMENTS.md           ピン割り当ての論点
│
└── TORANEKO.Mk3/                 CubeIDE プロジェクト
    ├── TORANEKO.Mk3.ioc          CubeMX 設定（ピン・クロック・周辺機能）
    ├── .project / .cproject      CubeIDE のプロジェクト設定（生成）
    ├── STM32F411CEUX_FLASH.ld    リンカスクリプト（生成）
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
    │   │   ├── syscalls.c        printf の出力先を UART に向ける
    │   │   └── sysmem.c
    │   └── Startup/
    │       └── startup_stm32f411ceux.s
    │
    ├── Drivers/                  ── HAL / CMSIS（生成・編集しない）
    │
    ├── App/                      ── 自作コードはすべてここ
    │   ├── app.c / app.h         app_init()、app_loop()。全体の入口
    │   │
    │   ├── config/               機体の定数
    │   │   ├── robot_params.h    タイヤ径、ギア比、トレッド幅、制御周期など
    │   │   └── ctrl_gains.h      PID ゲイン、フィードフォワード係数
    │   │
    │   ├── drv/                  ハードに直接触れる層（HAL を呼ぶのはここだけ）
    │   │   ├── encoder.c/.h      AS5047P ×2：角度の読み取り
    │   │   ├── imu.c/.h          LSM6DSR：角速度・加速度の読み取り
    │   │   ├── motor.c/.h        MP6551 ×2：PWM の出力（電圧を指定）
    │   │   ├── wall.c/.h         壁センサ：LED 4本を1組ずつ点灯し、AD 値を読む
    │   │   ├── battery.c/.h      バッテリ電圧の読み取り
    │   │   ├── fan.c/.h          吸引ファンの PWM
    │   │   ├── led.c/.h          UI 用 LED
    │   │   ├── buzzer.c/.h       ブザー
    │   │   └── uart.c/.h         ログ送信（DMA 送信・リングバッファ）
    │   │
    │   ├── ctrl/                 制御
    │   │   ├── odometry.c/.h     エンコーダと IMU から速度・距離・角度を求める
    │   │   ├── pid.c/.h          汎用の PID 計算
    │   │   ├── trajectory.c/.h   台形加減速などの目標速度の生成
    │   │   ├── wall_ctrl.c/.h    壁のあり・なし判定、壁に沿った姿勢の補正
    │   │   └── run.c/.h          直進・スラローム・斜めなどの動作
    │   │
    │   ├── maze/                 迷路（HAL に依存しない。PC でテストできる）
    │   │   ├── maze_map.c/.h     壁の情報の保持・更新・保存
    │   │   ├── search.c/.h       足立法・歩数マップ・行き止まり潰し
    │   │   └── path.c/.h         最短経路（ダイクストラ）と、経路の圧縮・斜めへの変換
    │   │
    │   └── sys/                  全体の管理
    │       ├── scheduler.c/.h    1kHz の周期処理（センサ → 制御 → 出力の順に呼ぶ）
    │       ├── mode.c/.h         走行モードの選択と実行
    │       ├── ui.c/.h           モード選択の操作、LED・ブザーでの通知
    │       ├── log.c/.h          走行ログの記録と吸い出し
    │       ├── failsafe.c/.h     異常の検知（衝突、電圧低下など）と停止
    │       └── flash.c/.h        迷路・パラメータの Flash への保存
    │
    └── test/                     ── PC 上で動かすテスト（CI で実行）
        ├── test_maze.c           迷路探索・最短経路のテスト
        └── Makefile
```

### 層の依存関係

```
sys  →  ctrl / maze  →  drv  →  HAL（Core/, Drivers/）
```

- 呼び出しは上の層から下の層への一方向のみ。`drv` から `ctrl` を呼ぶような逆向きの呼び出しはしない
- HAL を直接呼ぶのは `drv/` と `sys/scheduler.c`（タイマー割り込みの起動）だけ
- `maze/` は `drv` も HAL も使わない。PC 上でそのままテストできる
- `Core/` 側の生成コードには、USER CODE 欄に `App/` の関数呼び出しを1行書くだけにする（CubeMX で再生成しても自作コードが消えないように）

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
| 0 | F411 用の空プロジェクト作成（CubeIDE、クロック設定） | `.ioc`, `Core/`, `App/app.c` | `KOGUMA.MkIIl.ioc` | 設定値決定済み（[CUBEMX_SETUP.md](CUBEMX_SETUP.md)）。プロジェクト作成待ち |
| 1 | LED点灯とシリアル出力（printf） | `drv/led.c`, `drv/uart.c` | `UI.c`, `syscalls.c` | 未着手 |
| 2 | 2kHz 制御周期割り込み | `sys/scheduler.c` | `PL_timer.c` | 未着手 |
| 3 | エンコーダ読み取り（AS5047P） | `drv/encoder.c` | `PL_encoder.c` | 未着手 |
| 4 | IMU 読み取り（LSM6DSR） | `drv/imu.c` | `lsm6dsr.c` | 未着手 |
| 5 | モータ PWM | `drv/motor.c` | `motor.c`（出力部分） | 未着手 |
| 6 | 壁センサ（4組を1組ずつ点灯） | `drv/wall.c`, `ctrl/wall_ctrl.c` | `PL_sensor.c`, `Wallsensor.c` | 未着手 |
| 7 | 速度・角度制御（PID、フィードフォワード） | `ctrl/odometry.c`, `ctrl/pid.c`, `ctrl/trajectory.c`, `ctrl/run.c` | `motor.c`, `Move.c` | 未着手 |
| 8 | 迷路探索と最短経路 | `maze/*`, `test/` | `Maze.c` | 未着手（PCでテスト可） |
| 9 | 走行モード、ログ、フェイルセーフ、保存 | `sys/mode.c`, `sys/ui.c`, `sys/log.c`, `sys/failsafe.c`, `sys/flash.c` | `main.c`, `UI.c`, `LOG.c`, `Failsafe.c` | 未着手 |

### KOGUMA のファイルとの対応

| KOGUMA | ToraNeko |
|---|---|
| `PL_encoder.c` | `drv/encoder.c` |
| `lsm6dsr.c` | `drv/imu.c` |
| `PL_sensor.c` | `drv/wall.c`（LED点灯・AD読み取り）、`drv/battery.c` |
| `Wallsensor.c` | `ctrl/wall_ctrl.c` |
| `motor.c` | `drv/motor.c`（PWM出力）、`ctrl/pid.c`、`ctrl/odometry.c`、`drv/fan.c` |
| `Move.c` | `ctrl/trajectory.c`、`ctrl/run.c` |
| `Maze.c` | `maze/maze_map.c`、`maze/search.c`、`maze/path.c` |
| `PL_timer.c` | `sys/scheduler.c` |
| `main.c`（モード選択部分） | `sys/mode.c` |
| `UI.c` | `sys/ui.c`、`drv/led.c`、`drv/buzzer.c` |
| `LOG.c` | `sys/log.c` |
| `Failsafe.c` | `sys/failsafe.c` |
| `Define.h` | `config/robot_params.h`、`config/ctrl_gains.h` |
