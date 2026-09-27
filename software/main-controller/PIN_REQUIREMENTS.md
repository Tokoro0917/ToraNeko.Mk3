# ピン割り当て（ToraNeko.Mk3 確定版）と KOGUMA.MkIIl との対応

ToraNeko.Mk3 メイン基板（STM32F411CEU6, UFQFPN48）の確定したピン割り当てと、KOGUMA.MkIIl（STM32F446RE）との違いをまとめたものです。
CubeMX の具体的な設定値は [CUBEMX_SETUP.md](CUBEMX_SETUP.md) を参照。

## ToraNeko.Mk3 の確定ピン割り当て

回路図 `hardware/kicad/TORANEKO.Mk3/TORANEKO.Mk3.kicad_sch`（コミット `d0b5a54`）から抽出。

| ピン番号 | ピン | 回路図のネット名 | 用途 | 周辺機能（AF） | CubeMX ラベル案 |
|---|---|---|---|---|---|
| 2 | PC13 | - | 未使用 | - | - |
| 3 | PC14 | - | 未使用 | - | - |
| 4 | PC15 | ENCO-R-NSS | 右エンコーダ CS | GPIO 出力（低速のみ可） | `ENC_R_CS` |
| 5 / 6 | PH0 / PH1 | - | 10MHz セラミック発振子（CSTNE10M0G52Z000R0） | RCC_OSC_IN / OUT | - |
| 7 | NRST | NRST | リセット（J6） | - | - |
| 10 | PA0 | LED7 | 表示LED7 | GPIO 出力 | `LED7` |
| 11 | PA1 | FunPWM | 吸引ファン PWM（Q5 ゲート） | TIM2_CH2（AF1） | - |
| 12 | PA2 | BATT | バッテリ電圧（100k/47k 分圧、C24 0.1µF） | ADC1_IN2 | - |
| 13 | PA3 | Wallsen-4 | 壁センサ受光4 | ADC1_IN3 | - |
| 14 | PA4 | Wallsen-3 | 壁センサ受光3 | ADC1_IN4 | - |
| 15 | PA5 | Wallsen-2 | 壁センサ受光2 | ADC1_IN5 | - |
| 16 | PA6 | Wallsen-1 | 壁センサ受光1 | ADC1_IN6 | - |
| 17 | PA7 | WallLED-2 | 壁センサ発光 組B: 右前向き＋左向き（IC4/D8 と IC9/D10 の VEN を同時駆動） | GPIO 出力 | `WALL_LED_B` |
| 18 | PB0 | WallLED-1 | 壁センサ発光 組A: 左前向き＋右向き（IC3/D7 と IC8/D9 の VEN を同時駆動） | GPIO 出力 | `WALL_LED_A` |
| 19 | PB1 | LED6 | 表示LED6 | GPIO 出力 | `LED6` |
| 20 | PB2 | LED5 | 表示LED5（BOOT1 兼用。BOOT0=Low なので影響なし） | GPIO 出力 | `LED5` |
| 21 | PB10 | LED4 | 表示LED4 | GPIO 出力 | `LED4` |
| 25 | PB12 | LED3 | 表示LED3 | GPIO 出力 | `LED3` |
| 26 | PB13 | ENCO-SCK | エンコーダ SPI SCK | SPI2_SCK（AF5） | - |
| 27 | PB14 | ENCO-MISO | エンコーダ SPI MISO | SPI2_MISO（AF5） | - |
| 28 | PB15 | ENCO-MOSI | エンコーダ SPI MOSI | SPI2_MOSI（AF5） | - |
| 29 | PA8 | ENCO-L-NSS | 左エンコーダ CS | GPIO 出力 | `ENC_L_CS` |
| 30 | PA9 | UART-TX | ログ出力（STLINK-V3MODS 仮想COMポートへ） | USART1_TX（AF7） | - |
| 31 | PA10 | LED2 | 表示LED2 | GPIO 出力 | `LED2` |
| 32 | PA11 | LED1 | 表示LED1 | GPIO 出力 | `LED1` |
| 33 | PA12 | - | ユーザースイッチ SW1（押すと High、R9 10k プルダウン） | GPIO 入力 | `SW_USER` |
| 34 | PA13 | SWDIO | SWD | SYS_JTMS-SWDIO | - |
| 37 | PA14 | SWCLK | SWD | SYS_JTCK-SWCLK | - |
| 38 | PA15 | IMU-NSS | IMU CS | GPIO 出力 | `IMU_CS` |
| 39 | PB3 | IMU-SCK | IMU SPI SCK | SPI1_SCK（AF5） | - |
| 40 | PB4 | IMU-MISO | IMU SPI MISO | SPI1_MISO（AF5） | - |
| 41 | PB5 | IMU-MOSI | IMU SPI MOSI | SPI1_MOSI（AF5） | - |
| 42 | PB6 | PWML-1 | 左モータ MP6551 IN1 | TIM4_CH1（AF2） | - |
| 43 | PB7 | PWML-2 | 左モータ MP6551 IN2 | TIM4_CH2（AF2） | - |
| 44 | BOOT0 | - | R8 10k で GND | - | - |
| 45 | PB8 | PWMR-1 | 右モータ MP6551 IN1 | TIM4_CH3（AF2） | - |
| 46 | PB9 | PWMR-2 | 右モータ MP6551 IN2 | TIM4_CH4（AF2） | - |

### KOGUMA.MkIIl との違い（ソフトの修正点）

| 機能 | KOGUMA.MkIIl | ToraNeko.Mk3 | ソフトへの影響 |
|---|---|---|---|
| モータ PWM | TIM3 CH1〜4 | **TIM4 CH1〜4** | `htim3` → `htim4`。チャンネル割り当て（左 CH1/2、右 CH3/4）は同じ |
| 吸引ファン PWM | TIM2 CH1 | **TIM2 CH2** | チャンネル番号のみ変更 |
| ブザー | TIM4 CH4 | **なし** | ブザー関連コードは移植しない |
| 制御周期割り込み | TIM6 | **TIM11（案、検討中）** | F411 に TIM6 がないため |
| IMU | SPI1（PA5〜7）、CS=PA4 | **SPI1（PB3〜5）**、CS=PA15 | ハンドルは同じ `hspi1`。CS ピンのみ変更 |
| エンコーダ | SPI3、CS=PC5/PA15 | **SPI2**、CS=PA8/PC15 | `hspi3` → `hspi2` |
| 壁センサ受光 | ADC1 IN10〜13 | **ADC1 IN3〜6** | チャンネル番号を変更。DMA バッファの並び（[0]=バッテリ、[4]=センサ1）は同じにできる |
| バッテリ電圧 | ADC1 IN9、分圧 10k/47k | **ADC1 IN2、分圧 47k/100k** | 換算式を `× (100+47)/47` に変更 |
| 壁センサ発光 | GPIO 2本（2組ずつ同時点灯） | **GPIO 2本（2組ずつ同時点灯）**。ただし LED 1個ごとに NCR321PAS 1個 | 点灯の順番は KOGUMA と同じ2段。ピンと組み合わせだけ変更 |
| 表示LED | 12個 | **7個** | UI 表示を作り直す |
| シリアル | USART2 半二重（PA2）115200bps | **USART1 TX のみ（PA9）** | `huart2` → `huart1` |
| スイッチ | - | **PA12 ×1** | モード選択の操作を1ボタン＋エンコーダ等で設計（ステップ9） |

## 参考: KOGUMA.MkIIl（STM32F446RE）での割り当て

| 機能 | 周辺機能 | ピン | 使用箇所 | 備考 |
|---|---|---|---|---|
| 左モータ PWM | TIM3 CH1 / CH2 | PC6 / PC7 | `motor.c` | 1モータ2本（IN1/IN2 方式）。ARR=200 |
| 右モータ PWM | TIM3 CH3 / CH4 | PC8 / PC9 | `motor.c` | 同上 |
| 吸引ファン PWM | TIM2 CH1 | PB8 | `motor.c`, `main.c` | ARR=100 |
| ブザー PWM | TIM4 CH4 | PB9 | `UI.c`, `main.c` | |
| 制御周期割り込み（1kHz） | TIM6 | - | `PL_timer.c` | |
| IMU（LSM6DSR） | SPI1 | PA5 SCK / PA6 MISO / PA7 MOSI | `lsm6dsr.c` | CS は `ICM_NSS`（PA4）。名前は旧IMU（ICM）の名残 |
| エンコーダ（AS5047P ×2） | SPI3 | PC10 SCK / PC11 MISO / PB0 MOSI | `PL_encoder.c` | CS は `ENCO_L_NSS`（PC5）/ `ENCO_R_NSS`（PA15） |
| バッテリ電圧 | ADC1 IN9 | PB1 | `PL_sensor.c` | DMAバッファ `g_ADCBuffer[0]` |
| 壁センサ受光 ×4 | ADC1 IN10〜IN13 | PC0〜PC3 | `PL_sensor.c` | `g_ADCBuffer[1..4]`。DMA（DMA2 Stream0 CH0） |
| 壁センサ発光 | GPIO ×2 | PA1 `WallLED_12` / PA0 `WallLED_34` | `PL_sensor.c` | 2組ずつ同時点灯 |
| UI LED | GPIO ×12 | PB12〜15, PB4〜7（LED1〜8）、PA3, PC13〜15（LEDF1〜4） | `UI.c` | |
| シリアル（printf） | USART2 半二重 | PA2 | `syscalls.c` | 115200bps |
| 書き込み・デバッグ | SWD | PA13 / PA14 | | |
| 外部クロック | HSE 10MHz | PH0 / PH1 | | SYSCLK 180MHz |
| 未使用のGPIO出力 | | PA10 / PA11 / PA12 | | 用途なし（初期化のみ） |

## STM32F411CEU6 へ移すときの論点（回路図作成前の検討メモ。結論は上の確定表）

| 項目 | 問題 | 対応案 |
|---|---|---|
| クロック | 最大 100MHz（F446 は 180MHz） | クロック設定と全タイマーの分周値を再計算。HSE の周波数も回路で確定する |
| モータ PWM | PC6〜PC9 は48ピン版に存在しない | TIM3 CH1〜4 を PA6/PA7/PB0/PB1 か PB4/PB5/PB0/PB1 に割り当てれば `motor.c` は無修正。ただし PA6/PA7 は SPI1 と、PB0/PB1 は ADC と競合するので注意。TIM1 CH1〜4（PA8〜PA11）に移すなら `htim3` → `htim1` の置換と MOE 設定が必要。PA11 は USB と競合 |
| 制御周期割り込み | F411 に TIM6 がない | TIM5 / TIM9 / TIM10 / TIM11 のいずれかで代用（`PL_timer.c` と割り込みハンドラを修正） |
| エンコーダ SPI | PC10/PC11 が存在しない | SPI2（PB13/PB14/PB15）か、SPI3 を PB3/PB4/PB5 に割り当て。IMU と同じ SPI1 に相乗りも可（その場合 `hspi3` → `hspi1`） |
| 壁センサ ADC | PC0〜PC3 が存在しない。F411 の ADC 入力は PA0〜PA7, PB0, PB1 の10本のみ | 受光4ch＋バッテリ電圧1ch の計5本を PA0〜PA7/PB0/PB1 から確保。SPI1（PA5〜PA7）と取り合いになるので要検討 |
| 壁センサ発光 | ToraNeko は NCR321PAS ×4 で1組ずつ点灯する計画（KOGUMA は2本で2組ずつ） | GPIO（またはPWM）4本を確保。`PL_sensor.c` の点灯シーケンスを4段に書き換える |
| UI LED | 12本は48ピンでは厳しい | 本数を減らすか、LEDドライバ/シフトレジスタ等を検討。`UI.c` を合わせて修正 |
| USB | ToraNeko では PA11/PA12 を USB に使う計画 | KOGUMA では未使用なので競合なし |
| シリアル | USART2 PA2 | そのまま使える（PA2/PA3 は ADC 入力とも兼用なので取り合いに注意） |
| 吸引ファン・ブザー | TIM2 CH1（PB8 は F411 では TIM4/TIM10）、TIM4 CH4（PB9） | F411 でも TIM4 CH4 = PB9 は使える。TIM2 CH1 は PA0/PA5/PA15 のいずれか |

## 回路図完成後の照合項目

- [x] 上表の各機能に、実際に割り当てたピン・周辺機能を記入（上の確定表）
- [x] ADC 5ch が全て ADC 入力可能ピンか（PA2〜PA6 = IN2〜IN6）
- [x] タイマーのチャンネルと AF 番号（TIM4 CH1〜4 = PB6〜PB9 AF2、TIM2 CH2 = PA1 AF1）
- [x] SPI の CS ピン（IMU=PA15、エンコーダ左=PA8、右=PC15）
- [ ] MP6551 の入力方式（IN1/IN2 PWM）とソフトの PWM 出し方（`motor.c` の `200 - Motor_Voltage`）が合っているか（ステップ5で MP6551 データシートと照合）
- [x] HSE の周波数（10MHz セラミック発振子）
