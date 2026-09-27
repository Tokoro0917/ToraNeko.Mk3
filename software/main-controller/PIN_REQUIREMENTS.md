# ソフト側が期待する周辺機能とピン（KOGUMA.MkIIl → ToraNeko.Mk3）

KOGUMA.MkIIl のファームウェアが使っている周辺機能と、STM32F411CEU6（UFQFPN48）へ移すときの注意点をまとめたものです。
回路図を起こす際の参考、および回路図完成後の照合に使います。

**周辺機能の構成（どのSPI・どのタイマーを何に使うか）を KOGUMA に合わせるほど、ソフトの修正が少なくて済みます。**
ピン自体は変わっても、`main.h` のラベル定義と `.ioc` を直すだけで吸収できます。

## KOGUMA.MkIIl（STM32F446RE）での割り当て

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

## STM32F411CEU6 へ移すときの論点

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

- [ ] 上表の各機能に、実際に割り当てたピン・周辺機能を記入
- [ ] ADC 5ch が全て ADC 入力可能ピンか
- [ ] タイマーのチャンネルと AF 番号がデータシートと一致しているか
- [ ] SPI の CS ピン（IMU、エンコーダ ×2）
- [ ] MP6551 の入力方式（IN1/IN2 PWM）とソフトの PWM 出し方（`motor.c` の `200 - Motor_Voltage`）が合っているか
- [ ] HSE の周波数
