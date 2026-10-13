# CubeMX 設定（ステップ0）

STM32CubeMX で `TORANEKO.Mk3` プロジェクトを新規作成するときの `.ioc` 設定値です。コードは CMake 形式で生成し、VS Code（STM32Cube for Visual Studio Code 拡張）でビルド・書き込み・デバッグします。
ピン割り当ては [PIN_REQUIREMENTS.md](PIN_REQUIREMENTS.md) の確定表に従います。

「KOGUMA」列は参考用に KOGUMA.MkIIl（F446, 180MHz）の設定を載せています。

## 1. プロジェクト作成

| 項目 | 設定 |
|---|---|
| MCU | STM32F411CEUx（UFQFPN48） |
| Project Name | `TORANEKO.Mk3` |
| 作成場所 | `software/main-controller/TORANEKO.Mk3/` |
| Project Manager → Project → **Toolchain / IDE** | **CMake**（EWARM・STM32CubeIDE は使わない） |
| Project Manager → Project → Do not generate the main() | OFF |
| Project Manager → Code Generator → STM32Cube MCU packages and embedded software packs | **Copy only the necessary library files**（`Drivers/` をプロジェクト内に置き、コミットする） |
| Project Manager → Code Generator → Generate peripheral initialization as a pair of '.c/.h' files per peripheral | **ON**（`Core/Src/tim.c` などに分かれる。README のファイル構成どおり） |
| Project Manager → Code Generator → Keep User Code when re-generating | ON |
| Project Manager → Code Generator → Delete previously generated files when not re-generated | ON |

## 2. System Core

### SYS

| 項目 | 設定 | 備考 |
|---|---|---|
| Debug | **Serial Wire** | これを設定しないと PB3/PB4/PA15（JTAG 兼用ピン）が IMU の SPI・CS に使えない |
| Timebase Source | SysTick | |

### RCC

| 項目 | 設定 |
|---|---|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator**（CSTNE10M0G52Z000R0、負荷容量内蔵） |
| Low Speed Clock (LSE) | Disable |

### GPIO（出力・入力）

| ピン | モード | 初期出力 | プル | 速度 | User Label |
|---|---|---|---|---|---|
| PA11 | GPIO_Output | Low | なし | Low | `LED1` |
| PA10 | GPIO_Output | Low | なし | Low | `LED2` |
| PB12 | GPIO_Output | Low | なし | Low | `LED3` |
| PB10 | GPIO_Output | Low | なし | Low | `LED4` |
| PB2 | GPIO_Output | Low | なし | Low | `LED5` |
| PB1 | GPIO_Output | Low | なし | Low | `LED6` |
| PA0 | GPIO_Output | Low | なし | Low | `LED7` |
| PB0 | GPIO_Output | Low | なし | Low | `WALL_LED_A` |
| PA7 | GPIO_Output | Low | なし | Low | `WALL_LED_B` |
| PA15 | GPIO_Output | **High** | なし | High | `IMU_CS` |
| PA8 | GPIO_Output | **High** | なし | High | `ENC_L_CS` |
| PC15 | GPIO_Output | **High** | なし | **Low**（PC13〜15 は 2MHz 以下の制約あり） | `ENC_R_CS` |
| PA12 | GPIO_Input | - | なし（外付け 10k プルダウンあり） | - | `SW_USER` |
| PC13, PC14 | 未設定（Analog のまま） | - | - | - | - |

- CS ピンは初期値 High（非選択）にする。Low で起動すると、電源投入直後に IMU・エンコーダが選択状態になる
- 壁センサ LED（`WALL_LED_A` / `WALL_LED_B`）は初期値 Low（消灯）

## 3. クロック構成（Clock Configuration）

| 項目 | 値 | KOGUMA |
|---|---|---|
| Input frequency (HSE) | **10 MHz** | 10 MHz |
| PLL Source | HSE | HSE |
| PLLM | **/5**（VCO 入力 2MHz） | - |
| PLLN | **×200**（VCO 400MHz） | - |
| PLLP | **/4** | - |
| PLLQ | /9 など（USB 不使用。48MHz 以下になれば何でもよい） | - |
| System Clock Mux | PLLCLK | PLLCLK |
| **SYSCLK / HCLK** | **100 MHz** | 180 MHz |
| AHB Prescaler | /1 | /1 |
| APB1 Prescaler | **/2 → PCLK1 = 50 MHz**（APB1 タイマー = 100 MHz） | 45 MHz（タイマー 90 MHz） |
| APB2 Prescaler | **/1 → PCLK2 = 100 MHz**（APB2 タイマー = 100 MHz） | 90 MHz（タイマー 180 MHz） |
| Power Regulator Voltage Scale | Scale 1（100MHz 動作に必要） | |
| Flash Latency | 3 WS（CubeMX が自動設定） | |

**全タイマーのクロックが 100MHz にそろう**ので、分周値の計算が簡単になります。

## 4. タイマー

| 用途 | タイマー | モード | PSC | ARR（Period） | 周波数 | 備考 |
|---|---|---|---|---|---|---|
| モータ PWM | **TIM4** CH1〜CH4 | PWM Generation CH1〜CH4 | 10-1 | **100-1** | **100 kHz** | 分解能は 100（KOGUMA は 200）。`motor.c` の `200 - Motor_Voltage` は ARR に合わせて `100 - …` に読み替える（ステップ5で `drv/motor.c` は ARR を定数で持つ）。MP6551 の入力 PWM 周波数の上限はステップ5で確認 |
| 吸引ファン PWM | **TIM2** CH2 | PWM Generation CH2 | 20-1 | 100-1 | **50 kHz** | KOGUMA と同じ周波数・分解能 |
| 制御周期 | **TIM11** | Activated（内部クロック、ピン不使用） | 100-1 | **500-1** | **2 kHz** | NVIC: `TIM1_TRG_COM_TIM11_IRQn` を有効化。4kHz にする場合は ARR を 250-1 に |
| 壁センサの順番制御 | **TIM3** | Activated（内部クロック、ピン不使用）、**One Pulse Mode** | 100-1 | 35-1 | 1回 35µs | Trigger Event Selection（TRGO）= **Update Event**。LED 点灯と同時に起動し、35µs 後に ADC を起動する。割り込みは不要 |
| 処理時間計測・µs 待ち | **TIM5** | Activated（内部クロック） | 100-1 | 0xFFFFFFFF | 1 MHz でカウント | 32bit フリーランカウンタ |

KOGUMA の設定（参考）: TIM3 PSC 9-1 / ARR 200-1（50kHz）、TIM2 PSC 18-1 / ARR 100-1（50kHz）、TIM6 PSC 90-1 / ARR 1000-1（1kHz）。

## 5. ADC1

| 項目 | 設定 | KOGUMA |
|---|---|---|
| 入力 | IN2, IN3, IN4, IN5, IN6 | IN9〜IN13 |
| Clock Prescaler | **PCLK2 / 4 = 25 MHz**（上限 36MHz） | PCLK2 / 8 |
| Resolution | 12 bit | 12 bit |
| Scan Conversion Mode | Enable | Enable |
| Continuous Conversion Mode | Disable（1回ずつソフトで開始） | Enable |
| DMA Continuous Requests | Disable | Enable |
| External Trigger Conversion Source | **Timer 3 Trigger Out event**（立ち上がり） | Software |
| Number Of Conversion | 5 | 5 |
| DMA | ADC1 → **DMA2 Stream0**、Peripheral to Memory、Half Word、Normal | DMA2 Stream0 |

変換順（Rank）は KOGUMA と同じ並びにして、DMA バッファの添字を流用できるようにします。

| Rank | チャンネル | ピン | 信号 | バッファ | Sampling Time |
|---|---|---|---|---|---|
| 1 | IN2 | PA2 | BATT | `[0]` | 15 cycles |
| 2 | IN3 | PA3 | Wallsen-4 | `[1]` | 15 cycles |
| 3 | IN4 | PA4 | Wallsen-3 | `[2]` | 15 cycles |
| 4 | IN5 | PA5 | Wallsen-2 | `[3]` | 15 cycles |
| 5 | IN6 | PA6 | Wallsen-1 | `[4]` | 15 cycles |

- 5ch の変換時間は (15+12) × 5 / 25MHz ≒ 5.4µs
- 壁センサは「組A 点灯と同時に TIM3 を起動 → 35µs 後に TIM3 が ADC を起動 → 5ch スキャン → DMA 完了割り込みで組A 消灯・組B 点灯して TIM3 を再起動 → …」を2段連鎖させる（受光素子 LTR-209 の立ち上がり 10µs・立ち下がり 15µs のため 35µs 待つ）。消灯時の値（外乱光）も連鎖の中で1回測る。連鎖全体で約 100µs。詳細はステップ6で詰める
- バッテリ電圧の換算: `V = AD × 3.3 / 4095 × (100 + 47) / 47`（KOGUMA は `× (47 + 10) / 10`）

## 6. SPI

| 項目 | SPI1（IMU: LSM6DSRX） | SPI2（エンコーダ: AS5047P ×2） |
|---|---|---|
| ピン | PB3 SCK / PB4 MISO / PB5 MOSI（**CubeMX の初期ピン PA5〜7 から付け替える**） | PB13 SCK / PB14 MISO / PB15 MOSI |
| Mode | Full-Duplex Master | Full-Duplex Master |
| Hardware NSS | Disable（CS は GPIO で制御） | Disable |
| Data Size | 8 bit | 8 bit（KOGUMA と同じく 2 バイトで 16bit フレームを送る） |
| First Bit | MSB | MSB |
| CPOL / CPHA | High / 2 Edge（モード3、KOGUMA と同じ） | High / 2 Edge（KOGUMA で動作実績あり。データシート上のモードはステップ3で確認） |
| Prescaler | **/16 → 6.25 MHz**（APB2 100MHz。LSM6DSRX 上限 10MHz） | **/8 → 6.25 MHz**（APB1 50MHz。AS5047P 上限 10MHz） |
| KOGUMA | /8（11.25MHz） | SPI3 /2（22.5MHz） |

KOGUMA は両方ともデータシートの上限を超えるクロックで動かしていたので、ToraNeko では上限内に収めます。速度が足りなければステップ3・4で見直します。

## 7. USART1（ログ出力）

| 項目 | 設定 | KOGUMA |
|---|---|---|
| Mode | **Single Wire (Half-Duplex)**（Asynchronous は選べない。下の注記） | USART2 Single Wire (Half-Duplex) |
| Baud Rate | 115200 bps（まずはこれで動作確認。ログ吸い出し用に後で 1〜2Mbps へ上げる） | 115200 bps |
| Word Length / Parity / Stop | 8 bit / None / 1 | 同じ |
| Data Direction | **Transmit Only** | - |
| TX ピン | PA9 | PA2 |
| DMA | USART1_TX → DMA2 Stream7（Memory to Peripheral、Byte、Normal） | なし |

- USART1_RX に使えるピン（PA10 / PB7 / PB3）は LED2・TIM4（モータ）・SPI1（IMU）で全て埋まっているため、CubeMX では Asynchronous が選べず Single Wire しか選べない
- Single Wire だと TX ピンがオープンドレインで生成され、STLINK-V3MODS 側にプルアップがないと信号が出ない可能性がある。そこで `Core/Src/usart.c` の `USER CODE BEGIN USART1_MspInit 1` で **PA9 をプッシュプル（`GPIO_MODE_AF_PP`）に設定し直す**（USER CODE 欄なので再生成しても消えない）
- 受信はしない（Transmit Only）ので、半二重モード（HDSEL）のままでも送信の動作は通常の非同期と同じ

## 8. NVIC（割り込み優先度の案）

| 割り込み | 優先度 | 用途 |
|---|---|---|
| DMA2 Stream0（ADC1） | 1 | 壁センサの連鎖（次の LED へ進める）・バッテリの変換完了 |
| TIM1_TRG_COM_TIM11 | 2 | 2kHz 制御周期 |
| DMA2 Stream7（USART1_TX） | 5 | ログ送信完了 |
| USART1 global interrupt | 5 | HAL の DMA 送信は、DMA 完了後に USART の送信完了割り込みで `HAL_UART_TxCpltCallback` を呼ぶので有効にする |
| SysTick | CubeMX の既定値 | `HAL_Delay` 用。割り込み処理の中では `HAL_Delay` を使わない |

壁センサの連鎖を制御周期より高い優先度にし、制御処理が長引いてもセンサ測定が遅れないようにします。

## 9. 生成後にやること

1. `Core/Src/main.c` の USER CODE 欄に `app_init();` と `app_loop();` の呼び出しだけを書く（README のファイル構成どおり）
2. `App/` をビルド対象に加える（下の「CMake のビルド設定」）
3. `cmake --preset Debug && cmake --build --preset Debug` でビルドが通ること、STLINK-V3MODS で書き込めることを確認する
4. `software/main-controller/TORANEKO.Mk3/` を `Drivers/` も含めてコミットする（`build/` は `.gitignore` で除外）

## 10. CMake のビルド設定

CubeMX が生成するファイルのうち、`cmake/stm32cubemx/CMakeLists.txt`（`Core/`・`Drivers/` のソース一覧）は再生成のたびに上書きされます。
プロジェクト直下の `CMakeLists.txt` は初回だけ生成され、以後は上書きされないので、`App/` の追加はこちらに書きます。

```cmake
# User sources (App/). Add one file per line as each step is implemented
set(APP_SOURCES
    App/app.c
    # App/drv/*.c などはステップごとに追記する
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${APP_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App
)

# Warnings for user code only (App/); generated HAL code is left as is
set_source_files_properties(${APP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
```

- ソースは `file(GLOB ...)` を使わず 1 ファイルずつ書く（ファイルを足したのに CMake を再実行し忘れてビルドされない、を防ぐ）
- インクルードは `App/` を起点に書く（例: `#include "drv/encoder.h"`、`#include "config/robot_params.h"`）。各サブフォルダを個別にインクルードパスに足さない
- 警告（`-Wall -Wextra`）は `APP_SOURCES` のファイルだけに付ける。生成コード（HAL）の警告は直せないので対象外にする
- CI（`.github/workflows/firmware-build.yml`）で同じビルドを実行する
- PC 用のテスト（`software/main-controller/test/`）はこのプロジェクトの外に置き、ホストの gcc でビルドする

## 11. VS Code の設定

| 項目 | 内容 |
|---|---|
| 拡張機能 | **STM32Cube for Visual Studio Code**（ST 公式。CMake Tools・Cortex-Debug 相当の機能を含む） |
| 必要なツール | arm-none-eabi-gcc、CMake、Ninja、STM32CubeProgrammer（拡張機能の案内に従ってインストール） |
| プロジェクトを開く | `software/main-controller/TORANEKO.Mk3/` を VS Code で開き、CMake のプリセット `Debug` を選ぶ |
| 書き込み・デバッグ | 拡張機能が `.vscode/launch.json` を作る（ST-LINK GDB server 経由）。`.vscode/` は個人設定なのでコミットしない |
