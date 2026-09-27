# main-controller

ToraNeko.Mk3 メイン基板（STM32F411CEU6）のファームウェアです。

KOGUMA.MkIIl のコード（[../reference/koguma-mkiil/](../reference/koguma-mkiil/)）を参照しつつ、機能ごとに内容を確認しながら一つずつ実装していきます。
確認・実装が済んだものだけをこのディレクトリに置きます。

- [PIN_REQUIREMENTS.md](PIN_REQUIREMENTS.md): KOGUMA が使っている周辺機能・ピンと、F411CEU6 へ移すときの論点

## 実装の進め方

各ステップで「KOGUMA の該当コードを読む → ToraNeko で変える点を決める → 実装 → 実機/PCで確認」を行います。

| # | 内容 | KOGUMA側の対応ファイル | 状態 |
|---|---|---|---|
| 0 | F411 用の空プロジェクト作成（CubeIDE、クロック設定） | `.ioc` | 未着手（ピン確定待ち） |
| 1 | LED点灯とシリアル出力（printf） | `UI.c`, `syscalls.c` | 未着手 |
| 2 | 1kHz 制御周期割り込み | `PL_timer.c` | 未着手 |
| 3 | エンコーダ読み取り（AS5047P） | `PL_encoder.c` | 未着手 |
| 4 | IMU 読み取り（LSM6DSR） | `lsm6dsr.c` | 未着手 |
| 5 | モータ PWM | `motor.c`（出力部分） | 未着手 |
| 6 | 壁センサ（4組を1組ずつ点灯） | `PL_sensor.c`, `Wallsensor.c` | 未着手 |
| 7 | 速度・角度制御（PID、フィードフォワード） | `motor.c`, `Move.c` | 未着手 |
| 8 | 迷路探索と最短経路 | `Maze.c` | 未着手（PCでテスト可） |
| 9 | 走行モード、ログ、フェイルセーフ | `main.c`, `LOG.c`, `Failsafe.c` | 未着手 |
