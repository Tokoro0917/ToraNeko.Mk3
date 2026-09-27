# main-controller

ToraNeko.Mk3 メイン基板（STM32F411CEU6）のファームウェアです。

## 出どころ

[KOGUMA.MkIIl](https://github.com/Tokoro0917/KOGUMA.MkIIl) のコミット
`e456da38832444fc30eb78e9e4e20f32978636c6`（2026-09-25）をそのままコピーしたものです。
旧版一式の `KOGUMA.MkII/` と `.github/` は持ち込んでいません（CIはリポジトリ直下の `.github/workflows/` に移設）。

元のREADME（迷路アルゴリズム・動作モードの説明）は [README.koguma.md](README.koguma.md) にあります。

## 現状

**まだ STM32F446RE（LQFP64）向けのまま**で、ToraNeko.Mk3 の基板では動きません。
`.ioc`・プロジェクト名・リンカスクリプト・起動コードも KOGUMA.MkIIl のままです。

移植の流れ:

1. ToraNeko.Mk3 の回路図・ピン割り当てを確定する（[PIN_REQUIREMENTS.md](PIN_REQUIREMENTS.md) を参考に）
2. 回路図とソフト側の要求を照合し、食い違いを洗い出す
3. STM32F411CEU6 向けに書き換える（`.ioc`、起動コード、リンカスクリプト、クロック 100MHz 化、タイマー番号、ピン定義）

## ホスト上のテスト

`Core/Src/Maze.c` はHALに依存しないので、PC上でテストできます。

```bash
gcc -std=c11 -Wall -I Core/Inc -o test_deadend test/test_deadend.c Core/Src/Maze.c && ./test_deadend
gcc -std=c11 -Wall -DMAZE_SIZE=32 -I Core/Inc -o test_deadend32 test/test_deadend.c Core/Src/Maze.c && ./test_deadend32
```
