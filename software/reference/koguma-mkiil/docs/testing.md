# テストとCI

README から移した(2026-10-12)。README にはテストの一覧だけを残した。

`Core/Src/Maze.c`はHALに依存しないので、ホストのgccにそのままリンクして迷路アルゴリズムを検証できる。

```sh
# 16×16
gcc -std=c11 -Wall -I Core/Inc -o test_deadend test/test_deadend.c Core/Src/Maze.c && ./test_deadend
# 32×32
gcc -std=c11 -Wall -DMAZE_SIZE=32 -I Core/Inc -o test_deadend32 test/test_deadend.c Core/Src/Maze.c && ./test_deadend32
```

`test/test_deadend.c`が行き止まり潰しについて検査する内容:

| # | 内容 |
|---|---|
| 1 | 完全迷路(全域木)200種。連鎖が根元まで効いているか、最短経路上のマスが1つも潰れていないか、スタートとゴール2×2区画が守られているか |
| 2 | ループあり迷路300種。閉鎖マスを通行不可としても最短距離が変わらないか |
| 3 | 壁を4割「未確認」に落とした探索途中の状態300種。部分知識でも同上 |
| 4 | 初期状態(外周のみ既知)で1マスも閉じないか(誤爆検出) |
| 5 | `G_DeadEnd_Fill_Enable`による有効/無効の切り替え |

1〜3では、素朴な「変化がなくなるまで全マス走査」で書いた**参照実装**と閉鎖集合が完全一致するかも突き合わせている。仕様そのものの正しさは最短距離不変の検査が、連鎖機構(ワークリストの積み忘れ・打ち切り)のバグは参照実装との一致が担保する役割分担。

テストは不変条件が壊れると非0で終了する。実際に`Maze_Cell_IsProtected()`の保護を外した改変版を通すと失敗を検出することを確認済み。

`test/test_walldist.c`は距離変換の表を検査する(センサ値が増えると距離が増えないこと、表の範囲が84mmを含むこと、補間の値)。

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_walldist test/test_walldist.c Core/Src/WallDistance.c && ./test_walldist
```

`test/test_speedplan.c`はターンごとの通過速度の速度計画を検査する(全ターン2000のとき従来と同じ速度になること、連続ターンが遅い方にそろうこと、MAXやスタート・ゴールの上限を超えないこと、ランダムなパス2000本で全直線の加減速が距離内に収まること)。後距離で速度を変えるモード(`Short_NANAME_MoveTurnV`)についても、全ターン2400のとき`Short_NANAME_Move2400`と同じ速度になること、連続ターンが後距離で変えきれれば自分の速度のままになること、変えきれなければ速い方が変えきれる速度まで下がること、ランダムなパス2000本で後距離の速度の変化が後距離の8割に収まることを確かめる。

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_speedplan test/test_speedplan.c Core/Src/SpeedPlan.c -lm && ./test_speedplan
```

`test/test_shortpath.c`は最短走行の命令列を検査する。BFS版とダイクストラ版で`G_Short_Pass_NANAME`を作り、`Move.c`と同じ解釈で迷路の上を半区画単位に走らせて、壁や柱を通らないか、`Move.c`が実行しない命令が残っていないか、最後にゴール2×2区画で止まるか、BFS版が最短マス数か、をランダム迷路で確認する。引数で迷路の数を変えられる(既定300)。`--dump`は実機の迷路の出力の確認(`maze_algorithm.md`の「迷路の出力」)。

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_shortpath test/test_shortpath.c Core/Src/Maze.c && ./test_shortpath
```

`test/test_disthist.c`は距離ごとのセンサ値の履歴(`DistHist.c`)を検査する(直線の信号なら300〜6000mm/sのどの速度でも5mm前の値が正確に取れること、範囲外、止まっているとき、大きく飛んだとき)。

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_disthist test/test_disthist.c Core/Src/DistHist.c -lm && ./test_disthist
```

`test/test_pillaredge.c`は柱の端の検出(`PillarEdge.c`)を検査する。区画中心で柱の真ん中を見るセンサのモデルで値を作り、ファームウェアと同じ流れ(1msごと、LPF、3回の中央値、遅れの分だけ距離を戻す)で入れて、壁の終わり(#2)と柱だけ(#3)で下がる端が1つだけ出て、速度300〜3000・信号の大きさ0.6〜1.4倍で位置のばらつきが1mm以内か、壁が続く所(#1)と上がるだけの所(#4)で出ないか、1回だけの落ち込みで出ないか、横のずれを壁までの距離で直せるか、左右の平均で向きのずれが打ち消されるかを確かめる。従来の閾値80のばらつきも並べて表示する。

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_pillaredge test/test_pillaredge.c Core/Src/PillarEdge.c Core/Src/DistHist.c -lm && ./test_pillaredge
```

CIは`.github/workflows/maze-test.yml`で、masterへのpushとPRのときに行き止まり潰しと最短走行の命令列の検査(それぞれ16×16と32×32)、距離変換の表の検査、速度計画の検査、距離ごとの履歴と柱の端の検出の検査を実行する。ファームウェア本体のARMビルドは対象外。
