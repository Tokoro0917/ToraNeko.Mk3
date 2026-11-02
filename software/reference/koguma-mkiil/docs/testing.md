# テストとCI

README から移した(2026-10-12)。README にはテストの一覧だけを残した。

`Core/Src/Maze.c`はHALに依存しないので、ホストのgccにそのままリンクして迷路アルゴリズムを検証できる。ダイクストラのコストに最短走行のターンの値を使うので、`Core/Src/ShortParam.c`(ターンの表 `G_Turn2000` / `G_Turn2400` と `G_Short_Run_Set`)と `-lm` も一緒にリンクする(2026-10-27)。

```sh
# 16×16
gcc -std=c11 -Wall -I Core/Inc -o test_deadend test/test_deadend.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && ./test_deadend
# 32×32
gcc -std=c11 -Wall -DMAZE_SIZE=32 -I Core/Inc -o test_deadend32 test/test_deadend.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && ./test_deadend32
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

`test/test_shortpath.c`は最短走行の命令列を検査する。BFS版とダイクストラ版で`G_Short_Pass_NANAME`を作り、`Move.c`と同じ解釈で迷路の上を半区画単位に走らせて、壁や柱を通らないか、`Move.c`が実行しない命令が残っていないか、最後にゴール2×2区画で止まるか、BFS版が最短マス数か、ダイクストラ版の経路のコストが本当の最短と同じか(状態を「壁の中点 + 来た向き + 同じ向きが続いた数」にして変化がなくなるまで見直す素朴な参照実装と比べる。2026-10-14 追加。2026-10-27 に変更。以前のコストのときはこの参照実装と比べ、最短走行の時間のコスト(既定)のときは、ダイクストラが返したコスト `G_Dijk_Best_Cost` が命令列の時間(`tools/route_time.py` と同じ数え方を、`Maze.c` が作った表 `G_Dijk_Cost` で数える)と同じか、BFS と以前のコストの経路より遅くないかを確かめる。`G_Short_Run_Set` の 2000 と 2400 の両方で)、をランダム迷路で確認する。引数で迷路の数を変えられる(既定300)。`--dump`は実機の迷路の出力の確認(`maze_algorithm.md`の「迷路の出力」)。

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_shortpath test/test_shortpath.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && ./test_shortpath
```

`--brute 迷路の数 ループの数` は、小さい迷路(`-DMAZE_SIZE=6` などでビルド)で、スタートからゴールまでの同じ区画を2回通らない経路を全部作り、同じ手順で命令列にして時間を数え、一番速いものとダイクストラのコストが同じかを確かめる(2026-10-27)。6×6・ループ40で約13万本、8×8・ループ30で約6万本の経路と比べる。ダイクストラのターンの一つ(斜め入り135)を外す、斜めの直線の表を1つずらす、斜め出45のコストを1減らす、のどれでも失敗することを確かめてある。

```sh
gcc -std=c11 -O2 -Wall -DMAZE_SIZE=6 -I Core/Inc -o test_shortpath6 test/test_shortpath.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && ./test_shortpath6 --brute 300 40
```

`test/test_unknown_search.c` はダイクストラ誘導の未知壁探索(モード0 No.6・7・10〜12)を検査する(2026-10-27)。シミュレータ(`sim/sim_main.c` のシナリオB。`main.c` と同じ手順)でランダム迷路を探索させ、探索が終わるか、探索で見た壁だけ(見ていない壁は壁)で求めた最短が、`G_Short_Run_Set` の全部の組で本当の迷路の最短と同じ速さかを確かめる。ランダム迷路では起きにくいことがあるので、大会の迷路でも確かめた: `--mazes 一覧` で、`tools/mazes_to_lines.py` で作った迷路の一覧(1行に1迷路)を読む。micromouseonline/mazefiles の classic(16×16、510個)では、見に行く途中で目標の区画に行けなくなったときに計算し直す直しを外すと、探索が終わらなくなる。

```sh
gcc -std=c11 -Wall -DSIM_DEBUG -I Core/Inc -I . -I sim -o test_unknown_search test/test_unknown_search.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && ./test_unknown_search
git clone https://github.com/micromouseonline/mazefiles /tmp/mazefiles
python3 tools/mazes_to_lines.py /tmp/mazefiles/classic/*.txt > /tmp/mazes.txt
./test_unknown_search --mazes /tmp/mazes.txt
```

`test/test_maze_undo.c` は、フェイルセーフで止まったときに最後に壁を書いた5区画を戻す処理(`Maze.c` の `Maze_Undo_Recent_Cells()`)を検査する(2026-10-27)。同じシミュレータでランダム迷路をいろいろな歩数まで探索させて止め、戻したあとの壁・見たかどうか・`G_MAZE_Explored` が、最後の5回の `Maze_Wall_Update()` の前と同じか、2回呼んでも2回目は何も戻さないか、`Maze_Undo_Clear()`(探索の始め)より前の壁は戻さないかを確かめる。

```
gcc -std=c11 -Wall -DSIM_DEBUG -I Core/Inc -I . -I sim -o test_maze_undo test/test_maze_undo.c Core/Src/Maze.c Core/Src/ShortParam.c -lm && ./test_maze_undo
```

`mazes/` には大会の迷路を、実機の迷路の出力(モード3 No.0)と同じ形で置いてある。`--dump` で読むと、PCで作った BFS とダイクストラ(2000 の表)の経路が壁に当たらないか、ファイルに書いた経路(`BFS_NANAME` / `DIJK_NANAME`)と同じかを確かめる。参考に、2400 の表と以前のコストのダイクストラの経路が同じかも出す(失敗にはしない)。経路の決め方(ダイクストラの重みなど)を変えて経路が変わると失敗するので、変えたときは新しい経路が速いかを確かめてからファイルの経路を書き換える。

| ファイル | 迷路 | 備考 |
|---|---|---|
| `mazes/2025_alljapan_classic.txt` | 2025 第46回全日本大会 クラシック | 2026-10-25 の九州地区大会でも使用。`DIJK_NANAME` は実際に走った経路(南回り、最短 2.982 秒) |

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_shortpath test/test_shortpath.c Core/Src/Maze.c Core/Src/ShortParam.c -lm
for f in mazes/*.txt; do ./test_shortpath --dump "$f"; done
```

`test/test_disthist.c`は距離ごとのセンサ値の履歴(`DistHist.c`)を検査する(直線の信号なら300〜6000mm/sのどの速度でも5mm前の値が正確に取れること、範囲外、止まっているとき、大きく飛んだとき)。

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_disthist test/test_disthist.c Core/Src/DistHist.c -lm && ./test_disthist
```

`test/test_pillaredge.c`は柱の端の検出(`PillarEdge.c`)を検査する。区画中心で柱の真ん中を見るセンサのモデルで値を作り、ファームウェアと同じ流れ(1msごと、LPF、3回の中央値、遅れの分だけ距離を戻す)で入れて、壁の終わり(#2)と柱だけ(#3)で下がる端が1つだけ出て、速度300〜3000・信号の大きさ0.6〜1.4倍で位置のばらつきが1mm以内か、壁が続く所(#1)と上がるだけの所(#4)で出ないか、1回だけの落ち込みで出ないか、横のずれを壁までの距離で直せるか、左右の平均で向きのずれが打ち消されるかを確かめる。従来の閾値80のばらつきも並べて表示する。

```sh
gcc -std=c11 -Wall -I Core/Inc -o test_pillaredge test/test_pillaredge.c Core/Src/PillarEdge.c Core/Src/DistHist.c -lm && ./test_pillaredge
```

CIは`.github/workflows/maze-test.yml`で、masterへのpushとPRのときに行き止まり潰しと最短走行の命令列の検査(それぞれ16×16と32×32)、小さい迷路で全部の経路と比べる検査(6×6・8×8)、未知壁探索の検査、距離変換の表の検査、速度計画の検査、距離ごとの履歴と柱の端の検出の検査を実行する。ファームウェア本体のARMビルドは対象外。
