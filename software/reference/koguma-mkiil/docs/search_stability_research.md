# 読み違いを止まって払い、探索を完走させる

KOGUMA MkIIl の探索を安定させるいちばんの近道は、速度を落とすことではない。**確信のない値を地図にも位置にも書き込まないこと**と、**おかしいと気づいたら壁に当たる前に止まること**である。探索速度(スラローム 500 mm/s、吸引 1000 mm/s)は、2019年全日本クラシック優勝の華金(探索 1.0 m/s)や、吸引のクラシック機 ZoroBot3(0.8 m/s)と同じ帯にあり、速さそのものは問題の中心ではない。コードを読むと、KOGUMA には大きな穴が4つある。1つ目は、前壁を左右センサの AND(両方 60 超)で地図に書く一方、同じ瞬間のターン判断は左右の平均で行っていること。2つ目は、一度書いた壁を二度と見直さないこと。3つ目は、ジャイロのオフセットを引く処理がコメントアウトされ、±0.5 dps の不感帯でごまかしていること。4つ目は、フェイルセーフが1サンプルで判定し、角度のずれもターンの時間切れも見ず、止まっても迷路を守らないことである。先人の対策の中心は新しい推定器ではなく、「ガードと記録」である。壁は「あり・なし・不確か」で扱い、不確かなら次の区画で読み直す(UKMARS、KERISE)。補正には妥当性の窓・不感帯・上限を付ける(発散値: 壁切れは ±20 mm、前壁補正は不感帯 10 mm・ゲイン 0.5・上限 15 mm)。曲がる側に壁が見えたら止まり、行き過ぎたら超信地旋回に逃げる(KERISE)。異常が 50〜100 ms 続いたら止め、直近の壁を消す(加藤、KERISE)。大会(九州地区大会、2026-10-25)まで6日なので、順番はこうなる。まず挙動を変えない記録を入れる。次にジャイロのオフセット、前壁の判定、フェイルセーフ、電池電圧の補正を入れる。推定器の作り直しとターンの再調整は大会のあとに回す。なお、KOGUMA の失敗ログはまだ解析していない(手元の最新ログには SCUT の記録がない)。本稿で KOGUMA 向けに示す数値の多くは、先人の数値から導いた出発点であり、実機のログで確かめることが前提である。

## 3つの症状は、どれも「1回の値で決めて、あとで見直さない」に行き着く

壁に当たる、位置がずれる、壁を読み違える、の3つの症状は互いにつながっている。読み違えた壁は、地図の上にありもしない袋小路や通れない通路を作り、最短走行を壁に突っ込ませる。位置のずれは壁を読む瞬間の姿勢を乱し、次の読み違いを生む。そしてフェイルセーフがなければ、1回ぶつかったあとも探索を続けて迷路情報を壊す。加藤雄資(全日本クラシック優勝 2010・2012)のトラブル事例集は、まさに「ぶつかって位置情報がずれた後も探索し続けて迷路情報が壊れる」を典型例に挙げている([加藤 2013](https://www.ntf.or.jp/mouse/mm2013lecture.pdf))。rofumi は2019年の全日本で6分以上探索したが、帰り道で読んだ壁が間違っていたため、余分に探索した時間が無駄になった([rofumi](https://rofumi.net/wall-judgement-using-machine-learning/))。下の表は、KOGUMA のコードで確かめた原因と、それに対応する先人の対策を並べたものである。

| 症状 | KOGUMA で見つかった原因(コードで確認) | 先人の対策 |
|---|---|---|
| 壁を読み違える | 前壁は FL>60 **かつ** FR>60 のときだけ地図に書く([Maze.c L387](../Core/Src/Maze.c))。閾値は1本で、不確かな帯がない。壁は `Known_Flag == 0` のときに OR で足すだけで、見直さない | 前壁は和か平均で、近づいてから決める(UKMARS)。ヒステリシス(KERISE)。食い違ったら未知に戻す(MazeLib)、または止まって読み直す(発散値) |
| 位置がずれる | 前壁トリガの「早い側」に妥当性の窓がない([motor.c](../Core/Src/motor.c))。電池電圧の補正は走る前の1回だけ。吸引ありのタイヤ径を確かめていない | 前壁補正に窓・不感帯・上限(KERISE、発散値)。壁切れは U ターン後を使わず ±20 mm(発散値)。毎周期の電圧補正(UKMARS、ZoroBot3) |
| 向きがずれる | ジャイロのオフセットを引かず、±0.5 dps の不感帯で0にしている([lsm6dsr.c](../Core/Src/lsm6dsr.c))。動作ごとに角度を0に戻すので、ターンで残った誤差を捨てている | 走る直前、ファンを回してから数百サンプルの平均を引く(KERISE、ZoroBot3、発散値)。残った誤差を次の動作に持ち越す(KERISE) |
| 壁に当たる | 曲がる前に、曲がる側の壁を確かめていない。フェイルセーフは1サンプルで判定し、左右差の閾値 100000 は事実上効かない。角度とターン時間を見ていない。止めると同時に吸引も切る([Failsafe.c](../Core/Src/Failsafe.c)) | 曲がる側に壁が見えたら AEBS で止まる(KERISE)。50〜100 ms 続いたら止める(加藤 80 ms、ZoroBot3 100 ms)。ブレーキ → 止まったのを確認 → ファンを止める(pidream) |

以下では、この表の各行を、先人の具体的な数値とやり方とともに掘り下げる。

## 壁の読み取り: 横は姿勢で外れ、前は AND で書くのが危ない

### 読む位置は境目の少し手前でそろっており、KOGUMA の横センサには余裕がある

先人はほぼ全員、1区画に1回、区画の境目かその少し手前で壁を読む。UKMARS は 180 mm 区画の 170 mm 地点、つまり**境目の 10 mm 手前**で読む([UKMARS config-robot-osmium.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/config-robot-osmium.h#L356-L357))。KERISE は区画の切り替わり位置で、1 ms ごとにヒステリシス付きで更新している壁フラグを写し取る([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L745-L750)、[KERISE wall_detector.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/supporters/wall_detector.h#L196-L231))。RT の HM-StarterKit は境目で読む([HM-StarterKit search.c](https://github.com/rt-net/HM_StarterKit/blob/3666932b215fa851d479ce50d081bddaa249b2d2/Sample_software/step7_maze/search.c#L445))。発散値(2026年の東北地区大会向け)は「区画境界の少し手前」で読み、しかも読む位置を「目標位置ではなく実測位置」で決めて、追従の遅れで早く読まないようにしている([Qiita 発散値](https://qiita.com/akiaki96/items/6498b00ca979ef92b548))。頑丈さの源は多数決ではない。光の点を柱と柱の間に当てる向き、ヒステリシス付きのフラグ、実測位置での読み取りの3つである。

KOGUMA の横センサは、機体の中心から約 91 mm 先を見ている。判断位置(機体の中心が境目にあるとき)では、光の点は次の区画の壁の中ほどに落ちる。点が柱に乗るのは、前の区画の中心付近(-91 mm)か次の境目付近(+89 mm)にいるときだけなので、横壁の読み取り位置には**およそ ±70〜80 mm の余裕**がある(柱と光の点の大きさを差し引いた幾何の推定)。したがって KOGUMA の横壁の読み違いは、読む位置よりも、横のずれ・向きのずれ・遠くの壁や次の柱の映り込み・ノイズから来ると考えるのが妥当である。速度の影響も小さい。センサの周期が 1 kHz なら(TIM6 からの推定なので確認が必要)、10 サンプルの移動平均 `g_sensor_av10` は 1000 mm/s で約 10 mm 分の幅と約 5 mm の遅れを持つ。これは横の余裕に比べれば無視できる。前壁は距離で値が大きく変わるので、同じ閾値でも 1000 mm/s では約 5 mm 遠くで判定される計算になる。横の余裕が大きいことから、境目の -30・-15・0 mm の3点で読んで2点が一致したほうを採る多数決も、安全に足せる。ただし、この形を実装した先人のコードは見つからなかった。UKMARS の文書が「何回か読む」ことを勧めているだけである([UKMARS sensors.md](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/documents/sensors.md))。

### 閾値は「中心の値の半分」が目安で、足りないのは不確かな帯

UKMARS は、会場で各センサを区画の中心に置いたときの値が 100 になるように正規化する。横の閾値は 40(文書では、光の点が半分だけ壁に乗る値として 50 %)、前は左右の和で 20(別の機体では 45)である([UKMARS config-robot-osmium.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/config-robot-osmium.h#L286-L304)、[UKMARS sensors.md](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/documents/sensors.md))。KERISE は反射の値を対数で距離に直し、中心で 500 サンプル測った基準をフラッシュに保存して差し引く。横は相対距離 24.25 mm 未満で壁あり、25.75 mm 超で壁なしとし、その間は前の判定を保つ(ハーフサイズの値、[KERISE wall_detector.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/supporters/wall_detector.h#L202-L235))。HM-StarterKit は「車体を区画の端に寄せたときの、壁ありの値」を閾値にする([HM-StarterKit parameters.h](https://github.com/rt-net/HM_StarterKit/blob/3666932b215fa851d479ce50d081bddaa249b2d2/Sample_software/step7_maze/parameters.h#L24-L27)、[RT HM-StarterKit入門 Part 11](https://rt-net.jp/mobility/archives/17239))。garberas は、10 ビットの ADC で壁ありと壁なしが 50〜100 カウント以上離れるようにハードを作るべきだと言い、例として壁なし約 20、壁あり 150〜200 を挙げている([garberas](https://garberas.com/archives/1698))。

閾値で判定するやり方の弱点を数字で示したのは rofumi である。399 サンプル(中心か片側に置き、±5° 回したもの)で学習したロジスティック回帰は、新しいデータで誤りが0だった。同じデータで閾値の方法は3回誤り、**3回とも、軌道を補正するうちに壁へ斜めに近づいたとき**だった([rofumi](https://rofumi.net/wall-judgement-using-machine-learning/))。RT の鍬形は、感度(1 mm あたりの値の変化)が高いところに閾値を置き、前センサを約 5° 外へ開くと姿勢の乱れに強くなると報告している([RT 光センサの角度とセンサ感度](https://rt-net.jp/mobility/archives/17695))。KOGUMA は `{60, 90, 90, 60}`(FL、R、L、FR)の固定閾値に加えて、LED を消したときの値の差し引きと、中心の値で閾値を倍率補正する `Wall_Center_Calibration()`(500 サンプル平均)をすでに持っている。UKMARS の正規化と同じ考えは入っている([Wallsensor.c](../Core/Src/Wallsensor.c)、[PL_sensor.c](../Core/Src/PL_sensor.c))。足りないのは2つである。境目ぎりぎりの値をそのまま確定させないための**不確かな帯(ヒステリシス)**と、閾値の**余裕が実測で確かめられていること**である。

会場の光も無視できない。rofumi は2018年の東北地区大会の会場で、環境光の赤外が多くて多くのマウスが完走できなかったのを見ている。環境光の AD 値は最大約 300 あった。LED を点けてから読むまでの待ちを約 30 µs にすると、環境光のわずかな変化で値が 30〜40 カウント揺れたので、約 80 µs にした([rofumi 環境光](https://rofumi.net/ambient-light-on-infrared-sensor/))。T-MPI は2024年の東北地区大会で「ろくに壁が読めずに走れませんでした」と書き、同じ年の学生大会では「リセット時に横壁センサの基準値がずれてしまいリタイア」している([しらたきの甘露煮](https://t-mpi.hatenablog.com/entry/Micromouse-student2024))。KOGUMA では、会場の照明の下で `Wall_Center_Calibration()` を必ず走らせ、結果をリセットで失わないようにすることが、いちばん安い保険になる。吸引ファンが壁センサに与える影響を測った資料は見つからなかった。ただ、KOGUMA にはファンの PWM に合わせてセンサを読む実験がすでにある([SensorNoise.c](../Core/Src/SensorNoise.c))。余裕の試験は、ファンを探索の Duty で回した状態で行うべきである。

### 前壁は和か平均で、近づいてから決める

UKMARS の文書によれば、前壁は近づくまで反応が小さく、小さな角度の誤差でも値が大きく変わる。だから「確実に判定できるほど近づくまで前壁を判定しない」、目安は約 200 mm 以内で、判定には左右の正規化値の**和**を使う([UKMARS sensors.md](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/documents/sensors.md)、[UKMARS sensors.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/sensors.h#L215-L229))。光沢のある壁では、小さな角度のずれで前センサの値が「思った以上に落ちる」(同)。RT には、壁の端で姿勢が乱れて前センサが傾き、値が落ちて前壁を見落とし、壁に当たった例がある([RT 17695](https://rt-net.jp/mobility/archives/17695))。

KOGUMA の地図は、FL>60 **かつ** FR>60 のときだけ前壁を書く([Maze.c L387](../Core/Src/Maze.c))。境目での前壁の値は約 140 あるので、一見すると余裕は 2.3 倍ある。しかし向きのずれで片方が 60 を切れば「前壁なし」と記録され、最短走行がそこへ突っ込む。しかも同じ `Motor_Sula_before()` の中で、ターンに前壁トリガを使うかどうかは `Wall_Flont_Av() > 60` で決めている([motor.c L874](../Core/Src/motor.c))。地図は各センサの 10 サンプル平均を AND で、ターンは最新の値の左右平均で、と**同じ壁を同じ瞬間に別の信号と別の規則で判定している**([Wallsensor.c L127](../Core/Src/Wallsensor.c))。そのため片方のセンサだけが落ちると、ロボットは前壁があるものとして曲がるのに、地図には壁がない、という食い違いが起きる。地図の判定も平均(または和)にそろえ、たとえば「平均 60 超で壁あり、40 未満で壁なし、その間は書かずに未知のまま残す」とするのがよい。前壁が不確かなまま直進するのは安全である。次の判断位置で 180 mm 近づいてから、もう一度読めるからだ(UKMARS の「近づくまで判定しない」と同じ考え)。下側の 40 は先人の数値ではなく出発点としての提案である。2区画先に壁があるときの前センサの値を 500 mm/s と 1000 mm/s で記録して確かめる必要がある(この値を公開した資料は見つからなかった)。AND の判定を残すなら、誤って壁ありとするほうが害になる U ターンの前壁合わせの判断だけでよい。

## 読み違いは起きる前提で数え、取り消せるようにする

公開されているコードの、読み違いへの方針は3つに分かれる。UKMARS は最初の読みを確定させ、二度と見直さない([UKMARS maze.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/maze.h#L332-L353))。KERISE が使う MazeLib は、既知の壁と新しい読みが食い違ったら、その壁を未知に戻し、エラー音とログを出して、探索アルゴリズムにもう一度見に行かせる([MazeLib Maze.cpp](https://github.com/kerikun11/micromouse-maze-library/blob/3170f7d50be544328257ab63180f2d279793c00a/src/Maze.cpp#L130-L152)、[MazeLib RobotBase.cpp](https://github.com/kerikun11/micromouse-maze-library/blob/3170f7d50be544328257ab63180f2d279793c00a/src/RobotBase.cpp#L246-L248))。発散値は、食い違ったら止まって前の区画の中心まで下がり、同じ位置で読み直す。それでも違えば、センサを信じて地図を書き換える([Qiita 発散値](https://qiita.com/akiaki96/items/6498b00ca979ef92b548))。KERISE はさらに、探索の前に迷路が解けない状態なら、解けるようになるまで直近の壁の記録を **6件**ずつ消す。クラッシュのあとも6件消してから自己位置同定に入り、同定の最初の **12件**の壁は「復帰直後は壁の読み間違いがありそう」として本体の迷路に混ぜない([KERISE maze_robot.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/maze_robot.h#L313-L356)、[MazeLib SearchAlgorithm.cpp](https://github.com/kerikun11/micromouse-maze-library/blob/3170f7d50be544328257ab63180f2d279793c00a/src/SearchAlgorithm.cpp#L627-L635))。壁を消すには、見つけた壁を1件2バイト(x 6ビット、y 6ビット、向き3ビット、有無1ビット)で順に足していく「壁ログ」が要る。16×16 なら最大 480 バイトで足りる([KERI's Lab 壁情報のバックアップ](https://www.kerislab.jp/posts/2021-12-12-maze-saving-method/))。KOGUMA の `Maze_Wall_Update_Body()` は、`Known_Flag == 0` のときだけ壁のビットを OR で足す。消すことも、もう一度見たときに地図と比べることもしない([Maze.c L378-405](../Core/Src/Maze.c))。つまり**幻の壁は永久に残り**、隣の区画や帰り道でもう一度見た情報は捨てられている。

大会前に入れるなら、段階を踏むのが安全である。最初に入れるのは、既知の区画を通るたびに今の読みと地図を比べ、食い違えばブザーを鳴らして(x, y, 向き, 地図の値, 読んだ値, 4つのセンサ値, 角度)を記録するだけの「食い違いカウンタ」である。挙動は変わらないが、会場で読み違いが起きているかどうかがこれだけで分かる。次が「解けるかどうかの保険」で、ゴールまでの歩数マップが作れなくなったら直近の壁を消す。カウンタで必要と分かったら、MazeLib と同じく、食い違った壁とその「見た」ビットを消して歩数マップを作り直す方針へ進む。発散値の「下がって読み直す」は動きが増えるので、大会前に入れるには重い。

読み違いを衝突に変えないための、安い確認もある。KERISE は、スラロームの前の直線を走り終えた時点で曲がる側の横センサが壁を見ていたら、曲がらずに AEBS(自動緊急ブレーキ)で止まる。直進の前に、前の ToF がもう1区画より近いと示したときも止まる([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L846-L890))。Micras は、曲がるのは「ないと見た壁」の向きだけにし、区画の中心で止まれる最後の点を過ぎても前壁が未知なら、止まって決め直す。コードには「壁を遅れて見つけたら、クラッシュではなく停止で払う」と書いてある([Micras mission.hpp](https://github.com/Team-Micras/MicrasFirmware/blob/4dac6e2c7299ca664712ef5de177e5911152ad63/micras_nav/include/micras/nav/mission.hpp#L35-L50))。KOGUMA でも、「曲がる側の横センサが壁ありの閾値を超えていたら、スラロームをやめて止まる」はセンサの比較1回で作れる。

## 位置の補正は「合っていそうなときだけ、少しだけ」直す

### 前壁補正: 1回の IR の読みは σ 11〜15 mm ぶれる

前壁は前後の位置の唯一の絶対的な基準なので、誰もが使う。ただし上手な実装は、必ず関門で囲んでいる。KERISE(**ハーフサイズ、つまり 90 mm 区画**の機体なので、距離はクラシックの約半分。`docs/wall_research.md` は KERISE の数値をこの注記なしで引いているので、そのまま借りないこと)は、次の条件がそろったときだけ前後位置の推定を直す: ToF が 90±30 mm で、測ってから 1 ms 以内、向きが区画に対して 0.5° 以内。直す量は誤差の 10 %(ターン前の強制モードでは 40 %)ずつで、30 mm を超える誤差は「壁ではない」として捨てる([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L321-L362))。基準は幾何の値ではなく、実機で合わせたオフセット(`wall_fix_offset = -5`)である([KERISE model.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/config/model.h#L35-L43))。ターンの前には3つの確認をする: 境目を 5 mm より行き過ぎたか、速度が探索速度の 1.2 倍を超えたか、前壁があるのに距離が予想から 20 mm 以上外れたか。どれかに当たれば、スラロームをやめて止まり、前壁に合わせてから超信地旋回する([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L856-L893))。

いちばん参考になるのは発散値である。境目の 10 mm 手前で、前の左右のセンサをそれぞれ距離に直す(140〜260 mm の範囲だけ有効)。左右それぞれの基準(187.3 mm、195.2 mm)との差を平均して誤差とし、**不感帯 10 mm・ゲイン 0.5・上限 15 mm** で次の S90 の前距離を伸び縮みさせる([発散値 mouse_config.hpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Inc/config/mouse_config.hpp#L304-L325)、[発散値 front_correction.cpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Src/common/front_correction.cpp))。ここまで慎重なのには理由がある。探索のログで、1回の前壁の読みは **σ≈11〜15 mm** ぶれ、次の壁切れの誤差との相関は **r = −0.08(n = 36)** とほとんどなかった。作者は「1回の読みはうるさすぎる(壁の反射、向き、横のずれ)」と結論している([発散値 SEARCH.md](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/tools/SEARCH.md))。基準でも一度失敗している。別の迷路で、直進のあとの読みの中央値から基準を取ったところ、本当の誤差がほぼ0のはずの最初の行動で −10 mm と出て、補正が負の側へ押しすぎた([発散値 mouse_config.hpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Inc/config/mouse_config.hpp#L313-L318))。UKMARS は、前の和が閾値を超えたらターンを始める。横壁が1枚見えるごとに、閾値を 5〜6 カウント(基準 82 または 110 に対して)上げる。横壁の反射が前センサにも入るからである([UKMARS mouse.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/mouse.h#L106-L139))。Harrison は、よいセンサならターンを始める位置のばらつきは ±5 mm 程度で、しかもこの誤差は積み重ならないと書いている([micromouseonline 2011](https://micromouseonline.com/2011/10/12/using-the-front-sensors-for-triggering-turns/))。

KOGUMA は、前の左右を距離に直して平均し(`Front_Wall_mm()`)、前壁までが「174 − 前距離」(前距離 15 mm なら 159 mm)になったところでターンに入る。前壁があるときは、距離だけで決まる終点を 15 mm 先へ延ばしている([motor.c](../Core/Src/motor.c))。遅い側は +15 mm で頭打ちになるが、**早い側には窓がない**。ノイズや反射で一瞬でも近く読めば、オドメトリがまだ遠いと言っていても、その場でターンに入ってしまう。まず、前壁トリガで直進を打ち切ったときの `G_Motor_X − X0`(距離だけで走った場合より何 mm 早いか遅いか)、FL−FR、横壁の有無を記録する。その分布から早い側の窓を決める。KERISE の 20 mm(ハーフ)をクラシックに引き延ばすと 30〜40 mm が目安だが、これは推定であり、ログで確かめるべき値である。横壁があるときに前センサの値が上がっていないか(UKMARS が閾値を補正する理由)も、同じログで確かめられる。

### 壁切れ・柱補正: 上級者ほど探索では使わないか、きつく縛る

KERISE は探索の壁切れ補正をコンパイル時に切っている(`MOVE_ACTION_WALL_CUT_ENABLED 0`、理由は書かれていない)。UKMARS も探索の直進では壁切れを使わない([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L461-L497)、[UKMARS mouse.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/mouse.h#L214-L217))。使っている発散値の `WallEdge` には、何重もの制限がある。使うのは直線(目標角速度0、目標速度 100 mm/s 以上)だけ。端は ON 350 / OFF 250 のヒステリシスで取る。その前に 20 mm 以上壁が見えていたときだけ端と認める(柱1本の反射を除くため)。窓は、地図から予想した境目 −91 mm を中心に開く。1つの境目は1回しか直さない([発散値 wall_edge.cpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Src/common/wall_edge.cpp)、[発散値 mouse_config.hpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Inc/config/mouse_config.hpp#L208-L238))。

この形に落ち着くまでに、2度失敗している。1度目は窓の広さである。窓を 30 mm に広げたところ、24〜26 mm ずれた誤った壁切れを2回受け入れ、約 50 mm 前へずれて横壁を読み違え、**クラッシュ**した。そこで窓は、S90 の出口の誤差(約 10 mm)に余裕を足した **±20 mm** になった([発散値 mouse_config.hpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Inc/config/mouse_config.hpp#L230-L238)、[発散値 WALL_EDGE.md](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/tools/WALL_EDGE.md))。2度目は U ターンである。横センサは約 100 mm 先を見るので、U ターンする区画の手前の境目の壁切れは、折り返し点より前に来る。その境目を補正に使ったところ、中心へ止まりに行く途中で見た壁切れがその境目に対応づけられ、**−23.8 mm** の誤補正が出た。そこで U ターン後の境目は使わないことにした([発散値 search.cpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Src/app/search.cpp#L626-L629))。基準のオフセットは、試験コースではなく探索2回の中央値から取る。試験コースでは横にずれて置かれていたせいで左 −101 mm・右 −85 mm と出たが、探索では左右とも約 −91 mm だった。補正は、注入試験(境目を +10 mm ずらして教え、最初の補正が +10±1 mm になるか)に通るまで、記録だけにしておく([発散値 WALL_EDGE.md](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/tools/WALL_EDGE.md))。micromouseonline の本は、大きな前後補正が必要になること自体が「何かがひどくおかしい印」だと書いている([micromouseonline Straights](https://micromouseonline.com/micromouse-book/navigation/straights/))。

KOGUMA の探索の直進の壁切れは、既定が記録だけ(`Search_WallCut = 1`)である。直すモード2は、予想+基準 ±20 mm に入った推定だけを使い、`Search_Corr_Gain`(今は 1.0)×(dc − 基準)を ±10 mm まで直す。両側とも壁が続く区画では何もしない([docs/search_wallcut_design.md](../docs/search_wallcut_design.md))。窓と上限は発散値と同じか、より慎重であり、壁が続く区画で何もしないのも先人と同じである。足すべきガードは2つある。1つ目は、直前の行動が U ターン(prev = 2)の直進は、記録だけにして直さないこと。KOGUMA の横センサも約 91 mm 先を見るので、発散値と同じ取り違えが起こりうる。2つ目は、基準のずれを直前の行動ごとに見ること。直進後・右ターン後・左ターン後の中央値が 5 mm 以上違うなら、基準を行動ごとに持つ。あるいは、まず直進後(prev = 0)だけでモード2を試し、ゲインは 0.5 から始める。ただし、手元の最新ログ(20261018-195554)にはまだ SCUT の記録がない。ばらつきが数 mm しかなければ、直す必要はない。**大会は記録だけ(1)か使わない(0)で走るのを既定とし**、全面探索2回分の SCUT が直す価値のあるばらつきを示したときだけ2にする。設計資料のこの方針は、先人の経験とも合っている。

### ターンの前後: どの実装も「本当の境目」ではなく「合わせた基準」に直している

garberas によれば、オフセット区間は約 15 mm である。壁制御ありの直線 → スラローム → 壁制御なしの直線 → 停止、と走らせ、止まった位置から前後の距離を決める。滑り角のために、前距離は理論より短く、後距離は長くなる([garberas スラロームの調整](https://garberas.com/archives/866))。pidream は、前壁があればターンの前に距離を合わせる。ターン中の壁切れから測った距離を基準と比べ、後距離を伸び縮みさせる。これで、ある迷路の形を抜けられるようになった([pidream 2021](https://www.pidream.net/category/mm/mm-2021))。発散値は、S90 の出口の誤差(約 10 mm)が S90 の連続で積み重なるので、前壁で測った誤差を次の S90 の前距離に入れる([発散値 front_correction.hpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Inc/common/front_correction.hpp))。KERISE は、ターンのあとに予定の終点を引くだけで、残った誤差は次の動作に持ち越す([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L600-L639))。読んだコードはどれも、本当の幾何の境目ではなく、実機で合わせた基準に向かって直していた。KOGUMA の設計は、`Search_Edge_Ref`(今のターンの調整が前提にしている判断位置と、本当の境目との差)を基準にして、ばらつきだけを直す。これはこの慣行と一致する。前後の距離を合わせ直して基準を0にするのは、大会のあとの仕事である。

### U ターン: 合わせは収束を確かめ、押し当ては優しく

KERISE の前壁合わせは、16 サンプル平均した前の左右の距離の誤差に 30 を掛けて前後と回転の指令にし、180 mm/s と π rad/s で頭打ちにする。二乗和が 0.4 未満になれば成功で、最大 2000 周期(約2秒)で打ち切り、成功と失敗で別の音を鳴らす。合わせに失敗したら、90° 回って横にあった壁で合わせ、もう 90° 回る([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L277-L319)、[KERISE u_turn](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L659-L678))。UKMARS は、前の和が基準に届くまで 30 mm/s で進み、500 ms で打ち切る。基準から 50 カウント以上ずれていれば、±10 mm だけ動く([UKMARS mouse.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/mouse.h#L154-L251))。RT の HM-StarterKit は、壁ごとに、30 mm 下がって壁に当てる(1000 ms で打ち切り)→ 500 ms 待つ → 20 mm 進む、を行う。ただし、PID とセンサの基準をきちんと合わせる前にこれに頼ると、かえって完走率が下がると注意している([RT 袋小路での車体位置補正](https://rt-net.jp/mobility/archives/18603))。発散値は、止まるあいだも壁制御を入れていたら横の補正が終わらず、曲がった向きのまま止まって、袋小路ごとに **−11° と −13°** ずれて衝突した。そこで、止まるあいだは壁制御を切った([発散値 mouse_config.hpp](https://github.com/akiaki96/divergence_v3/blob/533949097d63c249580545180231fbb18cc11a24/Core/Inc/config/mouse_config.hpp#L199-L202))。

背中を壁に当てるやり方にも差がある。KERISE は制御を切り、−0.2 duty で 400〜500 ms 押す開ループである([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L693-L733))。KOGUMA の `Motor_Back()` は、速度 PID のまま −300 mm/s で 250 ms 押す([motor.c L308](../Core/Src/motor.c))。壁に押し付けたまま速度 PID を回すと積分がたまり、機体をねじるおそれがある。前壁合わせと背中当ての前後で、ジャイロの角度と FL・FR を記録するとよい。当てたあとの角度が区画に対して約1°以内に収まる回数を数えれば、ゆっくり押すべきか、開ループにすべきかを判断できる。止まって合わせると時間はかかる。KERI's Lab の見積もりでは、ハーフの探索で U ターン1回に約3秒かかる([KERI's Lab 32x32迷路の分析](https://www.kerislab.jp/posts/2020-03-08-all-japan-32x32-maze/))。それでも 16×16 の持ち時間に対しては、払う価値がある。

### オドメトリと電池: 吸引 1000 mm/s のタイヤ径と、走る前に1回だけ読む電圧

小島は、タイヤ径はノギスで測ったあと、長い距離を走らせて合わせよと言う(こじまうす7の目標は、3 m 走って 5 mm 以内のばらつき)。短い距離では、直進の誤差と位置制御の誤差を区別できない。ノギスでは、機体の重さでタイヤがつぶれる分が分からない([小島 2012](https://www.ntf.or.jp/mouse/pertinent_material/121125_mm2012lecture.pdf))。pidream が9区画のパルス数で測り直したタイヤ径は、1年半前より約 10 % 違っていた([pidream 2021](https://www.pidream.net/category/mm/mm-2021))。加藤の事例集によれば、全力でグリップしていてもタイヤは変形して滑る。ある機体では、最大で滑り率約 25 %、滑り角約 20° を測っている([加藤 2013](https://www.ntf.or.jp/mouse/mm2013lecture.pdf))。KOGUMA 自身の測定でも、次のことが分かっている: 柱の端は速度 1000 mm/s あたり約 −6 mm ずれる。吸引なし・500 mm/s・加速度 20000 では、動き出しの空転で全部の端が約 37 mm 遅れた。2000 mm/s の最短走行では、境目がオドメトリの予想より 13〜38 mm 先にあった([docs/pillar_cut_design.md](../docs/pillar_cut_design.md))。吸引 1000 mm/s の探索は、探索の中でいちばん滑りやすい。**ファンを探索の Duty で回したまま、8〜9区画以上**走らせてタイヤ径を確かめるのは、数時間でできる。

電池の電圧も見落とされやすい。UKMARS、発散値、ZoroBot3 は、毎周期「欲しい電圧 ÷ 今の電池電圧」で PWM を決めている([UKMARS motors.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/motors.h#L209-L237)、[ZoroBot3 control.c](https://github.com/OPRobots/ZoroBot3/blob/1056343e78bb8af8ee88d99a58d62865dda423c4/source_code/src/control.c#L67)、[Qiita 発散値](https://qiita.com/akiaki96/items/6498b00ca979ef92b548))。KOGUMA は、走る前に1回読んだ `Run_Voltage` で倍率を固定している([motor.c L144-215](../Core/Src/motor.c))。吸引で数分探索すると、ファンの電流で電圧が下がり続ける。するとフィードフォワードの効きが変わり、ターンの形が少しずつ変わる。`Run_Voltage` を、100 ms 程度のローパスをかけた今の電圧(下限付き)に置き換えるのは数行で済む。

## 向きはジャイロのオフセットから直し、速さは「止まりきれるか」で決める

### オフセットを引かないと、1 dps で既知の直線7区画に約 27 mm 横へずれる

KOGUMA の LSM6DSR は、データシートの typ 値で、ゼロ点のずれが **±1 dps**、温度によるずれが ±0.005 dps/°C である([LSM6DSR datasheet](https://www.farnell.com/datasheets/2802570.pdf))。20 °C 上がっても 0.1 dps にしかならないので、効くのは温度補償ではなく「走る直前に止まって平均を取り、引く」ことである。KERISE は、探索の前と最短走行の前に 200 サンプルの平均を2回取る([KERISE imu.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/hardware/imu.h#L152-L200)、[MazeLib RobotBase.cpp](https://github.com/kerikun11/micromouse-maze-library/blob/3170f7d50be544328257ab63180f2d279793c00a/src/RobotBase.cpp#L215-L219))。ZoroBot3(同じ LSM6DSR)は、フルスケールごとに 200 サンプル(5 ms おき)で測ってフラッシュに保存する([ZoroBot3 lsm6dsr.c](https://github.com/OPRobots/ZoroBot3/blob/1056343e78bb8af8ee88d99a58d62865dda423c4/source_code/src/lsm6dsr.c#L162-L196))。発散値は、ファンを先に回し、振動と電圧が落ち着いてから測る([Qiita 発散値](https://qiita.com/akiaki96/items/6498b00ca979ef92b548))。加藤も、気温や通電による発熱でゼロ点が狂うことには「走行の直前にゼロ点補正を行うことで対応」としている([加藤 2013](https://www.ntf.or.jp/mouse/mm2013lecture.pdf))。

KOGUMA は `gyro_z_offset = 0.2` を持っているが、引く処理(`//- gyro_z_offset`)はコメントアウトされている。倍率は 1.01 に決め打ちで、|z| < 0.5 dps を0にする不感帯でごまかしている([lsm6dsr.c L21, L129-141](../Core/Src/lsm6dsr.c))。このため、走っているあいだ、本当の回転とバイアスの和が ±0.5 dps に入っている限り、ジャイロのループには0に見え、向きは直されないまま流れる。不感帯の外では、バイアスがそのまま目標からのずれになる。0.5〜1 dps 流れたとすると、1区画(1000 mm/s で 0.18 s)では 0.1〜0.2° と小さい。しかし横壁のない既知の直線7区画を 500 mm/s で走ると、1 dps なら 2.5 s で 2.5° ずれ、終わりで横に**約 27 mm** ずれる計算になる。手順はこうなる。まず 60 秒止めて角度を積分し(ファンあり・なしの両方)、ずれを数字にする。次に、ファンを回して落ち着いてから 300〜500 サンプルの平均をオフセットにする。ばらつき(標準偏差)が大きければ測り直す。最後に不感帯を外す(残すなら 1 LSB、つまり ±4000 dps のとき 0.14 dps 以下にする)。標準偏差でやり直す部分は、先人のコードにはない追加の提案である。

倍率の合わせ方にも、先人の工夫がある。Micras は壁に向けて置いて5回転し、回る前と後の壁に対する向きを前の2つのセンサで測る。本当に回った角度 ÷ 積分した角度が倍率になる([Micras gyroscope_calibration.hpp](https://github.com/Team-Micras/MicrasFirmware/blob/4dac6e2c7299ca664712ef5de177e5911152ad63/micras_nav/include/micras/nav/gyroscope_calibration.hpp))。小島は、左旋回と右旋回で別の係数(例: 0.01890 と 0.01914)を勧めている([小島 2012](https://www.ntf.or.jp/mouse/pertinent_material/121125_mm2012lecture.pdf))。garberas は、MPU-6500 の感度の誤差を 0.98 倍で直したら、I ゲインを上げてごまかさなくても 180° ターンが合うようになった([garberas ジャイロの誤差の補正](https://garberas.com/archives/1673))。ただ、これらは今のターンの調整を崩しうるので、大会のあとに回すのが無難である。

### KOGUMA は、ターンの終わりに残った向きの誤差を毎回捨てている

KOGUMA は、動作を始めるたびに `G_Motor_Angle` とジャイロの I 項を0に戻している([motor.c](../Core/Src/motor.c))。そのため、スラロームの終わりに残った 1〜2° の誤差は捨てられる。次の直線はその向きを「まっすぐ」として保ち、直すのは横壁制御だけになる。KERISE は毎周期、向きを積分する。ターンの終わりには予定の角度だけを引き、**残った誤差を次の直線に持ち越して**、軌道の追従で直す([KERISE speed_controller.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/supporters/speed_controller.h#L142-L166))。超信地旋回では、角度の加速度プロファイルで回ったあと、角度の誤差の PI(Kp = Ki = 10)で |Kp·e| + |Ki·∫e| < 0.1π になるまで追い込む([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L567-L599))。1000 mm/s の吸引探索で「ターンのあとにふらつく」「片側にしか壁がないところで寄っていく」のなら、これで説明がつく可能性がある。ただし、誤差の持ち越しをオフセットの補正より先に入れると、バイアスまで持ち越してしまう。大会前は、全ターンの終わりの「`G_Motor_Angle` − 目標角」を記録し、大きさと左右の差を見るところまでにとどめる。

### 探索中の横壁制御: 止まった状態からの動き出しと、止まる途中では切る

UKMARS は「never steer from zero speed」、つまり止まった状態から動き出すときは横壁制御を切る。ターン、区画の中心で止まる動作、U ターンでも切り、区画ごとのループの最初で入れ直す([UKMARS mouse.h](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/mouse.h#L399))。KERISE は逆に、次の行動が決まるまで減速しながら待つあいだも、前壁と横壁の補正を続ける([KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L768-L790))。Micras は、光の点が地図上で分かっている1枚の壁の面に当たっているときだけ横のセンサを使い、ずれがもっともらしいかを確かめる。1回の補正量には、位置 0.002 m・向き 0.01 rad の上限を付ける。「間違った読みは捨て、地図の壁が間違っていても少ししか動かない」ためである([Micras localizer.hpp](https://github.com/Team-Micras/MicrasFirmware/blob/4dac6e2c7299ca664712ef5de177e5911152ad63/micras_nav/include/micras/nav/localizer.hpp#L18-L185))。

迷路そのものも完全ではない。規定は、壁と床に「1ｍｍ程度の隙間あるいは段差が生じることがある」と認めている([NTF クラシック競技規定](https://www.ntf.or.jp/mouse/kitei_classic.html))。華金は「段差が0.3mmでもあるとタイヤが浮いてしまいフェイルセーフがかかっていました」と書いている([haido 2019-12-21](https://haido.blog.jp/archives/1076453798.html))。KOGUMA の横壁制御のゲインは速度/500 倍なので、低速では小さい。それでも、ログで確かめる価値のあることが2つある。スタート直後と U ターンのあとの動き出しで、柱を見て向きが乱れていないか。そして発散値のように、止まるあいだの壁制御で袋小路での向きが曲がっていないか。

### 探索速度: KOGUMA はすでに上位と同じ帯にいる

| 機体 | 探索速度 | 条件 | 出典 |
|---|---|---|---|
| 華金(2019年 全日本クラシック優勝) | 直進・旋回とも 1.0 m/s、加速度 15 m/s² | 吸引機(探索で吸引したかは書かれていない) | [haido 2019-12-03](https://haido.blog.jp/archives/1076428466.html) |
| ZoroBot3(180 mm) | 0.8 m/s、8 m/s²、既知区間 3.5 m/s | 吸引(2S で 35 %、3S で 20 %) | [ZoroBot3 move.c](https://github.com/OPRobots/ZoroBot3/blob/1056343e78bb8af8ee88d99a58d62865dda423c4/source_code/src/move.c#L802-L822) |
| Micras(180 mm) | 最大 1.0 m/s(タイヤが出せる力の 0.5 倍まで) | 吸引なし | [Micras constants.hpp](https://github.com/Team-Micras/MicrasFirmware/blob/4dac6e2c7299ca664712ef5de177e5911152ad63/config/constants.hpp#L285-L296) |
| UKMARS | 0.4 m/s、2 m/s²、ターン 0.3 m/s | ジャイロなし | [UKMARS config](https://github.com/ukmars/mazerunner-core/blob/92fb779794c7fced5959f151efea359dac9a37e6/mazerunner-core/config-robot-osmium.h#L271-L273) |
| KERISE | 0.33 m/s(未知区間加速で約 0.72 m/s) | ハーフサイズ | [KERISE move_action.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/move_action.h#L829-L899)、[KERI's Lab 未知区間加速](https://www.kerislab.jp/posts/2019-10-12-unknown-accel/) |
| KOGUMA MkIIl | 0.5 m/s(スラローム)、1.0 m/s(吸引) | 2輪、吸引 | — |

Micras は、探索速度を決めているのは前壁に向かうブレーキだと書く。タイヤが出せる力の半分を超えて走ると、前壁で位置が遅れて直ったときに、タイヤが出せる以上のブレーキが要るからである([Micras constants.hpp](https://github.com/Team-Micras/MicrasFirmware/blob/4dac6e2c7299ca664712ef5de177e5911152ad63/config/constants.hpp#L285-L296))。止まれる距離 v²/(2a) は、1000 mm/s・5 m/s² で 100 mm、10 m/s² で 50 mm である。1000 mm/s の吸引探索では、前壁を「区画の中心で止まれる最後の点」より前に確実に見ているかを、ログで確かめるべきである。Harrison も、たまに起こる誤差でターンの前に十分減速できないことがあったため、探索のターン速度を 700 mm/s に抑えた([micromouseonline 2014](https://micromouseonline.com/2014/09/07/exploration-tune/))。

既知区間の加速は、時間を縮めるが不具合の元にもなる。pidream は2024年の全日本の直前に、既知区間を加速したすぐあとの未知の直線で、壁切れの距離補正が1区画ずれる不具合を見つけた。最後は既知区間加速の処理を全部外して出場した([pidream 2024](http://www.pidream.net/category/mm/mm-2024))。KOGUMA も既知区間を `Motor_trapezoid_PID(1000, 4000, 1000, ...)` で走っている([Move.c L300-312](../Core/Src/Move.c))。既知区間から未知の直線へのつなぎ目を記録で確かめ、確かめきれなければ、大会では既知区間も探索速度で走るという選択肢がある。同じ考え方で、大会での速度の切り替え基準も決められる。食い違いカウンタが 1000 mm/s の吸引探索で読み違いを示したら、未知区間だけ 500 mm/s に落とす。KERISE が未知区間を遅く、既知区間を速く走るのと同じ発想である。ファンについても例がある。華金は、吸引モーターの電圧設定を 2S のまま走らせて吸引が弱まり、グリップが抜けて壁に当たり、軸を曲げ、吸引の MOSFET も焼いた([haido 2019-12-03](https://haido.blog.jp/archives/1076428466.html))。Duty 100 % では5秒で焼けるので、実用は 50 % までという例もある([xfa273 Ca.161/bis](https://xfa273-backofchirashi.hatenablog.com/entry/2023/12/19/151923))。5分近い吸引探索では、Duty と発熱を確認項目に入れたい。

## フェイルセーフは「続いたら止め、壁を消して迷路を残す」

加藤は「フェールセーフには誤作動のリスクもありますが、フェールセーフがないことによる上記のリスクのほうが圧倒的に有害です」と言い切る。自分のマウスでは、**角度偏差か速度偏差が一定以上の状態が 80 ms 続いたらエラー**とし、急に変わる値(加速度や角速度)は判定に使わず、エンコーダと慣性センサを組み合わせてタイヤのロックと空転の両方を見ていた。エラーが出たら、直前に見た壁をいくつか消して迷路を保存した([加藤 2013](https://www.ntf.or.jp/mouse/mm2013lecture.pdf))。今の実装例の数字もよく似ている。pidream は、速度が目標の半分未満の状態が 0.5 秒続いたとき、角度が直線で 30°・ターンで 90° ずれたとき、ターンの入りから出までが 0.5 秒を超えたときに止める。止め方は、PWM 0 % のブレーキで回転と速度が止まるのを待ってから、吸引ファンを止める([pidream クラッシュ時の止め方](http://www.pidream.net/2023/12/231202.html))。華金は、左右の速度差が 50 ms 続いたら止める([haido 2019-12-21](https://haido.blog.jp/archives/1076453798.html))。閾値は、壁に軽く触れたくらいでは止まらず、ひどくぶつかったら止まるように合わせる。走行中のマウスに触れてはいけない規定があるので、自分で止まれば回収が早いからである([haido マイクロマウスにおける安全文化](https://haido.blog.jp/archives/1057813907.html))。ZoroBot3 は、PWM が 1000/1024 以上の状態が 100 周期(1 kHz で 100 ms)を超えて続いたら止まる([ZoroBot3 motors.c](https://github.com/OPRobots/ZoroBot3/blob/1056343e78bb8af8ee88d99a58d62865dda423c4/source_code/src/motors.c#L84-L130))。Micras は、次のどれかで止まる: 横の加速度が、ファンありでタイヤが出せる限界の 1.25 倍を 5 ms 続けて超えた、モーターが 100 ms 張り付いた、IMU から 5 ms 値が来ない([Micras constants.hpp](https://github.com/Team-Micras/MicrasFirmware/blob/4dac6e2c7299ca664712ef5de177e5911152ad63/config/constants.hpp#L55-L123))。KERISE は、モーター指令の絶対値が 1.3 を超えたら非常停止する([KERISE motor.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/hardware/motor.h#L129-L161))。

KOGUMA のフェイルセーフを読むと、左右の速度差の閾値は 100000 で、事実上効かない。「目標 − 実際 > 2000」と、探索中(目標 500〜1200、加速0)の「|速度| < 100」は、**1回の値で即座に**止まる([Failsafe.c](../Core/Src/Failsafe.c))。角度のずれ、ターンの時間切れ、IMU の異常は見ていない(起動時の WHO_AM_I の確認もコメントアウトされている、[lsm6dsr.c L60-67](../Core/Src/lsm6dsr.c))。止めるときは割り込みの中で `Motor_Free()` を呼ぶ。これはモーターの PWM を止めるのと同時に、`Suction_Stop()` でファンも切る([stm32f4xx_it.c L222-231](../Core/Src/stm32f4xx_it.c)、[motor.c L298](../Core/Src/motor.c))。つまり 1000 mm/s で走っている最中に、押し付ける力を一気に失い、ブレーキもかけずに惰性で滑ることになる。迷路は RAM にしかない。

大会前に足す形は、先人の数字をそのまま借りられる。今の条件に「50〜100 ms 続いたら」を付ける(1000 mm/s で 80 ms は約 80 mm 進む距離)。角度のずれ(直線 30°、ターン 90°)と、ターンの時間切れ(0.5 秒か予定の2倍)を足す。KOGUMA は動作ごとに角度を0にしているので、`G_Motor_Angle` と目標角の差でそのまま判定できる。止め方は、ブレーキ → 止まったのを確かめる → ファンを止める、の順にする。続いた時間の条件には、華金が悩んだような小さな段差での誤作動を防ぐ意味もある。練習迷路に紙やテープで 1 mm 程度の段差を作り、誤作動しないかを試すとよい。

止まったあとの扱いは規定に関わる。NTF クラシック競技規定の 3-2 は、競技中に「スイッチ操作等で、迷路に関する情報を修正、あるいは部分的に消去することはできない」と定める。3-5 は、競技委員長が「あきらかに走行に異常が認められた場合」は走行中止の申し出を認め、それ以外の中止は迷路の記憶をすべて消すことを条件にすると定める([NTF クラシック競技規定](https://www.ntf.or.jp/mouse/kitei_classic.html))。したがって、壁を消すのは操作者ではなく、**異常を見つけたマウスが自動で**行うのが筋である。加藤も KERISE もこの形をとっている(KERISE は直近6件、だいたい2区画分、[KERISE maze_robot.h](https://github.com/kerikun11/micromouse-kerise-firmware/blob/34d90675168d1875af36425a0ba0f440e8248563/src/agents/maze_robot.h#L313-L356))。この自動の消去が 3-2 に触れないと委員会が明文で認めた資料は見つからなかったが、上位者がふつうにやっている形である。KOGUMA には自己位置同定がないので、立て直しは「止まる → 許可を得て回収する → スタートに置く → 残った迷路で探索を続ける」になる。そのためには**電源を切らずに済むこと**(RAM の迷路が残ること)が最低限で、電源を切るならフラッシュへの保存が要る。

フラッシュへの保存には注意が要る。STM32F4 はバンクが1つなので、消去や書き込みのあいだはフラッシュが読めず、CPU と割り込みが止まる([ST Community](https://community.st.com/stm32-mcus-products-25/cpu-stalling-when-erasing-flash-on-stm32f405-89560))。128 KB セクタの消去は、同じ系統の別の型番で約1〜2秒かかり、Micras の記述では約2秒、最悪4秒かかる([alldatasheet STM32F412](https://www.alldatasheet.net/html-pdf/2178984/STMICROELECTRONICS/STM32F412RGU6P/269151/118/STM32F412RGU6P.html)、[Micras constants.hpp](https://github.com/Team-Micras/MicrasFirmware/blob/4dac6e2c7299ca664712ef5de177e5911152ad63/config/constants.hpp#L176-L185))。F446 そのものの値は確認できていない。だから保存は、ZoroBot3 のように、ゴールで止まったときと探索の終わりに、モーターとファンを止めた状態でだけ行う([ZoroBot3 floodfill.c](https://github.com/OPRobots/ZoroBot3/blob/1056343e78bb8af8ee88d99a58d62865dda423c4/source_code/src/floodfill.c#L1240-L1253))。最後のセクタ(128 KB)を迷路用に空ける案はリンカの設定の変更を伴うので、セクタ構成を RM0390 で確かめてからにすべきである。エキスパートクラスでは途中で電池を交換できない([NTF Classic rules EN 2010](https://ntf.or.jp/mouse/micromouse2010/ruleclassic-EN.html))。探索と最短走行4回を持ち時間いっぱい走らせ、最後の電圧を見る通し練習も、フェイルセーフの一部と考えたい。九州地区大会2026の持ち時間と走行回数は、大会の案内で確かめる必要がある。

## 6日で入れるもの、大会のあとに回すもの

大会までの6日は、新しい推定器や自己位置同定に手を出す時間ではない。入れる順番は、挙動を変えない記録 → 壊れにくく効果が見えやすいもの → 動きが変わるもの、とする。動きが変わるものは、全面探索を何回か回してから入れるかどうかを決める。どれを入れるかは、記録が示す失敗の種類で決める。下の表の「目安」の時間は推定である。

| 段階 | やること(具体的な中身) | 目安 | もとにした先人 |
|---|---|---|---|
| 1 記録 | 判断位置ごとに x, y, 向き、4つのセンサ値(生の値と av10)、実測の位置、ジャイロの角度を記録する | 半日 | UKMARS、発散値 |
| 1 記録 | 既知の区画で、地図と読みの食い違いを数え、ブザーを鳴らして記録する(挙動は変えない) | 2〜3時間 | KERISE / MazeLib |
| 1 記録 | 前壁トリガで打ち切ったときの `G_Motor_X − X0`、FL−FR、横壁の有無。全ターンの終わりの角度の誤差。背中当ての前後の角度 | 2〜3時間 | 発散値、UKMARS |
| 1 記録 | ジャイロを60秒止めて角度を積分する(ファンあり・なし) | 1時間 | 小島、データシート |
| 2 壊れにくい | ジャイロのオフセット: ファンを回して落ち着いてから 300〜500 サンプルの平均を引く。ばらつきが大きければやり直す。±0.5 dps の不感帯を外す | 半日 | KERISE、ZoroBot3、発散値 |
| 2 壊れにくい | 前壁の地図の判定を AND から平均に。60 超で壁あり、40 未満で壁なし、その間は未知のまま次の判断位置で読む(40 は確かめるべき出発点) | 2〜3時間 | UKMARS |
| 2 壊れにくい | フェイルセーフ: 50〜100 ms 続いたら止める。角度のずれ 30°/90°、ターン 0.5 秒か予定の2倍。ブレーキ → 止まったのを確認 → ファンを止める | 半日 | 加藤、pidream、華金、ZoroBot3 |
| 2 壊れにくい | `Run_Voltage` を、100 ms 程度のローパスをかけた今の電圧にする(下限付き) | 1〜2時間 | UKMARS、発散値、ZoroBot3 |
| 2 壊れにくい | タイヤ径を、吸引 ON・探索の Duty で 8〜9 区画以上走らせて確かめる | 2〜3時間 | 小島、pidream |
| 2 壊れにくい | 余裕の試験: ±20 mm の横ずれ・±3〜5° の向きで「壁あり」の最小値、前壁や次の柱がある状態で「壁なし」の最大値を記録し、その間に閾値を置く。ファン ON。会場の照明で `Wall_Center_Calibration()` | 半日+会場 | HM-StarterKit、rofumi、UKMARS |
| 3 動きが変わる | 曲がる側の横センサが壁ありなら、スラロームをやめて止まる。直進の前に、前の値がすでに壁の近さなら止まる | 半日 | KERISE AEBS、Micras |
| 3 動きが変わる | 前壁トリガの早い側に窓を付ける(30〜40 mm、ログで決める)。外れたら距離で曲がるか、止まって超信地旋回 | 半日 | KERISE |
| 3 動きが変わる | フェイルセーフが働いたら、直近の壁(2区画分程度)を自動で消し、迷路は RAM に残す。歩数マップが作れなければ直近の壁を消す | 半日〜1日 | 加藤、KERISE、KERI's Lab |
| 3 動きが変わる | 壁切れは記録だけ(1)で出場する。SCUT が直す価値を示したときだけ、prev = 2 を除き、prev = 0 だけ、ゲイン 0.5 でモード2 | 記録しだい | 発散値 |
| 3 動きが変わる | 既知区間 → 未知の直線のつなぎ目を確かめる。だめなら既知区間も探索速度で走る。1000 mm/s で食い違いが出るなら、未知区間は 500 mm/s | 半日 | pidream、KERISE |
| 4 前日・当日 | タイヤを粘着テープで掃除(溶剤は禁止)、車高とスカート、ファンの電圧設定、ねじのゆるみ、持ち時間いっぱいの通し走行での電池、モードとゴール位置の表示、1 mm の段差での誤作動 | — | 規定、華金、加藤 |

記録の結果しだいで足すものもある。袋小路で向きが曲がっていれば、止まるあいだの壁制御を切る(発散値)。背中当てのあとに角度が1°を超えて残るなら、押す速度を下げるか開ループにする(KERISE、RT)。余裕の試験で横の余裕が小さいと分かった方向には、3点での多数決を足す。迷路のフラッシュ保存は、大会では電源を切らない運用で代え、時間が余ったときだけ、止まっている状態に限って入れる。大会のあとに回すのは、作りを変えるものである。

| 大会のあとにやること | もとにした先人 |
|---|---|
| 前後と向きの推定値を、ゲイン 0.05〜0.4 と向き 0.5° 以内の関門で少しずつ直す形に作り直す | KERISE |
| スラロームの前後の距離を、本当の境目(基準0)で合わせ直す。柱や前壁の測定をそのまま使えるようにする | garberas、KOGUMA の設計資料 |
| 地図から予想した端の待ち行列(期限切れ、非対称の窓)と、+10 mm の注入試験 | 発散値 |
| 前壁の誤差を次の S90 の前距離に入れる(不感帯 10 mm、ゲイン 0.5、上限 15 mm)。壁までの距離でターンの半径を調整する | 発散値、pidream |
| ターンの終わりの向きの誤差を持ち越す。超信地旋回の最後を角度の PI で閉じる。前壁合わせに失敗したら 90° → 横の壁で合わせる → 90° | KERISE |
| 止まるたびにジャイロとエンコーダを比べてバイアスを更新する。壁に向けて5回転して倍率を測る。左右で別の係数にする。探索は ±2000 dps(フルスケールごとにオフセット) | Micras、小島、ZoroBot3 |
| 止まって下がって読み直す。壁ごとの確からしさを数える。両側の区画からの観測を使う | 発散値 |
| センサを距離に直し、mm の閾値とヒステリシスで判定する。ジャイロの角度で補正するか、ロジスティック回帰を使う | KERISE、rofumi |
| 壁ログ(1件2バイト)か、チェックサムと連番の付いたジャーナルを2つの領域で交互に使い、U ターンのたびに保存する | KERI's Lab、発散値、KERISE |
| 区画の中心で止まるたびに、数えた区画と位置の推定を比べ、食い違えば壁を書かずに走行をやめる | Micras |
| 加速度とエンコーダの融合(α = 0.2)、滑り角のモデル | KERISE v4 |
| LED の変調(10 kHz)、点けてから読むまでの待ちを長くする(80 µs)、前センサを 5° 外へ開く | id研、rofumi、RT |

## 段階2を入れた結果(2026-10-19)

上の表の「2 壊れにくい」のうち、ジャイロ・前壁・電池・フェイルセーフを検討した。2026-10-18 のログで確かめると、2つは表のままでは壊れにくくなかったので、中身を変えた。

| やること | どうしたか | 理由(ログ・コード) |
|---|---|---|
| ジャイロのオフセット | **入れた**。`Motor_Setup` のたびに 0.3 秒止まって平均を取り、引く(`lsm6dsr.c` の `Gyro_Calibrate`)。ばらつき 1.5 dps 超か平均 5 dps 超なら使わず、前の値のまま。不感帯は、一度測れたら 0.5 → 0.15 dps(1 LSB = 0.14 dps)、まだなら 0.5 のまま | 表では「ファンを回して落ち着いてから」としたが、10-18 の最短走行のログで、吸引を回して止まっているあいだの角速度は **標準偏差 4〜5 dps**(平均 +0.1〜+0.6 dps)だった。この揺れの中で 0.3 秒平均しても ±0.3 dps ほどしか決まらない。そこで吸引を回す前(探索・最短とも、最初の `Motor_Setup` は吸引の前)に測り、吸引を回したあとの `Motor_Setup` では、ばらつきで弾かれて前の値のままになる |
| 前壁の地図の判定 | **記録だけ**(`motor.c` の `Front_Wall_Map` = 1)。判断位置で前の左右の片方だけが閾値を超えたとき(分かれたとき)を `FWALL` に残す。2 にすると、平均でも壁ありにする(`Wallsensor.c` の `Wall_Front_For_Map`。平均が閾値を超え、低いほうも閾値の 1/3 を超えたとき) | 境目で前壁があれば左右とも約 140(`SULA_FRONT_WALL_TH` のコメント)で、閾値は 60。壁があって片方だけ 60 を切るには、左右で 2.3 倍以上の差が要り、ふつうは起きない。むしろ壁がないのに片方が柱などを見て 60 を超えることのほうがありうる。そのとき平均で書くと、ない壁を地図に書いて道をふさぐ。10-18 の 8×6 の迷路では読み違いは0。どちらが起きるか分からないので、まず分かれたところを記録して、本当の迷路と比べてから決める。「40 未満で壁なし、その間は未知」は迷路の側の作り直しが要るので大会のあと |
| 電池の電圧 | **入れなかった**(`Run_Voltage` は今までどおり、走る前の止まった電圧) | 10-18 の最短走行(2000・2400)で、電池は止まって 16.0 V、吸引を回して 15.7 V、加速中は **13〜14 V、いちばん低くて 12.4 V** まで下がる。ターンは「走る前の止まった電圧」で倍率を決めて合わせてある。今の電圧に追わせると(100 ms のローパスでも)加速中のモーターの電圧が合わせたときより 10〜20 % 上がり、電流が増えるとさらに電圧が下がって倍率が上がる。探索の数分で電池が少しずつ下がる分(0.数 V)より、こちらの影響のほうがずっと大きい。入れるならターンを合わせ直すときに(大会のあと) |
| フェイルセーフ | **入れなかった** | 今は1サンプルで判定し、働くと割り込みで `Motor_Free()`(PWM と吸引を止める)するだけ。動きのループ(`G_Motor_X > X` を待つ `while`)は抜けないので、機体はその場で止まったまま、リセットするしかない(迷路は区間の終わりごとにフラッシュに保存した分だけ残る)。50〜100 ms 続いたら、という条件は足せるが、角度のずれを見るには `G_Motor_Angle` をどの動きでも0から数える必要があり(`Wallcut_*` や `Motor_Stop` では戻していない)、「止めて迷路を残す」にはループを抜ける作りが要る。大会のあと |

**2026-10-20 の実機**: ゼロ点は3回とも測れていなかった(`gyro_rej=3`、2回走って2回とも)。ジャイロの上位と下位を別々に読んでいて、そのあいだに値が替わると ±35.8 dps に飛ぶ(止まっていても約1%)。300回のうち1回飛ぶだけでばらつきが 2 dps を超えるので、閾値 1.5 dps で弾かれていた。読み方を直した(上位を読み直し、替わっていたら下位も読み直す。`docs/changelog.md` の 2026-10-20)。前壁の読み分かれは2回とも0。直したあと(16:18)は、吸引の前と探索のあとでゼロ点が測れた(ばらつき 0.31〜0.36 dps、ゼロ点 0.10〜0.15 dps)。吸引を回すと、飛びがなくてもばらつき 4 dps(本当の振動)なので、吸引の前に測る作りのままでよい。走行ログの飛びは0になり、最短の走りは変わらなかった。

次の全面探索(ゴール (7,7))で見ること: 出力の `SCUT` の見出しの `gyro_ofs`・`gyro_sd`・`gyro_rej`(ビューアの「探索の直進の壁切れ」にも出る)、`FWALL`(ビューアの「前壁の読み分かれ」)。走行ログの走り出す前(吸引を回して止まっているあいだ)の角速度の平均が 0 に近づいていれば、吸引なしで測ったゼロ点が吸引ありでも合っている。

## 結論

2026年のいちばん詳しい記録(発散値)から読み取れる教訓は、補正を増やしても安定するとは限らないということである。1回の IR の前壁の読みは σ 11〜15 mm ぶれ、次の壁切れの誤差とはほぼ無相関(r = −0.08)だった。壁切れの窓を 20 mm から 30 mm に広げただけでクラッシュした。上位者の頑丈さは、1回の値では動かないことと、「おかしい」を補正ではなく停止に変えることから来ている。この見方に立つと、KOGUMA が動作ごとに角度と位置を0に戻す作りは、弱点であると同時に、誤差を次の動作に広げない防波堤にもなっている。だから大会前の安いガードは、今の作りによくなじむ。KERISE や Micras のように推定値を少しずつ直す作りは大会のあとの行き先だが、それにはジャイロのオフセットを引いてあることが前提になる。どの道を選んでも、オフセットの補正が最初の一歩になるのはそのためである。

もう一つの結論は、足りないのは手法ではなくデータだということである。1000 mm/s で吸引しながら探索したときの読み違いの率や、探索中の柱の端のばらつきを数字で出した公開資料は、一つも見つからなかった。KOGUMA 自身の記録(食い違いカウンタ、SCUT、前壁トリガの誤差、ターンの終わりの角度)が唯一の証拠になる。だから1日目の記録が、残りすべての判断を決める。大会でいちばん大きな判断である「未知区間を 1000 mm/s で探すか、500 mm/s に落とすか」も、その記録で決めることになる。
