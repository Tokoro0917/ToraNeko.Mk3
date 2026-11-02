# tools/vscode/

VS Code の設定のひな形。`.vscode/` はリポジトリに入れない決まり（`.gitignore`）なので、ここに置いて、使う人がコピーする。

## tasks.json：■ を押さずに書き込む（Ctrl+Shift+B）

F5 は「書き込み ＋ デバッグ」なので、書き込んだあともデバッグが続き、■（停止）を押すまで次の書き込みができない。
`tasks.json` の「書き込み」は、ビルド → 書き込み → リセットして走らせる、だけをする（デバッグを始めない）。

### 入れ方（1 回だけ）

1. VS Code で `software/main-controller/TORANEKO.Mk3` フォルダを開いていることを確かめる
   （左のエクスプローラーのいちばん上が `TORANEKO.MK3`。その中に `.vscode` フォルダと `CMakePresets.json` がある）
2. `TORANEKO.Mk3/.vscode/` の中に `tasks.json` が**ない**ことを確かめる
   - もうあるときは上書きしない（中の設定が消える）。Claude に「tasks.json がもうある」と伝える
3. このフォルダの `tasks.json` を、`TORANEKO.Mk3/.vscode/tasks.json` にコピーする

### 使い方

1. ロボットに ST-LINK をつなぐ（F5 のときと同じ）
2. **Ctrl+Shift+B** を押す
3. 下の「ターミナル」に、ビルドのあと `File download complete` と `MCU Reset` が出たら成功。ロボットは書き込んだプログラムで動き始める
4. 次に書き込むときも、そのまま Ctrl+Shift+B（■ は押さない）

### うまくいかないとき

| 出たメッセージ | どうする |
|---|---|
| `STM32_Programmer_CLI` が見つからない（`認識されません`・`not found`） | 下の「書き込みソフトの場所」を見る。わからなければ、出たメッセージを Claude に貼る |
| `ST-LINK` に接続できない（`No STM32 target found`・`in use` など） | F5 のデバッグが続いていないか見る（画面の上にボタンの帯が出ていたら ■ で止める）。ケーブルと電源も確かめる |
| ビルドで止まる | 1 回だけ F5（またはコマンド `cmake --preset Debug`）でビルドの準備をしてから、もう一度 Ctrl+Shift+B |

### 書き込みソフトの場所（STM32_Programmer_CLI が見つからないとき）

STM32CubeIDE しか入っていない PC（作者の Linux のノート PC がそう）では、書き込みソフトが IDE のフォルダの奥にあって、
そのままでは見つからない。1 回だけ、次のようにしてどこからでも使えるようにする（2026-11-02 にこれで書き込めた）。

1. 場所を探す：`find /opt/st -name STM32_Programmer_CLI`
   （例：`/opt/st/stm32cubeide_1.14.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.1.100.202311100844/tools/bin/STM32_Programmer_CLI`）
2. よく使う場所に目印（シンボリックリンク）を置く：`sudo ln -s <1 で出た行> /usr/local/bin/STM32_Programmer_CLI`
3. 確かめる：`STM32_Programmer_CLI --version` で `STM32CubeProgrammer version: 2.15.0` などと出れば成功

CubeIDE を新しい版に入れ替えると、1 の場所が変わって目印が切れる。そのときは `sudo rm /usr/local/bin/STM32_Programmer_CLI` で消してから、1〜3 をやり直す。
