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
| `STM32_Programmer_CLI` が見つからない（`認識されません`・`not found`） | STM32CubeCLT（STM32Cube 拡張と一緒に入れたもの）の `STM32CubeProgrammer\bin` を PATH に足して、VS Code を開き直す。わからなければ、出たメッセージを Claude に貼る |
| `ST-LINK` に接続できない（`No STM32 target found`・`in use` など） | F5 のデバッグが続いていないか見る（画面の上にボタンの帯が出ていたら ■ で止める）。ケーブルと電源も確かめる |
| ビルドで止まる | 1 回だけ F5（またはコマンド `cmake --preset Debug`）でビルドの準備をしてから、もう一度 Ctrl+Shift+B |
