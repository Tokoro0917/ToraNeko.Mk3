#!/bin/bash
# push_logs.sh
# logs/ に保存したログを、まとめて GitHub に送る（git pull → git add → git commit → git push を 1 回で）
#
# ■ 使い方
#   1 回だけ、次の命令で「git logs」という短い命令を作っておく（ToraNeko.Mk3 のどのフォルダでもよい）：
#     git config alias.logs '!bash software/main-controller/tools/push_logs.sh'
#   そのあとは、ログを保存するたびに、ターミナルで次を打つだけ：
#     git logs
#   （git の「alias（別名）」の機能。先頭に ! を付けると、git は ToraNeko.Mk3 のいちばん上のフォルダで、
#     その後ろの命令を実行する）
#
# ■ すること
#   1. ToraNeko.Mk3 のいちばん上のフォルダへ移る
#   2. git pull で、GitHub の新しい変更を取り込む
#   3. software/main-controller/logs/ のファイルだけを git add する（ほかのファイルは入れない）
#   4. 新しいログがあれば、日時を名前にしてコミットする
#   5. git push で送る

# どこかで失敗したら、そこで止める（失敗したまま次へ進まない）
set -e

cd "$(git rev-parse --show-toplevel)"

echo "== 1/3 GitHub の新しい変更を取り込む（git pull）"
git pull

echo "== 2/3 logs/ のファイルをコミットする"
git add software/main-controller/logs/
if git diff --cached --quiet; then
  echo "新しいログはありません（コミットしない）"
else
  git commit -m "ログ $(date '+%Y-%m-%d %H:%M')"
fi

echo "== 3/3 GitHub に送る（git push）"
git push

echo "== 送れました"
