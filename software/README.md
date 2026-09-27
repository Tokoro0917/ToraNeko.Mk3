# software/

ToraNeko.Mk3 のソフトウェア一式を格納します。

## 構成の考え方

サブシステムやマイコンボードごとにディレクトリを分けて配置してください。例:

```
software/
├── main-controller/   # メイン基板（例: STM32等）のファームウェア
├── motor-driver/      # モータドライバ基板のファームウェア
└── pc-tools/          # デバッグ・調整用のPC側ツール
```

## 既存資産について

KOGUMA.MkIIl のファームウェアを参照用として `reference/koguma-mkiil/` に取り込みました。メイン基板のファームウェアは、これを参照しながら `main-controller/` に一つずつ実装していきます。
