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

メイン基板のファームウェアは KOGUMA.MkIIl をベースにし、`main-controller/` に取り込みました（STM32F411CEU6 への移植はこれから）。詳細は [main-controller/README.md](main-controller/README.md) を参照。
