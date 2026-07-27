# hardware/kicad/

KiCad プロジェクトを基板（サブシステム）ごとにディレクトリを分けて配置してください。

例:

```
kicad/
├── main-board/
│   ├── main-board.kicad_pro
│   ├── main-board.kicad_sch
│   ├── main-board.kicad_pcb
│   ├── BOM.csv
│   └── README.md
└── motor-driver-board/
    └── ...
```

## 現在のプロジェクト

- [main-board/](./main-board/) — メイン基板（1枚基板・全機能統合）。機能ブロック・ペリフェラル割り当て案・BOMドラフトを先行して作成済み。KiCadプロジェクト本体（`.kicad_pro`等）は未作成。
