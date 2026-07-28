# hardware/kicad/

KiCad プロジェクトを基板（サブシステム）ごとにディレクトリを分けて配置してください。

例:

```
kicad/
├── TORANEKO.Mk3/
│   ├── TORANEKO.Mk3.kicad_pro
│   ├── TORANEKO.Mk3.kicad_sch
│   ├── TORANEKO.Mk3.kicad_pcb
│   ├── BOM.csv
│   └── README.md
└── motor-driver-board/
    └── ...
```

## 現在のプロジェクト

- [TORANEKO.Mk3/](./TORANEKO.Mk3/) — メイン基板（1枚基板・全機能統合）。KiCadプロジェクト作成済み（現状は空、これから回路図を起こす）。機能ブロック・ペリフェラル割り当て案・BOMドラフトも同ディレクトリに格納。
