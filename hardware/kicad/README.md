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

まだプロジェクトがないため、このディレクトリは現在空です。
