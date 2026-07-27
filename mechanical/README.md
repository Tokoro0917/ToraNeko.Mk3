# mechanical/

機構部品・筐体・3Dモデル・図面など、機械設計に関するデータを格納します。

## 構成の考え方

パーツ・アセンブリ単位、もしくはサブシステム単位でディレクトリを分けて配置してください。例:

```
mechanical/
├── frame/         # 車体・フレーム
├── drive-unit/    # 駆動系機構
└── exported/      # 発注・共有用に書き出したSTEP/STL等
```

CADツール（Fusion360 / SolidWorks 等）は未確定です。決まり次第、本READMEに追記します。
