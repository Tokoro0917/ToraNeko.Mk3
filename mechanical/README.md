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

## 補助パーツ（FDM 印刷）

機体に取り付ける補助パーツと、製作・調整用の治具です。どれも CadQuery の生成スクリプトと、印刷用の STL、Fusion 360 で機体に重ねて確認できる STEP（機体の STEP と同じ座標）を置いています。

| ディレクトリ | 内容 |
|---|---|
| [skirt_slider/](./skirt_slider/) | 吸引ファン用スカート枠（前後のスライダー付き）と、スカートなしで使う前後のスライダー |
| [battery_holder/](./battery_holder/) | バッテリー 2 本を上部固定基板の上に固定するホルダー |
| [jigs/maintenance_stand/](./jigs/maintenance_stand/) | 機体を載せるとタイヤが浮くメンテナンス用スタンド |
| [jigs/encoder_solder_jig/](./jigs/encoder_solder_jig/) | エンコーダ基板はんだ付け治具 |

CADツールは **Fusion 360** を使用しています。
