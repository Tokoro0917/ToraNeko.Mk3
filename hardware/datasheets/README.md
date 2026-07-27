# hardware/datasheets/

使用部品のデータシートPDFを格納する場所です。リンク切れやメーカーサイトの仕様変更に備えて、
実際に使う部品のデータシートはPDFの実体をここに置いておきます。

## アップロード方法

1. メーカーまたは販売代理店（DigiKey/Mouser/RSなど）のサイトからPDFをダウンロードする
2. ファイル名は `<型番>.pdf` とする（例: `NCR321PAS.pdf`）
3. GitHubのWeb UIでこのディレクトリにドラッグ＆ドロップでアップロード、またはローカルにcloneしてpushする

## 部品一覧

| 型番 | 用途 | メーカー | 参考リンク | PDF |
|---|---|---|---|---|
| STM32F411CEU6 | MCU | STMicroelectronics | https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html | 未アップロード |
| MP6551 | モータドライバ | Monolithic Power Systems | https://www.monolithicpower.com/en/mp6551.html | 未アップロード |
| NFP-D0812-1-3.7 | ブラシ付きDCモータ | - | - | 未アップロード |
| LSM6DSRXTR | IMU（ジャイロ/加速度） | STMicroelectronics | https://www.st.com/en/mems-and-sensors/lsm6dsrx.html | 未アップロード |
| AS5047P | 磁気式エンコーダ | ams OSRAM | https://ams-osram.com/products/sensor-solutions/position-sensors/ams-as5047p-high-resolution-position-sensor | 未アップロード |
| LTR-209 | 壁センサ受光（フォトトランジスタ） | Lite-On | - | 未アップロード |
| OSI5FU3A11C | 壁センサ発光（IR LED） | Optosupply | https://www.optosupply.com/uppic/20241121109582.pdf | 未アップロード |
| NCR321PAS | 壁センサLEDドライバ（定電流） | Nexperia | https://assets.nexperia.com/documents/data-sheet/NCR320PAS_NCR321PAS.pdf | 未アップロード |
| LXDC55FAAA-203 | 3.3Vレギュレータ | - | - | 未アップロード |

「参考リンク」が空欄の部品は型番からの検索が必要です。分かる方はこのテーブルに追記してください。

## 備考

- この実行環境（Claude Code on the web）はアウトバウンド通信が許可ドメインのみに制限されているため、
  Claude自身がこれらのPDFを取得してアップロードすることはできません。人手でのアップロードが必要です。
