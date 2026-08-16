# Dyne M2: ESP32実機2台のWi-Fi直結境界同期

手順書は ../docs/2026-08-16-m2-guide.md、設計は ../docs/2026-08-16-m2-design.md。

要約: menuconfig の "Dyne M2" → "Node role" でA/Bを選び、それぞれのボードに
書き込む。Aがアクセスポイント(SSID: dyne-m2)を立て、Bが接続。2000ステップの
協調推論後、各ノードがCRC32で自己判定する(期待値はファーム内蔵)。
