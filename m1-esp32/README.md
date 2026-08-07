# Dyne M1: ESP32実機でのWASMアクター実行

手順書は ../docs/2026-08-05-m1-guide.md を参照。

要約: ESP-IDF (v5.1以降) 環境で
  idf.py set-target esp32
  idf.py build
  idf.py -p COMx flash monitor
を実行すると、M0のheat1d_actor.wasm (無変更) をWAMRで2000ステップ実行し、
「M1 REPORT」として RAM・フラッシュ・ステップ時間・誤差を表示する。
