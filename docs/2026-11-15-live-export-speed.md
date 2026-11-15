# 常駐版の書き出しを速くした(GitHub issue #3)(docs\2026-11-15-live-export-speed.md)

2026-11-15。Claude Code(この環境)。docs/2026-11-13-pc-pipeline-rl.md §5(常駐版 G5)・§11(書き出し 1 回 3.5〜6.5 秒)の続き。
コミットはしていない。

## 1. 何を変えたか

| ファイル | 変更 |
|---|---|
| scripts/twin_demo.py | main() を build_parser() と export(a, cache, quiet) に分けた(コマンドとしての使い方・出力は従来どおり)。形(φ・w)に依らない部分(場面ファイルの読み取り・正解の面の標本・温度のダミーとその表示用の要素・USD の温度の配列)を ExportCache で使い回せるようにした(場面ファイルの更新時刻・大きさ、格子の範囲、関係する引数を鍵にし、変われば作り直す)。`--no-html` を追加。JSON の書き出しを json.dump(ファイル)から json.dumps(C の符号化器)に変えた(出力のバイト列は同じ) |
| scripts/twin_live.py | 書き出しのたびに twin_demo.py を起動するのをやめ、同じプロセスで export() を呼ぶ(pxr の読み込みは起動時に 1 回)。HTML は既定で最初の 1 回だけ(`--html once`。ダッシュボードは `?poll` で room.json を読み直すので足りる)。以前の動作は `--subprocess`(HTML も毎回)、同じプロセスで毎回すべて作り直すのは `--no-cache --html every`。status.json に exportMode を追加 |

温度のダミーは USD にも入っている(/Room/TemperatureField/Floor)ので、書き出しから外すのではなく「場面ごとに 1 回だけ解いて使い回す」にした
(同じ入力で USD が変わらないことを優先した小さな実装判断)。JSON(ダッシュボードの自動更新が読む)は毎回書く。

## 2. 書き出し 1 回の時間(偽のノード、room6_move・4×4 ノード・格子 73×55、--every 5、70 秒)

| | 1 回目 | 2 回目以降 |
|---|---|---|
| 以前(毎回 twin_demo.py を起動) | 5.73 秒 | 3.27〜3.53 秒(13 回) |
| いま(同じプロセス・使い回し・HTML は 1 回目だけ) | 2.51 秒(温度のダミーを解く + HTML) | 0.43〜0.55 秒(13 回) |

同じ PC で続けて測った(他の担当の作業も同時に動いていたので、絶対値は ±数割ゆれる。別の時刻の測定では以前の方式が 1 回 2.8〜6.5 秒)。
残りの約 0.45 秒の大半は pxr で箱(約 300 個)の衝突判定を組み立てて書く時間と JSON の書き出し。

## 3. 出力が変わらないことの確かめ

- 記録した 5 つの入力(ステップ 279 と 599)を、以前の twin_demo.py(別プロセス)・新しい twin_demo.py(別プロセス)・
  新しい export()(同じプロセス、使い回しあり。HTML あり/なし)で書き出し、room.usda・room.xml・room.json・room.html が
  **すべてバイト単位で一致**。
- 常駐版(同じプロセス)の 14 回の書き出し(frames/)を、以前の twin_demo.py で同じ入力・通し番号・時刻で書き出し直して比べ、
  room.usda・room.xml は**バイト単位で一致**、room.json は記録された出力先のパスを除いて一致(14/14)。
- 最後の latest/room.usda の usd_check.py: PASS(箱の基本形 310 個、問題 0、UsdValidation の指摘 0 件)。

## 4. 限界

- 遅れの残り: センサを振る周期(約 4.3 秒)とノードの送る間隔(1 秒)、書き出しの間隔(`--every`、既定 5 秒)は PC 側では消せない
  (`--every` を短くするのは可能。書き出しが 0.5 秒になったので 1〜2 秒も取れる)。
- 1 回目は温度のダミーを解くので 2〜3 秒かかる。
- `--html once` では latest/room.html に埋め込まれたデータは 1 回目のもの(`?poll` なしで開くと古い表示)。

## 5. 実行方法

```
powershell -ExecutionPolicy Bypass -File scripts\twin_live_demo.ps1 -Seconds 60     # 既定(同じプロセス)
.venv\Scripts\python.exe scripts\twin_live.py ... --subprocess                       # 以前の動作
.venv\Scripts\python.exe scripts\usd_check.py demo_out\twin_live\latest\room.usda --scene crates\dyne-sim2d\scenes\room6_move.txt
```
