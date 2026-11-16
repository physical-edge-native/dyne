# 波動方程式をノード群で解く実験の手順書(2×2・4×2・4×4)(2026-11-16)

作成: 2026-11-16(Claude Code、GitHub issue #4)。対象: 11/17・11/20 の実機実験のうち、2次元の波動方程式
(docs/2026-11-11-wave2d.md の実験10)の部分。ESP32-DevKitC-VE 16 台、専用の Wi-Fi のアクセスポイント、PC の MQTT ブローカー。
書き込み・配線・電源はユーザー、PC のコマンドは本書のとおり。

この文書は内部向け。合意済みの用語(界面・境界・格子型アクター・メールボックス方式)はそのまま使う。
略語: MQTT(PC とノードの間の小さなメッセージの配送の仕組み。ブローカー = 配送の中継役)、OTA(無線経由の書き換え)、
CRC32(32 ビットの検査値。値の列が 1 ビットでも違えば変わる)、UDP(ノードどうしの界面の値の送受信に使う通信の方式)。

---

## 0. 結論(この手順で何を確かめるか)

- 縮小版の wave2d アクター(計算部品)を全台に OTA で配り、2×2(4台)・4×2(8台)・4×4(16台)の分割で
  81×81 点・400 ステップのガウス型の山を解かせ、**各ノードの最後の状態の CRC32 が dyne-sim2d の同じ区画の値と全台一致**
  すれば合格(ビット一致)。
- PC 側の流れは新しい道具 **scripts/wave2d_cluster.py** の1コマンド(設定の作成 → 注入 → 受理の確認 → 開始 → 収集 → 照合)。
  1次元用の scripts/mqtt_config.py は 2次元の設定(field2d・faces・ifaces)を作れないので使わない。
- 同じ流れを、実機なしで、偽のノード(scripts/wave2d_fake_nodes.py)と PC 内のブローカーで通しで動かせる
  (scripts/wave2d_rehearsal.py。CI の scripts/ci-check.sh の [9])。**前日に PC で一度通しておく**(§2)。

---

## 1. 全体の流れと本書の範囲

実験の順(予定): 一括の最初の書き込み(16台)→ 時刻同期 → 全台へ OTA → **波動 2×2(mode.test_drop で損失からの
回復も確かめる)→ 4×2 → 4×4** → IMU → 実機の入力の再生。本書は太字の部分。ほかの段は次を見る:

| 段 | 文書 | 主なコマンド |
|---|---|---|
| 最初の書き込み(USB) | docs/2026-11-13-firmware-nodes.md §9.0.1 | `powershell -ExecutionPolicy Bypass -File scripts\first_flash.ps1` |
| ノード番号の重なりの直し | 同 §9.0.2 | ctl 命令 node_id |
| 時刻同期 | docs/2026-11-11-timesync-ota.md §7 | `python scripts\dyne_ntp_server.py --port 123`、`scripts\dyne_timesync.py` |
| ファームの OTA | 同 §4.4 | `scripts\dyne_ota.py fw ...` |
| 同期が止まらないことの確認(issue #29) | docs/2026-11-13-firmware-nodes.md §14.4 | 本書の §4 がその手順を兼ねる |
| IMU・再生 | docs/2026-11-13-imu-fusion.md、docs/2026-11-15-hw-replay.md | — |

波動方程式の実行には時刻同期は要らない(ステップは隣との値のやり取りでそろう)。ただし、ファームは
issue #29 の修正(同じステップの重複への応答、mode.stall_ms、mode.test_drop)を含む版であること(§3.2)。

---

## 2. 前日までの準備(PC だけ。実機不要)

以下、PowerShell でリポジトリの直下から。`$PY = ".venv\Scripts\python.exe"` とする。

### 2.1 ビルド

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build-actor-wasm.ps1      # wave2d のフル版と縮小版も作る
cargo build --release -p dyne-sim2d
```

できるもの:

| ファイル | 用途 |
|---|---|
| crates\wave2d-actor\target\wasm32-unknown-unknown\release\wave2d_actor.wasm | フル版。PC の dyne-sim2d が期待値を作るのに使う |
| crates\wave2d-actor\target-small\wasm32-unknown-unknown\release\wave2d_actor.wasm | **縮小版。ノードへ OTA で配る**(線形メモリ 64KB、区画は1辺48点まで) |
| target\release\dyne-sim2d.exe | 期待値の計算 |

### 2.2 通しの予行(偽のノード)

```powershell
& $PY scripts\wave2d_rehearsal.py
```

期待する出力(時間は手元の PC で約2分。2x2-drop30 は 30% の損失で再送が増えるので 40〜50 秒かかる):

```
cfg-before-ota   OK (exit 2, ...)
ota              OK (exit 0, ...)
status           OK (exit 0, ...)
2x2-drop30       OK (exit 0, ...)
4x2              OK (exit 0, ...)
4x4              OK (exit 0, ...)
4x2-absorb       OK (exit 0, ...)
stall-ota        OK (exit 0, ...)
stall            OK (exit 1, ...)
ALL OK
```

各段の出力は target\ci-wave2d\<段の名前>.log。各段の上に、実際に打ったコマンドが `$ ...` で出る
(本書の §3〜§6 のコマンドと同じ形。ブローカーとポートだけが違う)。

偽のノードを mosquitto に対して動かして、当日と同じ形で手で練習することもできる:

```powershell
& "C:\Program Files\mosquitto\mosquitto.exe" -p 1883                            # 窓1
& $PY scripts\wave2d_fake_nodes.py --broker 127.0.0.1 --ids 11,12,13,14         # 窓2
& $PY scripts\dyne_ota.py actor --broker 127.0.0.1 --wasm crates\wave2d-actor\target-small\wasm32-unknown-unknown\release\wave2d_actor.wasm --name wave2d_actor_small --nodes 11,12,13,14 --http-host 127.0.0.1
& $PY scripts\wave2d_cluster.py run --broker 127.0.0.1 --layout 2x2 --nodes 11,12,13,14 --test-drop 0.3
```

(2026-11-16 に、この組み合わせ(mosquitto 2 系 + paho-mqtt 2.2、および mosquitto + 標準ライブラリの MQTT)で
2×2 が PASS (bit-exact) になることを確かめた。)

---

## 3. 当日: 波動の前の共通の準備

### 3.1 PC の常駐

```powershell
# 窓1: ブローカー(既存の設定どおり。アクセスポイントの LAN 側で待ち受ける)
# 窓2: ノードの様子を見る(任意)
& $PY scripts\mqtt_observe.py --broker <PC>
```

以下 `<PC>` は PC の LAN 側のアドレス(ノードの sdkconfig.local の CONFIG_DYNE_BROKER_URI と同じ)。

### 3.2 ノードの一覧と、使うノードの決定

```powershell
& $PY scripts\wave2d_cluster.py status --broker <PC>
```

出力の例(番号・アドレスは例):

```
  id state       idx ip:port                actor (last online)    fw               up_s
  11 online       -1 192.168.10.11:7777     heat1d_actor_small     dyne-node ...    35
  ...
```

- 全台が online であること。fw(ファームの版)が issue #29 の修正を含む版であること(含まなければ先に
  `scripts\dyne_ota.py fw` で全台を更新。docs/2026-11-11-timesync-ota.md §4.4)。
- 同じ一覧が wave2d_out\nodes.json に保存される(ノードの IP の控え。§7 の「IP が分からない」を参照)。
- **使うノードの番号を、区画の順に決めて基板のラベルと一緒に記録する**。区画の順は dyne-sim2d と同じ
  「番号 = by·px + bx(x 方向が先)」。2×2 なら 左下・右下・左上・右上 の順。無線なので置き場所は計算に関係しないが、
  机の上でもこの並びに置くと、止まったときに隣を探しやすい。
  - 2×2: 4台(A1〜A4 とする)。4×2: A1〜A4 + 4台。4×4: 全 16 台。
  - 以下のコマンドの `--nodes` には、この順に番号をカンマ区切りで書く。

### 3.3 縮小版の wave2d アクターを全台へ OTA

```powershell
& $PY scripts\dyne_ota.py actor --broker <PC> --wasm crates\wave2d-actor\target-small\wasm32-unknown-unknown\release\wave2d_actor.wasm --name wave2d_actor_small --nodes all --csv wave2d_out\ota_actor.csv
```

- **合格**: 最後に `期待どおり 16/16 台`。各ノードの result が ok。ノードは書き込み後に再起動し、online の status の
  actor が `{"src":"slot0.wasm" または "slot1.wasm", "name":"wave2d_actor_small"}` になる(`wave2d_cluster.py status` で見える)。
- fallback: 新しいアクターを積めず、ファームに埋め込んだ 1次元のアクターに戻った。起動ログの `wasm load` の誤り、
  `[dyne] heap(after-wasm)` の largest を見る(線形メモリ 64KB の連続の空きが要る)。
- 波動の実験の間に再起動しても、アクターは書いた枠から積まれる(OTA のやり直しは不要)。

---

## 4. 2×2(4台)

### 4.1 損失あり(mode.test_drop = 0.3)

```powershell
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 2x2 --nodes A1,A2,A3,A4 --test-drop 0.3
```

(A1〜A4 は §3.2 で決めた番号。)道具が行うこと:

1. dyne-sim2d で期待値を作る(全域 81×81 点、値固定0の外周、400 ステップ。数秒)。
2. 4台が online か configured か done で、実行中でなく、IP が分かることを確かめる。
3. 設定(区画・面・界面の ID・相手の ip:port・mode)を dyne/cfg/<id> に retained で送り、4台の受理
   (ack の ok・gen・送った中身の CRC32 の一致)を待つ。1台でも断られたら**開始しない**(終了コード 2)。
4. 4台それぞれに start を送り、status を集める(10 秒ごとに進み具合の行。止まっていることの報告はすぐ表示)。
5. done の crc32 を期待値と比べる。

期待する出力(要点):

```
== gauss_dirichlet 2x2 (4 nodes), 400 steps; dyne-sim2d whole field crc32: split 217432b2 single 217432b2
...
all 4 acks ok (gen ..., payload CRC32 match, wire 2)
start sent to 4 nodes
...
idx node expected   got        match min/max   sent resend replay  echo stall dropped
  0   A1 0xB3E5371A 0xB3E5371A yes   yes        800    ...
  1   A2 0xCAB30D63 0xCAB30D63 yes   yes        800    ...
  2   A3 0x234F06AC 0x234F06AC yes   yes        800    ...
  3   A4 0x7B3D7C5C 0x7B3D7C5C yes   yes        800    ...
record: wave2d_out\<日時>_2x2_gauss_dirichlet_drop0.3
PASS (bit-exact): 4/4 nodes match dyne-sim2d (gauss_dirichlet 2x2, 400 steps, ... s)
```

**合格の条件**:

1. 最後の行が `PASS (bit-exact): 4/4`(4台の crc32 が docs/2026-11-11-wave2d.md §7.3 の値と一致)。
2. dropped(done の test_dropped)が 4台とも 0 でない(損失が本当に入った)。0 なら「test_drop が効いていない
   (古いファーム?)」の WARNING が出る → §3.2 の fw を確かめる。
3. echo(同じステップの重複への応答の数)と stall(止まっていることの報告の数)を記録する。stall は 0 が望ましい
   (0 でなくても最後に PASS なら、規則は働いている)。
4. min/max の列が yes(最後の field2d の要約の最小・最大が dyne-sim2d と一致。QoS 0 なので取りこぼすと `-`)。

時間の見込み: 1ステップの計算は約 27 ms(bench-c5、1台)。損失 30% では、界面の値が届かずに再送を待つ
(100 ms)ステップが多くなり、偽のノードでは 400 ステップに約 45 秒かかった。実機では1〜2分を見込む。
`--timeout`(既定 600 秒)を超えたら FAIL にする。

### 4.2 損失なし(従来どおり終わること)

```powershell
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 2x2 --nodes A1,A2,A3,A4
```

合格: `PASS (bit-exact): 4/4`。crc32 は 4.1 と同じ値(損失は通信の計数だけを変え、計算の結果は変えない)。
resend・echo は 0 か少数。かかった時間を記録する(1ステップの実時間 = 時間 / 400。10 秒ごとの rt の step_ms も
wave2d_out\<実行>\events.jsonl に残る)。

### 4.3 吸収境界(任意。時間があれば)

```powershell
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 2x2 --nodes A1,A2,A3,A4 --scene gauss_absorb
```

合格: `PASS (bit-exact): 4/4`(期待値は §7.3 の吸収境界の表: 0x41B5CB48, 0x78C7037B, 0x430A079E, 0xC67B53C5)。

### 4.4 止まっていることの報告を見る(docs/2026-11-13-firmware-nodes.md §14.4 の 5.)

```powershell
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 2x2 --nodes A1,A2,A3,A4 --steps 4000 --timeout 120 --stop-on-fail
```

(4000 ステップにして実行を長くする。期待値は道具がその場で作る。)走り始めて 10 秒ほどたったら、**A2 の電源を切る**。
期待する出力:

```
STALL  node A1 (idx 0) at step ..., waited 10003 ms, missing: iface 0 from idx 1 (node A2)  [...]
STALL  node A4 (idx 3) at step ..., waited 10004 ms, missing: iface 3 from idx 1 (node A2)  [...]
...(10 秒ごとに繰り返す)
FAIL: 0/4 nodes match dyne-sim2d; timeout after 120 s (...)
sent stop to the nodes that did not finish
```

合格: A2 の隣(A1・A4)が、A2 からの界面を missing に挙げた報告を 10 秒ごとに出す。再起動しない。最後の stop で
A1・A3・A4 の status が `configured`(note stopped)になる。A2 の status は offline(遺言)になる。
終わったら A2 の電源を入れ、online に戻ったことを `status` で見る。

### 4.5 2×2 の記録

各実行の wave2d_out\<日時>_<分割>_<場面>\ に、expected.json(期待値)、cfg_node<id>.json(送った設定)、
result.json・result.csv(判定と計数)、events.jsonl(受けた MQTT のメッセージの全部)が残る。§8 の注意を守って
measured\ へ写す。

---

## 5. 4×2(8台)

```powershell
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 4x2 --nodes B0,B1,B2,B3,B4,B5,B6,B7 --test-drop 0.3
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 4x2 --nodes B0,B1,B2,B3,B4,B5,B6,B7
```

- 区画の順: 下の段の左から右へ B0〜B3、上の段の左から右へ B4〜B7。
- 合格: どちらも `PASS (bit-exact): 8/8`。期待値(値固定0): 0xED8D8312, 0x4B0A73D2, 0xDCCAB468, 0x4D061237,
  0xC9BC3546, 0x4E057FA6, 0x58AF8F33, 0x499E65E5。損失ありでは dropped が全台で 0 でないこと。
- 2×2 で使った 4 台を含めてよい。2×2 の後は retained の status が done になり IP が載っていないが、道具は
  wave2d_out\nodes.json の控えを使う。

## 6. 4×4(16台)

```powershell
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 4x4 --nodes C0,C1,...,C15 --test-drop 0.3
& $PY scripts\wave2d_cluster.py run --broker <PC> --layout 4x4 --nodes C0,C1,...,C15
```

- 区画の順: 下の段から上の段へ、各段は左から右(C0〜C3 が最下段)。
- 合格: どちらも `PASS (bit-exact): 16/16`。期待値(値固定0): docs/2026-11-11-wave2d.md §7.3 の 16 台の表
  (0xC03FE916 … 0x2378832B)。道具の表の expected の列にも出る。
- 16 台が同時に Wi-Fi で送るので、損失なしでも resend が増えうる。計数を記録する。

---

## 7. 失敗したとき

| 症状(道具の出力) | 考えられる原因 | すること |
|---|---|---|
| `NOT STARTED: ... has no status` | ノードがブローカーにつながっていない | 電源・Wi-Fi・ブローカーのアドレス(sdkconfig.local)。シリアルのログ |
| `NOT STARTED: ... is offline` | 電源が落ちた・Wi-Fi が切れた(遺言) | 電源を入れ直し、`status` で online を確かめる |
| `NOT STARTED: ... is still running` | 前の実行が終わっていない | `wave2d_cluster.py stop --broker <PC> --nodes ...` の後にやり直す |
| `NOT STARTED: ... IP unknown` | retained の status が online でなく(configured・done などは IP を載せない)、控え(nodes.json)にもない | `--reboot-missing` を付けて再実行(そのノードを再起動して online を出させる)。または ctl 命令 reboot |
| `configuration rejected: cluster/node/bc missing` と `OTA the wave2d actor first` | ノードが 1次元のアクター(埋め込みの heat1d)のまま。2次元の設定を 1次元の形として読んで断った | §3.3 の OTA。fallback なら起動ログを見る |
| `configuration rejected: iface/face mismatch`・`bad ...` | 設定の作り方の誤り(道具の不具合) | cfg_node<id>.json を保存して報告(偽のノードでも同じ断り方をするので、予行で再現を試す) |
| `no ack within 10 s` | cfg が届いていない・ノードが固まった | `--verbose` で MQTT を全部表示して再実行。シリアルのログ |
| `STALL node X ... missing: iface I from idx J (node Y)` が出続ける | Y が止まった・電源が落ちた・Y への UDP が届かない | Y の status(offline か、running で step が進んでいないか)。Y の電源・Wi-Fi。全台 stop して再実行 |
| STALL が出た後 `STALL cleared` | 損失が重なって一時的に待った(issue #29 の規則で回復) | 正常。計数を記録 |
| `timeout after ... s` で done が揃わない | 遅い・止まった | 進み具合の行(steps reported)で止まったノードを探す。`--timeout` を延ばす。`--stop-on-fail` で止めて再実行 |
| done したが `match NO` | 計算の結果が違う(ビット一致しない) | 下の「ハッシュが違うとき」 |
| `WARNING: ... no node reports test_dropped > 0` | ファームが古い(mode.test_drop を知らない) | §3.2 の fw。ファームを OTA |
| `ERROR node X: {"state":"err","err":"v2: ..."}` | ファームの実行の誤り(inbox rejected など) | 誤りの文を記録し、シリアルのログを取る |

**ハッシュが違うとき**(docs/2026-11-11-wave2d.md §7.2 の 3.):

1. 一部のノードだけ違う → そのノードの区画(expected.json)と、隣との界面を疑う。events.jsonl の field2d の要約
   (100 ステップごと)を、dyne-sim2d の同じ時点と比べると、どのステップから違うかが分かる。
   シミュレータの途中の値は `wave2d_cluster.py expected --layout 2x2 --steps 100 --out e100.json` で作れる
   (100・200・300 ステップの値を作り、field2d の要約の crc32 と比べる)。
2. 全ノードが違う → 設定の数値(h = 0.0125、dt = 0.00625)の読み取り(ファームの strtod の丸め)を疑う。
   最初のステップから違うはずなので、`--steps 1` の実行で確かめる(期待値はその場で作られる)。
3. 損失ありだけ違い、損失なしは合う → 同期の受信の規則(取り置き・応答)の不具合。events.jsonl と計数を保存して報告。

**途中で止めるとき**: `& $PY scripts\wave2d_cluster.py stop --broker <PC> --nodes all`(全台に stop。走っていない
ノードは何もしない)。

**mode.test_drop の使い方**: 0 以上 1 未満(ノードが待ちの間に受けた UDP のデータグラムをこの割合で捨てる。
dyne-sim2d の `--loss` に当たる)。1 以上は設定を断られる(`mode.test_drop: 0 <= p < 1`)。0.3 で輪の行き詰まり
(issue #26・#29)が起きやすくなる。本番の計測(1ステップの時間)は 0 で行う。

---

## 8. 記録の扱い(公開リポジトリ)

- wave2d_out\ は git 管理外。残す記録は measured\ へ写す。
- **events.jsonl と nodes.json には実在の MAC アドレス(online の status の mac)と、アクセスポイントの LAN の
  IP アドレスが入る**。measured\ へ写すときは、MAC を既存の規則どおり仮の値(02:00:00:00:…)に置き換える
  (CLAUDE.md「公開と非公開」)。result.csv・result.json・expected.json・cfg_node<id>.json に MAC は無い
  (IP は入る。アクセスポイントの内側の私的なアドレスなので、置き換えるかはユーザーが決める)。

---

## 9. 作ったもの(GitHub issue #4)

| ファイル | 内容 |
|---|---|
| scripts/wave2d_cluster.py(新規) | 2次元の流れの道具: status / expected / run / stop |
| scripts/wave2d_fake_nodes.py(新規) | 偽のノード。ファームと同じ MQTT のトピックと JSON、同じ同期の規則(UDP、127.0.0.1)、wave2d アクターの Python への移植(libm の exp を含む) |
| scripts/wave2d_rehearsal.py(新規) | 本書の流れを偽のノードで通す予行。CI の [9] |
| scripts/dyne_mqtt_lite.py(新規) | 標準ライブラリだけの MQTT(paho-mqtt の使っている範囲の互換のクライアントと、試験用のブローカー) |
| scripts/wave2d_expected.py | 関数を分けた(blocks・expected_nodes・node_cfg)。出力は従来とバイト一致を確認。dyne-sim2d の既定のパスを OS に合わせた |
| scripts/dyne_ota.py | paho-mqtt が無ければ dyne_mqtt_lite を使う(CI で偽のノードに配るため) |
| scripts/ci-check.sh, .github/workflows/ci.yml | [9] を追加(wave2d のフル版・縮小版のビルドと予行)。キャッシュに crates/wave2d-actor |
| .gitignore | wave2d_out/ |

### 9.1 偽のノードの仕組み

- 各偽のノードは MQTT のクライアントとして dyne/cfg/<id>・dyne/ctl/<id>・dyne/ctl/all を購読し、online・configured・
  running(step・stall・stall_cleared)・done(crc32 と計数)・err を dyne/node/<id>/status に retained で出す
  (遺言は offline)。設定の検査と誤りの文は node_v2.c の parse_cfg_v2 と同じ。
- 最初は埋め込みの 1次元のアクターの状態で、2次元の設定を `cluster/node/bc missing` で断る(実機と同じ)。
  ota_actor 命令で .wasm を HTTP で受け取り、大きさと SHA-256 を確かめ、メールボックス方式の差込口の名前を含めば
  「再起動」して online の actor を更新し、ok を返す(.wasm を実行はしない)。
- 計算は crates/wave2d-actor の Python への移植(同じ演算を同じ順に。初期条件の exp は Rust の libm 0.2 と同じ手順を
  移植)。Python の浮動小数点は IEEE の倍精度で、積和の融合をしないので、WASM と同じビットになる。予行で
  2×2・4×2・4×4・吸収境界のすべてで dyne-sim2d と一致した(これ自体が、区画の作り方と界面の受け渡しの独立な検算になる)。
- 隣との値は 127.0.0.1 の UDP(偽の形のデータグラム。ワイヤ形式v2 の外箱ではない)で、node_v2.c と同じ同期の規則
  (直近2ステップの控え・先着分の取り置き・過去分と同じステップの重複への応答・100 ms で再送・mode.stall_ms ごとの
  報告・mode.test_drop・終了後 3 秒の応答)。実機のように、開始前に届いた分は捨てる。
- `--die ID@STEP` でそのノードをそのステップから黙らせる(電源断の代わり)。

### 9.2 小さな実装判断(事後に明記)

1. **MQTT を標準ライブラリで書いた**(scripts/dyne_mqtt_lite.py)。CI は Python のパッケージを入れず、ブローカーも
   無いため。道具は paho-mqtt があればそれを使い(当日の PC は .venv の paho)、無ければこれを使う(`--mqtt` で選べる)。
   mosquitto + paho、mosquitto + 本モジュールの両方で 2×2 が PASS になることを手元で確かめた。
2. **偽のノードの計算は Python への移植**にした。期待値の写しを返すだけの偽物では、道具の設定の誤り(区画・面・界面の ID・
   相手)を見逃すため。代わりに、アクターを変えたら移植も直す必要がある(CI の [9] が落ちて気づく)。
   ガウス型以外の初期条件と、1辺 48 点を超える区画は断る(実験の場面だけを対象にした)。
3. **start は dyne/ctl/all ではなく、実行に使うノードへ1台ずつ送る**。前の実行の retained の設定を持った、実行に
   入っていないノードが動き出さないように。同期は隣とのやり取りでそろうので、送る時刻の差(ミリ秒)は問題にならない。
4. **ノードの IP の控え(wave2d_out\nodes.json)**。online 以外の retained の status は IP を載せないため、2回目以降の
   実行で IP が分からなくなる(mqtt_config.py の --peers auto と同じ問題)。online を見るたびに控えを更新する。
5. **アクターの名前の確かめは警告だけ**にした。名前は online の status にしか載らないので、本当の確かめは
   設定の受理(1次元のアクターは断る)で行う。
6. 偽のノードの online の status に、実機に無い `udp_port` を足した(1台の PC で複数のノードが別のポートを使うため)。
   道具は無ければ 7777(CONFIG_DYNE_UDP_PORT)を使う。
7. 期待値は実行のたびに dyne-sim2d で作る(数秒)。PC に dyne-sim2d が無い場合は `expected` で作った JSON を
   `--expected` で渡せる。
8. 時間切れでも既定では stop を送らない(様子を見られるように)。`--stop-on-fail` で送る。

### 9.3 確認したこと・未確認のこと

- 予行(scripts/wave2d_rehearsal.py)は 9 段すべて OK(Windows、Python 3.14)。`bash scripts/ci-check.sh` は ALL CHECKS PASSED。
- **実機では未確認**。GitHub issue #4 の完了の条件の「実機で少なくとも 2×2 のハッシュが一致」は 11/17・11/20 の実験で確かめる。
- 実機に固有で偽のノードでは確かめられないこと: ワイヤ形式v2 の外箱(通番・CRC)、設定の数値の strtod の丸め、
  Wi-Fi の損失と遅れ、16 台が同時に送るときの混雑、ESP-IDF の MQTT の振る舞い。
