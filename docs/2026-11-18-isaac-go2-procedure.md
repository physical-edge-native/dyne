# Isaac Lab で部屋の USD に Unitree Go2 を歩かせる手順(GPU の計算機・Linux)— 2026-11-18

Issue #93。11/22 の発表で、部屋のデジタルツインの中で 4 脚ロボット(Unitree Go2)が強化学習の方策で
動く様子を見せるための手順。GPU の計算機(Linux、Isaac Sim は導入済み、Isaac Lab は未確認)でユーザーが行う。

**この文書の内容は、この PC(GPU なし)では Isaac を一度も動かしていない。** 確認済みと未確認を各所に書く。
どの案で進めるか・どこまでやるかはユーザーが決める(§1)。

## 0. 全体の流れと時間の目安

| 段階 | やること | 時間の目安(未確認。初回の起動が遅い) |
|---|---|---|
| 1 | 計算機の確認(GPU・ドライバ・OS) | 5 分 |
| 2 | Isaac Sim の版を調べる | 5 分 |
| 3 | Isaac Lab があるか調べ、無ければ版を合わせて入れる | 20〜60 分 |
| 4 | Isaac Lab の動作確認と課題の一覧 | 10〜20 分(初回はシェーダの準備で長い) |
| 5 | 部屋の USD の取り込みの確認(円柱を壁へ押す) | 5〜10 分 |
| 6 | Go2 の歩行の方策を用意(配布の学習済み、無ければ学習) | 5 分〜1 時間 |
| 7 | 部屋の中で Go2 を動かし、画面・動画・記録を残す | 15〜30 分 |
| 8 | (11/20 以降)本物の部屋の USD で 7 をやり直す | 15 分 |

まずダミーの部屋(シミュレータで作った 6 畳の部屋 room6)で 1〜7 を通し、11/20 に作る部屋で 8 を行う(Issue #93 の順)。

## 1. 案の比較(ユーザーが選ぶ)

Go2 の歩行そのものは、Isaac Lab 付属の課題(速度の指令に従って歩く。§6・§8)で学習した方策を使う。
違いは「目標へ向かう判断」を何で行うか。

| 案 | 中身 | 準備の量 | 見せられること |
|---|---|---|---|
| **A(推奨。台本あり)** | 歩行は学習した方策、目標へ向かう判断は単純な規則(目標の方向へ向き、近いほど遅く)。障害物は避けない代わりに、直線で通れる目標の点だけを自動で選ぶ | 学習 1 回(配布の学習済みがあれば 0 回) | 部屋の中を Go2 が目標の点をめぐって歩く。壁・家具の衝突判定が効いている |
| B | Isaac Lab の ANYmal-C(別の 4 脚ロボット)の「目標の位置へ行く」課題と同じ作りで、上の層も強化学習にする。下の層に Go2 の歩行の方策を入れる | Go2 用の設定を新しく書く(台本なし)、学習 2 回。家具を避けるには障害物の観測(距離の光線など)も足す必要がある | 「目標へ行く判断も学習した」と言える |
| C | 歩くだけ(目標なし) | A の台本で `--goals` に 1 点だけ渡せばほぼ同じ | 部屋の床の上で歩ける |

推奨は A。理由: 発表まで 4 日で、GPU の計算機での試行の回数が限られる。A は Isaac Lab 付属の課題をそのまま使い、
部屋の取り込みと規則の部分だけが新しい。B は A が動いた後に足せる(下の層が同じ)。
B の作り方の要点は §10 に書いた。

## 2. 持っていくもの

この PC で作って確かめたもの:

- `scripts/isaaclab_go2_room.py` … 今回の台本(案 A)。Isaac なしの確認 `--check-only` はこの PC で通った(§9)
- `scripts/isaaclab_room.py` … 部屋の USD の取り込みの確認(円柱を壁へ押す。2026-11-11 作、未実行)
- `demo_out/twin/room.usda` … ダミーの部屋の USD(git 管理外。§9 の手順でこの PC で作る)。
  比較用に `room_truth.usda`(正解の形)も持っていく

運び方(どちらか):

- (a) USB メモリ: 上の 3 つ(+ room_truth.usda)を、GPU の計算機の `~/dyne/` の下に同じフォルダの形で置く
  (`~/dyne/scripts/…`、`~/dyne/demo_out/twin/room.usda`)
- (b) リポジトリ(公開)を clone し、USD だけ USB で運ぶ:
  ```bash
  cd ~ && git clone https://github.com/physical-edge-native/dyne.git && cd dyne
  git checkout 93-isaac-go2-room   # PR がマージされる前。マージ後は main のまま
  mkdir -p demo_out/twin           # ここへ room.usda を置く
  ```

以下では `~/dyne` に置いたとして書く。

## 3. 段階 1: 計算機の確認

```bash
nvidia-smi                 # GPU の機種、VRAM(Memory)、ドライバの版(Driver Version)
lsb_release -a             # Ubuntu の版
free -h                    # メモリ
df -h ~                    # 空き容量(Isaac Lab と資産で数十 GB 使うことがある)
```

基準(Isaac Sim 5.1 の公式の要件): Ubuntu 22.04 / 24.04、GPU は GeForce RTX 4080(VRAM 16 GB)以上が「最低」、
メモリ 32 GB 以上、ドライバ 580.65.06 以上 [S5]。RT コアの無い GPU(A100・H100)は不可 [S5]。
これより小さい RTX の GPU でも、画面なし(`--headless`)で環境の数を減らせば動くことが多いと言われるが、**未確認**。

**記録**: `nvidia-smi` の出力をそのまま保存する(`nvidia-smi > ~/go2_record/nvidia-smi.txt`)。

## 4. 段階 2: Isaac Sim の版を調べる

Isaac Sim の入れ方によって調べ方が違う。

1. 置き場所を探す:
   ```bash
   ls -d ~/isaacsim ~/isaac-sim* ~/.local/share/ov/pkg/isaac* 2>/dev/null
   find ~ /opt -maxdepth 4 -name isaac-sim.sh 2>/dev/null
   ```
2. 見つかったフォルダ(以下 `ISAACSIM_PATH`)の版のファイルを読む:
   ```bash
   export ISAACSIM_PATH="$HOME/isaacsim"      # 1 で見つかった場所に書き換える
   cat ${ISAACSIM_PATH}/VERSION               # 例: 5.1.0-… (ファイル名・中身の形は要確認)
   ```
   VERSION が無いときは、フォルダ名(`isaac-sim-4.5.0` など)か、Isaac Sim を起動して Help → About で見る。
3. pip で入れた場合(フォルダが見つからない):
   ```bash
   conda env list ; ls ~/*/bin/activate 2>/dev/null     # Isaac Sim を入れた仮想環境を探す
   source <仮想環境>/bin/activate                         # conda なら conda activate <名前>
   pip show isaacsim | head -3                            # Version: が Isaac Sim の版
   ```

**記録**: 版(例 5.1.0)と、入れ方(フォルダか pip か)と置き場所。

## 5. 段階 3: Isaac Lab の版を合わせる

### 5.1 Isaac Sim の版 → 使う Isaac Lab の版

| Isaac Sim | Isaac Lab | 根拠 |
|---|---|---|
| 5.1 | **2.3.2**(推奨。今回の台本の前提) | 2.3.0 は「Isaac Sim 5.1 の上に作った」[S2]。2.3.x の文書は 5.1 を推奨、4.2 以下は非対応 [S3] |
| 5.0 | 2.2.x | 2.2.0 は 5.0 に完全対応、4.5 とも後方互換 [S4] |
| 4.5 | 2.2.x または 2.1.x | 2.2.0 の後方互換 [S4]。2.1 と 4.5 の組は記憶による(**未確認**) |
| 4.2 以下 | (使えない) | Isaac Lab 2.3 は 4.2 以下に非対応 [S3]。Isaac Sim を入れ直す |
| 6.x | 3.0.0 | 3.0.0 は Isaac Sim 6.2、Python 3.12、5.1 以下は非対応 [S1][S6]。**台本は 2.x 向けなので直しが要る見込み**(§5.4) |

Python の版は Isaac Sim に合わせる(5.x は 3.11、4.x は 3.10)[S3]。

### 5.2 Isaac Lab がもう入っているか

```bash
find ~ -maxdepth 3 -name isaaclab.sh 2>/dev/null       # 見つかったフォルダが Isaac Lab
cd <見つかったフォルダ> && git describe --tags && cat VERSION
ls -l _isaac_sim                                        # どの Isaac Sim につないでいるか(フォルダ版の場合)
```

版が §5.1 の表と合っていれば 5.3 は飛ばして段階 4 へ。合わないとき・無いときは 5.3。
既存の Isaac Lab を他の人が使っている場合は、上書きせず別のフォルダに入れる(ユーザーが判断)。

### 5.3 入れる(Isaac Sim 5.1 のフォルダ版の場合。公式の手順 [S7] のとおり)

```bash
export ISAACSIM_PATH="$HOME/isaacsim"                   # 段階 2 で見つけた場所
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')"   # この 1 行が出れば Isaac Sim は動く

cd ~ && git clone --branch v2.3.2 https://github.com/isaac-sim/IsaacLab.git IsaacLab-2.3.2
cd ~/IsaacLab-2.3.2
ln -s ${ISAACSIM_PATH} _isaac_sim
sudo apt install cmake build-essential
./isaaclab.sh --install rsl_rl                          # 学習の道具 RSL-RL だけ入れる(全部なら --install)
```

- Isaac Sim 5.0 / 4.5 なら §5.1 の表の版の最新のタグ(`git ls-remote --tags https://github.com/isaac-sim/IsaacLab.git | grep v2.2` で確かめる)を
  `--branch` に渡す。
- Isaac Sim を pip で入れた場合: その仮想環境を有効にしてから、`_isaac_sim` のリンクを作らずに `./isaaclab.sh --install rsl_rl`
  (公式の pip の手順 [S8] による。**この組み合わせは未確認**)。
- 以前に別の版の Isaac Sim を使っていた場合は、最初に一度 `${ISAACSIM_PATH}/isaac-sim.sh --reset-user` [S7]。

### 5.4 Isaac Sim が 6.x だった場合

Isaac Lab 3.0 は入れ方(`uv`)・課題の名前(`-v0` が付かない。例 `Isaac-Velocity-Flat-UnitreeGo2`)・起動の仕方
(`AppLauncher` が限定的な互換だけになった)・学習と再生のコマンド(`isaaclab train` / `isaaclab play`、
`--checkpoint pretrained`)が変わっている [S1][S9]。今回の台本(`isaaclab_go2_room.py`)は 2.x 向けで、3.0 で動くかは
**未確認(直しが要る見込みが高い)**。選択肢:

- (i) Isaac Sim 5.1 を pip で別の仮想環境に入れ、Isaac Lab 2.3.2 を使う(既存の 6.x はそのまま)。
- (ii) 3.0 のまま進め、エラーの出力を持ち帰って台本を 3.0 向けに直す(Claude Code に頼む)。
- 配布の学習済みの再生だけなら 3.0 でも: `./isaaclab.sh play --rl_library rsl_rl --task Isaac-Velocity-Rough-UnitreeGo2 --num_envs 4 --checkpoint pretrained --viz kit`
  (第三者の解説ページにあったコマンド [S10]。**未確認**)。

## 6. 段階 4: Isaac Lab の動作確認と Go2 の課題

```bash
cd ~/IsaacLab-2.3.2
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py          # 黒い画面の窓が出れば OK。Ctrl+C で終わる
./isaaclab.sh -p scripts/environments/list_envs.py | grep -i go2
```

2 行目の期待する出力(2.3.2 の登録 [S11] による): `Isaac-Velocity-Flat-Unitree-Go2-v0`、`Isaac-Velocity-Flat-Unitree-Go2-Play-v0`、
`Isaac-Velocity-Rough-Unitree-Go2-v0`、`Isaac-Velocity-Rough-Unitree-Go2-Play-v0`。

- 初回の起動はシェーダの準備などで 5〜20 分かかることがある(止まって見えても待つ)。
- Go2 の模型・床の材質はインターネット上の NVIDIA の置き場から取ってくる。社内のネットワークで止まるときは §11。

## 7. 段階 5: 部屋の USD の取り込みの確認

```bash
cd ~/IsaacLab-2.3.2
./isaaclab.sh -p ~/dyne/scripts/isaaclab_room.py --usd ~/dyne/demo_out/twin/room.usda
```

期待する出力の最後の行: 円柱の最後の x ≈ 2.45 m(ベッドの手前で止まる。MuJoCo では 2.451 m)、z ≈ 0.10 m。
x が 3 m を超えたら衝突判定が効いていない(§11)。**スクリーンショット 1 枚**(部屋の全体と止まった円柱)。

## 8. 段階 6: Go2 の歩行の方策を用意する

使うのは平らな床の課題(`Flat`)。部屋の床は平らで、でこぼこの課題(`Rough`)の方策は床の高さの走査を観測に使うため、
部屋の中では使わない。

### 8.1 配布の学習済みを試す(数分)

```bash
cd ~/IsaacLab-2.3.2
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-Play-v0 --num_envs 16 --use_pretrained_checkpoint
```

- `A pre-trained checkpoint is currently unavailable for this task.` と出たら、この課題には配布が無い → 8.2。
  (Go2 の平らな課題に配布があるかは**未確認**。でこぼこの課題で使った例はある [S10]。オプションの使い方は [S12]。)
- 動いたら: 平らな床に 16 頭の Go2 が並び、矢印(指令の速度)の方へ歩く。**スクリーンショット 1 枚**。
- 方策のファイル: `~/IsaacLab-2.3.2/.pretrained_checkpoints/rsl_rl/Isaac-Velocity-Flat-Unitree-Go2-v0/exported/policy.pt`
  (play.py が再生のときに `exported/policy.pt` を書き出す [S13]。置き場所は台本の読みから。`find . -name policy.pt` で確かめる)

### 8.2 自分で学習する(配布が無いとき)

```bash
cd ~/IsaacLab-2.3.2
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 --headless 2>&1 | tee ~/go2_record/train.log
```

- 設定: 4096 環境、300 回の更新(反復)[S11]。時間は GPU 次第で、RTX 4080 級で 10〜30 分ほどと見込む(**未確認**)。
  最初の数回の反復の時間(`Iteration time` の行)× 300 で見積もれる。
- 途中の様子: `Mean reward` が増え、`Episode_Termination/base_contact`(転倒)が減っていけばよい。
- 終わったら再生して方策を書き出す(最新の学習を自動で読む):
  ```bash
  ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Flat-Unitree-Go2-Play-v0 --num_envs 16
  ```
  方策のファイル: `~/IsaacLab-2.3.2/logs/rsl_rl/unitree_go2_flat/<日時>/exported/policy.pt`
- **記録**: train.log、学習にかかった時間、GPU の機種(発表で「GPU 1 台で何分」と言えるように)。

## 9. 段階 7: 部屋の中で Go2 を動かす(案 A)

### 9.1 この PC で済ませたこと(確認済み、2026-11-18)

ダミーの部屋の USD を作って検査した(リポジトリの .venv):

```powershell
# demo_out\twin\ に shape.csv・room.usda などを作る(cargo と .venv が要る)
powershell -ExecutionPolicy Bypass -File scripts\twin_demo.ps1
# 台本の Isaac なしの確認
.venv\Scripts\python.exe scripts\isaaclab_go2_room.py --usd demo_out\twin\room.usda --check-only
```

結果: `usd_check.py` は PASS(衝突判定 91 個、正解との外接箱の差は最大 23.4 mm、許容 30 mm)。
`--check-only` の出力(抜粋):

```
[dyne] 障害物の点 26.9%、Go2 の中心を置ける点(障害物から 0.45 m 以上)894 点 = 2.24 m²、最も広い点の余裕 1.05 m
[dyne] 出発点 (1.53, 1.53) 余裕 1.05 m
[dyne] 目標 1 (2.03, 0.58) 余裕 0.58 m、前の点からの直線 1.07 m 通れる
[dyne] 目標 2 (0.58, 1.78) 余裕 0.58 m、前の点からの直線 1.88 m 通れる
[dyne] 目標 3 (2.08, 2.13) 余裕 0.56 m、前の点からの直線 1.54 m 通れる
[dyne] PASS: Go2 を置いて目標の点 3 個をめぐれる形(…)
```

6 畳(3.6 m × 2.7 m)は Go2(体の長さ約 0.7 m)には狭く、置ける場所は約 2.2 m²。未観測の場所(測距が届かなかった所)は
物とみなす箱になっている(`twin_demo.py` の既定)。

### 9.2 GPU の計算機で実行【未検証】

```bash
mkdir -p ~/go2_record
cd ~/IsaacLab-2.3.2
find . -path '*exported/policy.pt'        # 段階 6 の方策のファイルを探す(Go2 の Flat のものを選ぶ)
POLICY=$PWD/<上で選んだパス>                # 例 $PWD/logs/rsl_rl/unitree_go2_flat/2026-11-19_10-00-00/exported/policy.pt
./isaaclab.sh -p ~/dyne/scripts/isaaclab_go2_room.py --usd ~/dyne/demo_out/twin/room.usda \
    --policy $POLICY --seconds 60 --out ~/go2_record/dummy 2>&1 | tee ~/go2_record/dummy_run.log
```

期待する出力:

- 最初に 9.1 と同じ確認の行(`[dyne] PASS: …`)。
- `[dyne] 環境 1 個、1 手 0.020 s、3000 手。…`
- 1 秒ごとに `[dyne] t=… 環境 0: 位置 (…)、目標 k まで … m`、着くたびに `目標 k に到着`。
- 最後に `[dyne] 結果: 到着の合計 [n] 回(環境ごと)、やり直し m 回。軌跡 …/trajectory_env0.csv`。
  合格の目安(案): 60 秒で到着が 3 回以上、やり直し 0 回。

画面: 部屋を斜め上から見る視点で始まる。Go2 の上に指令の速度の矢印が出る。

動画を残す(画面なし):

```bash
./isaaclab.sh -p ~/dyne/scripts/isaaclab_go2_room.py --usd ~/dyne/demo_out/twin/room.usda \
    --policy $POLICY --seconds 30 --video --video_length 1500 --headless --out ~/go2_record/dummy_video
ls ~/go2_record/dummy_video/videos/       # mp4 ができる
```

よく使う指定: `--goals 2.0 0.6 0.6 1.8`(目標の点を手で渡す。部屋の座標 [m])、`--spawn X Y`、`--yaw 90`、
`--max-speed 0.4`(ゆっくり)、`--clearance 0.5`(余裕を広く)、`--num_envs 4`(部屋を 4 つ並べる)。

### 9.3 残すもの

- スクリーンショット: (1) 部屋の全体と Go2(斜め上)、(2) Go2 が目標に近づくところ、(3) 衝突判定の表示を出した画面
  (表示のメニューの Physics → Collision の表示。壁・家具の見えない箱が出ること)
- 動画(9.2 の mp4)、`dummy_run.log`、`trajectory_env0.csv`、`nvidia-smi.txt`、版(Isaac Sim・Isaac Lab)、各段階にかかった時間
- **置き場所**: ダミーの部屋(シミュレータの room6)の画像・動画は公開してよいもの。リポジトリに入れるか(例 docs/demo/)は
  ユーザーが決める。**場所A(11/27 までの仮の部屋)の部屋の画像・動画は公開のリポジトリに入れず、非公開の
  dyne-private に置く**。
- 撮り方・ファイル名・公開の前の確認は docs/2026-11-19-capture-procedure.md(GitHub issue #94)に従う。

## 10. 段階 8: 11/20 に作る部屋で

1. この PC で、実測から作った部屋の USD を書き出す(`twin_live.py` の定期の書き出し、または `twin_demo.py`)。
2. この PC で `usd_check.py` と `isaaclab_go2_room.py --check-only` を通す。置ける場所が無い・狭いときは、
   `--clearance` を 0.40 まで下げるか、`--goals` で手で渡す(0.39 m 未満は Go2 がその場で回ると当たりうる)。
3. GPU の計算機へ運び、9.2 の `--usd` をその USD に変えて実行。記録は `~/go2_record/siteA/` に分け、**dyne-private へ**。

### 案 B(上の層も学習する)を後で足すときの要点(未着手)

Isaac Lab の ANYmal-C の目標の位置へ行く課題(2.3.2 では `Isaac-Navigation-Flat-Anymal-C-v0`、3.0 では
`IsaacContrib-Navigation-Flat-AnymalC`)は、学習済みの歩行の方策(`policy.pt`)を「行動の項」(`PreTrainedPolicyActionCfg`)に入れ、
上の層は 10 倍の間隔で速度の指令を出す。観測は胴の速度・重力の向き・目標の位置と向き、報酬は目標との距離と向きの差 [S14]。
Go2 にするには: 下の層の設定を `UnitreeGo2FlatEnvCfg` に、`policy_path` を 8 章の `policy.pt` に、目標の範囲(±3 m)を部屋の
置ける範囲に、場面に部屋を足す。家具を避けるには障害物の観測(光線の距離など)が要る。学習 1 回の時間は**未確認**。

## 11. うまくいかないとき

| 症状 | 見るところ・対処 |
|---|---|
| `ModuleNotFoundError: No module named 'isaacsim'` | 仮想環境を有効にし `source _isaac_sim/setup_conda_env.sh` [S7] |
| `ModuleNotFoundError: isaaclab_tasks.manager_based…` | Isaac Lab が 3.0(§5.4)か、`--install` が済んでいない |
| 起動が何分も止まる | 初回のシェーダの準備。`nvidia-smi` で GPU が使われていれば待つ |
| 資産(Go2 の模型・材質)が取れない | インターネットへの接続・社内のプロキシ。エラーの URL を記録 |
| ドライバのエラー・画面が真っ黒 | `nvidia-smi` のドライバの版が §3 の基準以上か。`--headless` で試す |
| 円柱・Go2 が壁を通り抜ける | `room_truth.usda` で同じことをする(推定の形か、書き出し方か)。衝突判定の表示で箱が見えるか |
| 部屋が表示されない・ずれている | `isaaclab_go2_room.py` は部屋を「出発点が原点」になるようずらして置く。`--spawn` を確認 |
| Go2 がすぐ倒れる・やり直しが続く | `policy.pt` が平らな課題(Flat)のものか。`--max-speed 0.4` に下げる。出発点の余裕(確認の行)を見る |
| `cfg.scene.room` を足す所でエラー | 場面の設定に後から項目を足す書き方が版で通らない可能性(**未確認**)。エラーの全文を持ち帰る |
| 動画ができない | `--video` は `--enable_cameras` を自動で付ける。`--headless` と一緒に使う |
| GPU のメモリ不足 | `--num_envs 1`。学習なら `--num_envs 1024` |

エラーが出たら、**コマンド・出力の全文・版**を保存して持ち帰る(この PC の Claude Code で台本を直す)。

## 12. 台本について確信がない点(GPU の計算機で確かめる)

1. 場面の設定(`cfg.scene`)に、作った後から `room` を足して部屋が各環境に複製されるか。
2. Isaac Lab の平らな床(z = 0)と部屋の床の箱(上面 z = 0)が重なっても問題が無いか。
3. 速度の指令(`vel_command_b`)を毎手上書きする方法で、指令が方策の観測に入るか(2.3.2 の指令の項の読みでは入る)。
4. 胴が家具に触れたときの終了(`base_contact`)が部屋の箱にも効くか(効けば「やり直し」と表示される)。
5. Isaac の Python で、起動の後に `from pxr import Usd` が読めるか(確認の部分で使う)。

## 出典

- [S1] Isaac Lab v3.0.0 のリリースノート https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0
- [S2] Isaac Lab v2.3.0 のリリースノート https://github.com/isaac-sim/IsaacLab/releases/tag/v2.3.0
- [S3] Isaac Lab 2.3.2 の導入の文書 https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/index.html
- [S4] Isaac Lab v2.2.0 のリリースノート https://github.com/isaac-sim/IsaacLab/releases/tag/v2.2.0
- [S5] Isaac Sim 5.1 の要件 https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html
- [S6] Isaac Lab(main)の導入の文書 https://isaac-sim.github.io/IsaacLab/main/source/isaaclab/installation.html
- [S7] Isaac Lab 2.3.2 のフォルダ版の Isaac Sim への導入 https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/binaries_installation.html
- [S8] Isaac Lab 2.3.2 の pip の導入(この文書の作成時には読んでいない。**未確認**)https://isaac-sim.github.io/IsaacLab/v2.3.2/source/setup/installation/pip_installation.html
- [S9] Isaac Lab(main)の課題の一覧 https://isaac-sim.github.io/IsaacLab/main/source/isaaclab/environments.html
- [S10] 第三者の Go2 の解説(ros2-essentials)https://j3soon.github.io/ros2-essentials/go2-ws
- [S11] Isaac Lab 2.3.2 の Go2 の課題の登録・学習の設定(ソース)
  https://github.com/isaac-sim/IsaacLab/tree/v2.3.2/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2
- [S12] Isaac Lab 2.3.2 の既存の学習の台本の使い方 https://isaac-sim.github.io/IsaacLab/v2.3.2/source/overview/reinforcement-learning/rl_existing_scripts.html
- [S13] Isaac Lab 2.3.2 の rsl_rl の再生の台本(ソース。学習済みの取得と policy.pt の書き出し)
  https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/scripts/reinforcement_learning/rsl_rl/play.py 、
  https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab_rl/isaaclab_rl/utils/pretrained_checkpoint.py
- [S14] ANYmal-C の目標の位置へ行く課題の設定(ソース)
  https://github.com/isaac-sim/IsaacLab/tree/v2.3.2/source/isaaclab_tasks/isaaclab_tasks/manager_based/navigation/config/anymal_c 、
  3.0: https://github.com/isaac-sim/IsaacLab/tree/v3.0.0/source/isaaclab_tasks/isaaclab_tasks/contrib/navigation/config/anymal_c
