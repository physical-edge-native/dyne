# USD への衝突判定の追加と、物理シミュレータへの取り込みの検査 (docs\2026-11-11-usd-physics.md)

作成: 2026-11-11(Claude Code)。対象: docs/2026-11-11-two-week-plan.md の §5 G7(USD の衝突判定の情報)、
§4 実験7(USD を強化学習(RL)の環境に取り込む)の事前準備、§6(案 D=借りる GPU が本命、案 C=CPU で動く
物理シミュレータ MuJoCo が保険)。

## 1. 要旨

- 部屋のデジタルツインのデモ(scripts/twin_demo.ps1)が書き出す USD(Pixar が作った 3 次元の場面の記述形式)に、
  UsdPhysics(USD の物理の取り決め)の**静的な衝突判定**を付けた。床・壁・家具が衝突判定を持ち、
  ノードとセンサの目印・温度場(ダミー)・比較用の正解の形は衝突判定を持たない。
- 同じ形を MuJoCo(CPU で動く物理シミュレータ)の模型の記述形式 MJCF でも書き出す(room.xml)。
- 検査の台本を 2 本作った。**理想条件の 6 畳の部屋(room6)で両方 PASS**:
  - scripts/usd_check.py: 単位・物理の場・衝突判定の付き方・メッシュが閉じていること・外接箱を検査。
    OpenUSD 自身の検査の枠組み(UsdValidation、検査器 30 個)の指摘は **0 件**。
  - scripts/mjcf_check.py: MuJoCo 4.0.0 で球を落とし、円柱のロボットを 4 方向へ押して壁・家具で止まるかを確かめる。
    room.xml と、**USD の衝突判定そのものを MuJoCo に変換したもの**の両方で、止まる位置と正解の差は
    **最大 1.2 mm**、めり込みは最大 3.6 mm(MuJoCo の柔らかい接触の既定値による)。
- Isaac Sim / Isaac Lab(NVIDIA の GPU が要る物理シミュレータと RL の枠組み)で読み込む最小の台本を 2 本用意した。
  **GPU が無いので未検証(UNTESTED)**。借りた GPU の計算機で確かめる手順を §6 に書いた。

## 2. 変更・追加したファイル

| ファイル | 内容 |
|---|---|
| scripts/twin_physics.py(新規) | 推定表面 → 間引いた折れ線 → 閉じた板(衝突判定の形)。pxr(usd-core)で USD を組み立てる部品と、MJCF の書き出し |
| scripts/twin_demo.py(変更) | USD の書き出しを手書きの文字列から pxr に置き換え、衝突判定・物理の場・作り方の記録を追加。`--truth-usd`(正解の形の USD)、MJCF の書き出し、板の厚み・間引きの許容などの引数を追加 |
| scripts/twin_demo_view.html(変更) | 「衝突判定」ボタン(衝突判定の板を赤い半透明で重ねる)。USD の説明の文を更新 |
| scripts/twin_demo.ps1(変更) | 正解の形の USD を常に作る。最後に usd_check.py と mjcf_check.py を実行(`-NoCheck` で省略) |
| scripts/usd_check.py(新規) | USD の検査(PASS/FAIL) |
| scripts/mjcf_check.py(新規) | MuJoCo での衝突の検査(PASS/FAIL) |
| scripts/isaac_sim_room.py(新規、未検証) | Isaac Sim で部屋を読み、Jetbot(Isaac 標準の車輪ロボット)と球を動かす |
| scripts/isaaclab_room.py(新規、未検証) | Isaac Lab 2.x で部屋を読み、剛体の円柱を壁へ押す |

.venv に `mujoco`(4.0.0。CPU 版の配布物。依存の glfw・PyOpenGL も入った)を追加した。usd-core は 26.11 が既に入っていた。

## 3. USD の構成(prim のパスは書き出し直しても変えない)

```
/Room                          defaultPrim。Xform、kind=assembly。customData の dyne に作り方の記録
  /Room/Visual/<名前>          見た目の形(従来の押し出した面。両面表示。衝突判定なし)
  /Room/Collision/Floor        床。Cube(厚み 0.1 m、上面 z=0)、CollisionAPI、見える
  /Room/Collision/<名前>       壁・家具の衝突判定。Mesh(閉じた板の集まり)、CollisionAPI + MeshCollisionAPI、見えない
  /Room/PhysicsMaterials/Default  摩擦・反発(UsdPhysics.MaterialAPI。静止摩擦 0.8、動摩擦 0.6、反発 0)。
                               衝突判定に physics の用途で結び付ける
  /Room/TemperatureField/Floor 温度場(ダミー)。z = 3 mm(床の上面と重ならないよう)
  /Room/Markers/Nodes/Node_<番号>/{Body, Sensor_<番号>}  ノードとセンサの目印。衝突判定なし
  /Room/Reference/GroundTruth/...  比較用の正解の形(見た目のみ、既定で非表示)
/PhysicsScene                  重力 -Z、9.81 m/s²
```

- `<名前>` は `Walls`(外壁)、`Furniture_<k>`(場面ファイルの k 番目の家具)、`Other`(どれにも当たらない面)。
- 舞台の単位: upAxis = Z、metersPerUnit = 1、kilogramsPerUnit = 1。
- 作り方の記録(層の customLayerData と /Room の customData の両方に `dyne` として): 作成時刻(UTC)、通し番号
  (`--sequence`)、場面ファイル・形の CSV の名前、格子の刻み・点数・原点、mu、部屋と家具の高さ、板の厚み、
  間引きの許容、approximation、板の数、分割解と単一ノード解の一致の行。参照で取り込むと層の記録は失われるので、
  /Room にも同じものを置いた。
- 定期的な書き出し(実験6)では、同じパスのまま中身を差し替える想定。読み込む側は同じファイルを開き直す
  (または参照を再読み込みする)だけでよい。

## 4. 設計の選択

### 4.1 衝突判定の形 = 表面から物の側へ立てた閉じた板

推定表面(符号つき距離場 phi のゼロ等値線)は、そのままだと厚みの無い面(押し出した帯)で、閉じた立体にならない。
次の手順で閉じた立体の集まりにした。

1. マーチングスクエア法の線分を折れ線につなぎ、分類(外壁 / 家具 k / その他)が変わる所で切る
   (壁際の家具は、部屋の内側の 1 本の閉じた折れ線の中で壁と続いているため)。
2. Ramer-Douglas-Peucker 法(元の線からのずれが許容値以下になるよう頂点を間引く方法)で間引く。
   room6 では線分 736 本 → 板 77 枚。
3. 間引いた線分ごとに、表面から phi が負の側(物の側。phi の勾配の逆向き)へ厚み t の直方体を立てる。
   外壁は t = 0.10 m(部屋の外側へ)、家具は t = 0.05 m(家具の内側へ)。

板の表面側の面は推定表面そのものなので、ロボットが当たる位置は推定表面と同じ(間引きのずれ以内)。
板の 1 枚ずつが閉じた凸の立体なので:
- USD: 分類ごとに 1 つの Mesh にまとめても、どの辺もちょうど 2 面に共有され、面の向きがそろい、外向きになる
  (検査で確認)。
- MuJoCo: mesh の geom は凸包として扱われるため、部屋全体を 1 つの mesh にすると中が詰まった塊になる。
  板 1 枚 = box の geom 1 つにすれば形が変わらない。

**小さな実装判断**:
- 板の方式を選んだ。代わりの案は「穴あき多角形を三角形に分けて 1 つの水密な立体にする」だが、実装が重く、
  MuJoCo では結局凸の部品に分ける必要がある。
- 間引きの許容は 5 mm(`--simplify-tol`)。測距の雑音(理想条件で 2 mm)と格子の刻み(2.5 cm)より小さい。
- 板の厚み: 外壁 0.10 m、家具 0.05 m(`--wall-thickness`、`--furniture-thickness`)。
- 家具の上面は衝突判定に入れていない(板は家具の輪郭だけ。中は空洞)。車輪ロボットが家具の上に乗る課題には
  向かない。見た目には上面がある。
- 天井は衝突判定に入れていない(車輪ロボットには不要)。
- 床は格子の範囲から 0.5 m 外まで広げた直方体(`--floor-margin`)。

### 4.2 approximation = "none"(三角形メッシュのまま)

UsdPhysics.MeshCollisionAPI の approximation は "none" にした(`--approximation` で変えられる)。
PhysX(Isaac Sim の物理エンジン)は、**静的な**物体なら三角形メッシュをそのまま衝突判定に使える
(動く剛体には使えず、凸包や凸分割が要る)。部屋は動かないので最も忠実な "none" を選んだ。
板が凸なので "convexDecomposition" にしても形はほぼ変わらない。GPU の計算機で "none" に問題が出たときの代わりにする。

**小さな実装判断**: RigidBodyAPI はどこにも付けない(付けないと PhysX は静的な衝突物として扱う)。

### 4.3 見た目と衝突判定を分けた

見た目は従来どおりの押し出した面(両面表示、温度場の色はそのまま)。衝突判定は別の prim で、見えない
(visibility = invisible)。理由: 衝突判定の板は表面の裏へ厚みを持つので、見た目に出すと壁が太って見える。
HTML の表示では「衝突判定」ボタンで重ねて見られる。

**要確認(GPU の計算機)**: PhysX は見えない prim の衝突判定も有効にする、と理解しているが、この PC では
確かめられない。§6 の手順 4 で確かめる。

### 4.4 物理の場は defaultPrim の外に置いた

Isaac Sim / Isaac Lab は、取り込むファイルの defaultPrim(/Room)を自分の場面へ参照で入れ、物理の場は自分で作る。
物理の場を /Room の中に置くと、物理の場が 2 つになる。そこで /PhysicsScene は /Room の外に置いた。
ファイルを単独で開いたときは /PhysicsScene が使われる。

### 4.5 書き出しを pxr に切り替えた

これまでは USD を文字列として手で書いていた。物理の取り決め(apiSchemas の付け方など)を間違えないよう、
pxr(usd-core)の API で組み立てるようにした。twin_demo.py は .venv の Python が必要になった(twin_demo.ps1 は
もともと .venv を使う)。

## 5. 検査の結果(2026-11-11)

### 5.1 理想条件の 6 畳の部屋(`twin_demo.ps1`、既定 room6)

形状推定: `PASS (bit-exact): 15805 points`(分割解と単一ノード解のビット一致は従来どおり)。
推定表面と正解の面の距離 平均 1.4 mm、95%点 5.8 mm、最大 13.9 mm。

usd_check.py(room.usda、PASS):

| 衝突判定 | 面 | 連結成分(板) | 体積 | 開いた辺 | 向きの誤り | 面積 0 の面 | 外接箱の正解との差(最大) |
|---|---|---|---|---|---|---|---|
| Walls | 348 | 29 | 1.616 m³ | 0 | 0 | 0 | 7.1 mm |
| Furniture_0(机) | 192 | 16 | 0.059 m³ | 0 | 0 | 0 | 23.4 mm |
| Furniture_1(ベッド) | 276 | 23 | 0.065 m³ | 0 | 0 | 0 | 19.8 mm |
| Furniture_2(本棚) | 108 | 9 | 0.111 m³ | 0 | 0 | 0 | 17.6 mm |
| Floor | Cube | 1 | | | | | 部屋と壁の板を覆う。上面 z = 0 |

- 単位・defaultPrim・物理の場(重力 -Z、9.81 m/s²)・RigidBodyAPI なし・目印 86 個と見た目 9 個に衝突判定なし: すべて OK。
- 見た目のメッシュ: 面 13850、問題 0。
- UsdValidation(OpenUSD 26.11 の検査の枠組み。以前の UsdUtils.ComplianceChecker は 26.11 の Python には無い): 検査器 30 個、指摘 0 件。
- 外接箱の比べ方: 推定には隠れた面が無いので、推定の箱が正解の内側に縮むのは許し、はみ出しだけを許容 30 mm と比べる。
  家具の板が壁際の角の丸みで部屋の外(壁の中)へ出る分(机で最大 35.5 mm)は、もともと物の中なので除いて比べる。
- 正解の形の USD(room_truth.usda)も同じ検査で PASS(外接箱の差 0.0 mm)。

mjcf_check.py(MuJoCo 4.0.0、PASS。ロボットの出発点 (1.55, 1.50)、正解の面までの余裕 1.05 m):

| 試験 | room.xml(box 77 個) | USD の衝突判定を変換(凸の mesh 77 個) | 正解の形の USD |
|---|---|---|---|
| 球を 1 m から落とす(2 秒後の中心の高さ、期待 0.100 m) | 0.0998 m | 0.0998 m | 0.0998 m |
| ロボット +x(ベッド)止まった位置の差 | +1.1 mm | +1.1 mm | 0.0 mm |
| ロボット -x(西の壁) | +0.2 mm | +0.2 mm | 0.0 mm |
| ロボット +y(北の壁) | -1.2 mm | -1.2 mm | 0.0 mm |
| ロボット -y(机の手前の南の壁) | -0.1 mm | -0.1 mm | 0.0 mm |
| 最大のめり込み | 3.6 mm | 3.6 mm | 3.4 mm |

止まる位置の「正解」は、場面ファイルの形に対して、半径 0.15 m の円の前縁を 41 本の光線で近似して求めた。
正解の形の USD で差が 0.0 mm なので、推定の USD の差(最大 1.2 mm)は形の推定の誤差そのもの。

### 5.2 物の多い部屋(`twin_demo.ps1 -Preset cluttered`)

2×2 ノード、センサ 4 台・1 台 8 方向の厳しい条件。形状推定は `PASS (bit-exact): 6561 points`。
見える面の覆い率 5.2%、推定表面と正解の面の距離 平均 38.7 mm(この条件の推定はもともと粗い)。

- usd_check.py: 形式の検査はすべて PASS(衝突判定 4 個、開いた辺・向きの誤り・面積 0 の面すべて 0、UsdValidation の指摘 0 件)。
  外接箱は Furniture_1(ベッド)が正解から 50 mm はみ出す。twin_demo.ps1 は room6 以外では外接箱を「注意」として
  報告だけにする(`--bbox-warn-only`)。
- mjcf_check.py(`--strict` なし): 球は 0.0998 m で床に止まり、めり込みは最大 4.0 mm で PASS。
  ロボットは +x で推定表面に当たって止まった(正解より 57 mm 手前。MJCF の box と USD から変換した凸の mesh で
  差が 4 mm 違うのは、斜めの板の端に当たったときの接触の計算の違いと見ている。未確認)。
  -x・+y・-y は**推定の形の穴(未観測の所)を通り抜けた**。衝突判定の書き出しの問題ではなく、推定の覆い率の問題。

**小さな実装判断**: 検査の合否を 2 段に分けた。「形式・物理エンジンとして効くか」は常に合否に含め、
「推定の形の精度(外接箱、止まる位置、通り抜け)」は理想条件の room6 だけ合否に含める(`--strict`)。

## 6. Isaac での確認手順(借りる GPU の計算機。未検証)

この PC には GPU が無く、Isaac Sim は動かせない。以下は**一度も実行していない**。
API は Isaac Sim 4.5 / 5.x(`isaacsim.*`)と Isaac Lab 2.x(`isaaclab.*`)を前提にした。

### 6.1 事前に持っていくもの

- `demo_out/twin/room.usda`(推定)、`room_truth.usda`(正解。比較用)、`room.xml`(MuJoCo の模型)
- `scripts/isaac_sim_room.py`、`scripts/isaaclab_room.py`
- この PC での MuJoCo の結果(§5.1。Isaac の結果と比べる)

### 6.2 チェックリスト

1. 計算機の確認(ユーザー): GPU の機種と VRAM、ドライバ、Isaac Sim / Isaac Lab の版(Isaac Sim 4.5 か 5.x、
   Isaac Lab 2.x であること。1.x なら台本の import を `omni.isaac.lab.*` に直す必要がある)。
2. 目で読み込む: Isaac Sim の画面で File → Open で room.usda を開く。壁・家具・床・ノードの目印・温度の床が見え、
   単位の警告(metersPerUnit・upAxis)が出ないこと。
3. 物理の情報が読まれているか: Stage の木で /Room/Collision/Walls などを選び、Property に Physics の
   Collider(approximation = None / Triangle Mesh)が出ること。
4. 衝突判定を見る: 表示メニューの Physics の Collision の表示(Show Collision Shapes 相当)を全部にして、
   **見えない衝突判定の板が表示されること**(PhysX が見えない prim を衝突判定に使っているかの確認。§4.3)。
5. `isaac_sim_room.py --usd room.usda` を実行。合格の目安: 球の中心の高さ 0.09〜0.11 m、Jetbot が x < 2.6 m
   (ベッドの手前)で止まる。Jetbot の資産の場所が見つからないと止まる(版で場所が違う。台本は 2 か所を試す)。
6. `isaaclab.sh -p isaaclab_room.py --usd room.usda --headless` を実行。合格の目安: 円柱の最後の x ≈ 2.45 m
   (MuJoCo では 2.451 m)、z ≈ 0.10 m。
7. 失敗したときの切り分け: (a) room_truth.usda で同じことをする(推定の形の問題か、書き出し方の問題か)、
   (b) `twin_demo.py --approximation convexDecomposition` で書き出し直す、
   (c) 衝突判定を見えるようにする(twin_physics.py の collider_mesh の visible)。
8. 注力2(GPU 使用料の低減)の比較のため、使った時間・GPU の機種を記録する。

### 6.3 台本について確信がない点(GPU の計算機で確かめる)

- `isaacsim.storage.native.get_assets_root_path` と Jetbot の資産のパス(`/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd`
  か `/Isaac/Robots/Jetbot/jetbot.usd`)。
- `WheeledRobot` の車輪の関節名(`left_wheel_joint` / `right_wheel_joint`)。
- Isaac Lab の `RigidObject.set_external_force_and_torque` の引数の形(環境数 × 物体数 × 3)。
- 見えない prim の衝突判定(§4.3)。

## 7. 実行方法

```
powershell -ExecutionPolicy Bypass -File scripts\twin_demo.ps1                     # room6、検査まで
powershell -ExecutionPolicy Bypass -File scripts\twin_demo.ps1 -Preset cluttered
.venv\Scripts\python.exe scripts\usd_check.py demo_out\twin\room.usda --scene crates\dyne-sim2d\scenes\room6.txt
.venv\Scripts\python.exe scripts\mjcf_check.py --xml demo_out\twin\room.xml --usd demo_out\twin\room.usda --scene crates\dyne-sim2d\scenes\room6.txt --strict
```

出力(demo_out/twin/、git 管理外): room.html、room.usda、room_truth.usda、room.xml。

## 8. まだできていないこと・ユーザーが決めること

- **Isaac Sim / Isaac Lab での取り込みは未検証**(§6)。
- 家具の名前(Furniture_k)と分類は、推定表面の裏側にある場面ファイルの家具で決めている(高さも場面ファイルから)。
  実機(実験5)では場面ファイルの代わりに実験4の図面を使うことになる。図面が無い物(段ボール箱など)は Other になる。
  動く箱(実験6)を名前で追うかどうかは、ユーザーの判断が要る。
- 推定の形の穴(未観測の所)は衝突判定にも穴として残る。ロボットがそこを通り抜けられる。穴を塞ぐ(未観測を壁とみなす)
  かどうかは、RL の課題の設計(ユーザーの判断)による。
- 家具の上面・天井の衝突判定は無い(§4.1)。
- 板の厚み・間引きの許容・摩擦の値は仮の値。
