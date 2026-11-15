# ロボティクス向け物理シミュレータ比較：扱える物理現象と必要な計算資源

作成日：2026-11-13
対象：Isaac Sim / Isaac Lab（PhysX・Newton）、MuJoCo系（MuJoCo / MJX / MuJoCo Warp）、Genesis World、ManiSkill3（SAPIEN）、Gazebo、Webots、CoppeliaSim、PyBullet、Drake、Brax、RaiSim、Project Chrono

> **注意**
> - 各シミュレータの対応状況はバージョンで大きく変わります（特にIsaac Lab 3.0、Newton、MuJoCo Warpは2026年に大きく変化）。ここでは2026年11月時点で確認できた情報と、一般的に知られている仕様をもとにしています。
> - 「対応」は**APIで普通に使える**レベルを指します。プラグインや自作コードで無理やり実現できるものは「△」にしています。
> - 計算資源の数値は公式要件が出ているものを除き**目安**です。

---

## 1. 記号と分類の定義

| 記号 | 意味 |
|---|---|
| ◎ | 中核機能。高精度・高速で実用レベル |
| ○ | 標準でサポート。実用可能 |
| △ | 限定的／実験的／プラグインや近似モデルでのみ対応 |
| × | 非対応（実質的に扱えない） |

### 物理現象の分類

| 分類 | 内容の例 |
|---|---|
| **剛体・多関節** | ロボットアーム、脚ロボット、関節の拘束、関節リミット |
| **接触・摩擦** | 把持、歩行時の足裏接触、滑り、転がり摩擦 |
| **アクチュエータ** | モータの特性（トルク・速度の制限、遅延、ギア比、反射慣性） |
| **体積変形体（ソフトボディ）** | ゴム、スポンジ、ぬいぐるみ、臓器（FEMなど） |
| **布・薄膜（シェル）** | 衣服、シート、紙袋 |
| **ケーブル・ロープ（1次元変形体）** | ロープ、ホース、配線 |
| **粒状体・弾塑性体** | 砂、土、土木作業、粘土（MPM・DEMなど） |
| **流体（粒子法）** | 液体を注ぐ、こぼれる（SPH・PBDなど） |
| **流体力（近似）** | 浮力、抗力、揚力、風（流体を解かずに力だけを与えるモデル） |
| **本格的なCFD** | ナビエ–ストークス方程式による高精度な流れ解析 |
| **熱・電磁気** | 発熱、温度変化、磁力、電気回路 |
| **破壊・切断** | 割れる、ちぎれる、切る |
| **微分可能性** | 勾配を使った最適化や学習（ロールアウト全体を微分できるか） |

---

## 2. 総合比較表：扱える物理現象

| | 剛体・多関節 | 接触・摩擦 | アクチュエータ | 体積変形体 | 布・薄膜 | ケーブル | 粒状体・MPM | 流体（粒子） | 流体力（近似） | CFD | 熱・電磁気 | 破壊 | 微分可能 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **Isaac Sim / Lab（PhysX）** | ◎ | ◎ | ◎ | ○ (FEM) | ○ | × | △ | △ (PBD) | △ | × | × | × | × |
| **Isaac Lab 3.0（Newton）** | ◎ | ◎ | ◎ | ○ (VBD) | ○ (VBD) | ○ (VBD) | ○ (陰的MPM) | △ | △ | × | × | × | ○ (一部ソルバー) |
| **MuJoCo（CPU）** | ◎ | ◎ | ◎ | ○ (flex) | ○ (flex) | ○ (flex/composite) | × | × | ○ | × | × | × | △ (有限差分) |
| **MJX（JAX）** | ◎ | ○ | ○ | × | × | △ | × | × | ○ | × | × | × | ○ (JAX) |
| **MuJoCo Warp** | ◎ | ◎ | ◎ | △〜○ | △〜○ | △ | × | × | △ | × | × | × | △ |
| **Genesis World** | ○ | ○ | ○ | ◎ (FEM/MPM/IPC) | ◎ | ○ | ◎ (MPM) | ○ (SPH/PBD) | △ | × | × | △ (MPM) | ○ (一部ソルバー) |
| **ManiSkill3（SAPIEN）** | ◎ | ◎ | ○ | △ | △ | × | × | × | × | × | × | × | × |
| **Gazebo（Harmonic/Jetty）** | ○ | ○ | ○ | × | × | × | × | × | ○ (浮力・水中流体力・揚力抗力・風) | × | × | × | × |
| **Webots** | ○ | ○ | ○ | × | × | × | × | × | ○ (Fluid node) | × | × | × | × |
| **CoppeliaSim** | ○ | ○ | ○ | △ (エンジン次第) | △ | △ | × | × | △ | × | × | × | × |
| **PyBullet** | ○ | ○ | △ | ○ (FEM/ばね質点) | ○ | △ | × | × | × | × | × | × | × |
| **Drake** | ◎ | ◎ (hydroelastic) | ○ | ○ (FEM) | × | × | × | × | × | × | × | × | ○ (AutoDiff) |
| **Brax** | ○ | △〜○ | △ | × | × | × | × | × | × | × | × | × | ◎ |
| **RaiSim** | ◎ | ◎ | ○ | × | × | × | × | × | × | × | × | × | × |
| **Project Chrono** | ○ | ○ | ○ | ◎ (FEA) | ○ | ◎ (ビーム/ケーブル) | ◎ (DEM, 地盤) | ○ (SPH, Chrono::FSI) | ○ | △ | × | △ | × |

### 表から読み取れる全体傾向

1. **剛体・接触はどれでも扱える**が、精度と速度はIsaac（PhysX/Newton）、MuJoCo、RaiSim、Drakeが頭一つ抜けています。
2. **変形体・粒子系を本気でやるならGenesis、Newton、Chrono**。PhysXもFEMや布を持っていますが、Isaac Labでの扱いはまだ発展途上です。
3. **本格的なCFD、熱、電磁気、破壊は、ロボット向けシミュレータのほぼすべてで対象外**です。必要な場合はOpenFOAM、ANSYS、COMSOLなどの専門ツールとの連成になります。
4. **微分可能性**はBrax、MJX、Newton、Genesis、Drakeが持っていますが、接触を含む勾配は不安定になりやすく、実務の主流は依然として強化学習（勾配を使わない方法）です。

---

## 3. シミュレータ別の詳細

### 3.1 Isaac Sim / Isaac Lab（NVIDIA）

**物理バックエンド**
- **PhysX 5**：従来からのエンジン。Isaac Lab 2.xの標準。
- **Newton**：2026年に1.0が正式公開。Warp・OpenUSDベースで、Google DeepMind・Disney Researchと共同開発。
- Isaac Lab 3.0では、PhysX・Newton・OvPhysX・Isaac Sim無しで動く構成を切り替えられる「マルチバックエンド」になりました。

**扱える物理現象**
- 剛体・多関節・接触・アクチュエータ：◎
  - 理想的なPD制御、DCモータのモデル、ニューラルネットで学習したアクチュエータ（Actuator Net）などのモデルが標準で用意されています。
  - 吸着グリッパ（サクション）用の環境もあります。
- 変形体：Isaac Lab 3.0では次の3種類に整理されています。
  - **体積変形体**（ゴム、ぬいぐるみ、臓器）：四面体メッシュ
  - **面変形体**（布、シート、膜）：三角形メッシュ
  - **ケーブル**（ロープ、ホース、配線）

  バックエンドごとの対応状況：

  | | PhysX | Newton | OvPhysX |
  |---|---|---|---|
  | 体積 | ○ | ○（VBDソルバーのみ） | ○（CUDAのみ） |
  | 面（布） | ○ | ○（VBDソルバーのみ） | ○（CUDAのみ） |
  | ケーブル | × | ○（VBDソルバーのみ） | × |

  - 体積変形体の計算方法は、PhysXが有限要素法（FEM）、Newtonが VBD（Vertex Block Descent）ソルバーです。
  - 3種類とも**まだ開発途上**とされています。
- 流体・粒状体：流体や粒状体のような粒子ベースの材料は「変形体」とは別の資産・ソルバー（陰的MPM）で扱います。
  - PhysXにはPBDベースの粒子流体もありますが、Isaac Labの学習ワークフローではあまり使われていません。
- **Newtonのソルバー群**：XPBD、VBD、MuJoCo、Featherstone、SemiImplicit、Kamino、ImplicitMPM、Style3D。
  - 陰的MPMは粒状体や弾塑性体向けです。ETH Zurichが土砂の掘削に使っている例があります。
  - 1.4では、複数ソルバー（MuJoCo、Kamino、VBD、XPBD、MPM）を一つのワークフローで組み合わせる**実験的な連成機能**が入りました。

**扱えない・苦手な物理現象**
- 本格的なCFD、熱、電磁気、破壊は扱えません。
- 変形体と強化学習の大規模並列の組み合わせは、剛体ほど成熟していません（メモリと速度の制約が大きい）。
- 吸着グリッパは粒子系の布をつかめない、という報告がフォーラムにあります（機能の組み合わせによる制約）。

**計算資源**

| 項目 | 内容 |
|---|---|
| GPU | **NVIDIA RTX必須**（PhysXのGPU計算と、RTXレンダリングのため） |
| VRAM | Isaac Lab 2.3の公式要件は16GB以上（描画を伴う場合はさらに必要）。Isaac Sim 5.1の最低要件はRTX 4080（16GB） |
| RAM | 32GB以上（公式）。実用上は64GBが安心 |
| OS | Ubuntu 22.04/24.04、Windows 11。**macOSは非対応**。aarch64版はDGX Sparkのみ |
| ストレージ | 50GB以上のSSD（アセット込みでもっと必要） |
| 並列規模の目安 | 剛体の歩行タスクなら、1GPUで数千〜数万環境（公式ブログに約4万環境の例あり） |
| 備考 | カメラを使う学習は、環境数が一桁以上減ることが多い。Isaac Lab 3.0ではIsaac Sim無しの軽い構成も選べる |

---

### 3.2 MuJoCo系（Google DeepMind）

| 実装 | 動作環境 | 特徴 |
|---|---|---|
| **MuJoCo（C/C++）** | CPU（Mac・Windows・Linux） | 正統な実装。機能が最も揃っている |
| **MJX** | JAX（CPU・GPU・TPU） | JAXで書き直したもの。微分可能で、TPUでも動く |
| **MuJoCo Warp（MJWarp）** | NVIDIA GPU（CUDA） | 2026年2月に正式リリース。接触の多いシーンではMJXより並列化の効率が良い |

**扱える物理現象**
- 剛体・多関節・接触：◎。MuJoCoの最大の強みで、関節空間（一般化座標）による精密な計算と、ソフト制約による安定した接触が特徴です。
- アクチュエータ：◎。モータ、位置・速度サーボ、筋肉モデル、腱（テンドン）、スライダークランク機構、ギアなどの伝達機構があります。3.6では遅延（Delays）も追加されました。
- 変形体：MuJoCo 3.0から `flex` 要素が入り、1次元（伸びる線）、2次元（三角形）、3次元（四面体）の変形体を定義できます。
  - 3.7では、硬い変形体を時間刻みを小さくせずに安定して計算できるよう、陰的積分に対応しました。
  - MuJoCo Warpも変形体同士の衝突にネイティブ対応しました。
- 流体力（近似）：慣性ボックス近似や楕円体近似のモデルで、粘性・揚力・抗力・付加質量を計算できます。水中や空中の「流体から受ける力」は扱えますが、流体そのものは解きません。

**扱えない・苦手な物理現象**
- 粒子流体、MPMによる粒状体、CFD、熱、電磁気、破壊は扱えません。
- MJX・MJWarpはCPU版と完全に同等ではありません。未対応の機能を含むモデルを読み込むと例外が出ます。
  - 過去のバージョンでは、flex（変形体）、一部の積分器と流体抗力の組み合わせ、一部ソルバー、プラグインなどが未対応でした。
  - 最新の対応状況は公式の「API Compatibility」表で確認してください。
- 描画は簡素です（OpenGL）。フォトリアルな描画には外部レンダラー（Madronaなど）が必要です。

**計算資源**

| 実装 | 目安 |
|---|---|
| MuJoCo CPU | **ノートPCで十分**。8コア程度なら `mujoco.rollout` によるマルチスレッドで数十〜数百環境 |
| MJX | GPUなら消費者向けのRTX（8GB程度）から使える。TPUにも対応 |
| MuJoCo Warp | NVIDIA GPU必須（開発・デバッグ用にCPUでも動く）。RTX 3060〜4090級で数千環境規模 |
| MuJoCo Playground | `pip install playground` で入り、1台のGPUで数分〜数時間で方策を学習できる設計 |

---

### 3.3 Genesis World（Genesis AI）

**扱える物理現象**：今回比較した中で**最も幅広い**です。
- 剛体、FEM、MPM、粒子系（PBD・SPH）、IPCを同じシーン・同じ状態の中で連成して計算できます。
- 剛体・多関節：○。速度は非常に速いと公称されていますが、精度や実機転移の実績はIsaac・MuJoCoほど蓄積されていません。
- 変形体：◎。2026年5月の1.0では、IPC向けの外部関節拘束（External Articulation Constraint）と、バリア関数を使わない弾性体ソルバーが追加されました。
- 粒状体・弾塑性体（MPM）：◎。砂、雪、粘土など。
- 流体（SPH・PBD）：○。液体を注ぐ、かき混ぜるなど。
- 破壊：△。MPMで一部表現できます。

**扱えない・苦手な物理現象**
- 本格的なCFD、熱、電磁気は扱えません。
- 大規模な並列強化学習のエコシステム（タスク集、ドキュメント、実機転移の事例）はIsaac・MuJoCoに劣ります。
- 初回公開時の速度ベンチマークは、条件が限定的だったとして批判を受けました。

**計算資源**

| 項目 | 内容 |
|---|---|
| バックエンド | 独自コンパイラ（Quadrants、旧Taichiベース）で、CUDA・CPU・Apple Metalなど複数に対応 |
| OS | Linux・macOS・Windows |
| 剛体の並列学習 | RTX 4090級1台で高速（公称で毎秒数千万フレーム。条件次第） |
| 変形体・流体 | 粒子数・メッシュ数に応じてVRAMが急増する。数十万粒子なら16〜24GB級が目安 |
| 描画 | パストレーシング（Nyx）は重く、ハイエンドGPU向け |

---

### 3.4 ManiSkill3（SAPIEN、UCサンディエゴ）

**扱える物理現象**
- 剛体・接触：◎（PhysXベース）。物体操作（把持・挿入・組み立て）向けに調整されています。
- GPU上で物理計算と描画をまとめて並列化でき、「環境ごとに異なる物体」を同時に動かせます。
- 変形体：△。過去のManiSkill2にはソフトボディのタスクがありましたが、ManiSkill3の主軸ではありません。

**扱えない・苦手な物理現象**
- 流体、粒状体、布などは基本的に対象外です。
- 歩行ロボット向けのタスクは少なめです。

**計算資源**

| 項目 | 内容 |
|---|---|
| GPUでの並列計算 | NVIDIA GPU＋Linux（Vulkan/CUDA）。RTX 4090でRGBD＋セグメンテーションを3万FPS以上取得できると公称 |
| メモリ効率 | 他のプラットフォームよりGPUメモリ使用量が2〜4分の1と公称 |
| CPUモード | Mac・Windowsでも単一環境なら動作 |

---

### 3.5 Gazebo（Open Robotics）

**扱える物理現象**
- 剛体・接触：○。物理エンジンは差し替え可能で、標準はDART、Bulletなども選べます。精度・速度はMuJoCoやPhysXに一歩譲ります。
- **流体力（近似）が比較的充実**しています（いずれもシステムプラグインとして提供）。
  - 浮力（Buoyancy）
  - 水中機体の流体力（Hydrodynamics）
  - プロペラ・スラスタ（Thruster）
  - 揚力・抗力（LiftDrag）
  - 風（Wind）

  水中ロボット、ドローン、固定翼機のシミュレーションでよく使われます。
- センサー：◎（LiDAR、カメラ、深度、IMU、GPS、磁気センサー、気圧計、熱カメラなど）。
  - 熱カメラは「見え方」の模擬で、熱伝導を計算するわけではありません。

**扱えない・苦手な物理現象**
- 変形体、粒子流体、MPMは扱えません。
- GPUでの大規模並列（数千環境）はできません。基本的に1プロセス1ワールドです。

**計算資源**

| 項目 | 内容 |
|---|---|
| CPU | 物理計算はCPU。普通のPCで動く |
| GPU | センサーの描画（Ogre2）にはOpenGL対応GPUが望ましい。ヘッドレス（画面なし）でも可 |
| RAM | 8〜16GBで小〜中規模のシーンが動く |
| OS | Ubuntuが主。macOS・Windowsは一部制限あり |

---

### 3.6 Webots（Cyberbotics）

- **扱える**：剛体・接触（ODEベース）、Fluid nodeによる浮力・抗力、豊富なセンサー、既製のロボットモデル。
- **扱えない**：変形体、粒子流体、GPU並列。
- **計算資源**：軽い。ノートPCで十分。Windows・macOS・Linux対応。

### 3.7 CoppeliaSim（旧V-REP）

- **扱える**：物理エンジンをBullet・ODE・Vortex・Newton Dynamics・MuJoCoから選べます。剛体・接触は○。
  - 変形体やロープは、選んだエンジン（MuJoCoのflexやcompositeなど）の機能に依存して△。
- **扱えない**：粒子流体、MPM、GPU並列。
- **計算資源**：CPU中心で軽い。Windows・macOS・Linux対応。商用利用にはライセンスが必要です。

### 3.8 PyBullet

- **扱える**：剛体・接触○。ソフトボディ（FEMやばね質点モデル）や布○。
- **扱えない**：流体、粒状体、GPU並列。
- **計算資源**：CPUのみで非常に軽い。ただし近年は更新が少なく、新規採用は減っています。

### 3.9 Drake（MIT・トヨタ・リサーチ・インスティテュート）

- **扱える**：
  - 剛体・多関節◎
  - **hydroelastic接触**（接触面の圧力分布を計算する高精度な接触モデル）◎
  - FEMによる変形体○（把持研究向け）
  - AutoDiffによる微分可能性○
  - 最適化（軌道最適化、数理計画）◎
- **扱えない**：流体、粒状体、GPU並列による大量サンプル収集。
- **計算資源**：CPU中心。速度より正確さを重視した設計です。

### 3.10 Brax（Google）

- **扱える**：剛体（複数の計算パイプライン）、JAXによる微分可能性◎、TPU・GPUでの並列計算。
- **扱えない**：変形体、流体など。
- **現状**：物理計算はMJXへの移行が進み、現在は主に強化学習の学習ライブラリ（PPO、SACなど）として使われています。

### 3.11 RaiSim（ETH Zurich発）

- **扱える**：剛体・多関節・接触◎（高速・高精度）、高さマップ地形。
- **扱えない**：変形体、流体、GPU並列（CPUのマルチスレッドで並列化）。
- **計算資源**：CPUのみ。ライセンス条件に注意が必要です。

### 3.12 Project Chrono

- **扱える**：工学寄りの多物理シミュレーションで**最も守備範囲が広い**部類です。
  - FEA（ビーム、シェル、体積要素）◎
  - 粒状体のDEM（離散要素法）◎
  - 車両と地盤の相互作用（テラメカニクス）◎
  - SPHによる流体と構造物の連成（Chrono::FSI）○
- **扱えない**：ロボット学習向けのエコシステムは弱いです。
- **計算資源**：CPU中心。一部モジュール（粒状体、流体連成など）はGPU対応。大規模な粒状体計算は数十GB級のメモリを要することがあります。

---

## 4. 計算資源の比較まとめ

| シミュレータ | 最低限動く環境 | 実用的な推奨環境 | NVIDIA依存 | macOS |
|---|---|---|---|---|
| Isaac Sim / Lab | RTX・VRAM 16GB・RAM 32GB | RTX 4090/5090・VRAM 24〜32GB・RAM 64GB | **必須** | × |
| MuJoCo（CPU） | 普通のノートPC | 8〜16コアCPU | 不要 | ○ |
| MJX | CPUでも可（遅い） | RTX 8GB以上、またはTPU | 不要（JAX次第） | △（CPUのみ） |
| MuJoCo Warp | NVIDIA GPU | RTX 3060〜4090 | **必須**（CPUはデバッグ用） | × |
| Genesis | CPUでも可 | RTX 4090級（変形体ならVRAM 24GB以上） | 不要 | ○（Metal） |
| ManiSkill3 | CPU（単一環境） | RTX 4090級＋Linux | GPUモードは必須 | △（CPUのみ） |
| Gazebo | 普通のPC | 8コア・RAM 16GB・OpenGL GPU | 不要 | △ |
| Webots | ノートPC | ― | 不要 | ○ |
| CoppeliaSim | ノートPC | ― | 不要 | ○ |
| PyBullet | ノートPC | ― | 不要 | ○ |
| Drake | ノートPC | 多コアCPU | 不要 | ○ |
| RaiSim | ノートPC | 多コアCPU | 不要 | △ |
| Chrono | 普通のPC | 多コアCPU（＋GPUモジュール） | 一部モジュールのみ | ○ |

### 計算資源に関する一般的な法則

1. **状態のみの強化学習（剛体）は安い**：1GPUで数千〜数万環境を動かせます。
2. **カメラ画像を入力にすると一気に重くなる**：描画コストとVRAMで、環境数が10分の1〜100分の1になりがちです。ManiSkill3やMadrona（MuJoCo Playground）など、並列描画に特化した仕組みがあると有利です。
3. **変形体・流体は剛体の数十〜数百倍重い**：自由度が数千〜数百万に増えるためです。並列環境数を減らすか、ハイエンドGPU（VRAM 24GB以上）が必要になります。
4. **CPU系シミュレータ（Gazebo、Webots、Drake）は1環境あたりは軽い**が、強化学習の大量サンプル収集には向きません。

---

## 5. 「この物理現象を扱いたいなら」逆引き表

| 扱いたい現象 | 第一候補 | 次点 |
|---|---|---|
| 脚ロボットの歩行（剛体） | Isaac Lab、MuJoCo Playground / mjlab | RaiSim、Genesis |
| 精密な把持・組み立て | Isaac Lab（PhysX/Newton）、ManiSkill3 | Drake（hydroelastic接触） |
| ゴム・スポンジ・臓器（体積変形体） | Genesis、Isaac Lab（PhysX FEM / Newton VBD） | Drake、MuJoCo flex、Chrono |
| 布・衣服 | Newton（VBD・Style3D）、Genesis | Isaac Sim（PhysX）、PyBullet |
| ロープ・ケーブル | Newton（VBD）、Chrono | MuJoCo flex/composite、Genesis |
| 砂・土・雪（粒状体） | Genesis（MPM）、Newton（陰的MPM）、Chrono（DEM） | ― |
| 液体を注ぐ・こぼす | Genesis（SPH/PBD） | Isaac Sim（PBD）、Chrono::FSI |
| 水中ロボット・ドローン（流体力の近似） | Gazebo、MuJoCo（流体モデル） | Webots |
| 車両と地盤の相互作用 | Chrono | CARLA（車両運動中心） |
| 勾配を使った最適化 | MJX、Brax、Drake | Newton、Genesis |
| 高精度なCFD・熱・電磁気 | **ロボット向けシミュレータでは不可** → OpenFOAM、ANSYS、COMSOLなどとの連成を検討 | ― |

---

## 6. 参考情報源

- [Isaac Lab 3.0 Deformables（公式ドキュメント）](https://isaac-sim.github.io/IsaacLab/main/source/concepts/deformables.html)
- [Isaac Lab 3.0 Deformables（release/3.0.0）](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/concepts/deformables.html)
- [Isaac Lab Deformable Object Tutorial](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/tutorials/01_assets/run_deformable_object.html)
- [Isaac Lab 2.3 Local Installation（システム要件）](https://docs.nvidia.com/isaaclab/2.3/isaac-lab/local-installation)
- [Isaac Lab 3.0 Release Notes](https://docs.nvidia.com/isaaclab/references/release-notes)
- [NVIDIA Forum: Isaac Sim 5.1のVRAM要件](https://forums.developer.nvidia.com/t/is-8gb-vram-enough-for-basic-isaac-sim-5-1-usage/373600)
- [NVIDIA Forum: Isaac Sim 5.1 aarch64](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
- [NVIDIA Forum: Surface Gripperと布](https://forums.developer.nvidia.com/t/using-surface-gripper-extension-for-cloth-manipulation-in-isaac-sim/309363)
- [NVIDIA Blog: Isaac Lab 2.3](https://developer.nvidia.com/blog/?p=106540)
- [NVIDIA Blog: Isaac Lab と Newton による布操作](https://developer.nvidia.com/blog/?p=106618)
- [Newton Physics（Algolia index）](https://docsearch.algolia.com/mcp/docs/repo/newton-physics/newton)
- [Newton Solvers 1.2.0](https://newton-physics.github.io/newton/1.2.0/api/newton_solvers.html)
- [Newton v1.4.0 リリース](https://sourceforge.net/projects/newton.mirror/files/v1.4.0/)
- [Dataconomy: Newton 1.0](https://dataconomy.com/2026/03/17/nvidia-launches-newton-1-0-physics-engine-for-industrial-robot-training/)
- [MuJoCo Changelog 3.5.0](https://mujoco.readthedocs.io/en/3.5.0/_sources/changelog.rst.txt)
- [MuJoCo Changelog 3.7.0](https://mujoco.readthedocs.io/en/3.7.0/changelog.html)
- [MuJoCo 3.0 changelog（flex導入）](https://pyup.io/packages/pypi/mujoco/changelog?page=2)
- [MuJoCo passive force / fluid model ソース](https://huggingface.co/datasets/introvoyz041/mujoco/blob/main/data/src/engine/engine_passive.h)
- [MuJoCo Warp ドキュメント](https://mujoco.readthedocs.io/en/3.7.0/mjwarp/)
- [MuJoCo Warp Feature Parity](https://mujoco.readthedocs.io/en/stable/mjwarp)
- [MuJoCo Warp GitHub](https://github.com/google-deepmind/mujoco_warp)
- [MuJoCo Playground](https://aitopics.org/doc/arxivorg:DC4378F0)
- [Genesis World GitHub](https://github.com/Genesis-Embodied-AI/genesis-world)
- [Genesis World 1.0 ブログ](https://www.genesis.ai/blog/the-role-of-simulation-in-scalable-robotics-genesis-world-10-and-the-path-forward)
- [ManiSkill3 論文](https://arxiv.org/html/2410.00425v1)
- [ManiSkill（PyPI）](https://pypi.org/project/mani-skill-nightly/)
- [Gazebo Jetty Release Notes](https://gazebosim.org/docs/all/release_notes/)
