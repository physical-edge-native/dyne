# OpenFOAM との接続(書き出しと速度場の読み込み)— GitHub issue #57 の後半(2026-11-17)

Issue #57「場の状態を VTK と OpenFOAM へ書き出す」の後半。前半(VTK ImageData、PR #63)に続き、
OpenFOAM(広く使われているオープンソースの数値流体力学(CFD)のパッケージ)との接続を両方向で作った。

- Dyne → OpenFOAM: 正則格子のスカラー場(温度・CO2・湿度など)を、`scalarTransportFoam`(与えた流速の中での
  移流・拡散を解く OpenFOAM の解法)の case(計算の一式のフォルダ)として書き出す。
- OpenFOAM → Dyne: OpenFOAM で計算した流速場を、Dyne の格子に写したものとして読み込む
  (docs/design/2026-11-16-fluids-for-now.md の「当面、気流は外から与える」の最初の入口)。

OpenFOAM はこの環境(Windows、WSL の Ubuntu にも未導入、Docker なし)になく、ユーザーの決定どおり CI でも動かさない。
確かめたのは形式と、自分が書いたものを読み直したときのビット一致だけである。**OpenFOAM で実際に blockMesh・
scalarTransportFoam・mapFields を通したことはまだない**(下の「未確認のこと」)。

## 作ったもの

- `crates/dyne-export/src/foam.rs`(依存なし): `Grid`・`Case`(書き出しと読み直し)・`import_velocity`、
  OpenFOAM の辞書の形式の小さな読み手(コメント、語、数、文字列、`( ) { } [ ] ;`)。
- `crates/dyne-export/src/bin/dyne-foam.rs`: 道具 `dyne-foam`
  - `dyne-foam case --vti IN.vti --out DIR [...]`: .vti の場(既定は最初の配列)から case を書く
  - `dyne-foam info DIR`: case を読み直し、ファイルどうしの整合を確かめ、T の CRC-32 を出す
  - `dyne-foam velocity DIR TIME [--vti OUT.vti]`: `DIR/TIME/U` を読み、成分ごとの CRC-32 を出す。
    `--vti` で成分 `ux`・`uy`・`uz` を Dyne の格子の .vti に書く
- 合成の例: `crates/dyne-export/tests/fixtures/openfoam-velocity/`(`system/dyneGrid` と `0/U`。3×2×2 = 12 セル)。
  OpenFOAM が ASCII で書く体裁を手で書いたもの(OpenFOAM の出力の写しではなく、OpenFOAM のバナーの文面も使っていない。
  ライセンスはリポジトリと同じ)。
- テスト: 単体テスト(foam.rs)と結合テスト(tests/foam_fixture.rs)、scripts/ci-check.sh の [14]。

## 決めたこと

### 格子の対応(大きめの判断。推奨案で実装し、ほかの案を残す)

OpenFOAM の有限体積法は値をセルの中心に持ち、Dyne は格子点に持つ。対応のさせ方の案:

- **案A(採用、推奨)**: Dyne の格子点1つを OpenFOAM のセル1つの中心にする。NX×NY×NZ 点・間隔 h の格子は、
  NX×NY×NZ セル・大きさ h の1ブロックになり、ブロックは最初と最後の点から半間隔ずつ外へ広がる
  (x_min = x0 − h/2、x_max = x0 + (NX − 1/2)·h)。
  - 利点: 値が補間なしで1対1に移る(書き出した T の値は Dyne の値そのもの)。blockMesh は1ブロックのセルを
    x が最も速く、次に y、次に z の順に番号づけるので、Dyne の場の並びと同じ。速度場の読み込みも同じ対応で1対1。
  - 欠点: 外周の面が Dyne の端の点から半間隔外にずれる。固定値の条件は Dyne では端の点に、OpenFOAM では
    半間隔外の面に掛かる。
- 案B: Dyne の格子点を OpenFOAM のメッシュの頂点にする((NX−1)×… セル)。境界の位置は一致するが、セルの値を
  頂点の値から平均で作る必要があり、値がビット一致で移らない。読み込みも逆の補間が要る。
- 案C: Dyne の内側の点だけをセルの中心にし、端の点の値を境界の面の値にする。境界の値は残るが、面の位置は
  やはり端の点から半間隔ずれ(一様なセルでは避けられない)、セルの数も Dyne と合わない。

案Aでも、OpenFOAM が頂点から計算するセルの中心の座標は、Dyne の点の座標と丸め(最後の桁の数単位)の範囲で
一致するだけで、ビット一致ではない。

### 2次元の扱い

z 方向に1点の格子(NZ = 1)は2次元の case として書く: セル1枚の厚さ(x の間隔と同じ。.vti の2次元の約束と同じで
z = 0 を中心にする)、前後の面は1つのパッチ `frontAndBack`(種類 `empty`)。3次元で z に1セルだけの格子を
作りたい場合はこの形では書けない(制限)。

### 書き出すファイル(OpenFOAM.com 版 v2xxx の形式)

| ファイル | 中身 |
|---|---|
| system/blockMeshDict | `scale 1`、頂点8つ、`hex (0 1 2 3 4 5 6 7) (NX NY NZ) simpleGrading (1 1 1)`、パッチ west・east(x)・south・north(y)・bottom・top(z)は種類 `patch`。2次元は `frontAndBack`(`empty`) |
| system/controlDict | `application scalarTransportFoam`、`endTime`・`deltaT`・`writeInterval`(既定 1・0.01・0.1 秒)、`writeFormat ascii`、`writePrecision 17` |
| system/fvSchemes | 時間 Euler、移流 `Gauss linearUpwind grad(T)`、拡散 `Gauss linear corrected` |
| system/fvSolution | T は PBiCGStab + DILU、許容 1e-10、`SIMPLE { nNonOrthogonalCorrectors 0; }` |
| system/dyneGrid | Dyne の格子そのもの(点の数・原点・間隔を最短の10進で)。読み直しと速度場の読み込みに使う。OpenFOAM は読まない |
| system/mapFieldsDict | mapFields 用: `patchMap ( )`、`cuttingPatches ( west east ... )` |
| constant/transportProperties | `DT [0 2 -1 0 0 0 0] 値`(既定 2.2e-5 m²/s、空気の熱拡散率の目安) |
| 0/T | `nonuniform List<scalar>` で全セルの値、面ごとの条件 |
| 0/U | 与えた流速(`uniform (ux uy uz)` かセルごとの `nonuniform List<vector>`)、全面 `zeroGradient` |

- 値の書き方: Rust の最短で元に戻る10進表記(`0.1`、`1e-5`、`-0.0`)。正しく丸める10進の読み手(Rust、glibc の
  strtod)なら同じ 64 ビットの値に戻る。有限でない値(NaN・無限大)は書き出しで拒否する(OpenFOAM が使えないため)。
  非正規化数は書けるが、OpenFOAM の読み手が受け付けるかは未確認。
- 移流の離散化に linearUpwind を選んだのは小さな判断(OpenFOAM の scalarTransportFoam の例題と同じ。中心差分は
  ペクレ数が大きいと振動する)。

### 面の条件の対応(小さな判断)

`Face` は Dyne 側の部品に依らない独自の列挙にした(dyne-export を依存なしに保つため)。

| Dyne の面 | OpenFOAM の T の条件 |
|---|---|
| 固定値(Dirichlet。dyne-native の `Outer::Fixed(v)`、sim2d の `dirichlet(v)`) | `fixedValue`(`value uniform v`)。ただし面の位置は半間隔外(上記) |
| 勾配ゼロ(Neumann。`Outer::ZeroGradient`、sim2d の `zero_grad`) | `zeroGradient` |
| (Dyne にまだない)流出 | `inletOutlet`(流れが入る所は `inletValue`、出る所は勾配ゼロ)。開いた面向け |
| 波動の吸収境界(`Outer::Absorbing`) | 対応させていない。移流・拡散の場では意味が違う(近いものは OpenFOAM の `advective`)。必要なら選択肢として相談 |

U の境界はすべて `zeroGradient` にした(scalarTransportFoam は U を解かず、面の流束を U から作るだけなので)。
壁で法線方向の流速をゼロにしたい場合は、読み込んだ流速場か U の境界を利用者が直す。

### 速度場の読み込みの経路(推奨案で実装)

- **採用**: Dyne が書き出したメッシュ上の `U` の `internalField`(ASCII)を読む。CFD の計算(どんなメッシュでもよい)から
  Dyne の格子へは OpenFOAM の `mapFields` で写す:

  ```text
  dyne-foam case --vti field.vti --out flowGrid   # Dyne の格子の case(T は使わない)
  cd flowGrid
  blockMesh
  mapFields ../roomFlow -sourceTime latestTime    # roomFlow は CFD の case。flowGrid/0 に写す
  cd ..
  dyne-foam velocity flowGrid 0 --vti flow.vti
  ```

  mapFields は写す先の system/mapFieldsDict を使い(Dyne の面はすべて cutting patch = 元の場の内部から補間)、写す先の
  controlDict の設定(ASCII・17 桁)で書く。CFD の case のほかの場(p や T)も写されるので、Dyne の状態の case とは
  別の case(flowGrid)を使う。CFD のメッシュの外にある Dyne のセルは、書き出したときの流速(既定はゼロ)のまま。
  Dyne のメッシュの上で直接流れを解いた場合は、その最後の時刻のフォルダを読めばよい。
- ほかの案(採らなかった): (1) `sample`/`postProcess` の sets(raw・csv)で点の列に書き出す — 版ごとに書式と設定の名前が
  違い、点の並びを Dyne の格子に合わせる手間が利用者に残る。(2) foamToVTK の非構造格子 VTK — 読み手が大きくなり、
  格子への写しは別に要る。
- 読めるもの: `uniform (a b c)`、`nonuniform List<vector> N ( ... )`、OpenFOAM の短い書き方 `N{(a b c)}`・
  `N((a b c) ...)`。読めないもの: binary(`foamFormatConvert` で ASCII に直す)、圧縮(.gz)、`#include`・`$変数`
  (OpenFOAM が書いた時刻フォルダの場は展開済みなので通常は出ない)。
- 読み込みの正確さ: 10進の文字を正しく丸めて読むので、読み込んだ値はファイルに書かれた数そのもの。
  `writePrecision 17` で書かれていれば OpenFOAM の 64 ビットの値と一致する。既定の 6 桁で書かれた場合は、
  ファイルの数とは一致するが OpenFOAM の内部の値とは最大で相対 5×10⁻⁷ ほど違う。

### 道具と CI(小さな判断)

- シミュレータ(dyne-sim2d、dyne-native-wave2d/3d)に `--foam` を足さず、.vti を入力にする変換の道具 `dyne-foam` にした。
  .vti を書ける実行はすべてそのまま使え、シミュレータの変更がない。
- CI の [14]: [13] の .vti(dyne-sim2d の熱 2次元 33×25、native の 3次元波動 25×23×21)を case にし、読み直した T の
  CRC-32 が実行の出したハッシュ(sim2d の熱の値、2ca26bb5)と同じで、セルの数が合うことを確かめる。合成の速度場は、
  成分ごとの CRC-32(ux 488e48e0、uy 1c058935、uz bca80f3a)が固定値と一致し、.vti に書いて dyne-vti で読み直しても同じこと。
  3次元の波動の場を「温度」として書くのは形式の確認のためで、物理の意味はない。
- 変更の記録の断片: 前半の `changelog.d/57.added.md` がマージ済みなので、`changelog.d/57.added.2.md` を足した
  (changelog.d/README.md は `<Issue番号>.<節>.md` だけを定めていて、同じ Issue の2つ目の PR の名前は決まっていない)。

## 比較について(許容誤差はまだ主張しない)

OpenFOAM の結果と Dyne の結果は、離散化が違う(セル中心と格子点、境界の面の位置が半間隔ずれる、移流の
linearUpwind と拡散の補正、時間積分、線形解法の許容 1e-10)ので、ビット一致はしない。比べるときは許容誤差と
その根拠を先に書く必要があるが、今回はまだ比較をしておらず、許容誤差も主張しない。比べられるのは、Dyne に
移流・拡散の解法ができ、OpenFOAM で実際に case を流してからである。

一致するのは次の2つだけ(CI で確認):
- 書き出した case を Dyne が読み直したときの値(T・U・面の値・DT・時間の設定・格子)はビット一致。
- 合成の速度場の読み込みは、ファイルの数とビット一致。

## 未確認のこと(OpenFOAM がないため)

- blockMesh・scalarTransportFoam・mapFields が書き出したファイルをそのまま受け付けるか(OpenFOAM.com v2xxx)。
- OpenFOAM.org(Foundation 版)との違い: blockMeshDict の倍率は `convertToMeters` と書かれている(`scale` を受け付ける
  かは未確認)。最近の版は解法を foamRun で動かすモジュールに移し、多くの解法で transportProperties を
  physicalProperties に改名しているので、scalarTransportFoam が無い・設定の名前が違う可能性がある。
- 非正規化数の値を OpenFOAM が読めるか。
- mapFields が cutting patch の値を内部から補間する動作と、写す先にない場の扱い(版による)。

OpenFOAM の入った環境(WSL・Docker など)が用意できたら、上の手順を手で通して、この節を更新する。
