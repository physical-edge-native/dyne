//! dyne-sim: M-S0 Nノードシミュレータ基盤
//!
//! 1台のPC上でNノードのDyneクラスタ(1D縦切り)をシミュレートする。
//!
//! 設計(docs C章 M-S0):
//! - 単一プロセス・離散イベント・仮想時刻。壁時計を使わず、乱数シード固定で
//!   完全に再現可能。
//! - 計算は実機と同一のWASMアクター(heat1d_actor.wasm)をwasmiで実行する。
//!   計算を「シミュレート」するのではなく本物を動かす(検証の3層構造・上層)。
//! - 仮想ネットワーク: 遅延(平均+ジッタ)・パケットロス・一時分断を注入。
//! - 実機校正値の注入: 1ステップの計算時間(既定79.5ms=M1実測)を仮想時刻で消費。
//!   RAM上限(既定250KB)から載らない構成を起動前に拒否する。
//! - 故障注入: ノードの一時停止(pause)。恒久死からの回復は状態引き継ぎ
//!   (Orchestrator、将来)の領分であり本シミュレータの対象外。
//!
//! プロトコル(A-4 + M0/M1知見の反映):
//! - BSPステップ同期、論理ステップのみ。
//! - 直近2ステップの送信控えを保持し、過去ステップの重複受信には控えを再送。
//! - タイムアウト再送には最小間隔(バックオフ)を適用(M1知見4)。
//! - **界面IDはクラスタ全体で一意**(M0知見3の初実装)。界面 j はノード j と
//!   j+1 の間と定義し、メッセージはこのIDを運ぶ。送受での面変換は存在しない。
//!
//! 使い方:
//!   dyne-sim --wasm heat1d_actor.wasm --nodes 16 --points 65 --steps 2000 \
//!     --loss 0.10 --lat-ms 5 --lat-jitter-ms 3 --step-ms 79.5 --seed 42 \
//!     --pause 7@20000:40000 --partition 8@60000:90000
//!   (pause: ノード7を仮想20〜40秒停止。partition: ノード7|8間を仮想60〜90秒分断)
//!
//! M-S1a後半 (シナリオC = 解釈βの領域拡張ADD):
//!   --add 5@30000        … ノード5は開始時不在。隣接ノードは当該面を外部境界u=0
//!                          (BC_DIRICHLET_ZERO) として計算を止めずに進め、仮想30秒に
//!                          ノード5が新規参加すると界面へ切り替える (再init+状態復元)。
//!                          検証は複合参照解 (同じイベント時系列の単一配列逐次計算) との
//!                          ビット一致。界面残差 |grad_A - grad_B| を毎ステップ記録し、
//!                          参加直後の乱れピークと回復時間を報告する。
//!   --residual-csv f.csv … 界面残差の時系列をCSVへ書き出す (省略可)
//!
//! M-S1c (T2 自動トリガー, 2026-08-29):
//!   --auto-grow on | ratio=4,window=10,base=100,cooldown=100,step=8,max=32,min=20
//!       各界面の残差を移動窓 (直近 window ステップの中央値) で監視し、移動ベースライン
//!       (その直前 base ステップの中央値) の ratio 倍を超えたら、界面を挟む両ノードの
//!       隠れ層幅を step だけ広げる (GROW、max まで)。ヒステリシス: 発火後 cooldown
//!       ステップは再評価せず、さらに残差が ratio/2 倍を下回って再武装するまで発火しない。
//!       界面の履歴が min 未満 (参加直後の新界面) の間は隣の界面のベースラインで代用する。
//!   --event-csv f.csv … 参加・GROW/SHRINK・自動GROW・handover の時系列 (ダッシュボード用)
//!   --field-csv f.csv [--field-every k] … 温度場のスナップショット (kステップごと)

// 2026-09-02 (発注6・案B): モデル非依存の部品 (wasmi実行ラッパ・決定論乱数・
// 離散イベント・仮想ネットワーク) を crates/dyne-core へ切り出した。移設のみで
// ロジックは不変 (全回帰のビット一致で証明)。

use std::collections::HashMap;

use dyne_core::actor::{
    Actor, BC_DIRICHLET, BC_INTERFACE, BC_LINEAR_EXTRAP, BC_TRANSPARENT, BC_ZERO_GRAD, FACE_LEFT,
    FACE_RIGHT,
};
use dyne_core::event::EventQueue;
use dyne_core::net::VirtNet;
use dyne_core::transparent::TransparentCoef;

// ---------- 設定 ----------

#[derive(Clone)]
struct Config {
    wasm: String,
    nodes: usize,
    points: usize,
    steps: u64,
    alpha: f64,
    dt: f64,
    loss: f64,
    lat_us: u64,
    lat_jitter_us: u64,
    step_us: u64,
    timeout_us: u64,
    resend_min_us: u64,
    seed: u64,
    ram_cap_bytes: u64,
    /// (node, start_us, end_us)
    pauses: Vec<(usize, u64, u64)>,
    /// 状態引き渡しつき離脱→復帰: (node, leave_us, rejoin_us)
    /// 離脱時に u[] と現在ステップを取り出し、復帰時に**新しいアクターインスタンス**へ
    /// dyne_init + dyne_set で移植する (M-S1a: REMOVE→ADDの機構検証)
    handovers: Vec<(usize, u64, u64)>,
    /// (split_index k: ノードk-1|k間で分断, start_us, end_us)
    partitions: Vec<(usize, u64, u64)>,
    /// シナリオC (M-S1a後半): 解釈βの領域拡張ADD。(node, t_add_us)
    /// 指定ノードは開始時不在。隣接ノードは当該面を外部境界u=0
    /// (BC_DIRICHLET_ZERO) として計算を止めずに進め、参加時に界面へ切り替える。
    adds: Vec<(usize, u64)>,
    /// 界面残差 |grad_A - grad_B| の毎ステップ記録をCSVに書き出す先 (省略可)
    residual_csv: Option<String>,
    /// M-S1b: 学習型アクター (heat1d-pinn-actor) のハイパーパラメータ (dyne_cfg のキー, 値)。
    /// 全インスタンス (初期・handover復帰・ADD・bc切替の再init) の dyne_init 前に適用する
    pinn_cfg: Vec<(u32, f64)>,
    /// M-S1b: ADDで参加するインスタンスだけ温め反復回数を上書き (白紙=0 との比較用)
    add_warm: Option<f64>,
    /// M-S1b: GROW/SHRINK。(node, t_us, new_width)。dyne_resize を持つアクターのみ
    resizes: Vec<(usize, u64, u32)>,
    /// ノードごと・ステップごとの参照解との誤差をCSVに書き出す先 (省略可)
    error_csv: Option<String>,
    /// M-S1c: T2 自動トリガー (None = 無効)
    auto_grow: Option<AutoGrow>,
    /// イベント時系列 (参加・幅変更・handover) の CSV 出力先
    event_csv: Option<String>,
    /// 温度場スナップショットの CSV 出力先
    field_csv: Option<String>,
    /// 温度場スナップショットの間隔 (ステップ)
    field_every: u64,
    quiet: bool,
    /// F1 (発注3、2026-09-14): 未観測領域との境界の扱い (設定情報。コードに焼き込まない)。
    /// 適用されるのは「不在の隣接ノードに接する面」のみ。全域の外縁 (x=0, x=1) は
    /// 問題設定そのものの境界であり、常に u=0 の Dirichlet (F1 の対象外)。
    unobserved_bc: UBc,
    /// F1: 恒久不在ノード (部分観測シナリオ用。実行中の参加はしない)。--add と排他
    absent: Vec<usize>,
    /// F2: ADD参加ノードの初期場の作り方。false = 白紙 (sin初期値、従来どおり)、
    /// true = 両隣の値+勾配から外挿した場を初期値にする (学習型は短い事前学習つき)
    join_warm: bool,
    /// transparent の指数和構成の刻み (実験用。既定 SOE_H=0.6 → M=32。
    /// 0.3 → M=64。誤差床の切り分け用で、既定運用では変えない)
    soe_h: f64,
    /// 発注8: 観測点ナッジングの強度 γ (0.0 = 無効が既定。u ← u + γ(y−u))
    nudge_gamma: f64,
    /// 発注8: 観測周期 (ステップ)。各現存ノードは担当区間の中央格子点を観測する
    obs_period: u64,
    /// 発注8: 合成観測の正規ノイズ σ (観測 y = 基準解 + σ·N(0,1)。シード固定)
    obs_sigma: f64,
    /// 発注8: イノベーション d = y − u (適用前) の時系列CSV出力先
    innovation_csv: Option<String>,
    /// ノード別の温度配列バイト列のCRC32を表示する (実機 status:done の crc32 と
    /// 同一の定義 = f64リトルエンディアン連結のIEEE CRC32。実機照合用)
    crc: bool,
    // ---- 発注11 (実験1準備、2026-09-21) ----
    /// 全域外縁 (x=0, x=L) の境界条件 (設定情報化)。既定 dirichlet:0 = 従来と
    /// ビット同一。実験1 (摂氏運転) では zero_grad を指定する。
    /// dirichlet / zero_grad のみ許可 (transparent等は不在隣接境界の専用)
    outer_bc: UBc,
    /// 空間の全長 L [m] (既定 1.0 = 従来の無次元運転とビット同一)。h = L/(points-1)
    length: f64,
    /// 初期場を一様値で上書き (実験1: 実測平均で初期化)。--add と併用不可
    init_uniform: Option<f64>,
    /// 合成真値を式で生成する物理運転 (リハーサル用):
    /// T(x,t) = tc + (tw−tc)·x/L + amp·sin(2πt/period)·x/L
    /// (一端が暖かい定常勾配+緩い時間変動)。観測 y = T + σN(0,1)。
    /// 指定時は基準解も本式 (モデル誤差込みの検証になる)
    truth_grad: Option<(f64, f64, f64, f64)>, // (tc, tw, period_s, amp)
    /// 保留ノード (抜き取り検証): 観測を同化に使わず (γ=0適用)、
    /// イノベーションのみ記録する。実機の holdout と同一意味論
    holdout: Option<usize>,
}

/// F1: 未観測領域境界の設定キー (docs/2026-09-14-f1-boundary-design.md)
#[derive(Clone, Copy, PartialEq, Debug)]
enum UBc {
    /// 値固定。現行の u=0 は dirichlet(0) として残し回帰比較に用いる
    Dirichlet(f64),
    /// ゼロ勾配 (断熱端)。ゴースト点は鏡映 u_{-1}=u_1。当面の既定 (理論ノートT9)
    ZeroGrad,
    /// 線形外挿。拡散系では端点が時間凍結する (理論ノートT8) — 選択時に警告
    LinearExtrap,
    /// 半無限透過境界 (理論ノートT10)。1/√t核の指数和近似+補助変数の漸化式
    Transparent,
}

impl UBc {
    fn parse(spec: &str) -> Self {
        if let Some(v) = spec.strip_prefix("dirichlet") {
            let v = v.strip_prefix(':').unwrap_or("0");
            return UBc::Dirichlet(v.parse().expect("--unobserved-bc dirichlet:<値>"));
        }
        match spec {
            "zero_grad" => UBc::ZeroGrad,
            "linear_extrap" => UBc::LinearExtrap,
            "transparent" => UBc::Transparent,
            _ => panic!("--unobserved-bc dirichlet[:v] | zero_grad | linear_extrap | transparent"),
        }
    }
    /// アクターへ渡す境界条件コード
    fn code(&self) -> u32 {
        match self {
            UBc::Dirichlet(_) => BC_DIRICHLET,
            UBc::ZeroGrad => BC_ZERO_GRAD,
            UBc::LinearExtrap => BC_LINEAR_EXTRAP,
            UBc::Transparent => BC_TRANSPARENT,
        }
    }
    fn label(&self) -> String {
        match self {
            UBc::Dirichlet(v) => format!("dirichlet({v})"),
            UBc::ZeroGrad => "zero_grad".to_string(),
            UBc::LinearExtrap => "linear_extrap".to_string(),
            UBc::Transparent => "transparent".to_string(),
        }
    }
}

/// M-S1c: 界面残差による自動 GROW (T2) のパラメータ
#[derive(Clone, Copy, Debug)]
struct AutoGrow {
    /// 発火条件: 現在値 > ratio × ベースライン
    ratio: f64,
    /// 現在値 = 直近 window ステップの残差の中央値
    window: usize,
    /// ベースライン = 現在窓の直前 base ステップの残差の中央値
    base: usize,
    /// ベースラインに必要な最小標本数 (未満なら隣の界面で代用)
    min: usize,
    /// 発火後の不応期 (ステップ)
    cooldown: u64,
    /// 1回の GROW で広げる幅
    step: u32,
    /// 幅の上限
    max: u32,
}

impl AutoGrow {
    fn parse(spec: &str) -> Self {
        let mut a = AutoGrow { ratio: 4.0, window: 10, base: 100, min: 20, cooldown: 100, step: 8, max: 32 };
        if spec == "on" || spec.is_empty() {
            return a;
        }
        for kv in spec.split(',') {
            let (k, v) = kv.split_once('=').expect("--auto-grow key=val[,key=val] | on");
            match k {
                "ratio" => a.ratio = v.parse().unwrap(),
                "window" => a.window = v.parse().unwrap(),
                "base" => a.base = v.parse().unwrap(),
                "min" => a.min = v.parse().unwrap(),
                "cooldown" => a.cooldown = v.parse().unwrap(),
                "step" => a.step = v.parse().unwrap(),
                "max" => a.max = v.parse().unwrap(),
                _ => panic!("--auto-grow: unknown key {k}"),
            }
        }
        assert!(a.window >= 1 && a.min >= 1 && a.step >= 1);
        a
    }
}

/// --pinn のキー名 → dyne_cfg のキー番号 (heat1d-pinn-actor の CFG_* と一致させる)
fn pinn_key(name: &str) -> u32 {
    match name {
        "iters" => 0,
        "lr" => 1,
        "warm" => 2,
        "width" => 3,
        "depth" => 4,
        "lambda_b" => 5,
        "seed" => 6,
        "warm_resize" => 7,
        "lambda_g" => 8,
        "opt" => 9,
        "lr_step" => 10,
        "momentum" => 11,
        "iface" => 12,
        "sub" => 13,
        _ => panic!("--pinn: unknown key {name}"),
    }
}

fn parse_args() -> Config {
    let mut c = Config {
        wasm: String::new(),
        nodes: 16,
        points: 65,
        steps: 2000,
        alpha: 1.0,
        dt: 1e-4,
        loss: 0.0,
        lat_us: 0,
        lat_jitter_us: 0,
        step_us: 79_500, // M1実測: 79.5ms/step (Classic通訳, 240MHz)
        timeout_us: 100_000,
        resend_min_us: 20_000, // 再送バックオフ(M1知見4)
        seed: 42,
        ram_cap_bytes: 250 * 1024, // ESP32実測ベースの実効上限
        pauses: Vec::new(),
        handovers: Vec::new(),
        partitions: Vec::new(),
        adds: Vec::new(),
        residual_csv: None,
        pinn_cfg: Vec::new(),
        add_warm: None,
        resizes: Vec::new(),
        error_csv: None,
        auto_grow: None,
        event_csv: None,
        field_csv: None,
        field_every: 10,
        quiet: false,
        unobserved_bc: UBc::ZeroGrad, // 既定 (2026-09-14 ユーザー決定。理論ノートT9)
        absent: Vec::new(),
        join_warm: false, // 既定は白紙 (dirichlet(0)指定だけで既存回帰とビット一致を保つ)
        soe_h: dyne_core::transparent::SOE_H,
        outer_bc: UBc::Dirichlet(0.0),
        length: 1.0,
        init_uniform: None,
        truth_grad: None,
        holdout: None,
        nudge_gamma: 0.0,
        obs_period: 10,
        obs_sigma: 0.0,
        innovation_csv: None,
        crc: false,
    };
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        let key = args[i].as_str();
        let val = || args.get(i + 1).cloned().unwrap_or_default();
        match key {
            "--wasm" => c.wasm = val(),
            "--nodes" => c.nodes = val().parse().unwrap(),
            "--points" => c.points = val().parse().unwrap(),
            "--steps" => c.steps = val().parse().unwrap(),
            "--alpha" => c.alpha = val().parse().unwrap(),
            "--dt" => c.dt = val().parse().unwrap(),
            "--loss" => c.loss = val().parse().unwrap(),
            "--lat-ms" => c.lat_us = (val().parse::<f64>().unwrap() * 1000.0) as u64,
            "--lat-jitter-ms" => c.lat_jitter_us = (val().parse::<f64>().unwrap() * 1000.0) as u64,
            "--step-ms" => c.step_us = (val().parse::<f64>().unwrap() * 1000.0) as u64,
            "--timeout-ms" => c.timeout_us = (val().parse::<f64>().unwrap() * 1000.0) as u64,
            "--seed" => c.seed = val().parse().unwrap(),
            "--ram-cap-kb" => c.ram_cap_bytes = val().parse::<u64>().unwrap() * 1024,
            "--handover" => {
                let v = val();
                let (n, rest) = v.split_once('@').expect("--handover node@leave:rejoin");
                let (a, b) = rest.split_once(':').expect("--handover node@leave:rejoin");
                c.handovers.push((
                    n.parse().unwrap(),
                    a.parse::<u64>().unwrap() * 1000,
                    b.parse::<u64>().unwrap() * 1000,
                ));
            }
            "--add" => {
                // node@t_ms: ノードを仮想時刻tにβ意味論で新規参加させる
                let v = val();
                let (n, t) = v.split_once('@').expect("--add node@t_ms");
                c.adds
                    .push((n.parse().unwrap(), t.parse::<u64>().unwrap() * 1000));
            }
            "--residual-csv" => c.residual_csv = Some(val()),
            "--error-csv" => c.error_csv = Some(val()),
            "--event-csv" => c.event_csv = Some(val()),
            "--field-csv" => c.field_csv = Some(val()),
            "--field-every" => c.field_every = val().parse().unwrap(),
            "--auto-grow" => c.auto_grow = Some(AutoGrow::parse(&val())),
            "--pinn" => {
                // key=val[,key=val...]: 学習型アクターのハイパーパラメータ
                for kv in val().split(',') {
                    let (k, v) = kv.split_once('=').expect("--pinn key=val[,key=val]");
                    c.pinn_cfg.push((pinn_key(k), v.parse().expect("--pinn value")));
                }
            }
            "--add-warm" => c.add_warm = Some(val().parse().unwrap()),
            "--resize" => {
                // node@t_ms:width: 仮想時刻 t に隠れ層の幅を変更 (GROW/SHRINK)
                let v = val();
                let (n, rest) = v.split_once('@').expect("--resize node@t_ms:width");
                let (t, w) = rest.split_once(':').expect("--resize node@t_ms:width");
                c.resizes.push((
                    n.parse().unwrap(),
                    t.parse::<u64>().unwrap() * 1000,
                    w.parse().unwrap(),
                ));
            }
            "--pause" => {
                // node@start_ms:end_ms
                let v = val();
                let (n, rest) = v.split_once('@').expect("--pause node@start:end");
                let (a, b) = rest.split_once(':').expect("--pause node@start:end");
                c.pauses.push((
                    n.parse().unwrap(),
                    a.parse::<u64>().unwrap() * 1000,
                    b.parse::<u64>().unwrap() * 1000,
                ));
            }
            "--partition" => {
                let v = val();
                let (k, rest) = v.split_once('@').expect("--partition k@start:end");
                let (a, b) = rest.split_once(':').expect("--partition k@start:end");
                c.partitions.push((
                    k.parse().unwrap(),
                    a.parse::<u64>().unwrap() * 1000,
                    b.parse::<u64>().unwrap() * 1000,
                ));
            }
            "--unobserved-bc" => c.unobserved_bc = UBc::parse(&val()),
            "--soe-h" => c.soe_h = val().parse().expect("--soe-h <正の実数>"),
            "--outer-bc" => c.outer_bc = UBc::parse(&val()),
            "--length" => c.length = val().parse().expect("--length <L[m]>"),
            "--init-uniform" => c.init_uniform = Some(val().parse().expect("--init-uniform <T>")),
            "--truth-grad" => {
                // tc,tw,period_s,amp
                let v = val();
                let ps: Vec<f64> = v.split(',').map(|x| x.parse().expect("--truth-grad tc,tw,period,amp")).collect();
                assert!(ps.len() == 4, "--truth-grad tc,tw,period_s,amp");
                c.truth_grad = Some((ps[0], ps[1], ps[2], ps[3]));
            }
            "--holdout" => c.holdout = Some(val().parse().expect("--holdout <node>")),
            "--nudge-gamma" => c.nudge_gamma = val().parse().expect("--nudge-gamma <γ>"),
            "--obs-period" => c.obs_period = val().parse().expect("--obs-period <ステップ>"),
            "--obs-sigma" => c.obs_sigma = val().parse().expect("--obs-sigma <σ>"),
            "--innovation-csv" => c.innovation_csv = Some(val()),
            "--absent" => {
                // 恒久不在ノードのカンマ区切りリスト (部分観測シナリオ)
                c.absent = val().split(',').map(|s| s.parse().expect("--absent n,n,...")).collect();
            }
            "--join-init" => {
                c.join_warm = match val().as_str() {
                    "blank" => false,
                    "warm" => true,
                    _ => panic!("--join-init blank | warm"),
                };
            }
            "--crc" => {
                c.crc = true;
                i -= 1;
            }
            "--quiet" => {
                c.quiet = true;
                i -= 1;
            }
            _ => panic!("unknown arg: {key}"),
        }
        i += 2;
    }
    assert!(!c.wasm.is_empty(), "--wasm is required");
    assert!(c.nodes >= 1);
    for &(a, _) in &c.adds {
        assert!(a < c.nodes, "--add: node {a} out of range");
        // 隣接ノード同士のADDは、参加時のbc決定が適用順に依存するため未対応
        assert!(
            !c.adds.iter().any(|&(b, _)| b + 1 == a || a + 1 == b),
            "--add: 隣接ノード同士のADDは未対応 (node {a})"
        );
    }
    if !c.absent.is_empty() {
        for &a in &c.absent {
            assert!(a < c.nodes, "--absent: node {a} out of range");
        }
        assert!(c.absent.len() < c.nodes, "--absent: 全ノード不在は不可");
        // 部分観測は誤差計測 (全域基準解との比較) の意味を単純に保つため --add と排他
        assert!(c.adds.is_empty(), "--absent は --add と併用不可 (部分観測シナリオ専用)");
        for &(n, _, _) in c.pauses.iter().chain(&c.handovers) {
            assert!(!c.absent.contains(&n), "--absent のノード {n} は pause/handover 対象にできない");
        }
        for &(n, _, _) in &c.resizes {
            assert!(!c.absent.contains(&n), "--absent のノード {n} は resize 対象にできない");
        }
    }
    assert!(
        matches!(c.outer_bc, UBc::Dirichlet(_) | UBc::ZeroGrad),
        "--outer-bc は dirichlet[:値] か zero_grad のみ (それ以外は不在隣接境界専用)"
    );
    assert!(c.length > 0.0, "--length は正の値");
    if c.init_uniform.is_some() {
        assert!(c.adds.is_empty(), "--init-uniform は --add と併用不可 (複合参照解が sin 初期化前提)");
    }
    if c.truth_grad.is_some() {
        // γ=0 も許す (2026-09-23: 同化なし対照run = イノベーション記録のみ)
        let (_, _, period, _) = c.truth_grad.unwrap();
        assert!(period > 0.0, "--truth-grad の period は正の値");
    }
    if let Some(hn) = c.holdout {
        assert!(hn < c.nodes, "--holdout: node {hn} out of range");
        assert!(
            c.nudge_gamma > 0.0 || c.truth_grad.is_some(),
            "--holdout は --nudge-gamma > 0 か --truth-grad と併用"
        );
    }
    if c.nudge_gamma != 0.0 {
        assert!(c.nudge_gamma > 0.0, "--nudge-gamma は正の値");
        assert!(c.obs_period >= 1, "--obs-period は1以上");
        // 最小形: 観測=単一ノード基準解ベース。領域拡張 (--add) との併用は未対応
        assert!(c.adds.is_empty(), "--nudge-gamma は --add と併用不可 (最小形)");
    }
    c
}

// ---------- メッセージとイベント ----------
// (決定論的乱数 Rng とアクター実行ラッパ Actor は dyne-core へ移設)

/// 界面メッセージ。iface はクラスタ全体で一意な界面ID(ノードjとj+1の間 = j)。
/// M0の「送信者から見た面」方式を廃止(M0知見3)。
#[derive(Clone, Copy, Debug)]
struct Msg {
    iface: usize,
    step: u64,
    value: f64,
    grad: f64,
}

enum Ev {
    Deliver { node: usize, msg: Msg },
    Wake { node: usize },
    StepDone { node: usize },
}

// ---------- ノード状態機械 ----------

#[derive(PartialEq)]
enum Phase {
    /// step の境界値を待っている(送信済み)
    Waiting { step: u64 },
    /// 計算中(StepDoneイベントで完了)
    Computing,
    Done,
}

struct Node {
    actor: Actor,
    /// dyne_init の引数一式 (handover 復帰時の新インスタンス構築用)
    init_args: (u32, f64, f64, u32, u32),
    /// (界面ID, その界面の自分側の面)
    ifaces: Vec<(usize, u32)>,
    /// β意味論ADD: このノードがまだ参加していない間 false。イベントは発生しない。
    present: bool,
    /// 界面の動的登録の予約: (界面ID, 自分側の面, 有効化ステップ)。
    /// 有効化ステップの start_step 冒頭で、bcの切り替え (再init+状態復元) と
    /// ifaces への登録を行う (隣接ノードにADDが起きた側で使う)。
    iface_adds: Vec<(usize, u32, u64)>,
    phase: Phase,
    cur_step: u64,
    /// 現ステップで受領済みの界面
    got: Vec<bool>,
    /// 現ステップで受領した相手側勾配 (界面残差の計測用。got と同じ添字)
    recv_grad: Vec<f64>,
    /// 現ステップで受領した相手側の値 (F2ウォームスタートの初期場構成用。got と同じ添字)
    recv_val: Vec<f64>,
    /// F1: 恒久不在 (部分観測シナリオ)。参加イベントは起きない
    permanent_absent: bool,
    /// F2: 参加直後の最初のステップで、受領した両隣の値+勾配から初期場を構成する予約。
    /// 予約中は自分の送信を初期場の構成後まで遅らせる (構成前の白紙値を送らないため)
    warm_join_pending: bool,
    /// 先着した将来ステップ: (iface,step) -> msg
    pending: HashMap<(usize, u64), Msg>,
    /// 送信控え(直近2ステップ): (iface,step) -> msg
    sent: HashMap<(usize, u64), Msg>,
    last_resend_us: u64,
    // 統計
    stat_sent: u64,
    stat_lost: u64,
    stat_resend: u64,
    stat_replay: u64,
    wait_us: u64,
    wait_since: u64,
}

// ---------- シミュレータ本体 ----------

/// ADD参加の記録 (複合参照解と計測レポートの入力)
#[derive(Clone, Copy, Debug)]
struct JoinRec {
    node: usize,
    /// 参加ステップ S: このステップからノードは計算に加わる
    step: u64,
    /// 参加時に dyne_init へ渡した境界条件 (複合参照解の初期化に使う)
    bc: (u32, u32),
    t_us: u64,
    /// F2: ウォームスタート参加 (複合参照解の編入時の初期場構成に使う)
    warm: bool,
}

/// GROW/SHRINK の適用記録 (レポート用)
#[derive(Clone, Debug)]
struct ResizeRec {
    node: usize,
    t_us: u64,
    step: u64,
    width: u32,
    /// 変更直後にモデル出力がビット単位で保たれた格子点数 (GROW は全点保存が期待値)
    preserved: usize,
    n: usize,
}

/// 自動 GROW (T2) の発火記録
#[derive(Clone, Debug)]
struct AutoRec {
    step: u64,
    t_us: u64,
    iface: usize,
    node: usize,
    from: u32,
    to: u32,
    current: f64,
    baseline: f64,
}

/// 界面ごとのトリガー状態 (ヒステリシス)
#[derive(Clone, Copy, Debug)]
struct TrigState {
    last_fire: Option<u64>,
    armed: bool,
}

/// ダッシュボード向けのイベント
#[derive(Clone, Debug)]
struct EventRec {
    kind: &'static str,
    step: u64,
    t_us: u64,
    node: usize,
    iface: Option<usize>,
    width: Option<u32>,
    current: f64,
    baseline: f64,
}

struct Sim {
    cfg: Config,
    nodes: Vec<Node>,
    /// 界面ID j はノード j と j+1 を結ぶ
    now_us: u64,
    /// 離散イベントキュー (dyne-core::event へ移設)
    queue: EventQueue<Ev>,
    /// 仮想ネットワーク (dyne-core::net へ移設。損失・遅延・分断と乱数を保有)
    net: VirtNet,
    engine: wasmi::Engine,
    module: wasmi::Module,
    /// 実行中に確定したADD参加の記録
    join_log: Vec<JoinRec>,
    /// 界面残差 |grad_A - grad_B| の時系列: 界面ID -> (step, 残差)。
    /// 界面の左側ノードが計算開始直前に記録する (M-S1cのT2トリガーのセンサー前倒し)。
    residuals: HashMap<usize, Vec<(u64, f64)>>,
    /// M-S1b: GROW/SHRINK の適用記録
    resize_log: Vec<ResizeRec>,
    /// --error-csv 指定時: ステップ完了ごとの各ノードの状態 (node, step, u[])
    snap_log: Vec<(usize, u64, Vec<f64>)>,
    /// M-S1c: ノードごとの現在の隠れ層幅 (学習型のみ意味を持つ。handover 復帰時に引き継ぐ)
    width: Vec<u32>,
    /// M-S1c: 界面ごとのトリガー状態
    trig: HashMap<usize, TrigState>,
    /// M-S1c: 自動 GROW の発火記録
    auto_log: Vec<AutoRec>,
    /// ダッシュボード向けイベント時系列
    event_log: Vec<EventRec>,
    /// --field-csv 指定時: (node, step, x[], u[])
    field_log: Vec<(usize, u64, Vec<f64>, Vec<f64>)>,
    // ---- F1/F2 (発注3) ----
    /// 格子の分割 (node -> (開始添字, 点数))。F2の初期場構成で座標を引くのに使う
    starts: Vec<(usize, usize)>,
    /// グローバル格子幅
    h_global: f64,
    /// 透過境界の指数和係数 (unobserved_bc = transparent のときのみ Some)
    t_coef: Option<TransparentCoef>,
    /// handover による透過境界の補助変数リセットの記録: (node, 有効ステップ)。
    /// 複合参照解が同じ位置でリセットする (担当替え = 透過境界の再発動、理論ノートT10)
    reset_log: Vec<(usize, u64)>,
    // ---- 発注8: 観測点ナッジング ----
    /// 合成観測の真値 = 単一ノード基準解の全履歴 [step][gi] (γ>0 のときのみ)
    obs_truth: Vec<Vec<f64>>,
    /// 観測ノイズ用の決定論的乱数 (通信の乱数とは独立の系列)
    obs_rng: dyne_core::rng::Rng,
    /// イノベーションの記録: (node, step, 大域格子点, 観測値y, d=y−u適用前)
    innovation_log: Vec<(usize, u64, usize, f64, f64)>,
}

/// F1: 未観測領域境界の面ごとの設定をアクターへ注入する。dyne_init の**前**に呼ぶ
/// (init 時の Dirichlet 端の丸め落としが bc_value を使うため)。
/// - 不在隣接に接する Dirichlet 面 (グローバル外縁は対象外) へ値を渡す
/// - transparent の面があれば指数和係数を全項渡す (係数は init をまたいで保持される)
fn apply_face_config(
    actor: &mut Actor,
    node: usize,
    n_nodes: usize,
    bcl: u32,
    bcr: u32,
    ubc: &UBc,
    outer: &UBc,
    coef: &Option<TransparentCoef>,
) {
    for (face, code, global_edge) in
        [(FACE_LEFT, bcl, node == 0), (FACE_RIGHT, bcr, node + 1 == n_nodes)]
    {
        if code == BC_DIRICHLET {
            // 面の由来で値の出所を選ぶ: 外縁 = --outer-bc、不在隣接 = --unobserved-bc
            let v = if global_edge {
                if let UBc::Dirichlet(v) = outer { *v } else { 0.0 }
            } else if let UBc::Dirichlet(v) = ubc {
                *v
            } else {
                0.0
            };
            if v != 0.0 {
                if let Some(f) = actor.set_bc_value {
                    f.call(&mut actor.store, (face, v)).unwrap();
                }
            }
        }
    }
    if bcl == BC_TRANSPARENT || bcr == BC_TRANSPARENT {
        let c = coef.as_ref().expect("transparent係数が未計算");
        let f = actor
            .transparent_coef
            .expect("このアクターは dyne_transparent_coef を持たない (transparent 非対応)");
        for j in 0..c.decay.len() {
            f.call(&mut actor.store, (j as u32, c.decay[j], c.half[j], c.weight[j]))
                .unwrap();
        }
    }
}

/// F2: 参加ノードの初期場 (両隣の値+勾配からの外挿)。
/// 両隣あり = 3次エルミート補間 (両端の値と勾配の4条件を厳密に満たす)、
/// 片側のみ = その側からの線形延長。1回きりの初期化なので線形外挿の端点凍結
/// (理論ノートT8) の問題は生じない (2026-09-14 検収仕様の明確化)。
/// シミュレータ本体と複合参照解の両方がこの関数を使う (ビット一致の前提)。
fn warm_field(
    left: Option<(f64, f64, f64)>,  // (x, 値, 勾配) 左隣の境界
    right: Option<(f64, f64, f64)>, // 同 右隣
    x: f64,
) -> f64 {
    match (left, right) {
        (Some((xa, va, ga)), Some((xb, vb, gb))) => {
            let d = xb - xa;
            let t = (x - xa) / d;
            let omt = 1.0 - t;
            let h00 = (1.0 + 2.0 * t) * omt * omt;
            let h10 = t * omt * omt;
            let h01 = t * t * (3.0 - 2.0 * t);
            let h11 = t * t * (t - 1.0);
            h00 * va + h10 * d * ga + h01 * vb + h11 * d * gb
        }
        (Some((xa, va, ga)), None) => va + ga * (x - xa),
        (None, Some((xb, vb, gb))) => vb + gb * (x - xb),
        (None, None) => 0.0, // 参加ノードは必ず現存する隣接を持つため起こらない
    }
}

impl Sim {
    /// 時刻を過ぎた GROW/SHRINK (--resize) を適用する。
    /// 適用前後で dyne_eval を比較し、機能保存 (ビット一致点数) を記録する。
    fn apply_resizes(&mut self) {
        let due: Vec<(usize, u64, u32)> = self
            .cfg
            .resizes
            .iter()
            .filter(|&&(_, t, _)| self.now_us >= t)
            .copied()
            .collect();
        if due.is_empty() {
            return;
        }
        let now = self.now_us;
        self.cfg.resizes.retain(|&(_, t, _)| now < t);
        for (node, t, width) in due {
            assert!(self.nodes[node].present, "--resize: node {node} is absent");
            if let Some(r) = self.do_resize(node, width) {
                self.event_log.push(EventRec {
                    kind: "resize", step: r.step, t_us: t, node, iface: None,
                    width: Some(width), current: 0.0, baseline: 0.0,
                });
                self.resize_log.push(ResizeRec { t_us: t, ..r });
            }
        }
    }

    /// 幅変更を実行し、前後の機能保存 (ビット一致点数) を記録して返す。
    /// dyne_resize を持たないアクター (差分法) では None。
    fn do_resize(&mut self, node: usize, width: u32) -> Option<ResizeRec> {
        {
            let nd = &mut self.nodes[node];
            let (Some(resize), Some(eval)) = (nd.actor.resize, nd.actor.eval) else {
                eprintln!("[sim] resize ignored: node {node} のアクターは dyne_resize を持たない");
                return None;
            };
            let n = nd.init_args.0;
            let before: Vec<f64> = (0..n)
                .map(|i| eval.call(&mut nd.actor.store, i).unwrap())
                .collect();
            let rc = resize.call(&mut nd.actor.store, width).unwrap();
            assert_eq!(rc, 0, "dyne_resize({width}) failed on node {node}");
            let after: Vec<f64> = (0..n)
                .map(|i| eval.call(&mut nd.actor.store, i).unwrap())
                .collect();
            let preserved = before
                .iter()
                .zip(&after)
                .filter(|(a, b)| a.to_bits() == b.to_bits())
                .count();
            let step = nd.cur_step;
            self.width[node] = width;
            eprintln!(
                "[sim] resize applied: node {} width -> {} at t={}ms (step {}), model output preserved {}/{} pts bit-exact",
                node,
                width,
                self.now_us / 1000,
                step,
                preserved,
                n
            );
            Some(ResizeRec { node, t_us: self.now_us, step, width, preserved, n: n as usize })
        }
    }

    /// M-S1c T2: 界面 j の残差が更新された直後に呼ぶ。移動窓と移動ベースラインを比較し、
    /// 閾値超過なら界面を挟む両ノードを GROW する。使う情報は界面 j (と履歴不足時は隣の
    /// 界面) の残差だけ = 界面の両側ノードが自分で持てる局所情報。
    fn check_trigger(&mut self, j: usize, step: u64) {
        let Some(ag) = self.cfg.auto_grow else { return };
        let series = &self.residuals[&j];
        if series.len() < ag.window {
            return;
        }
        let cur = {
            let mut w: Vec<f64> = series[series.len() - ag.window..].iter().map(|&(_, r)| r).collect();
            median(&mut w)
        };
        // ベースライン窓 = 現在窓の直前 base 標本
        let base_of = |v: &Vec<(u64, f64)>| -> Option<f64> {
            if v.len() < ag.window + ag.min {
                return None;
            }
            let hi = v.len() - ag.window;
            let lo = hi.saturating_sub(ag.base);
            let mut b: Vec<f64> = v[lo..hi].iter().map(|&(_, r)| r).collect();
            Some(median(&mut b))
        };
        let baseline = match base_of(series) {
            Some(b) => b,
            None => {
                // 新界面 (参加直後): 隣の界面 j-1, j+1 のベースラインの中央値で代用
                let mut nb: Vec<f64> = [j.checked_sub(1), Some(j + 1)]
                    .into_iter()
                    .flatten()
                    .filter_map(|k| self.residuals.get(&k).and_then(base_of))
                    .collect();
                if nb.is_empty() {
                    return;
                }
                median(&mut nb)
            }
        };
        if baseline <= 0.0 {
            return;
        }
        let st = self.trig.entry(j).or_insert(TrigState { last_fire: None, armed: true });
        if let Some(lf) = st.last_fire {
            if step < lf + ag.cooldown {
                return;
            }
        }
        if !st.armed {
            if cur < 0.5 * ag.ratio * baseline {
                st.armed = true;
            }
            return;
        }
        if cur <= ag.ratio * baseline {
            return;
        }
        // 発火: 界面 j の両側 (ノード j, j+1) を GROW
        st.armed = false;
        st.last_fire = Some(step);
        for node in [j, j + 1] {
            if node >= self.nodes.len() || !self.nodes[node].present {
                continue;
            }
            let from = self.width[node];
            if from >= ag.max {
                eprintln!("[sim] auto-grow: iface {j} step {step} node {node} は幅上限 {} のため見送り", ag.max);
                continue;
            }
            let to = (from + ag.step).min(ag.max);
            if let Some(r) = self.do_resize(node, to) {
                eprintln!(
                    "[sim] auto-grow fired: iface {j} step {step} current={cur:.3e} baseline={baseline:.3e} (x{:.1}) -> node {node} width {from} -> {to}",
                    cur / baseline
                );
                self.auto_log.push(AutoRec { step, t_us: self.now_us, iface: j, node, from, to, current: cur, baseline });
                self.event_log.push(EventRec {
                    kind: "auto_grow", step, t_us: self.now_us, node, iface: Some(j),
                    width: Some(to), current: cur, baseline,
                });
                self.resize_log.push(r);
            }
        }
    }

    fn schedule(&mut self, t: u64, ev: Ev) {
        self.queue.schedule(t, ev);
    }

    fn in_pause(&self, node: usize, t: u64) -> Option<u64> {
        self.cfg
            .pauses
            .iter()
            .find(|&&(n, a, b)| n == node && t >= a && t < b)
            .map(|&(_, _, b)| b)
            .or_else(|| {
                self.cfg
                    .handovers
                    .iter()
                    .find(|&&(n, a, b)| n == node && t >= a && t < b)
                    .map(|&(_, _, b)| b)
            })
    }

    /// 復帰時刻を過ぎた handover 対象を、新インスタンスへ移植する。
    /// 旧インスタンスから u[] を読み出し、新インスタンスに dyne_init + dyne_set。
    /// プロトコル状態 (現在ステップ・送信控え・取り置き) はホスト側が保持継続する。
    fn apply_handovers(&mut self) {
        let due: Vec<usize> = self
            .cfg
            .handovers
            .iter()
            .filter(|&&(_, _, b)| self.now_us >= b)
            .map(|&(n, _, _)| n)
            .collect();
        if due.is_empty() {
            return;
        }
        // 一度処理した handover は除去して再適用を防ぐ
        let now = self.now_us;
        self.cfg.handovers.retain(|&(_, _, b)| now < b);
        for node in due {
            let n_pts;
            let mut saved = Vec::new();
            {
                let nd = &mut self.nodes[node];
                n_pts = nd.init_args.0;
                for i in 0..n_pts {
                    let v = nd.actor.get.call(&mut nd.actor.store, i).unwrap();
                    saved.push(v);
                }
            }
            // 新インスタンス構築 (旧インスタンスは破棄 = REMOVE、新設 = ADD)
            let mut actor = Actor::load_with(&self.engine, &self.module, &self.cfg.pinn_cfg);
            // M-S1c: GROW 済みの幅は引き継ぐ (学習型のみ。差分法では dyne_cfg が無く無視)
            if let Some(f) = actor.cfg {
                f.call(&mut actor.store, (3, self.width[node] as f64)).unwrap();
            }
            let (n, x0, x1, bcl, bcr) = self.nodes[node].init_args;
            apply_face_config(
                &mut actor, node, self.nodes.len(), bcl, bcr,
                &self.cfg.unobserved_bc, &self.cfg.outer_bc, &self.t_coef,
            );
            let rc = actor
                .init
                .call(
                    &mut actor.store,
                    (n, x0, x1, self.cfg_alpha(), self.cfg_dt(), bcl, bcr),
                )
                .unwrap();
            assert_eq!(rc, 0, "handover re-init failed");
            // 透過境界の補助変数は新インスタンスでリセット = 担当替えの再発動
            // (理論ノートT10)。複合参照解にも同じ位置でのリセットを伝える。
            // 有効ステップ: 計算中なら現ステップの結果は旧インスタンスで確定済み
            // なので次ステップから、待機中なら現ステップから
            if bcl == BC_TRANSPARENT || bcr == BC_TRANSPARENT {
                let s = self.nodes[node].cur_step
                    + if matches!(self.nodes[node].phase, Phase::Computing) { 1 } else { 0 };
                self.reset_log.push((node, s));
            }
            for (i, v) in saved.iter().enumerate() {
                actor
                    .set
                    .call(&mut actor.store, (i as u32, *v))
                    .unwrap();
            }
            self.nodes[node].actor = actor;
            // 受領済み記録をリセット: 受領値は旧インスタンスのゴースト点にあり
            // 新インスタンスには存在しない。falseに戻せば、繰り延べられていた
            // Wakeが自ステップを再送し、相手の応答再送(replay)がゴーストを
            // 新インスタンスへ再注入する (プロトコル本来の治癒力で回収)。
            if matches!(self.nodes[node].phase, Phase::Waiting { .. }) {
                for g in self.nodes[node].got.iter_mut() {
                    *g = false;
                }
            }
            let st = self.nodes[node].cur_step;
            self.event_log.push(EventRec {
                kind: "handover", step: st, t_us: self.now_us, node, iface: None,
                width: None, current: 0.0, baseline: 0.0,
            });
            eprintln!(
                "[sim] handover applied: node {} reincarnated at t={}ms (state {} pts carried)",
                node,
                self.now_us / 1000,
                n_pts
            );
        }
    }

    /// 参加時刻を過ぎたADD (シナリオC / 解釈β) を適用する。
    /// - 参加ステップ S = 1 + max(現存する隣接ノードの現在ステップ)。
    ///   これにより隣接ノードはまだステップ S の交換を始めていないことが保証される。
    ///   (シミュレータ内では大域情報で揃える。実システムでの合わせ方はM3以降の課題)
    /// - 参加ノード: 新インスタンスを dyne_init (現存隣接の面=BC_INTERFACE、
    ///   外縁の面=BC_DIRICHLET_ZERO) し、初期条件 sin(pi x) でステップ S を開始する
    ///   (引き継ぐ過去なし = B-16のADDの意味論)。
    /// - 隣接ノード: 界面の動的登録を予約 (iface_adds)。ステップ S の開始時に
    ///   bc切り替え (再init+状態復元 = handoverと同じ機構の再利用) と界面登録を行う。
    fn apply_adds(&mut self) {
        let due: Vec<(usize, u64)> = self
            .cfg
            .adds
            .iter()
            .filter(|&&(_, t)| self.now_us >= t)
            .copied()
            .collect();
        if due.is_empty() {
            return;
        }
        let now = self.now_us;
        self.cfg.adds.retain(|&(_, t)| now < t);
        for (node, t_add) in due {
            assert!(
                !self.nodes[node].present,
                "--add: node {node} is already present"
            );
            let neighbors: Vec<usize> = [node.checked_sub(1), Some(node + 1)]
                .into_iter()
                .flatten()
                .filter(|&p| p < self.nodes.len() && self.nodes[p].present)
                .collect();
            let s_join = 1 + neighbors
                .iter()
                .map(|&p| {
                    assert!(
                        self.nodes[p].phase != Phase::Done,
                        "--add: node {node} の参加が遅すぎる (隣接ノード {p} は既に完了)"
                    );
                    self.nodes[p].cur_step
                })
                .max()
                .unwrap_or(0);
            assert!(
                s_join < self.cfg.steps,
                "--add: 参加ステップ {s_join} が総ステップ数以上"
            );
            // 参加ノードのbcと界面を決め、隣接ノードへ界面の動的登録を予約する
            // (界面にならない面はグローバル外縁のみ → --outer-bc に従う。既定は従来同一)
            let oc = self.cfg.outer_bc.code();
            let mut bc = [oc, oc];
            let mut ifaces = Vec::new();
            if node > 0 && self.nodes[node - 1].present {
                bc[0] = BC_INTERFACE;
                ifaces.push((node - 1, FACE_LEFT));
                self.nodes[node - 1]
                    .iface_adds
                    .push((node - 1, FACE_RIGHT, s_join));
            }
            if node + 1 < self.nodes.len() && self.nodes[node + 1].present {
                bc[1] = BC_INTERFACE;
                ifaces.push((node, FACE_RIGHT));
                self.nodes[node + 1]
                    .iface_adds
                    .push((node, FACE_LEFT, s_join));
            }
            let mut actor = Actor::load_with(&self.engine, &self.module, &self.cfg.pinn_cfg);
            // 学習型アクター: ADDで参加するインスタンスの温め回数。
            // F2ウォームスタートの既定は 200 (2026-09-14 検収仕様)。--add-warm で上書き可
            let warm_iters = if self.cfg.join_warm {
                Some(self.cfg.add_warm.unwrap_or(200.0))
            } else {
                self.cfg.add_warm
            };
            if let (Some(w), Some(f)) = (warm_iters, actor.cfg) {
                f.call(&mut actor.store, (2, w)).unwrap();
            }
            let (n, x0, x1, _, _) = self.nodes[node].init_args;
            apply_face_config(
                &mut actor, node, self.nodes.len(), bc[0], bc[1],
                &self.cfg.unobserved_bc, &self.cfg.outer_bc, &self.t_coef,
            );
            let rc = actor
                .init
                .call(
                    &mut actor.store,
                    (n, x0, x1, self.cfg_alpha(), self.cfg_dt(), bc[0], bc[1]),
                )
                .unwrap();
            assert_eq!(rc, 0, "add init failed");
            {
                let nd = &mut self.nodes[node];
                nd.actor = actor;
                nd.init_args = (n, x0, x1, bc[0], bc[1]);
                nd.ifaces = ifaces;
                nd.present = true;
                nd.warm_join_pending = self.cfg.join_warm;
            }
            self.join_log.push(JoinRec {
                node,
                step: s_join,
                bc: (bc[0], bc[1]),
                t_us: t_add,
                warm: self.cfg.join_warm,
            });
            self.event_log.push(EventRec {
                kind: "join", step: s_join, t_us: self.now_us, node, iface: None,
                width: None, current: 0.0, baseline: 0.0,
            });
            eprintln!(
                "[sim] add applied: node {} joined at t={}ms as step {} (bc=[{},{}], β=領域拡張)",
                node,
                self.now_us / 1000,
                s_join,
                bc[0],
                bc[1]
            );
            self.start_step(node, s_join);
        }
    }

    /// 予約済みの界面の動的登録を適用する (start_step 冒頭、送信より前に呼ぶ)。
    /// アクターのbcは初期化時固定なので、再init+状態復元で境界条件を切り替える。
    /// u[] は保存→復元するので物理状態は不変 (運ぶのは物理状態のみの原則)。
    fn apply_iface_adds(&mut self, node: usize, step: u64) {
        let due: Vec<(usize, u32)> = self.nodes[node]
            .iface_adds
            .iter()
            .filter(|&&(_, _, s)| s <= step)
            .map(|&(j, f, _)| (j, f))
            .collect();
        if due.is_empty() {
            return;
        }
        self.nodes[node].iface_adds.retain(|&(_, _, s)| s > step);
        let (n, x0, x1, mut bcl, mut bcr) = self.nodes[node].init_args;
        let mut saved = Vec::new();
        {
            let nd = &mut self.nodes[node];
            for i in 0..n {
                saved.push(nd.actor.get.call(&mut nd.actor.store, i).unwrap());
            }
        }
        for &(_, f) in &due {
            if f == FACE_LEFT {
                bcl = BC_INTERFACE;
            } else {
                bcr = BC_INTERFACE;
            }
        }
        if let Some(set_bc) = self.nodes[node].actor.set_bc {
            // 学習型アクター: 重み (学習結果) を保ったまま境界条件だけ切り替える。
            // 再init+状態復元だと重みが乱数初期値に戻り、標本からの温め直しになる。
            let nd = &mut self.nodes[node];
            for &(_, f) in &due {
                set_bc.call(&mut nd.actor.store, (f, BC_INTERFACE)).unwrap();
            }
            nd.init_args = (n, x0, x1, bcl, bcr);
            for &(j, f) in &due {
                nd.ifaces.push((j, f));
            }
            eprintln!(
                "[sim] iface registered (in-place bc switch, weights kept): node {} interface {:?} from step {} (t={}ms)",
                node,
                due.iter().map(|&(j, _)| j).collect::<Vec<_>>(),
                step,
                self.now_us / 1000
            );
            return;
        }
        let mut actor = Actor::load_with(&self.engine, &self.module, &self.cfg.pinn_cfg);
        // 再initで透過境界の補助変数はリセットされる (残っている未観測面も含む)。
        // これは担当替えのリセット規定 (理論ノートT10) の再init側の帰結で、
        // 複合参照解も参加イベント時に隣接ノードの補助変数をリセットして鏡写しする
        apply_face_config(
            &mut actor, node, self.nodes.len(), bcl, bcr,
            &self.cfg.unobserved_bc, &self.cfg.outer_bc, &self.t_coef,
        );
        let rc = actor
            .init
            .call(
                &mut actor.store,
                (n, x0, x1, self.cfg_alpha(), self.cfg_dt(), bcl, bcr),
            )
            .unwrap();
        assert_eq!(rc, 0, "iface-add re-init failed");
        for (i, v) in saved.iter().enumerate() {
            actor.set.call(&mut actor.store, (i as u32, *v)).unwrap();
        }
        let nd = &mut self.nodes[node];
        nd.actor = actor;
        nd.init_args = (n, x0, x1, bcl, bcr);
        for &(j, f) in &due {
            nd.ifaces.push((j, f));
        }
        eprintln!(
            "[sim] iface registered: node {} bc switched to interface {:?} from step {} (t={}ms)",
            node,
            due.iter().map(|&(j, _)| j).collect::<Vec<_>>(),
            step,
            self.now_us / 1000
        );
    }

    fn cfg_alpha(&self) -> f64 { self.cfg.alpha }
    fn cfg_dt(&self) -> f64 { self.cfg.dt }

    /// 界面IDから相手ノードを引く
    fn peer_of(&self, node: usize, iface: usize) -> usize {
        if iface == node { node + 1 } else { node - 1 }
    }

    fn send_iface(&mut self, node: usize, iface: usize, step: u64, is_resend: bool) {
        // 自分側の面から境界値を読む
        let my_face = if iface == node { FACE_RIGHT } else { FACE_LEFT };
        let (value, grad) = {
            let n = &mut self.nodes[node];
            let v = n.actor.bval.call(&mut n.actor.store, my_face).unwrap();
            let g = n.actor.bgrad.call(&mut n.actor.store, my_face).unwrap();
            (v, g)
        };
        let msg = Msg { iface, step, value, grad };
        self.nodes[node].sent.insert((iface, step), msg);
        self.nodes[node]
            .sent
            .retain(|&(_, s), _| s + 2 > step);
        self.transmit(node, msg, is_resend);
    }

    /// 損失・分断・遅延を適用して配送をスケジュールする
    fn transmit(&mut self, from: usize, msg: Msg, is_resend: bool) {
        let to = self.peer_of(from, msg.iface);
        let n = &mut self.nodes[from];
        n.stat_sent += 1;
        if is_resend {
            n.stat_resend += 1;
        }
        // 損失・分断・遅延の判定は dyne-core::net (乱数の消費順も従来と同一)
        let Some(t) = self.net.transit(from, to, self.now_us) else {
            self.nodes[from].stat_lost += 1;
            return;
        };
        self.schedule(t, Ev::Deliver { node: to, msg });
    }

    /// stepを開始: 全界面へ送信し、待ち状態に入る。
    /// F2ウォームスタート参加の最初のステップだけは送信を遅らせる: 受領した両隣の
    /// 値+勾配で初期場を構成してから (begin_compute 冒頭で) 送る。白紙のsin場の
    /// 境界値を隣に使わせないため (複合参照解とのビット一致の前提でもある)
    fn start_step(&mut self, node: usize, step: u64) {
        self.apply_iface_adds(node, step);
        if !self.nodes[node].warm_join_pending {
            let ifaces: Vec<usize> = self.nodes[node].ifaces.iter().map(|&(j, _)| j).collect();
            for j in &ifaces {
                self.send_iface(node, *j, step, false);
            }
        }
        let n = &mut self.nodes[node];
        n.cur_step = step;
        n.phase = Phase::Waiting { step };
        n.got = vec![false; n.ifaces.len()];
        n.recv_grad = vec![0.0; n.ifaces.len()];
        n.recv_val = vec![0.0; n.ifaces.len()];
        n.wait_since = self.now_us;
        n.last_resend_us = self.now_us;
        if n.ifaces.is_empty() {
            self.begin_compute(node);
            return;
        }
        // 取り置き分の即時消費
        self.try_pending(node);
        if !matches!(self.nodes[node].phase, Phase::Computing) {
            let t = self.now_us + self.cfg.timeout_us;
            self.schedule(t, Ev::Wake { node });
        }
    }

    fn try_pending(&mut self, node: usize) {
        let step = self.nodes[node].cur_step;
        let ifaces: Vec<(usize, usize, u32)> = self.nodes[node]
            .ifaces
            .iter()
            .enumerate()
            .map(|(k, &(j, f))| (k, j, f))
            .collect();
        for (k, j, f) in ifaces {
            if self.nodes[node].got[k] {
                continue;
            }
            if let Some(m) = self.nodes[node].pending.remove(&(j, step)) {
                let n = &mut self.nodes[node];
                n.actor
                    .set_neighbor
                    .call(&mut n.actor.store, (f, m.value, m.grad))
                    .unwrap();
                n.got[k] = true;
                n.recv_grad[k] = m.grad;
                n.recv_val[k] = m.value;
            }
        }
        if self.nodes[node].got.iter().all(|&g| g) {
            self.begin_compute(node);
        }
    }

    /// F2: 参加ノードの初期場を、受領済みの両隣の値+勾配から構成して書き込む。
    /// 隣の値・勾配はステップ S の交換メッセージそのもの (= 隣のステップ S−1 完了時の
    /// 境界)。複合参照解の編入時の初期場構成と厳密に同じ入力・同じ式になる。
    fn build_warm_field(&mut self, node: usize) {
        let (n, x0, x1, bcl, bcr) = self.nodes[node].init_args;
        let (s, cnt) = self.starts[node];
        let h = self.h_global;
        let h_node = (x1 - x0) / ((n - 1) as f64);
        let mut left = None;
        let mut right = None;
        for (k, &(_, f)) in self.nodes[node].ifaces.iter().enumerate() {
            let v = self.nodes[node].recv_val[k];
            let g = self.nodes[node].recv_grad[k];
            if f == FACE_LEFT {
                left = Some((((s - 1) as f64) * h, v, g));
            } else {
                right = Some((((s + cnt) as f64) * h, v, g));
            }
        }
        let mut vals = Vec::with_capacity(n as usize);
        for i in 0..n {
            let x = x0 + (i as f64) * h_node;
            vals.push(warm_field(left, right, x));
        }
        // 外縁 (グローバル境界) の Dirichlet 端は外縁設定値に落とす (initと同じ規則。
        // 既定 0.0 = 従来とビット同一)
        let ov = if let UBc::Dirichlet(v) = self.cfg.outer_bc { v } else { 0.0 };
        if bcl == BC_DIRICHLET {
            vals[0] = ov;
        }
        if bcr == BC_DIRICHLET {
            vals[(n - 1) as usize] = ov;
        }
        {
            let nd = &mut self.nodes[node];
            for (i, v) in vals.iter().enumerate() {
                nd.actor.set.call(&mut nd.actor.store, (i as u32, *v)).unwrap();
            }
        }
        eprintln!(
            "[sim] warm join: node {} の初期場を両隣の値+勾配から構成 (left={:?} right={:?})",
            node,
            left.map(|(_, v, g)| (v, g)),
            right.map(|(_, v, g)| (v, g)),
        );
    }

    fn begin_compute(&mut self, node: usize) {
        // F2ウォームスタート参加の最初のステップ: 全界面の受領が揃ったこの時点で
        // 初期場を構成し、遅らせていた自分の送信をここで行う
        if self.nodes[node].warm_join_pending {
            self.nodes[node].warm_join_pending = false;
            self.build_warm_field(node);
            let step = self.nodes[node].cur_step;
            let ifaces: Vec<usize> = self.nodes[node].ifaces.iter().map(|&(j, _)| j).collect();
            for j in ifaces {
                self.send_iface(node, j, step, false);
            }
        }
        // 界面残差 |grad_A - grad_B| を記録 (状態は step 実行前 = 交換値と同時点)。
        // 二重記録を避けるため、界面の左側 (自分の面が右) のノードだけが記録する。
        // これは M-S1c の自動トリガー T2 のセンサーの前倒し実装。
        let step = self.nodes[node].cur_step;
        let right_ifaces: Vec<(usize, usize, u32)> = self.nodes[node]
            .ifaces
            .iter()
            .enumerate()
            .filter(|(_, &(_, f))| f == FACE_RIGHT)
            .map(|(k, &(j, f))| (k, j, f))
            .collect();
        for (k, j, f) in right_ifaces {
            let g = {
                let n = &mut self.nodes[node];
                n.actor.bgrad.call(&mut n.actor.store, f).unwrap()
            };
            let resid = (g - self.nodes[node].recv_grad[k]).abs();
            self.residuals.entry(j).or_default().push((step, resid));
            self.check_trigger(j, step);
        }
        {
            let n = &mut self.nodes[node];
            n.wait_us += self.now_us - n.wait_since;
            n.actor.step.call(&mut n.actor.store, ()).unwrap();
            n.phase = Phase::Computing;
        }
        let t = self.now_us + self.cfg.step_us;
        self.schedule(t, Ev::StepDone { node });
    }

    fn on_deliver(&mut self, node: usize, msg: Msg) {
        // 自分にとってその界面がどの面か
        let Some(k) = self.nodes[node].ifaces.iter().position(|&(j, _)| j == msg.iface) else {
            // 未登録の界面宛 = 参加したばかりの隣接ノードからの先行送信。
            // 取り置いておけば、界面登録後の start_step → try_pending が消費する。
            let nd = &mut self.nodes[node];
            if (msg.iface == node || msg.iface + 1 == node) && msg.step >= nd.cur_step {
                nd.pending.insert((msg.iface, msg.step), msg);
            }
            return;
        };
        let my_face = self.nodes[node].ifaces[k].1;
        match self.nodes[node].phase {
            Phase::Waiting { step } if msg.step == step => {
                if !self.nodes[node].got[k] {
                    let n = &mut self.nodes[node];
                    n.actor
                        .set_neighbor
                        .call(&mut n.actor.store, (my_face, msg.value, msg.grad))
                        .unwrap();
                    n.got[k] = true;
                    n.recv_grad[k] = msg.grad;
                    n.recv_val[k] = msg.value;
                    if n.got.iter().all(|&g| g) {
                        self.begin_compute(node);
                    }
                }
            }
            _ if msg.step > self.nodes[node].cur_step
                || (msg.step == self.nodes[node].cur_step
                    && matches!(self.nodes[node].phase, Phase::Computing)) =>
            {
                // 将来分(自分がまだそのステップに達していない): 取り置き
                self.nodes[node].pending.insert((msg.iface, msg.step), msg);
            }
            _ => {
                // 過去分: 相手はまだその値を受け取れていない。控えを再送(replay)
                if let Some(m) = self.nodes[node].sent.get(&(msg.iface, msg.step)).copied() {
                    self.nodes[node].stat_replay += 1;
                    self.transmit(node, m, false);
                }
            }
        }
    }

    fn on_wake(&mut self, node: usize) {
        if let Phase::Waiting { step } = self.nodes[node].phase {
            // 再送バックオフ: 最小間隔を空ける(M1知見4)
            if self.now_us - self.nodes[node].last_resend_us >= self.cfg.resend_min_us {
                let missing: Vec<usize> = self.nodes[node]
                    .ifaces
                    .iter()
                    .enumerate()
                    .filter(|(k, _)| !self.nodes[node].got[*k])
                    .map(|(_, &(j, _))| j)
                    .collect();
                for j in missing {
                    if let Some(m) = self.nodes[node].sent.get(&(j, step)).copied() {
                        self.transmit(node, m, true);
                    }
                }
                self.nodes[node].last_resend_us = self.now_us;
            }
            let t = self.now_us + self.cfg.timeout_us;
            self.schedule(t, Ev::Wake { node });
        }
    }

    /// 発注8: 観測点ナッジング。ステップ完了直後、観測周期ごとに、担当区間の
    /// 中央格子点の計算値を合成観測 y = 基準解 + σ·N(0,1) へ引き寄せる。
    /// 式そのものはアクター内 (dyne_observe) にあり、戻り値のイノベーション
    /// d = y − u (適用前) を記録する (閾値判定はしない。計測のみ)。
    fn apply_observation(&mut self, node: usize) {
        let step = self.nodes[node].cur_step;
        let (s, cnt) = self.starts[node];
        let li = cnt / 2; // 各ノード1観測点 = 担当区間の中央格子点 (実装側の設計)
        let gi = s + li;
        let truth = self.obs_truth[step as usize][gi];
        let y = if self.cfg.obs_sigma > 0.0 {
            // Box-Muller (libm = 決定論)。消費順はイベント処理順で固定
            let u1 = self.obs_rng.uniform().max(1e-300);
            let u2 = self.obs_rng.uniform();
            let n01 = libm::sqrt(-2.0 * libm::log(u1))
                * libm::cos(2.0 * std::f64::consts::PI * u2);
            truth + self.cfg.obs_sigma * n01
        } else {
            truth
        };
        // 発注11: 保留ノード (抜き取り検証) は γ=0 で適用 = 同化に使わず
        // イノベーションのみ記録 (実機の holdout と同一意味論)
        let gamma = if self.cfg.holdout == Some(node) { 0.0 } else { self.cfg.nudge_gamma };
        let nd = &mut self.nodes[node];
        let f = nd.actor.observe.expect("このアクターは dyne_observe を持たない");
        let d = f.call(&mut nd.actor.store, (li as u32, y, gamma)).unwrap();
        self.innovation_log.push((node, step, gi, y, d));
    }

    fn on_step_done(&mut self, node: usize) {
        if self.cfg.field_csv.is_some() && self.nodes[node].cur_step % self.cfg.field_every == 0 {
            let nd = &mut self.nodes[node];
            let n = nd.init_args.0;
            let xs: Vec<f64> = (0..n).map(|i| nd.actor.x.call(&mut nd.actor.store, i).unwrap()).collect();
            let us: Vec<f64> = (0..n).map(|i| nd.actor.get.call(&mut nd.actor.store, i).unwrap()).collect();
            self.field_log.push((node, nd.cur_step, xs, us));
        }
        if self.cfg.error_csv.is_some() {
            let nd = &mut self.nodes[node];
            let n = nd.init_args.0;
            let u: Vec<f64> = (0..n)
                .map(|i| nd.actor.get.call(&mut nd.actor.store, i).unwrap())
                .collect();
            self.snap_log.push((node, nd.cur_step, u));
        }
        // 発注8: 観測周期ごとのナッジング (計算完了直後 = 次ステップの交換前。
        // 実機ファームの適用点と同一)
        if (self.cfg.nudge_gamma > 0.0 || self.cfg.truth_grad.is_some())
            && (self.nodes[node].cur_step + 1) % self.cfg.obs_period == 0
        {
            self.apply_observation(node);
        }
        let next = self.nodes[node].cur_step + 1;
        if next >= self.cfg.steps {
            self.nodes[node].phase = Phase::Done;
            // ドレインは不要: プロセス内に居続けるので過去分replayには常時応答できる
            return;
        }
        self.start_step(node, next);
    }

    fn run(&mut self) {
        for node in 0..self.nodes.len() {
            if self.nodes[node].present {
                self.start_step(node, 0);
            }
        }
        // 参加時刻に確実にイベント処理が走るよう、ADD対象へWakeを予約しておく
        // (全ノードが早期にDoneになってもヒープが空にならないための保険)
        let adds: Vec<(usize, u64)> = self.cfg.adds.clone();
        for (node, t) in adds {
            self.schedule(t, Ev::Wake { node });
        }
        while let Some((t, ev)) = self.queue.pop() {
            self.now_us = t;
            self.apply_handovers();
            self.apply_adds();
            self.apply_resizes();
            // 一時停止中のノード宛イベントは停止解除時刻へ繰り延べ
            let target = match &ev {
                Ev::Deliver { node, .. } | Ev::Wake { node } | Ev::StepDone { node } => *node,
            };
            if let Some(resume) = self.in_pause(target, t) {
                self.schedule(resume, ev);
                continue;
            }
            match ev {
                Ev::Deliver { node, msg } => self.on_deliver(node, msg),
                Ev::Wake { node } => self.on_wake(node),
                Ev::StepDone { node } => self.on_step_done(node),
            }
            if self
                .nodes
                .iter()
                .all(|n| n.phase == Phase::Done || (n.permanent_absent && !n.present))
            {
                break;
            }
        }
    }
}

// ---------- 分割・検証 ----------

fn main() {
    let cfg = parse_args();
    let wasm = std::fs::read(&cfg.wasm).expect("read wasm");
    let engine = wasmi::Engine::default();
    let module = wasmi::Module::new(&engine, &wasm[..]).expect("compile wasm");

    // 格子の分割(重複なし・M0方式の一般化)
    let n_pts = cfg.points;
    let n_nodes = cfg.nodes;
    assert!(n_pts >= n_nodes * 3, "各ノードに最低3点必要 (points >= nodes*3)");
    // 発注11: h = L/(points-1)。既定 L=1.0 は従来の 1.0/(points-1) とビット同一
    let h = cfg.length / ((n_pts - 1) as f64);
    // 陽解法の安定条件 (M0ホストと同じ検証)。r ≤ 1/2 は**差分法**の条件であり、
    // 学習型アクターの安定領域は別物 (理論ノートN7) なので、学習型では警告に
    // 落として掃引を許す (発注5、2026-09-20。判定は後段の Linf/非有限検出)
    let r = cfg.alpha * cfg.dt / (h * h);
    let base = n_pts / n_nodes;
    let extra = n_pts % n_nodes;
    let mut starts = Vec::new();
    let mut idx = 0usize;
    for i in 0..n_nodes {
        let cnt = base + if i < extra { 1 } else { 0 };
        starts.push((idx, cnt));
        idx += cnt;
    }

    // RAM上限チェック(実機校正: 線形メモリ+ランタイム概算がノード上限を超えないか)
    let probe = Actor::load_with(&engine, &module, &cfg.pinn_cfg);
    if r > 0.5 {
        if probe.is_learning() {
            eprintln!(
                "[sim] r = alpha*dt/h^2 = {r:.3} > 0.5: 差分法の安定条件外だが学習型のため続行 \
                 (N7掃引。判定は Linf と非有限検出で行う)"
            );
        } else {
            panic!("stability violated: alpha*dt/h^2 = {r:.3} > 0.5 (dtを下げるかpointsを減らすこと)");
        }
    }
    let linear_bytes = probe.memory_pages * 64 * 1024;
    let runtime_overhead = 10 * 1024; // M1実測: 使用129KBのうち線形メモリ128KB → 管理構造は約1KB+余裕
    let need = linear_bytes + runtime_overhead;
    if need > cfg.ram_cap_bytes {
        eprintln!(
            "[sim] REJECT: ノードあたり必要RAM {}KB > 上限 {}KB (実機に載らない構成)",
            need / 1024,
            cfg.ram_cap_bytes / 1024
        );
        std::process::exit(2);
    }
    drop(probe);

    // F1: 未観測領域境界の設定 (透過境界なら指数和係数をここで一度だけ構成する。
    // パラメータ・総和順は固定 = 決定論。libm による計算で OS 間もビット一致)
    let t_coef: Option<TransparentCoef> = if cfg.unobserved_bc == UBc::Transparent {
        let c = dyne_core::transparent::coefficients_with(cfg.alpha, cfg.dt, cfg.soe_h);
        eprintln!(
            "[sim] transparent境界: 指数和 M={} (h={}, c1={:.0e}, c2={:.0e}, T_ref={})",
            c.decay.len(),
            cfg.soe_h,
            dyne_core::transparent::SOE_C1,
            dyne_core::transparent::SOE_C2,
            dyne_core::transparent::SOE_T_REF,
        );
        Some(c)
    } else {
        None
    };
    if cfg.unobserved_bc == UBc::LinearExtrap {
        eprintln!(
            "[sim] 警告: linear_extrap は拡散系では未観測領域境界の端点が時間凍結する \
             (理論ノートT8)。拡散タスクでは zero_grad か transparent を推奨"
        );
    }

    // 発注11: 全域外縁の設定情報化 (既定 dirichlet:0 = 従来とビット同一)
    let outer_code = cfg.outer_bc.code();
    let outer_val = if let UBc::Dirichlet(v) = cfg.outer_bc { v } else { 0.0 };
    let _ = outer_val; // 値は apply_face_config が cfg から読む (将来の直接利用に備え保持)

    // 発注8: 合成観測の真値。既定 = 単一ノード基準解 (同一wasm・全域) の全履歴。
    // 発注11: --truth-grad 指定時は式で生成する (一端が暖かい定常勾配+緩い時間変動。
    // モデルの外にある真値なので、モデル誤差込みの検証になる = 実験1のリハーサル)
    let obs_truth: Vec<Vec<f64>> = if let Some((tc, tw, period, amp)) = cfg.truth_grad {
        let mut hist = Vec::with_capacity(cfg.steps as usize);
        for step in 0..cfg.steps {
            let t = cfg.dt * ((step + 1) as f64);
            let osc = amp * libm::sin(2.0 * std::f64::consts::PI * t / period);
            hist.push(
                (0..n_pts)
                    .map(|gi| {
                        let xf = (gi as f64) * h / cfg.length; // x/L ∈ [0,1]
                        tc + (tw - tc) * xf + osc * xf
                    })
                    .collect(),
            );
        }
        hist
    } else if cfg.nudge_gamma > 0.0 {
        let mut single = Actor::load_with(&engine, &module, &cfg.pinn_cfg);
        apply_face_config(&mut single, 0, 1, outer_code, outer_code, &cfg.unobserved_bc, &cfg.outer_bc, &t_coef);
        let rc = single
            .init
            .call(
                &mut single.store,
                (n_pts as u32, 0.0, cfg.length, cfg.alpha, cfg.dt, outer_code, outer_code),
            )
            .unwrap();
        assert_eq!(rc, 0);
        if let Some(t0) = cfg.init_uniform {
            for i in 0..n_pts {
                single.set.call(&mut single.store, (i as u32, t0)).unwrap();
            }
        }
        let mut hist = Vec::with_capacity(cfg.steps as usize);
        for _ in 0..cfg.steps {
            single.step.call(&mut single.store, ()).unwrap();
            hist.push(
                (0..n_pts)
                    .map(|i| single.get.call(&mut single.store, i as u32).unwrap())
                    .collect(),
            );
        }
        hist
    } else {
        Vec::new()
    };

    // ノード生成。--add 指定ノードは開始時不在、--absent 指定ノードは恒久不在 (β意味論):
    // 隣接ノードは当該面を「未観測領域境界」(--unobserved-bc の設定キー、既定 zero_grad)・
    // 界面登録なしで初期化する。グローバル外縁 (x=0, x=1) は常に Dirichlet u=0。
    let ub_code = cfg.unobserved_bc.code();
    let absent_at_start: Vec<bool> = (0..n_nodes)
        .map(|i| cfg.adds.iter().any(|&(a, _)| a == i) || cfg.absent.contains(&i))
        .collect();
    let mut nodes = Vec::new();
    for i in 0..n_nodes {
        let (s, cnt) = starts[i];
        let x0 = (s as f64) * h;
        let x1 = ((s + cnt - 1) as f64) * h;
        // ローカル格子幅がグローバルとビット一致しない分割では、参照解との
        // ビット一致は保証されない (既定の points=65 等の2べき系格子では一致する)
        let h_local = (x1 - x0) / ((cnt - 1) as f64);
        if h_local.to_bits() != h.to_bits() {
            eprintln!("[sim] WARN: node {i} local h != global h (ビット一致は保証されない)");
        }
        let present = !absent_at_start[i];
        let bc_l = if i == 0 {
            outer_code
        } else if absent_at_start[i - 1] {
            ub_code
        } else {
            BC_INTERFACE
        };
        let bc_r = if i == n_nodes - 1 {
            outer_code
        } else if absent_at_start[i + 1] {
            ub_code
        } else {
            BC_INTERFACE
        };
        let mut actor = Actor::load_with(&engine, &module, &cfg.pinn_cfg);
        if present {
            apply_face_config(&mut actor, i, n_nodes, bc_l, bc_r, &cfg.unobserved_bc, &cfg.outer_bc, &t_coef);
            let rc = actor
                .init
                .call(
                    &mut actor.store,
                    (cnt as u32, x0, x1, cfg.alpha, cfg.dt, bc_l, bc_r),
                )
                .unwrap();
            assert_eq!(rc, 0, "dyne_init failed on node {i}");
            // 発注11: 初期場の一様上書き (実験1: 実測平均で初期化。基準解側も同様)
            if let Some(t0) = cfg.init_uniform {
                for k in 0..cnt {
                    actor.set.call(&mut actor.store, (k as u32, t0)).unwrap();
                }
            }
        }
        let mut ifaces = Vec::new();
        if present {
            if i > 0 && !absent_at_start[i - 1] {
                ifaces.push((i - 1, FACE_LEFT)); // 界面 i-1 = 左隣との境界
            }
            if i < n_nodes - 1 && !absent_at_start[i + 1] {
                ifaces.push((i, FACE_RIGHT)); // 界面 i = 右隣との境界
            }
        }
        nodes.push(Node {
            actor,
            init_args: (cnt as u32, x0, x1, bc_l, bc_r),
            ifaces,
            present,
            iface_adds: Vec::new(),
            phase: Phase::Waiting { step: 0 },
            cur_step: 0,
            got: Vec::new(),
            recv_grad: Vec::new(),
            recv_val: Vec::new(),
            permanent_absent: cfg.absent.contains(&i),
            warm_join_pending: false,
            pending: HashMap::new(),
            sent: HashMap::new(),
            last_resend_us: 0,
            stat_sent: 0,
            stat_lost: 0,
            stat_resend: 0,
            stat_replay: 0,
            wait_us: 0,
            wait_since: 0,
        });
    }

    // 学習型アクターの初期幅 (--pinn width=… 指定があればそれ、無ければアクター既定16)
    let init_width: u32 = cfg
        .pinn_cfg
        .iter()
        .rev()
        .find(|&&(k, _)| k == 3)
        .map(|&(_, v)| v as u32)
        .unwrap_or(16);
    let mut sim = Sim {
        cfg: cfg.clone(),
        nodes,
        now_us: 0,
        queue: EventQueue::new(),
        net: VirtNet::new(cfg.loss, cfg.lat_us, cfg.lat_jitter_us, cfg.partitions.clone(), cfg.seed),
        engine,
        module,
        join_log: Vec::new(),
        residuals: HashMap::new(),
        resize_log: Vec::new(),
        snap_log: Vec::new(),
        width: vec![init_width; n_nodes],
        trig: HashMap::new(),
        auto_log: Vec::new(),
        event_log: Vec::new(),
        field_log: Vec::new(),
        starts: starts.clone(),
        h_global: h,
        t_coef,
        reset_log: Vec::new(),
        obs_truth,
        // 観測ノイズの系列は通信 (VirtNet) の乱数と独立。タグ 0xD1E で導出
        obs_rng: dyne_core::rng::Rng::new(dyne_core::rng::derive_seed(cfg.seed, 0xD1E)),
        innovation_log: Vec::new(),
    };
    sim.run();
    assert_eq!(
        sim.join_log.len(),
        cfg.adds.len(),
        "一部のADDが適用されなかった (参加時刻が実行時間より後の可能性)"
    );

    // 参照解:
    //   ADDなし → 単一ノード参照解 (同一wasm・全域) — 従来どおり
    //   ADDあり → 複合参照解 (同じイベント時系列を単一配列で再現、ホスト側純Rust)
    // 発注11: 解析解 (sin減衰) 比較が意味を持つのは従来の無次元・外縁0固定・
    // sin初期値の構成のみ
    let analytic_ok = cfg.adds.is_empty()
        && cfg.absent.is_empty()
        && cfg.nudge_gamma == 0.0
        && cfg.truth_grad.is_none()
        && cfg.length == 1.0
        && cfg.init_uniform.is_none()
        && matches!(cfg.outer_bc, UBc::Dirichlet(v) if v == 0.0);
    let mut ref_history: Vec<Vec<f64>> = Vec::new();
    let reference: Vec<f64> = if cfg.nudge_gamma > 0.0 {
        // ナッジング有効時: 基準解 = 観測の真値 (単一ノード解、同化なし) を再利用
        let r = sim.obs_truth.last().unwrap().clone();
        if cfg.error_csv.is_some() {
            ref_history = std::mem::take(&mut sim.obs_truth);
        }
        r
    } else if cfg.adds.is_empty() {
        // 部分観測 (--absent) でも基準解は**全域**の単一ノード解 (N10 の実験設計:
        // 観測領域の誤差を全域基準解に対して測る)
        let mut single = Actor::load_with(&sim.engine, &sim.module, &cfg.pinn_cfg);
        apply_face_config(&mut single, 0, 1, outer_code, outer_code, &cfg.unobserved_bc, &cfg.outer_bc, &sim.t_coef);
        let rc = single
            .init
            .call(
                &mut single.store,
                (n_pts as u32, 0.0, cfg.length, cfg.alpha, cfg.dt, outer_code, outer_code),
            )
            .unwrap();
        assert_eq!(rc, 0);
        if let Some(t0) = cfg.init_uniform {
            for i in 0..n_pts {
                single.set.call(&mut single.store, (i as u32, t0)).unwrap();
            }
        }
        let want_hist = cfg.error_csv.is_some() && (!cfg.absent.is_empty() || !analytic_ok);
        for _ in 0..cfg.steps {
            single.step.call(&mut single.store, ()).unwrap();
            if want_hist {
                ref_history.push(
                    (0..n_pts)
                        .map(|i| single.get.call(&mut single.store, i as u32).unwrap())
                        .collect(),
                );
            }
        }
        (0..n_pts)
            .map(|i| single.get.call(&mut single.store, i as u32).unwrap())
            .collect()
    } else {
        let (r, hist) = composite_reference(
            &cfg,
            &starts,
            h,
            &absent_at_start,
            &sim.join_log,
            &sim.reset_log,
            &sim.t_coef,
            cfg.error_csv.is_some(),
        );
        ref_history = hist;
        r
    };
    let learning = sim.nodes[0].actor.is_learning();

    let mut exact = 0usize;
    let mut obs_pts = 0usize;
    let mut max_diff = 0.0f64;
    let mut l2r = 0.0f64;
    let mut l2 = 0.0f64;
    let mut linf = 0.0f64;
    // NaN は f64::max に無視されるため明示的に検出する (発注8で顕在化したバグの修正:
    // 発散した場が「Linf=0」と報告され安定と誤判定されていた)
    let mut nonfinite = false;
    let t_final = cfg.dt * (cfg.steps as f64);
    let decay = (-cfg.alpha * std::f64::consts::PI.powi(2) * t_final).exp();
    let mut node_linf_ref = vec![0.0f64; n_nodes];
    for i in 0..n_nodes {
        if !sim.nodes[i].present {
            continue; // 恒久不在ノード (部分観測): 計測対象外
        }
        let (s, cnt) = starts[i];
        for k in 0..cnt {
            let gi = s + k;
            let n = &mut sim.nodes[i];
            let u = n.actor.get.call(&mut n.actor.store, k as u32).unwrap();
            let x = n.actor.x.call(&mut n.actor.store, k as u32).unwrap();
            let us = reference[gi];
            let d = (u - us).abs();
            if !u.is_finite() {
                nonfinite = true;
            }
            if u == us {
                exact += 1;
            }
            obs_pts += 1;
            max_diff = max_diff.max(d);
            l2r += d * d;
            node_linf_ref[i] = node_linf_ref[i].max(d);
            let e = (u - (std::f64::consts::PI * x).sin() * decay).abs();
            l2 += e * e;
            linf = linf.max(e);
        }
    }
    l2 = (l2 / (obs_pts as f64)).sqrt();
    l2r = (l2r / (obs_pts as f64)).sqrt();

    // 集計
    let makespan_ms = sim.now_us as f64 / 1000.0;
    let ideal_ms = (cfg.steps as f64) * (cfg.step_us as f64) / 1000.0;
    let sent: u64 = sim.nodes.iter().map(|n| n.stat_sent).sum();
    let lost: u64 = sim.nodes.iter().map(|n| n.stat_lost).sum();
    let resend: u64 = sim.nodes.iter().map(|n| n.stat_resend).sum();
    let replay: u64 = sim.nodes.iter().map(|n| n.stat_replay).sum();
    let max_wait_ms = sim
        .nodes
        .iter()
        .map(|n| n.wait_us as f64 / 1000.0)
        .fold(0.0f64, f64::max);

    println!("==== M-S0 REPORT ====");
    println!(
        "nodes={} points={} steps={} seed={} | loss={:.0}% lat={}±{}ms step={}ms",
        n_nodes,
        n_pts,
        cfg.steps,
        cfg.seed,
        cfg.loss * 100.0,
        cfg.lat_us / 1000,
        cfg.lat_jitter_us / 1000,
        cfg.step_us as f64 / 1000.0
    );
    if !cfg.pauses.is_empty() {
        println!("pauses={:?} (node@us)", cfg.pauses);
    }
    if !cfg.partitions.is_empty() {
        println!("partitions={:?} (k@us)", cfg.partitions);
    }
    if !cfg.adds.is_empty() {
        println!("adds={:?} (node@us, β=領域拡張)", cfg.adds);
    }
    if !cfg.absent.is_empty() {
        println!("absent={:?} (恒久不在=未観測領域、部分観測シナリオ)", cfg.absent);
    }
    // F1/F2 の設定表示。未観測領域境界が存在しない構成 (adds も absent もなし) では
    // 設定は挙動に影響しないため表示しない (切り出し前基準出力との行単位比較を保つ)
    if !cfg.adds.is_empty() || !cfg.absent.is_empty() {
        println!(
            "unobserved-bc = {} | join-init = {}",
            cfg.unobserved_bc.label(),
            if cfg.join_warm { "warm" } else { "blank" }
        );
    }
    // 発注8: ナッジング設定の表示 (無効時は非表示 = 既存基準出力との行単位一致を保つ)
    if cfg.nudge_gamma > 0.0 {
        let truth_name = if let Some((tc, tw, pd, amp)) = cfg.truth_grad {
            format!("式生成 grad({tc}→{tw}℃, 周期{pd}s, 振幅{amp})")
        } else {
            "単一基準解".to_string()
        };
        println!(
            "nudging: gamma={} period={} sigma={} obs={}本 holdout={:?} (各現存ノードの中央格子点、y={}+σN(0,1))",
            cfg.nudge_gamma,
            cfg.obs_period,
            cfg.obs_sigma,
            sim.innovation_log.len(),
            cfg.holdout,
            truth_name
        );
    }
    let ref_name = if cfg.adds.is_empty() { "single" } else { "composite" };
    if learning {
        // 学習型アクター: モデルが違うのでビット一致は定義されない。誤差で報告する。
        // 参照解は single = 全域1ノードの同一アクター (学習型同士)、composite = 差分法の逐次計算。
        println!(
            "actor = learning (heat1d-pinn-actor), pinn_cfg={:?} add_warm={:?}",
            cfg.pinn_cfg, cfg.add_warm
        );
        println!(
            "bit-exact vs {}: n/a (学習型) | Linf vs {} = {:.3e}, per-node Linf = [{}]",
            ref_name,
            ref_name,
            max_diff,
            node_linf_ref
                .iter()
                .map(|v| format!("{:.1e}", v))
                .collect::<Vec<_>>()
                .join(", ")
        );
    } else if !cfg.absent.is_empty() {
        // 部分観測 (N10): 全域基準解に対する観測領域の誤差で報告 (ビット一致は
        // 期待しない — 未観測領域の代理境界そのもののモデル化誤差を測っている)
        println!(
            "partial observation: observed {}/{} pts | Linf vs full single = {:.3e}, L2 = {:.3e}",
            obs_pts, n_pts, max_diff, l2r
        );
        println!(
            "per-node Linf vs single = [{}] (absent は -)",
            (0..n_nodes)
                .map(|i| if sim.nodes[i].present {
                    format!("{:.1e}", node_linf_ref[i])
                } else {
                    "-".to_string()
                })
                .collect::<Vec<_>>()
                .join(", ")
        );
    } else if cfg.nudge_gamma > 0.0 {
        // ナッジング有効: 基準解 = 同化なしの真値なのでビット一致は主張しない。
        // 誤差 (同化の効果) で報告する。truth-grad 時は式生成の真値が基準
        let refname = if cfg.truth_grad.is_some() { "truth" } else { "single(no assim)" };
        println!("nudging: Linf vs {} = {:.3e}, L2 = {:.3e}", refname, max_diff, l2r);
        println!(
            "per-node Linf vs {} = [{}]",
            refname,
            (0..n_nodes)
                .map(|i| if sim.nodes[i].present {
                    format!("{:.2e}", node_linf_ref[i])
                } else {
                    "-".to_string()
                })
                .collect::<Vec<_>>()
                .join(", ")
        );
    } else {
        println!(
            "bit-exact vs {}: {}/{} points, max_diff={:.3e}",
            ref_name, exact, n_pts, max_diff
        );
    }
    if !cfg.absent.is_empty() {
        // 観測領域は端が解析解の境界と一致しないため解析解比較は意味を持たない
        println!("L2 vs analytic: n/a (部分観測シナリオ)");
    } else if !cfg.adds.is_empty() {
        // 領域が途中から拡張されるため sin減衰の解析解は適用外
        println!("L2 vs analytic: n/a (領域拡張シナリオ)");
    } else if analytic_ok {
        println!("L2 vs analytic = {:.3e}, Linf = {:.3e}", l2, linf);
    } else if cfg.nudge_gamma == 0.0 {
        // 物理単位運転・外縁変更等: sin減衰の解析解は適用外
        println!("L2 vs analytic: n/a (既定外の構成)");
    }

    // ---- 実機照合用: ノード別の温度配列CRC32 (--crc。2026-09-18追加) ----
    // 定義は実機ファーム (status: done の crc32) と同一: 各ノードの u[] を
    // f64 リトルエンディアンで連結したバイト列の IEEE CRC32。
    if cfg.crc {
        let mut parts = Vec::new();
        for i in 0..n_nodes {
            if !sim.nodes[i].present {
                parts.push(format!("node {i}: -"));
                continue;
            }
            let (_, cnt) = starts[i];
            let mut bytes = Vec::with_capacity(cnt * 8);
            for k in 0..cnt {
                let n = &mut sim.nodes[i];
                let u = n.actor.get.call(&mut n.actor.store, k as u32).unwrap();
                bytes.extend_from_slice(&u.to_le_bytes());
            }
            parts.push(format!("node {i}: 0x{:08X}", crc32_ieee(&bytes)));
        }
        println!("CRC32(u bytes) = [{}]", parts.join(", "));
    }

    // ---- シナリオC計測: 界面残差の乱れ (ピーク) と回復時間 ----
    if !sim.join_log.is_empty() {
        println!("==== ADD REPORT (scenario C / 解釈β) ====");
        let s_min = sim.join_log.iter().map(|j| j.step).min().unwrap();
        let new_ifaces: Vec<usize> = sim
            .join_log
            .iter()
            .flat_map(|j| {
                let mut v = Vec::new();
                if j.bc.0 == BC_INTERFACE {
                    v.push(j.node - 1);
                }
                if j.bc.1 == BC_INTERFACE {
                    v.push(j.node);
                }
                v
            })
            .collect();
        // ベースライン: 参加直前100ステップの既存界面の残差の中央値
        // (全期間だと序盤の過渡で膨らむため、参加直前の水準を取る)。
        // 既存界面がない構成 (2ノード等) では、参加界面の末尾25%区間の中央値で代用。
        let w_lo = s_min.saturating_sub(100);
        let mut pre: Vec<f64> = sim
            .residuals
            .iter()
            .filter(|(j, _)| !new_ifaces.contains(j))
            .flat_map(|(_, v)| {
                v.iter()
                    .filter(|&&(s, _)| s >= w_lo && s < s_min)
                    .map(|&(_, r)| r)
            })
            .collect();
        let (baseline, base_src) = if !pre.is_empty() {
            (median(&mut pre), "参加直前100ステップの既存界面の残差の中央値")
        } else {
            let mut tail: Vec<f64> = new_ifaces
                .iter()
                .flat_map(|j| {
                    let v = &sim.residuals[j];
                    v[v.len() - v.len() / 4..].iter().map(|&(_, r)| r)
                })
                .collect();
            (
                median(&mut tail),
                "参加界面の末尾25%の中央値 (既存界面なしのため代用)",
            )
        };
        println!("residual baseline = {:.3e} ({})", baseline, base_src);
        for jr in &sim.join_log {
            let mut own = Vec::new();
            if jr.bc.0 == BC_INTERFACE {
                own.push(jr.node - 1);
            }
            if jr.bc.1 == BC_INTERFACE {
                own.push(jr.node);
            }
            println!(
                "join: node {} at t={}ms -> 参加ステップ {}",
                jr.node,
                jr.t_us / 1000,
                jr.step
            );
            for ifc in own {
                let series = &sim.residuals[&ifc];
                let (pk_step, pk) = series
                    .iter()
                    .fold((0u64, 0.0f64), |acc, &(s, r)| if r > acc.1 { (s, r) } else { acc });
                // 回復時間 = 参加からのステップ数で、乱れピーク後に残差が最初に
                // 閾値を下回るまで (乱れは参加の数十ステップ後にピークを迎えることが
                // あるため、探索はピーク以降から)。ピークが閾値未満なら乱れ自体が
                // 平常変動の範囲内なので「対象外」とする。
                let recovery = |threshold: f64| -> String {
                    if pk < threshold {
                        return "対象外 (ピークが閾値未満)".to_string();
                    }
                    series
                        .iter()
                        .filter(|&&(s, _)| s >= pk_step)
                        .find(|&&(_, r)| r < threshold)
                        .map(|&(s, _)| format!("{} steps", s - jr.step))
                        .unwrap_or_else(|| "未回復".to_string())
                };
                println!(
                    "  iface {}: 乱れピーク={:.3e} (step {}), 回復時間(<2x baseline)={}",
                    ifc,
                    pk,
                    pk_step,
                    recovery(2.0 * baseline)
                );
                // 補助指標: 残差は局所曲率に依存し界面ごとに水準が違うため、
                // 参加界面自身の定常値 (末尾25%中央値) に対する回復も併記する
                let mut tail: Vec<f64> = series[series.len() - series.len() / 4..]
                    .iter()
                    .map(|&(_, r)| r)
                    .collect();
                let own_steady = median(&mut tail);
                println!(
                    "           自定常値={:.3e} (末尾25%中央値), 回復時間(<2x 自定常)={}",
                    own_steady,
                    recovery(2.0 * own_steady)
                );
            }
        }
    }
    if !sim.resize_log.is_empty() {
        println!("==== RESIZE REPORT (GROW/SHRINK) ====");
        for r in &sim.resize_log {
            println!(
                "node {} width -> {} at t={}ms (step {}): model output preserved {}/{} pts bit-exact ({})",
                r.node,
                r.width,
                r.t_us / 1000,
                r.step,
                r.preserved,
                r.n,
                if r.preserved == r.n { "機能保存" } else { "関数が変化 (SHRINK は温めで回復)" }
            );
        }
    }
    if cfg.auto_grow.is_some() {
        println!("==== AUTO-GROW REPORT (T2) ====");
        println!("params = {:?}", cfg.auto_grow.unwrap());
        println!("fires = {}", sim.auto_log.len());
        for r in &sim.auto_log {
            println!(
                "step {} (t={}ms): iface {} current={:.3e} baseline={:.3e} (x{:.1}) -> node {} width {} -> {}",
                r.step, r.t_us / 1000, r.iface, r.current, r.baseline, r.current / r.baseline, r.node, r.from, r.to
            );
        }
        let widths: Vec<String> = sim.width.iter().map(|w| w.to_string()).collect();
        println!("final widths = [{}]", widths.join(","));
    }
    if let Some(path) = &cfg.event_csv {
        let mut out = String::from("kind,step,t_ms,node,iface,width,current,baseline\n");
        let mut ev = sim.event_log.clone();
        ev.sort_by_key(|e| (e.t_us, e.step));
        for e in &ev {
            out.push_str(&format!(
                "{},{},{},{},{},{},{:e},{:e}\n",
                e.kind,
                e.step,
                e.t_us / 1000,
                e.node,
                e.iface.map(|j| j.to_string()).unwrap_or_default(),
                e.width.map(|w| w.to_string()).unwrap_or_default(),
                e.current,
                e.baseline
            ));
        }
        std::fs::write(path, out).expect("write event csv");
        eprintln!("[sim] event csv written: {path}");
    }
    if let Some(path) = &cfg.field_csv {
        let mut out = String::from("node,step,i,x,u\n");
        sim.field_log.sort_by_key(|(n, s, _, _)| (*s, *n));
        for (node, step, xs, us) in &sim.field_log {
            for (i, (x, u)) in xs.iter().zip(us).enumerate() {
                out.push_str(&format!("{},{},{},{:e},{:e}\n", node, step, i, x, u));
            }
        }
        std::fs::write(path, out).expect("write field csv");
        eprintln!("[sim] field csv written: {path}");
    }
    if let Some(path) = &cfg.error_csv {
        // ノードごと・ステップごとの参照解との誤差。参照解は ADD なし = 解析解、
        // ADD あり = 複合参照解の履歴 (領域拡張シナリオでは解析解が定義できないため)
        let mut out = String::from("node,step,t,linf,l2\n");
        sim.snap_log.sort_by_key(|(n, s, _)| (*s, *n));
        for (node, step, u) in &sim.snap_log {
            let (s, cnt) = starts[*node];
            let t = cfg.dt * ((*step + 1) as f64);
            let dec = (-cfg.alpha * std::f64::consts::PI.powi(2) * t).exp();
            let mut linf = 0.0f64;
            let mut l2 = 0.0f64;
            for k in 0..cnt {
                let gi = s + k;
                let r = if analytic_ok {
                    (std::f64::consts::PI * (gi as f64) * h).sin() * dec
                } else {
                    // ADDあり = 複合参照解、部分観測/ナッジング = 全域単一ノード解の履歴
                    ref_history[*step as usize][gi]
                };
                let e = (u[k] - r).abs();
                linf = linf.max(e);
                l2 += e * e;
            }
            l2 = (l2 / (cnt as f64)).sqrt();
            out.push_str(&format!("{},{},{:e},{:e},{:e}\n", node, step, t, linf, l2));
        }
        std::fs::write(path, out).expect("write error csv");
        eprintln!("[sim] error csv written: {path}");
    }
    if let Some(path) = &cfg.innovation_csv {
        // 発注8: イノベーション d = y − u (ナッジング適用前) の時系列
        let mut out = String::from("node,step,gi,y,innovation\n");
        for &(node, step, gi, y, d) in &sim.innovation_log {
            out.push_str(&format!("{},{},{},{:e},{:e}\n", node, step, gi, y, d));
        }
        std::fs::write(path, out).expect("write innovation csv");
        eprintln!("[sim] innovation csv written: {path}");
    }
    if let Some(path) = &cfg.residual_csv {
        let mut out = String::from("iface,step,residual\n");
        let mut keys: Vec<&usize> = sim.residuals.keys().collect();
        keys.sort();
        for k in keys {
            for &(s, r) in &sim.residuals[k] {
                out.push_str(&format!("{},{},{:e}\n", k, s, r));
            }
        }
        std::fs::write(path, out).expect("write residual csv");
        eprintln!("[sim] residual csv written: {path}");
    }
    println!(
        "virtual makespan = {:.1} ms (ideal compute = {:.1} ms, sync overhead = {:.1}%)",
        makespan_ms,
        ideal_ms,
        (makespan_ms / ideal_ms - 1.0) * 100.0
    );
    println!(
        "msgs: sent={} lost={} resend={} replay={} | max node wait = {:.1} ms",
        sent, lost, resend, replay, max_wait_ms
    );
    let nudged = cfg.nudge_gamma > 0.0;
    if nonfinite {
        max_diff = f64::INFINITY; // 発散の明示 (NaN混入でも FAIL に落とす)
    }
    let pass = if learning || !cfg.absent.is_empty() || nudged {
        max_diff.is_finite() && !nonfinite
    } else {
        exact == n_pts && !nonfinite
    };
    if learning {
        println!(
            "RESULT: {}",
            if pass {
                format!("DONE (learning actor, Linf vs {ref_name} = {max_diff:.3e})")
            } else {
                "FAIL (NaN/inf in learning actor)".to_string()
            }
        );
    } else if !cfg.absent.is_empty() {
        println!(
            "RESULT: {}",
            if pass {
                format!("DONE (partial observation, Linf vs full single = {max_diff:.3e})")
            } else {
                "FAIL (NaN/inf in partial observation)".to_string()
            }
        );
    } else if nudged {
        let refname = if cfg.truth_grad.is_some() { "truth" } else { "single(no assim)" };
        println!(
            "RESULT: {}",
            if pass {
                format!("DONE (nudging, Linf vs {refname} = {max_diff:.3e})")
            } else {
                "FAIL (NaN/inf with nudging)".to_string()
            }
        );
    } else {
        println!("RESULT: {}", if pass { "PASS (bit-exact)" } else { "FAIL" });
    }
    if !pass {
        std::process::exit(1);
    }
}

/// 複合参照解 (シナリオC): 「同じイベント時系列を単一の配列で再現する逐次計算」。
/// 演算式・適用順序はアクター (heat1d-actor) と厳密に同一に保つこと。
/// - 各ノードの初期化はアクターの dyne_init と同じ式 (x = x0 + i*h_node, sin_pi)。
/// - 1ステップの更新は u[i] + r*(u[i-1] - 2.0*u[i] + u[i+1])。
/// - 区間端 (グローバル外縁 = Dirichlet 0、不在領域に接する面 = --unobserved-bc の
///   設定キー) はアクター dyne_step の各境界条件の式をそのまま鏡写しする (F1)。
///   透過境界の補助変数 (面ごと) もアクターと同じ漸化式・同じ総和順で更新する。
/// - 界面交換で結ばれた隣接ノード群は単一配列の連続区間と数値的に等価
///   (M-S0でビット一致を実証済み) なので、アクティブな連続区間ごとに更新する。
/// - 参加ステップ S のノードは、ステップ S の計算前に編入される。初期場は
///   白紙 = sin、ウォームスタート (F2) = 両隣の値+勾配からの外挿 (warm_field)。
/// - 参加イベントでは隣接ノードの再init (bc切替) が起きるため、その隣接ノードの
///   透過境界の補助変数をリセットする (担当替え = 再発動、理論ノートT10)。
///   handover の再init によるリセットは aux_resets (node, step) で受け取る。
#[allow(clippy::too_many_arguments)]
fn composite_reference(
    cfg: &Config,
    starts: &[(usize, usize)],
    h: f64,
    absent_at_start: &[bool],
    joins: &[JoinRec],
    aux_resets: &[(usize, u64)],
    coef: &Option<TransparentCoef>,
    want_history: bool,
) -> (Vec<f64>, Vec<Vec<f64>>) {
    let mut history = Vec::new();
    let n_nodes = starts.len();
    let n_pts = starts[n_nodes - 1].0 + starts[n_nodes - 1].1;
    let r = cfg.alpha * cfg.dt / (h * h);
    let ub_code = cfg.unobserved_bc.code();
    let ub_value = if let UBc::Dirichlet(v) = cfg.unobserved_bc { v } else { 0.0 };
    // 発注11: 全域外縁の設定 (既定 dirichlet:0 = 従来とビット同一)
    let outer_code = cfg.outer_bc.code();
    let outer_value = if let UBc::Dirichlet(v) = cfg.outer_bc { v } else { 0.0 };
    let m = coef.as_ref().map(|c| c.decay.len()).unwrap_or(0);
    let mut u = vec![0.0f64; n_pts];
    let mut un = vec![0.0f64; n_pts];
    let mut active: Vec<bool> = absent_at_start.iter().map(|&a| !a).collect();
    // 透過境界の補助変数 (アクターの t_y の鏡写し。[node][face])
    let mut ty = vec![[vec![0.0f64; m], vec![0.0f64; m]]; n_nodes];
    // 透過境界の端点更新 (アクターの transparent_delta と厳密に同一の式・総和順)
    let transparent_delta =
        |ty_face: &mut Vec<f64>, coef: &TransparentCoef, stencil: f64, r: f64, h: f64| -> f64 {
            let mut k_hist = 0.0;
            let mut s0 = 0.0;
            for j in 0..ty_face.len() {
                k_hist += coef.weight[j] * (coef.decay[j] * ty_face[j]);
                s0 += coef.weight[j] * coef.half[j];
            }
            let c2 = 2.0 * r * h;
            let delta = (r * stencil - c2 * k_hist) / (1.0 + c2 * s0);
            for j in 0..ty_face.len() {
                ty_face[j] = coef.decay[j] * ty_face[j] + coef.half[j] * delta;
            }
            delta
        };
    // dyne_init と同一の初期化 (bcがDirichletの端は境界値に丸め落とす。
    // グローバル外縁は 0、不在隣接への dirichlet(v) 面は v)
    let fill = |u: &mut Vec<f64>, i: usize, bcl: u32, bcr: u32| {
        let (s, cnt) = starts[i];
        let x0 = (s as f64) * h;
        let x1 = ((s + cnt - 1) as f64) * h;
        let hn = (x1 - x0) / ((cnt - 1) as f64);
        for k in 0..cnt {
            u[s + k] = sin_pi(x0 + (k as f64) * hn);
        }
        if bcl == BC_DIRICHLET {
            u[s] = if i == 0 { outer_value } else { ub_value };
        }
        if bcr == BC_DIRICHLET {
            u[s + cnt - 1] = if i == n_nodes - 1 { outer_value } else { ub_value };
        }
    };
    for i in 0..n_nodes {
        if active[i] {
            // 起動時のbc: 外縁は Dirichlet 0、不在隣接の面は設定キー
            // (シミュレータ本体と同一の規則)
            let bcl = if i == 0 {
                outer_code
            } else if absent_at_start[i - 1] {
                ub_code
            } else {
                BC_INTERFACE
            };
            let bcr = if i == n_nodes - 1 {
                outer_code
            } else if absent_at_start[i + 1] {
                ub_code
            } else {
                BC_INTERFACE
            };
            fill(&mut u, i, bcl, bcr);
        }
    }
    for step in 0..cfg.steps {
        // handover の再init: 透過境界の補助変数をリセット (担当替え = 再発動)
        for &(nd, _) in aux_resets.iter().filter(|&&(_, s)| s == step) {
            for face in 0..2 {
                for yj in ty[nd][face].iter_mut() {
                    *yj = 0.0;
                }
            }
        }
        // このステップから参加するノードを編入 (実行時に記録したbcを使う)
        for jr in joins.iter().filter(|j| j.step == step) {
            active[jr.node] = true;
            if jr.warm {
                // F2: 両隣 (界面がある側) の値+勾配から初期場を構成。値・勾配は
                // 隣のステップ S−1 完了時の境界 = 今の u[] から、アクターの
                // dyne_boundary_value / dyne_boundary_grad と同じ式で取り出す
                let (s, cnt) = starts[jr.node];
                let x0 = (s as f64) * h;
                let x1 = ((s + cnt - 1) as f64) * h;
                let hn = (x1 - x0) / ((cnt - 1) as f64);
                let left = (jr.bc.0 == BC_INTERFACE).then(|| {
                    let e = s - 1; // 左隣の右端 (FACE_RIGHT: 勾配 = (u[n-1]-u[n-2])/h)
                    ((e as f64) * h, u[e], (u[e] - u[e - 1]) / h)
                });
                let right = (jr.bc.1 == BC_INTERFACE).then(|| {
                    let e = s + cnt; // 右隣の左端 (FACE_LEFT: 勾配 = (u[1]-u[0])/h)
                    ((e as f64) * h, u[e], (u[e + 1] - u[e]) / h)
                });
                for k in 0..cnt {
                    u[s + k] = warm_field(left, right, x0 + (k as f64) * hn);
                }
                if jr.bc.0 == BC_DIRICHLET {
                    u[s] = outer_value; // 参加ノードの Dirichlet 面はグローバル外縁のみ
                }
                if jr.bc.1 == BC_DIRICHLET {
                    u[s + cnt - 1] = outer_value;
                }
            } else {
                fill(&mut u, jr.node, jr.bc.0, jr.bc.1);
            }
            // 隣接ノードの再init (bc切替) の鏡写し: 透過境界の補助変数をリセット
            for nb in [jr.node.checked_sub(1), Some(jr.node + 1)].into_iter().flatten() {
                if nb < n_nodes && active[nb] && nb != jr.node {
                    for face in 0..2 {
                        for yj in ty[nb][face].iter_mut() {
                            *yj = 0.0;
                        }
                    }
                }
            }
        }
        // アクティブな連続区間ごとに1ステップ。区間端はアクター dyne_step の
        // 境界条件の式を鏡写しする (グローバル外縁 = Dirichlet 0、
        // 不在領域への面 = --unobserved-bc)
        let mut i = 0;
        while i < n_nodes {
            if !active[i] {
                i += 1;
                continue;
            }
            let p = i;
            while i + 1 < n_nodes && active[i + 1] {
                i += 1;
            }
            let a = starts[p].0;
            let b = starts[i].0 + starts[i].1 - 1;
            let code_a = if p == 0 { outer_code } else { ub_code };
            let code_b = if i == n_nodes - 1 { outer_code } else { ub_code };
            let ua = match code_a {
                BC_ZERO_GRAD => u[a] + r * (u[a + 1] - 2.0 * u[a] + u[a + 1]),
                BC_LINEAR_EXTRAP => {
                    let gh = 2.0 * u[a] - u[a + 1];
                    u[a] + r * (gh - 2.0 * u[a] + u[a + 1])
                }
                BC_TRANSPARENT => {
                    let stencil = u[a + 1] - 2.0 * u[a] + u[a + 1];
                    u[a] + transparent_delta(&mut ty[p][0], coef.as_ref().unwrap(), stencil, r, h)
                }
                _ => {
                    if p == 0 {
                        outer_value
                    } else {
                        ub_value
                    }
                }
            };
            let ub = match code_b {
                BC_ZERO_GRAD => u[b] + r * (u[b - 1] - 2.0 * u[b] + u[b - 1]),
                BC_LINEAR_EXTRAP => {
                    let gh = 2.0 * u[b] - u[b - 1];
                    u[b] + r * (u[b - 1] - 2.0 * u[b] + gh)
                }
                BC_TRANSPARENT => {
                    let stencil = u[b - 1] - 2.0 * u[b] + u[b - 1];
                    u[b] + transparent_delta(&mut ty[i][1], coef.as_ref().unwrap(), stencil, r, h)
                }
                _ => {
                    if i == n_nodes - 1 {
                        outer_value
                    } else {
                        ub_value
                    }
                }
            };
            for k in a + 1..b {
                un[k] = u[k] + r * (u[k - 1] - 2.0 * u[k] + u[k + 1]);
            }
            un[a] = ua;
            un[b] = ub;
            i += 1;
        }
        std::mem::swap(&mut u, &mut un);
        if want_history {
            history.push(u.clone());
        }
    }
    (u, history)
}

/// sin(pi*x) — アクター (heat1d-actor) の sin_pi と厳密に同一の式 (複合参照解用)。
/// アクター側を変更したら必ずここも揃えること (ビット一致の前提)。
/// ※アクター側にある「上書きされる無効な最初の代入」は数値に影響しないため省略。
fn sin_pi(x: f64) -> f64 {
    const PI: f64 = std::f64::consts::PI;
    let mut t = PI * x;
    // 引数簡約: sin(t) = sin(pi - t)
    if t > PI / 2.0 {
        t = PI - t;
    }
    let t2 = t * t;
    // テイラー: t - t^3/3! + t^5/5! - t^7/7! + t^9/9! - t^11/11!
    t * (1.0
        + t2 * (-1.0 / 6.0
            + t2 * (1.0 / 120.0
                + t2 * (-1.0 / 5040.0 + t2 * (1.0 / 362880.0 + t2 * (-1.0 / 39916800.0))))))
}

/// IEEE CRC32 (dyne-proto::crc32・実機ファームの crc32_ieee と同一の式)
fn crc32_ieee(d: &[u8]) -> u32 {
    let mut c: u32 = 0xFFFF_FFFF;
    for &b in d {
        c ^= b as u32;
        for _ in 0..8 {
            let m = (c & 1).wrapping_neg();
            c = (c >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    !c
}

fn median(v: &mut Vec<f64>) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    }
}