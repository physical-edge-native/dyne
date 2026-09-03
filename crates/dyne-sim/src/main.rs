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

use dyne_core::actor::{Actor, BC_DIRICHLET, BC_INTERFACE, FACE_LEFT, FACE_RIGHT};
use dyne_core::event::EventQueue;
use dyne_core::net::VirtNet;

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
            let rc = actor
                .init
                .call(
                    &mut actor.store,
                    (n, x0, x1, self.cfg_alpha(), self.cfg_dt(), bcl, bcr),
                )
                .unwrap();
            assert_eq!(rc, 0, "handover re-init failed");
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
            let mut bc = [BC_DIRICHLET, BC_DIRICHLET];
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
            // 学習型アクター: ADDで参加するインスタンスの温め回数を上書き (--add-warm)
            if let (Some(w), Some(f)) = (self.cfg.add_warm, actor.cfg) {
                f.call(&mut actor.store, (2, w)).unwrap();
            }
            let (n, x0, x1, _, _) = self.nodes[node].init_args;
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
            }
            self.join_log.push(JoinRec {
                node,
                step: s_join,
                bc: (bc[0], bc[1]),
                t_us: t_add,
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

    /// stepを開始: 全界面へ送信し、待ち状態に入る
    fn start_step(&mut self, node: usize, step: u64) {
        self.apply_iface_adds(node, step);
        let ifaces: Vec<usize> = self.nodes[node].ifaces.iter().map(|&(j, _)| j).collect();
        for j in &ifaces {
            self.send_iface(node, *j, step, false);
        }
        let n = &mut self.nodes[node];
        n.cur_step = step;
        n.phase = Phase::Waiting { step };
        n.got = vec![false; n.ifaces.len()];
        n.recv_grad = vec![0.0; n.ifaces.len()];
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
            }
        }
        if self.nodes[node].got.iter().all(|&g| g) {
            self.begin_compute(node);
        }
    }

    fn begin_compute(&mut self, node: usize) {
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
            if self.nodes.iter().all(|n| n.phase == Phase::Done) {
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
    let h = 1.0 / ((n_pts - 1) as f64);
    // 陽解法の安定条件 (M0ホストと同じ検証)
    let r = cfg.alpha * cfg.dt / (h * h);
    assert!(
        r <= 0.5,
        "stability violated: alpha*dt/h^2 = {r:.3} > 0.5 (dtを下げるかpointsを減らすこと)"
    );
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

    // ノード生成。--add 指定ノードは開始時不在 (β意味論):
    // 隣接ノードは当該面を BC_DIRICHLET_ZERO (外部境界u=0)・界面登録なしで初期化する。
    let absent_at_start: Vec<bool> = (0..n_nodes)
        .map(|i| cfg.adds.iter().any(|&(a, _)| a == i))
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
        let bc_l = if i == 0 || absent_at_start[i - 1] {
            BC_DIRICHLET
        } else {
            BC_INTERFACE
        };
        let bc_r = if i == n_nodes - 1 || absent_at_start[i + 1] {
            BC_DIRICHLET
        } else {
            BC_INTERFACE
        };
        let mut actor = Actor::load_with(&engine, &module, &cfg.pinn_cfg);
        if present {
            let rc = actor
                .init
                .call(
                    &mut actor.store,
                    (cnt as u32, x0, x1, cfg.alpha, cfg.dt, bc_l, bc_r),
                )
                .unwrap();
            assert_eq!(rc, 0, "dyne_init failed on node {i}");
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
    let mut ref_history: Vec<Vec<f64>> = Vec::new();
    let reference: Vec<f64> = if cfg.adds.is_empty() {
        let mut single = Actor::load_with(&sim.engine, &sim.module, &cfg.pinn_cfg);
        let rc = single
            .init
            .call(
                &mut single.store,
                (n_pts as u32, 0.0, 1.0, cfg.alpha, cfg.dt, BC_DIRICHLET, BC_DIRICHLET),
            )
            .unwrap();
        assert_eq!(rc, 0);
        for _ in 0..cfg.steps {
            single.step.call(&mut single.store, ()).unwrap();
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
            cfg.error_csv.is_some(),
        );
        ref_history = hist;
        r
    };
    let learning = sim.nodes[0].actor.is_learning();

    let mut exact = 0usize;
    let mut max_diff = 0.0f64;
    let mut l2 = 0.0f64;
    let mut linf = 0.0f64;
    let t_final = cfg.dt * (cfg.steps as f64);
    let decay = (-cfg.alpha * std::f64::consts::PI.powi(2) * t_final).exp();
    let mut node_linf_ref = vec![0.0f64; n_nodes];
    for i in 0..n_nodes {
        let (s, cnt) = starts[i];
        for k in 0..cnt {
            let gi = s + k;
            let n = &mut sim.nodes[i];
            let u = n.actor.get.call(&mut n.actor.store, k as u32).unwrap();
            let x = n.actor.x.call(&mut n.actor.store, k as u32).unwrap();
            let us = reference[gi];
            let d = (u - us).abs();
            if u == us {
                exact += 1;
            }
            max_diff = max_diff.max(d);
            node_linf_ref[i] = node_linf_ref[i].max(d);
            let e = (u - (std::f64::consts::PI * x).sin() * decay).abs();
            l2 += e * e;
            linf = linf.max(e);
        }
    }
    l2 = (l2 / (n_pts as f64)).sqrt();

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
    } else {
        println!(
            "bit-exact vs {}: {}/{} points, max_diff={:.3e}",
            ref_name, exact, n_pts, max_diff
        );
    }
    if cfg.adds.is_empty() {
        println!("L2 vs analytic = {:.3e}, Linf = {:.3e}", l2, linf);
    } else {
        // 領域が途中から拡張されるため sin減衰の解析解は適用外
        println!("L2 vs analytic: n/a (領域拡張シナリオ)");
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
                let r = if cfg.adds.is_empty() {
                    (std::f64::consts::PI * (gi as f64) * h).sin() * dec
                } else {
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
    let pass = if learning { max_diff.is_finite() } else { exact == n_pts };
    if learning {
        println!(
            "RESULT: {}",
            if pass {
                format!("DONE (learning actor, Linf vs {ref_name} = {max_diff:.3e})")
            } else {
                "FAIL (NaN/inf in learning actor)".to_string()
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
/// - 1ステップの更新は u[i] + r*(u[i-1] - 2.0*u[i] + u[i+1])。区間端は
///   BC_DIRICHLET_ZERO 相当の 0.0 (不在領域に接する面もβ意味論で外部境界 u=0)。
/// - 界面交換で結ばれた隣接ノード群は単一配列の連続区間と数値的に等価
///   (M-S0でビット一致を実証済み) なので、アクティブな連続区間ごとに更新する。
/// - 参加ステップ S のノードは、ステップ S の計算前に sin 初期値で編入される。
fn composite_reference(
    cfg: &Config,
    starts: &[(usize, usize)],
    h: f64,
    absent_at_start: &[bool],
    joins: &[JoinRec],
    want_history: bool,
) -> (Vec<f64>, Vec<Vec<f64>>) {
    let mut history = Vec::new();
    let n_nodes = starts.len();
    let n_pts = starts[n_nodes - 1].0 + starts[n_nodes - 1].1;
    let r = cfg.alpha * cfg.dt / (h * h);
    let mut u = vec![0.0f64; n_pts];
    let mut un = vec![0.0f64; n_pts];
    let mut active: Vec<bool> = absent_at_start.iter().map(|&a| !a).collect();
    // dyne_init と同一の初期化 (bcがDirichletの端は0に丸め落とす)
    let fill = |u: &mut Vec<f64>, i: usize, bcl: u32, bcr: u32| {
        let (s, cnt) = starts[i];
        let x0 = (s as f64) * h;
        let x1 = ((s + cnt - 1) as f64) * h;
        let hn = (x1 - x0) / ((cnt - 1) as f64);
        for k in 0..cnt {
            u[s + k] = sin_pi(x0 + (k as f64) * hn);
        }
        if bcl == BC_DIRICHLET {
            u[s] = 0.0;
        }
        if bcr == BC_DIRICHLET {
            u[s + cnt - 1] = 0.0;
        }
    };
    for i in 0..n_nodes {
        if active[i] {
            // 起動時のbc: 外縁と不在隣接の面が Dirichlet (シミュレータ本体と同一の規則)
            let bcl = if i == 0 || absent_at_start[i - 1] {
                BC_DIRICHLET
            } else {
                BC_INTERFACE
            };
            let bcr = if i == n_nodes - 1 || absent_at_start[i + 1] {
                BC_DIRICHLET
            } else {
                BC_INTERFACE
            };
            fill(&mut u, i, bcl, bcr);
        }
    }
    for step in 0..cfg.steps {
        // このステップから参加するノードを sin 初期値で編入 (実行時に記録したbcを使う)
        for jr in joins.iter().filter(|j| j.step == step) {
            active[jr.node] = true;
            fill(&mut u, jr.node, jr.bc.0, jr.bc.1);
        }
        // アクティブな連続区間ごとに1ステップ。区間端は 0.0 (グローバル境界または
        // 不在領域への面 = どちらも Dirichlet 0)
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
            for k in a + 1..b {
                un[k] = u[k] + r * (u[k - 1] - 2.0 * u[k] + u[k + 1]);
            }
            un[a] = 0.0;
            un[b] = 0.0;
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