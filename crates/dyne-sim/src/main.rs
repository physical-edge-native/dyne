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

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

const FACE_LEFT: u32 = 0;
const FACE_RIGHT: u32 = 1;
const BC_DIRICHLET: u32 = 0;
const BC_INTERFACE: u32 = 1;

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
    quiet: bool,
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
    c
}

// ---------- 決定論的乱数 (xorshift64*) ----------

struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed.max(1))
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range_u64(&mut self, hi: u64) -> u64 {
        if hi == 0 { 0 } else { self.next_u64() % (hi * 2 + 1) }
    }
}

// ---------- アクター(wasmi) ----------

struct Actor {
    store: wasmi::Store<()>,
    init: wasmi::TypedFunc<(u32, f64, f64, f64, f64, u32, u32), i32>,
    step: wasmi::TypedFunc<(), ()>,
    bval: wasmi::TypedFunc<u32, f64>,
    bgrad: wasmi::TypedFunc<u32, f64>,
    set_neighbor: wasmi::TypedFunc<(u32, f64, f64), ()>,
    set: wasmi::TypedFunc<(u32, f64), ()>,
    get: wasmi::TypedFunc<u32, f64>,
    x: wasmi::TypedFunc<u32, f64>,
    len: wasmi::TypedFunc<(), u32>,
    /// wasm宣言の線形メモリ(ページ) — RAM上限チェック用
    memory_pages: u64,
}

impl Actor {
    fn load(engine: &wasmi::Engine, module: &wasmi::Module) -> Self {
        let mut store = wasmi::Store::new(engine, ());
        let linker: wasmi::Linker<()> = wasmi::Linker::new(engine);
        let instance = linker
            .instantiate(&mut store, module)
            .expect("instantiate")
            .start(&mut store)
            .expect("start");
        let memory_pages = instance
            .get_memory(&store, "memory")
            .map(|m| m.size(&store) as u64)
            .unwrap_or(0);
        macro_rules! func {
            ($name:literal, $p:ty, $r:ty) => {
                instance.get_typed_func::<$p, $r>(&store, $name).expect($name)
            };
        }
        Actor {
            init: func!("dyne_init", (u32, f64, f64, f64, f64, u32, u32), i32),
            step: func!("dyne_step", (), ()),
            bval: func!("dyne_boundary_value", u32, f64),
            bgrad: func!("dyne_boundary_grad", u32, f64),
            set_neighbor: func!("dyne_set_neighbor", (u32, f64, f64), ()),
            set: func!("dyne_set", (u32, f64), ()),
            get: func!("dyne_get", u32, f64),
            x: func!("dyne_x", u32, f64),
            len: func!("dyne_len", (), u32),
            store,
            memory_pages,
        }
    }
}

// ---------- メッセージとイベント ----------

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
    phase: Phase,
    cur_step: u64,
    /// 現ステップで受領済みの界面
    got: Vec<bool>,
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

struct Sim {
    cfg: Config,
    nodes: Vec<Node>,
    /// 界面ID j はノード j と j+1 を結ぶ
    now_us: u64,
    heap: BinaryHeap<Reverse<(u64, u64, usize)>>, // (time, seq, ev_index)
    events: Vec<Option<Ev>>,
    seq: u64,
    rng: Rng,
}

impl Sim {
    fn schedule(&mut self, t: u64, ev: Ev) {
        self.events.push(Some(ev));
        let idx = self.events.len() - 1;
        self.seq += 1;
        self.heap.push(Reverse((t, self.seq, idx)));
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
    fn apply_handovers(&mut self, engine: &wasmi::Engine, module: &wasmi::Module) {
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
            let mut actor = Actor::load(engine, module);
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
            eprintln!(
                "[sim] handover applied: node {} reincarnated at t={}ms (state {} pts carried)",
                node,
                self.now_us / 1000,
                n_pts
            );
        }
    }

    fn cfg_alpha(&self) -> f64 { self.cfg.alpha }
    fn cfg_dt(&self) -> f64 { self.cfg.dt }

    fn partition_blocks(&self, from: usize, to: usize, t: u64) -> bool {
        self.cfg.partitions.iter().any(|&(k, a, b)| {
            t >= a && t < b && ((from < k) != (to < k))
        })
    }

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
        if self.partition_blocks(from, to, self.now_us) {
            self.nodes[from].stat_lost += 1;
            return;
        }
        if self.cfg.loss > 0.0 && self.rng.uniform() < self.cfg.loss {
            self.nodes[from].stat_lost += 1;
            return;
        }
        let lat = self.cfg.lat_us + self.rng.range_u64(self.cfg.lat_jitter_us);
        let t = self.now_us + lat.max(1);
        self.schedule(t, Ev::Deliver { node: to, msg });
    }

    /// stepを開始: 全界面へ送信し、待ち状態に入る
    fn start_step(&mut self, node: usize, step: u64) {
        let ifaces: Vec<usize> = self.nodes[node].ifaces.iter().map(|&(j, _)| j).collect();
        for j in &ifaces {
            self.send_iface(node, *j, step, false);
        }
        let n = &mut self.nodes[node];
        n.cur_step = step;
        n.phase = Phase::Waiting { step };
        n.got = vec![false; n.ifaces.len()];
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
            }
        }
        if self.nodes[node].got.iter().all(|&g| g) {
            self.begin_compute(node);
        }
    }

    fn begin_compute(&mut self, node: usize) {
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
            return; // 自分の界面でない(ありえないが安全側)
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
        let next = self.nodes[node].cur_step + 1;
        if next >= self.cfg.steps {
            self.nodes[node].phase = Phase::Done;
            // ドレインは不要: プロセス内に居続けるので過去分replayには常時応答できる
            return;
        }
        self.start_step(node, next);
    }

    fn run(&mut self, engine: &wasmi::Engine, module: &wasmi::Module) {
        for node in 0..self.nodes.len() {
            self.start_step(node, 0);
        }
        while let Some(Reverse((t, _, idx))) = self.heap.pop() {
            self.now_us = t;
            self.apply_handovers(engine, module);
            let ev = self.events[idx].take().expect("event consumed twice");
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
    let probe = Actor::load(&engine, &module);
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

    // ノード生成
    let mut nodes = Vec::new();
    for i in 0..n_nodes {
        let (s, cnt) = starts[i];
        let x0 = (s as f64) * h;
        let x1 = ((s + cnt - 1) as f64) * h;
        let bc_l = if i == 0 { BC_DIRICHLET } else { BC_INTERFACE };
        let bc_r = if i == n_nodes - 1 { BC_DIRICHLET } else { BC_INTERFACE };
        let mut actor = Actor::load(&engine, &module);
        let rc = actor
            .init
            .call(
                &mut actor.store,
                (cnt as u32, x0, x1, cfg.alpha, cfg.dt, bc_l, bc_r),
            )
            .unwrap();
        assert_eq!(rc, 0, "dyne_init failed on node {i}");
        let mut ifaces = Vec::new();
        if i > 0 {
            ifaces.push((i - 1, FACE_LEFT)); // 界面 i-1 = 左隣との境界
        }
        if i < n_nodes - 1 {
            ifaces.push((i, FACE_RIGHT)); // 界面 i = 右隣との境界
        }
        nodes.push(Node {
            actor,
            init_args: (cnt as u32, x0, x1, bc_l, bc_r),
            ifaces,
            phase: Phase::Waiting { step: 0 },
            cur_step: 0,
            got: Vec::new(),
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

    let mut sim = Sim {
        cfg: cfg.clone(),
        nodes,
        now_us: 0,
        heap: BinaryHeap::new(),
        events: Vec::new(),
        seq: 0,
        rng: Rng::new(cfg.seed),
    };
    sim.run(&engine, &module);

    // 単一ノード参照解(同一wasm・全域)との突き合わせ
    let mut single = Actor::load(&engine, &module);
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

    let mut exact = 0usize;
    let mut max_diff = 0.0f64;
    let mut l2 = 0.0f64;
    let mut linf = 0.0f64;
    let t_final = cfg.dt * (cfg.steps as f64);
    let decay = (-cfg.alpha * std::f64::consts::PI.powi(2) * t_final).exp();
    for i in 0..n_nodes {
        let (s, cnt) = starts[i];
        for k in 0..cnt {
            let gi = (s + k) as u32;
            let n = &mut sim.nodes[i];
            let u = n.actor.get.call(&mut n.actor.store, k as u32).unwrap();
            let x = n.actor.x.call(&mut n.actor.store, k as u32).unwrap();
            let us = single.get.call(&mut single.store, gi).unwrap();
            let d = (u - us).abs();
            if u == us {
                exact += 1;
            }
            max_diff = max_diff.max(d);
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
    println!(
        "bit-exact vs single: {}/{} points, max_diff={:.3e}",
        exact, n_pts, max_diff
    );
    println!("L2 vs analytic = {:.3e}, Linf = {:.3e}", l2, linf);
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
    let pass = exact == n_pts;
    println!("RESULT: {}", if pass { "PASS (bit-exact)" } else { "FAIL" });
    if !pass {
        std::process::exit(1);
    }
}