//! dyne-sim2d: 2次元の領域分割シミュレータ (2026-10-10)
//!
//! 設計書 docs/2026-10-10-order10-design.md の「2次元の土台」と、
//! docs/2026-10-10-shape-grid-actor-design.md の「測距による形状構成」のシミュレータ段。
//! 1次元の dyne-sim には触れず (回帰の基盤を守るため)、dyne-core の共通部品 (乱数・
//! 離散イベント・仮想ネットワーク) の上に別のホストとして作る (群れの dyne-swarm-sim と同じ構成)。
//!
//! - 分割: 格子状の長方形ブロック (横 px × 縦 py)。ノード番号 = by·px + bx。
//! - アクターとの受け渡し: メールボックス方式。ホストは送信メールボックスから
//!   取り出したメッセージを、ヘッダ (界面ID・ステップ) だけ見て (界面ID, ステップ) ごとの
//!   配送単位にまとめて隣へ運び、届いた単位を受信メールボックスへ書き込む。中身は解釈しない。
//! - 同期プロトコル: dyne-sim と同じ (ステップ同期、直近2ステップの送信控え、
//!   過去ステップの重複受信には控えを再送、タイムアウト再送の最小間隔)。
//!   隣が最大4方向になり、1回の配送の中身がメッセージの列になった点だけが違う。
//! - 界面IDはクラスタ全体で一意: 縦の界面 (bx|bx+1, 行 by) = by·(px−1) + bx、
//!   横の界面 (by|by+1, 列 bx) = (px−1)·py + by·px + bx。
//! - 検証: 同じWASMを1インスタンスで全域に対して解いた単一ノード解と、全格子点の
//!   ビット一致を判定する (PASS (bit-exact))。
//!
//! アクターの種類はWASMのエクスポートで見分ける:
//! - 熱拡散 (heat2d-actor): 値固定0の境界なら解析解との誤差も出す。
//! - 格子型アクター (sdf2d-grid-actor。dyne_get_aux を持つ): --scene の模擬の部屋と
//!   模擬のセンサから測距フレームを作り、センサのあるノードへ渡す。φ・d・w の全格子点で
//!   ビット一致を判定し、正解の形状に対する採点を出す (scene.rs)。
//!
//! 使い方:
//!   dyne-sim2d --wasm heat2d_actor.wasm --px 2 --py 2 --nx 41 --ny 41 --steps 500 \
//!     --loss 0.10 --lat-ms 5 --lat-jitter-ms 3 --seed 42 \
//!     --pause 3@2000:6000 --partition 0-1@10000:15000
//!   (pause: ノード3を仮想2〜6秒停止。partition: ノード0と1の間を仮想10〜15秒分断)
//!   dyne-sim2d --wasm sdf2d_grid_actor.wasm --scene scenes/empty.txt --px 2 --py 2 --steps 300

mod scene;

use std::collections::HashMap;
use std::fmt::Write as _;

use dyne_core::event::EventQueue;
use dyne_core::net::VirtNet;
use dyne_core::rng::derive_seed;
use dyne_wire::abi::field2d::*;
use dyne_wire::abi::sdf2d;
use dyne_wire::MailboxReader;

use scene::{Scene, SensorModel};

/// UDPの1通に分割なしで入る目安 (イーサネットのMTU 1500 − IPヘッダ20 − UDPヘッダ8)
const UDP_PAYLOAD_MAX: usize = 1472;

// ---------- 設定 ----------

#[derive(Clone, Debug)]
struct Config {
    wasm: String,
    px: usize,
    py: usize,
    nx: usize,
    ny: usize,
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
    /// (node, start_us, end_us)
    pauses: Vec<(usize, u64, u64)>,
    /// (node a, node b, start_us, end_us): 期間中 a⇔b 間の通信を遮断
    partitions: Vec<(usize, usize, u64, u64)>,
    /// 外周の面の種類 (BC_DIRICHLET / BC_ZERO_GRAD)。熱拡散のみ
    outer_kind: u32,
    outer_value: f64,
    field_csv: Option<String>,
    quiet: bool,
    // ---- 格子型アクター (測距による形状構成) ----
    scene: Option<String>,
    /// 格子の刻み [m]
    h: f64,
    /// 切り詰め幅 μ [m] (None = 3h)
    mu: Option<f64>,
    w_max: f64,
    /// 統合の遅れ L (None = px + py − 1)
    delay: Option<u64>,
    noise_m: f64,
    range_def: u32,
    carve: bool,
    fov_deg: f64,
    max_range: f64,
    zones: u32,
}

fn parse_ms(s: &str) -> u64 {
    (s.parse::<f64>().expect("ミリ秒の数値") * 1000.0) as u64
}

fn parse_args() -> Config {
    let mut c = Config {
        wasm: String::new(),
        px: 2,
        py: 2,
        nx: 41,
        ny: 41,
        steps: 500,
        alpha: 1.0,
        dt: 1e-4,
        loss: 0.0,
        lat_us: 0,
        lat_jitter_us: 0,
        step_us: 79_500, // dyne-sim と同じ既定 (M1実測の1次元値。2次元の実測は実機の段で入れる)
        timeout_us: 100_000,
        resend_min_us: 20_000,
        seed: 42,
        pauses: Vec::new(),
        partitions: Vec::new(),
        outer_kind: BC_DIRICHLET,
        outer_value: 0.0,
        field_csv: None,
        quiet: false,
        scene: None,
        h: 0.05,
        mu: None,
        w_max: 50.0,
        delay: None,
        noise_m: 0.01,
        range_def: sdf2d::RANGE_DEF_RADIAL,
        carve: false,
        fov_deg: 45.0,
        max_range: 4.0,
        zones: 8,
    };
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        let key = args[i].as_str();
        let val = args.get(i + 1).cloned().unwrap_or_default();
        let mut used = true;
        match key {
            "--wasm" => c.wasm = val,
            "--px" => c.px = val.parse().unwrap(),
            "--py" => c.py = val.parse().unwrap(),
            "--nx" => c.nx = val.parse().unwrap(),
            "--ny" => c.ny = val.parse().unwrap(),
            "--steps" => c.steps = val.parse().unwrap(),
            "--alpha" => c.alpha = val.parse().unwrap(),
            "--dt" => c.dt = val.parse().unwrap(),
            "--loss" => c.loss = val.parse().unwrap(),
            "--lat-ms" => c.lat_us = parse_ms(&val),
            "--lat-jitter-ms" => c.lat_jitter_us = parse_ms(&val),
            "--step-ms" => c.step_us = parse_ms(&val),
            "--timeout-ms" => c.timeout_us = parse_ms(&val),
            "--resend-min-ms" => c.resend_min_us = parse_ms(&val),
            "--seed" => c.seed = val.parse().unwrap(),
            "--pause" => {
                // node@start_ms:end_ms
                let (n, r) = val.split_once('@').expect("--pause node@start:end");
                let (a, b) = r.split_once(':').expect("--pause node@start:end");
                c.pauses.push((n.parse().unwrap(), parse_ms(a), parse_ms(b)));
            }
            "--partition" => {
                // a-b@start_ms:end_ms
                let (ab, r) = val.split_once('@').expect("--partition a-b@start:end");
                let (a, b) = ab.split_once('-').expect("--partition a-b@start:end");
                let (s, e) = r.split_once(':').expect("--partition a-b@start:end");
                c.partitions.push((a.parse().unwrap(), b.parse().unwrap(), parse_ms(s), parse_ms(e)));
            }
            "--outer-bc" => {
                // dirichlet(値) | zero_grad
                if val == "zero_grad" {
                    c.outer_kind = BC_ZERO_GRAD;
                } else if let Some(v) = val.strip_prefix("dirichlet(").and_then(|s| s.strip_suffix(')')) {
                    c.outer_kind = BC_DIRICHLET;
                    c.outer_value = v.parse().unwrap();
                } else if val == "dirichlet" {
                    c.outer_kind = BC_DIRICHLET;
                    c.outer_value = 0.0;
                } else {
                    panic!("--outer-bc は dirichlet(値) か zero_grad");
                }
            }
            "--field-csv" => c.field_csv = Some(val),
            "--scene" => c.scene = Some(val),
            "--h" => c.h = val.parse().unwrap(),
            "--mu" => c.mu = Some(val.parse().unwrap()),
            "--w-max" => c.w_max = val.parse().unwrap(),
            "--delay" => c.delay = Some(val.parse().unwrap()),
            "--noise-m" => c.noise_m = val.parse().unwrap(),
            "--range-def" => {
                c.range_def = match val.as_str() {
                    "radial" => sdf2d::RANGE_DEF_RADIAL,
                    "perpendicular" => sdf2d::RANGE_DEF_PERPENDICULAR,
                    _ => panic!("--range-def は radial か perpendicular"),
                }
            }
            "--carve" => {
                c.carve = true;
                used = false;
            }
            "--fov-deg" => c.fov_deg = val.parse().unwrap(),
            "--max-range" => c.max_range = val.parse().unwrap(),
            "--zones" => c.zones = val.parse().unwrap(),
            "--quiet" => {
                c.quiet = true;
                used = false;
            }
            _ => panic!("未知の引数: {}", key),
        }
        i += if used { 2 } else { 1 };
    }
    assert!(!c.wasm.is_empty(), "--wasm が必要");
    c
}

// ---------- WASMアクターの実行ラッパ (2次元の場のアクター差込口) ----------

struct Actor2d {
    store: wasmi::Store<()>,
    memory: wasmi::Memory,
    set_face: wasmi::TypedFunc<(u32, u32, u64), ()>,
    init: wasmi::TypedFunc<(u32, u32, u32, u32, f64, f64, f64), i32>,
    step: wasmi::TypedFunc<(), i32>,
    inbox_write: wasmi::TypedFunc<(u32, u32), i32>,
    outbox_read: wasmi::TypedFunc<(u32, u32), u32>,
    get: wasmi::TypedFunc<u32, f64>,
    dropped: wasmi::TypedFunc<(), u64>,
    rejected: wasmi::TypedFunc<(), u64>,
    missing: wasmi::TypedFunc<(), u64>,
    // ---- 格子型アクターのみ ----
    cfg: Option<wasmi::TypedFunc<(u32, f64), ()>>,
    get_aux: Option<wasmi::TypedFunc<(u32, u32), f64>>,
    late: Option<wasmi::TypedFunc<(), u64>>,
    overflow: Option<wasmi::TypedFunc<(), u64>>,
    pending: Option<wasmi::TypedFunc<(), u32>>,
    buf: u32,
    cap: u32,
    memory_pages: u64,
}

impl Actor2d {
    fn load(engine: &wasmi::Engine, module: &wasmi::Module) -> Self {
        let mut store = wasmi::Store::new(engine, ());
        let linker: wasmi::Linker<()> = wasmi::Linker::new(engine);
        let instance = linker
            .instantiate(&mut store, module)
            .expect("instantiate")
            .start(&mut store)
            .expect("start");
        let memory = instance.get_memory(&store, "memory").expect("memory");
        let memory_pages = memory.size(&store) as u64;
        macro_rules! func {
            ($name:literal, $p:ty, $r:ty) => {
                instance.get_typed_func::<$p, $r>(&store, $name).expect($name)
            };
        }
        macro_rules! opt {
            ($name:literal, $p:ty, $r:ty) => {
                instance.get_typed_func::<$p, $r>(&store, $name).ok()
            };
        }
        let msg_buf = func!("dyne_msg_buf", (), u32);
        let msg_cap = func!("dyne_msg_cap", (), u32);
        let mut a = Actor2d {
            set_face: func!("dyne_set_face", (u32, u32, u64), ()),
            init: func!("dyne_init2d", (u32, u32, u32, u32, f64, f64, f64), i32),
            step: func!("dyne_step", (), i32),
            inbox_write: func!("dyne_inbox_write", (u32, u32), i32),
            outbox_read: func!("dyne_outbox_read", (u32, u32), u32),
            get: func!("dyne_get", u32, f64),
            dropped: func!("dyne_dropped", (), u64),
            rejected: func!("dyne_rejected", (), u64),
            missing: func!("dyne_missing", (), u64),
            cfg: opt!("dyne_cfg", (u32, f64), ()),
            get_aux: opt!("dyne_get_aux", (u32, u32), f64),
            late: opt!("dyne_late", (), u64),
            overflow: opt!("dyne_overflow", (), u64),
            pending: opt!("dyne_pending", (), u32),
            memory,
            store,
            buf: 0,
            cap: 0,
            memory_pages,
        };
        a.buf = msg_buf.call(&mut a.store, ()).unwrap();
        a.cap = msg_cap.call(&mut a.store, ()).unwrap();
        a
    }

    fn is_shape(&self) -> bool {
        self.get_aux.is_some()
    }

    /// 送信メールボックスの中身 (枠つきメッセージ列) を取り出す
    fn collect_outbox(&mut self) -> Vec<u8> {
        let n = self.outbox_read.call(&mut self.store, (self.buf, self.cap)).unwrap() as usize;
        let mut bundle = vec![0u8; n];
        if n > 0 {
            self.memory.read(&self.store, self.buf as usize, &mut bundle).unwrap();
        }
        bundle
    }

    /// 枠つきメッセージ列 (終端印なし) を受信メールボックスへ書き込む
    fn deliver(&mut self, framed: &[u8]) -> i32 {
        let mut b = Vec::with_capacity(framed.len() + 4);
        b.extend_from_slice(framed);
        b.extend_from_slice(&0u32.to_le_bytes());
        assert!(b.len() <= self.cap as usize, "配送単位が作業領域に入らない");
        self.memory.write(&mut self.store, self.buf as usize, &b).unwrap();
        self.inbox_write.call(&mut self.store, (self.buf, b.len() as u32)).unwrap()
    }

    fn cfg_set(&mut self, key: u32, val: f64) {
        let f = self.cfg.expect("dyne_cfg が無い");
        f.call(&mut self.store, (key, val)).unwrap();
    }

    fn aux(&mut self, q: u32, k: u32) -> f64 {
        self.get_aux.unwrap().call(&mut self.store, (q, k)).unwrap()
    }

    fn count(&mut self, f: Option<wasmi::TypedFunc<(), u64>>) -> u64 {
        f.map(|f| f.call(&mut self.store, ()).unwrap()).unwrap_or(0)
    }
}

// ---------- 分割 ----------

/// n 点を p 個に分ける (先頭のブロックから1点ずつ余りを配る。dyne-sim と同じ規則)
fn split(n: usize, p: usize) -> Vec<(usize, usize)> {
    let base = n / p;
    let extra = n % p;
    let mut v = Vec::new();
    let mut idx = 0;
    for i in 0..p {
        let cnt = base + if i < extra { 1 } else { 0 };
        v.push((idx, cnt));
        idx += cnt;
    }
    v
}

/// 1ノードの区画と面の設定
#[derive(Clone, Debug)]
struct Block {
    ix0: usize,
    iy0: usize,
    nx: usize,
    ny: usize,
    /// 面ごと (W, E, S, N): Some((界面ID, 相手ノード)) なら界面、None なら外周
    faces: [Option<(usize, usize)>; 4],
}

impl Block {
    fn owns(&self, gi: i64, gj: i64) -> bool {
        gi >= self.ix0 as i64
            && gi < (self.ix0 + self.nx) as i64
            && gj >= self.iy0 as i64
            && gj < (self.iy0 + self.ny) as i64
    }
}

fn build_blocks(cfg: &Config) -> Vec<Block> {
    let (px, py) = (cfg.px, cfg.py);
    let xs = split(cfg.nx, px);
    let ys = split(cfg.ny, py);
    let v_count = (px - 1) * py;
    let id_v = |bx: usize, by: usize| by * (px - 1) + bx;
    let id_h = |bx: usize, by: usize| v_count + by * px + bx;
    let mut blocks = Vec::new();
    for by in 0..py {
        for bx in 0..px {
            let node = by * px + bx;
            let mut faces = [None; 4];
            if bx > 0 {
                faces[FACE_W as usize] = Some((id_v(bx - 1, by), node - 1));
            }
            if bx + 1 < px {
                faces[FACE_E as usize] = Some((id_v(bx, by), node + 1));
            }
            if by > 0 {
                faces[FACE_S as usize] = Some((id_h(bx, by - 1), node - px));
            }
            if by + 1 < py {
                faces[FACE_N as usize] = Some((id_h(bx, by), node + px));
            }
            blocks.push(Block { ix0: xs[bx].0, iy0: ys[by].0, nx: xs[bx].1, ny: ys[by].1, faces });
        }
    }
    blocks
}

/// 格子の幾何 (熱拡散: 原点0・刻み1/(nx−1)。格子型: 部屋の外接長方形から)
#[derive(Clone, Copy, Debug)]
struct GridGeom {
    x0: f64,
    y0: f64,
    h: f64,
}

/// 格子型アクターの設定一式 (全ノード共通の部分)
#[derive(Clone, Debug)]
struct ShapeCfg {
    mu: f64,
    delay: u64,
}

/// アクターに設定・面を入れて初期化する。sensors はこのアクターに付けるセンサ
fn setup_actor(
    a: &mut Actor2d,
    b: &Block,
    cfg: &Config,
    g: &GridGeom,
    shape: Option<&ShapeCfg>,
    sensors: &[&scene::Sensor],
) {
    if let Some(sc) = shape {
        a.cfg_set(sdf2d::CFG_ORIGIN_X, g.x0);
        a.cfg_set(sdf2d::CFG_ORIGIN_Y, g.y0);
        a.cfg_set(sdf2d::CFG_MU, sc.mu);
        a.cfg_set(sdf2d::CFG_W_MAX, cfg.w_max);
        a.cfg_set(sdf2d::CFG_DELAY, sc.delay as f64);
        a.cfg_set(sdf2d::CFG_RANGE_DEF, cfg.range_def as f64);
        a.cfg_set(sdf2d::CFG_CARVE_NO_TARGET, if cfg.carve { 1.0 } else { 0.0 });
        a.cfg_set(sdf2d::CFG_MAX_RANGE, cfg.max_range);
        a.cfg_set(sdf2d::CFG_FOV, cfg.fov_deg.to_radians());
        a.cfg_set(sdf2d::CFG_ZONE_ROW, 0.0);
        for s in sensors {
            a.cfg_set(sdf2d::CFG_SENSOR_SEL, s.id as f64);
            a.cfg_set(sdf2d::CFG_SENSOR_X, s.x);
            a.cfg_set(sdf2d::CFG_SENSOR_Y, s.y);
            a.cfg_set(sdf2d::CFG_SENSOR_YAW, s.yaw);
        }
    }
    for face in 0..4u32 {
        let (kind, param) = match b.faces[face as usize] {
            Some((iface, _)) => (BC_INTERFACE, iface as u64),
            None => (cfg.outer_kind, cfg.outer_value.to_bits()),
        };
        a.set_face.call(&mut a.store, (face, kind, param)).unwrap();
    }
    let rc = a
        .init
        .call(&mut a.store, (b.nx as u32, b.ny as u32, b.ix0 as u32, b.iy0 as u32, g.h, cfg.alpha, cfg.dt))
        .unwrap();
    assert_eq!(rc, 0, "dyne_init2d が失敗 (rc={rc}, 区画 {:?})", b);
}

/// 1メッセージを枠 ([u32 len][メッセージ]) に入れて追記する
fn push_framed(out: &mut Vec<u8>, msg: &[u8]) {
    out.extend_from_slice(&(msg.len() as u32).to_le_bytes());
    out.extend_from_slice(msg);
}

/// 測距フレームのメッセージ (枠つき)
fn frame_unit(sensor: u32, step: u64, zones: &[(f64, u32)]) -> Vec<u8> {
    let mut data = vec![0u8; dyne_wire::ZONE_LEN * zones.len()];
    for (i, &(d, st)) in zones.iter().enumerate() {
        dyne_wire::range_zone_put(&mut data, i, d, st);
    }
    let hdr = dyne_wire::Header {
        kind: dyne_wire::KIND_RANGE_FRAME,
        quantity: dyne_wire::QTY_NONE,
        src: sensor,
        step,
        t_us: 0,
    };
    let n = zones.len() as u32;
    let pl = dyne_wire::Payload::RangeFrame { data: &data, count: n, cols: n };
    let mut buf = vec![0u8; dyne_wire::HDR_LEN + 8 + data.len()];
    let len = dyne_wire::encode(&mut buf, &hdr, &pl).unwrap();
    let mut out = Vec::new();
    push_framed(&mut out, &buf[..len]);
    out
}

// ---------- イベント・ノード状態機械 ----------

/// 配送単位: ある界面のあるステップのメッセージ列 (枠つき、終端印なし)。
/// ホストが見るのは界面IDとステップだけ
#[derive(Clone, Debug)]
struct Msg {
    iface: usize,
    step: u64,
    raw: Vec<u8>,
}

enum Ev {
    Deliver { node: usize, msg: Msg },
    Wake { node: usize },
    StepDone { node: usize },
}

#[derive(PartialEq)]
enum Phase {
    Waiting { step: u64 },
    Computing,
    Done,
}

struct Node {
    actor: Actor2d,
    block: Block,
    /// 界面ID の一覧 (面の順 W, E, S, N のうち界面のもの)
    ifaces: Vec<usize>,
    /// このノードに付いているセンサ (場面のセンサ一覧の添字)
    sensors: Vec<usize>,
    phase: Phase,
    cur_step: u64,
    got: Vec<bool>,
    pending: HashMap<(usize, u64), Msg>,
    sent: HashMap<(usize, u64), Msg>,
    last_resend_us: u64,
    stat_sent: u64,
    stat_lost: u64,
    stat_resend: u64,
    stat_replay: u64,
    stat_bytes: u64,
    /// 配送単位の最大バイト数 (実機ではUDPの1通に相当)
    stat_max_unit: usize,
    wait_us: u64,
    wait_since: u64,
}

struct Sim {
    cfg: Config,
    nodes: Vec<Node>,
    /// 界面ID → (ノードA, ノードB)
    iface_nodes: HashMap<usize, (usize, usize)>,
    queue: EventQueue<Ev>,
    net: VirtNet,
    now_us: u64,
    /// 測距フレーム [ステップ][センサ添字] (格子型アクターのみ)
    frames: Vec<Vec<Vec<(f64, u32)>>>,
    sensor_ids: Vec<u32>,
}

impl Sim {
    fn schedule(&mut self, t: u64, ev: Ev) {
        self.queue.schedule(t, ev);
    }

    fn peer_of(&self, node: usize, iface: usize) -> usize {
        let (a, b) = self.iface_nodes[&iface];
        if a == node { b } else { a }
    }

    fn in_pause(&self, node: usize, t: u64) -> Option<u64> {
        self.cfg
            .pauses
            .iter()
            .filter(|&&(n, a, b)| n == node && t >= a && t < b)
            .map(|&(_, _, b)| b)
            .max()
    }

    fn partition_blocks(&self, from: usize, to: usize, t: u64) -> bool {
        self.cfg.partitions.iter().any(|&(a, b, s, e)| {
            t >= s && t < e && ((a == from && b == to) || (a == to && b == from))
        })
    }

    /// 損失・分断・遅延を適用して配送をスケジュールする。分断の判定は乱数を消費しない
    /// (dyne-core::net の規約と同じ順序。分断はノードの組で指定するためホスト側で判定する)
    fn transmit(&mut self, from: usize, msg: Msg, is_resend: bool) {
        let to = self.peer_of(from, msg.iface);
        {
            let n = &mut self.nodes[from];
            n.stat_sent += 1;
            n.stat_bytes += msg.raw.len() as u64;
            if is_resend {
                n.stat_resend += 1;
            }
        }
        if self.partition_blocks(from, to, self.now_us) {
            self.nodes[from].stat_lost += 1;
            return;
        }
        let Some(t) = self.net.transit(from, to, self.now_us) else {
            self.nodes[from].stat_lost += 1;
            return;
        };
        self.schedule(t, Ev::Deliver { node: to, msg });
    }

    /// step を開始: 送信メールボックスを取り出して界面ごとの配送単位にまとめて送り、
    /// センサのあるノードには測距フレームを渡し、待ち状態に入る
    fn start_step(&mut self, node: usize, step: u64) {
        let bundle = self.nodes[node].actor.collect_outbox();
        // (界面ID) → 配送単位。並びは送信メールボックスに出た順のまま
        let mut units: Vec<Msg> = Vec::new();
        for item in MailboxReader::new(&bundle) {
            let (hdr, _, raw) = item.expect("送信メールボックスの形式不正");
            assert!(dyne_wire::routed_by_iface(hdr.kind), "界面で運ばない種別が送信メールボックスにある");
            assert_eq!(hdr.step, step, "アクターのステップ番号がホストと食い違う");
            let iface = hdr.src as usize;
            match units.iter_mut().find(|u| u.iface == iface) {
                Some(u) => push_framed(&mut u.raw, raw),
                None => {
                    let mut m = Msg { iface, step, raw: Vec::new() };
                    push_framed(&mut m.raw, raw);
                    units.push(m);
                }
            }
        }
        assert_eq!(
            units.len(),
            self.nodes[node].ifaces.len(),
            "送信メールボックスの配送単位の数が界面数と合わない (あふれ?)"
        );
        for m in units {
            let n = &mut self.nodes[node];
            n.stat_max_unit = n.stat_max_unit.max(m.raw.len());
            n.sent.insert((m.iface, step), m.clone());
            n.sent.retain(|&(_, s), _| s + 2 > step);
            self.transmit(node, m, false);
        }
        // 測距フレーム (ノード内のセンサからの入力。ネットワークを通らない)
        if !self.frames.is_empty() {
            let sensors = self.nodes[node].sensors.clone();
            for si in sensors {
                let unit = frame_unit(self.sensor_ids[si], step, &self.frames[step as usize][si]);
                let rc = self.nodes[node].actor.deliver(&unit);
                assert_eq!(rc, INBOX_OK, "測距フレームが受理されない (ノード{node})");
            }
        }
        let now = self.now_us;
        let n = &mut self.nodes[node];
        n.cur_step = step;
        n.phase = Phase::Waiting { step };
        n.got = vec![false; n.ifaces.len()];
        n.wait_since = now;
        n.last_resend_us = now;
        if n.ifaces.is_empty() {
            self.begin_compute(node);
            return;
        }
        self.try_pending(node);
        if !matches!(self.nodes[node].phase, Phase::Computing) {
            let t = self.now_us + self.cfg.timeout_us;
            self.schedule(t, Ev::Wake { node });
        }
    }

    fn accept(&mut self, node: usize, k: usize, msg: &Msg) {
        let n = &mut self.nodes[node];
        let rc = n.actor.deliver(&msg.raw);
        assert_eq!(rc, INBOX_OK, "受信メールボックスが受理しない (ノード{node}, 界面{}, ステップ{})", msg.iface, msg.step);
        n.got[k] = true;
    }

    fn try_pending(&mut self, node: usize) {
        let step = self.nodes[node].cur_step;
        for k in 0..self.nodes[node].ifaces.len() {
            if self.nodes[node].got[k] {
                continue;
            }
            let j = self.nodes[node].ifaces[k];
            if let Some(m) = self.nodes[node].pending.remove(&(j, step)) {
                self.accept(node, k, &m);
            }
        }
        if self.nodes[node].got.iter().all(|&g| g) {
            self.begin_compute(node);
        }
    }

    fn begin_compute(&mut self, node: usize) {
        let now = self.now_us;
        let n = &mut self.nodes[node];
        n.wait_us += now - n.wait_since;
        let rc = n.actor.step.call(&mut n.actor.store, ()).unwrap();
        assert_eq!(rc, STEP_OK, "未受領の界面を抱えて計算した (ノード{node})");
        n.phase = Phase::Computing;
        let t = now + self.cfg.step_us;
        self.schedule(t, Ev::StepDone { node });
    }

    fn on_deliver(&mut self, node: usize, msg: Msg) {
        let Some(k) = self.nodes[node].ifaces.iter().position(|&j| j == msg.iface) else {
            panic!("ノード{node}に無い界面{}宛のメッセージ", msg.iface);
        };
        match self.nodes[node].phase {
            Phase::Waiting { step } if msg.step == step => {
                if !self.nodes[node].got[k] {
                    self.accept(node, k, &msg);
                    if self.nodes[node].got.iter().all(|&g| g) {
                        self.begin_compute(node);
                    }
                }
            }
            _ if msg.step > self.nodes[node].cur_step
                || (msg.step == self.nodes[node].cur_step
                    && matches!(self.nodes[node].phase, Phase::Computing)) =>
            {
                // 将来分: 取り置き
                self.nodes[node].pending.insert((msg.iface, msg.step), msg);
            }
            _ => {
                // 過去分: 相手はまだこちらの値を受け取れていない。控えを再送 (replay)
                if let Some(m) = self.nodes[node].sent.get(&(msg.iface, msg.step)).cloned() {
                    self.nodes[node].stat_replay += 1;
                    self.transmit(node, m, false);
                }
            }
        }
    }

    fn on_wake(&mut self, node: usize) {
        if let Phase::Waiting { step } = self.nodes[node].phase {
            if self.now_us - self.nodes[node].last_resend_us >= self.cfg.resend_min_us {
                let missing: Vec<usize> = (0..self.nodes[node].ifaces.len())
                    .filter(|&k| !self.nodes[node].got[k])
                    .map(|k| self.nodes[node].ifaces[k])
                    .collect();
                for j in missing {
                    if let Some(m) = self.nodes[node].sent.get(&(j, step)).cloned() {
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
            // ドレイン不要: プロセス内に居続けるので過去分の replay には常時応答できる
            self.nodes[node].phase = Phase::Done;
            return;
        }
        self.start_step(node, next);
    }

    fn run(&mut self) {
        for node in 0..self.nodes.len() {
            self.start_step(node, 0);
        }
        while let Some((t, ev)) = self.queue.pop() {
            self.now_us = t;
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

// ---------- 検証 ----------

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

/// 全域の格子点列 (行優先 k = gy·nx + gx) の f64 ビット列のCRC32
fn field_crc(u: &[f64]) -> u32 {
    let mut bytes = Vec::with_capacity(u.len() * 8);
    for v in u {
        bytes.extend_from_slice(&v.to_bits().to_le_bytes());
    }
    crc32_ieee(&bytes)
}

/// 全域の場 (φ と、格子型アクターなら d・w)
struct Fields {
    phi: Vec<f64>,
    d: Vec<f64>,
    w: Vec<f64>,
}

fn read_block(a: &mut Actor2d, b: &Block, nx: usize, f: &mut Fields) {
    let shape = a.is_shape();
    for j in 0..b.ny {
        for i in 0..b.nx {
            let k = (j * b.nx + i) as u32;
            let g = (b.iy0 + j) * nx + (b.ix0 + i);
            f.phi[g] = a.get.call(&mut a.store, k).unwrap();
            if shape {
                f.d[g] = a.aux(sdf2d::AUX_D, k);
                f.w[g] = a.aux(sdf2d::AUX_W, k);
            }
        }
    }
}

fn count_mismatch(a: &[f64], b: &[f64]) -> (usize, f64) {
    let mut n = 0usize;
    let mut m = 0.0f64;
    for k in 0..a.len() {
        if a[k].to_bits() != b[k].to_bits() {
            n += 1;
            m = m.max((a[k] - b[k]).abs());
        }
    }
    (n, m)
}

/// 正解の形状に対する採点 (設計書 §4.3)
fn shape_report(out: &mut String, sc: &Scene, f: &Fields, nx: usize, ny: usize, g: &GridGeom, mu: f64) {
    let pos = |k: usize| (g.x0 + (k % nx) as f64 * g.h, g.y0 + (k / nx) as f64 * g.h);
    let truth: Vec<f64> = (0..nx * ny).map(|k| {
        let (x, y) = pos(k);
        sc.signed_distance(x, y)
    }).collect();
    let is_surf = |k: usize| f.w[k] > 0.0 && f.d[k].abs() < mu;
    // 2. 表面の近くの点の距離の誤差
    let mut se: Vec<f64> = (0..nx * ny).filter(|&k| is_surf(k)).map(|k| (f.phi[k] - truth[k]).abs()).collect();
    se.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean = |v: &[f64]| if v.is_empty() { f64::NAN } else { v.iter().sum::<f64>() / v.len() as f64 };
    let pct = |v: &[f64], p: f64| if v.is_empty() { f64::NAN } else { v[((v.len() - 1) as f64 * p) as usize] };
    // 3. 表面の覆い率
    let samples = sc.surface_samples(g.h);
    let mut covered = 0usize;
    for &(x, y) in &samples {
        let ci = ((x - g.x0) / g.h).round() as i64;
        let cj = ((y - g.y0) / g.h).round() as i64;
        let mut hit = false;
        for dj in -1..=1 {
            for di in -1..=1 {
                let (i, j) = (ci + di, cj + dj);
                if i < 0 || j < 0 || i >= nx as i64 || j >= ny as i64 {
                    continue;
                }
                let k = j as usize * nx + i as usize;
                let (px, py) = pos(k);
                if is_surf(k) && ((px - x).powi(2) + (py - y).powi(2)).sqrt() <= g.h {
                    hit = true;
                }
            }
        }
        if hit {
            covered += 1;
        }
    }
    // 4. 観測済みの空きの点 (表面の近く以外) の距離の場の誤差
    let mut fe: Vec<f64> = (0..nx * ny)
        .filter(|&k| f.w[k] > 0.0 && !is_surf(k) && f.d[k] > 0.0 && f.phi[k] < 1e29)
        .map(|k| (f.phi[k] - truth[k]).abs())
        .collect();
    fe.sort_by(|a, b| a.partial_cmp(b).unwrap());
    // 5. 未観測の割合 (部屋の空きの格子点のうち w = 0)
    let free: Vec<usize> = (0..nx * ny).filter(|&k| truth[k] > 0.0).collect();
    let unknown = free.iter().filter(|&&k| f.w[k] == 0.0).count();
    let unreached = free.iter().filter(|&&k| f.phi[k] >= 1e29).count();
    writeln!(
        out,
        "shape: surface points {} | |phi-truth| mean {:.4} m p95 {:.4} m max {:.4} m",
        se.len(), mean(&se), pct(&se, 0.95), se.last().copied().unwrap_or(f64::NAN)
    ).unwrap();
    writeln!(
        out,
        "shape: coverage {:.1}% ({} / {} surface samples at h)",
        100.0 * covered as f64 / samples.len().max(1) as f64, covered, samples.len()
    ).unwrap();
    writeln!(
        out,
        "shape: observed-free points {} | |phi-truth| mean {:.4} m p95 {:.4} m max {:.4} m",
        fe.len(), mean(&fe), pct(&fe, 0.95), fe.last().copied().unwrap_or(f64::NAN)
    ).unwrap();
    writeln!(
        out,
        "shape: unobserved {:.1}% of free grid points ({} / {}), no distance yet {}",
        100.0 * unknown as f64 / free.len().max(1) as f64, unknown, free.len(), unreached
    ).unwrap();
}

fn main() {
    let mut cfg = parse_args();
    let (px, py) = (cfg.px, cfg.py);
    assert!(px >= 1 && py >= 1, "--px/--py は1以上");
    let n_nodes = px * py;
    for &(n, _, _) in &cfg.pauses {
        assert!(n < n_nodes, "--pause のノード番号が範囲外");
    }
    for &(a, b, _, _) in &cfg.partitions {
        assert!(a < n_nodes && b < n_nodes && a != b, "--partition のノード番号が不正");
    }

    let bytes = std::fs::read(&cfg.wasm).expect("wasmの読み込み");
    let engine = wasmi::Engine::default();
    let module = wasmi::Module::new(&engine, &bytes[..]).expect("wasmのコンパイル");
    let is_shape = Actor2d::load(&engine, &module).is_shape();

    // 格子の幾何と、格子型アクターなら場面・測距フレーム
    let (geom, scene, shape_cfg) = if is_shape {
        let path = cfg.scene.clone().expect("格子型アクターには --scene が必要");
        let sc = Scene::parse(&std::fs::read_to_string(&path).expect("場面ファイルの読み込み"));
        let (x0, y0, x1, y1) = sc.bounds();
        cfg.nx = ((x1 - x0) / cfg.h).round() as usize + 1;
        cfg.ny = ((y1 - y0) / cfg.h).round() as usize + 1;
        let shape = ShapeCfg {
            mu: cfg.mu.unwrap_or(3.0 * cfg.h),
            delay: cfg.delay.unwrap_or((px + py - 1) as u64),
        };
        (GridGeom { x0, y0, h: cfg.h }, Some(sc), Some(shape))
    } else {
        assert!(cfg.scene.is_none(), "--scene は格子型アクター用");
        let h = 1.0 / ((cfg.nx - 1) as f64);
        let r = cfg.alpha * cfg.dt / (h * h);
        assert!(r <= 0.25, "陽解法の安定条件 r = α·dt/h² ≤ 1/4 を満たさない (r = {r})");
        (GridGeom { x0: 0.0, y0: 0.0, h }, None, None)
    };
    assert!(cfg.nx >= 3 && cfg.ny >= 3, "格子は3×3点以上");

    let blocks = build_blocks(&cfg);
    for (i, b) in blocks.iter().enumerate() {
        assert!(b.nx >= 3 && b.ny >= 3, "ノード{i}の区画が3×3点未満 ({}×{})", b.nx, b.ny);
    }
    let mut iface_nodes: HashMap<usize, (usize, usize)> = HashMap::new();
    for (node, b) in blocks.iter().enumerate() {
        for f in b.faces.iter().flatten() {
            let (iface, peer) = *f;
            iface_nodes.entry(iface).or_insert((node.min(peer), node.max(peer)));
        }
    }

    // センサの割り当て (センサの位置の最寄りの格子点を持つノード)
    let mut node_sensors: Vec<Vec<usize>> = vec![Vec::new(); n_nodes];
    let mut frames = Vec::new();
    let mut sensor_ids = Vec::new();
    if let Some(sc) = &scene {
        for (si, s) in sc.sensors.iter().enumerate() {
            let gi = ((s.x - geom.x0) / geom.h + 0.5).floor() as i64;
            let gj = ((s.y - geom.y0) / geom.h + 0.5).floor() as i64;
            let owner = blocks.iter().position(|b| b.owns(gi, gj)).expect("センサが格子の外");
            node_sensors[owner].push(si);
            sensor_ids.push(s.id);
        }
        let model = SensorModel {
            cols: cfg.zones,
            fov: cfg.fov_deg.to_radians(),
            max_range: cfg.max_range,
            noise_m: cfg.noise_m,
            range_def: cfg.range_def,
        };
        frames = scene::make_frames(sc, &model, cfg.steps, cfg.seed);
    }

    let mut nodes = Vec::new();
    for (ni, b) in blocks.iter().enumerate() {
        let mut actor = Actor2d::load(&engine, &module);
        let mine: Vec<&scene::Sensor> = match &scene {
            Some(sc) => node_sensors[ni].iter().map(|&si| &sc.sensors[si]).collect(),
            None => Vec::new(),
        };
        setup_actor(&mut actor, b, &cfg, &geom, shape_cfg.as_ref(), &mine);
        let ifaces: Vec<usize> = b.faces.iter().flatten().map(|&(j, _)| j).collect();
        nodes.push(Node {
            actor,
            block: b.clone(),
            got: vec![false; ifaces.len()],
            ifaces,
            sensors: node_sensors[ni].clone(),
            phase: Phase::Waiting { step: 0 },
            cur_step: 0,
            pending: HashMap::new(),
            sent: HashMap::new(),
            last_resend_us: 0,
            stat_sent: 0,
            stat_lost: 0,
            stat_resend: 0,
            stat_replay: 0,
            stat_bytes: 0,
            stat_max_unit: 0,
            wait_us: 0,
            wait_since: 0,
        });
    }
    let mem_pages = nodes[0].actor.memory_pages;

    let mut sim = Sim {
        net: VirtNet::new(
            cfg.loss,
            cfg.lat_us,
            cfg.lat_jitter_us,
            Vec::new(),
            derive_seed(cfg.seed, u64::MAX),
        ),
        cfg: cfg.clone(),
        nodes,
        iface_nodes,
        queue: EventQueue::new(),
        now_us: 0,
        frames,
        sensor_ids,
    };
    sim.run();

    // 分割解を全域に組み立てる
    let total = cfg.nx * cfg.ny;
    let blank = || Fields { phi: vec![f64::NAN; total], d: vec![0.0; total], w: vec![0.0; total] };
    let mut split_f = blank();
    for n in sim.nodes.iter_mut() {
        let b = n.block.clone();
        read_block(&mut n.actor, &b, cfg.nx, &mut split_f);
    }

    // 単一ノード解 (同じWASMの別インスタンスで全域を解く。センサは全部このノードに付ける)
    let whole = Block { ix0: 0, iy0: 0, nx: cfg.nx, ny: cfg.ny, faces: [None; 4] };
    let mut single = Actor2d::load(&engine, &module);
    let all_sensors: Vec<&scene::Sensor> = match &scene {
        Some(sc) => sc.sensors.iter().collect(),
        None => Vec::new(),
    };
    setup_actor(&mut single, &whole, &cfg, &geom, shape_cfg.as_ref(), &all_sensors);
    for s in 0..cfg.steps {
        for si in 0..sim.sensor_ids.len() {
            let unit = frame_unit(sim.sensor_ids[si], s, &sim.frames[s as usize][si]);
            assert_eq!(single.deliver(&unit), INBOX_OK);
        }
        let rc = single.step.call(&mut single.store, ()).unwrap();
        assert_eq!(rc, STEP_OK);
    }
    let mut single_f = blank();
    read_block(&mut single, &whole, cfg.nx, &mut single_f);

    let (mut mismatch, mut max_diff) = count_mismatch(&split_f.phi, &single_f.phi);
    if is_shape {
        for (a, b) in [(&split_f.d, &single_f.d), (&split_f.w, &single_f.w)] {
            let (n, m) = count_mismatch(a, b);
            mismatch += n;
            max_diff = max_diff.max(m);
        }
    }

    // 解析解 (熱拡散で値固定0の外周のとき): u = exp(−2π²αt)·sin(πx)·sin(πy)。
    // 領域が正方形 (nx=ny) のときだけ厳密解になる
    let pi = std::f64::consts::PI;
    let t_end = cfg.steps as f64 * cfg.dt;
    let analytic_linf = if !is_shape && cfg.outer_kind == BC_DIRICHLET && cfg.outer_value == 0.0 && cfg.nx == cfg.ny {
        let h = geom.h;
        let decay = libm::exp(-2.0 * pi * pi * cfg.alpha * t_end);
        let mut e = 0.0f64;
        for gy in 0..cfg.ny {
            for gx in 0..cfg.nx {
                let ex = decay * libm::sin(pi * gx as f64 * h) * libm::sin(pi * gy as f64 * h);
                e = e.max((single_f.phi[gy * cfg.nx + gx] - ex).abs());
            }
        }
        Some(e)
    } else {
        None
    };

    // 報告
    let mut out = String::new();
    if let Some(sc) = &shape_cfg {
        writeln!(
            out,
            "dyne-sim2d: {}x{} nodes, grid {}x{} (h = {} m, origin ({}, {})), steps {}, shape grid actor, mu = {} m, delay L = {}, wasm memory {} pages",
            px, py, cfg.nx, cfg.ny, geom.h, geom.x0, geom.y0, cfg.steps, sc.mu, sc.delay, mem_pages
        )
        .unwrap();
        writeln!(
            out,
            "scene: {} | sensors {} | zones {} fov {} deg max-range {} m noise {} m range-def {} carve {}",
            cfg.scene.as_deref().unwrap_or(""),
            sim.sensor_ids.len(),
            cfg.zones,
            cfg.fov_deg,
            cfg.max_range,
            cfg.noise_m,
            if cfg.range_def == sdf2d::RANGE_DEF_PERPENDICULAR { "perpendicular" } else { "radial" },
            cfg.carve
        )
        .unwrap();
        if !cfg.quiet {
            if let Some(scn) = &scene {
                for (si, se) in scn.sensors.iter().enumerate() {
                    let node = node_sensors.iter().position(|v| v.contains(&si)).unwrap();
                    writeln!(
                        out,
                        "sensor {}: at ({}, {}) yaw {} deg height {} m -> node {}",
                        se.id, se.x, se.y, se.yaw.to_degrees().round(), se.height, node
                    )
                    .unwrap();
                }
            }
        }
    } else {
        let r = cfg.alpha * cfg.dt / (geom.h * geom.h);
        let outer = if cfg.outer_kind == BC_ZERO_GRAD {
            "zero_grad".to_string()
        } else {
            format!("dirichlet({})", cfg.outer_value)
        };
        writeln!(
            out,
            "dyne-sim2d: {}x{} nodes, grid {}x{}, steps {}, r = {:.4}, outer = {}, wasm memory {} pages",
            px, py, cfg.nx, cfg.ny, cfg.steps, r, outer, mem_pages
        )
        .unwrap();
    }
    writeln!(
        out,
        "net: loss {} lat {}us jitter {}us, step {}us, timeout {}us, resend-min {}us, seed {}",
        cfg.loss, cfg.lat_us, cfg.lat_jitter_us, cfg.step_us, cfg.timeout_us, cfg.resend_min_us, cfg.seed
    )
    .unwrap();
    if !cfg.pauses.is_empty() || !cfg.partitions.is_empty() {
        writeln!(out, "faults: pauses {:?} partitions {:?}", cfg.pauses, cfg.partitions).unwrap();
    }
    let mut late_total = 0u64;
    let mut overflow_total = 0u64;
    let mut max_unit = 0usize;
    for (i, n) in sim.nodes.iter_mut().enumerate() {
        let dropped = n.actor.count(Some(n.actor.dropped));
        let rejected = n.actor.count(Some(n.actor.rejected));
        let missing = n.actor.count(Some(n.actor.missing));
        let late = n.actor.count(n.actor.late);
        let overflow = n.actor.count(n.actor.overflow);
        late_total += late;
        overflow_total += overflow;
        max_unit = max_unit.max(n.stat_max_unit);
        if !cfg.quiet {
            let b = &n.block;
            let extra = if is_shape {
                let pend = n.actor.pending.map(|f| f.call(&mut n.actor.store, ()).unwrap()).unwrap_or(0);
                format!(
                    " sensors {:?} max-unit {} B late {} overflow {} pending {}",
                    n.sensors.iter().map(|&si| sim.sensor_ids[si]).collect::<Vec<_>>(),
                    n.stat_max_unit,
                    late,
                    overflow,
                    pend
                )
            } else {
                String::new()
            };
            writeln!(
                out,
                "node {:>2}: [{}..{})x[{}..{}) ifaces {:?} sent {} ({} B) lost {} resend {} replay {} wait {:.1}ms dropped {} rejected {} missing {}{}",
                i,
                b.ix0,
                b.ix0 + b.nx,
                b.iy0,
                b.iy0 + b.ny,
                n.ifaces,
                n.stat_sent,
                n.stat_bytes,
                n.stat_lost,
                n.stat_resend,
                n.stat_replay,
                n.wait_us as f64 / 1000.0,
                dropped,
                rejected,
                missing,
                extra
            )
            .unwrap();
        }
    }
    writeln!(out, "virtual time: {:.3} s", sim.now_us as f64 / 1e6).unwrap();
    if let Some(e) = analytic_linf {
        writeln!(out, "analytic: Linf = {:.3e} (t = {})", e, t_end).unwrap();
    }
    let mut pass = mismatch == 0;
    if let (Some(sc), Some(shape)) = (&scene, &shape_cfg) {
        let single_late = single.count(single.late);
        let single_overflow = single.count(single.overflow);
        writeln!(
            out,
            "rays: late {} overflow {} (single: late {} overflow {}) | max delivery unit {} B{}",
            late_total,
            overflow_total,
            single_late,
            single_overflow,
            max_unit,
            if max_unit > UDP_PAYLOAD_MAX { format!(" — WARNING: exceeds one UDP datagram ({} B)", UDP_PAYLOAD_MAX) } else { String::new() }
        )
        .unwrap();
        if late_total + overflow_total + single_late + single_overflow > 0 {
            pass = false;
        }
        shape_report(&mut out, sc, &split_f, cfg.nx, cfg.ny, &geom, shape.mu);
        writeln!(
            out,
            "crc32: split phi {:08x} d {:08x} w {:08x} | single phi {:08x} d {:08x} w {:08x}",
            field_crc(&split_f.phi),
            field_crc(&split_f.d),
            field_crc(&split_f.w),
            field_crc(&single_f.phi),
            field_crc(&single_f.d),
            field_crc(&single_f.w)
        )
        .unwrap();
    } else {
        writeln!(
            out,
            "crc32: split {:08x} single {:08x}",
            field_crc(&split_f.phi),
            field_crc(&single_f.phi)
        )
        .unwrap();
    }
    if mismatch == 0 && pass {
        writeln!(out, "PASS (bit-exact): {} points", total).unwrap();
    } else if mismatch == 0 {
        writeln!(out, "FAIL: bit-exact but rays were late or overflowed").unwrap();
    } else {
        writeln!(out, "FAIL: {} values differ, max |diff| = {:e}", mismatch, max_diff).unwrap();
    }
    print!("{}", out);

    if let Some(path) = &cfg.field_csv {
        let mut s = String::new();
        if let Some(sc) = &scene {
            s.push_str("gx,gy,x,y,phi,d,w,truth\n");
            for gy in 0..cfg.ny {
                for gx in 0..cfg.nx {
                    let k = gy * cfg.nx + gx;
                    let (x, y) = (geom.x0 + gx as f64 * geom.h, geom.y0 + gy as f64 * geom.h);
                    writeln!(
                        s,
                        "{},{},{},{},{:e},{:e},{},{:e}",
                        gx, gy, x, y, split_f.phi[k], split_f.d[k], split_f.w[k], sc.signed_distance(x, y)
                    )
                    .unwrap();
                }
            }
        } else {
            s.push_str("gx,gy,x,y,u_split,u_single\n");
            for gy in 0..cfg.ny {
                for gx in 0..cfg.nx {
                    let k = gy * cfg.nx + gx;
                    writeln!(
                        s,
                        "{},{},{},{},{:e},{:e}",
                        gx, gy, gx as f64 * geom.h, gy as f64 * geom.h, split_f.phi[k], single_f.phi[k]
                    )
                    .unwrap();
                }
            }
        }
        std::fs::write(path, s).expect("--field-csv の書き込み");
    }

    std::process::exit(if mismatch == 0 && pass { 0 } else { 1 });
}
