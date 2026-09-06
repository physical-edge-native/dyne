//! dyne-swarm-sim: 群れデモのWASMアクター版ホスト (発注6 第2段、2026-09-04)
//!
//! 個体の規則は crates/swarm-agent-actor (WASM) に閉じ、ホストは環境だけを担う:
//!   - 温度場 (真値の計算。個体には自分の位置の温度しか渡さない)
//!   - 近傍決定 (無線の到達範囲 = 半径内の近い順 k 個体。個体は選べない)
//!   - メッセージ束の配送 (dyne-core の仮想ネットワーク: 損失・遅延・ジッタ)
//!   - 離散イベントの仮想時刻 (dyne-core のイベントキュー)
//!
//! 1ステップは知覚位相と判断位相の2位相 (仮想時刻で D=--step-ms を1周期とし、
//! 知覚を周期頭、判断を 3D/4 に置く)。既定 (損失0・遅延0) では配送は知覚+1µs に
//! 完了し、旧 crates/dyne-swarm と同じ情報タイミング (隣の現在位置・現在知覚温度・
//! 前ステップの勾配推定と伝聞) になる。遅延が 3D/4 を超えた便は次ステップ以降の
//! 判断に回る (混沌注入の意味論)。
//!
//! 乱数は3系統で分離: 初期配置 = Rng::new(seed)、個体の内蔵乱数 =
//! derive_seed(seed, id) を dyne_cfg で注入、ネットワーク = derive_seed(seed, u64::MAX)。
//! 全て整数演算のみのシード導出なので同一入力で全出力がビット一致する。
//!
//! 出力 (--out-dir): 旧 dyne-swarm と同一形式の traj.csv / metrics.csv / field.csv /
//! summary.txt (summary には net 行を追加)。

use std::fmt::Write as _;

use dyne_core::event::EventQueue;
use dyne_core::net::VirtNet;
use dyne_core::rng::{derive_seed, Rng};
use dyne_wire::abi::*;

// ---------- 設定 ----------

#[derive(Clone, Debug)]
struct Config {
    wasm: String,
    n: usize,
    steps: u64,
    k: usize,
    radius: f64,
    mode: Mode,
    seed: u64,
    t_opt: f64,
    tol: f64,
    speed: f64,
    noise: f64,
    field: FieldKind,
    start: (f64, f64, f64), // 初期配置の中心 x, y と半径
    out_dir: String,
    field_grid: usize,
    quiet: bool,
    // ---- 混沌注入 (dyne-sim と同じ流儀) ----
    loss: f64,
    lat_us: u64,
    lat_jitter_us: u64,
    /// 1ステップの仮想時間幅 [µs]
    step_us: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode {
    Consensus,
    Gossip,
    Follow,
    Solo,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum FieldKind {
    Bumps,
    Linear,
    Complex,
}

fn parse_args() -> Config {
    let mut c = Config {
        wasm: String::new(),
        n: 40,
        steps: 600,
        k: 6,
        radius: 0.12,
        mode: Mode::Consensus,
        seed: 42,
        t_opt: 24.0,
        tol: 1.0,
        speed: 0.004,
        noise: 0.3,
        field: FieldKind::Complex,
        start: (-1.0, -1.0, 0.0), // 負 = 全域に一様ランダム散布 (--start x,y,r で円内に固める)
        out_dir: "swarm_out".to_string(),
        field_grid: 80,
        quiet: false,
        loss: 0.0,
        lat_us: 0,
        lat_jitter_us: 0,
        step_us: 100_000,
    };
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        let key = args[i].as_str();
        let val = || args.get(i + 1).cloned().unwrap_or_default();
        match key {
            "--wasm" => c.wasm = val(),
            "--n" => c.n = val().parse().unwrap(),
            "--steps" => c.steps = val().parse().unwrap(),
            "--k" => c.k = val().parse().unwrap(),
            "--radius" => c.radius = val().parse().unwrap(),
            "--seed" => c.seed = val().parse().unwrap(),
            "--t-opt" => c.t_opt = val().parse().unwrap(),
            "--tol" => c.tol = val().parse().unwrap(),
            "--speed" => c.speed = val().parse().unwrap(),
            "--noise" => c.noise = val().parse().unwrap(),
            "--out-dir" => c.out_dir = val(),
            "--field-grid" => c.field_grid = val().parse().unwrap(),
            "--loss" => c.loss = val().parse().unwrap(),
            "--lat-ms" => c.lat_us = (val().parse::<f64>().unwrap() * 1000.0) as u64,
            "--lat-jitter-ms" => c.lat_jitter_us = (val().parse::<f64>().unwrap() * 1000.0) as u64,
            "--step-ms" => c.step_us = (val().parse::<f64>().unwrap() * 1000.0) as u64,
            "--mode" => {
                c.mode = match val().as_str() {
                    "consensus" => Mode::Consensus,
                    "gossip" => Mode::Gossip,
                    "follow" => Mode::Follow,
                    "solo" => Mode::Solo,
                    m => panic!("--mode: unknown {m}"),
                }
            }
            "--field" => {
                c.field = match val().as_str() {
                    "bumps" => FieldKind::Bumps,
                    "linear" => FieldKind::Linear,
                    "complex" => FieldKind::Complex,
                    f => panic!("--field: unknown {f}"),
                }
            }
            "--start" => {
                // x,y,r (円内に固める) / "scatter" (全域に散布)
                let v = val();
                if v == "scatter" {
                    c.start = (-1.0, -1.0, 0.0);
                    i += 2;
                    continue;
                }
                let p: Vec<f64> = v.split(',').map(|s| s.parse().unwrap()).collect();
                assert_eq!(p.len(), 3, "--start x,y,r");
                c.start = (p[0], p[1], p[2]);
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
    assert!(c.n >= 1 && c.k >= 1);
    assert!(c.step_us >= 4, "--step-ms too small");
    c
}

// ---------- 温度場 (環境。個体には位置の温度しか渡さない) ----------

struct Field {
    kind: FieldKind,
}

impl Field {
    /// 温度 [℃]。定義域は [0,1]^2 (旧 dyne-swarm と同一の式)
    fn temp(&self, x: f64, y: f64) -> f64 {
        match self.kind {
            FieldKind::Linear => 10.0 + 20.0 * x,
            FieldKind::Bumps => {
                let base = 10.0 + 30.0 * x;
                let bump = |cx: f64, cy: f64, a: f64, s: f64| {
                    let d2 = (x - cx).powi(2) + (y - cy).powi(2);
                    a * (-d2 / (2.0 * s * s)).exp()
                };
                base + bump(0.3, 0.75, 9.0, 0.10) + bump(0.75, 0.3, 6.0, 0.12)
            }
            FieldKind::Complex => {
                use std::f64::consts::PI;
                let bump = |cx: f64, cy: f64, a: f64, s: f64| {
                    let d2 = (x - cx).powi(2) + (y - cy).powi(2);
                    a * (-d2 / (2.0 * s * s)).exp()
                };
                12.0 + 4.0 * (3.0 * PI * x).sin() * (2.0 * PI * y).cos()
                    + bump(0.2, 0.8, 16.0, 0.09)
                    + bump(0.8, 0.7, 15.0, 0.07)
                    + bump(0.15, 0.2, 13.0, 0.08)
                    + bump(0.7, 0.2, 9.0, 0.13)
                    + bump(0.45, 0.5, -6.0, 0.15)
            }
        }
    }
}

fn norm(v: (f64, f64)) -> f64 {
    (v.0 * v.0 + v.1 * v.1).sqrt()
}

// ---------- WASMアクターの実行ラッパ (群れアクター差込口) ----------

struct SwarmActor {
    store: wasmi::Store<()>,
    memory: wasmi::Memory,
    init: wasmi::TypedFunc<(u32, f64, f64, f64), i32>,
    cfg: wasmi::TypedFunc<(u32, f64), ()>,
    sense: wasmi::TypedFunc<f64, ()>,
    step: wasmi::TypedFunc<(), ()>,
    get_x: wasmi::TypedFunc<(), f64>,
    get_y: wasmi::TypedFunc<(), f64>,
    inbox_write: wasmi::TypedFunc<(u32, u32), i32>,
    outbox_read: wasmi::TypedFunc<(u32, u32), u32>,
    dropped: wasmi::TypedFunc<(), u64>,
    /// メッセージ交換用スクラッチ領域 (アクター側が場所と容量を告知する)
    buf: u32,
    cap: u32,
}

impl SwarmActor {
    fn load(engine: &wasmi::Engine, module: &wasmi::Module) -> Self {
        let mut store = wasmi::Store::new(engine, ());
        let linker: wasmi::Linker<()> = wasmi::Linker::new(engine);
        let instance = linker
            .instantiate(&mut store, module)
            .expect("instantiate")
            .start(&mut store)
            .expect("start");
        let memory = instance.get_memory(&store, "memory").expect("memory");
        macro_rules! func {
            ($name:literal, $p:ty, $r:ty) => {
                instance.get_typed_func::<$p, $r>(&store, $name).expect($name)
            };
        }
        let msg_buf = func!("dyne_msg_buf", (), u32);
        let msg_cap = func!("dyne_msg_cap", (), u32);
        let mut a = SwarmActor {
            init: func!("swarm_init", (u32, f64, f64, f64), i32),
            cfg: func!("dyne_cfg", (u32, f64), ()),
            sense: func!("swarm_sense", f64, ()),
            step: func!("swarm_step", (), ()),
            get_x: func!("swarm_x", (), f64),
            get_y: func!("swarm_y", (), f64),
            inbox_write: func!("dyne_inbox_write", (u32, u32), i32),
            outbox_read: func!("dyne_outbox_read", (u32, u32), u32),
            dropped: func!("dyne_dropped", (), u64),
            memory,
            store,
            buf: 0,
            cap: 0,
        };
        a.buf = msg_buf.call(&mut a.store, ()).unwrap();
        a.cap = msg_cap.call(&mut a.store, ()).unwrap();
        a
    }

    fn pos(&mut self) -> (f64, f64) {
        let x = self.get_x.call(&mut self.store, ()).unwrap();
        let y = self.get_y.call(&mut self.store, ()).unwrap();
        (x, y)
    }

    /// 知覚位相: 真の温度を渡し、送信箱 (メッセージ束) を読み出す
    fn sense_and_collect(&mut self, temp: f64) -> Vec<u8> {
        self.sense.call(&mut self.store, temp).unwrap();
        let n = self.outbox_read.call(&mut self.store, (self.buf, self.cap)).unwrap() as usize;
        let mut bundle = vec![0u8; n];
        if n > 0 {
            self.memory.read(&self.store, self.buf as usize, &mut bundle).unwrap();
        }
        bundle
    }

    /// 配送: メッセージ束を受信箱に書き込む (0=全量, 1=あふれで一部, -1=不正)
    fn deliver(&mut self, bundle: &[u8]) -> i32 {
        assert!(bundle.len() <= self.cap as usize, "bundle exceeds scratch cap");
        self.memory.write(&mut self.store, self.buf as usize, bundle).unwrap();
        self.inbox_write.call(&mut self.store, (self.buf, bundle.len() as u32)).unwrap()
    }
}

// ---------- 離散イベント ----------

enum Ev {
    /// 知覚位相 (全個体が知覚し、メッセージ束を発送する)
    Sense,
    /// ステップ s の判断位相 (全個体が受信箱を消費して動く)
    Decide(u64),
    /// メッセージ束の到着
    Deliver { dst: usize, bundle: Vec<u8> },
}

// ---------- メイン ----------

fn main() {
    let cfg = parse_args();
    std::fs::create_dir_all(&cfg.out_dir).expect("create out dir");
    let field = Field { kind: cfg.field };

    // WASMアクターを個体数ぶんインスタンス化
    let wasm = std::fs::read(&cfg.wasm).expect("read wasm");
    let engine = wasmi::Engine::default();
    let module = wasmi::Module::new(&engine, &wasm[..]).expect("compile wasm");
    let mut actors: Vec<SwarmActor> = (0..cfg.n).map(|_| SwarmActor::load(&engine, &module)).collect();

    // 初期配置 (旧 dyne-swarm と同じ手順。乱数は dyne-core の系列)
    let mut place_rng = Rng::new(cfg.seed);
    let mut pos: Vec<(f64, f64)> = Vec::with_capacity(cfg.n);
    for _ in 0..cfg.n {
        let (sx, sy, sr) = cfg.start;
        let (x, y) = if sx < 0.0 {
            (place_rng.uniform(), place_rng.uniform())
        } else {
            let r = sr * place_rng.uniform().sqrt();
            let th = 2.0 * std::f64::consts::PI * place_rng.uniform();
            ((sx + r * th.cos()).clamp(0.0, 1.0), (sy + r * th.sin()).clamp(0.0, 1.0))
        };
        pos.push((x, y));
    }

    // アクター初期化と個体規則の注入 (シードは LO→HI の順。HI で内蔵乱数が初期化される)
    let mode_val = match cfg.mode {
        Mode::Solo => MODE_SOLO,
        Mode::Follow => MODE_FOLLOW,
        Mode::Consensus => MODE_CONSENSUS,
        Mode::Gossip => MODE_GOSSIP,
    } as f64;
    for (i, a) in actors.iter_mut().enumerate() {
        let (x, y) = pos[i];
        let t0 = field.temp(x, y);
        assert_eq!(a.init.call(&mut a.store, (i as u32, x, y, t0)).unwrap(), 0, "swarm_init");
        let seed_i = derive_seed(cfg.seed, i as u64);
        for (k, v) in [
            (CFG_MODE, mode_val),
            (CFG_T_OPT, cfg.t_opt),
            (CFG_TOL, cfg.tol),
            (CFG_SPEED, cfg.speed),
            (CFG_NOISE, cfg.noise),
            (CFG_RADIUS, cfg.radius),
            (CFG_GOSSIP_MAX_AGE, 400.0),
            (CFG_SEED_LO, (seed_i & 0xFFFF_FFFF) as f64),
            (CFG_SEED_HI, (seed_i >> 32) as f64),
        ] {
            a.cfg.call(&mut a.store, (k, v)).unwrap();
        }
    }

    // 仮想ネットワーク (シードは配置・個体と別系統に分離)
    let mut net = VirtNet::new(
        cfg.loss,
        cfg.lat_us,
        cfg.lat_jitter_us,
        Vec::new(),
        derive_seed(cfg.seed, u64::MAX),
    );

    // 個体 i の通信相手: 半径内の近い順 k 個体 (旧実装と同一の決定則)
    let neighbors = |pos: &[(f64, f64)], i: usize| -> Vec<usize> {
        let (ax, ay) = pos[i];
        let mut cand: Vec<(f64, usize)> = pos
            .iter()
            .enumerate()
            .filter(|&(j, &(bx, by))| {
                j != i && (bx - ax).powi(2) + (by - ay).powi(2) <= cfg.radius.powi(2)
            })
            .map(|(j, &(bx, by))| ((bx - ax).powi(2) + (by - ay).powi(2), j))
            .collect();
        cand.sort_by(|p, q| p.0.partial_cmp(&q.0).unwrap().then(p.1.cmp(&q.1)));
        cand.truncate(cfg.k);
        cand.into_iter().map(|(_, j)| j).collect()
    };

    // 指標: 最適域内の割合、平均 |T−T_opt| (真の温度で評価)、重心からの平均距離
    let metrics = |pos: &[(f64, f64)]| -> (f64, f64, f64) {
        let n = pos.len() as f64;
        let (mut ok, mut dev, mut cx, mut cy) = (0.0, 0.0, 0.0, 0.0);
        for &(x, y) in pos {
            let d = (field.temp(x, y) - cfg.t_opt).abs();
            if d < cfg.tol {
                ok += 1.0;
            }
            dev += d;
            cx += x;
            cy += y;
        }
        cx /= n;
        cy /= n;
        let spread = pos.iter().map(|&(x, y)| norm((x - cx, y - cy))).sum::<f64>() / n;
        (ok / n, dev / n, spread)
    };

    // ステップ s: 知覚 = (s-1)·D、判断 = (s-1)·D + 3D/4 (同時刻の配送より先に予約 → 先に処理)
    let d_us = cfg.step_us;
    let mut queue: EventQueue<Ev> = EventQueue::new();
    for s in 1..=cfg.steps {
        queue.schedule((s - 1) * d_us, Ev::Sense);
        queue.schedule((s - 1) * d_us + 3 * d_us / 4, Ev::Decide(s));
    }

    let mut traj = String::from("step,id,x,y,temp,n_nbr\n");
    let mut met = String::from("step,frac_ok,mean_dev,spread\n");
    let mut first_90: Option<u64> = None;
    let mut first_50: Option<u64> = None;
    let mut pending_nbr = vec![0usize; cfg.n];
    let mut bundles_sent: u64 = 0;
    let mut bundles_lost: u64 = 0;

    let mut log_step = |s: u64, pos: &[(f64, f64)], n_nbr: &[usize], traj: &mut String, met: &mut String| {
        for (i, &(x, y)) in pos.iter().enumerate() {
            let t = field.temp(x, y);
            let _ = writeln!(traj, "{},{},{:e},{:e},{:e},{}", s, i, x, y, t, n_nbr[i]);
        }
        let (ok, dev, spread) = metrics(pos);
        let _ = writeln!(met, "{},{:e},{:e},{:e}", s, ok, dev, spread);
        if ok >= 0.5 && first_50.is_none() {
            first_50 = Some(s);
        }
        if ok >= 0.9 && first_90.is_none() {
            first_90 = Some(s);
        }
        if !cfg.quiet && s % 100 == 0 {
            eprintln!("[swarm] step {s}: in-band {:.0}% mean|dT|={dev:.2} spread={spread:.3}", ok * 100.0);
        }
    };

    // s=0 (初期状態) の記録
    log_step(0, &pos, &vec![0usize; cfg.n], &mut traj, &mut met);

    while let Some((now, ev)) = queue.pop() {
        match ev {
            Ev::Sense => {
                // 1. 全個体が知覚してメッセージ束を作る (位置はこの位相では動かない)
                let bundles: Vec<Vec<u8>> = actors
                    .iter_mut()
                    .enumerate()
                    .map(|(i, a)| {
                        let (x, y) = pos[i];
                        a.sense_and_collect(field.temp(x, y))
                    })
                    .collect();
                // 2. 発送: 受信者 i ごとに近傍 j の束を仮想ネットワークへ (solo でも
                //    電波は飛ぶ。個体規則が無視するだけ — 旧実装の n_nbr 記録と同じ)
                for i in 0..cfg.n {
                    for j in neighbors(&pos, i) {
                        bundles_sent += 1;
                        match net.transit(j, i, now) {
                            Some(t) => queue.schedule(t, Ev::Deliver { dst: i, bundle: bundles[j].clone() }),
                            None => bundles_lost += 1,
                        }
                    }
                }
            }
            Ev::Deliver { dst, bundle } => {
                // 戻り値: 0=全量 1=あふれで一部 (アクター側が dropped に計上) -1=不正
                let rc = actors[dst].deliver(&bundle);
                assert!(rc >= 0, "malformed bundle delivered to {dst}");
                pending_nbr[dst] += 1;
            }
            Ev::Decide(s) => {
                for (i, a) in actors.iter_mut().enumerate() {
                    a.step.call(&mut a.store, ()).unwrap();
                    pos[i] = a.pos();
                }
                let n_nbr = pending_nbr.clone();
                pending_nbr.iter_mut().for_each(|c| *c = 0);
                log_step(s, &pos, &n_nbr, &mut traj, &mut met);
            }
        }
    }

    // 温度場の格子標本 (描画用。個体はこれを見ていない)
    let g = cfg.field_grid;
    let mut fld = String::from("i,j,x,y,temp\n");
    for j in 0..=g {
        for i in 0..=g {
            let (x, y) = (i as f64 / g as f64, j as f64 / g as f64);
            let _ = writeln!(fld, "{},{},{:e},{:e},{:e}", i, j, x, y, field.temp(x, y));
        }
    }

    let actor_dropped: u64 = actors
        .iter_mut()
        .map(|a| a.dropped.call(&mut a.store, ()).unwrap())
        .sum();
    let (ok, dev, spread) = metrics(&pos);
    let summary = format!(
        "mode={:?} field={:?} n={} k={} radius={} steps={} seed={} t_opt={} tol={} speed={} noise={}\n\
         final: in-band={:.1}% mean|dT|={:.3} spread={:.4}\n\
         first step with >=50% in-band: {}\nfirst step with >=90% in-band: {}\n\
         net: step_ms={} loss={} lat_ms={} lat_jitter_ms={} bundles_sent={} bundles_lost={} actor_dropped={}\n",
        cfg.mode, cfg.field, cfg.n, cfg.k, cfg.radius, cfg.steps, cfg.seed, cfg.t_opt, cfg.tol, cfg.speed, cfg.noise,
        ok * 100.0, dev, spread,
        first_50.map(|s| s.to_string()).unwrap_or_else(|| "never".into()),
        first_90.map(|s| s.to_string()).unwrap_or_else(|| "never".into()),
        cfg.step_us as f64 / 1000.0, cfg.loss, cfg.lat_us as f64 / 1000.0,
        cfg.lat_jitter_us as f64 / 1000.0, bundles_sent, bundles_lost, actor_dropped,
    );
    let d = &cfg.out_dir;
    std::fs::write(format!("{d}/traj.csv"), traj).unwrap();
    std::fs::write(format!("{d}/metrics.csv"), met).unwrap();
    std::fs::write(format!("{d}/field.csv"), fld).unwrap();
    std::fs::write(format!("{d}/summary.txt"), &summary).unwrap();
    print!("{summary}");
}
