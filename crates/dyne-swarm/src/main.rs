//! dyne-swarm: 生物の群れの再現 (タスク4b、2026-08-29 発注2)
//!
//! 2次元の温度場 (勾配つき、シミュレータが大域計算してよい) に N 個体を置く。
//! 個体が使ってよい情報は次の3つだけ:
//!   1. 自分の位置の温度 (局所知覚)
//!   2. 通信範囲 radius 内の近傍 k 個体から受け取ったメッセージ (位置・温度・勾配推定)
//!   3. 自分の過去 (前ステップの位置と温度)
//! 大域情報 (場の式、最適位置、他個体の一覧) は個体からは参照できない。
//! 目的: 非最適な場所に置かれた群れが、局所勾配追従と隣との合意だけで最適温度域
//! (|T − T_opt| < tol) へ集団移動できるかを見る。
//!
//! 個体の規則 (--mode):
//!   consensus … 自分と近傍の (位置, 温度) 標本から最小二乗で局所勾配を推定し、近傍の
//!               勾配推定と平均して合意する (既定)。温度が低すぎれば勾配方向、高すぎれば
//!               逆方向へ進む。凝集 (離れすぎない) と分離 (近すぎない) を弱く加える。
//!   gossip    … consensus に加え、「自分が知る最良地点 (位置, |T−T_opt|, 経過ステップ)」を
//!               隣へ伝え、より良い地点を聞いたら乗り換える (多段の伝聞。各段は隣とだけ)。
//!               局所最適の罠から抜けるのに使う。
//!   follow    … 温度だけ共有し、|T − T_opt| が最小の隣へ寄る最小構成。
//!   solo      … 通信なし。自分の前ステップとの温度差から進行方向の良し悪しだけを判断
//!               (比較用の対照群)。
//!
//! 乱数はシード固定 (xorshift64*)、演算順序固定なので同一入力で全出力がビット一致する。
//!
//! 出力 (--out-dir): traj.csv (step,id,x,y,temp,n_nbr) / metrics.csv (step,frac_ok,mean_dev,spread)
//!                  / field.csv (温度場の格子標本。描画用) / summary.txt

use std::fmt::Write as _;

// ---------- 設定 ----------

#[derive(Clone, Debug)]
struct Config {
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
    /// 左から右へ温度が上がる線形勾配 + 2つの温かい丘 (局所的な罠) 。最適域は丘の斜面の帯
    Bumps,
    /// 単純な線形勾配 (最適域は縦の帯)
    Linear,
    /// 入り組んだ場: うねる基調 + 温かい丘3つ + 冷たい窪み。最適域は互いに離れた2〜3個の
    /// スポット (丘Aの輪、丘Bの小さな島) と、届かない局所最適 (丘C: 最も近いが帯に入らない罠)
    Complex,
}

fn parse_args() -> Config {
    let mut c = Config {
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
    };
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        let key = args[i].as_str();
        let val = || args.get(i + 1).cloned().unwrap_or_default();
        match key {
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
    assert!(c.n >= 1 && c.k >= 1);
    c
}

// ---------- 決定論的乱数 (xorshift64*) ----------

struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) | 1)
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
    /// 一様乱数2つから標準正規 (Box–Muller)
    fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-300);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

// ---------- 温度場 (環境。個体からは位置の温度しか見えない) ----------

struct Field {
    kind: FieldKind,
}

impl Field {
    /// 温度 [℃]。定義域は [0,1]^2
    fn temp(&self, x: f64, y: f64) -> f64 {
        match self.kind {
            FieldKind::Linear => 10.0 + 20.0 * x,
            FieldKind::Bumps => {
                // 左が冷たく右が温かい線形勾配 (10→40℃) に、2つの温かい丘を重ねる。
                // 最適 24℃ の帯は x≈0.5 付近の縦帯と丘の周辺の輪。
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
                // うねる基調 (8〜16℃) + 丘A (最適24℃を越える → 輪状の最適域)
                // + 丘B (小さな島、24℃を越える) + 丘D (左下、同様) + 丘C (22℃止まりの罠: 局所最適だが帯に入らない)
                // + 冷たい窪み (群れを分断する谷)
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

// ---------- 個体 ----------

#[derive(Clone, Debug)]
struct Agent {
    x: f64,
    y: f64,
    /// 現在位置で知覚した温度 (センサ雑音込み)
    t: f64,
    /// 前ステップの位置と温度 (solo 用の自己履歴)
    prev: (f64, f64, f64),
    /// 自分の勾配推定 (通信で隣に渡す)
    grad: (f64, f64),
    /// 直前の移動方向 (慣性・solo の方向維持用)
    dir: (f64, f64),
    n_nbr: usize,
    /// gossip: 知っている最良地点 (x, y, |T−T_opt|, 経過ステップ)
    best: Option<(f64, f64, f64, u32)>,
}

/// 個体が隣へ送るメッセージ (これ以外の情報は流れない)
#[derive(Clone, Copy, Debug)]
struct Msg {
    x: f64,
    y: f64,
    t: f64,
    grad: (f64, f64),
    best: Option<(f64, f64, f64, u32)>,
}

/// (位置, 温度) 標本群から温度の平面 T ≈ a + gx·x + gy·y を最小二乗で当てはめ、(gx, gy) を返す。
/// 標本が3未満、または退化 (一直線上) なら None。
fn fit_gradient(samples: &[(f64, f64, f64)]) -> Option<(f64, f64)> {
    let n = samples.len();
    if n < 3 {
        return None;
    }
    let nf = n as f64;
    let (mut mx, mut my, mut mt) = (0.0, 0.0, 0.0);
    for &(x, y, t) in samples {
        mx += x;
        my += y;
        mt += t;
    }
    mx /= nf;
    my /= nf;
    mt /= nf;
    let (mut sxx, mut sxy, mut syy, mut sxt, mut syt) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for &(x, y, t) in samples {
        let (dx, dy, dt) = (x - mx, y - my, t - mt);
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
        sxt += dx * dt;
        syt += dy * dt;
    }
    let det = sxx * syy - sxy * sxy;
    if det.abs() < 1e-12 {
        return None;
    }
    let gx = (sxt * syy - syt * sxy) / det;
    let gy = (syt * sxx - sxt * sxy) / det;
    Some((gx, gy))
}

fn norm(v: (f64, f64)) -> f64 {
    (v.0 * v.0 + v.1 * v.1).sqrt()
}

fn unit(v: (f64, f64)) -> (f64, f64) {
    let n = norm(v);
    if n < 1e-12 {
        (0.0, 0.0)
    } else {
        (v.0 / n, v.1 / n)
    }
}

// ---------- シミュレーション ----------

struct Sim {
    cfg: Config,
    field: Field,
    rng: Rng,
    agents: Vec<Agent>,
}

impl Sim {
    fn new(cfg: Config) -> Self {
        let mut rng = Rng::new(cfg.seed);
        let field = Field { kind: cfg.field };
        let mut agents = Vec::new();
        for _ in 0..cfg.n {
            // 初期配置: 既定は全域に一様ランダム散布。--start x,y,r なら円内に固める
            let (sx, sy, sr) = cfg.start;
            let (x, y) = if sx < 0.0 {
                (rng.uniform(), rng.uniform())
            } else {
                let r = sr * rng.uniform().sqrt();
                let th = 2.0 * std::f64::consts::PI * rng.uniform();
                ((sx + r * th.cos()).clamp(0.0, 1.0), (sy + r * th.sin()).clamp(0.0, 1.0))
            };
            let t = field.temp(x, y);
            agents.push(Agent {
                x,
                y,
                t,
                prev: (x, y, t),
                grad: (0.0, 0.0),
                dir: (0.0, 0.0),
                n_nbr: 0,
                best: None,
            });
        }
        Sim { cfg, field, rng, agents }
    }

    /// 個体 i の通信相手: 半径内の近い順 k 個体 (環境=無線の到達範囲が決める。個体は選べない)
    fn neighbors(&self, i: usize) -> Vec<usize> {
        let a = &self.agents[i];
        let mut cand: Vec<(f64, usize)> = self
            .agents
            .iter()
            .enumerate()
            .filter(|&(j, b)| j != i && (b.x - a.x).powi(2) + (b.y - a.y).powi(2) <= self.cfg.radius.powi(2))
            .map(|(j, b)| ((b.x - a.x).powi(2) + (b.y - a.y).powi(2), j))
            .collect();
        // 距離→添字の順で安定に並べる (決定論のため同距離の順序も固定)
        cand.sort_by(|p, q| p.0.partial_cmp(&q.0).unwrap().then(p.1.cmp(&q.1)));
        cand.truncate(self.cfg.k);
        cand.into_iter().map(|(_, j)| j).collect()
    }

    fn step(&mut self) {
        let cfg = self.cfg.clone();
        // 1. 知覚: 各個体は自分の位置の温度を測る (センサ雑音は決定論的乱数)
        for a in self.agents.iter_mut() {
            a.t = self.field.temp(a.x, a.y) + cfg.noise * self.rng.normal();
        }
        // 2. 通信: 近傍からメッセージを受け取る (前ステップの勾配推定を含む)
        let n = self.agents.len();
        let inbox: Vec<Vec<Msg>> = (0..n)
            .map(|i| {
                self.neighbors(i)
                    .into_iter()
                    .map(|j| {
                        let b = &self.agents[j];
                        Msg { x: b.x, y: b.y, t: b.t, grad: b.grad, best: b.best }
                    })
                    .collect()
            })
            .collect();
        // 3. 判断と移動 (各個体は自分の状態と受信箱だけを見る)
        let mut moves: Vec<((f64, f64), (f64, f64), usize, (f64, f64), Option<(f64, f64, f64, u32)>)> =
            Vec::with_capacity(n);
        const GOSSIP_MAX_AGE: u32 = 400;
        for i in 0..n {
            let a = &self.agents[i];
            let msgs = &inbox[i];
            let dev = a.t - cfg.t_opt; // 正=暑すぎる、負=寒すぎる
            let want = -dev.signum(); // 進みたい向き: 寒ければ勾配方向 (+1)、暑ければ逆 (−1)
            // gossip: 最良地点の知識を更新 (自分の実測が最優先。隣の伝聞はより良ければ採用)
            let mut best = a.best.and_then(|(x, y, d, age)| if age + 1 > GOSSIP_MAX_AGE { None } else { Some((x, y, d, age + 1)) });
            if cfg.mode == Mode::Gossip {
                if dev.abs() < cfg.tol {
                    best = Some((a.x, a.y, dev.abs(), 0));
                }
                for m in msgs {
                    if let Some((bx, by, bd, bage)) = m.best {
                        let better = match best {
                            None => true,
                            Some((_, _, d, _)) => bd + 0.2 < d,
                        };
                        if better && bage + 1 <= GOSSIP_MAX_AGE {
                            best = Some((bx, by, bd, bage + 1));
                        }
                    }
                }
            }
            let (mut dir, grad_est) = match cfg.mode {
                Mode::Gossip if dev.abs() >= cfg.tol
                    && best.map_or(false, |(bx, by, bd, _)| bd < cfg.tol && norm((bx - a.x, by - a.y)) > 0.02) =>
                {
                    // 伝聞で知った最適域の地点へ向かう (勾配より優先)
                    let (bx, by, _, _) = best.unwrap();
                    (unit((bx - a.x, by - a.y)), (0.0, 0.0))
                }
                Mode::Consensus | Mode::Gossip => {
                    // 自分+近傍の標本で勾配を当てはめ、近傍の推定と平均して合意する
                    let mut samples = vec![(a.x, a.y, a.t)];
                    samples.extend(msgs.iter().map(|m| (m.x, m.y, m.t)));
                    let own = fit_gradient(&samples);
                    let mut acc = own.unwrap_or((0.0, 0.0));
                    let mut cnt = if own.is_some() { 1.0 } else { 0.0 };
                    for m in msgs {
                        if norm(m.grad) > 0.0 {
                            acc.0 += m.grad.0;
                            acc.1 += m.grad.1;
                            cnt += 1.0;
                        }
                    }
                    let g = if cnt > 0.0 { (acc.0 / cnt, acc.1 / cnt) } else { (0.0, 0.0) };
                    if norm(g) > 1e-9 {
                        let u = unit(g);
                        ((u.0 * want, u.1 * want), g)
                    } else {
                        // 勾配が分からない: 隣がいなければ探索 (乱歩)
                        let th = 2.0 * std::f64::consts::PI * self.rng.uniform();
                        ((th.cos(), th.sin()), (0.0, 0.0))
                    }
                }
                Mode::Follow => {
                    // 温度だけ共有: 自分より最適に近い隣のうち最も良い個体へ寄る
                    let best = msgs
                        .iter()
                        .filter(|m| (m.t - cfg.t_opt).abs() < dev.abs())
                        .min_by(|p, q| (p.t - cfg.t_opt).abs().partial_cmp(&(q.t - cfg.t_opt).abs()).unwrap());
                    match best {
                        Some(m) => (unit((m.x - a.x, m.y - a.y)), (0.0, 0.0)),
                        None => {
                            let th = 2.0 * std::f64::consts::PI * self.rng.uniform();
                            ((th.cos(), th.sin()), (0.0, 0.0))
                        }
                    }
                }
                Mode::Solo => {
                    // 通信なし: 前の一歩で |T−T_opt| が縮んだなら同じ向き、広がったなら向きを変える
                    let prev_dev = (a.prev.2 - cfg.t_opt).abs();
                    if norm(a.dir) > 0.0 && dev.abs() <= prev_dev {
                        (a.dir, (0.0, 0.0))
                    } else {
                        let th = 2.0 * std::f64::consts::PI * self.rng.uniform();
                        ((th.cos(), th.sin()), (0.0, 0.0))
                    }
                }
            };
            // 群れとしてのまとまり (通信ありのモードのみ): 遠い隣へ弱く寄り、近すぎる隣から離れる
            if cfg.mode != Mode::Solo && !msgs.is_empty() {
                let (mut cx, mut cy) = (0.0, 0.0);
                let (mut sx, mut sy) = (0.0, 0.0);
                for m in msgs {
                    cx += m.x - a.x;
                    cy += m.y - a.y;
                    let d = norm((m.x - a.x, m.y - a.y));
                    if d < 0.25 * cfg.radius && d > 1e-12 {
                        sx -= (m.x - a.x) / d;
                        sy -= (m.y - a.y) / d;
                    }
                }
                let mf = msgs.len() as f64;
                let coh = unit((cx / mf, cy / mf));
                let sep = unit((sx, sy));
                dir = (dir.0 + 0.3 * coh.0 + 0.6 * sep.0, dir.1 + 0.3 * coh.1 + 0.6 * sep.1);
                dir = unit(dir);
            }
            // 最適域内では歩幅を落として留まる (|dev| に比例、最大 speed)
            let gain = (dev.abs() / (2.0 * cfg.tol)).min(1.0);
            let sp = cfg.speed * (0.1 + 0.9 * gain);
            let mut nx = (a.x + sp * dir.0).clamp(0.0, 1.0);
            let mut ny = (a.y + sp * dir.1).clamp(0.0, 1.0);
            // 壁にぶつかって進めなかったら向きを変える (境界で張り付かない)
            if norm((nx - a.x, ny - a.y)) < 0.25 * sp && dev.abs() >= cfg.tol {
                let th = 2.0 * std::f64::consts::PI * self.rng.uniform();
                dir = (th.cos(), th.sin());
                nx = (a.x + sp * dir.0).clamp(0.0, 1.0);
                ny = (a.y + sp * dir.1).clamp(0.0, 1.0);
            }
            moves.push(((nx, ny), grad_est, msgs.len(), dir, best));
        }
        // 4. 同時更新
        for (i, ((nx, ny), g, nn, dir, best)) in moves.into_iter().enumerate() {
            let a = &mut self.agents[i];
            a.prev = (a.x, a.y, a.t);
            a.x = nx;
            a.y = ny;
            a.grad = g;
            a.n_nbr = nn;
            a.dir = dir;
            a.best = best;
        }
    }

    /// 指標: 最適域内の割合、平均 |T−T_opt| (真の温度で評価)、群れの広がり (重心からの平均距離)
    fn metrics(&self) -> (f64, f64, f64) {
        let n = self.agents.len() as f64;
        let (mut ok, mut dev, mut cx, mut cy) = (0.0, 0.0, 0.0, 0.0);
        for a in &self.agents {
            let t = self.field.temp(a.x, a.y);
            let d = (t - self.cfg.t_opt).abs();
            if d < self.cfg.tol {
                ok += 1.0;
            }
            dev += d;
            cx += a.x;
            cy += a.y;
        }
        cx /= n;
        cy /= n;
        let spread = self.agents.iter().map(|a| norm((a.x - cx, a.y - cy))).sum::<f64>() / n;
        (ok / n, dev / n, spread)
    }
}

fn main() {
    let cfg = parse_args();
    std::fs::create_dir_all(&cfg.out_dir).expect("create out dir");
    let mut sim = Sim::new(cfg.clone());

    let mut traj = String::from("step,id,x,y,temp,n_nbr\n");
    let mut met = String::from("step,frac_ok,mean_dev,spread\n");
    let mut first_90: Option<u64> = None;
    let mut first_50: Option<u64> = None;
    for s in 0..=cfg.steps {
        if s > 0 {
            sim.step();
        }
        for (i, a) in sim.agents.iter().enumerate() {
            let t = sim.field.temp(a.x, a.y);
            let _ = writeln!(traj, "{},{},{:e},{:e},{:e},{}", s, i, a.x, a.y, t, a.n_nbr);
        }
        let (ok, dev, spread) = sim.metrics();
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
    }
    // 温度場の格子標本 (描画用。個体はこれを見ていない)
    let g = cfg.field_grid;
    let mut fld = String::from("i,j,x,y,temp\n");
    for j in 0..=g {
        for i in 0..=g {
            let (x, y) = (i as f64 / g as f64, j as f64 / g as f64);
            let _ = writeln!(fld, "{},{},{:e},{:e},{:e}", i, j, x, y, sim.field.temp(x, y));
        }
    }
    let (ok, dev, spread) = sim.metrics();
    let summary = format!(
        "mode={:?} field={:?} n={} k={} radius={} steps={} seed={} t_opt={} tol={} speed={} noise={}\n\
         final: in-band={:.1}% mean|dT|={:.3} spread={:.4}\n\
         first step with >=50% in-band: {}\nfirst step with >=90% in-band: {}\n",
        cfg.mode, cfg.field, cfg.n, cfg.k, cfg.radius, cfg.steps, cfg.seed, cfg.t_opt, cfg.tol, cfg.speed, cfg.noise,
        ok * 100.0, dev, spread,
        first_50.map(|s| s.to_string()).unwrap_or_else(|| "never".into()),
        first_90.map(|s| s.to_string()).unwrap_or_else(|| "never".into()),
    );
    let d = &cfg.out_dir;
    std::fs::write(format!("{d}/traj.csv"), traj).unwrap();
    std::fs::write(format!("{d}/metrics.csv"), met).unwrap();
    std::fs::write(format!("{d}/field.csv"), fld).unwrap();
    std::fs::write(format!("{d}/summary.txt"), &summary).unwrap();
    print!("{summary}");
}
