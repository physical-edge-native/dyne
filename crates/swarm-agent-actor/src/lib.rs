//! swarm-agent-actor: 群れの個体1体ぶんのWASMアクター (発注6 第2段、2026-09-04)
//!
//! crates/dyne-swarm (ネイティブ単体の参照実装、凍結) の個体規則を、Dyneの土台
//! (WASMアクター+ワイヤ形式v2+仮想ネットワーク) に載せ替えたもの。個体が使って
//! よい情報は参照実装と同じ3つだけ:
//!   1. 自分の位置の温度 (ホストが swarm_sense で注入。センサ雑音は個体が内蔵乱数で加える)
//!   2. 受信箱に届いた近傍からのメッセージ (ワイヤ形式v2の POSITION / HEARSAY)
//!   3. 自分の過去 (前ステップの位置と温度)
//! 場の式・最適位置・他個体の一覧は個体からは参照できない。
//!
//! ## 乱数 (2026-09-02 ユーザー決定②)
//! アクター内蔵の xorshift64* (整数演算のみ、dyne-core::rng と同一アルゴリズム)。
//! 個体シードはホストが derive_seed(大域シード, 個体ID) で導出し、dyne_cfg
//! (CFG_SEED_LO / CFG_SEED_HI の2キー、各32ビット) で注入する。
//! ネイティブテストで dyne-core::rng との列一致を照合している (二重実装のドリフト防止)。
//!
//! ## 差込口 (2026-09-02 設計書 §5)
//!   dyne_inbox_write(ptr, len) -> i32 … ホスト→アクター。v2メッセージ列を受信箱へ。
//!       0=全て受理 / 1=一部あふれ (破棄+計数) / -1=形式不正
//!   dyne_outbox_read(ptr, cap) -> u32 … アクター→ホスト。送信箱の内容を書き出す
//!   dyne_msg_buf() / dyne_msg_cap() … 転送用スクラッチ領域 (ホストはここへ書く/ここから読む)
//!   dyne_dropped() -> u64 … あふれで破棄したメッセージの累計
//!
//! ## 1ステップの流れ (ホスト側 dyne-swarm-sim が駆動)
//!   swarm_sense(真の温度) … 知覚+送信箱の作成 (位置・知覚温度・前ステップの勾配推定、
//!                             gossip なら知っている最良地点も)
//!   (ホストが送信箱を近傍へ配送し、届いた分を dyne_inbox_write する)
//!   swarm_step() … 受信箱を読んで判断・移動 (参照実装 crates/dyne-swarm の規則と同一)
//!
//! no_std / 静的確保 / libm (純Rust) のみ依存。同一入力で全出力がビット一致する。

#![cfg_attr(target_arch = "wasm32", no_std)]

use dyne_wire as wire;

// ---------- 定数 ----------

/// dyne_cfg のキー (CFG_*) と個体規則の番号 (MODE_*) はホストとの共有定義
/// (dyne-wire::abi、二重定義の禁止)
pub use dyne_wire::abi::*;

const INBOX_CAP: usize = 16384;
const OUTBOX_CAP: usize = 256;
const SCRATCH_CAP: usize = 4096;
/// 1ステップに処理する近傍メッセージの上限 (超過は破棄+計数)
const MAX_MSGS: usize = 64;

const PI: f64 = core::f64::consts::PI;

// ---------- 内蔵乱数 (xorshift64*。dyne-core::rng と同一アルゴリズム) ----------

pub struct Rng(u64);

impl Rng {
    /// dyne-core の Rng::new(seed) と同じ: seed=0 は 1 に置き換える
    pub fn reseed(&mut self, seed: u64) {
        self.0 = if seed == 0 { 1 } else { seed };
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    /// [0,1) の一様乱数
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// 標準正規 (Box–Muller)。libm で純Rust計算 (プラットフォーム非依存)
    pub fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-300);
        let u2 = self.uniform();
        libm::sqrt(-2.0 * libm::log(u1)) * libm::cos(2.0 * PI * u2)
    }
}

// ---------- 幾何ヘルパ ----------

fn fabs(x: f64) -> f64 {
    libm::fabs(x)
}

fn norm(v: (f64, f64)) -> f64 {
    libm::sqrt(v.0 * v.0 + v.1 * v.1)
}

fn unit(v: (f64, f64)) -> (f64, f64) {
    let n = norm(v);
    if n < 1e-12 {
        (0.0, 0.0)
    } else {
        (v.0 / n, v.1 / n)
    }
}

// ---------- 受信メッセージ (POSITION + 同一送信元の直後の HEARSAY を1件に束ねる) ----------

#[derive(Clone, Copy)]
struct Msg {
    src: u32,
    x: f64,
    y: f64,
    t: f64,
    gx: f64,
    gy: f64,
    has_best: bool,
    bx: f64,
    by: f64,
    bd: f64,
    bage: u32,
}

const EMPTY_MSG: Msg = Msg {
    src: 0,
    x: 0.0,
    y: 0.0,
    t: 0.0,
    gx: 0.0,
    gy: 0.0,
    has_best: false,
    bx: 0.0,
    by: 0.0,
    bd: 0.0,
    bage: 0,
};

// ---------- 個体の状態 (静的確保) ----------

pub struct Agent {
    id: u32,
    step: u64,
    x: f64,
    y: f64,
    /// 知覚した温度 (センサ雑音込み)
    t: f64,
    /// 前ステップの位置と温度 (solo 用の自己履歴)
    prev: (f64, f64, f64),
    /// 自分の勾配推定 (送信箱で隣に渡す)
    grad: (f64, f64),
    /// 直前の移動方向 (solo の方向維持用)
    dir: (f64, f64),
    /// gossip: 知っている最良地点 (x, y, |T−T_opt|, 経過ステップ)
    best: Option<(f64, f64, f64, u32)>,
    rng: Rng,
    seed_lo: u32,
    // パラメータ (dyne_cfg で注入。既定は参照実装の既定値)
    mode: u32,
    t_opt: f64,
    tol: f64,
    speed: f64,
    noise: f64,
    radius: f64,
    gossip_max_age: u32,
    // 受信箱・送信箱 (ワイヤ形式v2の枠でメッセージを詰める) と転送用スクラッチ
    inbox: [u8; INBOX_CAP],
    inbox_used: usize,
    outbox: [u8; OUTBOX_CAP],
    outbox_used: usize,
    scratch: [u8; SCRATCH_CAP],
    /// あふれで破棄したメッセージの累計 (受信箱あふれ+処理上限超過)
    dropped: u64,
}

static mut S: Agent = Agent {
    id: 0,
    step: 0,
    x: 0.0,
    y: 0.0,
    t: 0.0,
    prev: (0.0, 0.0, 0.0),
    grad: (0.0, 0.0),
    dir: (0.0, 0.0),
    best: None,
    rng: Rng(1),
    seed_lo: 0,
    mode: MODE_GOSSIP,
    t_opt: 24.0,
    tol: 1.0,
    speed: 0.004,
    noise: 0.3,
    radius: 0.12,
    gossip_max_age: 400,
    inbox: [0; INBOX_CAP],
    inbox_used: 0,
    outbox: [0; OUTBOX_CAP],
    outbox_used: 0,
    scratch: [0; SCRATCH_CAP],
    dropped: 0,
};

fn state() -> &'static mut Agent {
    unsafe { &mut *core::ptr::addr_of_mut!(S) }
}

// ---------- 内部ロジック (ネイティブテストからも呼ぶ) ----------

impl Agent {
    fn init(&mut self, id: u32, x: f64, y: f64, t0: f64) {
        self.id = id;
        self.step = 0;
        self.x = x;
        self.y = y;
        self.t = t0;
        self.prev = (x, y, t0);
        self.grad = (0.0, 0.0);
        self.dir = (0.0, 0.0);
        self.best = None;
        self.inbox_used = 0;
        self.outbox_used = 0;
        self.dropped = 0;
    }

    fn cfg(&mut self, key: u32, val: f64) {
        match key {
            CFG_MODE => self.mode = val as u32,
            CFG_T_OPT => self.t_opt = val,
            CFG_TOL => self.tol = val,
            CFG_SPEED => self.speed = val,
            CFG_NOISE => self.noise = val,
            CFG_RADIUS => self.radius = val,
            CFG_GOSSIP_MAX_AGE => self.gossip_max_age = val as u32,
            CFG_SEED_LO => self.seed_lo = val as u32,
            CFG_SEED_HI => {
                let seed = ((val as u32 as u64) << 32) | self.seed_lo as u64;
                self.rng.reseed(seed);
            }
            _ => {}
        }
    }

    /// 知覚+送信箱の作成。temp は自位置の真の温度 (ホストが注入)。
    /// センサ雑音は内蔵乱数で個体が加える (毎ステップ必ず2本消費 = 近傍数に依らない)
    fn sense(&mut self, temp: f64) {
        self.step += 1;
        self.t = temp + self.noise * self.rng.normal();
        let mut w = wire::MailboxWriter::new(&mut self.outbox);
        let hdr = wire::Header {
            kind: wire::KIND_POSITION,
            quantity: wire::QTY_TEMPERATURE,
            src: self.id,
            step: self.step,
            t_us: 0,
        };
        let _ = w.push(
            &hdr,
            &wire::Payload::Position { x: self.x, y: self.y, temp: self.t, gx: self.grad.0, gy: self.grad.1 },
        );
        if let Some((bx, by, bd, bage)) = self.best {
            let hh = wire::Header { kind: wire::KIND_HEARSAY, ..hdr };
            let _ = w.push(
                &hh,
                &wire::Payload::Hearsay { x: bx, y: by, score: bd, age_steps: bage as u64 },
            );
        }
        let (used, drop2) = (w.used(), w.dropped);
        self.outbox_used = used;
        self.dropped += drop2;
    }

    /// 受信箱へv2メッセージ列 (枠つき) を追記。あふれたメッセージは破棄+計数。
    /// 戻り値: 0=全て受理 / 1=一部あふれ / -1=形式不正
    fn inbox_write(&mut self, data_len: usize) -> i32 {
        let mut overflow = false;
        let mut pos = 0usize;
        // scratch から読みつつ inbox へ書くため、枠の走査を手で行う
        while pos + 4 <= data_len {
            let len = u32::from_le_bytes(self.scratch[pos..pos + 4].try_into().unwrap()) as usize;
            if len == 0 {
                break;
            }
            if pos + 4 + len > data_len {
                return -1;
            }
            let frame = pos..pos + 4 + len;
            match wire::decode(&self.scratch[pos + 4..pos + 4 + len]) {
                Ok((_, _, consumed)) if consumed == len => {}
                _ => return -1,
            }
            if self.inbox_used + 4 + len <= INBOX_CAP {
                let (dst, src) = (self.inbox_used, frame.start);
                self.inbox[dst..dst + 4 + len].copy_from_slice(&self.scratch[src..src + 4 + len]);
                self.inbox_used = dst + 4 + len;
            } else {
                self.dropped += 1;
                overflow = true;
            }
            pos += 4 + len;
        }
        if overflow {
            1
        } else {
            0
        }
    }

    /// 送信箱の内容をスクラッチへ書き出し、使用バイト数を返す。cap 超過分は破棄+計数
    fn outbox_read(&mut self, cap: usize) -> usize {
        let mut out = 0usize;
        let mut pos = 0usize;
        while pos + 4 <= self.outbox_used {
            let len = u32::from_le_bytes(self.outbox[pos..pos + 4].try_into().unwrap()) as usize;
            if len == 0 {
                break;
            }
            if out + 4 + len <= cap.min(SCRATCH_CAP) {
                self.scratch[out..out + 4 + len].copy_from_slice(&self.outbox[pos..pos + 4 + len]);
                out += 4 + len;
            } else {
                self.dropped += 1;
            }
            pos += 4 + len;
        }
        out
    }

    /// 受信箱を読んで判断・移動 (参照実装 crates/dyne-swarm の step() の個体部分と同一規則)
    fn step(&mut self) {
        // 受信箱 → メッセージ束 (POSITION + 同一送信元の直後の HEARSAY)
        let mut msgs = [EMPTY_MSG; MAX_MSGS];
        let mut n_msgs = 0usize;
        for item in wire::MailboxReader::new(&self.inbox[..self.inbox_used]) {
            let Ok((h, p, _)) = item else { break };
            match p {
                wire::Payload::Position { x, y, temp, gx, gy } => {
                    if n_msgs < MAX_MSGS {
                        msgs[n_msgs] = Msg { src: h.src, x, y, t: temp, gx, gy, ..EMPTY_MSG };
                        n_msgs += 1;
                    } else {
                        self.dropped += 1;
                    }
                }
                wire::Payload::Hearsay { x, y, score, age_steps } => {
                    if n_msgs > 0 && msgs[n_msgs - 1].src == h.src {
                        let m = &mut msgs[n_msgs - 1];
                        m.has_best = true;
                        m.bx = x;
                        m.by = y;
                        m.bd = score;
                        m.bage = age_steps as u32;
                    }
                }
                _ => {}
            }
        }
        self.inbox_used = 0;
        let msgs = &msgs[..n_msgs];

        let dev = self.t - self.t_opt; // 正=暑すぎる、負=寒すぎる
        let want = if dev.is_sign_positive() { -1.0 } else { 1.0 }; // = -dev.signum()

        // gossip: 最良地点の知識を更新 (自分の実測が最優先。隣の伝聞はより良ければ採用)
        let mut best = self.best.and_then(|(x, y, d, age)| {
            if age + 1 > self.gossip_max_age {
                None
            } else {
                Some((x, y, d, age + 1))
            }
        });
        if self.mode == MODE_GOSSIP {
            if fabs(dev) < self.tol {
                best = Some((self.x, self.y, fabs(dev), 0));
            }
            for m in msgs {
                if m.has_best {
                    let better = match best {
                        None => true,
                        Some((_, _, d, _)) => m.bd + 0.2 < d,
                    };
                    if better && m.bage + 1 <= self.gossip_max_age {
                        best = Some((m.bx, m.by, m.bd, m.bage + 1));
                    }
                }
            }
        }

        let goto_best = self.mode == MODE_GOSSIP
            && fabs(dev) >= self.tol
            && best.map_or(false, |(bx, by, bd, _)| {
                bd < self.tol && norm((bx - self.x, by - self.y)) > 0.02
            });
        let (mut dir, grad_est) = if goto_best {
            // 伝聞で知った最適域の地点へ向かう (勾配より優先)
            let (bx, by, _, _) = best.unwrap();
            (unit((bx - self.x, by - self.y)), (0.0, 0.0))
        } else {
            match self.mode {
                MODE_CONSENSUS | MODE_GOSSIP => {
                    // 自分+近傍の標本で勾配を当てはめ、近傍の推定と平均して合意する
                    let own = fit_gradient((self.x, self.y, self.t), msgs);
                    let mut acc = own.unwrap_or((0.0, 0.0));
                    let mut cnt = if own.is_some() { 1.0 } else { 0.0 };
                    for m in msgs {
                        if norm((m.gx, m.gy)) > 0.0 {
                            acc.0 += m.gx;
                            acc.1 += m.gy;
                            cnt += 1.0;
                        }
                    }
                    let g = if cnt > 0.0 { (acc.0 / cnt, acc.1 / cnt) } else { (0.0, 0.0) };
                    if norm(g) > 1e-9 {
                        let u = unit(g);
                        ((u.0 * want, u.1 * want), g)
                    } else {
                        // 勾配が分からない: 隣がいなければ探索 (乱歩)
                        let th = 2.0 * PI * self.rng.uniform();
                        ((libm::cos(th), libm::sin(th)), (0.0, 0.0))
                    }
                }
                MODE_FOLLOW => {
                    // 温度だけ共有: 自分より最適に近い隣のうち最も良い個体へ寄る
                    let mut sel: Option<(f64, f64, f64)> = None; // (x, y, |dT|)
                    for m in msgs {
                        let d = fabs(m.t - self.t_opt);
                        if d < fabs(dev) && sel.map_or(true, |(_, _, sd)| d < sd) {
                            sel = Some((m.x, m.y, d));
                        }
                    }
                    match sel {
                        Some((mx, my, _)) => (unit((mx - self.x, my - self.y)), (0.0, 0.0)),
                        None => {
                            let th = 2.0 * PI * self.rng.uniform();
                            ((libm::cos(th), libm::sin(th)), (0.0, 0.0))
                        }
                    }
                }
                _ => {
                    // solo: 前の一歩で |T−T_opt| が縮んだなら同じ向き、広がったなら向きを変える
                    let prev_dev = fabs(self.prev.2 - self.t_opt);
                    if norm(self.dir) > 0.0 && fabs(dev) <= prev_dev {
                        (self.dir, (0.0, 0.0))
                    } else {
                        let th = 2.0 * PI * self.rng.uniform();
                        ((libm::cos(th), libm::sin(th)), (0.0, 0.0))
                    }
                }
            }
        };

        // 群れとしてのまとまり (通信ありのモードのみ): 遠い隣へ弱く寄り、近すぎる隣から離れる
        if self.mode != MODE_SOLO && !msgs.is_empty() {
            let (mut cx, mut cy) = (0.0, 0.0);
            let (mut sx, mut sy) = (0.0, 0.0);
            for m in msgs {
                cx += m.x - self.x;
                cy += m.y - self.y;
                let d = norm((m.x - self.x, m.y - self.y));
                if d < 0.25 * self.radius && d > 1e-12 {
                    sx -= (m.x - self.x) / d;
                    sy -= (m.y - self.y) / d;
                }
            }
            let mf = msgs.len() as f64;
            let coh = unit((cx / mf, cy / mf));
            let sep = unit((sx, sy));
            dir = (dir.0 + 0.3 * coh.0 + 0.6 * sep.0, dir.1 + 0.3 * coh.1 + 0.6 * sep.1);
            dir = unit(dir);
        }

        // 最適域内では歩幅を落として留まる (|dev| に比例、最大 speed)
        let gain = (fabs(dev) / (2.0 * self.tol)).min(1.0);
        let sp = self.speed * (0.1 + 0.9 * gain);
        let mut nx = (self.x + sp * dir.0).clamp(0.0, 1.0);
        let mut ny = (self.y + sp * dir.1).clamp(0.0, 1.0);
        // 壁にぶつかって進めなかったら向きを変える (境界で張り付かない)
        if norm((nx - self.x, ny - self.y)) < 0.25 * sp && fabs(dev) >= self.tol {
            let th = 2.0 * PI * self.rng.uniform();
            dir = (libm::cos(th), libm::sin(th));
            nx = (self.x + sp * dir.0).clamp(0.0, 1.0);
            ny = (self.y + sp * dir.1).clamp(0.0, 1.0);
        }

        self.prev = (self.x, self.y, self.t);
        self.x = nx;
        self.y = ny;
        self.grad = grad_est;
        self.dir = dir;
        self.best = best;
    }
}

/// (位置, 温度) 標本群 (自分 + 近傍メッセージ) から温度の平面 T ≈ a + gx·x + gy·y を
/// 最小二乗で当てはめ、(gx, gy) を返す。標本3未満または退化 (一直線上) なら None。
/// 参照実装の fit_gradient と同一の式 (標本の並びも同じ: 自分が先頭、以下受信順)
fn fit_gradient(own: (f64, f64, f64), msgs: &[Msg]) -> Option<(f64, f64)> {
    let n = 1 + msgs.len();
    if n < 3 {
        return None;
    }
    let nf = n as f64;
    let (mut mx, mut my, mut mt) = (own.0, own.1, own.2);
    for m in msgs {
        mx += m.x;
        my += m.y;
        mt += m.t;
    }
    mx /= nf;
    my /= nf;
    mt /= nf;
    let (mut sxx, mut sxy, mut syy, mut sxt, mut syt) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let mut acc = |x: f64, y: f64, t: f64| {
        let (dx, dy, dt) = (x - mx, y - my, t - mt);
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
        sxt += dx * dt;
        syt += dy * dt;
    };
    acc(own.0, own.1, own.2);
    for m in msgs {
        acc(m.x, m.y, m.t);
    }
    let det = sxx * syy - sxy * sxy;
    if libm::fabs(det) < 1e-12 {
        return None;
    }
    let gx = (sxt * syy - syt * sxy) / det;
    let gy = (syt * sxx - sxt * sxy) / det;
    Some((gx, gy))
}

// ---------- WASMエクスポート (Dyne C ABI の群れ拡張) ----------

/// 個体の初期化: ID・初期位置・初期位置の真の温度 (雑音なし。solo の自己履歴の初期値)
#[no_mangle]
pub extern "C" fn swarm_init(id: u32, x: f64, y: f64, t0: f64) -> i32 {
    state().init(id, x, y, t0);
    0
}

/// パラメータ・シードの注入 (キーは CFG_*)
#[no_mangle]
pub extern "C" fn dyne_cfg(key: u32, val: f64) {
    state().cfg(key, val);
}

/// 知覚+送信箱の作成。temp = 自位置の真の温度 (ホストの場計算)
#[no_mangle]
pub extern "C" fn swarm_sense(temp: f64) {
    state().sense(temp);
}

/// 受信箱を読んで判断・移動
#[no_mangle]
pub extern "C" fn swarm_step() {
    state().step();
}

#[no_mangle]
pub extern "C" fn swarm_x() -> f64 {
    state().x
}

#[no_mangle]
pub extern "C" fn swarm_y() -> f64 {
    state().y
}

/// 転送用スクラッチ領域の先頭アドレス (ホストはここへ書き/ここから読む)
#[no_mangle]
pub extern "C" fn dyne_msg_buf() -> u32 {
    core::ptr::addr_of!(state().scratch) as usize as u32
}

#[no_mangle]
pub extern "C" fn dyne_msg_cap() -> u32 {
    SCRATCH_CAP as u32
}

/// ホスト→アクター: ptr (= dyne_msg_buf) に置かれた v2メッセージ列 (枠つき、len バイト) を
/// 受信箱へ追記する。0=全て受理 / 1=一部あふれ (破棄+計数) / -1=形式不正
#[no_mangle]
pub extern "C" fn dyne_inbox_write(ptr: u32, len: u32) -> i32 {
    let s = state();
    if ptr != core::ptr::addr_of!(s.scratch) as usize as u32 || len as usize > SCRATCH_CAP {
        return -1;
    }
    s.inbox_write(len as usize)
}

/// アクター→ホスト: 送信箱の内容を ptr (= dyne_msg_buf) へ書き出し、使用バイト数を返す。
/// cap 超過分は破棄+計数
#[no_mangle]
pub extern "C" fn dyne_outbox_read(ptr: u32, cap: u32) -> u32 {
    let s = state();
    if ptr != core::ptr::addr_of!(s.scratch) as usize as u32 {
        return 0;
    }
    s.outbox_read(cap as usize) as u32
}

/// あふれで破棄したメッセージの累計
#[no_mangle]
pub extern "C" fn dyne_dropped() -> u64 {
    state().dropped
}

#[cfg(target_arch = "wasm32")]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// ---------- ネイティブテスト ----------

#[cfg(test)]
mod tests {
    use super::*;

    /// 内蔵乱数がホスト側 (dyne-core::rng) の規約とビット一致することを照合する。
    /// 二重実装 (アクターは no_std で dyne-core に依存できない) のドリフト防止。
    #[test]
    fn rng_matches_dyne_core() {
        for global_seed in [42u64, 1, 2, 3] {
            for id in [0u64, 1, 59] {
                let seed = dyne_core::rng::derive_seed(global_seed, id);
                let mut ours = Rng(1);
                ours.reseed(seed);
                let mut theirs = dyne_core::rng::Rng::new(seed);
                for _ in 0..64 {
                    assert_eq!(ours.next_u64(), theirs.next_u64());
                }
                let mut ours = Rng(1);
                ours.reseed(seed);
                let mut theirs = dyne_core::rng::Rng::new(seed);
                for _ in 0..16 {
                    assert_eq!(ours.uniform().to_bits(), theirs.uniform().to_bits());
                }
            }
        }
    }

    /// シード注入の2キー分割 (LO→HI) が u64 を正確に復元すること
    #[test]
    fn seed_injection_roundtrip() {
        let mut a = agent_for_test();
        let seed = 0xDEAD_BEEF_1234_5678u64;
        a.cfg(CFG_SEED_LO, (seed & 0xFFFF_FFFF) as f64);
        a.cfg(CFG_SEED_HI, (seed >> 32) as f64);
        let mut expect = dyne_core::rng::Rng::new(seed);
        assert_eq!(a.rng.next_u64(), expect.next_u64());
    }

    fn agent_for_test() -> Agent {
        Agent {
            id: 0,
            step: 0,
            x: 0.0,
            y: 0.0,
            t: 0.0,
            prev: (0.0, 0.0, 0.0),
            grad: (0.0, 0.0),
            dir: (0.0, 0.0),
            best: None,
            rng: Rng(1),
            seed_lo: 0,
            mode: MODE_GOSSIP,
            t_opt: 24.0,
            tol: 1.0,
            speed: 0.004,
            noise: 0.3,
            radius: 0.12,
            gossip_max_age: 400,
            inbox: [0; INBOX_CAP],
            inbox_used: 0,
            outbox: [0; OUTBOX_CAP],
            outbox_used: 0,
            scratch: [0; SCRATCH_CAP],
            dropped: 0,
        }
    }

    /// sense → outbox_read → (別個体の) inbox_write → step の一巡:
    /// 送信箱に POSITION (+gossip の HEARSAY) が入り、受信箱経由で反対側に届くこと
    #[test]
    fn mailbox_roundtrip_between_agents() {
        let mut a = agent_for_test();
        a.init(1, 0.3, 0.4, 20.0);
        a.cfg(CFG_NOISE, 0.0);
        a.best = Some((0.5, 0.5, 0.2, 3));
        a.sense(20.0);
        let n = a.outbox_read(SCRATCH_CAP);
        assert!(n > 0);

        let mut b = agent_for_test();
        b.init(2, 0.31, 0.41, 21.0);
        b.cfg(CFG_NOISE, 0.0);
        // a のスクラッチ → b のスクラッチへ転送 (ホストの役割)
        b.scratch[..n].copy_from_slice(&a.scratch[..n]);
        assert_eq!(b.inbox_write(n), 0);
        // 受信箱に2通 (POSITION + HEARSAY) が入っている
        let cnt = wire::MailboxReader::new(&b.inbox[..b.inbox_used]).count();
        assert_eq!(cnt, 2);
        b.step(); // 消費してクリア
        assert_eq!(b.inbox_used, 0);
        // b は伝聞を採用しているはず (b は最適域外、聞いた地点は score 0.2 < tol)
        let (bx, by, bd, bage) = b.best.expect("gossip adopted");
        assert_eq!((bx, by, bd), (0.5, 0.5, 0.2));
        assert_eq!(bage, 4); // 3 + 1 (伝聞の段数で加齢)
    }

    /// 受信箱あふれ: 入り切らないメッセージは破棄+計数、部分書き込みなし
    #[test]
    fn inbox_overflow_discards_and_counts() {
        let mut a = agent_for_test();
        a.init(1, 0.5, 0.5, 24.0);
        a.sense(24.0);
        let n = a.outbox_read(SCRATCH_CAP);
        let mut b = agent_for_test();
        b.init(2, 0.5, 0.5, 24.0);
        let mut wrote = 0u64;
        loop {
            b.scratch[..n].copy_from_slice(&a.scratch[..n]);
            let r = b.inbox_write(n);
            if r != 0 {
                assert_eq!(r, 1);
                break;
            }
            wrote += 1;
            assert!(wrote < 10_000, "overflow never happened");
        }
        assert!(b.dropped > 0);
        assert!(b.inbox_used <= INBOX_CAP);
        // 受信箱の中身はすべて完全なメッセージ (壊れた断片がない)
        for item in wire::MailboxReader::new(&b.inbox[..b.inbox_used]) {
            item.expect("intact message");
        }
    }
}
