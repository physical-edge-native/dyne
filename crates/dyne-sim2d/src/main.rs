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
//!   2026-11-15 追加 (GitHub issue #26): 同じステップの重複受信にも控えを再送で応える
//!   (間隔は再送の最小間隔以上)。2×2 以上の分割では隣どうしが輪になり、輪の各ノードが
//!   「隣の1つからだけ受け取れていない」状態で同じステップに止まると、過去分の応答だけでは
//!   誰も控えを送らず止まる (1次元の鎖では輪がないので起きない)。
//!   さらに、進まないまま仮想時刻が長く過ぎたら、待ちの状態を示して止める (安全網)。
//!   受信の規則はファーム (node-esp32/main/node_v2.c、dyne_sync_core.c。issue #29) と共通で、正本は
//!   docs/2026-11-15-sim2d-loss-jitter.md の「同期の受信の規則」。変えるときは両方を変える。
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
//!   dyne-sim2d --wasm heat2d_mlp_actor.wasm --model field.dynm --px 2 --py 2 --steps 500
//!   (--model: 外部で学習したモデル (ONNX → scripts/onnx_import.py → .dynm) を各ノードに配り、
//!    各ノードが自分の区画の初期場をそのモデルの予測で作る。dyne_model_load を持つアクター用。
//!    指定しなければ従来の経路と完全に同じ動作)
//!
//! Field export (2026-11-16, GitHub issue #57; nothing changes without the option):
//!   --vti PATH … writes the final split field as a VTK ImageData file (.vti) for ParaView or PyVista
//!     (crates/dyne-export): array `u` for the heat and wave solvers; arrays `phi`, `d`, `w` and the true
//!     signed distance `truth` for the shape grid solver. Origin and spacing are those of the grid
//!     (origin 0 and h = 1/(nx − 1), or the scene's corner and --h). `dyne-vti info PATH` reads it back
//!     and prints the CRC-32 of each array, equal to the `crc32: split` values.
//!
//! Temperature from point readings (2026-11-18, GitHub issue #96, docs/en/tutorial-room.md; nothing changes
//! without --temp-sensors / --init-value):
//!   --scene PATH --h H … with the heat solver: the grid covers the scene's bounding rectangle with spacing H
//!     (only the extent is used; walls, furniture and distance sensors are ignored by the heat solver)
//!   --init-value T0 … every grid point starts at T0 (written with dyne_set after dyne_init2d)
//!   --temp-sensors CSV … point readings (columns id, x, y, T). Each reading goes to the node owning its nearest
//!     grid point; after every --obs-period K-th step (default 1) that node calls dyne_observe(k, T, gamma), i.e.
//!     u_k ← u_k + gamma·(T − u_k) (nudging, gamma = --nudge-gamma, default 0.5, 0 < gamma ≤ 1). The single-node
//!     solution applies the same readings at the same steps in the same order, so the PASS (bit-exact) check still
//!     holds. The line "nudging: ..." reports |T − u| at the reading points after the last step. The result is a
//!     model state kept close to the readings, not a measured field: the model has no heat sources.
//!
//! 形の変化への追従 (2026-11-11、docs/2026-11-11-dynamic-shape.md):
//!   --decay λ --w-forget w --disagree τ --disagree-w w … 古い観測を忘れる仕組みの設定
//!   --sweep-every n … 形の格子型アクターの距離の反復を n ステップに1回にする (既定 毎ステップ)
//!   --decay-in-view … 減衰・忘却を視野の中 (そのステップに統合する光線が通る格子点) だけに効かせる
//!   --decay-out λ … --decay-in-view のとき、視野の外の格子点を毎ステップ λ で減衰させる (既定 保つ)
//!   --track-scale-m L … --track-csv の採点 (幻の表面の閾値 2L・覆い率の半径 L) を刻み h の代わりに L で行う
//!     (指定しなければアクターに渡さず、従来と同じ動作・出力)
//!   --track-csv 経路 [--track-every N] … 単一ノード解の N ステップごとの採点を CSV に書く
//!     (その時刻の正解の形状に対して。動く箱のある場面ファイルで追従の時間を測る)
//!
//! 実機の入力の再生 (2026-11-15、GitHub issue #2、docs/2026-11-15-hw-replay.md。指定しなければ従来と同じ動作):
//!   --replay 経路 … 実機のノードがアクターへ渡した測距フレーム・IMU の標本 (scripts/hw_replay.py が MQTT の記録から作る
//!     再生の入力) を同じステップで渡し、ノードが報告した CRC32 と比べる (`replay-check ... OK|DIFF`)。
//!     単一ノード解との比較は既定で行わない (--replay-single で行う)
//!   --replay-record 経路 [--replay-hash-every N] … アクターへ渡した入力と N ステップごとの CRC32 を、実機の MQTT と
//!     同じ形の JSON Lines に書く (合成の記録。既定 N = 50 = ファームの field_every の既定)

mod replay;
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
    /// --vti PATH: the final split field as VTK ImageData (issue #57)
    vti: Option<String>,
    quiet: bool,
    /// 初期場のモデル (.dynm) の経路と中身 (heat2d-mlp-actor 用。None なら使わない)
    model: Option<String>,
    model_bytes: Option<Vec<u8>>,
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
    /// 古い観測を忘れる仕組み (None = アクターに渡さない = アクターの既定値)
    decay: Option<f64>,
    w_forget: Option<f64>,
    dis_tau: Option<f64>,
    dis_w: Option<f64>,
    /// --sweep-every n: 形の格子型アクターの距離の反復を n ステップに1回にする (既定は渡さない = 毎ステップ。
    /// 2026-11-11、docs/2026-11-11-shape-speedup.md)
    sweep_every: Option<f64>,
    /// --decay-in-view: 減衰・忘却を視野の中の格子点だけに効かせる (2026-11-13、docs/2026-11-11-shape-speedup.md)
    decay_in_view: Option<f64>,
    /// --decay-out λ: --decay-in-view のとき視野の外の格子点に掛ける減衰率 (既定は渡さない = 保つ)
    decay_out: Option<f64>,
    /// --track-scale-m L: --track-csv の採点の尺度を格子の刻みの代わりに固定の長さ L にする (2026-11-13)
    track_scale: Option<f64>,
    track_csv: Option<String>,
    track_every: u64,
    /// --cfg キー=値 (繰り返し可): 格子型以外のアクターへ dyne_init2d の前に dyne_cfg で渡す
    /// (波動方程式のアクターの初期条件・吸収境界など。2026-11-11、docs/2026-11-11-wave2d.md)
    extra_cfg: Vec<(u32, f64)>,
    /// --dump-node N --dump-file 経路: ノード N のアクターへの呼び出し (設定・面・初期化・
    /// 受信メールボックス・送信メールボックス・ステップ) を記録し、最後に期待 CRC を付けて書き出す。
    /// 実機の計測ファーム (bench-c5) が同じ呼び出しを再生して時間を測り、CRC を照合する
    /// (2026-11-11、docs/2026-11-11-esp32-interp-bench.md)
    dump_node: Option<usize>,
    dump_file: Option<String>,
    /// --single-wasm 経路: 単一ノード解だけを別の wasm で解く (実機用の縮小版の分割解を、
    /// フル版の単一ノード解と比べるため。縮小版は全域を1区画に持てない)
    single_wasm: Option<String>,
    /// --replay 経路: 実機の入力の再生 (replay.rs)
    replay: Option<String>,
    /// --replay-record 経路: アクターへ渡した入力と CRC32 を実機の MQTT と同じ形で書く
    replay_record: Option<String>,
    /// --replay-hash-every N: --replay-record の途中の CRC32 の間隔 [ステップ]
    replay_hash_every: u64,
    /// --replay-single: 再生のときも単一ノード解を解いて比べる (既定は比べない)
    replay_single: bool,
    // ---- Temperature readings for the heat solver (issue #96; nothing changes without --temp-sensors) ----
    /// --temp-sensors PATH: CSV of point readings (columns id,x,y,T; extra columns ignored)
    temp_sensors: Option<String>,
    /// --nudge-gamma γ: strength of the nudging towards the readings (0 < γ ≤ 1)
    nudge_gamma: f64,
    /// --obs-period k: nudge after every k-th step
    obs_period: u64,
    /// --init-value T0: uniform initial value written with dyne_set after dyne_init2d
    init_value: Option<f64>,
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
        vti: None,
        quiet: false,
        model: None,
        model_bytes: None,
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
        decay: None,
        w_forget: None,
        dis_tau: None,
        dis_w: None,
        sweep_every: None,
        decay_in_view: None,
        decay_out: None,
        track_scale: None,
        track_csv: None,
        track_every: 1,
        extra_cfg: Vec::new(),
        dump_node: None,
        dump_file: None,
        single_wasm: None,
        replay: None,
        replay_record: None,
        replay_hash_every: 50,
        replay_single: false,
        temp_sensors: None,
        nudge_gamma: 0.5,
        obs_period: 1,
        init_value: None,
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
            "--vti" => c.vti = Some(val),
            "--model" => c.model = Some(val),
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
            "--decay" => c.decay = Some(val.parse().unwrap()),
            "--w-forget" => c.w_forget = Some(val.parse().unwrap()),
            "--disagree" => c.dis_tau = Some(val.parse().unwrap()),
            "--disagree-w" => c.dis_w = Some(val.parse().unwrap()),
            "--sweep-every" => c.sweep_every = Some(val.parse().unwrap()),
            "--decay-out" => c.decay_out = Some(val.parse().unwrap()),
            "--decay-in-view" => {
                c.decay_in_view = Some(1.0);
                used = false;
            }
            "--track-csv" => c.track_csv = Some(val),
            "--track-every" => c.track_every = val.parse().unwrap(),
            "--track-scale-m" => c.track_scale = Some(val.parse().unwrap()),
            "--dump-node" => c.dump_node = Some(val.parse().unwrap()),
            "--dump-file" => c.dump_file = Some(val),
            "--single-wasm" => c.single_wasm = Some(val),
            "--replay" => c.replay = Some(val),
            "--replay-record" => c.replay_record = Some(val),
            "--replay-hash-every" => c.replay_hash_every = val.parse().unwrap(),
            "--replay-single" => {
                c.replay_single = true;
                used = false;
            }
            "--temp-sensors" => c.temp_sensors = Some(val),
            "--nudge-gamma" => c.nudge_gamma = val.parse().expect("--nudge-gamma <gamma>"),
            "--obs-period" => c.obs_period = val.parse().expect("--obs-period <steps>"),
            "--init-value" => c.init_value = Some(val.parse().expect("--init-value <value>")),
            "--cfg" => {
                let (k, v) = val.split_once('=').expect("--cfg キー=値");
                c.extra_cfg.push((k.parse().unwrap(), v.parse().unwrap()));
            }
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
    // ---- 初期場をモデルで作るアクターのみ (heat2d-mlp-actor) ----
    model_buf: Option<wasmi::TypedFunc<(), u32>>,
    model_load: Option<wasmi::TypedFunc<u32, i32>>,
    model_init: Option<wasmi::TypedFunc<(), i32>>,
    // ---- 実機の入力の再生 (2026-11-15) ----
    /// 状態の点の数 (ファームの CRC32 は dyne_get の 0..dyne_len を回す。無ければ区画の点の数)
    len: Option<wasmi::TypedFunc<(), u32>>,
    /// IMU の融合のアクター (imu-fusion-actor) のみ
    imu_push: Option<wasmi::TypedFunc<(i64, f64, f64, f64, f64, f64, f64), i32>>,
    // ---- Heat solver with readings (issue #96) ----
    /// dyne_set(k, v): write one grid point (uniform initial value)
    set: Option<wasmi::TypedFunc<(u32, f64), ()>>,
    /// dyne_observe(k, y, gamma) -> y - u before nudging
    observe: Option<wasmi::TypedFunc<(u32, f64, f64), f64>>,
    /// dyne_cfg の呼び出しの控え (--replay-record の actor_setup 用)
    cfg_trace: Vec<(u32, f64)>,
    buf: u32,
    cap: u32,
    memory_pages: u64,
    /// 呼び出しの記録 (--dump-node のノードのみ Some)
    log: Option<Vec<u8>>,
}

// 呼び出しの記録の書式 (すべて little endian、bench-c5/main/bench.c の再生と一致させること)
//   先頭 "DYNEREC1"
//   1 CFG   u32 key, f64 val
//   2 FACE  u32 face, u32 kind, u64 param
//   3 INIT  u32 nx, ny, ix0, iy0, f64 h, alpha, dt
//   4 INBOX u32 len, len バイト (終端印 0 を含む)
//   5 OUTBOX
//   6 STEP
//   7 END   u32 点数, u32 crc(φ), u32 crc(d), u32 crc(w) (格子型でなければ d・w は 0)
const REC_CFG: u8 = 1;
const REC_FACE: u8 = 2;
const REC_INIT: u8 = 3;
const REC_INBOX: u8 = 4;
const REC_OUTBOX: u8 = 5;
const REC_STEP: u8 = 6;
const REC_END: u8 = 7;

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
            model_buf: opt!("dyne_load_buf", (), u32),
            model_load: opt!("dyne_model_load", u32, i32),
            model_init: opt!("dyne_model_init_field", (), i32),
            len: opt!("dyne_len", (), u32),
            imu_push: opt!("dyne_imu_push", (i64, f64, f64, f64, f64, f64, f64), i32),
            set: opt!("dyne_set", (u32, f64), ()),
            observe: opt!("dyne_observe", (u32, f64, f64), f64),
            cfg_trace: Vec::new(),
            memory,
            store,
            buf: 0,
            cap: 0,
            memory_pages,
            log: None,
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
        if let Some(l) = self.log.as_mut() {
            l.push(REC_OUTBOX);
        }
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
        if let Some(l) = self.log.as_mut() {
            l.push(REC_INBOX);
            l.extend_from_slice(&(b.len() as u32).to_le_bytes());
            l.extend_from_slice(&b);
        }
        self.memory.write(&mut self.store, self.buf as usize, &b).unwrap();
        self.inbox_write.call(&mut self.store, (self.buf, b.len() as u32)).unwrap()
    }

    fn cfg_set(&mut self, key: u32, val: f64) {
        let f = self.cfg.expect("dyne_cfg が無い");
        if let Some(l) = self.log.as_mut() {
            l.push(REC_CFG);
            l.extend_from_slice(&key.to_le_bytes());
            l.extend_from_slice(&val.to_le_bytes());
        }
        self.cfg_trace.push((key, val));
        f.call(&mut self.store, (key, val)).unwrap();
    }

    /// ノードの状態の CRC32 と最小・最大 (ファームの state_crc と同じ: dyne_get の 0..dyne_len の f64 のビット列。
    /// 最小・最大は NaN を飛ばす)
    fn state_crc(&mut self, b: &Block) -> (u32, f64, f64) {
        let n = match self.len {
            Some(f) => f.call(&mut self.store, ()).unwrap(),
            None => (b.nx * b.ny) as u32,
        };
        let mut v = Vec::with_capacity(n as usize);
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for k in 0..n {
            let u = self.get.call(&mut self.store, k).unwrap();
            if u < lo {
                lo = u;
            }
            if u > hi {
                hi = u;
            }
            v.push(u);
        }
        (field_crc(&v), lo, hi)
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

/// 区画の面ごと (W, E, S, N) の (種類, 値): 界面なら (BC_INTERFACE, 界面ID)、外周なら --outer-bc の値
fn face_settings(b: &Block, cfg: &Config) -> [(u32, u64); 4] {
    let mut out = [(0u32, 0u64); 4];
    for (face, o) in out.iter_mut().enumerate() {
        *o = match b.faces[face] {
            Some((iface, _)) => (BC_INTERFACE, iface as u64),
            None => (cfg.outer_kind, cfg.outer_value.to_bits()),
        };
    }
    out
}

/// アクターに設定・面を入れて初期化する。sensors はこのアクターに付けるセンサ。
/// cfg_override が Some なら、dyne_cfg はシミュレータ自身の設定の代わりにその列だけを渡す (実機の記録の再生)
fn setup_actor(
    a: &mut Actor2d,
    b: &Block,
    cfg: &Config,
    g: &GridGeom,
    shape: Option<&ShapeCfg>,
    sensors: &[&scene::Sensor],
    cfg_override: Option<&[(u32, f64)]>,
) {
    if let Some(list) = cfg_override {
        for &(key, v) in list {
            a.cfg_set(key, v);
        }
    } else {
        setup_cfg(a, cfg, g, shape, sensors);
    }
    setup_faces_init(a, b, cfg, g);
}

/// シミュレータ自身の dyne_cfg の列 (従来の setup_actor の前半)
fn setup_cfg(a: &mut Actor2d, cfg: &Config, g: &GridGeom, shape: Option<&ShapeCfg>, sensors: &[&scene::Sensor]) {
    for &(key, v) in &cfg.extra_cfg {
        a.cfg_set(key, v);
    }
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
        for (key, v) in [
            (sdf2d::CFG_DECAY, cfg.decay),
            (sdf2d::CFG_W_FORGET, cfg.w_forget),
            (sdf2d::CFG_DISAGREE, cfg.dis_tau),
            (sdf2d::CFG_DISAGREE_W, cfg.dis_w),
            (sdf2d::CFG_SWEEP_EVERY, cfg.sweep_every),
            (sdf2d::CFG_DECAY_IN_VIEW, cfg.decay_in_view),
            (sdf2d::CFG_DECAY_OUT, cfg.decay_out),
        ] {
            if let Some(v) = v {
                a.cfg_set(key, v);
            }
        }
        for s in sensors {
            a.cfg_set(sdf2d::CFG_SENSOR_SEL, s.id as f64);
            a.cfg_set(sdf2d::CFG_SENSOR_X, s.x);
            a.cfg_set(sdf2d::CFG_SENSOR_Y, s.y);
            a.cfg_set(sdf2d::CFG_SENSOR_YAW, s.yaw);
        }
    }
}

/// 面の設定と dyne_init2d (と、あればモデルによる初期場)。従来の setup_actor の後半
fn setup_faces_init(a: &mut Actor2d, b: &Block, cfg: &Config, g: &GridGeom) {
    let faces = face_settings(b, cfg);
    for face in 0..4u32 {
        let (kind, param) = faces[face as usize];
        if let Some(l) = a.log.as_mut() {
            l.push(REC_FACE);
            l.extend_from_slice(&face.to_le_bytes());
            l.extend_from_slice(&kind.to_le_bytes());
            l.extend_from_slice(&param.to_le_bytes());
        }
        a.set_face.call(&mut a.store, (face, kind, param)).unwrap();
    }
    if let Some(l) = a.log.as_mut() {
        l.push(REC_INIT);
        for v in [b.nx as u32, b.ny as u32, b.ix0 as u32, b.iy0 as u32] {
            l.extend_from_slice(&v.to_le_bytes());
        }
        for v in [g.h, cfg.alpha, cfg.dt] {
            l.extend_from_slice(&v.to_le_bytes());
        }
    }
    let rc = a
        .init
        .call(&mut a.store, (b.nx as u32, b.ny as u32, b.ix0 as u32, b.iy0 as u32, g.h, cfg.alpha, cfg.dt))
        .unwrap();
    assert_eq!(rc, 0, "dyne_init2d が失敗 (rc={rc}, 区画 {:?})", b);
    if let Some(model) = &cfg.model_bytes {
        // 各ノードへ .dynm を配り、ノード自身が自分の区画の初期場を推論で作る
        let buf = a.model_buf.expect("dyne_load_buf が無い").call(&mut a.store, ()).unwrap();
        a.memory.write(&mut a.store, buf as usize, model).unwrap();
        let rc = a.model_load.unwrap().call(&mut a.store, model.len() as u32).unwrap();
        assert_eq!(rc, 0, "dyne_model_load が失敗 (rc={rc})");
        let rc = a.model_init.expect("dyne_model_init_field が無い").call(&mut a.store, ()).unwrap();
        assert_eq!(rc, 0, "dyne_model_init_field が失敗 (rc={rc}, 区画 {:?})", b);
    }
    if let Some(v) = cfg.init_value {
        // Uniform initial value (issue #96): every grid point of the block, written after dyne_init2d
        let f = a.set.expect("--init-value needs an actor with dyne_set (heat2d-actor)");
        for k in 0..(b.nx * b.ny) as u32 {
            f.call(&mut a.store, (k, v)).unwrap();
        }
    }
}

/// A temperature reading at a point (issue #96): one row of --temp-sensors
#[derive(Clone, Debug)]
struct TempSensor {
    id: String,
    x: f64,
    y: f64,
    value: f64,
    /// Nearest grid point (global indices)
    gi: usize,
    gj: usize,
}

/// Reads --temp-sensors: a CSV with a header line that has the columns id, x, y and T
/// (any order, extra columns ignored; lines starting with # are comments)
fn read_temp_sensors(path: &str, g: &GridGeom, nx: usize, ny: usize) -> Vec<TempSensor> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("--temp-sensors {path}: {e}"));
    let mut lines = text.lines().map(|l| l.trim()).filter(|l| !l.is_empty() && !l.starts_with('#'));
    let head: Vec<&str> = lines.next().expect("--temp-sensors: empty file").split(',').map(|c| c.trim()).collect();
    let col = |name: &str| head.iter().position(|c| *c == name).unwrap_or_else(|| panic!("--temp-sensors: no column {name} (header {head:?})"));
    let (ci, cx, cy, ct) = (col("id"), col("x"), col("y"), col("T"));
    let mut out = Vec::new();
    for l in lines {
        let v: Vec<&str> = l.split(',').map(|c| c.trim()).collect();
        let num = |k: usize| v.get(k).and_then(|t| t.parse::<f64>().ok()).unwrap_or_else(|| panic!("--temp-sensors: bad line {l:?}"));
        let (x, y, value) = (num(cx), num(cy), num(ct));
        let gi = ((x - g.x0) / g.h + 0.5).floor();
        let gj = ((y - g.y0) / g.h + 0.5).floor();
        assert!(
            gi >= 0.0 && gj >= 0.0 && (gi as usize) < nx && (gj as usize) < ny,
            "--temp-sensors: reading {} at ({x}, {y}) is outside the grid",
            v[ci]
        );
        assert!(value.is_finite(), "--temp-sensors: reading {} is not finite", v[ci]);
        out.push(TempSensor { id: v[ci].to_string(), x, y, value, gi: gi as usize, gj: gj as usize });
    }
    assert!(!out.is_empty(), "--temp-sensors: no readings");
    out
}

/// 1メッセージを枠 ([u32 len][メッセージ]) に入れて追記する
fn push_framed(out: &mut Vec<u8>, msg: &[u8]) {
    out.extend_from_slice(&(msg.len() as u32).to_le_bytes());
    out.extend_from_slice(msg);
}

/// 測距フレームのメッセージ (枠つき)
fn frame_unit(sensor: u32, step: u64, zones: &[(f64, u32)]) -> Vec<u8> {
    let n = zones.len() as u32;
    frame_unit_full(sensor, step, 0, zones.iter().copied(), n, n)
}

/// 記録した測距フレーム (実機の再生) のメッセージ (枠つき)。ファームの dyne_range_frame_build と同じ並び
fn frame_unit_rec(step: u64, r: &replay::RangeRec) -> Vec<u8> {
    frame_unit_full(r.sid, step, r.t_us, r.d.iter().copied().zip(r.st.iter().copied()), r.count, r.cols)
}

fn frame_unit_full(sensor: u32, step: u64, t_us: u64, zones: impl ExactSizeIterator<Item = (f64, u32)>, count: u32, cols: u32) -> Vec<u8> {
    let mut data = vec![0u8; dyne_wire::ZONE_LEN * zones.len()];
    for (i, (d, st)) in zones.enumerate() {
        dyne_wire::range_zone_put(&mut data, i, d, st);
    }
    let hdr = dyne_wire::Header {
        kind: dyne_wire::KIND_RANGE_FRAME,
        quantity: dyne_wire::QTY_NONE,
        src: sensor,
        step,
        t_us,
    };
    let pl = dyne_wire::Payload::RangeFrame { data: &data, count, cols };
    let mut buf = vec![0u8; dyne_wire::HDR_LEN + 8 + data.len()];
    let len = dyne_wire::encode(&mut buf, &hdr, &pl)
        .unwrap_or_else(|e| panic!("測距フレームを組み立てられない (センサ {sensor}, ステップ {step}, count {count}, cols {cols}): {e:?}"));
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
    /// Temperature readings on this node (issue #96): (index in Sim::temp, local grid point k)
    obs: Vec<(usize, u32)>,
    phase: Phase,
    cur_step: u64,
    got: Vec<bool>,
    pending: HashMap<(usize, u64), Msg>,
    sent: HashMap<(usize, u64), Msg>,
    last_resend_us: u64,
    /// 界面ごとの、同じステップの重複受信への最後の応答の時刻 (ステップの開始で None に戻す。issue #26)
    last_echo_us: Vec<Option<u64>>,
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
    /// サーボの首振りの模擬 (場面ファイルの pan N D。無ければ全センサが毎ステップ測る)
    pan: Option<(u32, u64)>,
    // ---- 実機の入力の再生 (2026-11-15。--replay / --replay-record を指定したときだけ使う) ----
    /// 記録した入力 (Some なら模擬のセンサの代わりにこれを渡す)
    replay: Option<Vec<replay::ReplayNode>>,
    /// --replay-record の書き手
    rec: Option<replay::Recorder>,
    /// ノードごとの受け渡しの計数
    rcount: Vec<replay::Counts>,
    /// CRC32 の照合の結果 (ノード, ステップ, 出力の行) と、食い違いの数
    rchecks: Vec<(usize, u64, String)>,
    rcheck_bad: usize,
    /// 最後にどれかのノードが計算を始めた仮想時刻 (進まないことの検出。issue #26)
    last_progress_us: u64,
    /// Temperature readings (issue #96; empty without --temp-sensors)
    temp: Vec<TempSensor>,
    /// Last innovation y - u (before nudging) of each reading
    innov: Vec<f64>,
}

/// 進まないことの検出の猶予 (仮想時刻)。最後の計算の開始と、最後の一時停止・分断の終わりの
/// どちらからもこれだけ過ぎて誰も計算を始めなければ、止まっているとみなす (issue #26)。
/// 損失が極端でも、再送の機会 (既定 100 ms ごと) が数万回ある長さにしてある
const STALL_LIMIT_US: u64 = 3_600_000_000;

impl Sim {
    /// センサ (場面のセンサ一覧の添字 si) がステップ step で測るか (pan N D の首振りの模擬)
    fn sensor_active(&self, si: usize, step: u64) -> bool {
        match self.pan {
            Some((n, d)) => self.sensor_ids[si] % n == ((step / d) % n as u64) as u32,
            None => true,
        }
    }

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
        if let Some(rp) = &self.replay {
            // 実機の再生: 記録したフレームを記録の順に渡す。受理されなくても止めない (ファームと同じく数える)
            let recs = rp[node].range.get(&step).cloned().unwrap_or_default();
            for r in recs {
                let rc = self.nodes[node].actor.deliver(&frame_unit_rec(step, &r));
                if rc == INBOX_OK {
                    self.rcount[node].range_fed += 1;
                } else {
                    self.rcount[node].range_rej += 1;
                }
                if let Some(w) = self.rec.as_mut() {
                    w.range(self.now_us, node, step, r.sid, r.t_us, r.count, r.cols, &r.d, &r.st);
                }
            }
        } else if !self.frames.is_empty() {
            let sensors = self.nodes[node].sensors.clone();
            for si in sensors {
                if !self.sensor_active(si, step) {
                    continue;
                }
                let unit = frame_unit(self.sensor_ids[si], step, &self.frames[step as usize][si]);
                let rc = self.nodes[node].actor.deliver(&unit);
                assert_eq!(rc, INBOX_OK, "測距フレームが受理されない (ノード{node})");
                if let Some(w) = self.rec.as_mut() {
                    self.rcount[node].range_fed += 1;
                    let z = &self.frames[step as usize][si];
                    let d: Vec<f64> = z.iter().map(|x| x.0).collect();
                    let st: Vec<u32> = z.iter().map(|x| x.1).collect();
                    w.range(self.now_us, node, step, self.sensor_ids[si], 0, z.len() as u32, z.len() as u32, &d, &st);
                }
            }
        }
        let now = self.now_us;
        let n = &mut self.nodes[node];
        n.cur_step = step;
        n.phase = Phase::Waiting { step };
        n.got = vec![false; n.ifaces.len()];
        n.last_echo_us = vec![None; n.ifaces.len()];
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
        self.last_progress_us = now;
        let n = &mut self.nodes[node];
        n.wait_us += now - n.wait_since;
        // 実機の再生: IMU の標本を計算の直前に渡す (ファームの feed_imu と同じ位置)
        if let Some(rp) = &self.replay {
            if let Some(samples) = rp[node].imu.get(&n.cur_step) {
                let f = n.actor.imu_push.expect("再生の入力に imu 行があるが、アクターに dyne_imu_push が無い");
                for &(t, v) in samples {
                    let rc = f.call(&mut n.actor.store, (t, v[0], v[1], v[2], v[3], v[4], v[5])).unwrap();
                    self.rcount[node].imu_rc(rc);
                }
                if let Some(w) = self.rec.as_mut() {
                    w.imu(now, node, n.cur_step, samples);
                }
            }
        }
        if let Some(l) = n.actor.log.as_mut() {
            l.push(REC_STEP);
        }
        let rc = n.actor.step.call(&mut n.actor.store, ()).unwrap();
        assert_eq!(rc, STEP_OK, "未受領の界面を抱えて計算した (ノード{node})");
        // Nudging towards the temperature readings (issue #96): right after the step, before the next
        // exchange, in the order of the readings file (the single-node solution does the same)
        if !n.obs.is_empty() && (n.cur_step + 1) % self.cfg.obs_period == 0 {
            let f = n.actor.observe.expect("--temp-sensors needs an actor with dyne_observe (heat2d-actor)");
            for &(si, k) in &n.obs {
                let d = f.call(&mut n.actor.store, (k, self.temp[si].value, self.cfg.nudge_gamma)).unwrap();
                self.innov[si] = d;
            }
        }
        n.phase = Phase::Computing;
        if self.replay.is_some() || self.rec.is_some() {
            self.after_step_replay(node);
        }
        let t = now + self.cfg.step_us;
        self.schedule(t, Ev::StepDone { node });
    }

    /// 実機の再生・記録: 計算の後のノードの CRC32 を記録と比べる / 記録に書く
    fn after_step_replay(&mut self, node: usize) {
        let step = self.nodes[node].cur_step;
        let last = step + 1 == self.cfg.steps;
        let want = self.replay.as_ref().and_then(|rp| rp[node].checks.get(&step)).cloned().unwrap_or_default();
        let hash_every = self.rec.as_ref().map(|w| w.hash_every);
        let record = hash_every.is_some_and(|e| (step + 1) % e == 0 || last);
        if want.is_empty() && !record {
            return;
        }
        let n = &mut self.nodes[node];
        let b = n.block.clone();
        let (crc, lo, hi) = n.actor.state_crc(&b);
        for (c, src) in want {
            let ok = c == crc;
            if !ok {
                self.rcheck_bad += 1;
            }
            let line = format!("replay-check node {node} step {step} {src} sim {crc:08x} node {c:08x} {}", if ok { "OK" } else { "DIFF" });
            self.rchecks.push((node, step, line));
        }
        if record {
            let now = self.now_us;
            let w = self.rec.as_mut().unwrap();
            w.field2d(now, node, step, b.nx, b.ny, b.ix0, b.iy0, crc, lo, hi);
            if last {
                let has_range = !self.frames.is_empty() || self.replay.as_ref().is_some_and(|rp| !rp[node].range.is_empty());
                let has_imu = n.actor.imu_push.is_some();
                w.done(now, node, crc, &self.rcount[node], has_range, has_imu);
            }
        }
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
                } else {
                    // 同じステップの重複 (issue #26): 隣はまだこのステップにいて、こちらの値を受け取れて
                    // いない (隣の再送)。控えを再送で応える。隣が先へ進んでいた場合 (過去分の応答の重複) の
                    // 往復を続けないよう、応答の間隔を再送の最小間隔以上にする
                    let now = self.now_us;
                    let due = self.nodes[node].last_echo_us[k].is_none_or(|t| now - t >= self.cfg.resend_min_us);
                    if due {
                        if let Some(m) = self.nodes[node].sent.get(&(msg.iface, step)).cloned() {
                            self.nodes[node].last_echo_us[k] = Some(now);
                            self.nodes[node].stat_replay += 1;
                            self.transmit(node, m, false);
                        }
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
        // 一時停止・分断の最後の終わり (その間は進まなくても止まったとはみなさない)
        let disrupt_end = self
            .cfg
            .pauses
            .iter()
            .map(|&(_, _, b)| b)
            .chain(self.cfg.partitions.iter().map(|&(_, _, _, e)| e))
            .max()
            .unwrap_or(0);
        while let Some((t, ev)) = self.queue.pop() {
            self.now_us = t;
            if t > self.last_progress_us.max(disrupt_end) + STALL_LIMIT_US {
                self.report_stall();
            }
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

    /// 進まないまま STALL_LIMIT_US が過ぎた: 待ちの状態を示して止める (issue #26。結果は出さない)
    fn report_stall(&self) -> ! {
        eprintln!(
            "STALL: no node started a step for {} s of virtual time (now {} us, last progress {} us); \
             the synchronization did not make progress. Node states:",
            STALL_LIMIT_US / 1_000_000,
            self.now_us,
            self.last_progress_us
        );
        for (i, n) in self.nodes.iter().enumerate() {
            let state = match n.phase {
                Phase::Waiting { step } => {
                    let missing: Vec<usize> =
                        (0..n.ifaces.len()).filter(|&k| !n.got[k]).map(|k| n.ifaces[k]).collect();
                    format!("waiting at step {step}, missing interfaces {missing:?}")
                }
                Phase::Computing => format!("computing step {}", n.cur_step),
                Phase::Done => "done".to_string(),
            };
            eprintln!("  node {i}: {state} (sent {} lost {} resend {} replay {})", n.stat_sent, n.stat_lost, n.stat_resend, n.stat_replay);
        }
        std::process::exit(2);
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

/// 推定した表面 (φ のゼロ等値線) の通過点。表示 (scripts/twin_demo.py) と同じく、
/// 観測済み (w>0) で |φ| < 2μ の隣り合う2点の間で符号が変わる所を線形補間で取る
fn zero_crossings(f: &Fields, nx: usize, ny: usize, g: &GridGeom, mu: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let ok = |k: usize| f.w[k] > 0.0 && f.phi[k].abs() < 2.0 * mu;
    for j in 0..ny {
        for i in 0..nx {
            let a = j * nx + i;
            for (b, dx, dy) in [(a + 1, 1.0, 0.0), (a + nx, 0.0, 1.0)] {
                if (dx > 0.0 && i + 1 >= nx) || (dy > 0.0 && j + 1 >= ny) {
                    continue;
                }
                if !ok(a) || !ok(b) || (f.phi[a] < 0.0) == (f.phi[b] < 0.0) {
                    continue;
                }
                let t = f.phi[a] / (f.phi[a] - f.phi[b]);
                out.push((g.x0 + (i as f64 + t * dx) * g.h, g.y0 + (j as f64 + t * dy) * g.h));
            }
        }
    }
    out
}

/// 点から多角形の周までの距離
fn poly_dist(p: (f64, f64), poly: &[(f64, f64)]) -> f64 {
    let n = poly.len();
    (0..n)
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            let (ex, ey) = (b.0 - a.0, b.1 - a.1);
            let l2 = ex * ex + ey * ey;
            let t = if l2 > 0.0 { (((p.0 - a.0) * ex + (p.1 - a.1) * ey) / l2).clamp(0.0, 1.0) } else { 0.0 };
            ((a.0 + t * ex - p.0).powi(2) + (a.1 + t * ey - p.1).powi(2)).sqrt()
        })
        .fold(f64::MAX, f64::min)
}

/// 追従の採点 (CSV の1行)。step の時刻の正解の形状に対して:
/// 通過点の数, 静止した表面の付近の通過点の位置の誤差 (平均・95%点), 幻の表面 (正解の表面から 2h より
/// 遠い通過点) の数, そのうち動く箱の今でない位置の付近 (μ 以内) のもの, 動く箱の今の位置の
/// 表面の覆い率と誤差の平均, 静止した表面の覆い率
fn track_row(full: &Scene, step: u64, f: &Fields, nx: usize, ny: usize, g: &GridGeom, mu: f64, sc: f64) -> String {
    // sc = 採点の尺度 (幻の表面の閾値 2·sc、覆い率の半径・標本の間隔 sc)。既定は格子の刻み h
    // (--track-scale-m で固定の長さにでき、刻みの違う実行を同じ基準で比べられる)
    let now = full.at_step(step);
    let xs = zero_crossings(f, nx, ny, g, mu);
    let cur: Vec<&Vec<(f64, f64)>> = full.movers.iter().filter_map(|m| m.pose_at(step)).collect();
    let all_poses: Vec<&Vec<(f64, f64)>> =
        full.movers.iter().flat_map(|m| m.poses.iter().filter_map(|(_, p)| p.as_ref())).collect();
    let others: Vec<&Vec<(f64, f64)>> =
        all_poses.iter().copied().filter(|p| !cur.iter().any(|c| std::ptr::eq(*c, *p))).collect();
    let near = |p: (f64, f64), set: &[&Vec<(f64, f64)>], r: f64| set.iter().any(|q| poly_dist(p, q) <= r);
    let tau = 2.0 * sc;
    let mut stat = Vec::new();
    let mut mov = Vec::new();
    let (mut phantom, mut phantom_old) = (0usize, 0usize);
    for &p in &xs {
        let e = now.signed_distance(p.0, p.1).abs();
        if e > tau {
            phantom += 1;
            if near(p, &others, mu) {
                phantom_old += 1;
            }
            continue;
        }
        if near(p, &cur, tau) {
            mov.push(e);
        } else if !near(p, &all_poses, tau) {
            stat.push(e);
        }
    }
    stat.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean = |v: &[f64]| if v.is_empty() { f64::NAN } else { v.iter().sum::<f64>() / v.len() as f64 };
    let p95 = |v: &[f64]| if v.is_empty() { f64::NAN } else { v[((v.len() - 1) as f64 * 0.95) as usize] };
    // 覆い率: 表面上の間隔 h の標本点のうち、h 以内に通過点があるもの
    let covered = |pts: &[(f64, f64)]| -> (usize, usize) {
        let c = pts
            .iter()
            .filter(|s| xs.iter().any(|x| (x.0 - s.0).powi(2) + (x.1 - s.1).powi(2) <= sc * sc))
            .count();
        (c, pts.len())
    };
    let mut mov_samples = Vec::new();
    for p in &cur {
        let tmp = Scene { room: (*p).clone(), polys: Vec::new(), circles: Vec::new(), sensors: Vec::new(), movers: Vec::new(), pan: None };
        mov_samples.extend(tmp.surface_samples(sc));
    }
    // 静止した表面 = 動く箱を除いた場面の表面
    let stat_scene = Scene {
        room: full.room.clone(),
        polys: full.polys.clone(),
        circles: full.circles.clone(),
        sensors: Vec::new(),
        movers: Vec::new(),
        pan: None,
    };
    let stat_samples = stat_scene.surface_samples(sc);
    let (mc, mn) = covered(&mov_samples);
    let (sc, sn) = covered(&stat_samples);
    format!(
        "{},{},{:.6},{:.6},{},{},{:.4},{:.6},{:.4}",
        step,
        xs.len(),
        mean(&stat),
        p95(&stat),
        phantom,
        phantom_old,
        if mn > 0 { mc as f64 / mn as f64 } else { f64::NAN },
        mean(&mov),
        sc as f64 / sn.max(1) as f64
    )
}

/// 記録した初期化の値・面と、シミュレータの区画・面の食い違いを `replay-setup` の行にする。
/// IMU の融合のアクターは dt だけを使い、外周の面を無視するので、dt と界面かどうかだけを比べる
fn replay_setup_diff(ni: usize, r: &replay::ReplayNode, b: &Block, cfg: &Config, g: &GridGeom, is_imu: bool) -> Vec<String> {
    let mut out = Vec::new();
    if let Some((nx, ny, ix0, iy0, h, alpha, dt)) = r.setup {
        let ints = [("nx", nx, b.nx), ("ny", ny, b.ny), ("ix0", ix0, b.ix0), ("iy0", iy0, b.iy0)];
        let reals = [("h", h, g.h), ("alpha", alpha, cfg.alpha), ("dt", dt, cfg.dt)];
        for (name, rec, sim) in ints {
            if !is_imu && rec as usize != sim {
                out.push(format!("replay-setup node {ni}: {name} {rec} (sim {sim})"));
            }
        }
        for (name, rec, sim) in reals {
            if (!is_imu || name == "dt") && rec.to_bits() != sim.to_bits() {
                out.push(format!("replay-setup node {ni}: {name} {rec:e} (sim {sim:e})"));
            }
        }
    }
    let sim_faces = face_settings(b, cfg);
    for &(f, kind, param) in &r.faces {
        let (sk, sp) = sim_faces[f as usize];
        let same = if is_imu || kind == BC_INTERFACE { (kind == BC_INTERFACE) == (sk == BC_INTERFACE) } else { kind == sk && param == sp };
        if !same {
            out.push(format!("replay-setup node {ni}: face {f} kind {kind} param {param:#x} (sim kind {sk} param {sp:#x})"));
        }
    }
    out
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
    let probe = Actor2d::load(&engine, &module);
    let is_shape = probe.is_shape();
    // 波動方程式のアクター (dyne_courant を持つ。alpha = 波の速さ c): 熱拡散の安定条件の検査と
    // 解析解を使わず、クーラン数と最大振幅を報告する (限界をまたぐ掃引のため止めない)
    let is_wave = module.exports().any(|e| e.name() == "dyne_courant");
    // IMU の融合のアクター (dyne_imu_push を持つ。2026-11-15): 格子を持たず dt (合意の1反復の周期) だけを使うので、
    // 熱拡散の安定条件の検査と解析解を使わない。入力の IMU の標本は --replay の記録からだけ来る
    let is_imu = probe.imu_push.is_some();
    assert!(!cfg.replay_single || cfg.replay.is_some(), "--replay-single は --replay と使う");
    assert!(!(cfg.replay_single && is_imu), "IMU の融合のアクターの再生では単一ノード解と比べられない (--replay-single を外す)");
    assert!(cfg.extra_cfg.is_empty() || (!is_shape && probe.cfg.is_some()), "--cfg は dyne_cfg を持つ格子型以外のアクター用");
    if let Some(path) = &cfg.model {
        assert!(!is_shape, "--model は熱拡散のアクター用");
        assert!(
            probe.model_load.is_some() && probe.model_init.is_some(),
            "--model には dyne_model_load / dyne_model_init_field を持つアクター (heat2d-mlp-actor) が必要"
        );
        cfg.model_bytes = Some(std::fs::read(path).expect("--model の読み込み"));
    }
    drop(probe);

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
    } else if let Some(path) = cfg.scene.clone() {
        // Heat solver in a room (issue #96): the grid covers the scene's bounding rectangle with spacing
        // --h, like the shape grid actor. Only the room's extent is used (not the walls, furniture or
        // distance sensors): the heat solver diffuses over the whole rectangle
        assert!(cfg.temp_sensors.is_some(), "--scene with a heat solver is for --temp-sensors (issue #96)");
        assert!(!is_wave && !is_imu && cfg.model.is_none(), "--temp-sensors is for the heat solver (heat2d-actor)");
        let sc = Scene::parse(&std::fs::read_to_string(&path).expect("場面ファイルの読み込み"));
        let (x0, y0, x1, y1) = sc.bounds();
        cfg.nx = ((x1 - x0) / cfg.h).round() as usize + 1;
        cfg.ny = ((y1 - y0) / cfg.h).round() as usize + 1;
        let r = cfg.alpha * cfg.dt / (cfg.h * cfg.h);
        assert!(r <= 0.25, "陽解法の安定条件 r = α·dt/h² ≤ 1/4 を満たさない (r = {r})");
        (GridGeom { x0, y0, h: cfg.h }, None, None)
    } else {
        let h = 1.0 / ((cfg.nx - 1) as f64);
        let r = cfg.alpha * cfg.dt / (h * h);
        assert!(is_wave || is_imu || r <= 0.25, "陽解法の安定条件 r = α·dt/h² ≤ 1/4 を満たさない (r = {r})");
        (GridGeom { x0: 0.0, y0: 0.0, h }, None, None)
    };
    if cfg.temp_sensors.is_some() {
        assert!(!is_shape && !is_wave && !is_imu, "--temp-sensors is for the heat solver (heat2d-actor)");
        assert!(cfg.nudge_gamma > 0.0 && cfg.nudge_gamma <= 1.0, "--nudge-gamma must be in (0, 1]");
        assert!(cfg.obs_period >= 1, "--obs-period must be at least 1");
        assert!(cfg.replay.is_none() && cfg.dump_node.is_none(), "--temp-sensors cannot be used with --replay or --dump-node");
    }
    assert!(cfg.init_value.is_none() || cfg.dump_node.is_none(), "--init-value cannot be used with --dump-node");
    assert!(cfg.nx >= 3 && cfg.ny >= 3, "格子は3×3点以上");

    let blocks = build_blocks(&cfg);
    for (i, b) in blocks.iter().enumerate() {
        assert!(b.nx >= 3 && b.ny >= 3, "ノード{i}の区画が3×3点未満 ({}×{})", b.nx, b.ny);
    }
    // Temperature readings (issue #96): each one goes to the node that owns its nearest grid point
    let temp = match &cfg.temp_sensors {
        Some(p) => read_temp_sensors(p, &geom, cfg.nx, cfg.ny),
        None => Vec::new(),
    };
    let mut node_obs: Vec<Vec<(usize, u32)>> = vec![Vec::new(); n_nodes];
    let mut temp_owner = Vec::new();
    for (si, t) in temp.iter().enumerate() {
        let owner = blocks.iter().position(|b| b.owns(t.gi as i64, t.gj as i64)).expect("reading outside the grid");
        let b = &blocks[owner];
        node_obs[owner].push((si, ((t.gj - b.iy0) * b.nx + (t.gi - b.ix0)) as u32));
        temp_owner.push(owner);
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

    // 実機の入力の再生 (--replay) と記録 (--replay-record)
    let replay_in = cfg.replay.as_ref().map(|p| {
        replay::parse(&std::fs::read_to_string(p).expect("--replay の読み込み"), n_nodes, cfg.steps)
    });
    let mut recorder = cfg.replay_record.clone().map(|p| replay::Recorder::new(p, cfg.replay_hash_every));
    // 記録した設定とシミュレータの区画・面の食い違い (再生の結果を信じてよいかの前提)
    let mut setup_diff: Vec<String> = Vec::new();

    let mut nodes = Vec::new();
    for (ni, b) in blocks.iter().enumerate() {
        let mut actor = Actor2d::load(&engine, &module);
        if cfg.dump_node == Some(ni) {
            assert!(cfg.model_bytes.is_none(), "--dump-node は --model と併用できない");
            actor.log = Some(b"DYNEREC1".to_vec());
        }
        let mine: Vec<&scene::Sensor> = match &scene {
            Some(sc) => node_sensors[ni].iter().map(|&si| &sc.sensors[si]).collect(),
            None => Vec::new(),
        };
        let rnode = replay_in.as_ref().map(|rp| &rp[ni]);
        let cfg_override = rnode.filter(|r| r.setup.is_some()).map(|r| r.cfg.as_slice());
        if let Some(r) = rnode {
            setup_diff.extend(replay_setup_diff(ni, r, b, &cfg, &geom, is_imu));
        }
        setup_actor(&mut actor, b, &cfg, &geom, shape_cfg.as_ref(), &mine, cfg_override);
        if let Some(w) = recorder.as_mut() {
            let trace = actor.cfg_trace.clone();
            w.setup(ni, &trace, &face_settings(b, &cfg), b.nx, b.ny, b.ix0, b.iy0, geom.h, cfg.alpha, cfg.dt);
        }
        let ifaces: Vec<usize> = b.faces.iter().flatten().map(|&(j, _)| j).collect();
        nodes.push(Node {
            actor,
            block: b.clone(),
            got: vec![false; ifaces.len()],
            ifaces,
            sensors: node_sensors[ni].clone(),
            obs: node_obs[ni].clone(),
            phase: Phase::Waiting { step: 0 },
            cur_step: 0,
            pending: HashMap::new(),
            sent: HashMap::new(),
            last_resend_us: 0,
            last_echo_us: Vec::new(),
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
        pan: scene.as_ref().and_then(|s| s.pan),
        replay: replay_in,
        rec: recorder,
        rcount: vec![replay::Counts::default(); n_nodes],
        rchecks: Vec::new(),
        rcheck_bad: 0,
        last_progress_us: 0,
        innov: vec![f64::NAN; temp.len()],
        temp: temp.clone(),
    };
    sim.run();
    if let Some(w) = &sim.rec {
        std::fs::write(&w.path, &w.out).expect("--replay-record の書き込み");
    }
    let replaying = sim.replay.is_some();
    // 単一ノード解は、再生のときは --replay-single を指定したときだけ解く (実機の再生の確かめは実機とシミュレータの比較)
    let do_single = !replaying || cfg.replay_single;

    // 分割解を全域に組み立てる
    let total = cfg.nx * cfg.ny;
    let blank = || Fields { phi: vec![f64::NAN; total], d: vec![0.0; total], w: vec![0.0; total] };
    let mut split_f = blank();
    for n in sim.nodes.iter_mut() {
        let b = n.block.clone();
        read_block(&mut n.actor, &b, cfg.nx, &mut split_f);
    }
    if let Some(dn) = cfg.dump_node {
        let path = cfg.dump_file.clone().expect("--dump-node には --dump-file が必要");
        let n = &mut sim.nodes[dn];
        let b = n.block.clone();
        let cnt = b.nx * b.ny;
        let shape = n.actor.is_shape();
        let (mut phi, mut d, mut w) = (Vec::new(), Vec::new(), Vec::new());
        for k in 0..cnt as u32 {
            phi.push(n.actor.get.call(&mut n.actor.store, k).unwrap());
            if shape {
                d.push(n.actor.aux(sdf2d::AUX_D, k));
                w.push(n.actor.aux(sdf2d::AUX_W, k));
            }
        }
        let crcs = [field_crc(&phi), if shape { field_crc(&d) } else { 0 }, if shape { field_crc(&w) } else { 0 }];
        let mut l = n.actor.log.take().unwrap();
        l.push(REC_END);
        l.extend_from_slice(&(cnt as u32).to_le_bytes());
        for c in crcs {
            l.extend_from_slice(&c.to_le_bytes());
        }
        std::fs::write(&path, &l).expect("--dump-file の書き込み");
        eprintln!(
            "dump: node {dn} block {}x{} at ({}, {}) sensors {} -> {path} ({} bytes) crc phi {:08x} d {:08x} w {:08x}",
            b.nx, b.ny, b.ix0, b.iy0, n.sensors.len(), l.len(), crcs[0], crcs[1], crcs[2]
        );
    }

    // 単一ノード解 (同じWASMの別インスタンスで全域を解く。センサは全部このノードに付ける)
    let whole = Block { ix0: 0, iy0: 0, nx: cfg.nx, ny: cfg.ny, faces: [None; 4] };
    let mut single_f = blank();
    let (mut single_late, mut single_overflow) = (0u64, 0u64);
    if do_single {
        let single_module = cfg.single_wasm.as_ref().map(|p| {
            let b = std::fs::read(p).expect("--single-wasm の読み込み");
            wasmi::Module::new(&engine, &b[..]).expect("--single-wasm のコンパイル")
        });
        let mut single = Actor2d::load(&engine, single_module.as_ref().unwrap_or(&module));
        let all_sensors: Vec<&scene::Sensor> = match &scene {
            Some(sc) => sc.sensors.iter().collect(),
            None => Vec::new(),
        };
        setup_actor(&mut single, &whole, &cfg, &geom, shape_cfg.as_ref(), &all_sensors, None);
        let mut track = String::new();
        if cfg.track_csv.is_some() {
            assert!(is_shape, "--track-csv は格子型アクター用");
            assert!(cfg.track_every >= 1, "--track-every は1以上");
            track.push_str("step,n_cross,static_err_mean,static_err_p95,phantom,phantom_old,mover_cov,mover_err_mean,static_cov\n");
        }
        for s in 0..cfg.steps {
            if let Some(rp) = &sim.replay {
                // 再生 (--replay-single): 全ノードの記録したフレームを、ノードの順・記録の順に渡す
                for nd in rp {
                    for r in nd.range.get(&s).map(|v| v.as_slice()).unwrap_or(&[]) {
                        single.deliver(&frame_unit_rec(s, r));
                    }
                }
            } else {
                for si in 0..sim.sensor_ids.len() {
                    if !sim.sensor_active(si, s) {
                        continue;
                    }
                    let unit = frame_unit(sim.sensor_ids[si], s, &sim.frames[s as usize][si]);
                    assert_eq!(single.deliver(&unit), INBOX_OK);
                }
            }
            let rc = single.step.call(&mut single.store, ()).unwrap();
            assert_eq!(rc, STEP_OK);
            // Nudging (issue #96): the same readings, the same steps, the same order as the split run
            if !temp.is_empty() && (s + 1) % cfg.obs_period == 0 {
                let f = single.observe.expect("dyne_observe");
                for t in &temp {
                    f.call(&mut single.store, ((t.gj * cfg.nx + t.gi) as u32, t.value, cfg.nudge_gamma)).unwrap();
                }
            }
            if cfg.track_csv.is_some() && (s + 1) % cfg.track_every == 0 {
                let mut tf = blank();
                read_block(&mut single, &whole, cfg.nx, &mut tf);
                let sc = cfg.track_scale.unwrap_or(geom.h);
                let row = track_row(scene.as_ref().unwrap(), s, &tf, cfg.nx, cfg.ny, &geom, shape_cfg.as_ref().unwrap().mu, sc);
                track.push_str(&row);
                track.push('\n');
            }
        }
        if let Some(path) = &cfg.track_csv {
            std::fs::write(path, &track).expect("--track-csv の書き込み");
        }
        read_block(&mut single, &whole, cfg.nx, &mut single_f);
        single_late = single.count(single.late);
        single_overflow = single.count(single.overflow);
    } else {
        // 再生で単一ノード解を解かない: 比較の相手を分割解そのものにして、分割解と単一ノード解の判定を出さない
        assert!(cfg.track_csv.is_none(), "--track-csv は単一ノード解を使う (再生では --replay-single と使う)");
        single_f = Fields { phi: split_f.phi.clone(), d: split_f.d.clone(), w: split_f.w.clone() };
    }

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
    let analytic_linf = if !is_shape && !is_wave && !is_imu && temp.is_empty() && cfg.init_value.is_none() && cfg.outer_kind == BC_DIRICHLET && cfg.outer_value == 0.0 && cfg.nx == cfg.ny {
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
        if cfg.decay.is_some() || cfg.w_forget.is_some() || cfg.dis_tau.is_some() || cfg.dis_w.is_some() {
            let show = |v: Option<f64>| v.map_or("default".to_string(), |x| x.to_string());
            writeln!(
                out,
                "forget: w-max {} decay {} w-forget {} disagree {} disagree-w {}",
                cfg.w_max,
                show(cfg.decay),
                show(cfg.w_forget),
                show(cfg.dis_tau),
                show(cfg.dis_w)
            )
            .unwrap();
        }
        if cfg.decay_in_view.is_some() {
            writeln!(out, "decay-in-view: on (out-of-view decay {})", cfg.decay_out.map_or("none".to_string(), |x| x.to_string())).unwrap();
        }
        if let Some(n) = cfg.sweep_every {
            writeln!(out, "sweep-every: {}", n).unwrap();
        }
        if let Some(scn) = &scene {
            for m in &scn.movers {
                let poses: Vec<String> = m
                    .poses
                    .iter()
                    .map(|(st, p)| match p {
                        Some(p) => format!("{}@({:.2},{:.2})-({:.2},{:.2})", st, p[0].0, p[0].1, p[2].0, p[2].1),
                        None => format!("{}@off", st),
                    })
                    .collect();
                writeln!(out, "mover {}: {}", m.name, poses.join(" -> ")).unwrap();
            }
        }
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
        if let (Some(path), Some(m)) = (&cfg.model, &cfg.model_bytes) {
            writeln!(out, "model: {} ({} bytes), initial field = model(x, y) evaluated on each node", path, m.len()).unwrap();
        }
        if let Some(v) = cfg.init_value {
            writeln!(out, "initial value: {} everywhere", v).unwrap();
        }
        if !temp.is_empty() {
            writeln!(
                out,
                "temp-sensors: {} readings from {} | grid h = {} m origin ({}, {}) | nudging gamma {} every {} steps | alpha {} dt {}",
                temp.len(),
                cfg.temp_sensors.as_deref().unwrap_or(""),
                geom.h,
                geom.x0,
                geom.y0,
                cfg.nudge_gamma,
                cfg.obs_period,
                cfg.alpha,
                cfg.dt
            )
            .unwrap();
            if !cfg.quiet {
                for (si, t) in temp.iter().enumerate() {
                    writeln!(
                        out,
                        "temp-sensor {}: at ({}, {}) -> grid ({}, {}) node {} reading {}",
                        t.id, t.x, t.y, t.gi, t.gj, temp_owner[si], t.value
                    )
                    .unwrap();
                }
            }
        }
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
    if !temp.is_empty() {
        // How close the model state is to the readings at the reading points (after the last nudge),
        // and the last innovation (before that nudge)
        let res: Vec<f64> = temp.iter().map(|t| (t.value - split_f.phi[t.gj * cfg.nx + t.gi]).abs()).collect();
        let inn: Vec<f64> = sim.innov.iter().map(|d| d.abs()).collect();
        let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
        let max = |v: &[f64]| v.iter().fold(0.0f64, |m, &x| if x.is_nan() || m.is_nan() { f64::NAN } else { m.max(x) });
        writeln!(
            out,
            "nudging: |reading - model| at the {} reading points after the last step: mean {:.4e} max {:.4e} | last innovation |y - u| before nudging: mean {:.4e} max {:.4e}",
            temp.len(),
            mean(&res),
            max(&res),
            mean(&inn),
            max(&inn)
        )
        .unwrap();
    }
    if let Some(e) = analytic_linf {
        writeln!(out, "analytic: Linf = {:.3e} (t = {})", e, t_end).unwrap();
    }
    if is_wave {
        let nu = cfg.alpha * cfg.dt / geom.h;
        let amax = split_f.phi.iter().fold(0.0f64, |m, v| if v.is_nan() || m.is_nan() { f64::NAN } else { m.max(v.abs()) });
        writeln!(
            out,
            "wave: c = {} courant nu = {:.6} (1/sqrt2 = 0.707107{}) cfg {:?} | max|u| = {:e} (t = {})",
            cfg.alpha,
            nu,
            if nu > std::f64::consts::FRAC_1_SQRT_2 { ", ABOVE the infinite-domain limit" } else { "" },
            cfg.extra_cfg,
            amax,
            t_end
        )
        .unwrap();
    }
    let mut pass = mismatch == 0;
    if let (Some(sc), Some(shape)) = (&scene, &shape_cfg) {
        let single_rays = if do_single {
            format!("(single: late {} overflow {})", single_late, single_overflow)
        } else {
            "(single: not solved)".to_string()
        };
        writeln!(
            out,
            "rays: late {} overflow {} {} | max delivery unit {} B{}",
            late_total,
            overflow_total,
            single_rays,
            max_unit,
            if max_unit > UDP_PAYLOAD_MAX { format!(" — WARNING: exceeds one UDP datagram ({} B)", UDP_PAYLOAD_MAX) } else { String::new() }
        )
        .unwrap();
        if late_total + overflow_total + single_late + single_overflow > 0 {
            pass = false;
        }
        // 採点は最後のステップの形に対して (動く箱の無い場面では場面そのもの)
        let last = sc.at_step(cfg.steps.saturating_sub(1));
        shape_report(&mut out, &last, &split_f, cfg.nx, cfg.ny, &geom, shape.mu);
        let single_crc = if do_single {
            format!("single phi {:08x} d {:08x} w {:08x}", field_crc(&single_f.phi), field_crc(&single_f.d), field_crc(&single_f.w))
        } else {
            "single not solved".to_string()
        };
        writeln!(
            out,
            "crc32: split phi {:08x} d {:08x} w {:08x} | {}",
            field_crc(&split_f.phi),
            field_crc(&split_f.d),
            field_crc(&split_f.w),
            single_crc
        )
        .unwrap();
    } else {
        let single_crc = if do_single { format!("single {:08x}", field_crc(&single_f.phi)) } else { "single not solved".to_string() };
        writeln!(out, "crc32: split {:08x} {}", field_crc(&split_f.phi), single_crc).unwrap();
    }
    // 実機の再生: 設定の食い違い、CRC32 の照合、ノードごとの計数と最後の CRC32 (scripts/hw_replay.py が読む)
    let mut replay_ok = true;
    if replaying {
        for l in &setup_diff {
            writeln!(out, "{}", l).unwrap();
        }
        let mut lines = sim.rchecks.clone();
        lines.sort_by_key(|(n, s, _)| (*n, *s));
        for (_, _, l) in &lines {
            writeln!(out, "{}", l).unwrap();
        }
        for (i, n) in sim.nodes.iter_mut().enumerate() {
            let b = n.block.clone();
            let (crc, _, _) = n.actor.state_crc(&b);
            let c = &sim.rcount[i];
            writeln!(
                out,
                "replay-node {}: range fed {} rej {} imu pushed {} late {} early {} crc32 {:08x}",
                i, c.range_fed, c.range_rej, c.imu_pushed, c.imu_late, c.imu_early, crc
            )
            .unwrap();
        }
        let k = lines.len();
        replay_ok = sim.rcheck_bad == 0 && setup_diff.is_empty();
        if !replay_ok {
            writeln!(out, "REPLAY FAIL: {} of {} node hashes differ, {} setup differences", sim.rcheck_bad, k, setup_diff.len()).unwrap();
        } else if k == 0 {
            writeln!(out, "REPLAY: no node hashes in the replay input (nothing compared)").unwrap();
        } else {
            writeln!(out, "REPLAY PASS: {} node hashes bit-exact ({} nodes)", k, n_nodes).unwrap();
        }
    }
    if !do_single {
        pass = true; // 単一ノード解との比較をしていない (判定は上の REPLAY の行)
    } else if mismatch == 0 && pass {
        writeln!(out, "PASS (bit-exact): {} points", total).unwrap();
    } else if mismatch == 0 {
        writeln!(out, "FAIL: bit-exact but rays were late or overflowed").unwrap();
    } else {
        writeln!(out, "FAIL: {} values differ, max |diff| = {:e}", mismatch, max_diff).unwrap();
    }
    pass &= replay_ok;
    print!("{}", out);

    if let Some(path) = &cfg.field_csv {
        let mut s = String::new();
        if let Some(sc) = &scene {
            let last = sc.at_step(cfg.steps.saturating_sub(1));
            s.push_str("gx,gy,x,y,phi,d,w,truth\n");
            for gy in 0..cfg.ny {
                for gx in 0..cfg.nx {
                    let k = gy * cfg.nx + gx;
                    let (x, y) = (geom.x0 + gx as f64 * geom.h, geom.y0 + gy as f64 * geom.h);
                    writeln!(
                        s,
                        "{},{},{},{},{:e},{:e},{},{:e}",
                        gx, gy, x, y, split_f.phi[k], split_f.d[k], split_f.w[k], last.signed_distance(x, y)
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
                        gx,
                        gy,
                        geom.x0 + gx as f64 * geom.h,
                        geom.y0 + gy as f64 * geom.h,
                        split_f.phi[k],
                        single_f.phi[k]
                    )
                    .unwrap();
                }
            }
        }
        std::fs::write(path, s).expect("--field-csv の書き込み");
    }
    if let Some(path) = &cfg.vti {
        let mut d = dyne_export::vti::ImageData::new_2d([cfg.nx, cfg.ny], [geom.x0, geom.y0], [geom.h, geom.h]);
        if let Some(sc) = &scene {
            let last = sc.at_step(cfg.steps.saturating_sub(1));
            let truth: Vec<f64> = (0..cfg.nx * cfg.ny)
                .map(|k| last.signed_distance(geom.x0 + (k % cfg.nx) as f64 * geom.h, geom.y0 + (k / cfg.nx) as f64 * geom.h))
                .collect();
            d.add("phi", split_f.phi.clone()).add("d", split_f.d.clone()).add("w", split_f.w.clone()).add("truth", truth);
        } else {
            d.add("u", split_f.phi.clone());
        }
        if let Err(e) = d.write_vti(path) {
            eprintln!("--vti: {e}");
            std::process::exit(1);
        }
    }

    std::process::exit(if mismatch == 0 && pass { 0 } else { 1 });
}
