//! dyne-host: M0 のホストランタイム。
//!
//! 役割 (docs A-3/A-4):
//! - WASM アクター (純粋計算体) をロードして実行する。
//! - 通信ロジックの「注入」: 境界同期 (Boundary Sync Protocol) を
//!   ブローカーを経由しない P2P (UDP) で行い、境界値をアクターへ出し入れする。
//! - 同期モデル: ステップ同期 (BSP 型)。全境界面について step n の値を
//!   交換し終えてから n+1 へ進む。時刻は論理ステップのみで壁時計を使わない。
//! - 検証: 終了時に解析解 u(x,t) = sin(pi x) exp(-alpha pi^2 t) との誤差を出す。
//!
//! 使い方 (2 ノード分割の例):
//!   dyne-host --wasm heat1d_actor.wasm --n 33 --x0 0.0 --x1 0.5 \
//!     --bc-left dirichlet --bc-right interface \
//!     --listen 127.0.0.1:7001 --peer-right 127.0.0.1:7002 \
//!     --alpha 1.0 --dt 1e-4 --steps 2000 --out nodeA.csv
//!
//! 単一ノード参照解 (分割なし):
//!   dyne-host --wasm ... --n 65 --x0 0.0 --x1 1.0 \
//!     --bc-left dirichlet --bc-right dirichlet --steps 2000 --out single.csv

use std::collections::HashMap;
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use dyne_proto::{BoundaryMsg, DecodeError, WIRE_LEN};

const FACE_LEFT: u32 = 0;
const FACE_RIGHT: u32 = 1;

#[derive(Clone)]
struct Config {
    wasm: String,
    n: u32,
    x0: f64,
    x1: f64,
    alpha: f64,
    dt: f64,
    steps: u64,
    bc_left: String,
    bc_right: String,
    listen: Option<String>,
    peer_left: Option<String>,
    peer_right: Option<String>,
    out: Option<String>,
    label: String,
}

fn parse_args() -> Config {
    let mut c = Config {
        wasm: String::new(),
        n: 65,
        x0: 0.0,
        x1: 1.0,
        alpha: 1.0,
        dt: 1e-4,
        steps: 2000,
        bc_left: "dirichlet".into(),
        bc_right: "dirichlet".into(),
        listen: None,
        peer_left: None,
        peer_right: None,
        out: None,
        label: "node".into(),
    };
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        let key = args[i].as_str();
        let val = || args.get(i + 1).cloned().unwrap_or_default();
        match key {
            "--wasm" => c.wasm = val(),
            "--n" => c.n = val().parse().unwrap(),
            "--x0" => c.x0 = val().parse().unwrap(),
            "--x1" => c.x1 = val().parse().unwrap(),
            "--alpha" => c.alpha = val().parse().unwrap(),
            "--dt" => c.dt = val().parse().unwrap(),
            "--steps" => c.steps = val().parse().unwrap(),
            "--bc-left" => c.bc_left = val(),
            "--bc-right" => c.bc_right = val(),
            "--listen" => c.listen = Some(val()),
            "--peer-left" => c.peer_left = Some(val()),
            "--peer-right" => c.peer_right = Some(val()),
            "--out" => c.out = Some(val()),
            "--label" => c.label = val(),
            _ => panic!("unknown arg: {key}"),
        }
        i += 2;
    }
    assert!(!c.wasm.is_empty(), "--wasm is required");
    c
}

fn bc_code(s: &str) -> u32 {
    match s {
        "dirichlet" => 0,
        "interface" => 1,
        other => panic!("unknown bc: {other}"),
    }
}

/// アクターの薄いラッパ。エクスポートされた C ABI を型付きで呼ぶ。
struct Actor {
    store: wasmi::Store<()>,
    init: wasmi::TypedFunc<(u32, f64, f64, f64, f64, u32, u32), i32>,
    step: wasmi::TypedFunc<(), ()>,
    bval: wasmi::TypedFunc<u32, f64>,
    bgrad: wasmi::TypedFunc<u32, f64>,
    set_neighbor: wasmi::TypedFunc<(u32, f64, f64), ()>,
    len: wasmi::TypedFunc<(), u32>,
    get: wasmi::TypedFunc<u32, f64>,
    x: wasmi::TypedFunc<u32, f64>,
}

impl Actor {
    fn load(path: &str) -> Self {
        let bytes = std::fs::read(path).expect("read wasm");
        let engine = wasmi::Engine::default();
        let module = wasmi::Module::new(&engine, &bytes[..]).expect("compile wasm");
        let mut store = wasmi::Store::new(&engine, ());
        let linker: wasmi::Linker<()> = wasmi::Linker::new(&engine);
        let instance = linker
            .instantiate(&mut store, &module)
            .expect("instantiate")
            .start(&mut store)
            .expect("start");
        macro_rules! func {
            ($name:literal, $params:ty, $ret:ty) => {
                instance
                    .get_typed_func::<$params, $ret>(&store, $name)
                    .expect(concat!("export ", $name))
            };
        }
        Actor {
            init: func!("dyne_init", (u32, f64, f64, f64, f64, u32, u32), i32),
            step: func!("dyne_step", (), ()),
            bval: func!("dyne_boundary_value", u32, f64),
            bgrad: func!("dyne_boundary_grad", u32, f64),
            set_neighbor: func!("dyne_set_neighbor", (u32, f64, f64), ()),
            len: func!("dyne_len", (), u32),
            get: func!("dyne_get", u32, f64),
            x: func!("dyne_x", u32, f64),
            store,
        }
    }
}

/// 1 つの境界面の P2P チャネル。
struct Face {
    face_id: u16,
    peer: String,
    seq: u32,
    /// 先着した将来ステップのメッセージ (順序入れ替わり対策)
    pending: HashMap<u64, BoundaryMsg>,
    /// 送信済みメッセージの控え (過去ステップの再要求への応答用)。
    /// BSP 同期ではノード間のずれは最大 1 ステップなので直近 2 ステップ分で足りる。
    sent: HashMap<u64, [u8; WIRE_LEN]>,
    resend_count: u64,
    replay_count: u64,
    crc_errors: u64,
}

fn main() {
    let cfg = parse_args();
    let mut actor = Actor::load(&cfg.wasm);

    // 安定条件の検証 (陽解法: r <= 1/2)
    let h = (cfg.x1 - cfg.x0) / ((cfg.n - 1) as f64);
    let r = cfg.alpha * cfg.dt / (h * h);
    assert!(
        r <= 0.5,
        "stability violated: alpha*dt/h^2 = {r:.4} > 0.5 (reduce dt or n)"
    );

    let rc = actor
        .init
        .call(
            &mut actor.store,
            (
                cfg.n,
                cfg.x0,
                cfg.x1,
                cfg.alpha,
                cfg.dt,
                bc_code(&cfg.bc_left),
                bc_code(&cfg.bc_right),
            ),
        )
        .expect("init call");
    assert_eq!(rc, 0, "dyne_init failed: {rc}");

    // 境界面の構成
    let mut faces: Vec<Face> = Vec::new();
    if cfg.bc_left == "interface" {
        faces.push(Face {
            face_id: FACE_LEFT as u16,
            peer: cfg.peer_left.clone().expect("--peer-left required"),
            seq: 0,
            pending: HashMap::new(),
            sent: HashMap::new(),
            resend_count: 0,
            replay_count: 0,
            crc_errors: 0,
        });
    }
    if cfg.bc_right == "interface" {
        faces.push(Face {
            face_id: FACE_RIGHT as u16,
            peer: cfg.peer_right.clone().expect("--peer-right required"),
            seq: 0,
            pending: HashMap::new(),
            sent: HashMap::new(),
            resend_count: 0,
            replay_count: 0,
            crc_errors: 0,
        });
    }
    let socket = if !faces.is_empty() {
        let s = UdpSocket::bind(cfg.listen.clone().expect("--listen required")).expect("bind");
        s.set_read_timeout(Some(Duration::from_millis(100))).unwrap();
        Some(s)
    } else {
        None
    };

    let t_start = Instant::now();

    let trace = std::env::var("DYNE_TRACE").is_ok();
    // ===== BSP メインループ =====
    for step in 0..cfg.steps {
        if trace && step < 5 {
            eprintln!("[{}] enter step {}", cfg.label, step);
        }
        // 1) 自分の境界値を全ての interface 面へ送信 (控えを残す)
        for f in faces.iter_mut() {
            let value = actor.bval.call(&mut actor.store, f.face_id as u32).unwrap();
            let grad = actor.bgrad.call(&mut actor.store, f.face_id as u32).unwrap();
            f.seq += 1;
            let msg = BoundaryMsg { face_id: f.face_id, step, seq: f.seq, value, grad };
            let bytes = msg.encode();
            f.sent.insert(step, bytes);
            f.sent.retain(|&k, _| k + 2 > step); // 直近 2 ステップ分だけ保持
            socket
                .as_ref()
                .unwrap()
                .send_to(&bytes, &f.peer)
                .expect("send");
        }
        // 2) 各 interface 面について、相手の step 一致メッセージを受信するまで待つ
        //    (タイムアウトしたら自分の値を再送 = 単純な at-least-once)
        for fi in 0..faces.len() {
            loop {
                // 先着分があればそれを使う
                if let Some(m) = faces[fi].pending.remove(&step) {
                    // 注意: m.face_id は「送信者から見た面」。自分側の面は
                    // faces[fi].face_id (= opposite(m.face_id))。M0 で実際に
                    // 混入したバグ: ここに m.face_id を渡すと自分の反対側の
                    // ゴーストを上書きし、正しい面は古い値のまま計算が進む。
                    if trace {
                        eprintln!("[{}] step {} ghost(face {}) = {:.17e} (from pending, msg.step={})",
                            cfg.label, step, faces[fi].face_id, m.value, m.step);
                    }
                    let my_face = faces[fi].face_id as u32;
                    actor
                        .set_neighbor
                        .call(&mut actor.store, (my_face, m.value, m.grad))
                        .unwrap();
                    break;
                }
                let mut buf = [0u8; WIRE_LEN];
                match socket.as_ref().unwrap().recv_from(&mut buf) {
                    Ok((len, _src)) => match BoundaryMsg::decode(&buf[..len]) {
                        Ok(m) => {
                            // 相手の face_id は「相手から見た面」。1D の 2 ノードでは
                            // 自分の面と相手の面は必ず異なるので、自分のどの面宛かは
                            // 反対面として解釈する。
                            let my_face = opposite(m.face_id);
                            if let Some(idx) =
                                faces.iter().position(|f| f.face_id as u32 == my_face)
                            {
                                if m.step == step && idx == fi {
                                    if trace {
                                        eprintln!("[{}] step {} ghost(face {}) = {:.17e} (direct, msg.step={})",
                                            cfg.label, step, my_face, m.value, m.step);
                                    }
                                    actor
                                        .set_neighbor
                                        .call(
                                            &mut actor.store,
                                            (my_face as u32, m.value, m.grad),
                                        )
                                        .unwrap();
                                    break;
                                } else if m.step >= step {
                                    // 将来分 or 別面: 取り置き
                                    faces[idx].pending.insert(m.step, m);
                                    if faces[fi].pending.contains_key(&step) {
                                        continue;
                                    }
                                } else {
                                    // 過去分: 相手はそのステップの自分の値を
                                    // まだ受け取れていない可能性が高い。
                                    // 控えを再送して相手を前進させる (起動レース・
                                    // パケット消失の両方をこれで解消する)。
                                    let f = &mut faces[idx];
                                    if let Some(bytes) = f.sent.get(&m.step) {
                                        socket
                                            .as_ref()
                                            .unwrap()
                                            .send_to(bytes, &f.peer)
                                            .expect("replay");
                                        f.replay_count += 1;
                                    }
                                }
                            }
                        }
                        Err(DecodeError::Crc) => faces[fi].crc_errors += 1,
                        Err(_) => {}
                    },
                    Err(_) => {
                        // タイムアウト: 現ステップの控えを再送
                        let f = &mut faces[fi];
                        if let Some(bytes) = f.sent.get(&step) {
                            socket
                                .as_ref()
                                .unwrap()
                                .send_to(bytes, &f.peer)
                                .expect("resend");
                        }
                        f.resend_count += 1;
                        if trace && f.resend_count <= 5 {
                            eprintln!("[{}] step {} resend #{}", cfg.label, step, f.resend_count);
                        }
                    }
                }
            }
        }
        // 3) 全面が揃ったので 1 ステップ進める
        actor.step.call(&mut actor.store, ()).unwrap();
    }
    let elapsed = t_start.elapsed();

    // ===== 終了時ドレイン =====
    // 自分が先に最終ステップを終えた場合、相手はまだ自分の最終境界値を
    // 受け取れていない可能性がある。しばらく受信を続け、過去ステップの
    // 再要求 (重複メッセージ) に控えの再送で応答してから終了する。
    if let Some(sock) = socket.as_ref() {
        sock.set_read_timeout(Some(Duration::from_millis(50))).unwrap();
        let drain_deadline = Instant::now() + Duration::from_millis(500);
        while Instant::now() < drain_deadline {
            let mut buf = [0u8; WIRE_LEN];
            if let Ok((len, _)) = sock.recv_from(&mut buf) {
                if let Ok(m) = BoundaryMsg::decode(&buf[..len]) {
                    let my_face = opposite(m.face_id);
                    if let Some(f) =
                        faces.iter_mut().find(|f| f.face_id as u32 == my_face)
                    {
                        if let Some(bytes) = f.sent.get(&m.step) {
                            let _ = sock.send_to(bytes, &f.peer);
                            f.replay_count += 1;
                        }
                    }
                }
            }
        }
    }

    // ===== 検証: 解析解との比較 =====
    let t_final = cfg.dt * (cfg.steps as f64);
    let n = actor.len.call(&mut actor.store, ()).unwrap();
    let decay = (-cfg.alpha * std::f64::consts::PI.powi(2) * t_final).exp();
    let mut l2 = 0.0f64;
    let mut linf = 0.0f64;
    let mut rows = String::from("x,u_numeric,u_analytic\n");
    for i in 0..n {
        let x = actor.x.call(&mut actor.store, i).unwrap();
        let u = actor.get.call(&mut actor.store, i).unwrap();
        let ua = (std::f64::consts::PI * x).sin() * decay;
        let e = (u - ua).abs();
        l2 += e * e;
        linf = linf.max(e);
        rows.push_str(&format!("{x:.6},{u:.12e},{ua:.12e}\n"));
    }
    l2 = (l2 / (n as f64)).sqrt();

    if let Some(out) = &cfg.out {
        std::fs::write(out, rows).expect("write csv");
    }

    let resend: u64 = faces.iter().map(|f| f.resend_count).sum();
    let replay: u64 = faces.iter().map(|f| f.replay_count).sum();
    let crc: u64 = faces.iter().map(|f| f.crc_errors).sum();
    println!(
        "[{}] steps={} t={:.4} n={} interfaces={} elapsed={:.2?} resend={} replay={} crc_err={} L2={:.3e} Linf={:.3e}",
        cfg.label, cfg.steps, t_final, n, faces.len(), elapsed, resend, replay, crc, l2, linf
    );
}

fn opposite(face: u16) -> u32 {
    if face as u32 == FACE_LEFT { FACE_RIGHT } else { FACE_LEFT }
}
