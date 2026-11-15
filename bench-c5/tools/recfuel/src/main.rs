//! recfuel: bench-c5 の記録 (.rec) を wasmi で再生し、実行した命令の数 (wasmi の燃料) を数える。
//! (2026-11-11、docs/2026-11-11-shape-speedup.md)
//!
//! 使い方: recfuel <actor.wasm> <record.rec> [<record.rec> ...]
//!
//! 出力 (記録ごとに1行): 1ステップあたりの燃料の平均・最大、内訳 (アクターが dyne_step_part を
//! 持てば 反復 / 取り込み / 減衰と統合)、送信・受信メールボックスの燃料、最後の CRC の一致。
//! 記録の書式は crates/dyne-sim2d/src/main.rs の REC_* と bench-c5/main/bench.c に合わせる。
//!
//! 注意: 燃料は「インタプリタが実行する命令の数」の目安で、ESP32 の時間とは比例しない
//! (f64 の加減乗除・floor・sqrt は ESP32 ではソフトウェアの関数呼び出しで、1命令が重い)。
//! 実機の時間は bench-c5 で測る。

use wasmi::{Config, Engine, Linker, Module, Store, TypedFunc};

fn rd32(b: &[u8], p: usize) -> u32 {
    u32::from_le_bytes(b[p..p + 4].try_into().unwrap())
}
fn rd64(b: &[u8], p: usize) -> u64 {
    u64::from_le_bytes(b[p..p + 8].try_into().unwrap())
}

fn crc32(d: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &x in d {
        c ^= x as u32;
        for _ in 0..8 {
            let m = (c & 1).wrapping_neg();
            c = (c >> 1) ^ (0xEDB8_8320 & m);
        }
    }
    !c
}

const FUEL0: u64 = u64::MAX / 2;

struct Acc {
    n: u64,
    sum: u64,
    max: u64,
}
impl Acc {
    fn new() -> Self {
        Acc { n: 0, sum: 0, max: 0 }
    }
    fn add(&mut self, v: u64) {
        self.n += 1;
        self.sum += v;
        self.max = self.max.max(v);
    }
    fn mean(&self) -> f64 {
        if self.n == 0 { 0.0 } else { self.sum as f64 / self.n as f64 }
    }
}

fn replay(engine: &Engine, module: &Module, rec: &[u8], name: &str) {
    let mut store = Store::new(engine, ());
    store.set_fuel(FUEL0).unwrap();
    let linker: Linker<()> = Linker::new(engine);
    let inst = linker.instantiate(&mut store, module).unwrap().start(&mut store).unwrap();
    let mem = inst.get_memory(&store, "memory").unwrap();
    let f_cfg: TypedFunc<(u32, f64), ()> = inst.get_typed_func(&store, "dyne_cfg").unwrap();
    let f_face: TypedFunc<(u32, u32, u64), ()> = inst.get_typed_func(&store, "dyne_set_face").unwrap();
    let f_init: TypedFunc<(u32, u32, u32, u32, f64, f64, f64), i32> = inst.get_typed_func(&store, "dyne_init2d").unwrap();
    let f_mbuf: TypedFunc<(), u32> = inst.get_typed_func(&store, "dyne_msg_buf").unwrap();
    let f_in: TypedFunc<(u32, u32), i32> = inst.get_typed_func(&store, "dyne_inbox_write").unwrap();
    let f_out: TypedFunc<(u32, u32), u32> = inst.get_typed_func(&store, "dyne_outbox_read").unwrap();
    let f_mcap: TypedFunc<(), u32> = inst.get_typed_func(&store, "dyne_msg_cap").unwrap();
    let f_step: TypedFunc<(), i32> = inst.get_typed_func(&store, "dyne_step").unwrap();
    let f_part: Option<TypedFunc<u32, i32>> = inst.get_typed_func(&store, "dyne_step_part").ok();
    let f_get: TypedFunc<u32, f64> = inst.get_typed_func(&store, "dyne_get").unwrap();
    let f_aux: TypedFunc<(u32, u32), f64> = inst.get_typed_func(&store, "dyne_get_aux").unwrap();
    let f_len: TypedFunc<(), u32> = inst.get_typed_func(&store, "dyne_len").unwrap();
    let f_ovf: Option<TypedFunc<(), u64>> = inst.get_typed_func(&store, "dyne_overflow").ok();
    let mbuf = f_mbuf.call(&mut store, ()).unwrap();
    let mcap = f_mcap.call(&mut store, ()).unwrap();
    assert_eq!(&rec[..8], b"DYNEREC1");
    let mut p = 8;
    let (mut step, mut parts, mut inb, mut outb) = (Acc::new(), [Acc::new(), Acc::new(), Acc::new()], Acc::new(), Acc::new());
    let mut cur_io = 0u64;
    let mut total = Acc::new();
    let mut exp = None;
    let used = |s: &Store<()>| FUEL0 - s.get_fuel().unwrap();
    while p < rec.len() {
        let tag = rec[p];
        p += 1;
        match tag {
            1 => {
                f_cfg.call(&mut store, (rd32(rec, p), f64::from_bits(rd64(rec, p + 4)))).unwrap();
                p += 12;
            }
            2 => {
                f_face.call(&mut store, (rd32(rec, p), rd32(rec, p + 4), rd64(rec, p + 8))).unwrap();
                p += 16;
            }
            3 => {
                let v = |k: usize| f64::from_bits(rd64(rec, p + 16 + 8 * k));
                let rc = f_init
                    .call(&mut store, (rd32(rec, p), rd32(rec, p + 4), rd32(rec, p + 8), rd32(rec, p + 12), v(0), v(1), v(2)))
                    .unwrap();
                assert_eq!(rc, 0, "dyne_init2d");
                p += 40;
            }
            4 => {
                let len = rd32(rec, p) as usize;
                p += 4;
                assert!(len <= mcap as usize);
                mem.write(&mut store, mbuf as usize, &rec[p..p + len]).unwrap();
                let f0 = used(&store);
                let rc = f_in.call(&mut store, (mbuf, len as u32)).unwrap();
                let d = used(&store) - f0;
                inb.add(d);
                cur_io += d;
                assert_eq!(rc, 0, "dyne_inbox_write");
                p += len;
            }
            5 => {
                let f0 = used(&store);
                f_out.call(&mut store, (mbuf, mcap)).unwrap();
                let d = used(&store) - f0;
                outb.add(d);
                cur_io += d;
            }
            6 => {
                let f0 = used(&store);
                if let Some(fp) = f_part {
                    for (k, acc) in parts.iter_mut().enumerate() {
                        let a = used(&store);
                        fp.call(&mut store, k as u32).unwrap();
                        acc.add(used(&store) - a);
                    }
                } else {
                    f_step.call(&mut store, ()).unwrap();
                }
                let d = used(&store) - f0;
                step.add(d);
                total.add(d + cur_io);
                cur_io = 0;
            }
            7 => {
                exp = Some((rd32(rec, p), [rd32(rec, p + 4), rd32(rec, p + 8), rd32(rec, p + 12)]));
                p += 16;
                break;
            }
            t => panic!("未知の記録の種別 {t} at {p}"),
        }
    }
    let n = f_len.call(&mut store, ()).unwrap();
    let mut crcs = [0u32; 3];
    for (q, c) in crcs.iter_mut().enumerate() {
        let mut b = Vec::with_capacity(n as usize * 8);
        for k in 0..n {
            let v = if q == 0 { f_get.call(&mut store, k).unwrap() } else { f_aux.call(&mut store, (q as u32, k)).unwrap() };
            b.extend_from_slice(&v.to_bits().to_le_bytes());
        }
        *c = crc32(&b);
    }
    let ovf = f_ovf.map(|f| f.call(&mut store, ()).unwrap()).unwrap_or(0);
    let ok = exp == Some((n, crcs));
    let pages = mem.size(&store);
    // スタックの使用の目安: 線形メモリの先頭 (スタックは先頭から下へ伸びる) で最初に0でないバイトの位置
    let low = mem.data(&store).iter().position(|&b| b != 0).unwrap_or(0);
    print!(
        "recfuel item={name} steps={} step_mean={:.0} step_max={} total_mean={:.0} inbox_mean={:.0} outbox_mean={:.0}",
        step.n,
        step.mean(),
        step.max,
        total.mean(),
        inb.mean(),
        outb.mean()
    );
    if f_part.is_some() {
        print!(" sweep_mean={:.0} intake_mean={:.0} fuse_mean={:.0}", parts[0].mean(), parts[1].mean(), parts[2].mean());
    }
    println!(" pages={pages} stack_low={low} overflow={ovf} crc={:08x}/{:08x}/{:08x} match={}", crcs[0], crcs[1], crcs[2], ok as u8);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("使い方: recfuel <actor.wasm> <record.rec> ...");
        std::process::exit(2);
    }
    let mut cfg = Config::default();
    cfg.consume_fuel(true);
    let engine = Engine::new(&cfg);
    let bin = std::fs::read(&args[1]).expect("wasm の読み込み");
    let module = Module::new(&engine, &bin[..]).expect("wasm のコンパイル");
    for r in &args[2..] {
        let rec = std::fs::read(r).expect("記録の読み込み");
        let name = std::path::Path::new(r).file_stem().unwrap().to_string_lossy().to_string();
        replay(&engine, &module, &rec, &name);
    }
}
