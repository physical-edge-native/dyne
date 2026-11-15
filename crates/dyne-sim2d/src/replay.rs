//! 実機の入力の再生と、その記録 (2026-11-15、GitHub issue #2、docs/2026-11-15-hw-replay.md)。
//!
//! 実機の測距・IMU の値はその場限りの入力なので、実機の結果とシミュレータの結果のビット一致は、
//! ノードがアクターへ渡した入力をそのまま記録して、同じアクターに同じステップで渡し直したときにだけ確かめられる。
//!
//! - `--replay 経路`: 再生の入力 (下の書式のテキスト。scripts/hw_replay.py が MQTT の記録から作る) を読み、
//!   シミュレータの模擬のセンサの代わりに、記録した測距フレームと IMU の標本を各ノードのアクターへ渡す。
//!   記録にノードの CRC32 (check 行) があれば、そのステップの計算の後の CRC32 と比べる。
//!   単一ノード解との比較は既定で行わない (`--replay-single` で行う)。
//! - `--replay-record 経路`: アクターへ渡した入力と各ノードの CRC32 を、実機のファームが MQTT へ発行するのと
//!   同じトピック・同じ形の JSON Lines で書く (合成の記録。scripts/hw_replay.py check でそのまま再生できる)。
//!
//! 再生の入力の書式 (1行1要素、空白区切り、`#` 以降は注釈。f64 は全て 16 桁の16進数のビット列):
//!   dyne-replay 1
//!   setup N nx ny ix0 iy0 h alpha dt        ノード N の初期化の値 (シミュレータの区画と照合する)。この行があると、
//!                                          ノード N の dyne_cfg はシミュレータ自身の設定の代わりに下の cfg 行だけになる
//!   cfg N key val                          ノード N の dyne_cfg の呼び出し (記録の順)
//!   face N f kind param                    ノード N の面 f (0..4 = W, E, S, N) の設定 (シミュレータの面と照合する)
//!   range N step sid t_us count cols d×count st×count   ステップ step の送信の後に渡す測距フレーム
//!   imu N step t_us ax ay az gx gy gz      ステップ step の計算 (dyne_step) の直前に dyne_imu_push へ渡す標本
//!   check N step crc32(8桁の16進数) 出どころ   ステップ step の計算の後のノードの CRC32 (dyne_get の全点の f64)

use std::collections::HashMap;
use std::fmt::Write as _;

/// 記録した測距フレーム1枚
#[derive(Clone, Debug)]
pub struct RangeRec {
    pub sid: u32,
    pub t_us: u64,
    pub count: u32,
    pub cols: u32,
    pub d: Vec<f64>,
    pub st: Vec<u32>,
}

/// 1ノードぶんの再生の入力
#[derive(Clone, Debug, Default)]
pub struct ReplayNode {
    /// (nx, ny, ix0, iy0, h, alpha, dt)
    pub setup: Option<(u32, u32, u32, u32, f64, f64, f64)>,
    pub cfg: Vec<(u32, f64)>,
    /// (面, 種類, 値)
    pub faces: Vec<(u32, u32, u64)>,
    pub range: HashMap<u64, Vec<RangeRec>>,
    pub imu: HashMap<u64, Vec<(i64, [f64; 6])>>,
    /// step → (CRC32, 出どころ)
    pub checks: HashMap<u64, Vec<(u32, String)>>,
}

fn bits(t: &str, ln: usize) -> f64 {
    f64::from_bits(u64::from_str_radix(t, 16).unwrap_or_else(|_| panic!("再生の入力 {ln}行目: f64 のビット列でない '{t}'")))
}

fn num<T: std::str::FromStr>(t: &str, ln: usize) -> T {
    t.parse().unwrap_or_else(|_| panic!("再生の入力 {ln}行目: 整数でない '{t}'"))
}

/// 再生の入力を読む。ノードの番号は n_nodes 未満、ステップは steps 未満でなければならない
pub fn parse(text: &str, n_nodes: usize, steps: u64) -> Vec<ReplayNode> {
    let mut nodes = vec![ReplayNode::default(); n_nodes];
    let mut header = false;
    for (i, raw) in text.lines().enumerate() {
        let ln = i + 1;
        let line = raw.split('#').next().unwrap().trim();
        if line.is_empty() {
            continue;
        }
        let t: Vec<&str> = line.split_whitespace().collect();
        if !header {
            assert!(t == ["dyne-replay", "1"], "再生の入力の1行目は 'dyne-replay 1'");
            header = true;
            continue;
        }
        assert!(t.len() >= 2, "再生の入力 {ln}行目: 短すぎる");
        let n: usize = num(t[1], ln);
        assert!(n < n_nodes, "再生の入力 {ln}行目: ノード {n} が範囲外 (ノードは {n_nodes} 台。--px/--py を実機と同じにする)");
        let step_of = |s: &str| {
            let v: u64 = num(s, ln);
            assert!(v < steps, "再生の入力 {ln}行目: ステップ {v} が --steps {steps} を超える (--steps を実機と同じにする)");
            v
        };
        let nd = &mut nodes[n];
        match t[0] {
            "setup" => {
                assert_eq!(t.len(), 9, "再生の入力 {ln}行目: setup N nx ny ix0 iy0 h alpha dt");
                nd.setup = Some((num(t[2], ln), num(t[3], ln), num(t[4], ln), num(t[5], ln), bits(t[6], ln), bits(t[7], ln), bits(t[8], ln)));
            }
            "cfg" => {
                assert_eq!(t.len(), 4, "再生の入力 {ln}行目: cfg N key val");
                nd.cfg.push((num(t[2], ln), bits(t[3], ln)));
            }
            "face" => {
                assert_eq!(t.len(), 5, "再生の入力 {ln}行目: face N f kind param");
                let f: u32 = num(t[2], ln);
                assert!(f < 4, "再生の入力 {ln}行目: 面は 0..4");
                nd.faces.push((f, num(t[3], ln), num(t[4], ln)));
            }
            "range" => {
                assert!(t.len() >= 7, "再生の入力 {ln}行目: range N step sid t_us count cols d... st...");
                let count: u32 = num(t[5], ln);
                assert_eq!(t.len(), 7 + 2 * count as usize, "再生の入力 {ln}行目: 区画の数が count と合わない");
                let d = (0..count as usize).map(|k| bits(t[7 + k], ln)).collect();
                let st = (0..count as usize).map(|k| num(t[7 + count as usize + k], ln)).collect();
                let r = RangeRec { sid: num(t[3], ln), t_us: num(t[4], ln), count, cols: num(t[6], ln), d, st };
                nd.range.entry(step_of(t[2])).or_default().push(r);
            }
            "imu" => {
                assert_eq!(t.len(), 10, "再生の入力 {ln}行目: imu N step t_us ax ay az gx gy gz");
                let mut v = [0.0; 6];
                for (k, x) in v.iter_mut().enumerate() {
                    *x = bits(t[4 + k], ln);
                }
                nd.imu.entry(step_of(t[2])).or_default().push((num(t[3], ln), v));
            }
            "check" => {
                assert!(t.len() == 4 || t.len() == 5, "再生の入力 {ln}行目: check N step crc32 [出どころ]");
                let c = u32::from_str_radix(t[3], 16).unwrap_or_else(|_| panic!("再生の入力 {ln}行目: CRC32 でない"));
                let src = t.get(4).unwrap_or(&"record").to_string();
                nd.checks.entry(step_of(t[2])).or_default().push((c, src));
            }
            k => panic!("再生の入力 {ln}行目: 未知の要素 '{k}'"),
        }
    }
    assert!(header, "再生の入力が空");
    nodes
}

/// ノードごとの再生の計数 (ファームの done の計数と同じ意味)
#[derive(Clone, Debug, Default)]
pub struct Counts {
    pub range_fed: u64,
    pub range_rej: u64,
    pub imu_pushed: u64,
    pub imu_late: u64,
    pub imu_early: u64,
}

impl Counts {
    /// dyne_imu_push の戻り値を数える (ファームの feed_imu と同じ分け方)
    pub fn imu_rc(&mut self, rc: i32) {
        match rc {
            0 => self.imu_pushed += 1,
            1 => self.imu_late += 1,
            2 => self.imu_early += 1,
            _ => {}
        }
    }
}

/// JSON の数値 (f64 を正確に戻せる最短の表記。有限でない値は Python の json が読む形)
pub fn jf(v: f64) -> String {
    if v.is_nan() {
        "NaN".to_string()
    } else if v.is_infinite() {
        if v > 0.0 { "Infinity".to_string() } else { "-Infinity".to_string() }
    } else {
        format!("{:?}", v)
    }
}

/// --replay-record の書き手 (実機の MQTT の記録と同じ形の JSON Lines)
pub struct Recorder {
    pub path: String,
    pub hash_every: u64,
    pub out: String,
    seq: HashMap<usize, u64>,
}

impl Recorder {
    pub fn new(path: String, hash_every: u64) -> Self {
        assert!(hash_every >= 1, "--replay-hash-every は1以上");
        Recorder { path, hash_every, out: String::new(), seq: HashMap::new() }
    }

    fn line(&mut self, t_us: u64, node: usize, kind: &str, payload: &str) {
        writeln!(self.out, "{{\"t\":{},\"topic\":\"dyne/node/{}/{}\",\"payload\":{}}}", jf(t_us as f64 / 1e6), node, kind, payload).unwrap();
    }

    #[allow(clippy::too_many_arguments)]
    pub fn setup(&mut self, node: usize, cfg: &[(u32, f64)], faces: &[(u32, u64)], nx: usize, ny: usize, ix0: usize, iy0: usize, h: f64, alpha: f64, dt: f64) {
        let c: Vec<String> = cfg.iter().map(|&(k, v)| format!("[{},{}]", k, jf(v))).collect();
        let f: Vec<String> = faces.iter().map(|&(k, p)| format!("[{},{}]", k, p)).collect();
        let p = format!(
            "{{\"idx\":{node},\"id\":{node},\"cfg\":[{}],\"faces\":[{}],\"init\":{{\"nx\":{nx},\"ny\":{ny},\"ix0\":{ix0},\"iy0\":{iy0},\"h\":{},\"alpha\":{},\"dt\":{}}},\"trunc\":false}}",
            c.join(","),
            f.join(","),
            jf(h),
            jf(alpha),
            jf(dt)
        );
        self.line(0, node, "actor_setup", &p);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn range(&mut self, t_us: u64, node: usize, step: u64, sid: u32, t_hdr: u64, count: u32, cols: u32, d: &[f64], st: &[u32]) {
        let seq = self.seq.entry(node).or_insert(0);
        *seq += 1;
        let s = *seq;
        let mm: Vec<String> = d.iter().map(|&x| if x.is_finite() { format!("{}", (x * 1000.0).round() as i64) } else { "0".into() }).collect();
        let dm: Vec<String> = d.iter().map(|&x| jf(x)).collect();
        let ss: Vec<String> = st.iter().map(|x| x.to_string()).collect();
        let p = format!(
            "{{\"seq\":{s},\"step\":{step},\"sid\":{sid},\"pos\":-1,\"t_us\":{t_hdr},\"t_loc\":0,\"count\":{count},\"cols\":{cols},\"d_mm\":[{}],\"d_m\":[{}],\"st\":[{}]}}",
            mm.join(","),
            dm.join(","),
            ss.join(",")
        );
        self.line(t_us, node, "range_fed", &p);
    }

    pub fn imu(&mut self, t_us: u64, node: usize, step: u64, samples: &[(i64, [f64; 6])]) {
        let s: Vec<String> = samples
            .iter()
            .map(|(t, v)| format!("[{},{}]", t, v.iter().map(|&x| jf(x)).collect::<Vec<_>>().join(",")))
            .collect();
        let p = format!("{{\"step\":{step},\"i0\":0,\"s\":[{}]}}", s.join(","));
        self.line(t_us, node, "imu_fed", &p);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn field2d(&mut self, t_us: u64, node: usize, step: u64, nx: usize, ny: usize, ix0: usize, iy0: usize, crc: u32, lo: f64, hi: f64) {
        let p = format!(
            "{{\"step\":{step},\"nx\":{nx},\"ny\":{ny},\"ix0\":{ix0},\"iy0\":{iy0},\"crc32\":\"0x{crc:08X}\",\"min\":{},\"max\":{}}}",
            jf(lo),
            jf(hi)
        );
        self.line(t_us, node, "field2d", &p);
    }

    pub fn done(&mut self, t_us: u64, node: usize, crc: u32, c: &Counts, range: bool, imu: bool) {
        let mut p = format!("{{\"state\":\"done\",\"idx\":{node},\"up_s\":0,\"wire\":2,\"crc32\":\"0x{crc:08X}\",\"replay_skip\":0");
        if range {
            write!(p, ",\"range_fed\":{},\"range_skip\":0,\"range_rej\":{}", c.range_fed, c.range_rej).unwrap();
        }
        if imu {
            write!(p, ",\"imu_pushed\":{},\"imu_late\":{},\"imu_early\":{},\"imu_lost\":0,\"imu_wait_to\":0", c.imu_pushed, c.imu_late, c.imu_early).unwrap();
        }
        p.push('}');
        self.line(t_us, node, "status", &p);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_all_kinds() {
        let one = 1.0f64.to_bits();
        let text = format!(
            "dyne-replay 1\n# note\nsetup 1 3 4 0 0 {one:016x} {one:016x} {one:016x}\ncfg 1 7 {one:016x}\nface 1 2 1 5\n\
             range 1 3 9 100 2 2 {one:016x} {one:016x} 5 0\nimu 0 4 -12 {one:016x} {one:016x} {one:016x} {one:016x} {one:016x} {one:016x}\n\
             check 0 4 deadBEEF field2d\n"
        );
        let n = parse(&text, 2, 10);
        assert_eq!(n[1].setup.unwrap().0, 3);
        assert_eq!(n[1].cfg, vec![(7, 1.0)]);
        assert_eq!(n[1].faces, vec![(2, 1, 5)]);
        let r = &n[1].range[&3][0];
        assert_eq!((r.sid, r.t_us, r.count, r.cols, r.st.clone()), (9, 100, 2, 2, vec![5, 0]));
        assert_eq!(n[0].imu[&4][0].0, -12);
        assert_eq!(n[0].checks[&4][0], (0xdeadbeef, "field2d".to_string()));
    }

    #[test]
    #[should_panic]
    fn step_beyond_steps_is_rejected() {
        parse("dyne-replay 1\ncheck 0 10 00000000\n", 1, 10);
    }

    #[test]
    fn jf_round_trips() {
        for v in [0.1, -2.5e-300, 1e300, 123456789.0] {
            assert_eq!(jf(v).parse::<f64>().unwrap().to_bits(), (v as f64).to_bits());
        }
    }
}
