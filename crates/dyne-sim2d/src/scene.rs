//! 模擬の部屋 (場面ファイル) と模擬のセンサ。**シミュレータ専用**。
//!
//! 設計書 docs/2026-10-10-shape-grid-actor-design.md §4。正解の形状はここにだけあり、
//! アクターには渡さない (アクターが受け取るのは測距フレーム=センサが返すのと同じ形のデータ)。
//!
//! 場面ファイルの書式 (1行1要素、単位メートル、`#` 以降は注釈):
//!   room x0 y0 x1 y1              外壁 (長方形。内側が部屋)
//!   roompoly x1 y1 x2 y2 ...      外壁 (多角形。内側が部屋)
//!   box x0 y0 x1 y1               遮蔽物 (長方形)
//!   poly x1 y1 x2 y2 ...          遮蔽物 (多角形)
//!   circle cx cy r                遮蔽物 (円)
//!   sensor id x y yaw_deg [高さ]  センサ (向きは x軸から反時計回りの度。高さは記録のみ)

use dyne_core::rng::{derive_seed, Rng};

#[derive(Clone, Debug)]
pub struct Sensor {
    pub id: u32,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub height: f64,
}

#[derive(Clone, Debug)]
pub struct Scene {
    /// 外壁の多角形 (頂点列。閉じている)
    pub room: Vec<(f64, f64)>,
    /// 遮蔽物の多角形
    pub polys: Vec<Vec<(f64, f64)>>,
    /// 遮蔽物の円 (cx, cy, r)
    pub circles: Vec<(f64, f64, f64)>,
    pub sensors: Vec<Sensor>,
}

fn nums(toks: &[&str], line: usize) -> Vec<f64> {
    toks.iter()
        .map(|t| t.parse::<f64>().unwrap_or_else(|_| panic!("場面ファイル {line}行目: 数値でない '{t}'")))
        .collect()
}

fn rect(v: &[f64]) -> Vec<(f64, f64)> {
    let (x0, y0, x1, y1) = (v[0].min(v[2]), v[1].min(v[3]), v[0].max(v[2]), v[1].max(v[3]));
    vec![(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
}

fn pairs(v: &[f64], line: usize) -> Vec<(f64, f64)> {
    assert!(v.len() >= 6 && v.len() % 2 == 0, "場面ファイル {line}行目: 多角形は3頂点以上の x y の組");
    v.chunks(2).map(|c| (c[0], c[1])).collect()
}

impl Scene {
    pub fn parse(text: &str) -> Scene {
        let mut sc = Scene { room: Vec::new(), polys: Vec::new(), circles: Vec::new(), sensors: Vec::new() };
        for (ln, raw) in text.lines().enumerate() {
            let line = ln + 1;
            let body = raw.split('#').next().unwrap().trim();
            if body.is_empty() {
                continue;
            }
            let toks: Vec<&str> = body.split_whitespace().collect();
            let v = nums(&toks[1..], line);
            match toks[0] {
                "room" => {
                    assert_eq!(v.len(), 4, "場面ファイル {line}行目: room x0 y0 x1 y1");
                    sc.room = rect(&v);
                }
                "roompoly" => sc.room = pairs(&v, line),
                "box" => {
                    assert_eq!(v.len(), 4, "場面ファイル {line}行目: box x0 y0 x1 y1");
                    sc.polys.push(rect(&v));
                }
                "poly" => sc.polys.push(pairs(&v, line)),
                "circle" => {
                    assert_eq!(v.len(), 3, "場面ファイル {line}行目: circle cx cy r");
                    sc.circles.push((v[0], v[1], v[2]));
                }
                "sensor" => {
                    assert!(v.len() == 4 || v.len() == 5, "場面ファイル {line}行目: sensor id x y yaw_deg [高さ]");
                    sc.sensors.push(Sensor {
                        id: v[0] as u32,
                        x: v[1],
                        y: v[2],
                        yaw: v[3].to_radians(),
                        height: if v.len() == 5 { v[4] } else { 0.0 },
                    });
                }
                k => panic!("場面ファイル {line}行目: 未知の要素 '{k}'"),
            }
        }
        assert!(!sc.room.is_empty(), "場面ファイルに room / roompoly が無い");
        for s in &sc.sensors {
            assert!(s.id < 8, "センサIDは0〜7");
            assert!(sc.signed_distance(s.x, s.y) > 0.0, "センサ{}が部屋の空きの外にある", s.id);
        }
        sc
    }

    /// 外壁の外接長方形 (x0, y0, x1, y1)
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for &(x, y) in &self.room {
            b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
        }
        b
    }

    fn edges(&self) -> impl Iterator<Item = ((f64, f64), (f64, f64))> + '_ {
        let ring = |p: &Vec<(f64, f64)>| {
            let n = p.len();
            (0..n).map(move |i| (p[i], p[(i + 1) % n])).collect::<Vec<_>>()
        };
        ring(&self.room).into_iter().chain(self.polys.iter().flat_map(ring))
    }

    /// 光線 (原点, 単位ベクトル) が最初に当たる表面までの距離。当たらなければ None
    pub fn raycast(&self, ox: f64, oy: f64, vx: f64, vy: f64) -> Option<f64> {
        let mut best: Option<f64> = None;
        let mut take = |t: f64| {
            if t > 1e-9 && best.map_or(true, |b| t < b) {
                best = Some(t);
            }
        };
        for ((ax, ay), (bx, by)) in self.edges() {
            // o + t v = a + u (b − a)
            let (ex, ey) = (bx - ax, by - ay);
            let den = vx * ey - vy * ex;
            if den.abs() < 1e-15 {
                continue;
            }
            let (wx, wy) = (ax - ox, ay - oy);
            let t = (wx * ey - wy * ex) / den;
            let u = (wx * vy - wy * vx) / den;
            if (0.0..=1.0).contains(&u) {
                take(t);
            }
        }
        for &(cx, cy, r) in &self.circles {
            let (px, py) = (ox - cx, oy - cy);
            let b = px * vx + py * vy;
            let c = px * px + py * py - r * r;
            let disc = b * b - c;
            if disc >= 0.0 {
                let sq = libm::sqrt(disc);
                take(-b - sq);
                take(-b + sq);
            }
        }
        best
    }

    /// 正解の符号つき距離 (部屋の空きで正、遮蔽物の中・外壁の外で負)
    pub fn signed_distance(&self, x: f64, y: f64) -> f64 {
        let mut d = f64::MAX;
        for ((ax, ay), (bx, by)) in self.edges() {
            d = d.min(seg_dist(x, y, ax, ay, bx, by));
        }
        let mut inside_obstacle = false;
        for &(cx, cy, r) in &self.circles {
            let q = libm::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
            d = d.min((q - r).abs());
            if q < r {
                inside_obstacle = true;
            }
        }
        for p in &self.polys {
            if point_in_poly(x, y, p) {
                inside_obstacle = true;
            }
        }
        let free = point_in_poly(x, y, &self.room) && !inside_obstacle;
        if free { d } else { -d }
    }

    /// 表面上に間隔 step で点を取る (覆い率の採点用)
    pub fn surface_samples(&self, step: f64) -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        for ((ax, ay), (bx, by)) in self.edges() {
            let len = libm::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
            let n = (len / step).ceil().max(1.0) as usize;
            for k in 0..n {
                let t = (k as f64 + 0.5) / n as f64;
                out.push((ax + t * (bx - ax), ay + t * (by - ay)));
            }
        }
        for &(cx, cy, r) in &self.circles {
            let n = ((2.0 * std::f64::consts::PI * r) / step).ceil().max(4.0) as usize;
            for k in 0..n {
                let th = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
                out.push((cx + r * libm::cos(th), cy + r * libm::sin(th)));
            }
        }
        out
    }
}

fn seg_dist(x: f64, y: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let (ex, ey) = (bx - ax, by - ay);
    let l2 = ex * ex + ey * ey;
    let t = if l2 > 0.0 { (((x - ax) * ex + (y - ay) * ey) / l2).clamp(0.0, 1.0) } else { 0.0 };
    let (px, py) = (ax + t * ex - x, ay + t * ey - y);
    libm::sqrt(px * px + py * py)
}

fn point_in_poly(x: f64, y: f64, p: &[(f64, f64)]) -> bool {
    let mut inside = false;
    let n = p.len();
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = p[i];
        let (xj, yj) = p[j];
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// 模擬のセンサの設定
#[derive(Clone, Debug)]
pub struct SensorModel {
    pub cols: u32,
    pub fov: f64,
    pub max_range: f64,
    pub noise_m: f64,
    /// 0 = 斜めの距離、1 = 正面への垂直距離 (dyne_wire::abi::sdf2d::RANGE_DEF_*)
    pub range_def: u32,
}

/// 状態値: 有効 / 対象なし (VL53L5CX の値に合わせる)
pub const STATUS_VALID: u32 = 5;
pub const STATUS_NO_TARGET: u32 = 255;

/// 全ステップ×全センサの測距フレーム (区画ごとの (距離, 状態値)) を前もって作る。
/// 乱数はセンサごとに derive_seed(大域シード, 1000+センサID) で分離 (ネットワークの乱数とは独立)
pub fn make_frames(sc: &Scene, model: &SensorModel, steps: u64, seed: u64) -> Vec<Vec<Vec<(f64, u32)>>> {
    let mut rngs: Vec<Rng> = sc.sensors.iter().map(|s| Rng::new(derive_seed(seed, 1000 + s.id as u64))).collect();
    let mut frames = Vec::with_capacity(steps as usize);
    for _ in 0..steps {
        let mut per_sensor = Vec::with_capacity(sc.sensors.len());
        for (si, s) in sc.sensors.iter().enumerate() {
            let mut zones = Vec::with_capacity(model.cols as usize);
            for c in 0..model.cols {
                let off = dyne_wire::zone_angle_offset(c, model.cols, model.fov);
                let th = s.yaw + off;
                let (vx, vy) = (libm::cos(th), libm::sin(th));
                // 雑音は区画ごとに必ず2本消費する (当たり外れで乱数の列がずれないように)
                let u1 = rngs[si].uniform().max(1e-300);
                let u2 = rngs[si].uniform();
                let n01 = libm::sqrt(-2.0 * libm::log(u1)) * libm::cos(2.0 * std::f64::consts::PI * u2);
                match sc.raycast(s.x, s.y, vx, vy) {
                    Some(t) if t <= model.max_range => {
                        let mut dist = if model.range_def == 1 { t * libm::cos(off) } else { t };
                        dist += model.noise_m * n01;
                        zones.push((dist.max(1e-3), STATUS_VALID));
                    }
                    _ => zones.push((0.0, STATUS_NO_TARGET)),
                }
            }
            per_sensor.push(zones);
        }
        frames.push(per_sensor);
    }
    frames
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn room_box_circle() {
        let sc = Scene::parse("room 0 0 4 4\nbox 1 1 2 2\ncircle 3 3 0.1\nsensor 0 0.5 0.5 0\n");
        // 空きの中心付近は正、箱の中は負、壁の外は負
        assert!(sc.signed_distance(3.0, 1.0) > 0.0);
        assert!(sc.signed_distance(1.5, 1.5) < 0.0);
        assert!(sc.signed_distance(4.5, 1.0) < 0.0);
        assert!((sc.signed_distance(0.5, 3.0) - 0.5).abs() < 1e-12);
        // x 方向の光線は x=4 の壁に 3.5 で当たる
        assert!((sc.raycast(0.5, 0.5, 1.0, 0.0).unwrap() - 3.5).abs() < 1e-12);
        // 箱に当たる (1,1)-(2,2) へ y=1.5 の光線
        assert!((sc.raycast(0.5, 1.5, 1.0, 0.0).unwrap() - 0.5).abs() < 1e-12);
        // 円に当たる
        assert!((sc.raycast(2.0, 3.0, 1.0, 0.0).unwrap() - 0.9).abs() < 1e-12);
    }
}
