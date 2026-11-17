//! An initial field on the grid from a few point sensors (e.g. the temperature sensors of the
//! devices), by inverse-distance weighting.
//!
//! # Method
//!
//! At every grid point `p`, with readings `v_i` at positions `s_i` and distances `d_i = |p − s_i|`
//! (in the plane for a 2D grid):
//!
//! ```text
//! T(p) = Σ v_i / d_i²  /  Σ 1 / d_i²          (Shepard's inverse-distance weighting, power 2)
//! T(p) = v_i  if p is exactly at s_i
//! ```
//!
//! # What the result means, and its error
//!
//! The field is an **initial guess built from a few readings, not a measurement of the field**.
//! What holds exactly:
//!
//! - every value lies between the smallest and the largest reading (the result is clamped to that range
//!   so rounding cannot leave it), so the guess never invents a hotter or colder spot than measured;
//! - a grid point exactly at a sensor gets that reading; equal readings give a uniform field;
//! - far from all sensors the value tends to the mean of the readings (no extrapolated trend).
//!
//! What cannot be stated: the error between sensors. It depends on the real field, which the few
//! readings do not resolve; the only bound is the spread of the readings (largest − smallest) plus
//! the error of the sensors themselves (about ±0.5 °C for the DHT20, about ±1 °C for the BME280
//! over 0–65 °C, from the data sheets). Straight-line distances are used, so walls and furniture do not
//! block the weighting. The field near a sensor has a flat spot (zero gradient), a known property of
//! the method.
//!
//! The order of the sums is the order of the readings in the file, so the result is the same, bit
//! for bit, on every machine.
//!
//! # Input file
//!
//! A CSV file with a header line naming the columns: `x`, `y` (metres, the frame of Dyne's grid),
//! `z` (needed for a 3D grid, ignored in 2D) and `T` (the reading; the unit is the user's, and must
//! match the values given for fixed faces). Other columns (e.g. `node`) are ignored; empty lines and
//! lines starting with `#` are skipped.
//!
//! ```text
//! # 2026-11-20, readings at 14:00
//! node,x,y,T
//! 3,0.55,1.05,21.4
//! 7,2.15,0.75,22.0
//! ```

use crate::foam::{FoamError, Grid};

/// One reading of a point sensor.
#[derive(Debug, Clone, PartialEq)]
pub struct Reading {
    /// Position (x, y, z) in metres; z is 0 when the file has no `z` column.
    pub pos: [f64; 3],
    /// The value read.
    pub value: f64,
}

/// Reads the sensor file (see the module documentation).
pub fn read_csv(text: &str) -> Result<Vec<Reading>, FoamError> {
    let bad = |m: String| Err(FoamError(m));
    let mut lines = text.lines().enumerate().filter(|(_, l)| !l.trim().is_empty() && !l.trim_start().starts_with('#'));
    let Some((_, head)) = lines.next() else { return bad("empty file".into()) };
    let names: Vec<&str> = head.split(',').map(str::trim).collect();
    let col = |n: &str| names.iter().position(|c| *c == n);
    let (Some(cx), Some(cy), Some(ct)) = (col("x"), col("y"), col("T")) else {
        return bad("the header must name the columns x, y and T (and z for a 3D grid)".into());
    };
    let cz = col("z");
    let mut out: Vec<Reading> = Vec::new();
    for (ln, line) in lines {
        let f: Vec<&str> = line.split(',').map(str::trim).collect();
        let real = |c: usize| {
            let s = f.get(c).copied().unwrap_or("");
            match s.parse::<f64>() {
                Ok(v) if v.is_finite() => Ok(v),
                _ => Err(FoamError(format!("line {}: not a finite number: {s:?}", ln + 1))),
            }
        };
        let r = Reading { pos: [real(cx)?, real(cy)?, cz.map(real).transpose()?.unwrap_or(0.0)], value: real(ct)? };
        if out.iter().any(|o| o.pos == r.pos) {
            return bad(format!("line {}: two readings at the same position {:?}", ln + 1, r.pos));
        }
        out.push(r);
    }
    if out.is_empty() {
        return bad("no readings".into());
    }
    Ok(out)
}

/// The inverse-distance weighted field on every grid point (x fastest; see the module documentation).
/// For a 2D grid the distances are taken in the x-y plane.
pub fn idw(grid: &Grid, readings: &[Reading]) -> Vec<f64> {
    assert!(!readings.is_empty(), "at least one reading");
    let lo = readings.iter().map(|r| r.value).fold(f64::INFINITY, f64::min);
    let hi = readings.iter().map(|r| r.value).fold(f64::NEG_INFINITY, f64::max);
    let axes = if grid.is_2d() { 2 } else { 3 };
    let [nx, ny, _] = grid.points;
    (0..grid.cells())
        .map(|k| {
            let at = [k % nx, (k / nx) % ny, k / (nx * ny)];
            let p: [f64; 3] = std::array::from_fn(|a| grid.origin[a] + at[a] as f64 * grid.spacing[a]);
            let (mut num, mut den) = (0.0, 0.0);
            for r in readings {
                let d2: f64 = (0..axes).map(|a| (p[a] - r.pos[a]) * (p[a] - r.pos[a])).sum();
                if d2 == 0.0 {
                    return r.value;
                }
                num += r.value / d2;
                den += 1.0 / d2;
            }
            (num / den).clamp(lo, hi)
        })
        .collect()
}

/// Index of the grid point nearest to `pos` (x fastest), or `None` if `pos` is outside the cells of
/// the grid (more than half a spacing beyond the outer points).
pub fn nearest_point(grid: &Grid, pos: [f64; 3]) -> Option<usize> {
    let axes = if grid.is_2d() { 2 } else { 3 };
    let mut k = 0;
    let mut stride = 1;
    for a in 0..axes {
        let i = ((pos[a] - grid.origin[a]) / grid.spacing[a]).round();
        if !(i >= 0.0 && i < grid.points[a] as f64) {
            return None;
        }
        k += i as usize * stride;
        stride *= grid.points[a];
    }
    Some(k)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(x: f64, y: f64, v: f64) -> Reading {
        Reading { pos: [x, y, 0.0], value: v }
    }

    #[test]
    fn exact_at_sensors_and_bounded() {
        let g = Grid::new_2d([11, 9], [0.0, 0.0], [0.1, 0.1]);
        // The first three sensors sit exactly on grid points (coordinates computed as the grid does).
        let at = |i: f64, j: f64| (i * 0.1, j * 0.1);
        let ((x0, y0), (x1, y1), (x2, y2)) = (at(2.0, 1.0), at(9.0, 3.0), at(5.0, 7.0));
        let rs = [r(x0, y0, 21.4), r(x1, y1, 23.0), r(x2, y2, 19.5), r(0.15, 0.75, 22.2), r(1.3, 1.2, 20.0)];
        let t = idw(&g, &rs);
        for (k, want) in [(2 + 11, 21.4), (9 + 3 * 11, 23.0), (5 + 7 * 11, 19.5)] {
            assert_eq!(t[k].to_bits(), f64::to_bits(want));
        }
        assert!(t.iter().all(|&v| (19.5..=23.0).contains(&v)));
        assert_eq!(nearest_point(&g, [0.91, 0.29, 0.0]), Some(9 + 3 * 11));
        assert_eq!(nearest_point(&g, [1.3, 1.2, 0.0]), None); // outside the grid
        assert_eq!(nearest_point(&g, [-0.04, 0.0, 0.0]), Some(0));
    }

    #[test]
    fn uniform_and_symmetric() {
        let g = Grid::new_2d([5, 3], [0.0, 0.0], [0.5, 0.5]);
        assert!(idw(&g, &[r(0.3, 0.2, 20.5), r(1.7, 0.9, 20.5)]).iter().all(|&v| v == 20.5));
        // Two sensors at the same distance from a point: the mean (here exact in floating point).
        let t = idw(&g, &[r(0.0, 0.5, 20.0), r(2.0, 0.5, 22.0)]);
        assert_eq!(t[2 + 5], 21.0);
        // A 3D grid uses z, a 2D grid ignores it.
        let g3 = Grid::new_3d([3, 1, 3], [0.0; 3], [1.0; 3]);
        let rs = [Reading { pos: [1.0, 0.0, 0.0], value: 1.0 }, Reading { pos: [1.0, 0.0, 2.0], value: 3.0 }];
        let t = idw(&g3, &rs);
        assert_eq!((t[1], t[1 + 3], t[1 + 6]), (1.0, 2.0, 3.0));
    }

    #[test]
    fn csv() {
        let text = "# readings\nnode,x,y,T\n3,0.55,1.05,21.4\n\n7, 2.15 ,0.75,22\n";
        let rs = read_csv(text).unwrap();
        assert_eq!(rs, vec![r(0.55, 1.05, 21.4), r(2.15, 0.75, 22.0)]);
        assert!(read_csv("x,y,value\n0,0,1\n").is_err());
        assert!(read_csv("x,y,T\n0,0,1\n0,0,2\n").is_err()); // same position twice
        assert!(read_csv("x,y,T\n0,0,nan\n").is_err());
        assert!(read_csv("x,y,T\n").is_err());
        assert_eq!(read_csv("x,y,z,T\n1,2,3,4\n").unwrap()[0].pos, [1.0, 2.0, 3.0]);
    }
}
