//! The room shape as solid grid points: which cells of an OpenFOAM case lie inside walls and
//! obstacles.
//!
//! Dyne's devices build the shape of a room from distance sensors as a *signed distance field* on a
//! grid (the shape grid solver, `crates/sdf2d-grid-actor`): at every grid point, `phi` is the distance
//! to the nearest surface, positive on the free side and negative inside walls and furniture, and `w`
//! is the weight of the observations behind it (`w = 0`: not observed yet). The same arrays feed the
//! USD export of the room (`scripts/twin_demo.py`, `scripts/twin_physics.py`), and the rule here is
//! the one used there:
//!
//! - a grid point is **solid** if it was observed (`w > min_weight`) and `phi < 0`;
//! - a point that was not observed (`w ≤ min_weight`) is solid by default, as in the USD export
//!   (`Unobserved`; its option `--unobserved-solid on`), or free with [`SolidRule::unobserved_solid`]
//!   set to false;
//! - without a weight array, every point counts as observed.
//!
//! The shape is read from either file that Dyne writes:
//!
//! - a `.vti` file (`dyne-sim2d --vti`, arrays `phi`, `d`, `w`, `truth`): the grid is exact;
//! - a CSV file with the columns `gx,gy,x,y,phi,...,w` (`scripts/twin_live.py` writes
//!   `frames/<time>/shape.csv` this way from the devices' data; `dyne-sim2d --field-csv` too). The
//!   coordinates there have six decimals, so the origin and spacing are rounded to 1 µm, and every
//!   row's coordinates must agree with the grid within 1e-6 m (the rounding of six decimals is at most
//!   5e-7 m).
//!
//! The array names are parameters, so the true shape of a simulated room (`truth` in a `.vti` file of
//! `dyne-sim2d`) can be used instead of the estimate.

use crate::foam::{FoamError, Grid};
use crate::vti::ImageData;
use std::path::Path;

/// Tolerance on coordinates read from a CSV shape file, in metres (see the module documentation).
pub const CSV_TOLERANCE: f64 = 1e-6;

/// A signed distance field on a grid, with the weights of the observations if known.
#[derive(Debug, Clone, PartialEq)]
pub struct Shape {
    /// The grid of the field.
    pub grid: Grid,
    /// Signed distance at every grid point (x fastest): negative inside walls and obstacles.
    pub phi: Vec<f64>,
    /// Weight of the observations at every grid point (0 = not observed), if the file has it.
    pub weight: Option<Vec<f64>>,
}

/// How grid points become solid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidRule {
    /// Points with weight `≤ min_weight` count as not observed.
    pub min_weight: f64,
    /// Not observed points are solid (the default, as in the USD export) or free.
    pub unobserved_solid: bool,
}

impl Default for SolidRule {
    fn default() -> SolidRule {
        SolidRule { min_weight: 0.0, unobserved_solid: true }
    }
}

impl Shape {
    /// Reads a shape from a `.vti` file or, for any other extension, a CSV file. `phi_name` and
    /// `weight_name` are the names of the arrays (columns); a missing weight array means "all observed".
    pub fn read(path: impl AsRef<Path>, phi_name: &str, weight_name: &str) -> Result<Shape, FoamError> {
        let path = path.as_ref();
        let ctx = |e: String| FoamError(format!("{}: {e}", path.display()));
        if path.extension().is_some_and(|e| e.eq_ignore_ascii_case("vti")) {
            let d = ImageData::read_vti(path).map_err(|e| ctx(e.to_string()))?;
            Shape::from_image(&d, phi_name, weight_name).map_err(|e| ctx(e.0))
        } else {
            let text = std::fs::read_to_string(path).map_err(|e| ctx(e.to_string()))?;
            Shape::from_csv(&text, phi_name, weight_name).map_err(|e| ctx(e.0))
        }
    }

    /// The shape in the arrays of a `.vti` file.
    pub fn from_image(d: &ImageData, phi_name: &str, weight_name: &str) -> Result<Shape, FoamError> {
        let phi = d.array(phi_name).ok_or_else(|| FoamError(format!("no array {phi_name}")))?.values.clone();
        let weight = d.array(weight_name).map(|a| a.values.clone());
        Ok(Shape { grid: Grid { points: d.points, origin: d.origin, spacing: d.spacing }, phi, weight })
    }

    /// The shape in a CSV file with a header line naming the columns `gx`, `gy`, `x`, `y`, `phi_name`
    /// and optionally `weight_name` (other columns are ignored). Every grid point must appear once.
    pub fn from_csv(text: &str, phi_name: &str, weight_name: &str) -> Result<Shape, FoamError> {
        let bad = |m: String| Err(FoamError(m));
        let mut lines = text.lines().enumerate().filter(|(_, l)| !l.trim().is_empty() && !l.trim_start().starts_with('#'));
        let Some((_, head)) = lines.next() else { return bad("empty file".into()) };
        let names: Vec<&str> = head.split(',').map(str::trim).collect();
        let col = |n: &str| names.iter().position(|c| *c == n);
        let (Some(cgx), Some(cgy), Some(cx), Some(cy), Some(cphi)) = (col("gx"), col("gy"), col("x"), col("y"), col(phi_name)) else {
            return bad(format!("the header must name the columns gx, gy, x, y and {phi_name}"));
        };
        let cw = col(weight_name);
        let mut rows = Vec::new();
        for (ln, line) in lines {
            let f: Vec<&str> = line.split(',').map(str::trim).collect();
            let get = |c: usize| f.get(c).copied().unwrap_or("");
            let int = |c: usize| get(c).parse::<usize>().map_err(|_| FoamError(format!("line {}: not a grid index: {:?}", ln + 1, get(c))));
            let real = |c: usize| get(c).parse::<f64>().map_err(|_| FoamError(format!("line {}: not a number: {:?}", ln + 1, get(c))));
            rows.push((int(cgx)?, int(cgy)?, real(cx)?, real(cy)?, real(cphi)?, cw.map(real).transpose()?));
        }
        let nx = rows.iter().map(|r| r.0).max().map_or(0, |m| m + 1);
        let ny = rows.iter().map(|r| r.1).max().map_or(0, |m| m + 1);
        if nx < 2 || ny < 2 {
            return bad("the grid needs at least 2 points along x and y".into());
        }
        if rows.len() != nx * ny {
            return bad(format!("{} rows for a {nx}x{ny} grid", rows.len()));
        }
        let n = nx * ny;
        let mut seen = vec![false; n];
        let (mut phi, mut w) = (vec![0.0; n], vec![0.0; n]);
        let mut xy = vec![[0.0; 2]; n];
        for r in &rows {
            let k = r.0 + nx * r.1;
            if std::mem::replace(&mut seen[k], true) {
                return bad(format!("grid point ({}, {}) appears twice", r.0, r.1));
            }
            phi[k] = r.4;
            w[k] = r.5.unwrap_or(0.0);
            xy[k] = [r.2, r.3];
        }
        let snap = |v: f64| (v * 1e6).round() / 1e6;
        let origin = [snap(xy[0][0]), snap(xy[0][1])];
        let spacing = [snap((xy[nx - 1][0] - xy[0][0]) / (nx - 1) as f64), snap((xy[nx * (ny - 1)][1] - xy[0][1]) / (ny - 1) as f64)];
        let grid = Grid::new_2d([nx, ny], origin, spacing);
        grid.check()?;
        for (k, p) in xy.iter().enumerate() {
            let want = [origin[0] + (k % nx) as f64 * spacing[0], origin[1] + (k / nx) as f64 * spacing[1]];
            if (p[0] - want[0]).abs() > CSV_TOLERANCE || (p[1] - want[1]).abs() > CSV_TOLERANCE {
                return bad(format!("grid point ({}, {}) at ({}, {}), expected ({}, {}) on a regular grid", k % nx, k / nx, p[0], p[1], want[0], want[1]));
            }
        }
        Ok(Shape { grid, phi, weight: cw.map(|_| w) })
    }

    /// Solid flag of every grid point (see the module documentation).
    pub fn solid(&self, rule: SolidRule) -> Vec<bool> {
        (0..self.phi.len())
            .map(|k| {
                let observed = self.weight.as_ref().map_or(true, |w| w[k] > rule.min_weight);
                if observed {
                    self.phi[k] < 0.0
                } else {
                    rule.unobserved_solid
                }
            })
            .collect()
    }
}

/// True if two grids have the same point counts and their origins and spacings agree within `tol`
/// (absolute, in the grid's length unit).
pub fn same_grid(a: &Grid, b: &Grid, tol: f64) -> bool {
    a.points == b.points && (0..3).all(|k| (a.origin[k] - b.origin[k]).abs() <= tol && (a.spacing[k] - b.spacing[k]).abs() <= tol)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn csv_like_twin_live() {
        // The layout of scripts/twin_live.py: six decimals, an empty truth column.
        let mut s = String::from("gx,gy,x,y,phi,d,w,truth\n");
        for j in 0..3 {
            for i in 0..4 {
                let phi = if (i, j) == (1, 1) { -0.02 } else { 0.1 };
                let w = if (i, j) == (3, 2) { 0.0 } else { 1.0 };
                s.push_str(&format!("{},{},{:.6},{:.6},{:.7e},{:.7e},{:.7e},\n", i, j, 1.2 + i as f64 * 0.05, -0.3 + j as f64 * 0.05, phi, 0.0, w));
            }
        }
        let sh = Shape::from_csv(&s, "phi", "w").unwrap();
        assert_eq!(sh.grid, Grid::new_2d([4, 3], [1.2, -0.3], [0.05, 0.05]));
        let solid = sh.solid(SolidRule::default());
        let at = |i: usize, j: usize| solid[i + 4 * j];
        assert!(at(1, 1) && at(3, 2) && !at(0, 0));
        assert_eq!(solid.iter().filter(|&&x| x).count(), 2);
        let free = sh.solid(SolidRule { min_weight: 0.0, unobserved_solid: false });
        assert_eq!(free.iter().filter(|&&x| x).count(), 1);
        // Without the weight column every point is observed.
        let sh = Shape::from_csv(&s.replace("phi,d,w", "phi,d,weight"), "phi", "w").unwrap();
        assert!(sh.weight.is_none());
        assert_eq!(sh.solid(SolidRule::default()).iter().filter(|&&x| x).count(), 1);
    }

    #[test]
    fn csv_errors() {
        let ok = "gx,gy,x,y,phi,w\n0,0,0,0,1,1\n1,0,0.1,0,1,1\n0,1,0,0.1,1,1\n1,1,0.1,0.1,-1,1\n";
        assert!(Shape::from_csv(ok, "phi", "w").is_ok());
        assert!(Shape::from_csv(&ok.replace("1,1,0.1,0.1", "0,1,0.1,0.1"), "phi", "w").is_err()); // twice
        assert!(Shape::from_csv(&ok.replace("1,1,0.1,0.1", "1,1,0.1,0.13"), "phi", "w").is_err()); // irregular
        assert!(Shape::from_csv(&ok.replace("gx,", "i,"), "phi", "w").is_err()); // header
        assert!(Shape::from_csv("gx,gy,x,y,phi\n0,0,0,0,1\n", "phi", "w").is_err()); // too small
    }

    #[test]
    fn vti_shape() {
        let mut d = ImageData::new_2d([3, 2], [0.0, 0.0], [0.5, 0.5]);
        d.add("phi", vec![1.0, -0.0, -1.0, 0.0, 2.0, -3.0]).add("w", vec![1.0, 1.0, 1.0, 0.0, 1.0, 1.0]);
        let sh = Shape::from_image(&d, "phi", "w").unwrap();
        // -0.0 is not below zero: a point exactly on the surface is free.
        assert_eq!(sh.solid(SolidRule::default()), vec![false, false, true, true, false, true]);
        assert!(Shape::from_image(&d, "truth", "w").is_err());
        assert!(same_grid(&sh.grid, &Grid::new_2d([3, 2], [0.0, 5e-7], [0.5, 0.5]), CSV_TOLERANCE));
        assert!(!same_grid(&sh.grid, &Grid::new_2d([3, 2], [0.0, 0.0], [0.25, 0.5]), CSV_TOLERANCE));
    }
}
