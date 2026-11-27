//! The room shape as an MJCF file (MuJoCo's model format) and the free floor where a robot can start.
//!
//! The input is the same as for the OpenFOAM mesh with walls ([`crate::walls`]): the signed distance
//! field that Dyne's devices build, read from a `.vti` file (`dyne-sim2d --vti`) or a CSV shape file
//! (`scripts/twin_live.py`, `dyne-sim2d --field-csv`), and the same rule for solid grid points
//! ([`SolidRule`]: observed and `phi < 0`, or not observed when unobserved points count as solid).
//!
//! # Frame and units
//!
//! The frame is that of the USD export of the room (`scripts/twin_demo.py`, `scripts/twin_physics.py`):
//!
//! - x and y are the coordinates of the shape grid, unchanged: grid point `(i, j)` is at
//!   `(origin_x + i·spacing_x, origin_y + j·spacing_y)`, the `x`, `y` columns of the CSV shape or the
//!   origin and spacing of the `.vti` file. No offset or rotation is applied;
//! - z points up, and the top of the floor is at z = 0;
//! - lengths are in metres, angles in radians.
//!
//! # What is written
//!
//! - every solid grid point stands for the square cell of one spacing centred on it (as the boxes
//!   of unobserved points in the USD export); the cells are merged into rectangles row by row (the
//!   greedy rule of `twin_physics.greedy_rectangles`: start at the first uncovered cell of a row,
//!   extend to the right, then upwards while the whole width stays solid), and each rectangle is
//!   one static `box` geom from z = 0 to the room height. Observed solid points give the geoms
//!   `Solid_000`, `Solid_001`, ...; points that count as solid because no sensor observed them give
//!   `Unobserved_000`, ... (the USD export keeps them apart in the same way);
//! - four boxes `Bounds_000` to `Bounds_003` close the grid on the outside (one wall thickness,
//!   the room height), so a robot cannot leave the grid where the field has no solid cell at the edge;
//! - the floor is the geom `Floor`, a plane at z = 0 covering the grid plus a margin;
//! - the 2D shape carries no heights, so every box has the room height (furniture is a full-height
//!   obstacle).
//!
//! The free floor ([`SpawnArea`]) lists the grid points where a robot's centre can be placed: free
//! points of the largest connected free region (4-neighbours), whose distance to every solid cell
//! and to the outside of the grid is at least a given clearance. They are merged into rectangles in
//! the same way; every grid point inside a rectangle (edges included) is a spawn point.
//!
//! The output is deterministic: numbers are written with six decimals (1 µm, the resolution of a CSV
//! shape), so the same shape gives the same bytes on every platform, and a `.vti` file and the CSV
//! of the same run give the same files.

use crate::foam::{FoamError, Grid};
use crate::walls::{Shape, SolidRule};
use std::fmt::Write as _;

/// A rectangle of grid points, both ends included: `(i0, j0, i1, j1)`.
pub type Rect = (usize, usize, usize, usize);

/// Settings of the MJCF export.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MjcfOptions {
    /// Which grid points are solid.
    pub rule: SolidRule,
    /// Height of every box (walls and obstacles), in metres.
    pub height: f64,
    /// Thickness of the four boxes that close the grid on the outside, in metres.
    pub wall_thickness: f64,
    /// How far the floor plane reaches beyond the grid, in metres.
    pub floor_margin: f64,
    /// Smallest distance from a spawn point to a solid cell or to the outside of the grid, in metres.
    pub clearance: f64,
    /// Sliding friction of every geom.
    pub friction: f64,
}

impl Default for MjcfOptions {
    /// The defaults of the USD export: room height 2.4 m, outer walls 0.1 m thick, floor margin 0.5 m,
    /// friction 0.8; clearance 0 (every free point of the main region).
    fn default() -> MjcfOptions {
        MjcfOptions { rule: SolidRule::default(), height: 2.4, wall_thickness: 0.1, floor_margin: 0.5, clearance: 0.0, friction: 0.8 }
    }
}

/// The room as boxes, ready to be written.
#[derive(Debug, Clone, PartialEq)]
pub struct Room {
    /// The grid of the shape (2D).
    pub grid: Grid,
    /// Rectangles of observed solid points.
    pub solid: Vec<Rect>,
    /// Rectangles of points that count as solid because they were not observed.
    pub unobserved: Vec<Rect>,
    /// The free floor for spawn points.
    pub spawn: SpawnArea,
    /// The settings used.
    pub options: MjcfOptions,
}

/// The free floor where a robot's centre can be placed.
#[derive(Debug, Clone, PartialEq)]
pub struct SpawnArea {
    /// Number of spawn grid points.
    pub points: usize,
    /// Rectangles of spawn grid points (all points inside, edges included, are spawn points).
    pub rects: Vec<Rect>,
}

/// Merges the true entries of a row-major mask (`nx` per row) into rectangles that do not overlap:
/// row by row from the left, start at the first uncovered true entry, extend to the right, then
/// upwards while the whole width is true. The same rule as `greedy_rectangles` in
/// `scripts/twin_physics.py`, so the result is the same list in the same order.
pub fn greedy_rectangles(mask: &[bool], nx: usize) -> Vec<Rect> {
    if nx == 0 {
        return Vec::new();
    }
    let ny = mask.len() / nx;
    let mut m = mask.to_vec();
    let mut out = Vec::new();
    for j in 0..ny {
        let mut i = 0;
        while i < nx {
            if !m[i + nx * j] {
                i += 1;
                continue;
            }
            let mut i1 = i;
            while i1 + 1 < nx && m[i1 + 1 + nx * j] {
                i1 += 1;
            }
            let mut j1 = j;
            while j1 + 1 < ny && (i..=i1).all(|k| m[k + nx * (j1 + 1)]) {
                j1 += 1;
            }
            for jj in j..=j1 {
                for ii in i..=i1 {
                    m[ii + nx * jj] = false;
                }
            }
            out.push((i, j, i1, j1));
            i = i1 + 1;
        }
    }
    out
}

/// Marks the free points (`!solid`) outside the largest 4-connected free region; ties go to the
/// region that starts first in row-major order (as `polymesh::keep_largest_region`).
fn largest_free_region(solid: &[bool], nx: usize) -> Vec<bool> {
    let n = solid.len();
    let ny = n / nx;
    let mut label = vec![usize::MAX; n];
    let mut sizes: Vec<usize> = Vec::new();
    let mut stack = Vec::new();
    for start in 0..n {
        if solid[start] || label[start] != usize::MAX {
            continue;
        }
        let id = sizes.len();
        let mut size = 0;
        label[start] = id;
        stack.push(start);
        while let Some(p) = stack.pop() {
            size += 1;
            let (i, j) = (p % nx, p / nx);
            let mut near = [None; 4];
            if i > 0 {
                near[0] = Some(p - 1);
            }
            if i + 1 < nx {
                near[1] = Some(p + 1);
            }
            if j > 0 {
                near[2] = Some(p - nx);
            }
            if j + 1 < ny {
                near[3] = Some(p + nx);
            }
            for q in near.into_iter().flatten() {
                if !solid[q] && label[q] == usize::MAX {
                    label[q] = id;
                    stack.push(q);
                }
            }
        }
        sizes.push(size);
    }
    let best = (0..sizes.len()).max_by_key(|&r| (sizes[r], std::cmp::Reverse(r)));
    (0..n).map(|p| !solid[p] && Some(label[p]) == best).collect()
}

/// The spawn points: free points of the largest free region at least `clearance` away from every
/// solid cell (a square of one spacing around a solid point) and from the outside of the grid.
fn spawn_mask(grid: &Grid, solid: &[bool], clearance: f64) -> Vec<bool> {
    let [nx, ny, _] = grid.points;
    let [sx, sy, _] = grid.spacing;
    let mut ok = largest_free_region(solid, nx);
    if clearance <= 0.0 {
        return ok;
    }
    let c2 = clearance * clearance;
    // Gap between a point and the cell `d` cells away along one axis (0 for the point's own cell).
    let gap = |d: usize, s: f64| if d == 0 { 0.0 } else { d as f64 * s - 0.5 * s };
    let ri = (clearance / sx).ceil() as usize + 1;
    let rj = (clearance / sy).ceil() as usize + 1;
    for j in 0..ny {
        for i in 0..nx {
            let p = i + nx * j;
            if !ok[p] {
                continue;
            }
            // The outside of the grid starts half a spacing beyond the first and last points.
            let edge = [(i as f64 + 0.5) * sx, ((nx - 1 - i) as f64 + 0.5) * sx, (j as f64 + 0.5) * sy, ((ny - 1 - j) as f64 + 0.5) * sy];
            if edge.iter().any(|&e| e < clearance) {
                ok[p] = false;
                continue;
            }
            'near: for jj in j.saturating_sub(rj)..(j + rj + 1).min(ny) {
                for ii in i.saturating_sub(ri)..(i + ri + 1).min(nx) {
                    if solid[ii + nx * jj] {
                        let (dx, dy) = (gap(ii.abs_diff(i), sx), gap(jj.abs_diff(j), sy));
                        if dx * dx + dy * dy < c2 {
                            ok[p] = false;
                            break 'near;
                        }
                    }
                }
            }
        }
    }
    ok
}

impl Room {
    /// Builds the boxes and the spawn area of a 2D shape.
    pub fn build(shape: &Shape, options: MjcfOptions) -> Result<Room, FoamError> {
        let grid = shape.grid;
        grid.check()?;
        if !grid.is_2d() {
            return Err(FoamError(format!("the MJCF export needs a 2D shape (points {:?})", grid.points)));
        }
        for (name, v) in [("height", options.height), ("wall thickness", options.wall_thickness)] {
            if !(v.is_finite() && v > 0.0) {
                return Err(FoamError(format!("{name} must be finite and positive ({v})")));
            }
        }
        for (name, v) in [("floor margin", options.floor_margin), ("clearance", options.clearance), ("friction", options.friction)] {
            if !(v.is_finite() && v >= 0.0) {
                return Err(FoamError(format!("{name} must be finite and not negative ({v})")));
            }
        }
        let nx = grid.points[0];
        let solid = shape.solid(options.rule);
        let observed = |k: usize| shape.weight.as_ref().map_or(true, |w| w[k] > options.rule.min_weight);
        let seen: Vec<bool> = (0..solid.len()).map(|k| solid[k] && observed(k)).collect();
        let unseen: Vec<bool> = (0..solid.len()).map(|k| solid[k] && !observed(k)).collect();
        let spawn = spawn_mask(&grid, &solid, options.clearance);
        let spawn = SpawnArea { points: spawn.iter().filter(|&&x| x).count(), rects: greedy_rectangles(&spawn, nx) };
        Ok(Room { grid, solid: greedy_rectangles(&seen, nx), unobserved: greedy_rectangles(&unseen, nx), spawn, options })
    }

    /// The extent of the grid's cells: `[x_min, y_min, x_max, y_max]` (half a spacing beyond the
    /// first and last points).
    pub fn cell_extent(&self) -> [f64; 4] {
        let v = self.grid.vertices();
        [v[0][0], v[0][1], v[2][0], v[2][1]]
    }

    /// The extent of the grid's points: `[x_min, y_min, x_max, y_max]`.
    pub fn point_extent(&self) -> [f64; 4] {
        let g = &self.grid;
        [g.origin[0], g.origin[1], g.origin[0] + (g.points[0] - 1) as f64 * g.spacing[0], g.origin[1] + (g.points[1] - 1) as f64 * g.spacing[1]]
    }

    /// A rectangle of cells in metres: `[x_min, y_min, x_max, y_max]`.
    fn cell_rect(&self, r: &Rect) -> [f64; 4] {
        let g = &self.grid;
        let x = |i: f64| g.origin[0] + i * g.spacing[0];
        let y = |j: f64| g.origin[1] + j * g.spacing[1];
        [x(r.0 as f64 - 0.5), y(r.1 as f64 - 0.5), x(r.2 as f64 + 0.5), y(r.3 as f64 + 0.5)]
    }

    /// A rectangle of points in metres: `[x_min, y_min, x_max, y_max]`.
    fn point_rect(&self, r: &Rect) -> [f64; 4] {
        let g = &self.grid;
        [g.origin[0] + r.0 as f64 * g.spacing[0], g.origin[1] + r.1 as f64 * g.spacing[1], g.origin[0] + r.2 as f64 * g.spacing[0], g.origin[1] + r.3 as f64 * g.spacing[1]]
    }

    /// The four boxes that close the grid on the outside, as `[x_min, y_min, x_max, y_max]`:
    /// west and east over the full height of the grid plus the corners, then south and north.
    fn bounds(&self) -> [[f64; 4]; 4] {
        let [x0, y0, x1, y1] = self.cell_extent();
        let t = self.options.wall_thickness;
        [[x0 - t, y0 - t, x0, y1 + t], [x1, y0 - t, x1 + t, y1 + t], [x0, y0 - t, x1, y0], [x0, y1, x1, y1 + t]]
    }

    /// Number of geoms in the MJCF file (floor, four outer boxes, the solid and unobserved boxes).
    pub fn geom_count(&self) -> usize {
        1 + 4 + self.solid.len() + self.unobserved.len()
    }

    /// The MJCF file.
    pub fn to_mjcf(&self) -> String {
        let o = &self.options;
        let [cx0, cy0, cx1, cy1] = self.cell_extent();
        let m = o.floor_margin + o.wall_thickness;
        let (fx, fy) = ((cx0 + cx1) / 2.0, (cy0 + cy1) / 2.0);
        let mut s = String::new();
        s.push_str("<mujoco model=\"dyne_room\">\n");
        s.push_str("  <!-- Room shape computed by Dyne, written by dyne-mjcf (crates/dyne-export/src/mjcf.rs).\n");
        s.push_str("       Frame of the USD export: x, y = coordinates of the shape grid, z up, floor top at z = 0; metres.\n");
        s.push_str("       Static box geoms: Solid_* (observed, phi < 0), Unobserved_* (not observed), Bounds_* (outside the grid). -->\n");
        s.push_str("  <compiler angle=\"radian\"/>\n");
        s.push_str("  <option gravity=\"0 0 -9.81\" timestep=\"0.002\"/>\n");
        let _ = writeln!(s, "  <default><geom friction=\"{} 0.005 0.0001\" condim=\"3\"/></default>", f6(o.friction));
        s.push_str("  <custom>\n");
        let [px0, py0, px1, py1] = self.point_extent();
        let _ = writeln!(s, "    <numeric name=\"dyne_grid_extent\" data=\"{} {} {} {}\"/>", f6(px0), f6(py0), f6(px1), f6(py1));
        let _ = writeln!(s, "    <numeric name=\"dyne_grid_spacing\" data=\"{} {}\"/>", f6(self.grid.spacing[0]), f6(self.grid.spacing[1]));
        let _ = writeln!(s, "    <numeric name=\"dyne_room_height\" data=\"{}\"/>", f6(o.height));
        s.push_str("  </custom>\n");
        s.push_str("  <worldbody>\n");
        let _ = writeln!(s, "    <light pos=\"{} {} 3\" dir=\"0 0 -1\"/>", f6(fx), f6(fy));
        let _ = writeln!(
            s,
            "    <geom name=\"Floor\" type=\"plane\" pos=\"{} {} 0\" size=\"{} {} 0.1\" rgba=\"0.3 0.31 0.33 1\"/>",
            f6(fx),
            f6(fy),
            f6((cx1 - cx0) / 2.0 + m),
            f6((cy1 - cy0) / 2.0 + m)
        );
        let mut boxes = |name: &str, rgba: &str, rects: &mut dyn Iterator<Item = [f64; 4]>| {
            for (k, [x0, y0, x1, y1]) in rects.enumerate() {
                let _ = writeln!(
                    s,
                    "    <geom name=\"{name}_{k:03}\" type=\"box\" pos=\"{} {} {}\" size=\"{} {} {}\" rgba=\"{rgba}\"/>",
                    f6((x0 + x1) / 2.0),
                    f6((y0 + y1) / 2.0),
                    f6(o.height / 2.0),
                    f6((x1 - x0) / 2.0),
                    f6((y1 - y0) / 2.0),
                    f6(o.height / 2.0)
                );
            }
        };
        boxes("Bounds", "0.8 0.82 0.86 1", &mut self.bounds().into_iter());
        boxes("Solid", "0.8 0.82 0.86 1", &mut self.solid.iter().map(|r| self.cell_rect(r)));
        boxes("Unobserved", "0.45 0.45 0.5 0.5", &mut self.unobserved.iter().map(|r| self.cell_rect(r)));
        s.push_str("  </worldbody>\n</mujoco>\n");
        s
    }

    /// The free floor as JSON (layout in the module documentation and on the docs site).
    pub fn spawn_json(&self) -> String {
        let g = &self.grid;
        let o = &self.options;
        let mut s = String::new();
        s.push_str("{\n");
        s.push_str("  \"format\": \"dyne-room-spawn 1\",\n");
        s.push_str("  \"frame\": \"x, y = coordinates of the shape grid (as the USD export), z up, floor top at z = 0\",\n");
        s.push_str("  \"units\": \"m\",\n");
        let _ = writeln!(
            s,
            "  \"grid\": {{\"points\": [{}, {}], \"origin\": [{}, {}], \"spacing\": [{}, {}]}},",
            g.points[0],
            g.points[1],
            f6(g.origin[0]),
            f6(g.origin[1]),
            f6(g.spacing[0]),
            f6(g.spacing[1])
        );
        let _ = writeln!(s, "  \"clearance\": {},", f6(o.clearance));
        let _ = writeln!(s, "  \"spawn_points\": {},", self.spawn.points);
        let _ = writeln!(s, "  \"spawn_area_m2\": {},", f6(self.spawn.points as f64 * g.spacing[0] * g.spacing[1]));
        s.push_str("  \"rectangles\": [");
        for (k, r) in self.spawn.rects.iter().enumerate() {
            let [x0, y0, x1, y1] = self.point_rect(r);
            let _ = write!(s, "{}\n    [{}, {}, {}, {}]", if k == 0 { "" } else { "," }, f6(x0), f6(y0), f6(x1), f6(y1));
        }
        s.push_str(if self.spawn.rects.is_empty() { "]\n" } else { "\n  ]\n" });
        s.push_str("}\n");
        s
    }
}

/// Six decimals, without a negative zero.
fn f6(v: f64) -> String {
    let s = format!("{:.6}", v);
    if s.starts_with('-') && s[1..].bytes().all(|b| b == b'0' || b == b'.') {
        s[1..].to_string()
    } else {
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shape(nx: usize, ny: usize, solid: &[(usize, usize)], unseen: &[(usize, usize)]) -> Shape {
        let n = nx * ny;
        let mut phi = vec![0.1; n];
        let mut w = vec![1.0; n];
        for &(i, j) in solid {
            phi[i + nx * j] = -0.1;
        }
        for &(i, j) in unseen {
            w[i + nx * j] = 0.0;
        }
        Shape { grid: Grid::new_2d([nx, ny], [1.0, -2.0], [0.5, 0.25]), phi, weight: Some(w) }
    }

    #[test]
    fn rectangles_cover_exactly() {
        // An L shape and a single cell: rows first, then upwards.
        let nx = 5;
        let mut m = vec![false; nx * 4];
        for (i, j) in [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1), (0, 2), (4, 3)] {
            m[i + nx * j] = true;
        }
        let r = greedy_rectangles(&m, nx);
        assert_eq!(r, vec![(0, 0, 2, 1), (0, 2, 0, 2), (4, 3, 4, 3)]);
        let mut covered = vec![false; m.len()];
        for &(i0, j0, i1, j1) in &r {
            for j in j0..=j1 {
                for i in i0..=i1 {
                    assert!(!covered[i + nx * j], "overlap");
                    covered[i + nx * j] = true;
                }
            }
        }
        assert_eq!(covered, m);
    }

    #[test]
    fn boxes_frame_and_spawn() {
        // 6 x 4 grid, origin (1, -2), spacing 0.5 x 0.25; a 2-cell wall, one unobserved point, and a
        // free pocket at (5, 3) cut off by solid points.
        let sh = shape(6, 4, &[(2, 1), (2, 2), (4, 3)], &[(0, 0)]);
        let mut sh = sh;
        sh.phi[5 + 6 * 2] = -0.1; // (5, 2) solid: (5, 3) is cut off
        let room = Room::build(&sh, MjcfOptions::default()).unwrap();
        assert_eq!(room.solid, vec![(2, 1, 2, 2), (5, 2, 5, 2), (4, 3, 4, 3)]);
        assert_eq!(room.unobserved, vec![(0, 0, 0, 0)]);
        assert_eq!(room.geom_count(), 1 + 4 + 3 + 1);
        // Cell (2, 1)..(2, 2): x 2.0 ± 0.25, y from -2 + 0.75 = -1.25 ... -1.5 + 0.125.
        let xml = room.to_mjcf();
        assert!(xml.contains("<geom name=\"Solid_000\" type=\"box\" pos=\"2.000000 -1.625000 1.200000\" size=\"0.250000 0.250000 1.200000\""), "{xml}");
        assert!(xml.contains("<geom name=\"Unobserved_000\" type=\"box\" pos=\"1.000000 -2.000000 1.200000\" size=\"0.250000 0.125000 1.200000\""), "{xml}");
        // The outer boxes touch the cell extent (0.75 .. 3.75, -2.125 .. -1.125).
        assert_eq!(room.cell_extent(), [0.75, -2.125, 3.75, -1.125]);
        assert!(xml.contains("<geom name=\"Bounds_000\" type=\"box\" pos=\"0.700000 -1.625000 1.200000\" size=\"0.050000 0.600000 1.200000\""), "{xml}");
        assert_eq!(xml.matches("<geom name=").count(), room.geom_count());
        // Spawn: 24 points - 4 solid - 1 unobserved - 1 cut off.
        assert_eq!(room.spawn.points, 18);
        let json = room.spawn_json();
        assert!(json.contains("\"spawn_area_m2\": 2.250000,"), "{json}");
        // No spawn point lies in a solid cell or in the cut-off pocket.
        for &(i0, j0, i1, j1) in &room.spawn.rects {
            for j in j0..=j1 {
                for i in i0..=i1 {
                    assert!(sh.phi[i + 6 * j] > 0.0 && (i, j) != (0, 0) && (i, j) != (5, 3));
                }
            }
        }
        // Deterministic.
        assert_eq!(Room::build(&sh, MjcfOptions::default()).unwrap().to_mjcf(), xml);
    }

    #[test]
    fn clearance_keeps_away_from_cells_and_edges() {
        let sh = shape(9, 9, &[(4, 4)], &[]);
        let room = |c: f64| Room::build(&sh, MjcfOptions { clearance: c, ..MjcfOptions::default() }).unwrap().spawn;
        assert_eq!(room(0.0).points, 80);
        // Spacing 0.5 x 0.25. Clearance 0.3: the edge rule removes i = 0, 8 (0.25 < 0.3) and j = 0, 8
        // (0.125) and j = 1, 7 (0.375 ≥ 0.3 stays); the cell (4, 4) spans x ±0.25 and y ±0.125.
        let s = room(0.3);
        let mut want = 0;
        for j in 1..8 {
            for i in 1..8 {
                let dx = if i == 4 { 0.0 } else { (i as f64 - 4.0).abs() * 0.5 - 0.25 };
                let dy = if j == 4 { 0.0 } else { (j as f64 - 4.0).abs() * 0.25 - 0.125 };
                if dx * dx + dy * dy >= 0.09 {
                    want += 1;
                }
            }
        }
        assert_eq!(s.points, want);
        assert!(room(10.0).rects.is_empty());
        assert!(Room::build(&sh, MjcfOptions { height: 0.0, ..MjcfOptions::default() }).is_err());
    }

    #[test]
    fn no_negative_zero() {
        assert_eq!(f6(-0.0), "0.000000");
        assert_eq!(f6(-1e-9), "0.000000");
        assert_eq!(f6(-0.5), "-0.500000");
    }
}
