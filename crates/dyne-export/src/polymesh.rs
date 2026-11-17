//! An OpenFOAM mesh (`constant/polyMesh`) written cell by cell: the cells of a regular grid minus the
//! cells inside walls and obstacles.
//!
//! [`crate::foam`] writes a case without walls as one `blockMesh` block. When some grid points are
//! *solid* (inside a wall or a piece of furniture), those cells are left out and the faces between a
//! remaining cell and a solid one form the patch `walls` (type `wall`). Such a mesh is no longer one
//! block, so it is written directly in OpenFOAM's own mesh format instead of being described for
//! `blockMesh`: the five files `points`, `faces`, `owner`, `neighbour` and `boundary` of
//! `constant/polyMesh` (the files that `blockMesh` itself writes). The choice is recorded in
//! `docs/design/2026-11-18-foam-walls-and-sensor-field.md`.
//!
//! # Layout
//!
//! - Cells: the fluid (not solid) grid points in Dyne's order (x fastest, then y, then z). Cell `c` is
//!   the `c`-th fluid point; the cell is the box of size `spacing` centred on the point, as in the
//!   one-block case, so the vertices are the same (`origin + (i − 1/2)·spacing` along each axis).
//! - Faces: first the internal faces, ordered by owner cell and then by neighbour cell (OpenFOAM's
//!   "upper triangular" order), each pointing from the owner (the lower cell number) to the neighbour;
//!   then the boundary faces, patch by patch, each pointing out of its cell.
//! - Patches: `west`, `east`, `south`, `north` (and `bottom`, `top` in 3D) for the faces on the sides of
//!   the grid, as in the one-block case; `walls` for the faces between a fluid and a solid cell; in 2D,
//!   `frontAndBack` (type `empty`) for the front and back of every cell. A patch may have no faces.
//! - Points: only the vertices used by some face, in the order of their grid index.
//!
//! The same code writes 2D (one cell thick) and 3D meshes; nothing depends on the grid being 2D.
//!
//! [`PolyMesh::check`] verifies, without OpenFOAM, the properties that OpenFOAM's `checkMesh` reports
//! as errors for this kind of mesh (closed cells, faces pointing out of their owner, upper triangular
//! order, every point used, contiguous patches) and counts the connected regions. It is not
//! `checkMesh`; a run of `checkMesh` on a machine with OpenFOAM is still the reference.

use crate::foam::{header, num_str, FoamError, Grid, EMPTY_PATCH};
use std::fmt::Write as _;

/// Patch name of the faces between fluid cells and solid cells (type `wall`).
pub const WALL_PATCH: &str = "walls";

/// A patch of the mesh: a contiguous range of boundary faces.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Patch {
    /// Patch name (`west`, ..., `walls`, `frontAndBack`).
    pub name: &'static str,
    /// OpenFOAM patch type: `patch`, `wall` or `empty`.
    pub kind: &'static str,
    /// Index of the first face of the patch.
    pub start: usize,
    /// Number of faces.
    pub len: usize,
}

/// A mesh in OpenFOAM's face-based format, built from a grid and a solid mask by [`PolyMesh::build`].
#[derive(Debug, Clone, PartialEq)]
pub struct PolyMesh {
    /// Vertex coordinates.
    pub points: Vec<[f64; 3]>,
    /// Faces as four point indices; the normal (right-hand rule) points from the owner outwards.
    pub faces: Vec<[usize; 4]>,
    /// Owner cell of every face.
    pub owner: Vec<usize>,
    /// Neighbour cell of every internal face (the internal faces are the first `neighbour.len()`).
    pub neighbour: Vec<usize>,
    /// Patches of the boundary faces, in order.
    pub patches: Vec<Patch>,
    /// Grid point index (x fastest) of every cell.
    pub cells: Vec<usize>,
}

/// What [`PolyMesh::check`] found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeshReport {
    /// Number of cells.
    pub cells: usize,
    /// Number of faces.
    pub faces: usize,
    /// Number of internal faces.
    pub internal_faces: usize,
    /// Number of points.
    pub points: usize,
    /// Number of connected regions of cells (1 for a mesh in one piece).
    pub regions: usize,
}

impl PolyMesh {
    /// Builds the mesh of the fluid cells of `grid`: `solid[k]` is true for grid point `k` inside a
    /// wall or obstacle (one entry per grid point, x fastest). At least one point must be fluid.
    pub fn build(grid: &Grid, solid: &[bool]) -> Result<PolyMesh, FoamError> {
        grid.check()?;
        let n = grid.cells();
        if solid.len() != n {
            return Err(FoamError(format!("the solid mask has {} entries, the grid has {} points", solid.len(), n)));
        }
        let [nx, ny, _] = grid.points;
        let stride = [1, nx, nx * ny];
        let ijk = |p: usize| [p % nx, (p / nx) % ny, p / (nx * ny)];
        let cells: Vec<usize> = (0..n).filter(|&p| !solid[p]).collect();
        if cells.is_empty() {
            return Err(FoamError("every grid point is solid: no cell left".into()));
        }
        let mut compact = vec![usize::MAX; n];
        for (c, &p) in cells.iter().enumerate() {
            compact[p] = c;
        }
        // Vertex (i, j, k) of the grid of cell corners, (nx + 1) × (ny + 1) × (nz + 1).
        let vid = |v: [usize; 3]| v[0] + (nx + 1) * (v[1] + (ny + 1) * v[2]);
        // Face of the cell at `c` on side `s` (0 = minus, 1 = plus) of axis `a`, pointing outwards:
        // with b = a+1, c = a+2 (cyclic), the corners base, +b, +b+c, +c turn around +a.
        let quad = |c: [usize; 3], a: usize, s: usize| -> [usize; 4] {
            let mut base = c;
            base[a] += s;
            let (b, e) = ((a + 1) % 3, (a + 2) % 3);
            let v = |db: usize, de: usize| {
                let mut p = base;
                p[b] += db;
                p[e] += de;
                vid(p)
            };
            let q = [v(0, 0), v(1, 0), v(1, 1), v(0, 1)];
            if s == 1 {
                q
            } else {
                [q[0], q[3], q[2], q[1]]
            }
        };
        let dims = if grid.is_2d() { 2 } else { 3 };

        let mut faces = Vec::new();
        let mut owner = Vec::new();
        let mut neighbour = Vec::new();
        // Internal faces: per cell, the neighbours along +x, +y, +z have increasing cell numbers.
        for (c, &p) in cells.iter().enumerate() {
            let at = ijk(p);
            for a in 0..dims {
                if at[a] + 1 < grid.points[a] && !solid[p + stride[a]] {
                    faces.push(quad(at, a, 1));
                    owner.push(c);
                    neighbour.push(compact[p + stride[a]]);
                }
            }
        }
        let mut patches = Vec::new();
        let mut patch = |name: &'static str, kind: &'static str, faces: &mut Vec<[usize; 4]>, owner: &mut Vec<usize>, list: Vec<(usize, [usize; 4])>| {
            patches.push(Patch { name, kind, start: faces.len(), len: list.len() });
            for (c, q) in list {
                faces.push(q);
                owner.push(c);
            }
        };
        // Sides of the grid.
        for (f, name) in grid.face_names().iter().enumerate() {
            let (a, s) = (f / 2, f % 2);
            let edge = if s == 0 { 0 } else { grid.points[a] - 1 };
            let list = cells.iter().enumerate().filter(|(_, &p)| ijk(p)[a] == edge).map(|(c, &p)| (c, quad(ijk(p), a, s))).collect();
            patch(name, "patch", &mut faces, &mut owner, list);
        }
        // Walls: faces towards a solid neighbour, per cell in the order -x, +x, -y, +y, -z, +z.
        let mut list = Vec::new();
        for (c, &p) in cells.iter().enumerate() {
            let at = ijk(p);
            for a in 0..dims {
                if at[a] > 0 && solid[p - stride[a]] {
                    list.push((c, quad(at, a, 0)));
                }
                if at[a] + 1 < grid.points[a] && solid[p + stride[a]] {
                    list.push((c, quad(at, a, 1)));
                }
            }
        }
        patch(WALL_PATCH, "wall", &mut faces, &mut owner, list);
        if grid.is_2d() {
            let list = cells.iter().enumerate().flat_map(|(c, &p)| [(c, quad(ijk(p), 2, 0)), (c, quad(ijk(p), 2, 1))]).collect();
            patch(EMPTY_PATCH, "empty", &mut faces, &mut owner, list);
        }

        // Keep only the used vertices, numbered in the order of their grid index.
        let nv = (nx + 1) * (ny + 1) * (grid.points[2] + 1);
        let mut used = vec![usize::MAX; nv];
        for q in &faces {
            for &v in q {
                used[v] = 0;
            }
        }
        let mut points = Vec::new();
        for (v, slot) in used.iter_mut().enumerate() {
            if *slot == 0 {
                *slot = points.len();
                let at = [v % (nx + 1), (v / (nx + 1)) % (ny + 1), v / ((nx + 1) * (ny + 1))];
                points.push(std::array::from_fn(|a| grid.origin[a] + (at[a] as f64 - 0.5) * grid.spacing[a]));
            }
        }
        for q in &mut faces {
            for v in q.iter_mut() {
                *v = used[*v];
            }
        }
        Ok(PolyMesh { points, faces, owner, neighbour, patches, cells })
    }

    /// Number of cells.
    pub fn n_cells(&self) -> usize {
        self.cells.len()
    }

    /// The patch with this name.
    pub fn patch(&self, name: &str) -> Option<&Patch> {
        self.patches.iter().find(|p| p.name == name)
    }

    /// Checks the mesh without OpenFOAM: indices in range, every point used, internal faces in upper
    /// triangular order with owner < neighbour, patches contiguous and covering the boundary faces,
    /// every cell closed (each edge of a cell shared by exactly two of its faces, in opposite
    /// directions), every face of non-zero area pointing out of its owner (and into its neighbour).
    /// Returns the counts and the number of connected regions.
    pub fn check(&self) -> Result<MeshReport, FoamError> {
        let bad = |m: String| Err(FoamError(format!("mesh check: {m}")));
        let (nc, nf, ni, np) = (self.cells.len(), self.faces.len(), self.neighbour.len(), self.points.len());
        if self.owner.len() != nf || ni > nf {
            return bad(format!("{} owners and {} neighbours for {} faces", self.owner.len(), ni, nf));
        }
        let mut point_used = vec![false; np];
        for (f, q) in self.faces.iter().enumerate() {
            for &v in q {
                if v >= np {
                    return bad(format!("face {f}: point {v} out of range"));
                }
                point_used[v] = true;
            }
            let mut s = *q;
            s.sort_unstable();
            if s.windows(2).any(|w| w[0] == w[1]) {
                return bad(format!("face {f}: repeated point"));
            }
        }
        if let Some(v) = point_used.iter().position(|u| !u) {
            return bad(format!("point {v} is not used by any face"));
        }
        if let Some(f) = self.owner.iter().position(|&c| c >= nc) {
            return bad(format!("face {f}: owner out of range"));
        }
        for f in 0..ni {
            if self.neighbour[f] >= nc || self.neighbour[f] <= self.owner[f] {
                return bad(format!("internal face {f}: neighbour {} not above owner {}", self.neighbour[f], self.owner[f]));
            }
            if f > 0 && (self.owner[f], self.neighbour[f]) <= (self.owner[f - 1], self.neighbour[f - 1]) {
                return bad(format!("internal face {f}: not in upper triangular order"));
            }
        }
        let mut next = ni;
        for p in &self.patches {
            if p.start != next {
                return bad(format!("patch {}: starts at face {}, expected {next}", p.name, p.start));
            }
            next += p.len;
        }
        if next != nf {
            return bad(format!("patches cover faces up to {next}, the mesh has {nf}"));
        }

        // Faces of every cell, oriented outwards.
        let mut cell_faces: Vec<Vec<[usize; 4]>> = vec![Vec::new(); nc];
        for (f, q) in self.faces.iter().enumerate() {
            cell_faces[self.owner[f]].push(*q);
            if f < ni {
                cell_faces[self.neighbour[f]].push([q[0], q[3], q[2], q[1]]);
            }
        }
        let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let cross = |a: [f64; 3], b: [f64; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        let centre = |q: &[usize]| -> [f64; 3] {
            let mut c = [0.0; 3];
            for &v in q {
                for a in 0..3 {
                    c[a] += self.points[v][a];
                }
            }
            c.map(|x| x / q.len() as f64)
        };
        let mut cell_centre = Vec::with_capacity(nc);
        for (c, fs) in cell_faces.iter().enumerate() {
            let mut edges: Vec<(usize, usize)> = fs.iter().flat_map(|q| (0..4).map(move |k| (q[k], q[(k + 1) % 4]))).collect();
            edges.sort_unstable();
            let closed = edges.windows(2).all(|w| w[0] != w[1]) && edges.iter().all(|&(a, b)| edges.binary_search(&(b, a)).is_ok());
            if !closed {
                return bad(format!("cell {c}: not closed or faces not consistently oriented ({} faces)", fs.len()));
            }
            let mut vs: Vec<usize> = fs.iter().flatten().copied().collect();
            vs.sort_unstable();
            vs.dedup();
            cell_centre.push(centre(&vs));
        }
        for (f, q) in self.faces.iter().enumerate() {
            let p = q.map(|v| self.points[v]);
            let normal = cross(sub(p[2], p[0]), sub(p[3], p[1]));
            if !(dot(normal, normal) > 0.0) {
                return bad(format!("face {f}: zero area"));
            }
            let fc = centre(q);
            if !(dot(normal, sub(fc, cell_centre[self.owner[f]])) > 0.0) {
                return bad(format!("face {f}: does not point out of its owner cell {}", self.owner[f]));
            }
            if f < ni && !(dot(normal, sub(cell_centre[self.neighbour[f]], fc)) > 0.0) {
                return bad(format!("face {f}: does not point into its neighbour cell {}", self.neighbour[f]));
            }
        }

        // Connected regions through internal faces.
        let mut parent: Vec<usize> = (0..nc).collect();
        fn find(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }
        for f in 0..ni {
            let (a, b) = (find(&mut parent, self.owner[f]), find(&mut parent, self.neighbour[f]));
            if a != b {
                parent[a.max(b)] = a.min(b);
            }
        }
        let regions = (0..nc).filter(|&c| find(&mut parent, c) == c).count();
        Ok(MeshReport { cells: nc, faces: nf, internal_faces: ni, points: np, regions })
    }

    /// The five files of `constant/polyMesh` as (path relative to the case directory, contents).
    pub fn files(&self) -> Vec<(&'static str, String)> {
        let note = format!(
            "nPoints:{}  nCells:{}  nFaces:{}  nInternalFaces:{}",
            self.points.len(),
            self.cells.len(),
            self.faces.len(),
            self.neighbour.len()
        );
        let head = |class: &str, object: &str, with_note: bool| {
            let h = header(class, "constant/polyMesh", object);
            if with_note {
                h.replacen(&format!("    object      {object};\n"), &format!("    note        \"{note}\";\n    object      {object};\n"), 1)
            } else {
                h
            }
        };
        let mut points = head("vectorField", "points", false);
        let _ = write!(points, "{}\n(\n", self.points.len());
        for p in &self.points {
            let _ = writeln!(points, "({} {} {})", num_str(p[0]), num_str(p[1]), num_str(p[2]));
        }
        points.push_str(")\n");
        let mut faces = head("faceList", "faces", false);
        let _ = write!(faces, "{}\n(\n", self.faces.len());
        for q in &self.faces {
            let _ = writeln!(faces, "4({} {} {} {})", q[0], q[1], q[2], q[3]);
        }
        faces.push_str(")\n");
        let labels = |object: &str, list: &[usize]| {
            let mut s = head("labelList", object, true);
            let _ = write!(s, "{}\n(\n", list.len());
            for v in list {
                let _ = writeln!(s, "{v}");
            }
            s.push_str(")\n");
            s
        };
        let mut boundary = head("polyBoundaryMesh", "boundary", false);
        let _ = write!(boundary, "{}\n(\n", self.patches.len());
        for p in &self.patches {
            let group = if p.kind == "patch" { String::new() } else { format!("        inGroups        1({});\n", p.kind) };
            let _ = write!(
                boundary,
                "    {}\n    {{\n        type            {};\n{group}        nFaces          {};\n        startFace       {};\n    }}\n",
                p.name, p.kind, p.len, p.start
            );
        }
        boundary.push_str(")\n");
        vec![
            ("constant/polyMesh/points", points),
            ("constant/polyMesh/faces", faces),
            ("constant/polyMesh/owner", labels("owner", &self.owner)),
            ("constant/polyMesh/neighbour", labels("neighbour", &self.neighbour)),
            ("constant/polyMesh/boundary", boundary),
        ]
    }
}

/// Turns into solid every fluid cell outside the largest connected fluid region (cells connected
/// through faces; on a tie, the region with the lowest grid index). Small pockets of fluid enclosed
/// by walls, e.g. inside furniture whose inside the sensors could not see, would otherwise be separate
/// pieces of the mesh. Returns the number of cells turned solid.
pub fn keep_largest_region(grid: &Grid, solid: &mut [bool]) -> usize {
    let n = grid.cells();
    assert_eq!(solid.len(), n);
    let [nx, ny, nz] = grid.points;
    let stride = [1, nx, nx * ny];
    let dims = [nx, ny, nz];
    let mut label = vec![usize::MAX; n];
    let mut sizes = Vec::new();
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
            let at = [p % nx, (p / nx) % ny, p / (nx * ny)];
            for a in 0..3 {
                let mut try_push = |q: usize| {
                    if !solid[q] && label[q] == usize::MAX {
                        label[q] = id;
                        stack.push(q);
                    }
                };
                if at[a] > 0 {
                    try_push(p - stride[a]);
                }
                if at[a] + 1 < dims[a] {
                    try_push(p + stride[a]);
                }
            }
        }
        sizes.push(size);
    }
    let Some(best) = (0..sizes.len()).max_by_key(|&r| (sizes[r], std::cmp::Reverse(r))) else { return 0 };
    let mut changed = 0;
    for p in 0..n {
        if !solid[p] && label[p] != best {
            solid[p] = true;
            changed += 1;
        }
    }
    changed
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mask(s: &str) -> Vec<bool> {
        // Rows from the top (largest y) down, '#' = solid.
        let rows: Vec<&str> = s.lines().map(str::trim).filter(|l| !l.is_empty()).collect();
        rows.iter().rev().flat_map(|r| r.chars().map(|c| c == '#')).collect()
    }

    #[test]
    fn no_solid_is_the_block() {
        let g = Grid::new_2d([3, 2], [0.0, 0.0], [0.5, 0.25]);
        let m = PolyMesh::build(&g, &[false; 6]).unwrap();
        let r = m.check().unwrap();
        // 3x2 cells: internal faces 2*2 (x) + 3*1 (y) = 7; boundary 2+2+3+3 = 10 sides, 12 front/back.
        assert_eq!(r, MeshReport { cells: 6, faces: 29, internal_faces: 7, points: 24, regions: 1 });
        assert_eq!(m.patch(WALL_PATCH).unwrap().len, 0);
        assert_eq!(m.patch(EMPTY_PATCH).unwrap().len, 12);
        // The corners are the vertices of the one-block case.
        let v = g.vertices();
        assert_eq!(m.points[0], v[0]);
        assert_eq!(*m.points.last().unwrap(), v[6]);
    }

    #[test]
    fn walls_2d() {
        let g = Grid::new_2d([4, 3], [0.0, 0.0], [0.1, 0.1]);
        let s = mask(
            "....
             .#..
             ....",
        );
        let m = PolyMesh::build(&g, &s).unwrap();
        let r = m.check().unwrap();
        assert_eq!(r.cells, 11);
        assert_eq!(r.regions, 1);
        assert_eq!(m.patch(WALL_PATCH).unwrap().len, 4);
        assert_eq!(m.cells, vec![0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11]);
        // Every cell has six faces: 6 * 11 = 2 * internal + boundary.
        assert_eq!(6 * r.cells, 2 * r.internal_faces + (r.faces - r.internal_faces));
    }

    #[test]
    fn walls_3d() {
        let g = Grid::new_3d([4, 3, 3], [-1.0, 0.5, 2.0], [0.1, 0.2, 0.3]);
        let mut s = vec![false; 36];
        for k in [13, 14, 17, 25] {
            s[k] = true;
        }
        let m = PolyMesh::build(&g, &s).unwrap();
        let r = m.check().unwrap();
        assert_eq!(r.cells, 32);
        assert_eq!(r.regions, 1);
        assert!(m.patch(EMPTY_PATCH).is_none());
        assert_eq!(m.patches.iter().map(|p| p.name).collect::<Vec<_>>(), ["west", "east", "south", "north", "bottom", "top", "walls"]);
        // Wall faces = faces of the solid cells that touch a fluid cell.
        let ijk = |p: usize| [p % 4, (p / 4) % 3, p / 12];
        let mut want = 0;
        for p in (0..36).filter(|&p| s[p]) {
            let at = ijk(p);
            for (a, n) in [(0, 4), (1, 3), (2, 3)] {
                let st = [1, 4, 12][a];
                if at[a] > 0 && !s[p - st] {
                    want += 1;
                }
                if at[a] + 1 < n && !s[p + st] {
                    want += 1;
                }
            }
        }
        assert_eq!(m.patch(WALL_PATCH).unwrap().len, want);
        assert_eq!(6 * r.cells, 2 * r.internal_faces + (r.faces - r.internal_faces));
    }

    #[test]
    fn regions_and_largest() {
        let g = Grid::new_2d([5, 3], [0.0, 0.0], [1.0, 1.0]);
        let mut s = mask(
            "..#..
             ..#..
             ..#.#",
        );
        let m = PolyMesh::build(&g, &s).unwrap();
        assert_eq!(m.check().unwrap().regions, 2);
        // Left region 6 cells, right region 5: the right one becomes solid.
        assert_eq!(keep_largest_region(&g, &mut s), 5);
        let m = PolyMesh::build(&g, &s).unwrap();
        assert_eq!(m.check().unwrap(), MeshReport { cells: 6, faces: 7 + 10 + 12, internal_faces: 7, points: 24, regions: 1 });
        assert!(PolyMesh::build(&g, &[true; 15]).is_err());
        assert!(PolyMesh::build(&g, &[false; 14]).is_err());
    }

    #[test]
    fn check_catches_errors() {
        let g = Grid::new_2d([3, 3], [0.0, 0.0], [1.0, 1.0]);
        let s = mask(
            "...
             .#.
             ...",
        );
        let m = PolyMesh::build(&g, &s).unwrap();
        m.check().unwrap();
        let mut b = m.clone();
        let q = b.faces[0];
        b.faces[0] = [q[0], q[3], q[2], q[1]]; // flipped internal face
        assert!(b.check().is_err());
        let mut b = m.clone();
        let f = b.neighbour.len();
        let q = b.faces[f];
        b.faces[f] = [q[0], q[3], q[2], q[1]]; // flipped boundary face
        assert!(b.check().is_err());
        let mut b = m.clone();
        b.neighbour.swap(0, 1);
        b.faces.swap(0, 1);
        b.owner.swap(0, 1);
        assert!(b.check().is_err()); // order
        let mut b = m.clone();
        b.points.push([9.0, 9.0, 9.0]);
        assert!(b.check().is_err()); // unused point
        let mut b = m.clone();
        b.patches[0].len += 1;
        assert!(b.check().is_err());
    }
}
