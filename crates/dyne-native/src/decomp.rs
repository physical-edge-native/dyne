//! Splitting a regular grid of points into rectangular blocks (patches), in any number of
//! dimensions.
//!
//! The types are generic over the dimension `D` so that the same decomposition, face numbering and
//! border routing serve the 2D milestone and the 3D target; solvers exist for `D = 2` and `D = 3`.
//!
//! # Conventions (shared with Dyne's 2D solvers)
//!
//! - Points are numbered row-major with axis 0 (x) fastest: in 2D, `k = j * nx + i`.
//! - Faces of a block are numbered `face = 2 * axis + side`, where `side` 0 is the low side and 1
//!   the high side. In 2D this gives Dyne's west = 0, east = 1, south = 2, north = 3
//!   (`dyne_wire::abi::field2d::FACE_*`); in 3D the z faces are 4 (low) and 5 (high).
//! - `n` points split into `p` blocks: every block gets `n / p` points and the first `n % p` blocks
//!   one more (the same rule as Dyne's simulators).
//! - Every border between two blocks (an *interface*) has a number that both blocks use as the
//!   sender id of the border messages. Interfaces normal to axis 0 are numbered first, then those
//!   normal to axis 1, and so on; within one axis they are numbered by the position of the lower
//!   block, axis 0 fastest. In 2D this reproduces the numbering of Dyne's `dyne-sim2d`.
//! - Blocks that touch only along an edge or at a corner (*diagonal neighbours*, needed by solvers
//!   that exchange more than one layer of border values) get link numbers after all interfaces:
//!   for each block in order, for each offset towards a neighbour whose first non-zero component is
//!   +1 (offsets ordered as base-3 numbers with digits −1, 0, +1, axis 0 fastest), the next number.

use std::fmt;

/// One side of a block that touches a neighbouring block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Interface {
    /// Interface number, used as the sender id in the border messages crossing it.
    pub id: u32,
    /// Index of the neighbouring block in [`Decomposition::blocks`].
    pub neighbour: usize,
}

/// A block that touches another one only along an edge or at a corner (a *diagonal neighbour*).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Diagonal<const D: usize> {
    /// Offset of the neighbour in block coordinates: each component is −1, 0 or +1, and at least two
    /// are non-zero.
    pub offset: [isize; D],
    /// Link number, used as the sender id in the border messages between the two blocks. It is
    /// shared by both blocks and differs from every interface number.
    pub id: u32,
    /// Index of the neighbouring block in [`Decomposition::blocks`].
    pub neighbour: usize,
}

/// One block (patch) of the grid.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Block<const D: usize> {
    /// Position in [`Decomposition::blocks`] (row-major over block coordinates, axis 0 fastest).
    pub index: usize,
    /// Block coordinates (which block along each axis).
    pub coords: [usize; D],
    /// Global index of the block's first point along each axis.
    pub origin: [usize; D],
    /// Number of points along each axis.
    pub size: [usize; D],
    /// For each face (`2 * axis + side`): the interface, or `None` on the outer boundary.
    pub faces: Vec<Option<Interface>>,
    /// The diagonal neighbours (edges and corners), ordered by offset as base-3 numbers with digits
    /// −1, 0, +1 and axis 0 fastest. Only solvers that exchange more than one layer use them.
    pub diagonals: Vec<Diagonal<D>>,
}

impl<const D: usize> Block<D> {
    /// Number of points in the block.
    pub fn len(&self) -> usize {
        self.size.iter().product()
    }

    /// `true` if the block has no points (never the case for a valid decomposition).
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Number of faces that touch a neighbouring block.
    pub fn interface_count(&self) -> usize {
        self.faces.iter().filter(|f| f.is_some()).count()
    }

    /// The face (`2 * axis + side`) that carries the interface `id`, if any.
    pub fn face_of_interface(&self, id: u32) -> Option<usize> {
        self.faces.iter().position(|f| matches!(f, Some(i) if i.id == id))
    }
}

/// Error from [`Decomposition::new`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecompError(pub String);

impl fmt::Display for DecompError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for DecompError {}

/// A grid of `points` split into `parts` blocks along each axis.
#[derive(Clone, Debug)]
pub struct Decomposition<const D: usize> {
    /// Number of grid points along each axis.
    pub points: [usize; D],
    /// Number of blocks along each axis.
    pub parts: [usize; D],
    /// The blocks, row-major over block coordinates (axis 0 fastest).
    pub blocks: Vec<Block<D>>,
}

/// Splits `n` points into `p` consecutive runs: `(first index, count)` for each run.
/// The first `n % p` runs get one point more than the others.
pub fn split(n: usize, p: usize) -> Vec<(usize, usize)> {
    let (base, extra) = (n / p, n % p);
    let mut out = Vec::with_capacity(p);
    let mut start = 0;
    for i in 0..p {
        let count = base + usize::from(i < extra);
        out.push((start, count));
        start += count;
    }
    out
}

/// Row-major linear index of `coords` in a box of `dims` (axis 0 fastest).
fn linear<const D: usize>(coords: &[usize; D], dims: &[usize; D]) -> usize {
    let mut k = 0;
    for a in (0..D).rev() {
        k = k * dims[a] + coords[a];
    }
    k
}

/// All offsets in `{−1, 0, +1}^D` with at least two non-zero components, ordered as base-3 numbers
/// (digits −1, 0, +1, axis 0 fastest).
fn diagonal_offsets<const D: usize>() -> Vec<[isize; D]> {
    let mut out = Vec::new();
    for code in 0..3usize.pow(D as u32) {
        let mut o = [0isize; D];
        let mut c = code;
        for x in o.iter_mut() {
            *x = (c % 3) as isize - 1;
            c /= 3;
        }
        if o.iter().filter(|&&x| x != 0).count() >= 2 {
            out.push(o);
        }
    }
    out
}

/// Inverse of [`linear`].
fn unlinear<const D: usize>(mut k: usize, dims: &[usize; D]) -> [usize; D] {
    let mut c = [0; D];
    for a in 0..D {
        c[a] = k % dims[a];
        k /= dims[a];
    }
    c
}

impl<const D: usize> Decomposition<D> {
    /// Splits a grid of `points` into `parts` blocks along each axis. Every block must get at least
    /// `min_points` points along every axis (solvers have a minimum, e.g. 3 for Dyne's wave solver).
    pub fn new(points: [usize; D], parts: [usize; D], min_points: usize) -> Result<Self, DecompError> {
        for a in 0..D {
            if parts[a] == 0 {
                return Err(DecompError(format!("axis {a}: number of blocks must be at least 1")));
            }
            if points[a] / parts[a] < min_points.max(1) {
                return Err(DecompError(format!(
                    "axis {a}: {} points in {} blocks leaves fewer than {} points per block",
                    points[a], parts[a], min_points.max(1)
                )));
            }
        }
        let runs: Vec<Vec<(usize, usize)>> = (0..D).map(|a| split(points[a], parts[a])).collect();

        // Interface numbering: per axis, a box of (parts with one fewer along that axis).
        let mut iface_dims = [[0usize; D]; D];
        let mut iface_offset = [0usize; D];
        let mut offset = 0;
        for a in 0..D {
            let mut dims = parts;
            dims[a] -= 1;
            iface_dims[a] = dims;
            iface_offset[a] = offset;
            offset += dims.iter().product::<usize>();
        }
        let iface_id = |a: usize, lower: &[usize; D]| -> u32 {
            (iface_offset[a] + linear(lower, &iface_dims[a])) as u32
        };

        let count: usize = parts.iter().product();
        let mut blocks = Vec::with_capacity(count);
        for index in 0..count {
            let coords = unlinear(index, &parts);
            let mut origin = [0; D];
            let mut size = [0; D];
            for a in 0..D {
                (origin[a], size[a]) = runs[a][coords[a]];
            }
            let mut faces = vec![None; 2 * D];
            for a in 0..D {
                if coords[a] > 0 {
                    let mut lower = coords;
                    lower[a] -= 1;
                    faces[2 * a] = Some(Interface { id: iface_id(a, &lower), neighbour: linear(&lower, &parts) });
                }
                if coords[a] + 1 < parts[a] {
                    let mut upper = coords;
                    upper[a] += 1;
                    faces[2 * a + 1] =
                        Some(Interface { id: iface_id(a, &coords), neighbour: linear(&upper, &parts) });
                }
            }
            blocks.push(Block { index, coords, origin, size, faces, diagonals: Vec::new() });
        }

        // Diagonal links, numbered after the interfaces (see the module documentation).
        let offsets = diagonal_offsets::<D>();
        let neighbour_of = |coords: &[usize; D], o: &[isize; D]| -> Option<usize> {
            let mut c = [0usize; D];
            for a in 0..D {
                let x = coords[a] as isize + o[a];
                if x < 0 || x >= parts[a] as isize {
                    return None;
                }
                c[a] = x as usize;
            }
            Some(linear(&c, &parts))
        };
        let mut ids = std::collections::BTreeMap::new();
        let mut next = offset as u32;
        for b in &blocks {
            for o in &offsets {
                let positive = o.iter().find(|&&x| x != 0) == Some(&1);
                if let (true, Some(nb)) = (positive, neighbour_of(&b.coords, o)) {
                    ids.insert((b.index.min(nb), b.index.max(nb)), next);
                    next += 1;
                }
            }
        }
        for b in blocks.iter_mut() {
            for o in &offsets {
                if let Some(nb) = neighbour_of(&b.coords, o) {
                    let id = ids[&(b.index.min(nb), b.index.max(nb))];
                    b.diagonals.push(Diagonal { offset: *o, id, neighbour: nb });
                }
            }
        }
        Ok(Decomposition { points, parts, blocks })
    }

    /// The whole grid as one block (the single-patch reference).
    pub fn whole(points: [usize; D], min_points: usize) -> Result<Self, DecompError> {
        Self::new(points, [1; D], min_points)
    }

    /// Total number of grid points.
    pub fn len(&self) -> usize {
        self.points.iter().product()
    }

    /// `true` if the grid has no points.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Assembles the global field (row-major, axis 0 fastest) from one field per block
    /// (each row-major over the block, axis 0 fastest), in the order of [`Self::blocks`].
    pub fn gather(&self, fields: &[Vec<f64>]) -> Vec<f64> {
        assert_eq!(fields.len(), self.blocks.len(), "one field per block");
        let mut out = vec![0.0; self.len()];
        for (b, f) in self.blocks.iter().zip(fields) {
            assert_eq!(f.len(), b.len(), "field of block {} has the wrong length", b.index);
            // Copy rows along axis 0 (contiguous in both layouts).
            let rows = b.len() / b.size[0];
            let mut row_dims = b.size;
            row_dims[0] = 1;
            for r in 0..rows {
                let mut local = unlinear(r, &row_dims);
                let src = r * b.size[0];
                for a in 0..D {
                    local[a] += b.origin[a];
                }
                let dst = linear(&local, &self.points);
                out[dst..dst + b.size[0]].copy_from_slice(&f[src..src + b.size[0]]);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_gives_extra_points_to_the_first_runs() {
        assert_eq!(split(10, 3), vec![(0, 4), (4, 3), (7, 3)]);
        assert_eq!(split(9, 3), vec![(0, 3), (3, 3), (6, 3)]);
    }

    /// The 2D interface numbering is the one of Dyne's dyne-sim2d
    /// (vertical interfaces `by*(px-1)+bx`, then horizontal ones `(px-1)*py + by*px + bx`).
    #[test]
    fn interface_ids_match_dyne_sim2d_in_2d() {
        let (px, py) = (3, 2);
        let d = Decomposition::<2>::new([40, 33], [px, py], 3).unwrap();
        let v_count = (px - 1) * py;
        for b in &d.blocks {
            let [bx, by] = b.coords;
            let node = by * px + bx;
            assert_eq!(b.index, node);
            let w = (bx > 0).then(|| Interface { id: (by * (px - 1) + bx - 1) as u32, neighbour: node - 1 });
            let e = (bx + 1 < px).then(|| Interface { id: (by * (px - 1) + bx) as u32, neighbour: node + 1 });
            let s = (by > 0).then(|| Interface { id: (v_count + (by - 1) * px + bx) as u32, neighbour: node - px });
            let n = (by + 1 < py).then(|| Interface { id: (v_count + by * px + bx) as u32, neighbour: node + px });
            assert_eq!(b.faces, vec![w, e, s, n], "block {:?}", b.coords);
        }
    }

    /// Every interface is shared by exactly two blocks, on opposite faces, and the ids are 0..count.
    #[test]
    fn interfaces_pair_up_in_3d() {
        let d = Decomposition::<3>::new([10, 9, 8], [3, 2, 2], 2).unwrap();
        let mut seen = std::collections::BTreeMap::new();
        for b in &d.blocks {
            for (face, i) in b.faces.iter().enumerate() {
                if let Some(i) = i {
                    let other = &d.blocks[i.neighbour];
                    assert_eq!(other.faces[face ^ 1].map(|o| (o.id, o.neighbour)), Some((i.id, b.index)));
                    *seen.entry(i.id).or_insert(0) += 1;
                }
            }
        }
        // (3-1)*2*2 + 3*(2-1)*2 + 3*2*(2-1) = 8 + 6 + 6
        assert_eq!(seen.len(), 20);
        assert!(seen.keys().copied().eq(0..20));
        assert!(seen.values().all(|&c| c == 2));
    }

    /// Diagonal links pair up (same id on both blocks, opposite offsets), are numbered after the
    /// interfaces without gaps, and an inner block of a 3×3×3 split has all 20 of them.
    #[test]
    fn diagonal_links_pair_up() {
        let d = Decomposition::<3>::new([9, 9, 9], [3, 3, 3], 3).unwrap();
        let faces = 3 * (2 * 3 * 3);
        let mut seen = std::collections::BTreeMap::new();
        for b in &d.blocks {
            for g in &b.diagonals {
                let other = &d.blocks[g.neighbour];
                let back: [isize; 3] = std::array::from_fn(|a| -g.offset[a]);
                assert!(other.diagonals.iter().any(|h| h.id == g.id && h.offset == back && h.neighbour == b.index));
                for a in 0..3 {
                    assert_eq!(other.coords[a] as isize, b.coords[a] as isize + g.offset[a]);
                }
                *seen.entry(g.id).or_insert(0) += 1;
            }
        }
        assert_eq!(d.blocks[13].diagonals.len(), 20);
        // 3x3x3 blocks: edge links = 3 axes x 3 layers x 4 cells x 2 diagonals = 72;
        // corner links = 8 cells x 4 body diagonals = 32
        assert!(seen.keys().copied().eq(faces as u32..(faces + 72 + 32) as u32));
        assert!(seen.values().all(|&c| c == 2));
        let d2 = Decomposition::<2>::new([9, 9], [3, 3], 3).unwrap();
        let offsets: Vec<[isize; 2]> = d2.blocks[4].diagonals.iter().map(|g| g.offset).collect();
        assert_eq!(offsets, vec![[-1, -1], [1, -1], [-1, 1], [1, 1]]);
    }

    #[test]
    fn gather_puts_every_point_back() {
        let d = Decomposition::<3>::new([7, 5, 4], [2, 2, 3], 1).unwrap();
        let fields: Vec<Vec<f64>> = d
            .blocks
            .iter()
            .map(|b| {
                let mut f = Vec::new();
                for z in 0..b.size[2] {
                    for y in 0..b.size[1] {
                        for x in 0..b.size[0] {
                            let g = [b.origin[0] + x, b.origin[1] + y, b.origin[2] + z];
                            f.push(linear(&g, &d.points) as f64);
                        }
                    }
                }
                f
            })
            .collect();
        let g = d.gather(&fields);
        assert!(g.iter().enumerate().all(|(k, &v)| v == k as f64));
    }

    #[test]
    fn too_many_blocks_is_an_error() {
        assert!(Decomposition::<2>::new([8, 8], [3, 1], 3).is_err());
        assert!(Decomposition::<2>::new([9, 8], [3, 1], 3).is_ok());
    }
}
