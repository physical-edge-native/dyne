//! Border exchange with `k` layers ("wide borders", also called wide halos or ghost zones) for the
//! native wave solvers, in any number of dimensions.
//!
//! With `k = 1` a patch keeps one layer of ghost points and exchanges one layer of border values
//! with its face neighbours every step (the scheme of Dyne's WebAssembly solvers). With `k > 1` a
//! patch keeps `k` layers and exchanges once every `k` steps; in between it also advances the parts
//! of the neighbours' layers it holds (redundant computation), and the region it can advance
//! correctly shrinks by one layer per step, because the five- and seven-point stencils reach one
//! point per step.
//!
//! # Why every point still gets the same bits
//!
//! A point in the overlap is advanced with exactly the same operations, in the same order, as by
//! the patch that owns it, from the same input bits; outer boundaries are applied to the overlap in
//! the same face order as on the whole grid. Only how often values are exchanged changes.
//!
//! # What is exchanged
//!
//! The leapfrog scheme needs two time levels. At an exchange at step `s` (a multiple of `k`), a
//! patch needs the current level `u^s` on `k` layers and the previous level `u^{s−1}` on `k − 1`
//! layers of each neighbour (to advance the overlap at the first step after the exchange).
//! After `k` steps of shrinking, the patch holds `k − 1` layers of neither level itself, so both
//! are sent. One message per neighbour carries, as one IFACE_BAND list of `f64` values (Dyne's wire
//! format v2, one value per entry): the box of `u^s` values, then the box of `u^{s−1}` values, each
//! box listed with axis 0 fastest. For `k = 1` the second box is empty and the message is exactly
//! the one-layer message of Dyne's solvers.
//!
//! # Edges and corners
//!
//! With `k > 1`, the value of a point near a corner of the patch after `k` steps depends on values
//! of the blocks that touch the patch only at an edge or a corner (the dependence cone of the
//! stencil is a diamond of radius `k`). Those values are sent directly by those blocks: each patch
//! exchanges with its face neighbours *and* its diagonal neighbours (up to 8 in 2D, 26 in 3D), in a
//! single round per exchange. The alternative, exchanging along one axis after the other so that
//! corner values travel through the face neighbours, needs only face neighbours but one round of
//! waiting per axis (2 or 3 per exchange instead of 1), which would cancel most of the saving for
//! small `k`.
//!
//! # Limits
//!
//! A patch must have at least `k` points along every axis on which it has a neighbour, so that every
//! layer it needs comes from an adjacent block; wider borders are rejected with an error.

use crate::decomp::Block;
use crate::patch::PatchError;
use crate::wave2d::Outer;
use dyne_wire as wire;

/// What one face of a patch is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Face {
    Interface(u32),
    Fixed(f64),
    ZeroGradient,
    Absorbing,
}

/// A neighbour that values are exchanged with: link id and offset in block coordinates.
#[derive(Clone, Copy, Debug)]
struct Link<const D: usize> {
    id: u32,
    offset: [isize; D],
}

/// A box of points in local coordinates: `[lo, hi)` per axis.
pub(crate) type Range<const D: usize> = [(isize, isize); D];

/// Calls `g` for every point of `bx`, axis 0 fastest.
pub(crate) fn for_box<const D: usize>(bx: &Range<D>, mut g: impl FnMut([isize; D])) {
    if bx.iter().any(|&(lo, hi)| hi <= lo) {
        return;
    }
    let mut p: [isize; D] = std::array::from_fn(|a| bx[a].0);
    loop {
        g(p);
        let mut a = 0;
        loop {
            p[a] += 1;
            if p[a] < bx[a].1 {
                break;
            }
            p[a] = bx[a].0;
            a += 1;
            if a == D {
                return;
            }
        }
    }
}

fn box_len<const D: usize>(bx: &Range<D>) -> usize {
    bx.iter().map(|&(lo, hi)| (hi - lo).max(0) as usize).product()
}

const AXIS: [&str; 3] = ["x", "y", "z"];

/// The storage layout, the faces, the links and the exchange schedule of one patch.
pub(crate) struct Window<const D: usize> {
    /// Points of the block along each axis.
    pub n: [usize; D],
    /// Number of layers exchanged (border width); one exchange every `k` steps.
    pub k: usize,
    /// Face kinds, `2·axis + side`.
    pub face: Vec<Face>,
    /// Stored points below local index 0 along each axis (`k − 1` overlap layers on an interface,
    /// plus one layer of ghost points).
    below: [usize; D],
    /// Stride of each axis in the storage.
    stride: [usize; D],
    /// Number of stored values.
    total: usize,
    links: Vec<Link<D>>,
    got: Vec<bool>,
    /// Steps done.
    pub step: u64,
    /// Scratch space for the values of one message as little-endian bytes.
    bytes: Vec<u8>,
}

impl<const D: usize> Window<D> {
    /// Sets up block `b` with outer conditions `outer` (per face) and border width `k`.
    pub fn new(b: &Block<D>, outer: &[Outer], k: usize) -> Result<Self, PatchError> {
        if k == 0 {
            return Err(PatchError("border width (halo) must be at least 1".into()));
        }
        let n = b.size;
        let face: Vec<Face> = (0..2 * D)
            .map(|f| match (b.faces[f], outer[f]) {
                (Some(i), _) => Face::Interface(i.id),
                (None, Outer::Fixed(v)) => Face::Fixed(v),
                (None, Outer::ZeroGradient) => Face::ZeroGradient,
                (None, Outer::Absorbing) => Face::Absorbing,
            })
            .collect();
        for a in 0..D {
            let split_axis = matches!(face[2 * a], Face::Interface(_)) || matches!(face[2 * a + 1], Face::Interface(_));
            if split_axis && n[a] < k {
                return Err(PatchError(format!(
                    "halo width {k} is larger than the patch: {} points along {}; every patch needs at least \
                     {k} points along each split axis (use a smaller halo width or fewer patches)",
                    n[a],
                    AXIS.get(a).copied().unwrap_or("an axis"),
                )));
            }
        }
        let extra = |f: usize| if matches!(face[f], Face::Interface(_)) { k - 1 } else { 0 };
        let below: [usize; D] = std::array::from_fn(|a| extra(2 * a) + 1);
        let dims: [usize; D] = std::array::from_fn(|a| n[a] + extra(2 * a) + extra(2 * a + 1) + 2);
        let mut stride = [0; D];
        let mut s = 1;
        for a in 0..D {
            stride[a] = s;
            s *= dims[a];
        }
        let mut links = Vec::new();
        for (f, kind) in face.iter().enumerate() {
            if let Face::Interface(id) = *kind {
                let mut offset = [0isize; D];
                offset[f / 2] = if f % 2 == 0 { -1 } else { 1 };
                links.push(Link { id, offset });
            }
        }
        if k > 1 {
            links.extend(b.diagonals.iter().map(|d| Link { id: d.id, offset: d.offset }));
        }
        let got = vec![false; links.len()];
        Ok(Window { n, k, face, below, stride, total: s, links, got, step: 0, bytes: Vec::new() })
    }

    /// Number of stored values (block, overlap layers and ghost points).
    pub fn total(&self) -> usize {
        self.total
    }

    /// Storage index of local point `p` (`0..n` is the block).
    #[inline]
    pub fn at(&self, p: [isize; D]) -> usize {
        let mut k = 0;
        for a in 0..D {
            k += (p[a] + self.below[a] as isize) as usize * self.stride[a];
        }
        k
    }

    /// Stride of axis `a` in the storage.
    pub fn stride(&self, a: usize) -> usize {
        self.stride[a]
    }

    /// The block itself.
    pub fn own(&self) -> Range<D> {
        std::array::from_fn(|a| (0, self.n[a] as isize))
    }

    /// The points the current step advances: the block, widened on every interface by the overlap
    /// layers that are still correct (`k − 1` right after an exchange, one fewer each step).
    pub fn region(&self) -> Range<D> {
        let r = (self.k - 1 - (self.step % self.k as u64) as usize) as isize;
        std::array::from_fn(|a| {
            let lo = if matches!(self.face[2 * a], Face::Interface(_)) { -r } else { 0 };
            let hi = self.n[a] as isize + if matches!(self.face[2 * a + 1], Face::Interface(_)) { r } else { 0 };
            (lo, hi)
        })
    }

    /// `true` if border values are exchanged before the current step.
    pub fn exchange_due(&self) -> bool {
        self.step % self.k as u64 == 0
    }

    /// The plane of face `f` within `region` (depth 0), and the inward direction along its axis.
    fn face_plane(&self, f: usize, region: &Range<D>) -> (Range<D>, usize, isize) {
        let a = f / 2;
        let mut bx = *region;
        let (e, inward) = if f % 2 == 0 { (0, 1) } else { (self.n[a] as isize - 1, -1) };
        bx[a] = (e, e + 1);
        (bx, a, inward)
    }

    /// Fills the ghost points of the outer faces next to `region` from the current level `u`.
    pub fn fill_outer_ghosts(&self, u: &mut [f64], region: &Range<D>) {
        for f in 0..2 * D {
            let (bx, a, inward) = self.face_plane(f, region);
            match self.face[f] {
                Face::Interface(_) => {}
                Face::ZeroGradient | Face::Absorbing => for_box(&bx, |mut p| {
                    let e = p[a];
                    p[a] = e + inward;
                    let m = self.at(p);
                    p[a] = e - inward;
                    u[self.at(p)] = u[m];
                }),
                Face::Fixed(v) => for_box(&bx, |mut p| {
                    p[a] -= inward;
                    u[self.at(p)] = v;
                }),
            }
        }
    }

    /// Mur's absorbing condition on the absorbing faces within `region` (new level in `p`), in face
    /// order: `p[e] = u[m] + kmur · (p[m] − u[e])`.
    pub fn apply_absorbing(&self, u: &[f64], p: &mut [f64], kmur: f64, region: &Range<D>) {
        for f in 0..2 * D {
            if self.face[f] != Face::Absorbing {
                continue;
            }
            let (bx, a, inward) = self.face_plane(f, region);
            for_box(&bx, |mut q| {
                let e = self.at(q);
                q[a] += inward;
                let m = self.at(q);
                p[e] = u[m] + kmur * (p[m] - u[e]);
            });
        }
    }

    /// Resets the points of the fixed faces within `region`, in face order.
    pub fn apply_fixed(&self, u: &mut [f64], region: &Range<D>) {
        for f in 0..2 * D {
            if let Face::Fixed(v) = self.face[f] {
                let (bx, _, _) = self.face_plane(f, region);
                for_box(&bx, |q| u[self.at(q)] = v);
            }
        }
    }

    /// The boxes this patch sends to the neighbour at `offset`: current level, previous level.
    fn send_boxes(&self, offset: &[isize; D]) -> (Range<D>, Range<D>) {
        let k = self.k as isize;
        let pick = |a: usize, depth: isize| -> (isize, isize) {
            let n = self.n[a] as isize;
            match offset[a] {
                -1 => (0, depth),
                1 => (n - depth, n),
                _ => (0, n),
            }
        };
        (std::array::from_fn(|a| pick(a, k)), std::array::from_fn(|a| pick(a, k - 1)))
    }

    /// The boxes this patch fills from the neighbour at `offset`: current level, previous level.
    fn recv_boxes(&self, offset: &[isize; D]) -> (Range<D>, Range<D>) {
        let k = self.k as isize;
        let pick = |a: usize, depth: isize| -> (isize, isize) {
            let n = self.n[a] as isize;
            match offset[a] {
                -1 => (-depth, 0),
                1 => (n, n + depth),
                _ => (0, n),
            }
        };
        (std::array::from_fn(|a| pick(a, k)), std::array::from_fn(|a| pick(a, k - 1)))
    }

    /// Appends one framed IFACE_BAND message per neighbour (faces in face order, then diagonal
    /// neighbours) with the values of `u` (current level) and `p` (previous level).
    pub fn outbox(&mut self, u: &[f64], p: &[f64], out: &mut Vec<u8>) -> Result<(), PatchError> {
        if !self.exchange_due() {
            return Err(PatchError(format!(
                "step {}: no border exchange at this step (halo width {}, one exchange every {} steps)",
                self.step, self.k, self.k
            )));
        }
        let mut bytes = std::mem::take(&mut self.bytes);
        for l in &self.links {
            let (bu, bp) = self.send_boxes(&l.offset);
            bytes.clear();
            for_box(&bu, |q| bytes.extend_from_slice(&u[self.at(q)].to_bits().to_le_bytes()));
            for_box(&bp, |q| bytes.extend_from_slice(&p[self.at(q)].to_bits().to_le_bytes()));
            let count = bytes.len() / 8;
            let hdr = wire::Header {
                kind: wire::KIND_IFACE_BAND,
                quantity: wire::QTY_DISPLACEMENT,
                src: l.id,
                step: self.step,
                t_us: 0,
            };
            let payload = wire::Payload::IfaceBand { data: &bytes, count: count as u32, per_point: 1 };
            let start = out.len();
            out.resize(start + 4 + wire::HDR_LEN + 8 + 8 * count, 0);
            let mut mb = wire::MailboxWriter::new(&mut out[start..]);
            let pushed = mb.push(&hdr, &payload);
            let used = mb.used();
            if let Err(e) = pushed {
                self.bytes = bytes;
                return Err(PatchError(format!("cannot encode border message: {e:?}")));
            }
            out.truncate(start + used);
        }
        self.bytes = bytes;
        Ok(())
    }

    /// Stores the values of framed border messages into `u` and `p`.
    pub fn inbox(&mut self, u: &mut [f64], p: &mut [f64], framed: &[u8]) -> Result<(), PatchError> {
        for msg in wire::MailboxReader::new(framed) {
            let (hdr, payload, _) = msg.map_err(|e| PatchError(format!("malformed border message: {e:?}")))?;
            let wire::Payload::IfaceBand { data, count, per_point } = payload else {
                return Err(PatchError(format!("unexpected message kind {}", hdr.kind)));
            };
            let li = self
                .links
                .iter()
                .position(|l| l.id == hdr.src)
                .ok_or_else(|| PatchError(format!("message from unknown interface {}", hdr.src)))?;
            let (bu, bp) = self.recv_boxes(&self.links[li].offset);
            let expected = box_len(&bu) + box_len(&bp);
            if !self.exchange_due() || hdr.step != self.step || count as usize != expected || per_point < 1 {
                return Err(PatchError(format!(
                    "border message (interface {}, step {}, {} values) does not fit (step {}, {} values)",
                    hdr.src, hdr.step, count, self.step, expected
                )));
            }
            let pp = per_point as usize;
            let mut i = 0;
            for_box(&bu, |q| {
                u[self.at(q)] = wire::band_get(data, i * pp);
                i += 1;
            });
            for_box(&bp, |q| {
                p[self.at(q)] = wire::band_get(data, i * pp);
                i += 1;
            });
            self.got[li] = true;
        }
        Ok(())
    }

    /// Fails if this is an exchange step and a neighbour's values are missing.
    pub fn check_ready(&self) -> Result<(), PatchError> {
        if self.exchange_due() {
            if let Some(li) = self.got.iter().position(|g| !g) {
                let l = &self.links[li];
                let what = match self.face.iter().position(|f| *f == Face::Interface(l.id)) {
                    Some(f) => format!("face {f}"),
                    None => format!("diagonal neighbour at offset {:?}", l.offset),
                };
                return Err(PatchError(format!("step {}: border values of {what} missing", self.step)));
            }
        }
        Ok(())
    }

    /// Counts a finished step.
    pub fn advance(&mut self) {
        if self.exchange_due() {
            self.got.iter_mut().for_each(|g| *g = false);
        }
        self.step += 1;
    }

    /// The values of the block in `u`, row-major (axis 0 fastest).
    pub fn field(&self, u: &[f64]) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.n.iter().product());
        let mut rows = self.own();
        rows[0] = (0, 1);
        let n0 = self.n[0];
        for_box(&rows, |q| {
            let s = self.at(q);
            out.extend_from_slice(&u[s..s + n0]);
        });
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn for_box_visits_axis_0_fastest() {
        let mut v = Vec::new();
        for_box(&[(0, 2), (5, 7)], |p| v.push(p));
        assert_eq!(v, vec![[0, 5], [1, 5], [0, 6], [1, 6]]);
        let mut c = 0;
        for_box(&[(0, 2), (3, 3)], |_| c += 1);
        assert_eq!(c, 0);
    }
}
