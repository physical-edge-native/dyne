"""Idealised one-step map for the explicit heat scheme in 2D and 3D, with classical and learned regions mixed
(GitHub issue #197). Written by an AI assistant (Claude Code), 2026-11-27.

This extends the 1D model of checker.py to a d-dimensional box grid split into rectangular blocks. Everything here
is an idealised theoretical model, not simulator output.

Scheme (r = alpha*dt/h^2, the same h in every direction):
  - classical point (and every point of a learned block that has a neighbour in another block):
        u_i(t) = u_i(t-1) + r * sum_{j in nbr(i)} (u_j(t-1) - u_i(t-1))
    with 2d neighbours. This is explicit, using only the previous step's values, like heat1d-actor and the
    border points of heat1d-pinn-actor.
  - other points of a learned block, assuming its training converges exactly: backward Euler
        (1 + 2d r) u_i(t) - r * sum_{j in nbr(i)} u_j(t) = u_i(t-1)
    where neighbours that are border points of the same block take their new explicit values as targets.
  - outer edges: 'dirichlet' (the missing neighbour is a fixed value, 0 in the error map) or 'zero_grad'
    (mirror: the missing neighbour u_{-1} equals u_1), as in checker.py.
  - nudging u <- u + gamma (y - u) on observed points after the step (errors scale by 1 - gamma there).

Claim (proven in docs-site/theory.md, "Learned regions that leave a residual"): for 0 <= r <= 1/(2d) and
0 <= gamma <= 2, ||Phi||_inf <= 1 for every arrangement of blocks. Sketch: explicit rows have non-negative
coefficients summing to at most 1 (1 - 2d r >= 0). For the implicit points of a learned block, B has diagonal
1 + 2d r and non-positive off-diagonals, so B is an M-matrix and B^{-1} >= 0; with c the count of a point's
neighbours that are border points of the block or fixed outer values, B*1 = 1 + r c >= 1 + r c_border, so
B^{-1} (1 + r c_border) <= 1 elementwise, and the implicit rows inherit a row sum <= 1 from the border rows.
"""

import itertools

import numpy as np


class MixedHeatND:
    def __init__(self, shape, blocks, r, bc='dirichlet', gamma=0.0, obs=()):
        """shape: grid points per axis; blocks: list of (type, lo, hi) with type 'fd' or 'learned' and lo/hi
        index tuples (hi exclusive) covering the grid exactly once; bc: 'dirichlet' or 'zero_grad' on every
        outer face; obs: observed flat indices for nudging with gamma."""
        self.shape = tuple(int(s) for s in shape)
        self.d = len(self.shape)
        self.n = int(np.prod(self.shape))
        self.r = float(r)
        assert bc in ('dirichlet', 'zero_grad')
        self.bc = bc
        self.owner = -np.ones(self.shape, dtype=int)
        self.types = []
        for k, (t, lo, hi) in enumerate(blocks):
            assert t in ('fd', 'learned')
            sl = tuple(slice(a, b) for a, b in zip(lo, hi))
            assert (self.owner[sl] == -1).all(), 'blocks overlap'
            self.owner[sl] = k
            self.types.append(t)
        assert (self.owner >= 0).all(), 'blocks do not cover the grid'
        self.gamma = float(gamma)
        self.obs = np.array(sorted(obs), dtype=int)
        self.phi = self._build()

    def _neighbours(self, idx):
        """[(flat index or None for a fixed outer value, weight)] for the 2d neighbours of a point."""
        out = []
        for ax in range(self.d):
            for s in (-1, 1):
                j = list(idx)
                j[ax] += s
                if 0 <= j[ax] < self.shape[ax]:
                    out.append((np.ravel_multi_index(j, self.shape), 1.0))
                elif self.bc == 'zero_grad':
                    j[ax] -= 2 * s  # mirror: u_{-1} = u_1
                    out.append((np.ravel_multi_index(j, self.shape), 1.0))
                else:
                    out.append((None, 1.0))
        return out

    def _build(self):
        n, r, d = self.n, self.r, self.d
        owner = self.owner.ravel()
        # explicit rows for every point
        expl = np.zeros((n, n))
        nbrs = []
        for f, idx in enumerate(itertools.product(*[range(s) for s in self.shape])):
            nb = self._neighbours(idx)
            nbrs.append(nb)
            expl[f, f] += 1 - 2 * d * r
            for j, w in nb:
                if j is not None:
                    expl[f, j] += r * w
        # a learned point is a border point if a neighbour (inside the grid) belongs to another block
        border = np.zeros(n, dtype=bool)
        for f in range(n):
            if self.types[owner[f]] == 'learned':
                border[f] = any(j is not None and owner[j] != owner[f] for j, _ in nbrs[f])
        phi = expl.copy()
        for k, t in enumerate(self.types):
            if t != 'learned':
                continue
            inner = np.where((owner == k) & ~border)[0]
            if len(inner) == 0:
                continue
            pos = {f: a for a, f in enumerate(inner)}
            b = np.eye(len(inner)) * (1 + 2 * d * r)
            rhs = np.zeros((len(inner), n))
            for a, f in enumerate(inner):
                rhs[a, f] += 1.0
                for j, w in nbrs[f]:
                    if j is None:
                        continue  # fixed outer value: 0 in the error map
                    if j in pos:
                        b[a, pos[j]] -= r * w
                    else:  # border point of the same block, new explicit value
                        rhs[a] += r * w * expl[j]
            phi[inner] = np.linalg.solve(b, rhs)
        if len(self.obs):
            phi[self.obs] *= (1 - self.gamma)
        return phi

    def step(self, u):
        return self.phi @ u

    def norm(self):
        return np.abs(self.phi).sum(axis=1).max()


def grid_blocks(shape, splits, kinds):
    """Split each axis into nearly equal parts (splits[ax] parts; larger parts first, like
    dyne-native's Decomposition) and give each block a type from kinds(block multi-index)."""
    edges = []
    for s, p in zip(shape, splits):
        base, extra = divmod(s, p)
        sizes = [base + (1 if i < extra else 0) for i in range(p)]
        edges.append(np.concatenate([[0], np.cumsum(sizes)]).astype(int))
    blocks = []
    for bi in itertools.product(*[range(p) for p in splits]):
        lo = tuple(int(edges[ax][i]) for ax, i in enumerate(bi))
        hi = tuple(int(edges[ax][i + 1]) for ax, i in enumerate(bi))
        blocks.append((kinds(bi), lo, hi))
    return blocks


def block_centres(shape, blocks):
    return [int(np.ravel_multi_index(tuple((a + b) // 2 for a, b in zip(lo, hi)), shape)) for _, lo, hi in blocks]
