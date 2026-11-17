//! Loading and calling a 1D WebAssembly solver ("actor") with the wasmi interpreter.
//!
//! [`Actor`] holds the exports of the 1D solver interface. `dyne_init`, `dyne_step`,
//! `dyne_boundary_value`, `dyne_boundary_grad`, `dyne_set_neighbor`, `dyne_set`, `dyne_get`,
//! `dyne_x` and `dyne_len` are required; the others are optional and `None` when the module
//! does not export them. A minimal solver is written step by step in `docs/en/tutorial.md`.

/// Face: left (smaller x).
pub const FACE_LEFT: u32 = 0;
/// Face: right (larger x).
pub const FACE_RIGHT: u32 = 1;
/// Face condition: fixed value (Dirichlet); 0 unless set with `dyne_set_bc_value`.
pub const BC_DIRICHLET: u32 = 0;
/// Face condition: border shared with a neighbouring region (values exchanged).
pub const BC_INTERFACE: u32 = 1;
// ---- F1 (発注3、2026-09-14): 未観測領域境界の設定情報化で追加した3種 ----
/// Face condition: zero gradient (insulated); mirrored ghost point u₋₁ = u₁.
pub const BC_ZERO_GRAD: u32 = 2;
/// Face condition: linear extrapolation. For diffusion the edge point then stays frozen in
/// time, so the simulator warns when it is chosen.
pub const BC_LINEAR_EXTRAP: u32 = 3;
/// Face condition: transparent, as if the material continued without end (see
/// [`crate::transparent`]).
pub const BC_TRANSPARENT: u32 = 4;

/// One instance of a 1D solver module with its exports.
pub struct Actor {
    /// The wasmi store that owns the instance.
    pub store: wasmi::Store<()>,
    /// `dyne_init(n, x0, x1, alpha, dt, bc_left, bc_right) -> i32`: set up a region (0 = success).
    pub init: wasmi::TypedFunc<(u32, f64, f64, f64, f64, u32, u32), i32>,
    /// `dyne_step()`: advance one time step.
    pub step: wasmi::TypedFunc<(), ()>,
    /// `dyne_boundary_value(face) -> f64`: the region's value at a face.
    pub bval: wasmi::TypedFunc<u32, f64>,
    /// `dyne_boundary_grad(face) -> f64`: the region's gradient at a face.
    pub bgrad: wasmi::TypedFunc<u32, f64>,
    /// `dyne_set_neighbor(face, value, grad)`: the neighbour's border value and gradient.
    pub set_neighbor: wasmi::TypedFunc<(u32, f64, f64), ()>,
    /// `dyne_set(i, value)`: overwrite local point `i`.
    pub set: wasmi::TypedFunc<(u32, f64), ()>,
    /// `dyne_get(i) -> f64`: value at local point `i`.
    pub get: wasmi::TypedFunc<u32, f64>,
    /// `dyne_x(i) -> f64`: position of local point `i`.
    pub x: wasmi::TypedFunc<u32, f64>,
    /// `dyne_len() -> u32`: number of local points.
    pub len: wasmi::TypedFunc<(), u32>,
    // ---- optional exports of the learned solver (heat1d-pinn-actor) ----
    /// `dyne_cfg(key, value)`: hyper-parameters, set before `dyne_init` (learned solvers).
    pub cfg: Option<wasmi::TypedFunc<(u32, f64), ()>>,
    /// `dyne_set_bc(face, kind)`: change a face condition keeping the learned weights (without
    /// it, the host re-initialises and restores the state).
    pub set_bc: Option<wasmi::TypedFunc<(u32, u32), ()>>,
    /// `dyne_resize(width) -> i32`: change the width of the hidden layer while running.
    pub resize: Option<wasmi::TypedFunc<u32, i32>>,
    /// `dyne_eval(i) -> f64`: evaluate the current model at point `i` without stepping.
    pub eval: Option<wasmi::TypedFunc<u32, f64>>,
    // ---- optional exports for outer face conditions ----
    /// `dyne_set_bc_value(face, value)`: value of a fixed face (default 0), before `dyne_init`.
    pub set_bc_value: Option<wasmi::TypedFunc<(u32, f64), ()>>,
    /// `dyne_transparent_coef(j, decay, half, weight)`: one term of the transparent face's
    /// coefficients; kept across `dyne_init` (only the auxiliary variables are reset).
    pub transparent_coef: Option<wasmi::TypedFunc<(u32, f64, f64, f64), ()>>,
    /// `dyne_observe(i, y, gamma) -> f64`: nudge point `i` towards measurement `y` with
    /// strength γ; returns y − u_i before nudging.
    pub observe: Option<wasmi::TypedFunc<(u32, f64, f64), f64>>,
    /// Linear memory of the module in 64 KiB pages (to check that it fits a device).
    pub memory_pages: u64,
}

impl Actor {
    /// Instantiate `module` and look up its exports (panics if a required one is missing).
    pub fn load(engine: &wasmi::Engine, module: &wasmi::Module) -> Self {
        let mut store = wasmi::Store::new(engine, ());
        let linker: wasmi::Linker<()> = wasmi::Linker::new(engine);
        let instance = linker
            .instantiate(&mut store, module)
            .expect("instantiate")
            .start(&mut store)
            .expect("start");
        let memory_pages = instance
            .get_memory(&store, "memory")
            .map(|m| m.size(&store) as u64)
            .unwrap_or(0);
        macro_rules! func {
            ($name:literal, $p:ty, $r:ty) => {
                instance.get_typed_func::<$p, $r>(&store, $name).expect($name)
            };
        }
        Actor {
            init: func!("dyne_init", (u32, f64, f64, f64, f64, u32, u32), i32),
            step: func!("dyne_step", (), ()),
            bval: func!("dyne_boundary_value", u32, f64),
            bgrad: func!("dyne_boundary_grad", u32, f64),
            set_neighbor: func!("dyne_set_neighbor", (u32, f64, f64), ()),
            set: func!("dyne_set", (u32, f64), ()),
            get: func!("dyne_get", u32, f64),
            x: func!("dyne_x", u32, f64),
            len: func!("dyne_len", (), u32),
            cfg: instance.get_typed_func::<(u32, f64), ()>(&store, "dyne_cfg").ok(),
            set_bc: instance.get_typed_func::<(u32, u32), ()>(&store, "dyne_set_bc").ok(),
            resize: instance.get_typed_func::<u32, i32>(&store, "dyne_resize").ok(),
            eval: instance.get_typed_func::<u32, f64>(&store, "dyne_eval").ok(),
            set_bc_value: instance
                .get_typed_func::<(u32, f64), ()>(&store, "dyne_set_bc_value")
                .ok(),
            transparent_coef: instance
                .get_typed_func::<(u32, f64, f64, f64), ()>(&store, "dyne_transparent_coef")
                .ok(),
            observe: instance
                .get_typed_func::<(u32, f64, f64), f64>(&store, "dyne_observe")
                .ok(),
            store,
            memory_pages,
        }
    }

    /// [`Actor::load`], then pass `pinn_cfg` to `dyne_cfg` (ignored if the module has no
    /// `dyne_cfg`).
    pub fn load_with(engine: &wasmi::Engine, module: &wasmi::Module, pinn_cfg: &[(u32, f64)]) -> Self {
        let mut a = Actor::load(engine, module);
        if let Some(f) = a.cfg {
            for &(k, v) in pinn_cfg {
                f.call(&mut a.store, (k, v)).unwrap();
            }
        }
        a
    }

    /// Whether this is a learned solver (it exports `dyne_cfg`).
    pub fn is_learning(&self) -> bool {
        self.cfg.is_some()
    }
}
