//! Running Dyne's 2D WebAssembly patch solvers (for example `wave2d-actor`) as patches of the native runners.
//!
//! Dyne's 2D field solvers export one fixed set of functions: `dyne_cfg` (configuration),
//! `dyne_set_face` (what each face is), `dyne_init2d`, a message buffer (`dyne_msg_buf`,
//! `dyne_msg_cap`) used as the inbox and outbox (`dyne_inbox_write`, `dyne_outbox_read`), `dyne_step`
//! and `dyne_get`. This module loads such a solver with the `wasmi` interpreter (the same one Dyne's
//! simulators use) and wraps it as a [`PatchSolver`], so that the exact solver that runs on Dyne's
//! microcontrollers can be compared bit for bit with the native solvers.
//!
//! The interpreter is much slower than native code; this path is for checking, not for speed.

use crate::decomp::Block;
use crate::patch::{PatchError, PatchSolver};
use crate::wave2d::Wave2dProblem;
use dyne_wire::abi::field2d::{INBOX_OK, STEP_OK};

type Func<P, R> = wasmi::TypedFunc<P, R>;

/// One instance of a Dyne 2D field solver compiled to WebAssembly.
pub struct Wasm2dPatch {
    store: wasmi::Store<()>,
    memory: wasmi::Memory,
    inbox_write: Func<(u32, u32), i32>,
    outbox_read: Func<(u32, u32), u32>,
    step: Func<(), i32>,
    get: Func<u32, f64>,
    buf: u32,
    cap: u32,
    len: usize,
}

fn err<E: std::fmt::Debug>(what: &str) -> impl FnOnce(E) -> PatchError + '_ {
    move |e| PatchError(format!("{what}: {e:?}"))
}

impl Wasm2dPatch {
    /// Instantiates `module` and sets it up as block `b` of a wave problem
    /// (`dyne_cfg` calls, then `dyne_set_face` for the four faces, then `dyne_init2d`).
    pub fn new_wave(
        engine: &wasmi::Engine,
        module: &wasmi::Module,
        problem: &Wave2dProblem,
        b: &Block<2>,
    ) -> Result<Self, PatchError> {
        let mut store = wasmi::Store::new(engine, ());
        let linker: wasmi::Linker<()> = wasmi::Linker::new(engine);
        let instance = linker
            .instantiate(&mut store, module)
            .map_err(err("instantiate"))?
            .start(&mut store)
            .map_err(err("start"))?;
        let memory = instance.get_memory(&store, "memory").ok_or_else(|| PatchError("no exported memory".into()))?;
        macro_rules! func {
            ($name:literal, $p:ty, $r:ty) => {
                instance.get_typed_func::<$p, $r>(&store, $name).map_err(err($name))?
            };
        }
        let cfg = func!("dyne_cfg", (u32, f64), ());
        let set_face = func!("dyne_set_face", (u32, u32, u64), ());
        let init = func!("dyne_init2d", (u32, u32, u32, u32, f64, f64, f64), i32);
        let msg_buf = func!("dyne_msg_buf", (), u32);
        let msg_cap = func!("dyne_msg_cap", (), u32);
        let mut s = Wasm2dPatch {
            inbox_write: func!("dyne_inbox_write", (u32, u32), i32),
            outbox_read: func!("dyne_outbox_read", (u32, u32), u32),
            step: func!("dyne_step", (), i32),
            get: func!("dyne_get", u32, f64),
            store,
            memory,
            buf: 0,
            cap: 0,
            len: b.len(),
        };
        s.buf = msg_buf.call(&mut s.store, ()).map_err(err("dyne_msg_buf"))?;
        s.cap = msg_cap.call(&mut s.store, ()).map_err(err("dyne_msg_cap"))?;
        for (key, val) in problem.actor_cfg() {
            cfg.call(&mut s.store, (key, val)).map_err(err("dyne_cfg"))?;
        }
        for (face, (kind, param)) in problem.actor_faces(b).into_iter().enumerate() {
            set_face.call(&mut s.store, (face as u32, kind, param)).map_err(err("dyne_set_face"))?;
        }
        let [nx, ny] = b.size;
        let [ix0, iy0] = b.origin;
        let rc = init
            .call(&mut s.store, (nx as u32, ny as u32, ix0 as u32, iy0 as u32, problem.h, problem.c, problem.dt))
            .map_err(err("dyne_init2d"))?;
        if rc != 0 {
            return Err(PatchError(format!("dyne_init2d failed (code {rc}) for a {nx}×{ny} patch")));
        }
        Ok(s)
    }
}

impl PatchSolver for Wasm2dPatch {
    fn outbox(&mut self, out: &mut Vec<u8>) -> Result<(), PatchError> {
        let n = self.outbox_read.call(&mut self.store, (self.buf, self.cap)).map_err(err("dyne_outbox_read"))? as usize;
        let start = out.len();
        out.resize(start + n, 0);
        self.memory.read(&self.store, self.buf as usize, &mut out[start..]).map_err(err("read memory"))?;
        Ok(())
    }

    fn inbox(&mut self, framed: &[u8]) -> Result<(), PatchError> {
        let mut b = Vec::with_capacity(framed.len() + 4);
        b.extend_from_slice(framed);
        b.extend_from_slice(&0u32.to_le_bytes());
        if b.len() > self.cap as usize {
            return Err(PatchError(format!("{} bytes do not fit the solver's message buffer", b.len())));
        }
        self.memory.write(&mut self.store, self.buf as usize, &b).map_err(err("write memory"))?;
        let rc = self.inbox_write.call(&mut self.store, (self.buf, b.len() as u32)).map_err(err("dyne_inbox_write"))?;
        if rc != INBOX_OK {
            return Err(PatchError(format!("dyne_inbox_write rejected a border message (code {rc})")));
        }
        Ok(())
    }

    fn step(&mut self) -> Result<(), PatchError> {
        let rc = self.step.call(&mut self.store, ()).map_err(err("dyne_step"))?;
        if rc != STEP_OK {
            return Err(PatchError(format!("dyne_step: border values missing (code {rc})")));
        }
        Ok(())
    }

    fn field(&mut self) -> Result<Vec<f64>, PatchError> {
        (0..self.len as u32).map(|k| self.get.call(&mut self.store, k).map_err(err("dyne_get"))).collect()
    }
}
