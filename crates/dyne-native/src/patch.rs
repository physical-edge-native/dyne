//! The interface of a patch solver and the runners that advance a decomposed grid.
//!
//! A patch solver owns the values of one block of the grid. Patches cooperate only through
//! *border messages*: each step, every patch writes the values on its faces that touch a
//! neighbour, and reads the values its neighbours wrote. The messages use Dyne's wire format
//! version 2 (`dyne_wire`, kind `KIND_IFACE_BAND`: a header with the sender's interface id and the
//! step number, followed by the list of `f64` values on the face) and are framed as in a Dyne
//! mailbox (`[u32 length][message]`). Because WebAssembly solvers built for Dyne read and write
//! exactly these mailboxes, a native solver and a WebAssembly solver are interchangeable here.
//!
//! The interface does not depend on the dimension: a face of a 3D block is a 2D set of values,
//! sent as one flat list in the block's point order.

use crate::decomp::{Block, Decomposition};
use dyne_wire as wire;
use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc;
use std::sync::{Barrier, Mutex};
use std::time::{Duration, Instant};

/// Error from a patch solver or a runner.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PatchError(pub String);

impl fmt::Display for PatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for PatchError {}

/// A solver for one patch (block) of the grid.
///
/// One step of a decomposed run is, for every patch: [`outbox`](Self::outbox) (send the current
/// border values), [`inbox`](Self::inbox) for every message from the neighbours with the same step
/// number, then [`step`](Self::step).
///
/// A solver with a border width `k` = [`halo_width`](Self::halo_width) greater than 1 keeps `k`
/// layers of its neighbours' values: the exchange (outbox, then inbox) happens only before steps
/// that are multiples of `k`, and then also with the blocks that touch it at an edge or a corner
/// ([`Block::diagonals`]), whose link ids it uses as sender ids.
pub trait PatchSolver: Send {
    /// Appends this patch's border messages for the current step to `out`: one framed message
    /// (`[u32 length][message]`, no end marker) per face that touches a neighbour.
    fn outbox(&mut self, out: &mut Vec<u8>) -> Result<(), PatchError>;

    /// Delivers framed border messages from neighbours (`[u32 length][message]` repeated, no end
    /// marker needed). Messages for another step or an unknown interface are an error.
    fn inbox(&mut self, framed: &[u8]) -> Result<(), PatchError>;

    /// Advances one step. Fails if a neighbour's border values for this step are missing.
    fn step(&mut self) -> Result<(), PatchError>;

    /// The current values of the block, row-major over the block (axis 0 fastest).
    fn field(&mut self) -> Result<Vec<f64>, PatchError>;

    /// Number of layers of border values exchanged per message (the border width `k`): border
    /// values are exchanged once every `k` steps, and with diagonal neighbours if `k > 1`. All
    /// patches of a run must use the same width. The default is 1 (one layer, every step, face
    /// neighbours only), the scheme of Dyne's WebAssembly solvers.
    fn halo_width(&self) -> usize {
        1
    }
}

/// Advances one patch that has no neighbours (the single-patch reference) for `steps` steps on
/// the calling thread, and returns its field.
pub fn run_single<S: PatchSolver>(solver: &mut S, steps: u64) -> Result<Vec<f64>, PatchError> {
    for _ in 0..steps {
        solver.step()?;
    }
    solver.field()
}

/// Options of [`run_threaded_with`].
#[derive(Clone, Copy, Debug, Default)]
pub struct RunOptions {
    /// How long a worker that waits for a neighbour's border values polls its channel before it
    /// blocks (gives up its core). Zero (the default) blocks at once; a short spin can lower the
    /// wake-up delay when patches are small and a step takes only tens of microseconds.
    pub spin: Duration,
}

/// Result of [`run_threaded`].
#[derive(Clone, Debug)]
pub struct ThreadedRun {
    /// The global field (row-major, axis 0 fastest) after the last step.
    pub field: Vec<f64>,
    /// Wall time of the step loop only (from when every patch is set up until every patch has
    /// finished its last step).
    pub wall: Duration,
    /// Per block (in the order of [`Decomposition::blocks`]): time spent inside
    /// [`PatchSolver::step`]. The rest of the wall time is spent on border messages and waiting.
    pub compute: Vec<Duration>,
}

/// The step number of a framed border message.
fn framed_step(msg: &[u8]) -> Result<u64, PatchError> {
    if msg.len() < 4 {
        return Err(PatchError("truncated border message".into()));
    }
    match wire::decode(&msg[4..]) {
        Ok((h, _, _)) => Ok(h.step),
        Err(e) => Err(PatchError(format!("undecodable border message: {e:?}"))),
    }
}

/// How long a waiting worker blocks before checking whether another worker has failed.
const ABORT_POLL: Duration = Duration::from_millis(50);

/// [`run_threaded_with`] with the default options.
pub fn run_threaded<const D: usize, S, F>(decomp: &Decomposition<D>, steps: u64, make: F) -> Result<ThreadedRun, PatchError>
where
    S: PatchSolver,
    F: Fn(&Block<D>) -> Result<S, PatchError> + Sync,
{
    run_threaded_with(decomp, steps, RunOptions::default(), make)
}

/// Advances every block of `decomp` for `steps` steps, one worker thread per block.
///
/// `make` builds the solver of a block; it is called on the block's own worker thread (so that
/// the block's memory is first touched by the thread that uses it). Each worker exchanges border
/// messages only with the workers of neighbouring blocks, through channels; there is no global
/// synchronisation inside the step loop. A worker may run at most one step ahead of a neighbour,
/// so messages for the next step that arrive early are kept until that step. The result does not
/// depend on the timing of the threads.
///
/// With solvers whose [`PatchSolver::halo_width`] is `k > 1`, borders are exchanged only before
/// steps that are multiples of `k`, also with diagonal neighbours, and a worker may run at most one
/// exchange (`k` steps) ahead of a neighbour. All solvers must report the same width; a mismatch is
/// an error.
///
/// If any worker fails, the others stop and the first error is returned.
pub fn run_threaded_with<const D: usize, S, F>(
    decomp: &Decomposition<D>,
    steps: u64,
    options: RunOptions,
    make: F,
) -> Result<ThreadedRun, PatchError>
where
    S: PatchSolver,
    F: Fn(&Block<D>) -> Result<S, PatchError> + Sync,
{
    let n = decomp.blocks.len();
    let (senders, receivers): (Vec<_>, Vec<_>) = (0..n).map(|_| mpsc::channel::<Vec<u8>>()).unzip();
    let abort = AtomicBool::new(false);
    let start = Barrier::new(n + 1);
    let end = Barrier::new(n + 1);
    let error: Mutex<Option<PatchError>> = Mutex::new(None);
    // Border width of each block's solver (0 until built, or if building failed).
    let widths: Vec<AtomicUsize> = (0..n).map(|_| AtomicUsize::new(0)).collect();

    let fail = |e: PatchError| {
        abort.store(true, Ordering::SeqCst);
        let mut slot = error.lock().unwrap();
        if slot.is_none() {
            *slot = Some(e);
        }
    };

    let mut wall = Duration::ZERO;
    let results: Vec<Option<(Vec<f64>, Duration)>> = std::thread::scope(|scope| {
        let mut handles = Vec::with_capacity(n);
        for (b, rx) in decomp.blocks.iter().zip(receivers) {
            // Senders to this block's neighbours: (interface id, channel of the neighbour).
            let mut to: Vec<(u32, mpsc::Sender<Vec<u8>>)> =
                b.faces.iter().flatten().map(|i| (i.id, senders[i.neighbour].clone())).collect();
            // Senders to the diagonal neighbours, used only if the solver's border width is above 1.
            let to_diagonal: Vec<(u32, mpsc::Sender<Vec<u8>>)> =
                b.diagonals.iter().map(|d| (d.id, senders[d.neighbour].clone())).collect();
            let (make, abort, start, end, fail, widths) = (&make, &abort, &start, &end, &fail, &widths);
            handles.push(scope.spawn(move || -> Option<(Vec<f64>, Duration)> {
                let solver = make(b);
                let width = solver.as_ref().map_or(0, |s| s.halo_width());
                widths[b.index].store(width, Ordering::SeqCst);
                start.wait();
                let mut solver = match solver {
                    Ok(s) => s,
                    Err(e) => {
                        fail(PatchError(format!("block {}: {}", b.index, e)));
                        end.wait();
                        return None;
                    }
                };
                // Every built neighbour must use the same border width.
                let mismatch = b
                    .faces
                    .iter()
                    .flatten()
                    .map(|i| i.neighbour)
                    .chain(b.diagonals.iter().map(|d| d.neighbour))
                    .map(|nb| (nb, widths[nb].load(Ordering::SeqCst)))
                    .find(|&(_, w)| w != 0 && w != width);
                let result = if width == 0 {
                    Err(PatchError("border width (halo) must be at least 1".into()))
                } else if let Some((nb, w)) = mismatch {
                    Err(PatchError(format!(
                        "border width (halo) {width} differs from the width {w} of neighbouring block {nb};                          all patches must use the same width"
                    )))
                } else {
                    if width > 1 {
                        to.extend(to_diagonal);
                    }
                    let mut w = Worker { need: to.len(), to, rx, abort, spin: options.spin, every: width as u64 };
                    w.run(&mut solver, steps)
                };
                if result.is_err() {
                    // Make the neighbours stop waiting before this worker waits at the end.
                    abort.store(true, Ordering::SeqCst);
                }
                end.wait();
                match result.and_then(|compute| Ok((solver.field()?, compute))) {
                    Ok(r) => Some(r),
                    Err(e) => {
                        fail(PatchError(format!("block {}: {}", b.index, e)));
                        None
                    }
                }
            }));
        }
        drop(senders);
        start.wait();
        let t0 = Instant::now();
        end.wait();
        wall = t0.elapsed();
        handles.into_iter().map(|h| h.join().unwrap_or(None)).collect()
    });

    if let Some(e) = error.lock().unwrap().take() {
        return Err(e);
    }
    let (fields, compute): (Vec<Vec<f64>>, Vec<Duration>) =
        results.into_iter().map(|r| r.expect("every worker returned a result")).unzip();
    Ok(ThreadedRun { field: decomp.gather(&fields), wall, compute })
}

/// The message handling of one worker thread.
struct Worker<'a> {
    /// Number of neighbours (messages expected per step).
    need: usize,
    to: Vec<(u32, mpsc::Sender<Vec<u8>>)>,
    rx: mpsc::Receiver<Vec<u8>>,
    abort: &'a AtomicBool,
    spin: Duration,
    /// Border width: exchange before every `every`-th step.
    every: u64,
}

impl Worker<'_> {
    fn stopped(&self) -> Result<(), PatchError> {
        if self.abort.load(Ordering::Relaxed) {
            Err(PatchError("stopped because another block failed".into()))
        } else {
            Ok(())
        }
    }

    /// Next message from a neighbour: polls for `spin`, then blocks (checking for an abort).
    fn recv(&self) -> Result<Vec<u8>, PatchError> {
        let spin_until = Instant::now() + self.spin;
        loop {
            match self.rx.try_recv() {
                Ok(m) => return Ok(m),
                Err(mpsc::TryRecvError::Disconnected) => return Err(PatchError("neighbours stopped".into())),
                Err(mpsc::TryRecvError::Empty) if Instant::now() < spin_until => std::hint::spin_loop(),
                Err(mpsc::TryRecvError::Empty) => break,
            }
        }
        loop {
            match self.rx.recv_timeout(ABORT_POLL) {
                Ok(m) => return Ok(m),
                Err(mpsc::RecvTimeoutError::Timeout) => self.stopped()?,
                Err(mpsc::RecvTimeoutError::Disconnected) => return Err(PatchError("neighbours stopped".into())),
            }
        }
    }

    /// The step loop; returns the time spent inside `solver.step()`.
    fn run<S: PatchSolver>(&mut self, solver: &mut S, steps: u64) -> Result<Duration, PatchError> {
        let mut out = Vec::new();
        let mut early: Vec<Vec<u8>> = Vec::new();
        let mut compute = Duration::ZERO;
        for s in 0..steps {
            self.stopped()?;
            if self.need > 0 && s % self.every == 0 {
                // Send this step's border values, each message to the neighbour across its interface.
                out.clear();
                solver.outbox(&mut out)?;
                for m in wire::MailboxReader::new(&out) {
                    let (hdr, _, raw) = m.map_err(|e| PatchError(format!("bad outgoing message: {e:?}")))?;
                    let (_, tx) = self
                        .to
                        .iter()
                        .find(|(id, _)| *id == hdr.src)
                        .ok_or_else(|| PatchError(format!("outgoing message for unknown interface {}", hdr.src)))?;
                    let mut framed = Vec::with_capacity(4 + raw.len());
                    framed.extend_from_slice(&(raw.len() as u32).to_le_bytes());
                    framed.extend_from_slice(raw);
                    // A send fails only if the neighbour has stopped; it then reports its own error.
                    let _ = tx.send(framed);
                }

                // Receive the neighbours' border values for this step.
                let mut got = 0;
                let mut i = 0;
                while i < early.len() {
                    if framed_step(&early[i])? == s {
                        solver.inbox(&early.swap_remove(i))?;
                        got += 1;
                    } else {
                        i += 1;
                    }
                }
                while got < self.need {
                    let msg = self.recv()?;
                    match framed_step(&msg)? {
                        ms if ms == s => {
                            solver.inbox(&msg)?;
                            got += 1;
                        }
                        ms if ms == s + self.every => early.push(msg),
                        ms => return Err(PatchError(format!("message for step {ms} while at step {s}"))),
                    }
                }
            }
            let t = Instant::now();
            solver.step()?;
            compute += t.elapsed();
        }
        Ok(compute)
    }
}
