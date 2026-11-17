//! dyne-core: shared parts of Dyne's deterministic simulators.
//!
//! The simulators (`dyne-sim` for 1D, `dyne-sim2d` for 2D, `dyne-swarm-sim` for the
//! flocking demo) are built from these model-independent parts:
//!
//! - [`rng`]: deterministic random numbers (xorshift64*) and the rule for deriving
//!   per-device seeds;
//! - [`event`]: event queue in virtual time (discrete-event simulation);
//! - [`net`]: virtual network with loss, delay jitter and partitions;
//! - [`actor`]: loading and calling a 1D WebAssembly solver ("actor") with wasmi;
//! - [`transparent`]: coefficients for a boundary that lets heat flow out as if the
//!   material continued without end;
//! - [`wire`]: the message format (re-export of `dyne-wire`).
//!
//! Everything is deterministic: with the same inputs and seed, every run gives the same
//! bits on every platform (integer arithmetic for random numbers, a fixed order for
//! simultaneous events, pure-Rust `libm` for mathematical functions). Changing the order in
//! which random numbers are drawn or events are delivered changes results, and the
//! regression suite would report it.

#![warn(missing_docs)]

pub mod actor;
pub mod event;
pub mod net;
pub mod rng;
pub mod transparent;

/// The message format (re-export of `dyne-wire`).
pub use dyne_wire as wire;
