//! Native patch solvers on a PC, bit-exact with Dyne's WebAssembly solvers.
//!
//! The domain is split into patches (blocks); each patch has its own small solver, and patches
//! cooperate only by exchanging values at their borders, as Dyne's devices do. Here every patch runs
//! on a thread of one PC, with native Rust solvers that perform the same floating-point operations as
//! Dyne's WebAssembly solvers. They serve as a reference solver for checking a run on devices and as a
//! fast backend for simulating many nodes (design record:
//! `docs/design/2026-11-16-native-backend-from-barye.md`).
//!
//! The crate shares one definition of these parts with the rest of Dyne:
//!
//! - the border-exchange messages (wire format version 2), re-exported as [`wire`];
//! - the deterministic simulator parts (random numbers, discrete-event virtual time, virtual
//!   network, loading of WebAssembly solvers), re-exported as [`sim`].
//!
//! What is implemented:
//!
//! - [`decomp`]: splitting a grid of any dimension into blocks, with face and interface numbering;
//! - [`patch`]: the patch-solver interface and the runners (single patch; one thread per patch,
//!   exchanging only border messages with neighbouring patches);
//! - [`wave2d`]: a native 2D wave-equation patch solver that gives the same bits as Dyne's
//!   WebAssembly wave solver (`crates/wave2d-actor`);
//! - [`halo`]: wider borders for the native solvers (`k` layers, one exchange every `k` steps,
//!   bit-identical results);
//! - [`wasm2d`]: running Dyne's 2D WebAssembly patch solvers as patches.
//!
//! Moved from physical-edge-native/barye (crate `barye-core`).

pub mod decomp;
pub mod halo;
pub mod patch;
pub mod wasm2d;
pub mod wave2d;

/// Dyne's border-exchange format (wire format version 2) and mailbox conventions.
pub use dyne_wire as wire;

/// Dyne's deterministic simulator parts (random numbers, virtual time, virtual network,
/// WebAssembly solver loading).
pub use dyne_core as sim;

/// CRC-32 (IEEE 802.3) of the little-endian bits of a field: the field hash that Dyne's simulators
/// print, so that results of the native solvers and of the simulators can be compared.
pub fn field_crc32(field: &[f64]) -> u32 {
    let mut c: u32 = 0xFFFF_FFFF;
    for v in field {
        for b in v.to_bits().to_le_bytes() {
            c ^= b as u32;
            for _ in 0..8 {
                let m = (c & 1).wrapping_neg();
                c = (c >> 1) ^ (0xEDB8_8320 & m);
            }
        }
    }
    !c
}

#[cfg(test)]
mod tests {
    #[test]
    fn crc32_check_value() {
        // The standard check value: CRC-32 of the ASCII bytes "123456789" is 0xCBF43926. Here the
        // input must be whole f64 values, so use "12345678" (one value), whose CRC-32 is 0x9AE0DAAF.
        let v = f64::from_bits(u64::from_le_bytes(*b"12345678"));
        assert_eq!(super::field_crc32(&[v]), 0x9AE0_DAAF);
    }
}
