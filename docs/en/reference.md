# Reference: solver interface, message format and API documentation

## API documentation

The Rust API documentation of the core crates is generated with `cargo doc`:

```sh
cargo doc --no-deps -p dyne-wire -p dyne-core -p dyne-native -p dyne-export --open
```

CI builds it with warnings treated as errors, so every public item is documented.

| Crate | What it documents |
|---|---|
| `dyne-wire` | the message format exchanged at region borders (wire format v2), the mailbox framing, the envelope used on radio links, and (module `abi`) the constants of the solver interface |
| `dyne-core` | the deterministic simulator parts: random numbers, event queue in virtual time, virtual network, loading of 1D WebAssembly solvers, transparent boundary coefficients |
| `dyne-native` | native PC solvers that give the same bits as the WebAssembly solvers, domain splitting, wide borders, convergence checks |
| `dyne-export` | export to VTK image data and OpenFOAM (also with the walls and furniture of a room shape: modules `walls`, `polymesh`), import of OpenFOAM velocity fields |

## Solver interface

A solver ("actor") is a WebAssembly module (`wasm32-unknown-unknown`, usually a `no_std`
Rust crate) that exports functions; the host (a simulator or the device firmware) calls
them. The solver never handles time, the network or neighbours itself.

- **1D** (`dyne-sim`, `crates/heat1d-actor`): values are exchanged one per border through
  `dyne_set_neighbor` and `dyne_boundary_value` / `dyne_boundary_grad`. The required and
  optional exports are listed in the [tutorial](tutorial.md) and in `dyne_core::actor`.
- **2D** (`dyne-sim2d`, the firmware, `crates/heat2d-actor`, `crates/wave2d-actor` and
  others): border values travel as messages through an inbox and an outbox in the solver's
  memory. The call sequence and the constants are documented in `dyne_wire::abi`
  (module `field2d`), the message layout in `dyne_wire`.

## Message format

Each border message has a 24-byte header (version, kind, quantity, source border id,
step, time stamp) followed by a payload that depends on the kind: one value and gradient
(1D), or the values at all points of a border (2D and 3D). Messages are packed into
fixed-size mailboxes as `[u32 length][message]`; a message that does not fit is dropped
and counted, never written partially. On radio links each datagram additionally carries a
sequence number and a CRC-32 (`dyne_wire::envelope`). The full layout is in the
documentation of `dyne_wire`.

The ESP32 firmware uses a C copy of this layout; a shared table of reference bytes
(`node-esp32/main/dyne_wire2_golden.h`) is checked by the Rust tests, so the two cannot
drift apart unnoticed.
