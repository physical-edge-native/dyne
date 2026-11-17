//! Export of Dyne's field state to other tools.
//!
//! Dyne does not compete with general field-physics packages; detailed studies are delegated to them
//! by exporting the state that Dyne computed. This crate holds those exports, without dependencies.
//!
//! What is implemented:
//!
//! - [`vti`]: scalar fields on a regular 2D or 3D grid as a VTK ImageData file (`.vti`), readable by
//!   ParaView and PyVista. Values are stored losslessly (raw little-endian `Float64`), and
//!   [`vti::ImageData::from_vti_bytes`] reads back exactly what [`vti::ImageData::to_vti_bytes`]
//!   writes, so a round trip can be checked bit for bit.
//! - [`foam`]: a scalar field on a regular 2D or 3D grid as an OpenFOAM case for `scalarTransportFoam`
//!   (advection–diffusion in a given velocity field): mesh, initial field, face conditions, velocity,
//!   diffusivity. One OpenFOAM cell per Dyne grid point, values in shortest round-trip decimal form;
//!   [`foam::Case::read`] reads a written case back bit for bit. In the other direction,
//!   [`foam::import_velocity`] reads a velocity field computed by OpenFOAM on such a mesh.
//! - [`walls`] and [`polymesh`]: the room shape built by Dyne's devices (a signed distance field) as
//!   solid grid points, and the OpenFOAM mesh without them, whose faces towards walls and obstacles form
//!   the patch `walls`.
//! - [`sensors`]: an initial field from a few point sensors (inverse-distance weighting), e.g. the
//!   temperature readings of the devices for the initial `T` of an OpenFOAM case.
//!
//! The binary `dyne-vti` prints the grid and the CRC-32 of every array of a `.vti` file written by
//! this crate ([`field_crc32`], the field hash that Dyne's simulators print). The binary `dyne-foam`
//! writes an OpenFOAM case from a `.vti` file, reads a case back (printing the CRC-32 of the field),
//! and imports a velocity field into a `.vti` file.

#![warn(missing_docs)]

pub mod foam;
pub mod polymesh;
pub mod sensors;
pub mod vti;
pub mod walls;

/// CRC-32 (IEEE 802.3) of the little-endian bits of a field: the field hash that Dyne's simulators
/// and native solvers print, so that an exported field can be compared with a run's output.
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
        // Same check as dyne-native: the bytes "12345678" as one f64 give 0x9AE0DAAF.
        let v = f64::from_bits(u64::from_le_bytes(*b"12345678"));
        assert_eq!(super::field_crc32(&[v]), 0x9AE0_DAAF);
    }
}
