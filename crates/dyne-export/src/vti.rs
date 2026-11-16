//! VTK ImageData (`.vti`): scalar fields on a regular 2D or 3D grid.
//!
//! # File layout
//!
//! The file is VTK's XML format (file version 1.0) with the values in one appended binary block:
//!
//! ```text
//! <?xml version="1.0"?>
//! <VTKFile type="ImageData" version="1.0" byte_order="LittleEndian" header_type="UInt64">
//!   <ImageData WholeExtent="0 NX-1 0 NY-1 0 NZ-1" Origin="X0 Y0 Z0" Spacing="DX DY DZ">
//!     <Piece Extent="0 NX-1 0 NY-1 0 NZ-1">
//!       <PointData Scalars="FIRST">
//!         <DataArray type="Float64" Name="NAME" NumberOfComponents="1" format="appended" offset="O"/>
//!         ...
//!       </PointData>
//!       <CellData>
//!       </CellData>
//!     </Piece>
//!   </ImageData>
//!   <AppendedData encoding="raw">
//!    _[UInt64 byte count][Float64 values] [UInt64 byte count][Float64 values] ...
//!   </AppendedData>
//! </VTKFile>
//! ```
//!
//! - Values are point data (one value per grid point), numbered with x fastest, then y, then z:
//!   index `i + NX·(j + NY·k)`. This is the order of Dyne's fields (row-major, axis 0 fastest).
//! - The values are stored as raw little-endian IEEE 754 `Float64`, so every bit is kept: NaN
//!   payloads, infinities, `-0.0` and subnormal numbers included. Each array is preceded by its size
//!   in bytes as a little-endian `UInt64`; `offset` counts bytes from the first byte after `_`.
//! - Origin and spacing are written with Rust's shortest round-trip decimal form, so they are read
//!   back exactly.
//! - A 2D field is written as a grid with one point along z (`NZ = 1`, z origin 0, z spacing equal
//!   to the x spacing). VTK needs a positive spacing on every axis even when it has one point.
//!
//! Raw appended data is the uncompressed binary form that VTK's own XML writers produce; ParaView,
//! PyVista and VTK's Python package read it. It is about 25% smaller than the base64 form and needs
//! no encoder; the file is not plain text, which is acceptable for a data file.
//!
//! [`ImageData::from_vti_bytes`] reads exactly this layout (as written by
//! [`ImageData::to_vti_bytes`]) and rejects anything else; it is not a general VTK reader.

use std::fmt;
use std::fmt::Write as _;
use std::path::Path;

/// An error while building, writing or reading a `.vti` file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VtiError(pub String);

impl fmt::Display for VtiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for VtiError {}

fn err<T>(msg: impl Into<String>) -> Result<T, VtiError> {
    Err(VtiError(msg.into()))
}

/// One scalar value per grid point.
#[derive(Debug, Clone, PartialEq)]
pub struct PointArray {
    /// Name shown in ParaView: ASCII letters, digits, `_`, `-` and `.` only.
    pub name: String,
    /// Values, x fastest, then y, then z.
    pub values: Vec<f64>,
}

/// Scalar fields on a regular grid: point counts, origin, spacing and named arrays of point values.
#[derive(Debug, Clone, PartialEq)]
pub struct ImageData {
    /// Number of points along x, y and z (z is 1 for a 2D field).
    pub points: [usize; 3],
    /// Coordinates of the first point.
    pub origin: [f64; 3],
    /// Distance between neighbouring points along each axis (positive).
    pub spacing: [f64; 3],
    /// The arrays, in file order. The first one is marked as the active scalars.
    pub arrays: Vec<PointArray>,
}

impl ImageData {
    /// A 2D grid of `points[0] × points[1]` points, written with one point along z.
    pub fn new_2d(points: [usize; 2], origin: [f64; 2], spacing: [f64; 2]) -> ImageData {
        ImageData {
            points: [points[0], points[1], 1],
            origin: [origin[0], origin[1], 0.0],
            spacing: [spacing[0], spacing[1], spacing[0]],
            arrays: Vec::new(),
        }
    }

    /// A 3D grid of `points[0] × points[1] × points[2]` points.
    pub fn new_3d(points: [usize; 3], origin: [f64; 3], spacing: [f64; 3]) -> ImageData {
        ImageData { points, origin, spacing, arrays: Vec::new() }
    }

    /// Number of grid points.
    pub fn len(&self) -> usize {
        self.points.iter().product()
    }

    /// True if the grid has no points.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Adds an array of point values (checked when the file is written).
    pub fn add(&mut self, name: &str, values: Vec<f64>) -> &mut ImageData {
        self.arrays.push(PointArray { name: name.to_string(), values });
        self
    }

    /// The array with this name, if any.
    pub fn array(&self, name: &str) -> Option<&PointArray> {
        self.arrays.iter().find(|a| a.name == name)
    }

    fn check(&self) -> Result<(), VtiError> {
        if self.points.contains(&0) {
            return err(format!("every axis needs at least one point (points {:?})", self.points));
        }
        let n = self.points.iter().try_fold(1usize, |a, &p| a.checked_mul(p));
        let Some(n) = n.filter(|n| n.checked_mul(8).is_some()) else {
            return err("grid too large");
        };
        if !self.origin.iter().all(|v| v.is_finite()) {
            return err(format!("origin must be finite (origin {:?})", self.origin));
        }
        if !self.spacing.iter().all(|v| v.is_finite() && *v > 0.0) {
            return err(format!("spacing must be finite and positive (spacing {:?})", self.spacing));
        }
        if self.arrays.is_empty() {
            return err("no arrays to write");
        }
        for (i, a) in self.arrays.iter().enumerate() {
            let ok = !a.name.is_empty()
                && a.name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-' || b == b'.');
            if !ok {
                return err(format!("array name {:?}: use ASCII letters, digits, '_', '-' and '.'", a.name));
            }
            if self.arrays[..i].iter().any(|b| b.name == a.name) {
                return err(format!("array name {:?} used twice", a.name));
            }
            if a.values.len() != n {
                return err(format!("array {:?} has {} values, the grid has {} points", a.name, a.values.len(), n));
            }
        }
        Ok(())
    }

    fn extent(&self) -> String {
        let [nx, ny, nz] = self.points;
        format!("0 {} 0 {} 0 {}", nx - 1, ny - 1, nz - 1)
    }

    /// The `.vti` file as bytes (layout in the module documentation).
    pub fn to_vti_bytes(&self) -> Result<Vec<u8>, VtiError> {
        self.check()?;
        let extent = self.extent();
        let mut x = String::new();
        x.push_str("<?xml version=\"1.0\"?>\n");
        x.push_str("<VTKFile type=\"ImageData\" version=\"1.0\" byte_order=\"LittleEndian\" header_type=\"UInt64\">\n");
        let _ = writeln!(
            x,
            "  <ImageData WholeExtent=\"{extent}\" Origin=\"{}\" Spacing=\"{}\">",
            triple(&self.origin),
            triple(&self.spacing)
        );
        let _ = writeln!(x, "    <Piece Extent=\"{extent}\">");
        let _ = writeln!(x, "      <PointData Scalars=\"{}\">", self.arrays[0].name);
        let mut offset = 0usize;
        for a in &self.arrays {
            let _ = writeln!(
                x,
                "        <DataArray type=\"Float64\" Name=\"{}\" NumberOfComponents=\"1\" format=\"appended\" offset=\"{offset}\"/>",
                a.name
            );
            offset += 8 + 8 * a.values.len();
        }
        x.push_str("      </PointData>\n      <CellData>\n      </CellData>\n    </Piece>\n  </ImageData>\n");
        x.push_str("  <AppendedData encoding=\"raw\">\n   _");
        let mut out = x.into_bytes();
        out.reserve(offset + 40);
        for a in &self.arrays {
            out.extend_from_slice(&((8 * a.values.len()) as u64).to_le_bytes());
            for v in &a.values {
                out.extend_from_slice(&v.to_bits().to_le_bytes());
            }
        }
        out.extend_from_slice(b"\n  </AppendedData>\n</VTKFile>\n");
        Ok(out)
    }

    /// Writes the `.vti` file.
    pub fn write_vti(&self, path: impl AsRef<Path>) -> Result<(), VtiError> {
        let bytes = self.to_vti_bytes()?;
        std::fs::write(path.as_ref(), bytes).map_err(|e| VtiError(format!("cannot write {}: {e}", path.as_ref().display())))
    }

    /// Reads a `.vti` file written by [`ImageData::to_vti_bytes`].
    pub fn read_vti(path: impl AsRef<Path>) -> Result<ImageData, VtiError> {
        let bytes =
            std::fs::read(path.as_ref()).map_err(|e| VtiError(format!("cannot read {}: {e}", path.as_ref().display())))?;
        ImageData::from_vti_bytes(&bytes)
    }

    /// Parses the layout that [`ImageData::to_vti_bytes`] writes; anything else is an error.
    pub fn from_vti_bytes(bytes: &[u8]) -> Result<ImageData, VtiError> {
        const START: &[u8] = b"<AppendedData encoding=\"raw\">";
        let Some(at) = find(bytes, START) else { return err("no raw appended data block") };
        let Some(us) = bytes[at + START.len()..].iter().position(|&b| b == b'_') else {
            return err("appended data has no '_' marker");
        };
        if !bytes[at + START.len()..at + START.len() + us].iter().all(|b| b.is_ascii_whitespace()) {
            return err("unexpected text before the '_' marker");
        }
        let base = at + START.len() + us + 1;
        let head = std::str::from_utf8(&bytes[..at]).map_err(|_| VtiError("header is not UTF-8".into()))?;

        let tags = tags(head);
        let file = expect_tag(&tags, 0, "?xml")?;
        if attr(file, "version")? != "1.0" {
            return err("expected <?xml version=\"1.0\"?>");
        }
        let vtk = expect_tag(&tags, 1, "VTKFile")?;
        for (k, v) in [("type", "ImageData"), ("version", "1.0"), ("byte_order", "LittleEndian"), ("header_type", "UInt64")] {
            if attr(vtk, k)? != v {
                return err(format!("VTKFile {k} must be {v:?}"));
            }
        }
        let image = expect_tag(&tags, 2, "ImageData")?;
        let whole = parse_extent(attr(image, "WholeExtent")?)?;
        let origin = parse_triple(attr(image, "Origin")?)?;
        let spacing = parse_triple(attr(image, "Spacing")?)?;
        let piece = expect_tag(&tags, 3, "Piece")?;
        if parse_extent(attr(piece, "Extent")?)? != whole {
            return err("Piece Extent differs from WholeExtent");
        }
        let point_data = expect_tag(&tags, 4, "PointData")?;
        let scalars = attr(point_data, "Scalars")?;

        let mut d = ImageData { points: whole, origin, spacing, arrays: Vec::new() };
        let n = d.points.iter().try_fold(1usize, |a, &p| a.checked_mul(p)).ok_or(VtiError("grid too large".into()))?;
        let mut i = 5;
        let mut next_offset = 0usize;
        while let Some(t) = tags.get(i).filter(|t| t.starts_with("DataArray")) {
            if !t.ends_with('/') {
                return err("DataArray must be an empty element");
            }
            for (k, v) in [("type", "Float64"), ("NumberOfComponents", "1"), ("format", "appended")] {
                if attr(t, k)? != v {
                    return err(format!("DataArray {k} must be {v:?}"));
                }
            }
            let name = attr(t, "Name")?.to_string();
            let offset: usize = attr(t, "offset")?.parse().map_err(|_| VtiError("bad DataArray offset".into()))?;
            if offset != next_offset {
                return err(format!("array {name:?}: offset {offset}, expected {next_offset}"));
            }
            let start = base.checked_add(offset).filter(|s| s + 8 <= bytes.len()).ok_or(VtiError("offset past end".into()))?;
            let size = u64::from_le_bytes(bytes[start..start + 8].try_into().unwrap());
            if size != 8 * n as u64 {
                return err(format!("array {name:?}: {size} bytes, the grid has {n} points of 8 bytes"));
            }
            let data = start + 8;
            let end = data.checked_add(8 * n).filter(|e| *e <= bytes.len()).ok_or(VtiError("data past end".into()))?;
            let values =
                bytes[data..end].chunks_exact(8).map(|c| f64::from_bits(u64::from_le_bytes(c.try_into().unwrap()))).collect();
            d.arrays.push(PointArray { name, values });
            next_offset = end - base;
            i += 1;
        }
        for (k, want) in ["/PointData", "CellData", "/CellData", "/Piece", "/ImageData"].iter().enumerate() {
            if tags.get(i + k).map(|s| s.as_str()) != Some(*want) {
                return err(format!("expected <{want}>"));
            }
        }
        if tags.len() != i + 5 {
            return err("unexpected elements in the header");
        }
        let tail = &bytes[base + next_offset..];
        if tail != b"\n  </AppendedData>\n</VTKFile>\n" {
            return err("unexpected bytes after the appended data");
        }
        if d.arrays.first().map(|a| a.name.as_str()) != Some(scalars) {
            return err("PointData Scalars must name the first array");
        }
        d.check()?;
        Ok(d)
    }
}

/// Three numbers in Rust's shortest round-trip form (`Debug` of `f64`: `0.025`, `1e-300`, `-0.0`).
fn triple(v: &[f64; 3]) -> String {
    format!("{:?} {:?} {:?}", v[0], v[1], v[2])
}

fn parse_triple(s: &str) -> Result<[f64; 3], VtiError> {
    let v: Vec<f64> = s.split(' ').map(|t| t.parse::<f64>()).collect::<Result<_, _>>().map_err(|_| VtiError(format!("bad numbers {s:?}")))?;
    v.try_into().map_err(|_| VtiError(format!("expected three numbers, got {s:?}")))
}

/// `"0 NX-1 0 NY-1 0 NZ-1"` → point counts.
fn parse_extent(s: &str) -> Result<[usize; 3], VtiError> {
    let v: Vec<usize> = s.split(' ').map(|t| t.parse::<usize>()).collect::<Result<_, _>>().map_err(|_| VtiError(format!("bad extent {s:?}")))?;
    if v.len() != 6 || v[0] != 0 || v[2] != 0 || v[4] != 0 {
        return err(format!("extent must be \"0 NX-1 0 NY-1 0 NZ-1\", got {s:?}"));
    }
    Ok([v[1] + 1, v[3] + 1, v[5] + 1])
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// The text inside each `<...>` of the header (without the brackets), in order.
fn tags(head: &str) -> Vec<String> {
    head.split('<').skip(1).filter_map(|t| t.split_once('>').map(|(inner, _)| inner.trim().to_string())).collect()
}

fn expect_tag<'a>(tags: &'a [String], i: usize, name: &str) -> Result<&'a str, VtiError> {
    match tags.get(i) {
        Some(t) if t == name || t.starts_with(&format!("{name} ")) => Ok(t),
        _ => err(format!("expected <{name}> as element {i}")),
    }
}

/// The value of `key="value"` in a tag.
fn attr<'a>(tag: &'a str, key: &str) -> Result<&'a str, VtiError> {
    let pat = format!(" {key}=\"");
    let Some(p) = tag.find(&pat) else { return err(format!("attribute {key} missing")) };
    let rest = &tag[p + pat.len()..];
    let Some(q) = rest.find('"') else { return err(format!("attribute {key} not closed")) };
    Ok(&rest[..q])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// First byte after the `_` marker of the appended data.
    fn data_start(b: &[u8]) -> usize {
        let m = b"<AppendedData encoding=\"raw\">
   _";
        find(b, m).unwrap() + m.len()
    }

    fn same_bits(a: &[f64], b: &[f64]) -> bool {
        a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
    }

    /// Values that a text format or a careless conversion would change.
    fn awkward() -> Vec<f64> {
        vec![
            0.0,
            -0.0,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NAN,
            -f64::NAN,
            f64::from_bits(0x7FF0_0000_0000_0001), // signalling NaN with a payload
            f64::from_bits(0x7FF8_DEAD_BEEF_0001), // quiet NaN with a payload
            f64::MIN_POSITIVE,
            f64::from_bits(1), // smallest subnormal
            -f64::from_bits(0x000F_FFFF_FFFF_FFFF),
            f64::MAX,
            f64::MIN,
            f64::EPSILON,
            0.1,
            1.0 / 3.0,
        ]
    }

    fn round_trip(d: &ImageData) -> ImageData {
        let bytes = d.to_vti_bytes().unwrap();
        let r = ImageData::from_vti_bytes(&bytes).unwrap();
        assert_eq!(r.points, d.points);
        assert_eq!(r.arrays.len(), d.arrays.len());
        for k in 0..3 {
            assert_eq!(r.origin[k].to_bits(), d.origin[k].to_bits());
            assert_eq!(r.spacing[k].to_bits(), d.spacing[k].to_bits());
        }
        for (a, b) in r.arrays.iter().zip(&d.arrays) {
            assert_eq!(a.name, b.name);
            assert!(same_bits(&a.values, &b.values), "array {} changed", a.name);
        }
        // Writing what was read gives the same bytes.
        assert_eq!(r.to_vti_bytes().unwrap(), bytes);
        r
    }

    #[test]
    fn round_trip_2d_awkward_values() {
        let v = awkward();
        let mut d = ImageData::new_2d([4, 4], [-0.0, 1e-300], [0.025, 1.0 / 3.0]);
        d.add("u", v.clone()).add("u.rev", v.iter().rev().cloned().collect());
        let r = round_trip(&d);
        assert_eq!(r.points, [4, 4, 1]);
        assert!(r.array("u.rev").is_some());
    }

    #[test]
    fn round_trip_3d_pseudo_random_bits() {
        // Every bit pattern is allowed, NaN payloads included: fill with a simple 64-bit generator.
        let points = [5, 4, 3];
        let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut next = || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            f64::from_bits(s)
        };
        let a: Vec<f64> = (0..60).map(|_| next()).collect();
        let b: Vec<f64> = (0..60).map(|_| next()).collect();
        let mut d = ImageData::new_3d(points, [0.5, -2.0, 1e10], [0.1, 0.2, 0.30000000000000004]);
        d.add("phi", a).add("d", b);
        round_trip(&d);
    }

    #[test]
    fn single_point_axes() {
        let mut d = ImageData::new_3d([1, 1, 1], [0.0; 3], [1.0; 3]);
        d.add("x", vec![-0.0]);
        round_trip(&d);
    }

    #[test]
    fn header_and_layout() {
        let mut d = ImageData::new_2d([3, 2], [0.0, 0.0], [0.5, 0.5]);
        d.add("u", vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let b = d.to_vti_bytes().unwrap();
        let text = String::from_utf8_lossy(&b);
        assert!(text.contains("<ImageData WholeExtent=\"0 2 0 1 0 0\" Origin=\"0.0 0.0 0.0\" Spacing=\"0.5 0.5 0.5\">"));
        assert!(text.contains("offset=\"0\"/>"));
        // byte count (48) then the values, x fastest
        let at = data_start(&b);
        assert_eq!(u64::from_le_bytes(b[at..at + 8].try_into().unwrap()), 48);
        assert_eq!(f64::from_le_bytes(b[at + 8..at + 16].try_into().unwrap()), 1.0);
        assert_eq!(f64::from_le_bytes(b[at + 48..at + 56].try_into().unwrap()), 6.0);
    }

    #[test]
    fn rejects_bad_input() {
        let mut d = ImageData::new_2d([2, 2], [0.0, 0.0], [1.0, 1.0]);
        d.add("u", vec![0.0; 3]);
        assert!(d.to_vti_bytes().is_err()); // wrong length
        let mut d = ImageData::new_2d([2, 2], [0.0, 0.0], [0.0, 1.0]);
        d.add("u", vec![0.0; 4]);
        assert!(d.to_vti_bytes().is_err()); // zero spacing
        let mut d = ImageData::new_2d([2, 2], [f64::NAN, 0.0], [1.0, 1.0]);
        d.add("u", vec![0.0; 4]);
        assert!(d.to_vti_bytes().is_err()); // NaN origin
        let mut d = ImageData::new_2d([2, 2], [0.0, 0.0], [1.0, 1.0]);
        d.add("a b", vec![0.0; 4]);
        assert!(d.to_vti_bytes().is_err()); // name with a space
        let mut d = ImageData::new_2d([2, 2], [0.0, 0.0], [1.0, 1.0]);
        d.add("u", vec![0.0; 4]).add("u", vec![0.0; 4]);
        assert!(d.to_vti_bytes().is_err()); // duplicate name

        let mut d = ImageData::new_2d([2, 2], [0.0, 0.0], [1.0, 1.0]);
        d.add("u", vec![1.0; 4]);
        let good = d.to_vti_bytes().unwrap();
        assert!(ImageData::from_vti_bytes(&good[..good.len() - 1]).is_err()); // truncated
        let mut big_endian = good.clone();
        let p = find(&big_endian, b"LittleEndian").unwrap();
        big_endian[p..p + 12].copy_from_slice(b"BigEndian   ");
        assert!(ImageData::from_vti_bytes(&big_endian).is_err());
        let mut short = good.clone();
        let at = data_start(&short);
        short[at] = 24; // byte count no longer matches the grid
        assert!(ImageData::from_vti_bytes(&short).is_err());
    }

    #[test]
    fn file_round_trip() {
        let dir = std::env::temp_dir().join(format!("dyne-export-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("f.vti");
        let mut d = ImageData::new_3d([2, 3, 4], [0.0; 3], [0.25; 3]);
        d.add("u", (0..24).map(|i| i as f64 * 0.1).collect());
        d.write_vti(&path).unwrap();
        let r = ImageData::read_vti(&path).unwrap();
        assert_eq!(r, d);
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
