//! OpenFOAM cases: export of a scalar field as a case for `scalarTransportFoam`, and import of a
//! velocity field computed by OpenFOAM.
//!
//! The dictionaries follow the OpenFOAM.com (ESI, v2xxx) format. Nothing here runs OpenFOAM; the
//! files are written and read by hand, without dependencies.
//!
//! # Grid mapping: one OpenFOAM cell per Dyne grid point
//!
//! Dyne keeps one value per grid *point*; OpenFOAM's finite-volume solvers keep one value per *cell*
//! (at its centre). Each Dyne grid point becomes the centre of one OpenFOAM cell: a grid of
//! `NX × NY × NZ` points with spacing `h` becomes one `blockMesh` block of `NX × NY × NZ` cells of size
//! `h`, extending half a spacing beyond the first and the last point on every axis:
//!
//! ```text
//! x_min = x0 − h/2,   x_max = x0 + (NX − 1/2)·h        (same for y and z)
//! ```
//!
//! - Every value is carried over one to one, without interpolation, and `blockMesh` numbers the cells
//!   of a single block with x fastest, then y, then z: the order of Dyne's fields. The values in the
//!   files are therefore exactly Dyne's values.
//! - The boundary of the OpenFOAM domain lies half a spacing outside Dyne's edge points. A fixed value
//!   on a face is applied there, not at the edge points; this is one of the differences of
//!   discretisation that make a comparison of results need a tolerance.
//! - The cell centres that OpenFOAM computes from the vertices equal Dyne's point coordinates up to
//!   rounding (a few units in the last place), not bit for bit.
//!
//! A grid with one point along z (`NZ = 1`) is written as a 2D case: one cell thick, with the front
//! and back faces in one patch `frontAndBack` of type `empty`.
//!
//! # Files of an exported case
//!
//! | File | Content |
//! |---|---|
//! | `system/blockMeshDict` | one hex block; patches `west`, `east` (x), `south`, `north` (y), `bottom`, `top` (z) or `frontAndBack` (2D) |
//! | `system/controlDict` | `scalarTransportFoam`; ASCII output with 17 significant digits |
//! | `system/fvSchemes`, `system/fvSolution` | Euler in time, `linearUpwind` advection, central diffusion; PBiCGStab |
//! | `system/dyneGrid` | Dyne's grid exactly (point counts, origin, spacing), for reading the case back |
//! | `system/mapFieldsDict` | for `mapFields` from a CFD case: no patch mapping, every face a cutting patch |
//! | `constant/transportProperties` | diffusivity `DT` |
//! | `0/T` | the scalar field (`nonuniform List<scalar>`) and the condition on each face |
//! | `0/U` | the given velocity (`uniform` or `nonuniform List<vector>`), `zeroGradient` on every face |
//!
//! Values are written in Rust's shortest round-trip decimal form (`0.1`, `1e-5`, `-0.0`), which reads
//! back to the same 64-bit float with any correctly rounded decimal reader (Rust's, glibc's
//! `strtod`). Non-finite values are rejected.
//!
//! None of these files has been run through OpenFOAM in Dyne's tests (CI checks only their format and
//! that they read back). Differences in the Foundation releases (openfoam.org) to watch for, not
//! checked here: `blockMeshDict` is documented there with `convertToMeters` instead of `scale`;
//! recent releases moved solvers into modules run by `foamRun` and renamed `transportProperties` to
//! `physicalProperties` for many solvers, so `scalarTransportFoam` may need to be replaced by the
//! equivalent function or module there.
//!
//! # Importing a velocity field
//!
//! [`import_velocity`] reads the `internalField` of a `volVectorField` `U` (ASCII) in a case whose mesh
//! is the one exported here, so that cell `n` is Dyne's grid point `n`. The intended route maps the
//! velocity of a CFD computation (any mesh covering the same region, e.g. a `simpleFoam` run in
//! `roomFlow`) onto Dyne's grid with OpenFOAM's `mapFields`:
//!
//! ```text
//! dyne-foam case --vti field.vti --out flowGrid   # a case with Dyne's grid (its T is not needed)
//! cd flowGrid
//! blockMesh                                       # Dyne's grid as an OpenFOAM mesh
//! mapFields ../roomFlow -sourceTime latestTime    # interpolates the CFD fields into flowGrid/0
//! cd ..
//! dyne-foam velocity flowGrid 0 --vti flow.vti    # one velocity vector per Dyne grid point
//! ```
//!
//! `mapFields` uses `system/mapFieldsDict` of the target case (written here: the faces of Dyne's
//! block are cutting patches, so their values are interpolated from the CFD field) and writes with the
//! target's `controlDict` settings (ASCII, 17 digits). It maps the other fields of the CFD case
//! (e.g. `p`, or a `T`) too, which is why a separate case is used. Cells of Dyne's grid outside the CFD
//! mesh keep the exported velocity (zero by default). A run directly on the exported mesh works as
//! well: then read its last time directory. Uniform fields
//! (`uniform (a b c)`), lists (`nonuniform List<vector> N ( ... )`) and OpenFOAM's compact forms
//! (`N{(a b c)}`, `N((a b c) ...)`) are read. Binary or compressed files, `#include` and `$variable`
//! are not supported. The decimal text is parsed with correct rounding, so the imported values are
//! exactly the numbers in the file; with `writePrecision 17` they are OpenFOAM's 64-bit values.

use std::fmt;
use std::fmt::Write as _;
use std::path::Path;

/// An error while building, writing or reading OpenFOAM files.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FoamError(pub String);

impl fmt::Display for FoamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for FoamError {}

fn err<T>(msg: impl Into<String>) -> Result<T, FoamError> {
    Err(FoamError(msg.into()))
}

/// Patch names of the faces, in Dyne's face order `2·axis + side`.
pub const FACE_NAMES: [&str; 6] = ["west", "east", "south", "north", "bottom", "top"];

/// Patch name of the front and back faces of a 2D case (type `empty`).
pub const EMPTY_PATCH: &str = "frontAndBack";

/// Vertex indices of each face in `blockMeshDict`, ordered so that the normal points outwards.
const FACE_VERTICES: [[usize; 4]; 6] = [[0, 4, 7, 3], [2, 6, 5, 1], [1, 5, 4, 0], [3, 7, 6, 2], [0, 3, 2, 1], [4, 5, 6, 7]];

/// A regular grid of Dyne's points: counts, first point and spacing. `points[2] == 1` means 2D.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Grid {
    /// Number of points along x, y and z (= number of OpenFOAM cells along each axis).
    pub points: [usize; 3],
    /// Coordinates of the first point (= centre of the first cell).
    pub origin: [f64; 3],
    /// Distance between neighbouring points (= cell size) along each axis, positive.
    pub spacing: [f64; 3],
}

impl Grid {
    /// A 2D grid; the case is one cell thick along z, of thickness `spacing[0]`, centred on z = 0
    /// (the convention of [`crate::vti::ImageData::new_2d`]).
    pub fn new_2d(points: [usize; 2], origin: [f64; 2], spacing: [f64; 2]) -> Grid {
        Grid { points: [points[0], points[1], 1], origin: [origin[0], origin[1], 0.0], spacing: [spacing[0], spacing[1], spacing[0]] }
    }

    /// A 3D grid.
    pub fn new_3d(points: [usize; 3], origin: [f64; 3], spacing: [f64; 3]) -> Grid {
        Grid { points, origin, spacing }
    }

    /// True if the case is 2D (one point along z).
    pub fn is_2d(&self) -> bool {
        self.points[2] == 1
    }

    /// Number of cells (= grid points).
    pub fn cells(&self) -> usize {
        self.points.iter().product()
    }

    /// Patch names of the faces that carry a condition: four in 2D, six in 3D.
    pub fn face_names(&self) -> &'static [&'static str] {
        if self.is_2d() {
            &FACE_NAMES[..4]
        } else {
            &FACE_NAMES
        }
    }

    /// The eight vertices of the block (half a spacing outside the first and last points), in
    /// `blockMesh` order: 0 = (min, min, min), 1 = (max, min, min), 2 = (max, max, min),
    /// 3 = (min, max, min), then 4..7 the same at max z.
    pub fn vertices(&self) -> [[f64; 3]; 8] {
        let lo: [f64; 3] = std::array::from_fn(|a| self.origin[a] - 0.5 * self.spacing[a]);
        let hi: [f64; 3] = std::array::from_fn(|a| self.origin[a] + (self.points[a] as f64 - 0.5) * self.spacing[a]);
        let corner = |x: bool, y: bool, z: bool| [if x { hi[0] } else { lo[0] }, if y { hi[1] } else { lo[1] }, if z { hi[2] } else { lo[2] }];
        [
            corner(false, false, false),
            corner(true, false, false),
            corner(true, true, false),
            corner(false, true, false),
            corner(false, false, true),
            corner(true, false, true),
            corner(true, true, true),
            corner(false, true, true),
        ]
    }

    fn check(&self) -> Result<(), FoamError> {
        if self.points.contains(&0) {
            return err(format!("every axis needs at least one point (points {:?})", self.points));
        }
        if self.points.iter().try_fold(1usize, |a, &p| a.checked_mul(p)).is_none() {
            return err("grid too large");
        }
        if !self.origin.iter().all(|v| v.is_finite()) {
            return err(format!("origin must be finite (origin {:?})", self.origin));
        }
        if !self.spacing.iter().all(|v| v.is_finite() && *v > 0.0) {
            return err(format!("spacing must be finite and positive (spacing {:?})", self.spacing));
        }
        if !self.vertices().iter().flatten().all(|v| v.is_finite()) {
            return err("grid extent is not finite");
        }
        Ok(())
    }
}

/// Condition of the scalar field on one face of the domain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Face {
    /// Fixed value (`fixedValue`; Dyne's fixed-value / Dirichlet face).
    FixedValue(f64),
    /// Zero normal gradient (`zeroGradient`; Dyne's zero-gradient / Neumann face).
    ZeroGradient,
    /// Outflow where the flow leaves, fixed value where it enters (`inletOutlet` with this
    /// `inletValue`). Dyne has no such face yet; it suits open faces in a given airflow.
    InletOutlet(f64),
}

/// The given velocity of the case (`0/U`).
#[derive(Debug, Clone, PartialEq)]
pub enum Velocity {
    /// The same vector in every cell (`uniform (ux uy uz)`).
    Uniform([f64; 3]),
    /// One vector per cell, in the order of the grid (x fastest).
    Cells(Vec<[f64; 3]>),
}

/// An OpenFOAM case for `scalarTransportFoam`: a scalar field `T` on a regular grid, carried by a
/// given velocity `U` and diffusing with diffusivity `DT`.
#[derive(Debug, Clone, PartialEq)]
pub struct Case {
    /// The grid (one cell per point).
    pub grid: Grid,
    /// The scalar field, one value per point, x fastest.
    pub t: Vec<f64>,
    /// Physical dimensions of `T` as OpenFOAM's seven exponents (mass, length, time, temperature,
    /// amount, current, luminous intensity). Default: temperature `[0 0 0 1 0 0 0]`.
    pub t_dimensions: [i32; 7],
    /// Condition on each face in the order of [`Grid::face_names`] (four in 2D, six in 3D).
    pub faces: Vec<Face>,
    /// The given velocity.
    pub velocity: Velocity,
    /// Diffusivity `DT` in m²/s.
    pub diffusivity: f64,
    /// `endTime` of the run in seconds.
    pub end_time: f64,
    /// Time step `deltaT` in seconds.
    pub delta_t: f64,
    /// Interval between written time directories (`writeInterval`, `writeControl runTime`).
    pub write_interval: f64,
}

impl Case {
    /// A case with zero-gradient faces, zero velocity, `endTime 1`, `deltaT 0.01` and
    /// `writeInterval 0.1`; change the fields to suit the study.
    pub fn new(grid: Grid, t: Vec<f64>, diffusivity: f64) -> Case {
        let n = if grid.is_2d() { 4 } else { 6 };
        Case {
            grid,
            t,
            t_dimensions: [0, 0, 0, 1, 0, 0, 0],
            faces: vec![Face::ZeroGradient; n],
            velocity: Velocity::Uniform([0.0; 3]),
            diffusivity,
            end_time: 1.0,
            delta_t: 0.01,
            write_interval: 0.1,
        }
    }

    fn check(&self) -> Result<(), FoamError> {
        self.grid.check()?;
        let n = self.grid.cells();
        if self.t.len() != n {
            return err(format!("T has {} values, the grid has {} points", self.t.len(), n));
        }
        if let Some(k) = self.t.iter().position(|v| !v.is_finite()) {
            return err(format!("T[{k}] = {:?} is not finite; OpenFOAM cannot use it", self.t[k]));
        }
        if self.faces.len() != self.grid.face_names().len() {
            return err(format!("{} face conditions given, the grid has {} faces", self.faces.len(), self.grid.face_names().len()));
        }
        for (f, name) in self.faces.iter().zip(self.grid.face_names()) {
            if let Face::FixedValue(v) | Face::InletOutlet(v) = f {
                if !v.is_finite() {
                    return err(format!("face {name}: value {v:?} is not finite"));
                }
            }
        }
        match &self.velocity {
            Velocity::Uniform(u) if !u.iter().all(|v| v.is_finite()) => return err("velocity is not finite"),
            Velocity::Cells(c) if c.len() != n => return err(format!("U has {} vectors, the grid has {} points", c.len(), n)),
            Velocity::Cells(c) if !c.iter().flatten().all(|v| v.is_finite()) => return err("velocity is not finite"),
            _ => {}
        }
        if !(self.diffusivity.is_finite() && self.diffusivity >= 0.0) {
            return err(format!("diffusivity must be finite and not negative ({:?})", self.diffusivity));
        }
        for (name, v) in [("endTime", self.end_time), ("deltaT", self.delta_t), ("writeInterval", self.write_interval)] {
            if !(v.is_finite() && v > 0.0) {
                return err(format!("{name} must be finite and positive ({v:?})"));
            }
        }
        Ok(())
    }

    /// The files of the case as (path relative to the case directory, contents).
    pub fn files(&self) -> Result<Vec<(&'static str, String)>, FoamError> {
        self.check()?;
        Ok(vec![
            ("system/blockMeshDict", self.block_mesh_dict()),
            ("system/controlDict", self.control_dict()),
            ("system/fvSchemes", fv_schemes()),
            ("system/fvSolution", fv_solution()),
            ("system/dyneGrid", grid_dict(&self.grid)),
            ("system/mapFieldsDict", self.map_fields_dict()),
            ("constant/transportProperties", self.transport_properties()),
            ("0/T", self.t_file()),
            ("0/U", self.u_file()),
        ])
    }

    /// Writes the case into directory `dir` (created if needed; existing files are replaced).
    pub fn write(&self, dir: impl AsRef<Path>) -> Result<(), FoamError> {
        let dir = dir.as_ref();
        for (rel, text) in self.files()? {
            let path = dir.join(rel);
            if let Some(p) = path.parent() {
                std::fs::create_dir_all(p).map_err(|e| FoamError(format!("cannot create {}: {e}", p.display())))?;
            }
            std::fs::write(&path, text).map_err(|e| FoamError(format!("cannot write {}: {e}", path.display())))?;
        }
        Ok(())
    }

    /// Reads a case written by [`Case::write`] (all nine files) and checks that they agree with each
    /// other: the mesh of `blockMeshDict` must be the one of `system/dyneGrid`, and `T` and `U` must
    /// have one value per cell and the expected patches.
    pub fn read(dir: impl AsRef<Path>) -> Result<Case, FoamError> {
        let dir = dir.as_ref();
        let grid = read_grid(dir)?;
        let n = grid.cells();

        let bm = read_foam(&read_text(&dir.join("system/blockMeshDict"))?, "dictionary")?;
        check_block_mesh(&bm, &grid)?;

        let cd = read_foam(&read_text(&dir.join("system/controlDict"))?, "dictionary")?;
        if cd.word("application")? != "scalarTransportFoam" {
            return err("controlDict: application must be scalarTransportFoam");
        }
        let end_time = num(cd.word("endTime")?)?;
        let delta_t = num(cd.word("deltaT")?)?;
        let write_interval = num(cd.word("writeInterval")?)?;
        read_foam(&read_text(&dir.join("system/fvSchemes"))?, "dictionary")?;
        read_foam(&read_text(&dir.join("system/fvSolution"))?, "dictionary")?;
        let mf = read_foam(&read_text(&dir.join("system/mapFieldsDict"))?, "dictionary")?;
        let cutting: Vec<&Tok> = mf.stream("cuttingPatches")?.iter().filter(|t| matches!(t, Tok::W(_))).collect();
        if cutting.len() != grid.face_names().len() {
            return err("mapFieldsDict: cuttingPatches must name every face");
        }

        let tp = read_foam(&read_text(&dir.join("constant/transportProperties"))?, "dictionary")?;
        let dt = tp.stream("DT")?;
        let diffusivity = match dt {
            [Tok::P(b'['), dims @ .., Tok::W(v)] if dims.last() == Some(&Tok::P(b']')) => {
                if dimensions(&dt[..dt.len() - 1])? != [0, 2, -1, 0, 0, 0, 0] {
                    return err("transportProperties: DT must have dimensions [0 2 -1 0 0 0 0]");
                }
                num(v)?
            }
            _ => return err("transportProperties: expected DT [0 2 -1 0 0 0 0] value;"),
        };

        let tf = read_foam(&read_text(&dir.join("0/T"))?, "volScalarField")?;
        let t_dimensions = dimensions(tf.stream("dimensions")?)?;
        let t = match field(tf.stream("internalField")?, "scalar", scalar)? {
            Internal::Values(v) => v,
            Internal::Uniform(v) => vec![v; n],
        };
        if t.len() != n {
            return err(format!("0/T has {} values, the mesh has {} cells", t.len(), n));
        }
        let bf = tf.dict("boundaryField")?;
        check_patch_names(bf, &grid, "0/T")?;
        let mut faces = Vec::new();
        for name in grid.face_names() {
            let p = bf.dict(name)?;
            let face = match p.word("type")? {
                "fixedValue" => Face::FixedValue(uniform_scalar(p.stream("value")?)?),
                "zeroGradient" => Face::ZeroGradient,
                "inletOutlet" => {
                    let v = uniform_scalar(p.stream("inletValue")?)?;
                    if uniform_scalar(p.stream("value")?)?.to_bits() != v.to_bits() {
                        return err(format!("0/T {name}: value differs from inletValue"));
                    }
                    Face::InletOutlet(v)
                }
                other => return err(format!("0/T {name}: unsupported type {other}")),
            };
            faces.push(face);
        }

        let uf = read_foam(&read_text(&dir.join("0/U"))?, "volVectorField")?;
        if dimensions(uf.stream("dimensions")?)? != [0, 1, -1, 0, 0, 0, 0] {
            return err("0/U: dimensions must be [0 1 -1 0 0 0 0]");
        }
        let velocity = match field(uf.stream("internalField")?, "vector", vector)? {
            Internal::Uniform(u) => Velocity::Uniform(u),
            Internal::Values(c) => Velocity::Cells(c),
        };
        check_patch_names(uf.dict("boundaryField")?, &grid, "0/U")?;

        let case = Case { grid, t, t_dimensions, faces, velocity, diffusivity, end_time, delta_t, write_interval };
        case.check()?;
        Ok(case)
    }

    fn block_mesh_dict(&self) -> String {
        let g = &self.grid;
        let mut s = header("dictionary", "system", "blockMeshDict");
        s.push_str("// One block of cells; the centre of each cell is one point of Dyne's grid (see system/dyneGrid).\n\n");
        s.push_str("scale   1;\n\nvertices\n(\n");
        for v in g.vertices() {
            let _ = writeln!(s, "    {}", vec3(&v));
        }
        let [nx, ny, nz] = g.points;
        let _ = write!(s, ");\n\nblocks\n(\n    hex (0 1 2 3 4 5 6 7) ({nx} {ny} {nz}) simpleGrading (1 1 1)\n);\n\nedges\n(\n);\n\nboundary\n(\n");
        let quad = |f: usize| {
            let q = FACE_VERTICES[f];
            format!("({} {} {} {})", q[0], q[1], q[2], q[3])
        };
        for (f, name) in g.face_names().iter().enumerate() {
            let _ = write!(s, "    {name}\n    {{\n        type patch;\n        faces\n        (\n            {}\n        );\n    }}\n", quad(f));
        }
        if g.is_2d() {
            let _ = write!(
                s,
                "    {EMPTY_PATCH}\n    {{\n        type empty;\n        faces\n        (\n            {}\n            {}\n        );\n    }}\n",
                quad(4),
                quad(5)
            );
        }
        s.push_str(");\n\nmergePatchPairs\n(\n);\n");
        s
    }

    fn control_dict(&self) -> String {
        let mut s = header("dictionary", "system", "controlDict");
        let _ = write!(
            s,
            "application     scalarTransportFoam;\n\
             \n\
             startFrom       startTime;\n\
             startTime       0;\n\
             stopAt          endTime;\n\
             endTime         {};\n\
             deltaT          {};\n\
             \n\
             writeControl    runTime;\n\
             writeInterval   {};\n\
             purgeWrite      0;\n\
             // ASCII with 17 significant digits: every 64-bit value is written exactly.\n\
             writeFormat     ascii;\n\
             writePrecision  17;\n\
             writeCompression off;\n\
             timeFormat      general;\n\
             timePrecision   6;\n\
             runTimeModifiable true;\n",
            num_str(self.end_time),
            num_str(self.delta_t),
            num_str(self.write_interval)
        );
        s
    }

    fn map_fields_dict(&self) -> String {
        let mut s = header("dictionary", "system", "mapFieldsDict");
        s.push_str("// For mapFields from a CFD case: the faces of Dyne's block take values interpolated from the
");
        s.push_str("// source field instead of being mapped from source patches.
");
        let _ = write!(s, "patchMap        ( );

cuttingPatches  ( {} );
", self.grid.face_names().join(" "));
        s
    }

    fn transport_properties(&self) -> String {
        let mut s = header("dictionary", "constant", "transportProperties");
        let _ = writeln!(s, "DT              [0 2 -1 0 0 0 0] {};", num_str(self.diffusivity));
        s
    }

    fn t_file(&self) -> String {
        let mut s = header("volScalarField", "0", "T");
        let d = self.t_dimensions;
        let _ = write!(
            s,
            "dimensions      [{} {} {} {} {} {} {}];\n\ninternalField   nonuniform List<scalar>\n{}\n(\n",
            d[0],
            d[1],
            d[2],
            d[3],
            d[4],
            d[5],
            d[6],
            self.t.len()
        );
        for v in &self.t {
            s.push_str(&num_str(*v));
            s.push('\n');
        }
        s.push_str(")\n;\n\nboundaryField\n{\n");
        for (f, name) in self.faces.iter().zip(self.grid.face_names()) {
            let body = match f {
                Face::FixedValue(v) => format!("        type            fixedValue;\n        value           uniform {};\n", num_str(*v)),
                Face::ZeroGradient => "        type            zeroGradient;\n".to_string(),
                Face::InletOutlet(v) => format!(
                    "        type            inletOutlet;\n        inletValue      uniform {0};\n        value           uniform {0};\n",
                    num_str(*v)
                ),
            };
            let _ = write!(s, "    {name}\n    {{\n{body}    }}\n");
        }
        empty_patch(&mut s, &self.grid);
        s.push_str("}\n");
        s
    }

    fn u_file(&self) -> String {
        let mut s = header("volVectorField", "0", "U");
        s.push_str("dimensions      [0 1 -1 0 0 0 0];\n\n");
        match &self.velocity {
            Velocity::Uniform(u) => {
                let _ = writeln!(s, "internalField   uniform {};", vec3(u));
            }
            Velocity::Cells(c) => {
                let _ = write!(s, "internalField   nonuniform List<vector>\n{}\n(\n", c.len());
                for u in c {
                    s.push_str(&vec3(u));
                    s.push('\n');
                }
                s.push_str(")\n;\n");
            }
        }
        s.push_str("\nboundaryField\n{\n");
        for name in self.grid.face_names() {
            let _ = write!(s, "    {name}\n    {{\n        type            zeroGradient;\n    }}\n");
        }
        empty_patch(&mut s, &self.grid);
        s.push_str("}\n");
        s
    }
}

fn empty_patch(s: &mut String, g: &Grid) {
    if g.is_2d() {
        let _ = write!(s, "    {EMPTY_PATCH}\n    {{\n        type            empty;\n    }}\n");
    }
}

fn header(class: &str, location: &str, object: &str) -> String {
    format!(
        "/* Written by Dyne (crates/dyne-export, https://github.com/physical-edge-native/dyne).\n   \
         Numbers are in the shortest decimal form that reads back to the same 64-bit float. */\n\
         FoamFile\n{{\n    version     2.0;\n    format      ascii;\n    class       {class};\n    \
         location    \"{location}\";\n    object      {object};\n}}\n\n"
    )
}

fn fv_schemes() -> String {
    let mut s = header("dictionary", "system", "fvSchemes");
    s.push_str(
        "ddtSchemes\n{\n    default         Euler;\n}\n\n\
         gradSchemes\n{\n    default         Gauss linear;\n}\n\n\
         divSchemes\n{\n    default         none;\n    div(phi,T)      Gauss linearUpwind grad(T);\n}\n\n\
         laplacianSchemes\n{\n    default         Gauss linear corrected;\n}\n\n\
         interpolationSchemes\n{\n    default         linear;\n}\n\n\
         snGradSchemes\n{\n    default         corrected;\n}\n",
    );
    s
}

fn fv_solution() -> String {
    let mut s = header("dictionary", "system", "fvSolution");
    s.push_str(
        "solvers\n{\n    T\n    {\n        solver          PBiCGStab;\n        preconditioner  DILU;\n        \
         tolerance       1e-10;\n        relTol          0;\n    }\n}\n\n\
         SIMPLE\n{\n    nNonOrthogonalCorrectors 0;\n}\n",
    );
    s
}

fn grid_dict(g: &Grid) -> String {
    let mut s = header("dictionary", "system", "dyneGrid");
    s.push_str("// Dyne's grid: the centre of OpenFOAM cell n is Dyne's grid point n (x fastest, then y, then z).\n");
    s.push_str("// Read by Dyne when the case is read back or a velocity field is imported; OpenFOAM ignores it.\n");
    let _ = write!(
        s,
        "points          ({} {} {});\norigin          {};\nspacing         {};\n",
        g.points[0],
        g.points[1],
        g.points[2],
        vec3(&g.origin),
        vec3(&g.spacing)
    );
    s
}

/// Shortest round-trip decimal form (`Debug` of `f64`).
fn num_str(v: f64) -> String {
    format!("{v:?}")
}

fn vec3(v: &[f64; 3]) -> String {
    format!("({} {} {})", num_str(v[0]), num_str(v[1]), num_str(v[2]))
}

// ---------------------------------------------------------------------------------------------------
// Import

/// Reads `system/dyneGrid` of a case exported by [`Case::write`].
pub fn read_grid(case_dir: impl AsRef<Path>) -> Result<Grid, FoamError> {
    let d = read_foam(&read_text(&case_dir.as_ref().join("system/dyneGrid"))?, "dictionary")?;
    let p = labels3(d.stream("points")?)?;
    let mut i = 0;
    let origin = vector(d.stream("origin")?, &mut i)?;
    let mut i = 0;
    let spacing = vector(d.stream("spacing")?, &mut i)?;
    let g = Grid { points: p, origin, spacing };
    g.check()?;
    Ok(g)
}

/// Reads the velocity `U` of time directory `time` (e.g. `"0"` after `mapFields`, or the last time of
/// a run on this mesh) in a case exported by [`Case::write`], and returns Dyne's grid and one vector
/// per grid point (x fastest). A uniform field is repeated for every point.
pub fn import_velocity(case_dir: impl AsRef<Path>, time: &str) -> Result<(Grid, Vec<[f64; 3]>), FoamError> {
    let dir = case_dir.as_ref();
    let grid = read_grid(dir)?;
    let path = dir.join(time).join("U");
    let u = read_vector_field(&read_text(&path)?).map_err(|e| FoamError(format!("{}: {e}", path.display())))?;
    let n = grid.cells();
    let u = match u {
        Internal::Uniform(v) => vec![v; n],
        Internal::Values(v) if v.len() == n => v,
        Internal::Values(v) => {
            return err(format!("{}: {} vectors, the grid of system/dyneGrid has {} points", path.display(), v.len(), n));
        }
    };
    Ok((grid, u))
}

/// The `internalField` of a field file.
#[derive(Debug, Clone, PartialEq)]
pub enum Internal<T> {
    /// `uniform VALUE`.
    Uniform(T),
    /// `nonuniform List<...> N (...)`: one value per cell.
    Values(Vec<T>),
}

/// Parses the `internalField` of an ASCII `volVectorField` file (text of the file).
pub fn read_vector_field(text: &str) -> Result<Internal<[f64; 3]>, FoamError> {
    let d = read_foam(text, "volVectorField")?;
    field(d.stream("internalField")?, "vector", vector)
}

/// Parses the `internalField` of an ASCII `volScalarField` file (text of the file).
pub fn read_scalar_field(text: &str) -> Result<Internal<f64>, FoamError> {
    let d = read_foam(text, "volScalarField")?;
    field(d.stream("internalField")?, "scalar", scalar)
}

fn read_text(path: &Path) -> Result<String, FoamError> {
    std::fs::read_to_string(path).map_err(|e| FoamError(format!("cannot read {}: {e}", path.display())))
}

// ---------------------------------------------------------------------------------------------------
// A small reader of OpenFOAM's dictionary format (the subset that this module writes and that
// OpenFOAM writes for fields): comments, words, numbers, strings, ( ) { } [ ] ;

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    /// A word, number or quoted string (quotes kept).
    W(String),
    /// One of `( ) { } [ ] ;`.
    P(u8),
}

fn lex(s: &str) -> Result<Vec<Tok>, FoamError> {
    let b = s.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c.is_ascii_whitespace() {
            i += 1;
        } else if c == b'/' && b.get(i + 1) == Some(&b'/') {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else if c == b'/' && b.get(i + 1) == Some(&b'*') {
            let Some(p) = b[i + 2..].windows(2).position(|w| w == b"*/") else { return err("comment not closed") };
            i += 2 + p + 2;
        } else if b"(){}[];".contains(&c) {
            out.push(Tok::P(c));
            i += 1;
        } else if c == b'"' {
            let Some(p) = b[i + 1..].iter().position(|&x| x == b'"') else { return err("string not closed") };
            out.push(Tok::W(s[i..i + p + 2].to_string()));
            i += p + 2;
        } else {
            // A number stops at '('; a word may contain balanced parentheses, e.g. div(phi,T).
            let number = c.is_ascii_digit() || c == b'-' || c == b'+' || c == b'.';
            let start = i;
            let mut depth = 0usize;
            while i < b.len() {
                let c = b[i];
                let comment = c == b'/' && matches!(b.get(i + 1), Some(b'/' | b'*'));
                if c.is_ascii_whitespace() || comment || b"\"{}[];".contains(&c) {
                    break;
                }
                if c == b'(' {
                    if number {
                        break;
                    }
                    depth += 1;
                } else if c == b')' {
                    if depth == 0 {
                        break;
                    }
                    depth -= 1;
                }
                i += 1;
            }
            if depth != 0 {
                return err(format!("unbalanced parentheses in {:?}", &s[start..i]));
            }
            out.push(Tok::W(s[start..i].to_string()));
        }
    }
    Ok(out)
}

#[derive(Debug, Clone, PartialEq)]
enum Entry {
    Dict(Dict),
    Stream(Vec<Tok>),
}

#[derive(Debug, Clone, PartialEq, Default)]
struct Dict(Vec<(String, Entry)>);

impl Dict {
    fn get(&self, key: &str) -> Result<&Entry, FoamError> {
        self.0.iter().find(|(k, _)| k == key).map(|(_, e)| e).ok_or(FoamError(format!("entry {key} missing")))
    }

    fn dict(&self, key: &str) -> Result<&Dict, FoamError> {
        match self.get(key)? {
            Entry::Dict(d) => Ok(d),
            Entry::Stream(_) => err(format!("{key} must be a dictionary")),
        }
    }

    fn stream(&self, key: &str) -> Result<&[Tok], FoamError> {
        match self.get(key)? {
            Entry::Stream(s) => Ok(s),
            Entry::Dict(_) => err(format!("{key} must not be a dictionary")),
        }
    }

    fn word(&self, key: &str) -> Result<&str, FoamError> {
        match self.stream(key)? {
            [Tok::W(w)] => Ok(w),
            _ => err(format!("{key} must be one word or number")),
        }
    }

    fn keys(&self) -> Vec<&str> {
        self.0.iter().map(|(k, _)| k.as_str()).collect()
    }
}

fn parse_dict(t: &[Tok], i: &mut usize, closed: bool) -> Result<Dict, FoamError> {
    let mut d = Dict::default();
    loop {
        let key = match t.get(*i) {
            None if closed => return err("'}' missing"),
            None => return Ok(d),
            Some(Tok::P(b'}')) if closed => {
                *i += 1;
                return Ok(d);
            }
            Some(Tok::W(k)) => k.clone(),
            Some(Tok::P(c)) => return err(format!("unexpected '{}'", *c as char)),
        };
        if key.starts_with('#') || key.starts_with('$') {
            return err(format!("{key}: directives and variables are not supported (use a file written by OpenFOAM)"));
        }
        *i += 1;
        let entry = if t.get(*i) == Some(&Tok::P(b'{')) {
            *i += 1;
            Entry::Dict(parse_dict(t, i, true)?)
        } else {
            let start = *i;
            let mut depth = 0usize;
            loop {
                match t.get(*i) {
                    None => return err(format!("entry {key}: ';' missing")),
                    Some(Tok::P(b';')) if depth == 0 => break,
                    Some(Tok::P(b'(' | b'[' | b'{')) => depth += 1,
                    Some(Tok::P(b')' | b']' | b'}')) => {
                        depth = depth.checked_sub(1).ok_or(FoamError(format!("entry {key}: unbalanced brackets")))?;
                    }
                    Some(Tok::W(w)) if w.starts_with('$') || w.starts_with('#') => {
                        return err(format!("entry {key}: {w}: directives and variables are not supported"));
                    }
                    _ => {}
                }
                *i += 1;
            }
            let s = t[start..*i].to_vec();
            *i += 1;
            Entry::Stream(s)
        };
        if d.0.iter().any(|(k, _)| *k == key) {
            return err(format!("entry {key} appears twice"));
        }
        d.0.push((key, entry));
    }
}

/// Parses a whole file and checks its `FoamFile` header (ASCII, the expected class).
fn read_foam(text: &str, class: &str) -> Result<Dict, FoamError> {
    let t = lex(text)?;
    let mut i = 0;
    let d = parse_dict(&t, &mut i, false)?;
    let h = d.dict("FoamFile").map_err(|_| FoamError("no FoamFile header".into()))?;
    let format = h.word("format")?;
    if format != "ascii" {
        return err(format!(
            "format {format}: only ascii is read (set writeFormat ascii in system/controlDict and run foamFormatConvert)"
        ));
    }
    let c = h.word("class")?;
    if c != class {
        return err(format!("class {c}, expected {class}"));
    }
    Ok(d)
}

fn num(w: &str) -> Result<f64, FoamError> {
    w.parse::<f64>().map_err(|_| FoamError(format!("not a number: {w:?}")))
}

fn label(t: Option<&Tok>) -> Result<usize, FoamError> {
    match t {
        Some(Tok::W(w)) => w.parse::<usize>().map_err(|_| FoamError(format!("not a count: {w:?}"))),
        other => err(format!("expected a count, got {other:?}")),
    }
}

fn expect(t: &[Tok], i: &mut usize, c: u8) -> Result<(), FoamError> {
    if t.get(*i) == Some(&Tok::P(c)) {
        *i += 1;
        Ok(())
    } else {
        err(format!("expected '{}', got {:?}", c as char, t.get(*i)))
    }
}

fn scalar(t: &[Tok], i: &mut usize) -> Result<f64, FoamError> {
    match t.get(*i) {
        Some(Tok::W(w)) => {
            *i += 1;
            num(w)
        }
        other => err(format!("expected a number, got {other:?}")),
    }
}

fn vector(t: &[Tok], i: &mut usize) -> Result<[f64; 3], FoamError> {
    expect(t, i, b'(')?;
    let v = [scalar(t, i)?, scalar(t, i)?, scalar(t, i)?];
    expect(t, i, b')')?;
    Ok(v)
}

/// `uniform V` or `nonuniform List<type> N ( V ... )`, also OpenFOAM's `N{V}` and `N(V ...)`.
fn field<T: Clone>(
    t: &[Tok],
    type_name: &str,
    elem: fn(&[Tok], &mut usize) -> Result<T, FoamError>,
) -> Result<Internal<T>, FoamError> {
    let mut i = 1;
    let out = match t.first() {
        Some(Tok::W(w)) if w == "uniform" => Internal::Uniform(elem(t, &mut i)?),
        Some(Tok::W(w)) if w == "nonuniform" => {
            let list = format!("List<{type_name}>");
            if t.get(1) != Some(&Tok::W(list.clone())) {
                return err(format!("internalField: expected nonuniform {list}"));
            }
            let n = label(t.get(2))?;
            i = 3;
            match t.get(i) {
                Some(Tok::P(b'{')) => {
                    i += 1;
                    let v = elem(t, &mut i)?;
                    expect(t, &mut i, b'}')?;
                    Internal::Values(vec![v; n])
                }
                _ => {
                    expect(t, &mut i, b'(')?;
                    let mut v = Vec::with_capacity(n.min(1 << 24));
                    while t.get(i) != Some(&Tok::P(b')')) {
                        if i >= t.len() {
                            return err("internalField: list not closed");
                        }
                        v.push(elem(t, &mut i)?);
                    }
                    i += 1;
                    if v.len() != n {
                        return err(format!("internalField: the list says {n} values but has {}", v.len()));
                    }
                    Internal::Values(v)
                }
            }
        }
        _ => return err("internalField must start with uniform or nonuniform"),
    };
    if i != t.len() {
        return err("internalField: unexpected text after the values");
    }
    Ok(out)
}

fn uniform_scalar(t: &[Tok]) -> Result<f64, FoamError> {
    match field(t, "scalar", scalar)? {
        Internal::Uniform(v) => Ok(v),
        Internal::Values(_) => err("expected a uniform value"),
    }
}

fn dimensions(t: &[Tok]) -> Result<[i32; 7], FoamError> {
    let bad = || FoamError("dimensions must be [a b c d e f g] (seven integers)".into());
    if t.len() != 9 || t[0] != Tok::P(b'[') || t[8] != Tok::P(b']') {
        return Err(bad());
    }
    let mut d = [0i32; 7];
    for (k, tok) in t[1..8].iter().enumerate() {
        let Tok::W(w) = tok else { return Err(bad()) };
        d[k] = w.parse().map_err(|_| bad())?;
    }
    Ok(d)
}

fn labels3(t: &[Tok]) -> Result<[usize; 3], FoamError> {
    let mut i = 0;
    let v = list_of_labels(t, &mut i)?;
    match v[..] {
        [a, b, c] if i == t.len() => Ok([a, b, c]),
        _ => err("expected three counts in parentheses"),
    }
}

fn check_patch_names(bf: &Dict, g: &Grid, file: &str) -> Result<(), FoamError> {
    let mut want: Vec<&str> = g.face_names().to_vec();
    if g.is_2d() {
        want.push(EMPTY_PATCH);
    }
    if bf.keys() != want {
        return err(format!("{file}: patches {:?}, expected {:?}", bf.keys(), want));
    }
    if g.is_2d() && bf.dict(EMPTY_PATCH)?.word("type")? != "empty" {
        return err(format!("{file}: {EMPTY_PATCH} must be empty"));
    }
    Ok(())
}

/// Checks that `blockMeshDict` describes exactly the block of `g`: vertices (bit for bit), cell
/// counts, uniform grading and the patches with their faces.
fn check_block_mesh(bm: &Dict, g: &Grid) -> Result<(), FoamError> {
    if num(bm.word("scale")?)? != 1.0 {
        return err("blockMeshDict: scale must be 1");
    }
    let vt = bm.stream("vertices")?;
    let mut i = 0;
    expect(vt, &mut i, b'(')?;
    let mut verts = Vec::new();
    while vt.get(i) != Some(&Tok::P(b')')) && i < vt.len() {
        verts.push(vector(vt, &mut i)?);
    }
    let want = g.vertices();
    let same = verts.len() == 8 && verts.iter().zip(&want).all(|(a, b)| a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits()));
    if !same {
        return err("blockMeshDict: vertices differ from the grid of system/dyneGrid");
    }
    let bt = bm.stream("blocks")?;
    let mut i = 0;
    expect(bt, &mut i, b'(')?;
    if bt.get(i) != Some(&Tok::W("hex".into())) {
        return err("blockMeshDict: expected one hex block");
    }
    i += 1;
    let idx = list_of_labels(bt, &mut i)?;
    let counts = list_of_labels(bt, &mut i)?;
    if bt.get(i) != Some(&Tok::W("simpleGrading".into())) {
        return err("blockMeshDict: expected simpleGrading");
    }
    i += 1;
    let mut grading = Vec::new();
    expect(bt, &mut i, b'(')?;
    while bt.get(i) != Some(&Tok::P(b')')) && i < bt.len() {
        grading.push(scalar(bt, &mut i)?);
    }
    i += 1;
    expect(bt, &mut i, b')')?;
    if idx != [0, 1, 2, 3, 4, 5, 6, 7] || grading != [1.0, 1.0, 1.0] || i != bt.len() {
        return err("blockMeshDict: expected hex (0 1 2 3 4 5 6 7) (...) simpleGrading (1 1 1)");
    }
    if counts != g.points {
        return err(format!("blockMeshDict: {counts:?} cells, system/dyneGrid has {:?} points", g.points));
    }
    // boundary ( name { type ...; faces ( (a b c d) ... ); } ... )
    let b = bm.stream("boundary")?;
    if b.first() != Some(&Tok::P(b'(')) || b.last() != Some(&Tok::P(b')')) {
        return err("blockMeshDict: boundary must be a list");
    }
    let mut i = 1;
    let inner = &b[..b.len() - 1];
    let mut patches = Vec::new();
    while i < inner.len() {
        let Some(Tok::W(name)) = inner.get(i) else { return err("blockMeshDict: expected a patch name") };
        i += 1;
        expect(inner, &mut i, b'{')?;
        let d = parse_dict(inner, &mut i, true)?;
        let ft = d.stream("faces")?;
        let mut j = 0;
        expect(ft, &mut j, b'(')?;
        let mut faces = Vec::new();
        while ft.get(j) != Some(&Tok::P(b')')) && j < ft.len() {
            faces.push(list_of_labels(ft, &mut j)?);
        }
        patches.push((name.clone(), d.word("type")?.to_string(), faces));
    }
    let mut want = Vec::new();
    for (f, name) in g.face_names().iter().enumerate() {
        want.push((name.to_string(), "patch".to_string(), vec![FACE_VERTICES[f].to_vec()]));
    }
    if g.is_2d() {
        want.push((EMPTY_PATCH.to_string(), "empty".to_string(), vec![FACE_VERTICES[4].to_vec(), FACE_VERTICES[5].to_vec()]));
    }
    if patches != want {
        return err("blockMeshDict: patches differ from the expected ones");
    }
    Ok(())
}

fn list_of_labels(t: &[Tok], i: &mut usize) -> Result<Vec<usize>, FoamError> {
    expect(t, i, b'(')?;
    let mut v = Vec::new();
    while t.get(*i) != Some(&Tok::P(b')')) {
        if *i >= t.len() {
            return err("list not closed");
        }
        v.push(label(t.get(*i))?);
        *i += 1;
    }
    *i += 1;
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn same_bits(a: &[f64], b: &[f64]) -> bool {
        a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
    }

    /// Finite values that a careless decimal conversion would change.
    fn awkward(n: usize) -> Vec<f64> {
        let base = [
            0.0,
            -0.0,
            0.1,
            1.0 / 3.0,
            -2.0 / 3.0,
            f64::MIN_POSITIVE,
            f64::from_bits(1),
            f64::MAX,
            f64::MIN,
            f64::EPSILON,
            1e-300,
            123456789.12345679,
            0.30000000000000004,
            293.15,
        ];
        let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
        (0..n)
            .map(|k| {
                if k < base.len() {
                    return base[k];
                }
                loop {
                    s ^= s << 13;
                    s ^= s >> 7;
                    s ^= s << 17;
                    let v = f64::from_bits(s);
                    if v.is_finite() {
                        return v;
                    }
                }
            })
            .collect()
    }

    fn dir(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("dyne-foam-test-{}-{name}", std::process::id()))
    }

    fn round_trip(c: &Case, name: &str) -> Case {
        let d = dir(name);
        let _ = std::fs::remove_dir_all(&d);
        c.write(&d).unwrap();
        let r = Case::read(&d).unwrap();
        assert_eq!(r.grid.points, c.grid.points);
        assert!(same_bits(&r.grid.origin, &c.grid.origin));
        assert!(same_bits(&r.grid.spacing, &c.grid.spacing));
        assert!(same_bits(&r.t, &c.t), "T changed");
        assert_eq!(r.faces.len(), c.faces.len());
        for (a, b) in r.faces.iter().zip(&c.faces) {
            match (a, b) {
                (Face::FixedValue(x), Face::FixedValue(y)) | (Face::InletOutlet(x), Face::InletOutlet(y)) => {
                    assert_eq!(x.to_bits(), y.to_bits())
                }
                (Face::ZeroGradient, Face::ZeroGradient) => {}
                _ => panic!("face changed: {a:?} vs {b:?}"),
            }
        }
        match (&r.velocity, &c.velocity) {
            (Velocity::Uniform(a), Velocity::Uniform(b)) => assert!(same_bits(a, b)),
            (Velocity::Cells(a), Velocity::Cells(b)) => {
                assert!(same_bits(&a.concat(), &b.concat()));
            }
            _ => panic!("velocity changed"),
        }
        for (x, y) in [(r.diffusivity, c.diffusivity), (r.end_time, c.end_time), (r.delta_t, c.delta_t), (r.write_interval, c.write_interval)] {
            assert_eq!(x.to_bits(), y.to_bits());
        }
        assert_eq!(r.t_dimensions, c.t_dimensions);
        // Writing what was read gives the same files.
        assert_eq!(r.files().unwrap(), c.files().unwrap());
        // The velocity import reads the exported U the same way.
        let (g, u) = import_velocity(&d, "0").unwrap();
        assert_eq!(g, c.grid);
        let want = match &c.velocity {
            Velocity::Uniform(v) => vec![*v; c.grid.cells()],
            Velocity::Cells(v) => v.clone(),
        };
        assert!(same_bits(&u.concat(), &want.concat()));
        std::fs::remove_dir_all(&d).unwrap();
        r
    }

    #[test]
    fn round_trip_2d() {
        let g = Grid::new_2d([7, 5], [-0.0, 1e-300], [0.025, 1.0 / 3.0]);
        let mut c = Case::new(g, awkward(35), 2.2e-5);
        c.faces = vec![Face::FixedValue(-0.0), Face::ZeroGradient, Face::InletOutlet(293.15), Face::FixedValue(0.1)];
        c.velocity = Velocity::Uniform([0.1, -0.2, 0.0]);
        c.end_time = 600.0;
        c.delta_t = 0.05;
        c.write_interval = 60.0;
        c.t_dimensions = [0, 0, 0, 0, 0, 0, 0];
        round_trip(&c, "2d");
    }

    #[test]
    fn round_trip_3d_velocity_field() {
        let g = Grid::new_3d([4, 3, 2], [0.5, -2.0, 1e10], [0.1, 0.2, 0.30000000000000004]);
        let mut c = Case::new(g, awkward(24), 0.0);
        c.faces = vec![
            Face::ZeroGradient,
            Face::FixedValue(1.0),
            Face::InletOutlet(-0.0),
            Face::ZeroGradient,
            Face::FixedValue(f64::MIN_POSITIVE),
            Face::ZeroGradient,
        ];
        let a = awkward(72);
        c.velocity = Velocity::Cells((0..24).map(|k| [a[3 * k], a[3 * k + 1], a[3 * k + 2]]).collect());
        round_trip(&c, "3d");
    }

    #[test]
    fn single_cell() {
        let c = Case::new(Grid::new_3d([1, 1, 1], [0.0; 3], [1.0; 3]), vec![1.5], 1.0);
        round_trip(&c, "one");
    }

    #[test]
    fn mesh_matches_grid() {
        let g = Grid::new_2d([3, 2], [0.0, 0.0], [0.5, 0.25]);
        let c = Case::new(g, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0], 1e-3);
        let files = c.files().unwrap();
        let get = |p: &str| files.iter().find(|(k, _)| *k == p).unwrap().1.clone();
        let bm = get("system/blockMeshDict");
        assert!(bm.contains("    (-0.25 -0.125 -0.25)\n"), "{bm}");
        assert!(bm.contains("    (1.25 0.375 0.25)\n"), "{bm}");
        assert!(bm.contains("hex (0 1 2 3 4 5 6 7) (3 2 1) simpleGrading (1 1 1)"));
        assert!(bm.contains("    frontAndBack\n    {\n        type empty;"));
        let t = get("0/T");
        assert!(t.contains("internalField   nonuniform List<scalar>\n6\n(\n1.0\n2.0\n3.0\n4.0\n5.0\n6.0\n)\n;"), "{t}");
        assert!(get("0/U").contains("internalField   uniform (0.0 0.0 0.0);"));
        for (path, text) in &files {
            assert!(text.contains("FoamFile\n{\n    version     2.0;\n    format      ascii;\n"), "{path}");
            let object = path.rsplit('/').next().unwrap();
            assert!(text.contains(&format!("    object      {object};\n}}")), "{path}");
        }
        assert!(get("system/controlDict").contains("writePrecision  17;"));
        assert!(get("constant/transportProperties").contains("DT              [0 2 -1 0 0 0 0] 0.001;"));
        // 2D cell centres: OpenFOAM computes them from the vertices; here they come out exact.
        let v = g.vertices();
        assert_eq!(v[0], [-0.25, -0.125, -0.25]);
        assert_eq!(v[6], [1.25, 0.375, 0.25]);
    }

    #[test]
    fn rejects_bad_case() {
        let g = Grid::new_2d([2, 2], [0.0, 0.0], [1.0, 1.0]);
        assert!(Case::new(g, vec![0.0; 3], 1.0).files().is_err()); // wrong length
        assert!(Case::new(g, vec![0.0, 1.0, f64::NAN, 0.0], 1.0).files().is_err()); // NaN
        assert!(Case::new(g, vec![0.0; 4], -1.0).files().is_err()); // negative diffusivity
        let mut c = Case::new(g, vec![0.0; 4], 1.0);
        c.faces.pop();
        assert!(c.files().is_err()); // three faces for a 2D grid
        let mut c = Case::new(g, vec![0.0; 4], 1.0);
        c.velocity = Velocity::Cells(vec![[0.0; 3]; 3]);
        assert!(c.files().is_err());
        let mut c = Case::new(g, vec![0.0; 4], 1.0);
        c.delta_t = 0.0;
        assert!(c.files().is_err());
        let g0 = Grid::new_2d([2, 2], [0.0, 0.0], [0.0, 1.0]);
        assert!(Case::new(g0, vec![0.0; 4], 1.0).files().is_err()); // zero spacing
    }

    #[test]
    fn rejects_inconsistent_files() {
        let g = Grid::new_2d([3, 2], [0.0, 0.0], [0.5, 0.5]);
        let c = Case::new(g, vec![1.0; 6], 1e-3);
        let d = dir("bad");
        let _ = std::fs::remove_dir_all(&d);
        c.write(&d).unwrap();
        let edit = |rel: &str, from: &str, to: &str| {
            let p = d.join(rel);
            let s = std::fs::read_to_string(&p).unwrap();
            assert!(s.contains(from), "{rel}: {from}");
            std::fs::write(&p, s.replacen(from, to, 1)).unwrap();
            s
        };
        let restore = |rel: &str, s: String| std::fs::write(d.join(rel), s).unwrap();
        let s = edit("system/blockMeshDict", "(3 2 1)", "(3 3 1)");
        assert!(Case::read(&d).is_err());
        restore("system/blockMeshDict", s);
        let s = edit("system/blockMeshDict", "(1.25 ", "(1.2500000000000002 ");
        assert!(Case::read(&d).is_err());
        restore("system/blockMeshDict", s);
        let s = edit("0/T", "\n6\n(", "\n7\n(");
        assert!(Case::read(&d).is_err());
        restore("0/T", s);
        let s = edit("0/T", "format      ascii", "format      binary");
        assert!(Case::read(&d).is_err());
        restore("0/T", s);
        let s = edit("0/U", "frontAndBack", "front");
        assert!(Case::read(&d).is_err());
        restore("0/U", s);
        assert!(Case::read(&d).is_ok());
        std::fs::remove_dir_all(&d).unwrap();
    }

    const U_HEAD: &str = "FoamFile\n{\n    version 2.0;\n    format ascii;\n    arch \"LSB;label=32;scalar=64\";\n    class volVectorField;\n    location \"5\";\n    object U;\n}\n// a comment\ndimensions [0 1 -1 0 0 0 0];\n";
    const U_TAIL: &str = "\nboundaryField\n{\n    inlet { type fixedValue; value uniform (1 0 0); }\n    walls { type noSlip; }\n    outlet { type inletOutlet; inletValue uniform (0 0 0); value nonuniform List<vector> 2((1 0 0) (2 0 0)); }\n}\n// ****** //\n";

    #[test]
    fn reads_openfoam_forms() {
        let read = |internal: &str| read_vector_field(&format!("{U_HEAD}internalField {internal};{U_TAIL}"));
        assert_eq!(read("uniform (0.5 -0 1e-05)").unwrap(), Internal::Uniform([0.5, -0.0, 1e-5]));
        assert_eq!(read("nonuniform List<vector> 3{(1 2 3)}").unwrap(), Internal::Values(vec![[1.0, 2.0, 3.0]; 3]));
        assert_eq!(
            read("nonuniform List<vector> 2((1 2 3) (4 5 6))").unwrap(),
            Internal::Values(vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        );
        assert_eq!(
            read("nonuniform List<vector> \n2\n(\n(0.10000000000000001 0 0)\n/* c */(4 5 6)\n)\n").unwrap(),
            Internal::Values(vec![[0.1, 0.0, 0.0], [4.0, 5.0, 6.0]])
        );
        assert_eq!(read("nonuniform List<vector> 0()").unwrap(), Internal::Values(vec![]));
        assert!(read("nonuniform List<vector> 3((1 2 3) (4 5 6))").is_err()); // count
        assert!(read("nonuniform List<scalar> 1(1)").is_err()); // wrong type
        assert!(read("uniform (1 2)").is_err());
        assert!(read("uniform $flowVelocity").is_err());
        let scalar_file = "FoamFile { version 2.0; format ascii; class volScalarField; object p; }\ninternalField nonuniform List<scalar> 3(1 -2.5e-17 3);\nboundaryField {}\n";
        assert_eq!(read_scalar_field(scalar_file).unwrap(), Internal::Values(vec![1.0, -2.5e-17, 3.0]));
        assert!(read_vector_field(scalar_file).is_err()); // class
        let binary = format!("{U_HEAD}internalField uniform (0 0 0);{U_TAIL}").replace("format ascii", "format binary");
        assert!(read_vector_field(&binary).unwrap_err().0.contains("foamFormatConvert"));
        assert!(read_vector_field("#include \"x\"\n").is_err());
    }

    #[test]
    fn lexer_words_with_parentheses() {
        let t = lex("div(phi,T) Gauss linearUpwind grad(T); 3(1 2) x//c\n/*y*/z \"a b\"").unwrap();
        let w = |s: &str| Tok::W(s.to_string());
        assert_eq!(
            t,
            vec![
                w("div(phi,T)"),
                w("Gauss"),
                w("linearUpwind"),
                w("grad(T)"),
                Tok::P(b';'),
                w("3"),
                Tok::P(b'('),
                w("1"),
                w("2"),
                Tok::P(b')'),
                w("x"),
                w("z"),
                w("\"a b\"")
            ]
        );
    }
}
