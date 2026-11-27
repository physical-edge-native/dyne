//! Shared constants of the solver ("actor") interface.
//!
//! A solver is a WebAssembly module whose exported functions the host calls. The host and
//! the solver both depend on this module, so configuration keys, face numbers and return
//! codes are written down once.
//!
//! - The top level holds the configuration keys of the flocking agent
//!   (`swarm-agent-actor`, run by `dyne-swarm-sim`), set with `dyne_cfg(key, value)`.
//! - [`field2d`]: face numbers, face kinds and return codes shared by the 2D field solvers
//!   (`heat2d-actor`, `wave2d-actor`, `sdf2d-grid-actor`, `imu-fusion-actor`) and their
//!   hosts (`dyne-sim2d`, the firmware).
//! - [`sdf2d`], [`wave2d`], [`imu`]: configuration keys of individual solvers.
//!
//! The 1D solvers (`heat1d-actor`) use a smaller interface without mailboxes; it is
//! described in `docs/en/tutorial.md` and loaded by `dyne_core::actor`.
//!
//! ## The 2D interface, in the order a host calls it
//!
//! 1. `dyne_cfg(key, value)` for each configuration value (kept across re-initialisation);
//! 2. `dyne_set_face(face, kind, iface_or_bits)` for each of the four faces
//!    ([`field2d::FACE_W`] .. [`field2d::FACE_N`]; kind [`field2d::BC_DIRICHLET`],
//!    [`field2d::BC_INTERFACE`] with a border id, or [`field2d::BC_ZERO_GRAD`]);
//! 3. `dyne_init2d(nx, ny, ix0, iy0, h, alpha, dt)`: the block of `nx × ny` points whose
//!    first point has global indices `(ix0, iy0)`;
//! 4. every step: read the region's current border values from the outbox
//!    (`dyne_outbox_read`) and deliver them to the neighbours; write the messages received
//!    from the neighbours into the inbox (`dyne_inbox_write`, returns `INBOX_*`); call
//!    `dyne_step` (returns `STEP_*`). The host copies messages through the solver's work
//!    area (`dyne_msg_buf`, `dyne_msg_cap`); they use the wire format and mailbox framing
//!    of this crate, and the host routes them by border id and step without reading the
//!    values;
//! 5. read the state with `dyne_len` / `dyne_get`.

/// Flocking agent, configuration key: behaviour rule (value `MODE_*`).
pub const CFG_MODE: u32 = 0;
/// Flocking agent, configuration key: preferred temperature T_opt \[°C\].
pub const CFG_T_OPT: u32 = 1;
/// Flocking agent, configuration key: tolerance around T_opt (|T − T_opt| < tol).
pub const CFG_TOL: u32 = 2;
/// Flocking agent, configuration key: largest move per step.
pub const CFG_SPEED: u32 = 3;
/// Flocking agent, configuration key: standard deviation of the sensor noise.
pub const CFG_NOISE: u32 = 4;
/// Flocking agent, configuration key: communication radius (also the distance for separation).
pub const CFG_RADIUS: u32 = 5;
/// Flocking agent, configuration key: largest age of hearsay in steps (older news is discarded).
pub const CFG_GOSSIP_MAX_AGE: u32 = 6;
/// Flocking agent, configuration key: lower 32 bits of the agent's random seed (exact in an
/// f64). Set LO, then HI; writing HI seeds the agent's random generator.
pub const CFG_SEED_LO: u32 = 7;
/// Flocking agent, configuration key: upper 32 bits of the agent's random seed.
pub const CFG_SEED_HI: u32 = 8;

/// Behaviour rule: no communication (control group).
pub const MODE_SOLO: u32 = 0;
/// Behaviour rule: share temperature only and move towards the best neighbour.
pub const MODE_FOLLOW: u32 = 1;
/// Behaviour rule: share position and temperature and agree on the local gradient.
pub const MODE_CONSENSUS: u32 = 2;
/// Behaviour rule: consensus plus hearsay of the best place (default).
pub const MODE_GOSSIP: u32 = 3;

/// Constants of the 2D solver interface (heat2d-actor and others), shared by the hosts
/// (`dyne-sim2d`, the firmware) and the solvers.
pub mod field2d {
    /// Face: west (smaller x).
    pub const FACE_W: u32 = 0;
    /// Face: east (larger x).
    pub const FACE_E: u32 = 1;
    /// Face: south (smaller y).
    pub const FACE_S: u32 = 2;
    /// Face: north (larger y).
    pub const FACE_N: u32 = 3;

    /// Face kind: fixed value (Dirichlet); the points on the face keep a fixed value.
    pub const BC_DIRICHLET: u32 = 0;
    /// Face kind: border shared with a neighbouring region (values exchanged as `IFACE_BAND`).
    pub const BC_INTERFACE: u32 = 1;
    /// Face kind: zero gradient (insulated); the ghost points mirror the inner points.
    pub const BC_ZERO_GRAD: u32 = 2;

    /// `dyne_inbox_write` result: everything accepted.
    pub const INBOX_OK: i32 = 0;
    /// `dyne_inbox_write` result: some messages were not accepted (border id, step or number
    /// of points did not match; they are counted).
    pub const INBOX_REJECTED: i32 = 1;
    /// `dyne_inbox_write` result: malformed framing or message (the rest is not read).
    pub const INBOX_MALFORMED: i32 = -1;

    /// `dyne_step` result: the values of every neighbouring border were present.
    pub const STEP_OK: i32 = 0;
    /// `dyne_step` result: a border's values were missing (the previous values were used).
    pub const STEP_MISSING_GHOST: i32 = 1;
}

/// Configuration keys of the room-shape reconstruction solver (`sdf2d-grid-actor`), set
/// before `dyne_init2d`. Angles in radians, lengths in metres.
pub mod sdf2d {
    /// Grid origin x: room coordinate of the point with global indices (0, 0).
    pub const CFG_ORIGIN_X: u32 = 0;
    /// Grid origin y.
    pub const CFG_ORIGIN_Y: u32 = 1;
    /// Truncation distance μ of the signed distance.
    pub const CFG_MU: u32 = 2;
    /// Largest integration weight.
    pub const CFG_W_MAX: u32 = 3;
    /// Integration delay L in steps: a ray acquired at step s is integrated by every region
    /// at step s + L.
    pub const CFG_DELAY: u32 = 4;
    /// Range definition: 0 = along the ray (radial), 1 = perpendicular to the sensor's face.
    pub const CFG_RANGE_DEF: u32 = 5;
    /// Whether zones without a target mark free space up to the maximum range (0/1, default 0).
    pub const CFG_CARVE_NO_TARGET: u32 = 6;
    /// Maximum range of the sensor.
    pub const CFG_MAX_RANGE: u32 = 7;
    /// Horizontal field of view of the sensor (full angle).
    pub const CFG_FOV: u32 = 8;
    /// Row of the range frame that is used (from 0).
    pub const CFG_ZONE_ROW: u32 = 9;
    /// Selects the sensor (attached to this device) that the following
    /// `CFG_SENSOR_X`/`Y`/`YAW` refer to.
    pub const CFG_SENSOR_SEL: u32 = 10;
    /// Sensor position x.
    pub const CFG_SENSOR_X: u32 = 11;
    /// Sensor position y.
    pub const CFG_SENSOR_Y: u32 = 12;
    /// Sensor heading, counter-clockwise from the x axis.
    pub const CFG_SENSOR_YAW: u32 = 13;
    /// Forgetting old observations (this and the following keys; with their defaults nothing
    /// changes, bit for bit). Weight decay λ per step, w ← λ·w (default 1 = no decay).
    pub const CFG_DECAY: u32 = 14;
    /// Points whose decayed weight falls below this become unobserved again (w = 0, d = 0);
    /// default 0 = never.
    pub const CFG_W_FORGET: u32 = 15;
    /// Disagreement threshold τ: if a new observation differs from the integrated value by
    /// more than τ, the integrated weight is lowered to `CFG_DISAGREE_W` first (default 0 = off).
    pub const CFG_DISAGREE: u32 = 16;
    /// Weight kept on disagreement (default 1).
    pub const CFG_DISAGREE_W: u32 = 17;
    /// Run the distance sweep only on steps that are multiples of n (default 1 = every step).
    /// Near surfaces the distance is the observation itself, so thinning only slows how the
    /// distance spreads to points far from surfaces.
    pub const CFG_SWEEP_EVERY: u32 = 18;
    /// Apply decay and forgetting only to points in view (0/1; default 0 = all points every
    /// step). With 1, only points crossed by rays integrated in that step decay, once, so that
    /// walls do not fade while a turning sensor looks elsewhere.
    pub const CFG_DECAY_IN_VIEW: u32 = 19;
    /// With `CFG_DECAY_IN_VIEW = 1`: decay per step for points out of view (default 1 = keep).
    /// Below 1, old surfaces that are no longer visible are eventually forgotten too.
    pub const CFG_DECAY_OUT: u32 = 20;

    /// `CFG_RANGE_DEF` value: distance along the ray.
    pub const RANGE_DEF_RADIAL: u32 = 0;
    /// `CFG_RANGE_DEF` value: distance perpendicular to the sensor's face.
    pub const RANGE_DEF_PERPENDICULAR: u32 = 1;

    /// Whether a zone status of the VL53L5CX range sensor marks a valid distance (5 and 9,
    /// as in the sensor's documentation).
    pub fn status_valid(st: u32) -> bool {
        st == 5 || st == 9
    }

    /// `dyne_get_aux` quantity: observed distance d.
    pub const AUX_D: u32 = 1;
    /// `dyne_get_aux` quantity: integration weight w.
    pub const AUX_W: u32 = 2;
}

/// Configuration keys of the 2D wave-equation solver (`wave2d-actor`), set before
/// `dyne_init2d` and kept across re-initialisation. The rest of the interface is that of
/// [`field2d`]; the `alpha` argument of `dyne_init2d` is the wave speed c.
pub mod wave2d {
    /// Initial condition (`IC_*`, default `IC_SIN_MODE`); the initial velocity is always 0.
    pub const CFG_IC: u32 = 0;
    /// Mode number m of the standing wave in x (default 1).
    pub const CFG_MODE_M: u32 = 1;
    /// Mode number n of the standing wave in y (default 1).
    pub const CFG_MODE_N: u32 = 2;
    /// Amplitude (default 1).
    pub const CFG_AMP: u32 = 3;
    /// Centre x of the Gaussian bump (default 0.5).
    pub const CFG_GX: u32 = 4;
    /// Centre y of the Gaussian bump (default 0.5).
    pub const CFG_GY: u32 = 5;
    /// Width σ of the Gaussian bump (default 0.05).
    pub const CFG_SIGMA: u32 = 6;
    /// Bit set of outer faces that absorb waves (bit f = face f; default 0 = none; ignored on
    /// borders with neighbours): first-order Engquist–Majda condition, discretised after Mur.
    pub const CFG_ABSORB: u32 = 7;

    /// Initial condition: sin(mπx)·sin(nπy) (standing wave with fixed zero faces).
    pub const IC_SIN_MODE: u32 = 0;
    /// Initial condition: Gaussian bump A·exp(−((x−gx)² + (y−gy)²)/(2σ²)).
    pub const IC_GAUSS: u32 = 1;
    /// Initial condition: cos(mπx)·cos(nπy) (standing wave with zero-gradient faces).
    pub const IC_COS_MODE: u32 = 2;
}

/// Constants of the solver that fuses inertial-sensor (IMU) readings by exchanging only with
/// neighbours (`imu-fusion-actor`). Its interface is that of [`field2d`] plus `dyne_imu_push`
/// (hand in a sample) and `dyne_imu_need_until_us` (how long to wait for samples).
/// Neighbours are faces of kind [`field2d::BC_INTERFACE`] (up to four per device: line,
/// ring or grid). Of the `dyne_init2d` arguments only `dt` (period of one agreement
/// iteration \[s\]) is used.
pub mod imu {
    /// Configuration key: steps per epoch M (an epoch is the window over which samples are
    /// averaged, M·dt long; default 10).
    pub const CFG_EPOCH_STEPS: u32 = 0;
    /// Configuration key: how samples are assigned to epochs (`TIME_*`, default `TIME_STAMP`).
    pub const CFG_TIME_MODE: u32 = 1;
    /// Configuration key: time t0 \[µs\] of step 0 (synchronised time; default 0).
    pub const CFG_T0_US: u32 = 2;
    /// Configuration key: offset \[µs\] added to sample time stamps (to skew a clock on
    /// purpose in experiments; default 0).
    pub const CFG_TS_OFFSET_US: u32 = 3;
    /// Configuration key: complementary-filter coefficient α (angle = α·(previous angle +
    /// rate·epoch) + (1−α)·angle from acceleration; default 0.9).
    pub const CFG_CF_ALPHA: u32 = 4;
    /// Configuration key: number of initial epochs used to calibrate the gyroscope's zero
    /// (keep the board still meanwhile; default 0 = no calibration).
    pub const CFG_GYRO_CAL_EPOCHS: u32 = 5;
    /// Configuration key: agreement method (`CONS_*`, default `CONS_DYNAMIC`).
    pub const CFG_CONSENSUS: u32 = 6;
    /// Configuration key (added 2026-11-27, issue #194): number of initial epochs over which each
    /// device estimates its own zero offset (keep the devices still meanwhile; default 0 = off).
    /// From the next epoch on, each device subtracts its offset before sharing: for the angular
    /// rate, its own static mean (the true rate is 0); for the acceleration, its own static mean
    /// minus the fused static mean (so that gravity stays). The fused tilt filter holds its
    /// angle at the acceleration angle during these epochs and does not subtract the angular-rate
    /// offset again (its `CFG_GYRO_CAL_EPOCHS` calibration uses only later, corrected epochs).
    /// The device's own outputs (`G_LOCAL*`) stay uncorrected.
    pub const CFG_OFFSET_EPOCHS: u32 = 7;
    /// Configuration key (added 2026-11-27, issue #194): number of devices N in the run, used
    /// only for `G_FUSED_DEVICES` (default 0 = unknown).
    pub const CFG_DEVICES: u32 = 8;

    /// Agreement: reset the state to the device's own sums every epoch (the average is
    /// approached within the M iterations of the epoch).
    pub const CONS_RESET: u32 = 0;
    /// Agreement: dynamic average consensus; add only (this epoch's sums − last epoch's sums)
    /// to the state. The total over devices always equals the total of this epoch's sums, and
    /// constant disagreements (zero offsets) keep mixing across epochs and vanish. An
    /// iteration whose neighbour values did not arrive leaves a lasting error in the total.
    pub const CONS_DYNAMIC: u32 = 1;

    /// Assignment by time stamp: epoch e is (t0 + e·E, t0 + (e+1)·E\], E = M·dt.
    pub const TIME_STAMP: u32 = 0;
    /// Assignment by arrival: a sample goes into the epoch open when it arrives.
    pub const TIME_ARRIVAL: u32 = 1;

    /// Quantities per point of the `IFACE_BAND` exchanged for the agreement (count = 1):
    /// degree (number of neighbours) and the state (weight, weight × mean of ax, ay, az, gx,
    /// gy, gz). The weight is 1 for a device that had samples in the epoch, else 0.
    pub const PER_POINT: u32 = 8;

    /// `dyne_imu_push` result: accepted.
    pub const PUSH_OK: i32 = 0;
    /// `dyne_imu_push` result: the sample's epoch is already closed (late; dropped and counted).
    pub const PUSH_LATE: i32 = 1;
    /// `dyne_imu_push` result: the sample is too far ahead (outside the buffer; dropped and counted).
    pub const PUSH_EARLY: i32 = 2;
    /// `dyne_imu_push` result: called before `dyne_init2d`.
    pub const PUSH_NOT_READY: i32 = -1;

    /// `dyne_get` index (`dyne_len` = `G_LEN`; angles in rad, acceleration in m/s², angular
    /// rate in rad/s): last fused epoch (−1 if none yet).
    pub const G_EPOCH: u32 = 0;
    /// `dyne_get` index: fraction of devices that had samples (tends to 1 if all had).
    pub const G_FUSED_N: u32 = 1;
    /// `dyne_get` index: fused ax, ay, az, gx, gy, gz (six values from here).
    pub const G_FUSED: u32 = 2;
    /// `dyne_get` index: fused roll.
    pub const G_FUSED_ROLL: u32 = 8;
    /// `dyne_get` index: fused pitch.
    pub const G_FUSED_PITCH: u32 = 9;
    /// `dyne_get` index: number of the device's own samples in the epoch (for comparison).
    pub const G_LOCAL_N: u32 = 10;
    /// `dyne_get` index: the device's own means (six values from here).
    pub const G_LOCAL: u32 = 11;
    /// `dyne_get` index: roll from the device's own samples.
    pub const G_LOCAL_ROLL: u32 = 17;
    /// `dyne_get` index: pitch from the device's own samples.
    pub const G_LOCAL_PITCH: u32 = 18;
    /// `dyne_get` index: count of late samples.
    pub const G_LATE: u32 = 19;
    /// `dyne_get` index: count of samples too far ahead.
    pub const G_EARLY: u32 = 20;
    /// `dyne_get` index: count of neighbour values that did not arrive.
    pub const G_MISSING: u32 = 21;
    /// `dyne_get` index (added 2026-11-27, issue #194): number of devices whose samples the last
    /// fused epoch used, `floor(G_FUSED_N · N + 0.5)` with N = `CFG_DEVICES` (0 if N is not set).
    /// A device without samples in an epoch shares weight 0 and is left out of the average.
    pub const G_FUSED_DEVICES: u32 = 22;
    /// Number of `dyne_get` values (22 before 2026-11-27; indices 0..21 are unchanged).
    pub const G_LEN: u32 = 23;
}
