//! Modal program state (DIN 66025 modal groups) tracked by the decoder.

use crate::types::WorkPlane;

/// Modal motion group.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MotionMode {
    /// G0 - rapid traverse
    #[default]
    Rapid,
    /// G1 - linear interpolation
    Linear,
    /// G2 - circular interpolation, clockwise
    ArcCw,
    /// G3 - circular interpolation, counter-clockwise
    ArcCcw,
}

/// G90 / G91.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DistanceMode {
    /// G90 - axis words are absolute coordinates.
    #[default]
    Absolute,
    /// G91 - axis words are increments from the current position.
    Incremental,
}

/// G71/G21 (mm) or G70/G20 (inch).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Units {
    /// G71/G21 - millimeters.
    #[default]
    Mm,
    /// G70/G20 - inches.
    Inch,
}

impl Units {
    /// Scale factor from programmed units to internal mm.
    #[inline]
    pub fn to_mm(self) -> f64 {
        match self {
            Units::Mm => 1.0,
            Units::Inch => 25.4,
        }
    }
}

/// Tool radius correction mode (G40/G41/G42). Carried on segments and
/// consumed by the [`crate::preprocess::ToolRadiusCorrection`] preprocessor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TrcMode {
    /// G40 - correction off
    #[default]
    Off,
    /// G41 - tool left of the programmed path
    Left,
    /// G42 - tool right of the programmed path
    Right,
}

/// The decoder's modal state. Positions are in internal units (mm),
/// feed in internal units/s.
#[derive(Debug, Clone, PartialEq)]
pub struct ModalState {
    /// Active motion mode (G0/G1/G2/G3).
    pub motion: MotionMode,
    /// Active work plane (G17/G18/G19).
    pub plane: WorkPlane,
    /// Absolute or incremental distance mode (G90/G91).
    pub distance: DistanceMode,
    /// Active unit system (G70/G71).
    pub units: Units,
    /// Programmed feed, internal units/s (the `F` word, which is units/s).
    pub feed: f64,
    /// Programmed path acceleration from a positive `E` word, internal
    /// units/s². `None` leaves the machine limit as the only bound.
    pub accel: Option<f64>,
    /// Programmed path deceleration from a negative `E` word, stored as a
    /// positive magnitude, internal units/s².
    pub decel: Option<f64>,
    /// Programmed G0 feed from an `FF` word, internal units/s. `None` uses
    /// the decoder's configured rapid feed.
    pub rapid_feed: Option<f64>,
    /// Programmed G0 acceleration from a positive `EF` word, internal units/s².
    pub rapid_accel: Option<f64>,
    /// Programmed G0 deceleration from a negative `EF` word, internal units/s².
    pub rapid_decel: Option<f64>,
    /// Per-auxiliary-axis velocity limits from `F<axis>` words, in decoder
    /// order and in the auxiliary axes' own units/s.
    pub aux_feed: Vec<Option<f64>>,
    /// Per-auxiliary-axis acceleration limits from positive `E<axis>` words,
    /// in decoder order and in the auxiliary axes' own units/s².
    pub aux_accel: Vec<Option<f64>>,
    /// Per-auxiliary-axis deceleration limits from negative `E<axis>` words,
    /// positive magnitudes, in decoder order.
    pub aux_decel: Vec<Option<f64>>,
    /// Programmed spindle speed from the last S word.
    pub spindle_speed: f64,
    /// Active tool from the last T word.
    pub tool: u32,
    /// Active tool radius correction mode (G40/G41/G42).
    pub trc: TrcMode,
    /// Programmed position of the path axes (after G92 offset), internal units.
    pub position: [f64; 3],
    /// G92 offset: programmed = machine + offset, i.e. machine = programmed - offset.
    pub g92_offset: [f64; 3],
    /// Auxiliary (non-path) axis programmed positions, internal units.
    pub aux_position: Vec<f64>,
    /// G92 offsets of the auxiliary axes.
    pub aux_g92_offset: Vec<f64>,
}

impl ModalState {
    /// Initial modal state (all defaults, everything at 0) for a machine
    /// with `aux_axes` auxiliary axes.
    pub fn new(aux_axes: usize) -> Self {
        Self {
            motion: MotionMode::default(),
            plane: WorkPlane::default(),
            distance: DistanceMode::default(),
            units: Units::default(),
            feed: 0.0,
            accel: None,
            decel: None,
            rapid_feed: None,
            rapid_accel: None,
            rapid_decel: None,
            aux_feed: vec![None; aux_axes],
            aux_accel: vec![None; aux_axes],
            aux_decel: vec![None; aux_axes],
            spindle_speed: 0.0,
            tool: 0,
            trc: TrcMode::default(),
            position: [0.0; 3],
            g92_offset: [0.0; 3],
            aux_position: vec![0.0; aux_axes],
            aux_g92_offset: vec![0.0; aux_axes],
        }
    }
}
