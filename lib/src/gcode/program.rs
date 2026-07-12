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
    #[default]
    Absolute,
    Incremental,
}

/// G71/G21 (mm) or G70/G20 (inch).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Units {
    #[default]
    Mm,
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
    pub motion: MotionMode,
    pub plane: WorkPlane,
    pub distance: DistanceMode,
    pub units: Units,
    /// Programmed feed, internal units/s (F is units/min in the source).
    pub feed: f64,
    pub spindle_speed: f64,
    pub tool: u32,
    pub trc: TrcMode,
    /// Programmed position of the path axes (after G92 offset), internal units.
    pub position: [f64; 3],
    /// G92 offset: programmed = machine + offset, i.e. machine = programmed - offset.
    pub g92_offset: [f64; 3],
    /// Auxiliary (non-path) axis programmed positions, internal units.
    pub aux_position: Vec<f64>,
    pub aux_g92_offset: Vec<f64>,
}

impl ModalState {
    pub fn new(aux_axes: usize) -> Self {
        Self {
            motion: MotionMode::default(),
            plane: WorkPlane::default(),
            distance: DistanceMode::default(),
            units: Units::default(),
            feed: 0.0,
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
