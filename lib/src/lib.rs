//! # rscnc
//!
//! A CNC motion-control layer on top of the [rsruckig](https://crates.io/crates/rsruckig)
//! jerk-limited trajectory generator, modeled after the CoDeSys SoftMotion CNC pipeline:
//!
//! ```text
//! DIN 66025 G-code ──► Decoder ──► GeoQueue ──► Preprocessors ──► Interpolator ──► Kinematics ──► axis setpoints
//! (text)               (modal      (lines,      (corner round,    (look-ahead +    (path space
//!                       state)      arcs,        tool radius       Ruckig 1-DoF     to axis
//!                                   events)      corr., ...)       s-profile)       space)
//! ```
//!
//! The interpolator runs rsruckig as a **1-DoF planner on the path parameter `s`**
//! (arc length). Look-ahead computes junction velocities so segments blend without
//! stopping; feed override retargets the profile instantly on any cycle.

#![warn(missing_docs)]

pub mod decoder;
pub mod driver;
pub mod error;
pub mod gcode;
pub mod geometry;
pub mod interpolator;
pub mod kinematics;
pub mod lookahead;
pub mod preprocess;
pub mod queue;
pub mod types;

mod controller;
pub use controller::CncController;

/// Convenience re-exports of the types needed for typical use.
pub mod prelude {
    pub use crate::controller::CncController;
    pub use crate::decoder::{Decoder, DecoderConfig};
    pub use crate::driver::{AxisDriver, SimDriver, TrajectoryLog};
    pub use crate::error::{CncError, DecodeError, GcodeError};
    pub use crate::gcode::lexer::lex;
    pub use crate::geometry::{ArcSeg, GeoElem, GeoMove, Geometry, LineSeg, Segment, SyncEvent};
    pub use crate::interpolator::{Interpolator, IplOutput, IplState};
    pub use crate::kinematics::{CartesianKinematics, Kinematics};
    pub use crate::lookahead::{LookAhead, PlannedSegment};
    pub use crate::preprocess::{
        CornerRounding, CurvatureVelocityLimit, Pipeline, Preprocessor, ToolRadiusCorrection,
    };
    pub use crate::queue::GeoQueue;
    pub use crate::types::{AxisConfig, PathAxes, PathLimits, PathPoint, WorkPlane};
}
