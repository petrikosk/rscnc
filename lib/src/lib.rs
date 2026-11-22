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
//!
//! Where to start:
//!
//! - [`CncController`] ties the stages together: load a program text (or feed
//!   it in pieces while the path runs), call `cycle` every control period.
//! - [`decoder`] documents the G-code dialect: motion words, coordinate
//!   systems, splines, variables and program flow.
//! - [`preprocess`] holds the path transforms: tool radius correction,
//!   corner rounding and smoothing, curvature velocity limit.
//! - [`lookahead::LookAhead`] and [`types::AxisConfig`] hold the planning
//!   parameters and the path and per-axis limits.
//! - [`interpolator::Interpolator`] is the cyclic part: override, halt,
//!   single-step, backward motion, streaming.
//!
//! Feeds are in units per **second** (`F50` is 50 mm/s), as in CODESYS
//! SoftMotion.

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
    pub use crate::geometry::{
        ArcSeg, GeoElem, GeoMove, Geometry, LineSeg, Segment, SplineSeg, SwitchPoint, SyncEvent,
    };
    pub use crate::interpolator::{Interpolator, IplOutput, IplState};
    pub use crate::kinematics::{CartesianKinematics, Kinematics};
    pub use crate::lookahead::{LookAhead, PlannedSegment};
    pub use crate::preprocess::{
        CornerRounding, CornerSmoothing, CurvatureVelocityLimit, Pipeline, Preprocessor,
        ToolRadiusCorrection,
    };
    pub use crate::queue::GeoQueue;
    pub use crate::types::{
        AxisConfig, AxisLimit, AxisLimits, PathAxes, PathLimits, PathPoint, WorkPlane,
    };
}
