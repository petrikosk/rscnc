//! Geometric path elements (the `SMC_GEOINFO` analogue): lines, arcs,
//! dwell and synchronized events, all arc-length parameterized.

mod arc;
mod line;

pub use arc::ArcSeg;
pub use line::LineSeg;

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::gcode::TrcMode;
use crate::types::PathPoint;

/// Common interface of geometric segments: exact arc-length parameterization
/// `s ∈ [0, length()]` plus curvature for velocity planning.
pub trait Segment {
    /// Exact path length of the segment.
    fn length(&self) -> f64;
    /// Position at path parameter `s` (clamped to `[0, length]`).
    fn point_at(&self, s: f64) -> PathPoint;
    /// Unit tangent at path parameter `s`.
    fn tangent_at(&self, s: f64) -> PathPoint;
    /// Curvature κ ≥ 0 at path parameter `s` (0 for lines).
    fn curvature_at(&self, s: f64) -> f64;
    /// Start point of the segment (`point_at(0)`).
    fn start(&self) -> PathPoint;
    /// End point of the segment (`point_at(length())`).
    fn end(&self) -> PathPoint;
}

/// Closed set of segment geometries used in the queue's hot path.
#[derive(Debug, Clone, PartialEq)]
pub enum Geometry {
    /// A straight line segment.
    Line(LineSeg),
    /// A circular (or helical) arc segment.
    Arc(ArcSeg),
}

impl Segment for Geometry {
    fn length(&self) -> f64 {
        match self {
            Geometry::Line(l) => l.length(),
            Geometry::Arc(a) => a.length(),
        }
    }
    fn point_at(&self, s: f64) -> PathPoint {
        match self {
            Geometry::Line(l) => l.point_at(s),
            Geometry::Arc(a) => a.point_at(s),
        }
    }
    fn tangent_at(&self, s: f64) -> PathPoint {
        match self {
            Geometry::Line(l) => l.tangent_at(s),
            Geometry::Arc(a) => a.tangent_at(s),
        }
    }
    fn curvature_at(&self, s: f64) -> f64 {
        match self {
            Geometry::Line(l) => l.curvature_at(s),
            Geometry::Arc(a) => a.curvature_at(s),
        }
    }
    fn start(&self) -> PathPoint {
        match self {
            Geometry::Line(l) => l.start(),
            Geometry::Arc(a) => a.start(),
        }
    }
    fn end(&self) -> PathPoint {
        match self {
            Geometry::Line(l) => l.end(),
            Geometry::Arc(a) => a.end(),
        }
    }
}

/// A programmed move: geometry plus everything the planner needs.
#[derive(Debug, Clone, PartialEq)]
pub struct GeoMove {
    /// The move's path geometry.
    pub geo: Geometry,
    /// Programmed feed (internal units/s); the rapid feed for G0 moves.
    pub feed: f64,
    /// `true` for G0 rapid traverse moves.
    pub is_rapid: bool,
    /// Auxiliary-axis positions at segment start/end; interpolated linearly over `s`.
    pub aux_start: Vec<f64>,
    /// Auxiliary-axis positions at segment end; see [`GeoMove::aux_start`].
    pub aux_end: Vec<f64>,
    /// Tool radius correction mode active for this move.
    pub trc: TrcMode,
    /// 1-based source line the move was programmed on.
    pub src_line: usize,
    /// Monotone object number (the `SMC_GEOINFO.iObjNo` analogue).
    pub obj_no: u32,
}

impl GeoMove {
    /// Planning length of the move: the geometric path length, except for
    /// pure auxiliary-axis moves (zero geometric displacement), where the
    /// largest auxiliary displacement is used as the parameter length so the
    /// move can still be interpolated over `s`.
    pub fn param_length(&self) -> f64 {
        let geo_len = self.geo.length();
        if geo_len > 1e-12 {
            return geo_len;
        }
        self.aux_start
            .iter()
            .zip(self.aux_end.iter())
            .map(|(a, b)| (b - a).abs())
            .fold(0.0, f64::max)
    }
}

/// A synchronized event fired when the path crosses its queue position
/// (M functions with optional H parameter, spindle and tool changes).
/// In v1 events force an exact stop at their queue position.
#[derive(Debug, Clone, PartialEq)]
pub struct SyncEvent {
    /// M function numbers in programmed order.
    pub m: Vec<u32>,
    /// H parameter accompanying the M functions, if any.
    pub h: Option<f64>,
    /// New spindle speed from an S word, if any.
    pub spindle: Option<f64>,
    /// New tool from a T word, if any.
    pub tool: Option<u32>,
    /// 1-based source line the event was programmed on.
    pub src_line: usize,
}

/// One element of the geometry queue.
#[derive(Debug, Clone, PartialEq)]
pub enum GeoElem {
    /// A programmed move (line or arc).
    Move(GeoMove),
    /// G4 dwell.
    Dwell {
        /// Dwell duration in seconds.
        seconds: f64,
        /// 1-based source line the dwell was programmed on.
        src_line: usize,
    },
    /// A synchronized event (M/S/T).
    Event(SyncEvent),
}

impl GeoElem {
    /// 1-based source line the element was programmed on.
    pub fn src_line(&self) -> usize {
        match self {
            GeoElem::Move(m) => m.src_line,
            GeoElem::Dwell { src_line, .. } => *src_line,
            GeoElem::Event(e) => e.src_line,
        }
    }
}

pub(crate) fn sub(a: &PathPoint, b: &PathPoint) -> PathPoint {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn norm(v: &PathPoint) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

pub(crate) fn dot(a: &PathPoint, b: &PathPoint) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
