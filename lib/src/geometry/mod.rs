//! Geometric path elements (the `SMC_GEOINFO` analogue): lines, arcs,
//! dwell and synchronized events, all arc-length parameterized.

mod arc;
mod line;

pub use arc::ArcSeg;
pub use line::LineSeg;

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
    /// Programmed path acceleration limit for this move (internal units/s²)
    /// from an `E` word; `None` leaves the machine limit as the only bound.
    pub accel: Option<f64>,
    /// Programmed path deceleration limit for this move (internal units/s²,
    /// a positive magnitude) from a negative `E` word.
    pub decel: Option<f64>,
    /// Switch points (`H` functions) programmed on this move, sorted by
    /// position along the element.
    pub switch_points: Vec<SwitchPoint>,
    /// 1-based source line the move was programmed on.
    pub src_line: usize,
    /// Monotone object number (the `SMC_GEOINFO.iObjNo` analogue).
    pub obj_no: u32,
}

/// A switch point on a path element: the DIN 66025 `H` function, positioned
/// with `L` (absolute distance from the element's start, or from its end when
/// negative) or `O` (relative position in `[0, 1]`).
///
/// The interpolator reports one as it crosses it, so the application can
/// drive an output in sync with the path rather than with the block boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwitchPoint {
    /// Programmed `H` number: positive switches the function on, negative off.
    pub number: i32,
    /// Distance from the start of the path element, internal units.
    pub position: f64,
    /// 1-based source line the switch point was programmed on.
    pub src_line: usize,
}

impl SwitchPoint {
    /// The switch function's number, without the on/off sign.
    #[inline]
    pub fn id(&self) -> u32 {
        self.number.unsigned_abs()
    }

    /// `true` when the point switches its function on (a positive `H`).
    #[inline]
    pub fn is_on(&self) -> bool {
        self.number > 0
    }
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

    /// Move the switch points of an element whose start was cut back by
    /// `shift` and which is now `new_length` long (corner rounding). Positions
    /// keep their distance to the geometry that survives; a point inside the
    /// removed piece lands on the new element boundary rather than vanishing.
    pub fn shift_switch_points(&mut self, shift: f64, new_length: f64) {
        let end = new_length.max(0.0);
        for sp in &mut self.switch_points {
            sp.position = (sp.position - shift).clamp(0.0, end);
        }
    }

    /// Map the switch points onto a replacement element of a different length
    /// (tool radius correction offsets an element in parallel, so the relative
    /// position along it is what stays meaningful).
    pub fn scale_switch_points(&mut self, old_length: f64, new_length: f64) {
        if self.switch_points.is_empty() {
            return;
        }
        let factor = if old_length > 1e-12 {
            new_length / old_length
        } else {
            0.0
        };
        for sp in &mut self.switch_points {
            sp.position = (sp.position * factor).clamp(0.0, new_length.max(0.0));
        }
    }
}

/// A synchronized event fired when the path crosses its queue position
/// (M functions, spindle and tool changes). In v1 events force an exact stop
/// at their queue position.
///
/// `H` functions are *not* events: they are positioned inside a path element
/// and reported as [`SwitchPoint`]s while the interpolator crosses them.
#[derive(Debug, Clone, PartialEq)]
pub struct SyncEvent {
    /// M function numbers in programmed order.
    pub m: Vec<u32>,
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

/// The tighter of two optional limits, where `None` means "unbounded".
pub(crate) fn min_opt(a: Option<f64>, b: Option<f64>) -> Option<f64> {
    match (a, b) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (Some(a), None) => Some(a),
        (None, b) => b,
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
