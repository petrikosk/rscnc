//! Geometric path elements (the `SMC_GEOINFO` analogue): lines, arcs,
//! splines, dwell and synchronized events, all arc-length parameterized.

mod arc;
mod line;
mod spline;

pub use arc::ArcSeg;
pub use line::LineSeg;
pub use spline::SplineSeg;

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
    /// Position and unit tangent at path parameter `s` in one call (what the
    /// interpolator needs every cycle). The default simply calls
    /// [`point_at`](Self::point_at) and [`tangent_at`](Self::tangent_at);
    /// implementations may share work between the two.
    fn pose_at(&self, s: f64) -> (PathPoint, PathPoint) {
        (self.point_at(s), self.tangent_at(s))
    }
}

/// Closed set of segment geometries used in the queue's hot path.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Geometry {
    /// A straight line segment.
    Line(LineSeg),
    /// A circular (or helical) arc segment.
    Arc(ArcSeg),
    /// A polynomial spline segment (boxed: it carries its arc-length table).
    Spline(Box<SplineSeg>),
}

/// Samples used for the tangent and normal extents of a curved segment.
const EXTENT_SAMPLES: usize = 32;

impl Geometry {
    /// The highest curvature along the segment (0 for lines).
    pub fn max_curvature(&self) -> f64 {
        match self {
            Geometry::Line(_) => 0.0,
            // Monotone over the (spiral-corrected) arc: the higher end rules.
            Geometry::Arc(a) => a.curvature_at(0.0).max(a.curvature_at(a.length())),
            Geometry::Spline(s) => s.max_curvature(),
        }
    }

    /// The same geometry traversed from its end to its start.
    pub fn reversed(&self) -> Geometry {
        match self {
            Geometry::Line(l) => Geometry::Line(LineSeg::new(l.p1, l.p0)),
            Geometry::Arc(a) => Geometry::Arc(a.reversed()),
            Geometry::Spline(s) => Geometry::Spline(Box::new(s.reversed())),
        }
    }

    /// Largest magnitude of each component of the unit tangent, and of the
    /// unit normal (the direction of the centripetal acceleration), along the
    /// segment: how much of the path velocity and acceleration each path
    /// axis sees at most. Exact for lines; sampled, with a small allowance
    /// for what lies between the samples, for curves.
    pub fn axis_extents(&self) -> (PathPoint, PathPoint) {
        let len = self.length();
        if let Geometry::Line(l) = self {
            return (l.tangent_at(0.0).map(f64::abs), [0.0; 3]);
        }
        let mut tangent = [0.0f64; 3];
        let mut normal = [0.0f64; 3];
        let mut prev: Option<PathPoint> = None;
        for i in 0..=EXTENT_SAMPLES {
            let t = self.tangent_at(len * i as f64 / EXTENT_SAMPLES as f64);
            for k in 0..3 {
                tangent[k] = tangent[k].max(t[k].abs());
            }
            if let Some(p) = prev {
                let d = sub(&t, &p);
                let n = norm(&d);
                if n > 1e-12 {
                    for k in 0..3 {
                        normal[k] = normal[k].max((d[k] / n).abs());
                    }
                }
            }
            prev = Some(t);
        }
        // A component can peak between two samples: by 1 − cos of half the
        // sample spacing of the tangent's direction at most. One full turn
        // over the samples would be 0.5 %; allow for a little more.
        let pad = |e: f64| {
            if e > 0.0 {
                (e * 1.02 + 0.02).min(1.0)
            } else {
                0.0
            }
        };
        (tangent.map(pad), normal.map(pad))
    }

    /// Call `visit` with the points where the segment can reach furthest
    /// along a coordinate axis: its two ends, the axis extremes of an arc
    /// that lie inside its sweep, and — sampled — those of a spline.
    pub fn for_each_extreme(&self, mut visit: impl FnMut(PathPoint)) {
        visit(self.start());
        visit(self.end());
        match self {
            Geometry::Line(_) => {}
            Geometry::Arc(arc) => {
                let quarter = core::f64::consts::FRAC_PI_2;
                let (from, to) = if arc.sweep >= 0.0 {
                    (arc.theta0, arc.theta0 + arc.sweep)
                } else {
                    (arc.theta0 + arc.sweep, arc.theta0)
                };
                let mut k = (from / quarter).ceil();
                while k * quarter <= to {
                    let frac = if arc.sweep != 0.0 {
                        (k * quarter - arc.theta0) / arc.sweep
                    } else {
                        0.0
                    };
                    visit(arc.point_at(frac.clamp(0.0, 1.0) * arc.length()));
                    k += 1.0;
                }
            }
            Geometry::Spline(s) => {
                let (lo, hi) = s.bounding_box();
                visit(lo);
                visit(hi);
            }
        }
    }

    /// Axis-aligned bounding box of the segment (min corner, max corner).
    pub fn bounding_box(&self) -> (PathPoint, PathPoint) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        self.for_each_extreme(|p| {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        });
        (lo, hi)
    }
}

impl Segment for Geometry {
    fn length(&self) -> f64 {
        match self {
            Geometry::Line(l) => l.length(),
            Geometry::Arc(a) => a.length(),
            Geometry::Spline(sp) => sp.length(),
        }
    }
    fn point_at(&self, s: f64) -> PathPoint {
        match self {
            Geometry::Line(l) => l.point_at(s),
            Geometry::Arc(a) => a.point_at(s),
            Geometry::Spline(sp) => sp.point_at(s),
        }
    }
    fn tangent_at(&self, s: f64) -> PathPoint {
        match self {
            Geometry::Line(l) => l.tangent_at(s),
            Geometry::Arc(a) => a.tangent_at(s),
            Geometry::Spline(sp) => sp.tangent_at(s),
        }
    }
    fn curvature_at(&self, s: f64) -> f64 {
        match self {
            Geometry::Line(l) => l.curvature_at(s),
            Geometry::Arc(a) => a.curvature_at(s),
            Geometry::Spline(sp) => sp.curvature_at(s),
        }
    }
    fn start(&self) -> PathPoint {
        match self {
            Geometry::Line(l) => l.start(),
            Geometry::Arc(a) => a.start(),
            Geometry::Spline(sp) => sp.start(),
        }
    }
    fn end(&self) -> PathPoint {
        match self {
            Geometry::Line(l) => l.end(),
            Geometry::Arc(a) => a.end(),
            Geometry::Spline(sp) => sp.end(),
        }
    }
    #[inline]
    fn pose_at(&self, s: f64) -> (PathPoint, PathPoint) {
        match self {
            Geometry::Line(l) => l.pose_at(s),
            Geometry::Arc(a) => a.pose_at(s),
            Geometry::Spline(sp) => sp.pose_at(s),
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
    /// The same move traversed backward: reversed geometry and swapped
    /// auxiliary positions. Switch points are dropped (they belong to the
    /// forward direction); everything else is kept.
    pub fn reversed(&self) -> GeoMove {
        GeoMove {
            geo: self.geo.reversed(),
            aux_start: self.aux_end.clone(),
            aux_end: self.aux_start.clone(),
            switch_points: Vec::new(),
            ..self.clone()
        }
    }

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
/// (M functions, spindle and tool changes). An event forces an exact stop at
/// its queue position unless it is a *fly-by* event (see
/// [`SyncEvent::fly_by`]).
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
    /// `true` when the path does not stop for this event: the look-ahead
    /// blends the moves on its two sides and the interpolator reports the
    /// event in the cycle that crosses the junction. Set by the decoder for
    /// M functions listed in
    /// [`DecoderConfig::fly_by_m`](crate::decoder::DecoderConfig::fly_by_m).
    pub fly_by: bool,
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
    /// `true` when the path has to stand still at this element: a dwell, or
    /// an event that is not a fly-by event.
    pub fn is_stop(&self) -> bool {
        match self {
            GeoElem::Move(_) => false,
            GeoElem::Dwell { .. } => true,
            GeoElem::Event(e) => !e.fly_by,
        }
    }

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
