//! Look-ahead planner: per-segment velocity caps and junction (corner)
//! velocities, so consecutive segments blend without stopping.

use crate::geometry::{dot, GeoElem, GeoMove, Segment};
use crate::queue::GeoQueue;
use crate::types::PathLimits;

/// Junction and reachability rules.
#[derive(Debug, Clone)]
pub struct LookAhead {
    /// Grbl-style junction deviation δ: the corner is treated as an arc that
    /// deviates at most δ from the programmed corner point;
    /// v² ≤ a_n · δ · sin(θ/2) / (1 − sin(θ/2)), where θ is the angle
    /// *between the two segments* (π for a straight continuation, 0 for a
    /// full reversal). Internal units (mm).
    ///
    /// On top of that, a junction between short segments is treated as a
    /// sample of a curve of radius R ≈ (L₁ + L₂) / (2·φ) (φ = deflection
    /// angle) and capped at v² ≤ a_n · R, so a finely segmented arc obeys the
    /// same centripetal budget as a true `G2`/`G3` arc.
    pub junction_deviation: f64,
    /// Fraction of `max_acceleration` and `max_jerk` used in the
    /// velocity-reachability passes. The passes use the exact jerk-limited
    /// ramp distance, so this is pure headroom (for override changes and
    /// numerical slack) rather than a correction for the jerk limit. Values
    /// above 1 are treated as 1.
    pub accel_margin: f64,
}

impl Default for LookAhead {
    fn default() -> Self {
        Self {
            junction_deviation: 0.05,
            accel_margin: 0.8,
        }
    }
}

/// Velocity plan for one `GeoElem::Move` of the queue (at 100% override).
#[derive(Debug, Clone)]
pub struct PlannedSegment {
    /// Index of the move in the queue.
    pub queue_index: usize,
    /// Planning length (see [`GeoMove::param_length`]).
    pub length: f64,
    /// Cruise velocity cap: min(programmed feed, machine cap, curvature cap,
    /// aux caps) — and, for a run of short segments on a tight curve, the
    /// junction velocity limit along it (see [`LookAhead::plan`]).
    pub v_cap: f64,
    /// Exit (junction) velocity at the end of this segment.
    pub v_exit: f64,
    /// Acceleration limit on this segment: the machine limit, tightened by a
    /// programmed positive `E` word.
    pub accel: f64,
    /// Deceleration limit on this segment: the machine limit, tightened by a
    /// programmed negative `E` word.
    pub decel: f64,
}

/// The look-ahead result plus the per-segment facts the interpolator needs
/// on top of the public [`PlannedSegment`]s (all three vectors are parallel).
pub(crate) struct DetailedPlan {
    pub(crate) segs: Vec<PlannedSegment>,
    /// Velocity ceiling per segment: [`LookAhead::velocity_limit`], and the
    /// junction limits inside the segment's span.
    pub(crate) v_limit: Vec<f64>,
    /// `blend[i]`: the junction after segment `i` does not constrain the
    /// velocity, so one jerk-limited profile may run straight through it.
    pub(crate) blend: Vec<bool>,
}

impl LookAhead {
    /// Plan the whole queue. `aux_max_velocity` are the auxiliary axes'
    /// velocity limits in decoder order (used to cap path velocity so a
    /// synchronized aux axis never exceeds its own limit).
    ///
    /// Consecutive moves with the same velocity cap form one *span* when
    /// their junctions do not constrain the velocity — or constrain it so
    /// evenly that nothing is lost by cruising at the junction limit (short
    /// segments on a tight curve). The interpolator traverses a span with a
    /// single jerk-limited profile, so a contour chopped into many short
    /// segments accelerates like one long move. A span held back by its
    /// junctions has their limit as the [`PlannedSegment::v_cap`] of all its
    /// segments. For a junction inside a span, [`PlannedSegment::v_exit`] is
    /// the upper bound of the velocity there; at a span end it is the
    /// planned exit velocity.
    pub fn plan(
        &self,
        queue: &GeoQueue,
        limits: &PathLimits,
        aux_max_velocity: &[f64],
    ) -> Vec<PlannedSegment> {
        self.plan_detailed(queue, limits, aux_max_velocity).segs
    }

    pub(crate) fn plan_detailed(
        &self,
        queue: &GeoQueue,
        limits: &PathLimits,
        aux_max_velocity: &[f64],
    ) -> DetailedPlan {
        let mut segs: Vec<PlannedSegment> = Vec::new();
        let mut v_limit: Vec<f64> = Vec::new();
        let mut moves: Vec<&GeoMove> = Vec::new();
        for (qi, elem) in queue.iter().enumerate() {
            if let GeoElem::Move(mv) = elem {
                let cap = |programmed: Option<f64>| {
                    programmed.map_or(limits.max_acceleration, |a| a.min(limits.max_acceleration))
                };
                let limit = self.velocity_limit(mv, limits, aux_max_velocity);
                segs.push(PlannedSegment {
                    queue_index: qi,
                    length: mv.param_length(),
                    v_cap: mv.feed.min(limit),
                    v_exit: 0.0,
                    accel: cap(mv.accel),
                    decel: cap(mv.decel),
                });
                v_limit.push(limit);
                moves.push(mv);
            }
        }

        // Junction velocity limit between consecutive moves. Any non-move
        // element in between (dwell, synchronized event) forces an exact stop.
        let n = segs.len();
        let mut v_junction = vec![0.0; n];
        for i in 0..n.saturating_sub(1) {
            if segs[i + 1].queue_index == segs[i].queue_index + 1 {
                v_junction[i] = self.junction_velocity(moves[i], moves[i + 1], limits);
            }
        }
        drop(moves);

        // Spans. Moves with the same caps are joined into one span as long
        // as running the whole span at one cruise velocity — the lowest of
        // the junction limits inside it — costs none of its moves more than
        // SPAN_TOLERANCE of the velocity it could reach on its own, i.e.
        // from and to its own junction limits with zero acceleration there.
        // That is the case when the junctions do not bind at all, and for
        // short moves between similar junction limits (a finely segmented
        // curve), which could not accelerate between them anyway.
        let j_max = limits.max_jerk;
        let peak = |i: usize, cap: f64| {
            let len = segs[i].length;
            let reach = |v: f64, a_max: f64| {
                if v >= cap || ramp_distance(v, cap, a_max, j_max) <= len {
                    cap
                } else {
                    reachable_velocity(v, len, a_max, j_max)
                }
            };
            let v_in = if i > 0 { v_junction[i - 1] } else { 0.0 };
            reach(v_in, segs[i].accel).min(reach(v_junction[i], segs[i].decel))
        };
        let mut blend = vec![false; n];
        // (first segment, last segment, cruise limit of the junctions inside)
        let mut joined: Vec<(usize, usize, f64)> = Vec::new();
        let mut i = 0;
        while i < n {
            let first = i;
            let (cap, limit) = (segs[i].v_cap, v_limit[i]);
            // One profile has one pair of acceleration limits
            let (accel, decel) = (segs[i].accel, segs[i].decel);
            let mut inner = f64::INFINITY;
            // Highest attainable velocity of the span's moves; only needed
            // (and only computed) once a junction inside the span binds.
            let mut peak_max: Option<f64> = None;
            while i + 1 < n
                && v_junction[i] > 0.0
                && segs[i + 1].v_cap == cap
                && v_limit[i + 1] == limit
                && segs[i + 1].accel == accel
                && segs[i + 1].decel == decel
            {
                let inner_next = inner.min(v_junction[i]);
                if inner_next < cap {
                    let so_far = peak_max
                        .unwrap_or_else(|| (first..=i).map(|k| peak(k, cap)).fold(0.0, f64::max));
                    let peak_next = so_far.max(peak(i + 1, cap));
                    peak_max = Some(peak_next);
                    if inner_next < (1.0 - SPAN_TOLERANCE) * peak_next {
                        break;
                    }
                }
                blend[i] = true;
                inner = inner_next;
                i += 1;
            }
            if inner < cap || inner < limit {
                joined.push((first, i, inner));
            }
            i += 1;
        }
        for (first, last, inner) in joined {
            for k in first..=last {
                segs[k].v_cap = segs[k].v_cap.min(inner);
                v_limit[k] = v_limit[k].min(inner);
            }
        }
        for i in 0..n.saturating_sub(1) {
            segs[i].v_exit = v_junction[i].min(segs[i].v_cap).min(segs[i + 1].v_cap);
        }

        // Reachability, per span: a span is traversed by one profile that
        // starts and ends with zero acceleration, so what must fit is the
        // jerk-limited ramp between its entry and exit velocities. The
        // margin derates both acceleration and jerk, leaving real slack on
        // short spans (where the ramp never reaches the acceleration limit).
        // The planned ramps must stay within what the interpolator can do,
        // and that with a hair to spare (`PLAN_SLACK`) even at a margin of 1.
        // Every segment of a span has the same acceleration limits: the
        // span's `accel` rules its ramp up, its `decel` the ramp down.
        let margin = self.accel_margin.min(1.0);
        let j_eff = limits.max_jerk * margin;
        // (first segment, last segment, length) of every span
        let mut spans: Vec<(usize, usize, f64)> = Vec::new();
        let mut first = 0;
        let mut length = 0.0;
        for i in 0..n {
            length += segs[i].length;
            if !blend[i] {
                spans.push((first, i, length));
                first = i + 1;
                length = 0.0;
            }
        }

        // Backward pass: a span's entry velocity (the previous span's exit)
        // must allow braking down to the span's own exit velocity.
        for k in (1..spans.len()).rev() {
            let (_, end, len) = spans[k];
            let prev_end = spans[k - 1].1;
            let (v_in, v_out) = (segs[prev_end].v_exit, segs[end].v_exit);
            let d_eff = segs[end].decel * margin;
            let fit = len * (1.0 - PLAN_SLACK);
            if v_in > v_out && ramp_distance(v_out, v_in, d_eff, j_eff) > fit {
                segs[prev_end].v_exit = v_in.min(reachable_velocity(v_out, fit, d_eff, j_eff));
            }
        }

        // Forward pass: and the exit velocity must be reachable *up* from
        // the entry velocity (the program starts at rest).
        let mut v_in = 0.0;
        for &(first, end, len) in &spans {
            let v_out = segs[end].v_exit;
            let a_eff = segs[end].accel * margin;
            let d_eff = segs[end].decel * margin;
            let fit = len * (1.0 - PLAN_SLACK);
            if v_out > v_in && ramp_distance(v_in, v_out, a_eff, j_eff) > fit {
                segs[end].v_exit = v_out.min(reachable_velocity(v_in, fit, a_eff, j_eff));
            }
            let v_out = segs[end].v_exit;
            // Junctions inside the span: bound the velocity by what the
            // ramps from the span's two ends allow.
            let cap = segs[first].v_cap;
            let mut done = 0.0;
            for seg in &mut segs[first..end] {
                done += seg.length;
                let rest = len - done;
                let mut bound = cap;
                if ramp_distance(v_in, cap, a_eff, j_eff) > done {
                    bound = bound.min(reachable_velocity(v_in, done, a_eff, j_eff));
                }
                if ramp_distance(v_out, cap, d_eff, j_eff) > rest {
                    bound = bound.min(reachable_velocity(v_out, rest, d_eff, j_eff));
                }
                seg.v_exit = bound;
            }
            v_in = v_out;
        }

        DetailedPlan {
            segs,
            v_limit,
            blend,
        }
    }

    /// The machine's velocity ceiling on a move, independent of the
    /// programmed feed: min(path velocity cap, curvature cap, auxiliary axis
    /// caps). A feed override above 100 % can raise a move's speed up to this
    /// value but never beyond it.
    pub fn velocity_limit(
        &self,
        mv: &GeoMove,
        limits: &PathLimits,
        aux_max_velocity: &[f64],
    ) -> f64 {
        let mut cap = limits.max_velocity;
        // Curvature cap: v ≤ sqrt(a_n / κ). Curvature is monotone over the
        // (spiral-corrected) segment, so the endpoint maximum is the segment maximum.
        let len = mv.geo.length();
        let kappa = mv.geo.curvature_at(0.0).max(mv.geo.curvature_at(len));
        if kappa > 0.0 {
            cap = cap.min((limits.max_normal_acceleration / kappa).sqrt());
        }
        // Aux caps: an aux axis travelling Δ over parameter length L moves at
        // v_aux = v · Δ/L, so v ≤ v_aux_max · L/Δ.
        let param_len = if len > 1e-12 { len } else { mv.param_length() };
        if param_len > 0.0 {
            for (i, (a0, a1)) in mv.aux_start.iter().zip(mv.aux_end.iter()).enumerate() {
                let delta = (a1 - a0).abs();
                if delta > 0.0 {
                    if let Some(v_aux) = aux_max_velocity.get(i) {
                        cap = cap.min(v_aux * param_len / delta);
                    }
                }
            }
        }
        cap
    }

    /// Grbl-style junction-deviation corner velocity between two moves.
    ///
    /// The corner is evaluated in the *generalized* tangent space (path
    /// tangent plus the auxiliary axes' rates dA/ds), so a change of an
    /// auxiliary axis' rate at an otherwise straight junction slows the
    /// junction down just like a geometric corner does.
    fn junction_velocity(&self, a: &GeoMove, b: &GeoMove, limits: &PathLimits) -> f64 {
        let len_a = a.geo.length();
        let len_b = b.geo.length();
        let t_out = a.geo.tangent_at(len_a);
        let t_in = b.geo.tangent_at(0.0);
        // A pure aux move has no tangent: the path axes stand still anyway.
        if (t_out[0] == 0.0 && t_out[1] == 0.0 && t_out[2] == 0.0)
            || (t_in[0] == 0.0 && t_in[1] == 0.0 && t_in[2] == 0.0)
        {
            return 0.0;
        }
        // Generalized tangents (t, r₁, r₂, …) with rᵢ = dAᵢ/ds; their norms
        // are √(1 + Σr²).
        let mut dot_ab = dot(&t_out, &t_in);
        let (mut n_a, mut n_b) = (1.0, 1.0);
        for i in 0..a.aux_start.len().max(b.aux_start.len()) {
            let r_a = aux_rate(a, i, len_a);
            let r_b = aux_rate(b, i, len_b);
            dot_ab += r_a * r_b;
            n_a += r_a * r_a;
            n_b += r_b * r_b;
        }
        let (n_a, n_b): (f64, f64) = (n_a.sqrt(), n_b.sqrt());
        let cos_phi = (dot_ab / (n_a * n_b)).clamp(-1.0, 1.0);
        if cos_phi < -1.0 + 1e-12 {
            return 0.0; // full reversal
        }
        // sin(θ/2) with θ = π − φ the angle between the segments.
        let sin_half = ((1.0 + cos_phi) / 2.0).sqrt();
        if sin_half > 1.0 - 1e-12 {
            return f64::INFINITY; // colinear: other caps rule
        }
        let a_n = limits.max_normal_acceleration;
        let mut v_sqr = a_n * self.junction_deviation * sin_half / (1.0 - sin_half);
        // Finely segmented curve: the junction samples a curve of radius
        // R ≈ (L₁ + L₂) / (2φ); keep the centripetal budget on it.
        let phi = cos_phi.acos();
        v_sqr = v_sqr.min(a_n * (len_a * n_a + len_b * n_b) / (2.0 * phi));
        // Generalized speed → path speed.
        v_sqr.sqrt() / n_a.max(n_b)
    }
}

/// Share of a move's attainable velocity that may be given up for running
/// it in one span with its neighbours (see [`LookAhead::plan`]).
const SPAN_TOLERANCE: f64 = 0.1;

/// Relative part of a span's length the planned ramps leave unused, whatever
/// the acceleration margin. The interpolator never asks the trajectory
/// generator for a profile on the exact edge of feasibility (it keeps half
/// of this back itself), so a plan without any slack would be one it cannot
/// follow: each span would be left a little too fast, and a planned stop at
/// the end of the chain overrun.
pub(crate) const PLAN_SLACK: f64 = 2e-6;

/// Rate dA/ds of auxiliary axis `i` over a move with geometric length `len`.
fn aux_rate(mv: &GeoMove, i: usize, len: f64) -> f64 {
    match (mv.aux_start.get(i), mv.aux_end.get(i)) {
        (Some(a0), Some(a1)) if len > 0.0 => (a1 - a0) / len,
        _ => 0.0,
    }
}

/// Distance covered by a jerk-limited velocity change `v0 → v1` that starts
/// and ends with zero acceleration (symmetric in its arguments).
#[inline]
pub(crate) fn ramp_distance(v0: f64, v1: f64, a_max: f64, j_max: f64) -> f64 {
    let dv = (v1 - v0).abs();
    let t = if dv * j_max >= a_max * a_max {
        dv / a_max + a_max / j_max
    } else {
        2.0 * (dv / j_max).sqrt()
    };
    0.5 * (v0 + v1) * t
}

/// The highest velocity reachable from `v0 ≥ 0` over `dist` with a
/// jerk-limited ramp that starts and ends with zero acceleration — and, by
/// symmetry, the highest velocity from which `v0` can still be reached by
/// braking over `dist`.
pub(crate) fn reachable_velocity(v0: f64, dist: f64, a_max: f64, j_max: f64) -> f64 {
    if dist.is_nan() || dist <= 0.0 {
        return v0;
    }
    if dist == f64::INFINITY {
        return f64::INFINITY;
    }
    // Triangular acceleration profile: with x = √(Δv / j) the distance is
    // (2·v0 + j·x²)·x, so x solves j·x³ + 2·v0·x − dist = 0. The left side is
    // increasing and convex for x ≥ 0, so Newton from an upper bound of the
    // root converges monotonically.
    let mut x = (dist / j_max).cbrt();
    if v0 > 0.0 {
        x = x.min(dist / (2.0 * v0));
    }
    for _ in 0..32 {
        let f = (j_max * x * x + 2.0 * v0) * x - dist;
        let df = 3.0 * j_max * x * x + 2.0 * v0;
        let step = f / df;
        x -= step;
        if step.abs() <= 1e-15 * x.abs() {
            break;
        }
    }
    let dv = j_max * x * x;
    if dv * j_max <= a_max * a_max {
        return v0 + dv;
    }
    // Trapezoidal acceleration profile:
    // (v1² − v0²)/(2a) + (v0 + v1)·a/(2j) = dist, a quadratic in v1.
    let k = a_max * a_max / j_max;
    let disc = k * k - 4.0 * (v0 * k - v0 * v0 - 2.0 * a_max * dist);
    0.5 * (-k + disc.max(0.0).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::{Decoder, DecoderConfig};

    fn limits() -> PathLimits {
        PathLimits::new(50.0, 500.0, 5000.0, 500.0)
    }

    fn plan(src: &str) -> Vec<PlannedSegment> {
        let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        LookAhead::default().plan(&q, &limits(), &[])
    }

    #[test]
    fn colinear_junction_keeps_feed() {
        let p = plan("G1 F20 X10\nX20\nX30\n");
        assert_eq!(p.len(), 3);
        // F is units/s; colinear junctions blend at full feed
        assert!((p[0].v_exit - 20.0).abs() < 1e-9, "v_exit {}", p[0].v_exit);
        assert!((p[1].v_exit - 20.0).abs() < 1e-9);
        assert!(p[2].v_exit == 0.0);
    }

    #[test]
    fn right_angle_slows_reversal_stops() {
        let p = plan("G1 F20 X10\nY10\nX0\nX10\n");
        // 90° corner: finite but > 0
        assert!(p[0].v_exit > 0.0 && p[0].v_exit < 20.0);
        // reversal X0 -> X10 must stop
        assert!(p[2].v_exit == 0.0);
    }

    #[test]
    fn event_forces_stop_and_backward_pass_propagates() {
        let p = plan("G1 F1200 X10\nX20 M8\nX30\n");
        // M8 event sits between move 1 and move 2 in the queue
        assert!(p[1].v_exit == 0.0);
        // short final segment from rest: forward pass limits its exit to 0 anyway
        assert!(p[2].v_exit == 0.0);
    }

    #[test]
    fn arc_curvature_caps_velocity() {
        // Radius 2 arc: v ≤ sqrt(500 * 2) ≈ 31.6, but feed 40 mm/s on a line.
        // The junction line→arc is tangent-continuous, so the exit velocity
        // is capped by the arc's curvature cap, not the corner rule.
        let p = plan("G1 F40 X10\nG3 X12 Y2 I0 J2\n");
        let arc_cap = (500.0f64 * 2.0).sqrt();
        assert!(p[0].v_exit <= arc_cap + 1e-9, "v_exit {}", p[0].v_exit);
        assert!(p[1].v_cap <= arc_cap.min(40.0) + 1e-9);
        assert!(
            p[1].v_cap > arc_cap - 1e-9,
            "curvature should be the binding cap"
        );
    }

    #[test]
    fn short_segments_limit_reachability() {
        // 0.1 mm segments at high feed: forward pass must keep exits low
        let p = plan("G1 F100 X0.1\nX0.2\nX0.3\n");
        let a_eff = 500.0 * 0.8;
        let reach1 = (2.0f64 * a_eff * 0.1).sqrt();
        assert!(p[0].v_exit <= reach1 + 1e-9);
        assert!(p[1].v_exit <= (reach1 * reach1 + 2.0 * a_eff * 0.1).sqrt() + 1e-9);
    }

    /// Planned exit velocity of the first of two 10 mm moves meeting at a
    /// corner with the given deflection angle.
    fn corner_exit(deflection_deg: f64) -> f64 {
        let phi = deflection_deg.to_radians();
        let p = plan(&format!(
            "G1 F50 X10\nX{:.9} Y{:.9}\n",
            10.0 + 10.0 * phi.cos(),
            10.0 * phi.sin()
        ));
        p[0].v_exit
    }

    #[test]
    fn junction_velocity_falls_with_the_deflection_angle() {
        // The gentler the corner, the faster it may be taken.
        let exits: Vec<f64> = [5.0, 20.0, 45.0, 90.0, 135.0, 170.0]
            .iter()
            .map(|d| corner_exit(*d))
            .collect();
        for w in exits.windows(2) {
            assert!(w[0] > w[1], "not decreasing: {exits:?}");
        }
        // A barely noticeable corner does not slow the path down at all ...
        assert!((corner_exit(1.0) - 50.0).abs() < 1e-9);
        // ... a right angle gets v² = a_n·δ·sin(θ/2)/(1 − sin(θ/2)), θ = 90°
        let la = LookAhead::default();
        let sin_half = std::f64::consts::FRAC_1_SQRT_2;
        let expected = (500.0 * la.junction_deviation * sin_half / (1.0 - sin_half)).sqrt();
        assert!(
            (corner_exit(90.0) - expected).abs() < 1e-9,
            "{} vs {expected}",
            corner_exit(90.0)
        );
        // ... and a reversal is a full stop.
        assert_eq!(corner_exit(180.0), 0.0);
    }

    #[test]
    fn fine_polygon_is_limited_like_the_arc_it_samples() {
        // 1° steps on a radius of 2 mm: the centripetal budget allows
        // v = √(a_n·r) = √1000 there, whatever the corner formula says.
        let r = 2.0;
        let mut src = format!("G1 F50 X{r} Y0\n");
        for k in 1..=1440 {
            let th = (k as f64).to_radians();
            src += &format!("X{:.9} Y{:.9}\n", r * th.cos(), r * th.sin());
        }
        let p = plan(&src);
        let v_arc = (500.0f64 * r).sqrt();
        // (four turns: well clear of the ramps at both ends)
        for seg in &p[600..840] {
            assert!(
                seg.v_exit <= v_arc * 1.001 && seg.v_exit >= v_arc * 0.95,
                "v_exit {} vs arc limit {v_arc}",
                seg.v_exit
            );
            // the whole run is one span cruising at the junction limit
            assert!(seg.v_cap <= v_arc * 1.001 && seg.v_cap >= v_arc * 0.95);
        }
    }

    #[test]
    fn aux_rate_change_slows_a_straight_junction() {
        let cfg = DecoderConfig {
            aux_letters: vec!['A'],
            start_aux: vec![0.0],
            ..Default::default()
        };
        let plan_aux = |src: &str| {
            let q = Decoder::decode_source(cfg.clone(), src).unwrap();
            LookAhead::default().plan(&q, &limits(), &[1000.0])
        };
        // same aux rate on both sides: a plain colinear junction
        let same = plan_aux("G1 F20 X20 A20\nX40 A40\n");
        assert!((same[0].v_exit - 20.0).abs() < 1e-9, "{}", same[0].v_exit);
        // the aux axis reverses: its velocity would jump at full feed
        let reversed = plan_aux("G1 F20 X20 A20\nX40 A0\n");
        assert!(reversed[0].v_exit < 10.0, "{}", reversed[0].v_exit);
    }

    #[test]
    fn ramp_helpers_are_consistent() {
        let (a, j) = (500.0, 5000.0);
        for v0 in [0.0, 0.3, 7.0, 40.0] {
            for dist in [1e-6, 1e-3, 0.02, 0.7, 5.0, 300.0] {
                // reachable_velocity inverts ramp_distance
                let v1 = reachable_velocity(v0, dist, a, j);
                assert!(v1 > v0);
                let back = ramp_distance(v0, v1, a, j);
                assert!(
                    (back - dist).abs() <= 1e-9 * dist.max(1.0),
                    "v0 {v0} dist {dist}: v1 {v1} needs {back}"
                );
            }
        }
        // short ramps are jerk-limited: far less than the v² = 2·a·d of a
        // constant-acceleration ramp
        let v = reachable_velocity(0.0, 0.01, a, j);
        assert!(v < 0.5 * (2.0 * a * 0.01f64).sqrt(), "{v}");
        // long ramps approach it from below
        let v = reachable_velocity(0.0, 1000.0, a, j);
        let v_const = (2.0 * a * 1000.0f64).sqrt();
        assert!(v < v_const && v > 0.9 * v_const, "{v} vs {v_const}");
        assert_eq!(ramp_distance(3.0, 3.0, a, j), 0.0);
        assert_eq!(reachable_velocity(3.0, 0.0, a, j), 3.0);
    }

    #[test]
    fn planned_velocities_are_reachable_with_the_jerk_limit() {
        // Stop-to-stop over a few short moves: every boundary velocity must
        // be reachable from standstill within the path so far, and standstill
        // from it within the rest (the old constant-acceleration estimate
        // promised several times more).
        let lim = limits();
        let p = plan("G1 F50 X0.2\nX0.4\nX0.6\nX0.8\nX1.0\n");
        let total: f64 = p.iter().map(|s| s.length).sum();
        let mut done = 0.0;
        for seg in &p {
            done += seg.length;
            let up = reachable_velocity(0.0, done, lim.max_acceleration, lim.max_jerk);
            let down = reachable_velocity(0.0, total - done, lim.max_acceleration, lim.max_jerk);
            assert!(
                seg.v_exit <= up.min(down) + 1e-9,
                "v_exit {} at s={done} exceeds reachable {}",
                seg.v_exit,
                up.min(down)
            );
        }
        assert_eq!(p.last().unwrap().v_exit, 0.0);
    }
}
