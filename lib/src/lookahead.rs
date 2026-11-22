//! Look-ahead planner: per-segment velocity caps and junction (corner)
//! velocities, so consecutive segments blend without stopping.

use crate::geometry::{dot, GeoElem, GeoMove, Segment};
use crate::queue::GeoQueue;
use crate::types::{AxisLimit, AxisLimits, PathLimits};

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
    /// Carry acceleration across span boundaries. Without it every span
    /// starts and ends with zero acceleration, which makes a chain of short
    /// spans with stepwise changing velocity limits (a spline or an ellipse
    /// cut into pieces of different curvature, a run of small feed changes)
    /// brake and accelerate in many small S-curves instead of one ramp. With
    /// it the look-ahead plans, where that is faster, boundary states with
    /// a non-zero acceleration ([`PlannedSegment::v_carry`],
    /// [`PlannedSegment::a_carry`]): a braking ramp that runs on down a row
    /// of falling velocity limits, an accelerating one up a row of rising
    /// limits. On by default.
    ///
    /// The carried ramps use at most 90 % of the acceleration, deceleration
    /// and jerk limits (less where [`accel_margin`](Self::accel_margin) is
    /// lower), and never lift the velocity above a move's cap or a junction
    /// limit. The zero-acceleration plan ([`PlannedSegment::v_exit`]) is
    /// planned as before and stays the one the interpolator follows wherever
    /// it finds no profile that leads straight to a carried state — after an
    /// override change or a halt, for instance. Inside a carried braking
    /// ramp that plan can be out of reach; should the ramp not be
    /// continuable there either, the path brakes to a standstill
    /// (`IplState::Pausing`) and goes on from rest by itself.
    pub carry_acceleration: bool,
}

impl Default for LookAhead {
    fn default() -> Self {
        Self {
            junction_deviation: 0.05,
            accel_margin: 0.8,
            carry_acceleration: true,
        }
    }
}

/// Velocity plan for one `GeoElem::Move` of the queue (at 100% override).
#[derive(Debug, Clone)]
#[non_exhaustive]
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
    /// The velocity limit at the junction at the end of this segment: the
    /// corner rule and the axes' velocity-jump limits, and the velocity
    /// ceilings of the moves on its two sides. 0 where the path stops.
    pub v_junction: f64,
    /// Acceleration limit on this segment: the machine limit, tightened by a
    /// programmed positive `E` word.
    pub accel: f64,
    /// Deceleration limit on this segment: the machine limit, tightened by a
    /// programmed negative `E` word.
    pub decel: f64,
    /// Jerk limit on this segment: the path limit, tightened by the axes
    /// that take part in the move.
    pub jerk: f64,
    /// Velocity at the end of this segment on the *carried* plan (see
    /// [`LookAhead::carry_acceleration`]); equal to
    /// [`v_exit`](Self::v_exit) where no acceleration is carried.
    pub v_carry: f64,
    /// Path acceleration at the end of this segment on the carried plan: 0
    /// where the profile ends with zero acceleration, as it always does on
    /// the plain plan.
    pub a_carry: f64,
}

/// A sequence of path elements the planner can index: a [`GeoQueue`] or the
/// interpolator's own storage.
pub(crate) trait Elems {
    fn elem(&self, i: usize) -> Option<&GeoElem>;
    fn count(&self) -> usize;
}

impl Elems for GeoQueue {
    fn elem(&self, i: usize) -> Option<&GeoElem> {
        self.get(i)
    }
    fn count(&self) -> usize {
        self.len()
    }
}

impl Elems for [GeoElem] {
    fn elem(&self, i: usize) -> Option<&GeoElem> {
        self.get(i)
    }
    fn count(&self) -> usize {
        self.len()
    }
}

/// What the planner has to know about a path that is not planned in one
/// piece from its very start (see `Interpolator::append`).
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct PlanContext<'a> {
    /// Queue indices at which a separately appended piece begins. The
    /// junction there never joins two moves into one span, so appending
    /// leaves the spans before it as they were.
    pub(crate) seams: &'a [usize],
    /// Velocity the first move is entered with (0 for a path planned from
    /// its start).
    pub(crate) entry_velocity: f64,
    /// Junction velocity limit before the first move.
    pub(crate) entry_junction: f64,
    /// State (velocity, acceleration) the first move is entered with on the
    /// carried plan (see [`LookAhead::carry_acceleration`]).
    pub(crate) entry_carry: (f64, f64),
    /// That state is one the plan has to continue from (see
    /// [`DetailedPlan::committed`]).
    pub(crate) entry_committed: bool,
}

/// The look-ahead result plus the per-segment facts the interpolator needs
/// on top of the public [`PlannedSegment`]s (all vectors are parallel).
pub(crate) struct DetailedPlan {
    pub(crate) segs: Vec<PlannedSegment>,
    /// Velocity ceiling per segment: [`LookAhead::velocity_limit`], and the
    /// junction limits inside the segment's span.
    pub(crate) v_limit: Vec<f64>,
    /// `blend[i]`: the junction after segment `i` does not constrain the
    /// velocity, so one jerk-limited profile may run straight through it.
    pub(crate) blend: Vec<bool>,
    /// The junction velocity limit after each segment as the corner rules
    /// give it, before any reachability pass (0 where the path stops).
    pub(crate) v_junction: Vec<f64>,
    /// `committed[i]`: the carried state at the end of segment `i` lies on
    /// a braking ramp that runs on through the junction, so the plain plan
    /// of the next span may be out of reach from it and the carried plan
    /// has to be followed there.
    pub(crate) committed: Vec<bool>,
    /// For a span's last segment: the highest velocity at which the span
    /// may be left with zero acceleration — from which the path can still
    /// slow down for everything that lies ahead. The planned exit velocity
    /// may be lower (what can be reached from the start of the path).
    pub(crate) v_safe: Vec<f64>,
}

/// The machine's limits on one move (before the programmed feed and `E`
/// words are applied).
struct MoveLimits {
    velocity: f64,
    acceleration: f64,
    jerk: f64,
}

impl LookAhead {
    /// Plan the whole queue. `aux_max_velocity` are the auxiliary axes'
    /// velocity limits in decoder order (used to cap path velocity so a
    /// synchronized aux axis never exceeds its own limit). See
    /// [`plan_with_axis_limits`](Self::plan_with_axis_limits) for the full
    /// set of per-axis limits.
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
    ///
    /// Any element between two moves that makes the path stand still (a
    /// dwell, or an event that is not a
    /// [fly-by event](crate::geometry::SyncEvent::fly_by)) ends a span with
    /// an exact stop.
    pub fn plan(
        &self,
        queue: &GeoQueue,
        limits: &PathLimits,
        aux_max_velocity: &[f64],
    ) -> Vec<PlannedSegment> {
        self.plan_with_axis_limits(
            queue,
            limits,
            &AxisLimits::from_aux_velocity(aux_max_velocity),
        )
    }

    /// [`plan`](Self::plan) with per-axis limits on velocity, acceleration,
    /// jerk and the velocity step at junctions (see [`AxisLimits`]).
    ///
    /// A span takes the lowest limits of its moves; moves whose limits
    /// differ by more than a tenth are kept in separate spans, so a long
    /// move is never held back much by a short neighbour that runs along a
    /// weaker axis.
    pub fn plan_with_axis_limits(
        &self,
        queue: &GeoQueue,
        limits: &PathLimits,
        axis_limits: &AxisLimits,
    ) -> Vec<PlannedSegment> {
        self.plan_detailed(queue, limits, axis_limits, &PlanContext::default())
            .segs
    }

    pub(crate) fn plan_detailed<Q: Elems + ?Sized>(
        &self,
        queue: &Q,
        limits: &PathLimits,
        axis_limits: &AxisLimits,
        ctx: &PlanContext,
    ) -> DetailedPlan {
        let mut segs: Vec<PlannedSegment> = Vec::new();
        let mut v_limit: Vec<f64> = Vec::new();
        let mut moves: Vec<&GeoMove> = Vec::new();
        for qi in 0..queue.count() {
            if let Some(GeoElem::Move(mv)) = queue.elem(qi) {
                let machine = self.move_limits(mv, limits, axis_limits);
                let cap = |programmed: Option<f64>| {
                    programmed.map_or(machine.acceleration, |a| a.min(machine.acceleration))
                };
                segs.push(PlannedSegment {
                    queue_index: qi,
                    length: mv.param_length(),
                    v_cap: mv.feed.min(machine.velocity),
                    v_exit: 0.0,
                    v_junction: 0.0,
                    accel: cap(mv.accel),
                    decel: cap(mv.decel),
                    jerk: machine.jerk,
                    v_carry: 0.0,
                    a_carry: 0.0,
                });
                v_limit.push(machine.velocity);
                moves.push(mv);
            }
        }

        // Junction velocity limit between consecutive moves. An element in
        // between that makes the path stand still (a dwell, a synchronized
        // event that is not a fly-by event) forces an exact stop.
        let n = segs.len();
        let mut v_junction = vec![0.0; n];
        for i in 0..n.saturating_sub(1) {
            let mut between = segs[i].queue_index + 1..segs[i + 1].queue_index;
            if between.all(|q| !queue.elem(q).is_some_and(GeoElem::is_stop)) {
                v_junction[i] = self.junction_velocity(moves[i], moves[i + 1], limits, axis_limits);
            }
        }
        // A seam between separately appended pieces of the path lies between
        // segment i and the next one.
        let seam_after = |i: usize| {
            let (a, b) = (segs[i].queue_index, segs[i + 1].queue_index);
            ctx.seams.iter().any(|&s| a < s && s <= b)
        };

        // Spans. Moves with the same caps are joined into one span as long
        // as running the whole span at one cruise velocity — the lowest of
        // the junction limits inside it — costs none of its moves more than
        // SPAN_TOLERANCE of the velocity it could reach on its own, i.e.
        // from and to its own junction limits with zero acceleration there.
        // That is the case when the junctions do not bind at all, and for
        // short moves between similar junction limits (a finely segmented
        // curve), which could not accelerate between them anyway.
        //
        // One profile has one set of limits. Moves whose machine limits
        // (velocity ceiling, acceleration, deceleration, jerk) are all the
        // same join freely; moves programmed alike whose machine limits
        // differ by no more than SPAN_TOLERANCE (the path turns a little
        // relative to a limiting axis) join as well, and the span runs on
        // the lowest of them.
        let peak = |i: usize, cap: f64, accel: f64, decel: f64, jerk: f64| {
            let len = segs[i].length;
            let reach = |v: f64, a_max: f64| {
                if v >= cap || ramp_distance(v, cap, a_max, jerk) <= len {
                    cap
                } else {
                    reachable_velocity(v, len, a_max, jerk)
                }
            };
            let v_in = if i > 0 {
                v_junction[i - 1]
            } else {
                ctx.entry_junction
            };
            reach(v_in, accel).min(reach(v_junction[i], decel))
        };
        let own = |i: usize| [v_limit[i], segs[i].accel, segs[i].decel, segs[i].jerk];
        let mut blend = vec![false; n];
        // (first segment, last segment, velocity cap, [ceiling, accel,
        // decel, jerk], cruise limit of the junctions inside)
        let mut spans_found: Vec<(usize, usize, f64, [f64; 4], f64)> = Vec::new();
        let mut i = 0;
        while i < n {
            let first = i;
            let mut cap = segs[i].v_cap;
            let (mut lo, mut hi) = (own(i), own(i));
            let mut inner = f64::INFINITY;
            // Highest attainable velocity of the span's moves; only needed
            // (and only computed) once a junction inside the span binds.
            let mut peak_max: Option<f64> = None;
            while i + 1 < n && v_junction[i] > 0.0 && !seam_after(i) {
                let next = own(i + 1);
                let (mut lo_next, mut hi_next) = (lo, hi);
                for k in 0..4 {
                    lo_next[k] = lo_next[k].min(next[k]);
                    hi_next[k] = hi_next[k].max(next[k]);
                }
                let exact = lo_next == hi_next && segs[i + 1].v_cap == cap;
                let (a, b) = (moves[first], moves[i + 1]);
                let close = a.feed == b.feed
                    && a.accel == b.accel
                    && a.decel == b.decel
                    && (0..4).all(|k| lo_next[k] >= (1.0 - SPAN_TOLERANCE) * hi_next[k]);
                if !(exact || close) {
                    break;
                }
                let cap_next = if exact { cap } else { a.feed.min(lo_next[0]) };
                let inner_next = inner.min(v_junction[i]);
                if inner_next < cap_next {
                    let [_, accel, decel, jerk] = lo_next;
                    let so_far = peak_max.unwrap_or_else(|| {
                        (first..=i)
                            .map(|k| peak(k, cap_next, accel, decel, jerk))
                            .fold(0.0, f64::max)
                    });
                    let peak_next = so_far.max(peak(i + 1, cap_next, accel, decel, jerk));
                    peak_max = Some(peak_next);
                    if inner_next < (1.0 - SPAN_TOLERANCE) * peak_next {
                        break;
                    }
                }
                blend[i] = true;
                (inner, lo, hi, cap) = (inner_next, lo_next, hi_next, cap_next);
                i += 1;
            }
            spans_found.push((first, i, cap, lo, inner));
            i += 1;
        }
        drop(moves);
        for (first, last, cap, [ceiling, accel, decel, jerk], inner) in spans_found {
            for k in first..=last {
                segs[k].v_cap = cap.min(inner);
                v_limit[k] = ceiling.min(inner);
                segs[k].accel = accel;
                segs[k].decel = decel;
                segs[k].jerk = jerk;
            }
        }
        for i in 0..n.saturating_sub(1) {
            segs[i].v_exit = v_junction[i].min(segs[i].v_cap).min(segs[i + 1].v_cap);
            segs[i].v_junction = if blend[i] {
                v_junction[i].min(v_limit[i])
            } else {
                segs[i].v_exit
            };
        }

        // Reachability, per span: a span is traversed by one profile that
        // starts and ends with zero acceleration, so what must fit is the
        // jerk-limited ramp between its entry and exit velocities. The
        // margin derates both acceleration and jerk, leaving real slack on
        // short spans (where the ramp never reaches the acceleration limit).
        // The planned ramps must stay within what the interpolator can do,
        // and that with a hair to spare (`PLAN_SLACK`) even at a margin of 1.
        // Every segment of a span has the same limits: the span's `accel`
        // rules its ramp up, its `decel` the ramp down.
        let margin = self.accel_margin.min(1.0);
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
            let j_eff = segs[end].jerk * margin;
            let fit = len * (1.0 - PLAN_SLACK);
            if v_in > v_out && ramp_distance(v_out, v_in, d_eff, j_eff) > fit {
                segs[prev_end].v_exit = v_in.min(reachable_velocity(v_out, fit, d_eff, j_eff));
            }
        }

        let v_safe: Vec<f64> = segs.iter().map(|seg| seg.v_exit).collect();

        // Forward pass: and the exit velocity must be reachable *up* from
        // the entry velocity (a program starts at rest).
        let mut v_in = ctx.entry_velocity;
        for &(first, end, len) in &spans {
            let v_out = segs[end].v_exit;
            let a_eff = segs[end].accel * margin;
            let d_eff = segs[end].decel * margin;
            let j_eff = segs[end].jerk * margin;
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
        for seg in &mut segs {
            (seg.v_carry, seg.a_carry) = (seg.v_exit, 0.0);
        }
        let mut committed = vec![false; n];
        if self.carry_acceleration && spans.len() > 1 {
            carry_plan(&mut segs, &spans, &v_safe, margin, ctx, &mut committed);
        }

        DetailedPlan {
            segs,
            v_limit,
            blend,
            v_junction,
            committed,
            v_safe,
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
        self.move_limits(mv, limits, &AxisLimits::from_aux_velocity(aux_max_velocity))
            .velocity
    }

    /// Velocity ceiling, acceleration and jerk limits of a move: the path
    /// limits, tightened by every axis that takes part in the move.
    fn move_limits(
        &self,
        mv: &GeoMove,
        limits: &PathLimits,
        axis_limits: &AxisLimits,
    ) -> MoveLimits {
        let mut out = MoveLimits {
            velocity: limits.max_velocity,
            acceleration: limits.max_acceleration,
            jerk: limits.max_jerk,
        };
        // Curvature cap: v ≤ sqrt(a_n / κ) at the segment's tightest point.
        let len = mv.geo.length();
        let kappa = mv.geo.max_curvature();
        if kappa > 0.0 {
            out.velocity = out
                .velocity
                .min((limits.max_normal_acceleration / kappa).sqrt());
        }
        // Path axes: an axis sees the share |tᵢ| of the tangential motion
        // and |nᵢ| of the centripetal acceleration v²·κ.
        if len > 1e-12 && axis_limits.path.iter().any(AxisLimit::limits_motion) {
            let (tangent, normal) = mv.geo.axis_extents();
            for (k, axis) in axis_limits.path.iter().enumerate() {
                if tangent[k] > 0.0 {
                    out.velocity = out.velocity.min(axis.max_velocity / tangent[k]);
                    out.acceleration = out.acceleration.min(axis.max_acceleration / tangent[k]);
                    out.jerk = out.jerk.min(axis.max_jerk / tangent[k]);
                }
                if kappa > 0.0 && normal[k] > 0.0 {
                    out.velocity = out
                        .velocity
                        .min((axis.max_acceleration / (normal[k] * kappa)).sqrt());
                }
            }
        }
        // Aux axes: an aux axis travelling Δ over parameter length L moves at
        // v_aux = v · Δ/L, so v ≤ v_aux_max · L/Δ; the same holds for its
        // acceleration and jerk.
        let param_len = if len > 1e-12 { len } else { mv.param_length() };
        if param_len > 0.0 {
            for (i, (a0, a1)) in mv.aux_start.iter().zip(mv.aux_end.iter()).enumerate() {
                let delta = (a1 - a0).abs();
                if delta > 0.0 {
                    if let Some(axis) = axis_limits.aux.get(i) {
                        out.velocity = out.velocity.min(axis.max_velocity * param_len / delta);
                        out.acceleration = out
                            .acceleration
                            .min(axis.max_acceleration * param_len / delta);
                        out.jerk = out.jerk.min(axis.max_jerk * param_len / delta);
                    }
                }
            }
        }
        out
    }

    /// Grbl-style junction-deviation corner velocity between two moves,
    /// held within the axes' velocity-jump limits.
    ///
    /// The corner is evaluated in the *generalized* tangent space (path
    /// tangent plus the auxiliary axes' rates dA/ds), so a change of an
    /// auxiliary axis' rate at an otherwise straight junction slows the
    /// junction down just like a geometric corner does.
    fn junction_velocity(
        &self,
        a: &GeoMove,
        b: &GeoMove,
        limits: &PathLimits,
        axis_limits: &AxisLimits,
    ) -> f64 {
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
        // Velocity-jump limits: an axis' velocity steps by v·|Δtᵢ| (path
        // axes) or v·|Δrᵢ| (auxiliary axes) at the junction.
        let mut v_jump = f64::INFINITY;
        for (k, axis) in axis_limits.path.iter().enumerate() {
            let step = (t_in[k] - t_out[k]).abs();
            if step > 1e-12 {
                v_jump = v_jump.min(axis.max_velocity_jump / step);
            }
        }
        for i in 0..a.aux_start.len().max(b.aux_start.len()) {
            let r_a = aux_rate(a, i, len_a);
            let r_b = aux_rate(b, i, len_b);
            dot_ab += r_a * r_b;
            n_a += r_a * r_a;
            n_b += r_b * r_b;
            let step = (r_b - r_a).abs();
            if step > 1e-12 {
                if let Some(axis) = axis_limits.aux.get(i) {
                    v_jump = v_jump.min(axis.max_velocity_jump / step);
                }
            }
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
        // (an infinite deviation leaves the corner to the other rules)
        let mut v_sqr = if self.junction_deviation == f64::INFINITY {
            f64::INFINITY
        } else {
            a_n * self.junction_deviation * sin_half / (1.0 - sin_half)
        };
        // Finely segmented curve: the junction samples a curve of radius
        // R ≈ (L₁ + L₂) / (2φ); keep the centripetal budget on it.
        let phi = cos_phi.acos();
        v_sqr = v_sqr.min(a_n * (len_a * n_a + len_b * n_b) / (2.0 * phi));
        // Generalized speed → path speed.
        (v_sqr.sqrt() / n_a.max(n_b)).min(v_jump)
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

/// What the carried plan needs to know of a span (acceleration and jerk
/// limits with the margin applied).
struct Span {
    /// Plan index of the span's last segment.
    end: usize,
    /// The length the ramps may use.
    fit: f64,
    cap: f64,
    accel: f64,
    decel: f64,
    jerk: f64,
    /// The plain plan's velocity at the end of the span,
    v_exit: f64,
    /// the junction limit there,
    junction: f64,
    /// and the highest velocity that is safe there with zero acceleration.
    safe: f64,
}

/// The state `state(share)` gives for the highest share of the
/// acceleration limit, among those tried, at which it gives one: the limit
/// itself, or the best found by halving and then closing in again. (A
/// steady acceleration below the limit is what a ramp along a row of
/// falling or rising velocity caps takes.)
fn highest_share(mut state: impl FnMut(f64) -> Option<(f64, f64)>) -> Option<(f64, f64)> {
    if let Some(found) = state(1.0) {
        return Some(found);
    }
    let mut lo = 0.5;
    let mut found = loop {
        if let Some(found) = state(lo) {
            break found;
        }
        lo *= 0.5;
        if lo < 0.1 {
            return None;
        }
    };
    let mut hi = 2.0 * lo;
    for _ in 0..4 {
        let mid = 0.5 * (lo + hi);
        match state(mid) {
            Some(better) => (found, lo) = (better, mid),
            None => hi = mid,
        }
    }
    Some(found)
}

/// Whether the path can get out of the accelerating state `(v, a)` at the
/// end of span `k`: by bringing the acceleration to zero at once — over as
/// many spans as that takes, within their velocity caps and junction
/// limits — and leaving the span in which it gets there at a velocity that
/// is safe there. (The interpolator takes this way when it has no faster
/// one, and a halt stays below it.)
fn way_out(facts: &[Span], k: usize, mut v: f64, mut a: f64) -> bool {
    /// Number of spans the way out may take.
    const SPANS: usize = 32;
    for i in k + 1..facts.len().min(k + 1 + SPANS) {
        let (prev, span) = (&facts[i - 1], &facts[i]);
        // The junction the state crosses: it has to be sound under the
        // limits on both sides (a halt may catch the path just before it
        // or just after).
        let jerk = prev.jerk.min(span.jerk);
        if a > prev.accel.min(span.accel) * (1.0 + 1e-12)
            || v > prev.junction
            || v + a * a / (2.0 * jerk) > prev.cap.min(span.cap)
        {
            return false;
        }
        let v_nat = v + a * a / (2.0 * span.jerk);
        let target = v_nat.min(span.safe);
        if settle_distance(v, a, target, span.accel, span.decel, span.jerk) <= span.fit {
            return true;
        }
        if span.v_exit <= 0.0 {
            return false; // the path stops at the end of this span
        }
        (v, a) = bang(v, a, span.fit, 0.0, span.jerk);
        if a <= 0.0 {
            return false; // zero acceleration, but too fast
        }
    }
    false
}

/// The carried plan: boundary states with a non-zero acceleration between
/// spans, where they are faster than the plain plan's (see
/// [`LookAhead::carry_acceleration`]). Writes
/// [`PlannedSegment::v_carry`]/[`a_carry`](PlannedSegment::a_carry) of every
/// span's last segment.
///
/// Two kinds of carried states exist, and both keep the plain plan's
/// guarantee that the path can always stop where it has to:
///
/// * *Braking* states are built backward from the place the braking is
///   for: the state at a span's start is the one from which a ramp at a
///   steady deceleration arrives exactly in the carried state at its end.
///   Such a chain ends in a state of the plain plan, and each of its links
///   exists by construction — so once on it, the path follows it.
/// * *Accelerating* states are built forward from the state the span is
///   entered with, and only kept if there is a [`way_out`] of them to a
///   state with zero acceleration at a velocity that is safe where it is
///   reached (`DetailedPlan::v_safe`).
///
/// Whether a carried state can be reached from where the path actually is,
/// is for the interpolator to find out; the plain state is its fallback.
fn carry_plan(
    segs: &mut [PlannedSegment],
    spans: &[(usize, usize, f64)],
    v_safe: &[f64],
    margin: f64,
    ctx: &PlanContext,
    committed: &mut [bool],
) {
    // The carried states are met by profiles the trajectory generator has
    // to find, and it cannot be relied on for one that uses a limit to the
    // full all the way: keep clear of the limits whatever the margin.
    let margin = margin.min(CARRY_MARGIN);
    let facts: Vec<Span> = spans
        .iter()
        .map(|&(_, end, len)| Span {
            end,
            fit: len * (1.0 - PLAN_SLACK),
            cap: segs[end].v_cap,
            accel: segs[end].accel * margin,
            decel: segs[end].decel * margin,
            jerk: segs[end].jerk * margin,
            v_exit: segs[end].v_exit,
            junction: segs[end].v_junction,
            safe: v_safe[end],
        })
        .collect();
    let m = facts.len();
    // A carried state must beat the plain one by more than rounding.
    let better = |v: f64, plain: f64| v > plain * (1.0 + 1e-9) + 1e-12;

    // Backward: braking states (velocity, deceleration ≥ 0, carried).
    let mut chain = vec![(0.0, 0.0, false); m];
    chain[m - 1] = (facts[m - 1].v_exit, 0.0, false);
    for k in (0..m - 1).rev() {
        let (here, next) = (&facts[k], &facts[k + 1]);
        chain[k] = (here.v_exit, 0.0, false);
        if here.v_exit <= 0.0 {
            continue; // a planned stop
        }
        let (v1, a1, _) = chain[k + 1];
        let level = here.decel.min(next.decel);
        // A state at a junction has to be sound under the jerk limits on
        // both of its sides: a halt may catch the path just before it or
        // just after.
        let jerk = here.jerk.min(next.jerk);
        // Arriving at the steady deceleration that leads on to the next
        // state. Before the junction the velocity was higher by at least
        // what it takes to build that deceleration up; and the deceleration
        // must not be more than can be taken back before standstill.
        let mut found = highest_share(|share| {
            let (v, a) = bang(v1, a1, next.fit, level * share, next.jerk);
            (a > 1e-9 * level
                && a <= here.decel * (1.0 + 1e-12)
                && v <= here.junction
                && v + a * a / (2.0 * jerk) <= here.cap
                && a * a <= 2.0 * jerk * v
                && better(v, here.v_exit))
            .then_some((v, a))
        });
        if found.is_none() {
            // Arriving with zero acceleration, as fast as the junction and
            // the room for the ramp to the next state allow.
            let v_nat = v1 + a1 * a1 / (2.0 * next.jerk);
            let fits =
                |v: f64| settle_distance(v1, a1, v, next.decel, next.decel, next.jerk) <= next.fit;
            if v_nat <= here.junction && fits(v_nat) {
                let v = if fits(here.junction) {
                    here.junction
                } else {
                    let (mut ok, mut bad) = (v_nat, here.junction);
                    for _ in 0..64 {
                        let mid = 0.5 * (ok + bad);
                        if fits(mid) {
                            ok = mid;
                        } else {
                            bad = mid;
                        }
                        if bad - ok <= 1e-13 * (1.0 + ok) {
                            break;
                        }
                    }
                    ok
                };
                if better(v, here.v_exit) {
                    found = Some((v, 0.0));
                }
            }
        }
        if let Some((v, a)) = found {
            chain[k] = (v, a, true);
        }
    }

    // Forward: choose one state per span end.
    let (mut pv, mut pa, mut forced) = (ctx.entry_carry.0, ctx.entry_carry.1, ctx.entry_committed);
    let mut plain_in = ctx.entry_velocity;
    for k in 0..m - 1 {
        let (here, next) = (&facts[k], &facts[k + 1]);
        let (ev, ea, carried) = chain[k];
        let mut pick = (here.v_exit, 0.0, false);
        if here.v_exit <= 0.0 {
            // a planned stop
        } else if forced {
            // On a braking ramp: its next state, whatever that is.
            pick = (ev, -ea, carried);
        } else {
            // (not forced: the entry state has no deceleration)
            let pa0 = pa.max(0.0);
            // The braking state, if the entry state can get there: by
            // settling at the velocity the braking starts from, and then
            // building the deceleration up.
            let v_top = ev + ea * ea / (2.0 * here.jerk);
            let t = ea / here.jerk;
            let d_top = v_top * t - here.jerk * t * t * t / 6.0;
            let reach = settle_distance(pv, pa0, v_top, here.accel, here.decel, here.jerk);
            if carried && reach + d_top <= here.fit {
                pick = (ev, -ea, true);
            } else {
                // Accelerating on through the junction — where the plain
                // plan does not slow down over this span, and there is a
                // way out of the acceleration ahead.
                let level = here.accel.min(next.accel);
                let ahead = (here.v_exit >= plain_in)
                    .then(|| {
                        highest_share(|share| {
                            let (v, a) = bang(pv, pa0, here.fit, level * share, here.jerk);
                            (a > 1e-9 * level && better(v, here.v_exit) && way_out(&facts, k, v, a))
                                .then_some((v, a))
                        })
                    })
                    .flatten();
                if let Some((v, a)) = ahead {
                    pick = (v, a, false);
                } else if carried
                    && ea > 0.0
                    && bang(pv, pa0, here.fit, here.accel, here.jerk).0 >= ev
                {
                    // A braking state that may be within reach after all.
                    pick = (ev, -ea, true);
                }
            }
        }
        (segs[here.end].v_carry, segs[here.end].a_carry) = (pick.0, pick.1);
        committed[here.end] = pick.2;
        (pv, pa, forced) = pick;
        plain_in = here.v_exit;
    }
}

/// Highest share of the acceleration and jerk limits the carried plan is
/// built with (see [`carry_plan`]).
pub(crate) const CARRY_MARGIN: f64 = 0.9;

/// The state `(velocity, acceleration)` after `dist` of travel from the
/// velocity `v0 ≥ 0` with the acceleration `a0 ≥ 0`, when the acceleration
/// moves to `level ≥ 0` at the rate `j_max` and then stays there.
pub(crate) fn bang(v0: f64, a0: f64, dist: f64, level: f64, j_max: f64) -> (f64, f64) {
    if dist.is_nan() || dist <= 0.0 {
        return (v0, a0);
    }
    let jerk = if level >= a0 { j_max } else { -j_max };
    let t1 = (level - a0).abs() / j_max;
    let travelled = |t: f64| ((jerk * t / 6.0 + 0.5 * a0) * t + v0) * t;
    let velocity = |t: f64| (0.5 * jerk * t + a0) * t + v0;
    if travelled(t1) >= dist {
        // Still on the way to the level. The distance grows with time:
        // Newton for the time it takes, kept inside a bracket of the root.
        let (mut lo, mut hi) = (0.0, t1);
        let mut t = if v0 > 0.0 { (dist / v0).min(t1) } else { t1 };
        for _ in 0..64 {
            let f = travelled(t) - dist;
            if f < 0.0 {
                lo = t;
            } else {
                hi = t;
            }
            let slope = velocity(t);
            let mut next = if slope > 0.0 { t - f / slope } else { lo };
            if !(next > lo && next < hi) {
                next = 0.5 * (lo + hi);
            }
            let step = (next - t).abs();
            t = next;
            if step <= 1e-15 * t1 || hi - lo <= 1e-15 * t1 {
                break;
            }
        }
        return (velocity(t), a0 + jerk * t);
    }
    let v1 = velocity(t1);
    let rest = dist - travelled(t1);
    ((v1 * v1 + 2.0 * level * rest).sqrt(), level)
}

/// Shortest distance in which the state `(v, a)` can reach velocity `vt`
/// with zero acceleration, under the jerk limit `j_max` and the limits
/// `accel` on speeding up and `decel` on slowing down.
pub(crate) fn settle_distance(v: f64, a: f64, vt: f64, accel: f64, decel: f64, j_max: f64) -> f64 {
    // The limit that rules a velocity change in the given direction
    let limit = |up: bool| if up { accel } else { decel };
    if a == 0.0 {
        return ramp_distance(v, vt, limit(vt >= v), j_max);
    }
    let t1 = a.abs() / j_max; // time to bring the acceleration to zero
    let dv1 = 0.5 * a * t1; // velocity gained meanwhile
    let v_nat = v + dv1;
    if (vt - v_nat) * a > 0.0 {
        // The target lies further along the way the acceleration already
        // points. (v, a) is a point on the standard ramp that left
        // v_virt = v − dv1 with zero acceleration t1 ago and has covered
        // x_virt since; the rest of that ramp is the fastest way to vt.
        let v_virt = v - dv1;
        let x_virt = v_virt * t1 + a * t1 * t1 / 6.0;
        ramp_distance(v_virt, vt, limit(a > 0.0).max(a.abs()), j_max) - x_virt
    } else {
        // First bring the acceleration to zero (arriving at v_nat), then
        // ramp the other way.
        let d1 = v * t1 + a * t1 * t1 / 3.0;
        d1 + ramp_distance(v_nat, vt, limit(a < 0.0), j_max)
    }
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
