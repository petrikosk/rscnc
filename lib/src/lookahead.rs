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
    /// v² ≤ a_n · δ · sin(θ/2) / (1 − sin(θ/2)). Internal units (mm).
    pub junction_deviation: f64,
    /// Fraction of `max_acceleration` used in the velocity-reachability
    /// passes. < 1.0 leaves headroom for the jerk-limited profile, which
    /// needs more distance than the acceleration-only formula assumes.
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
    /// Cruise velocity cap: min(programmed feed, machine cap, curvature cap, aux caps).
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

impl LookAhead {
    /// Plan the whole queue. `aux_max_velocity` are the auxiliary axes'
    /// velocity limits in decoder order (used to cap path velocity so a
    /// synchronized aux axis never exceeds its own limit).
    pub fn plan(
        &self,
        queue: &GeoQueue,
        limits: &PathLimits,
        aux_max_velocity: &[f64],
    ) -> Vec<PlannedSegment> {
        let mut segs: Vec<PlannedSegment> = Vec::new();
        for (qi, elem) in queue.iter().enumerate() {
            if let GeoElem::Move(mv) = elem {
                let cap = |programmed: Option<f64>| {
                    programmed.map_or(limits.max_acceleration, |a| a.min(limits.max_acceleration))
                };
                segs.push(PlannedSegment {
                    queue_index: qi,
                    length: mv.param_length(),
                    v_cap: self.segment_cap(mv, limits, aux_max_velocity),
                    v_exit: 0.0,
                    accel: cap(mv.accel),
                    decel: cap(mv.decel),
                });
            }
        }

        // Junction velocity between consecutive moves. Any non-move element
        // in between (dwell, synchronized event) forces an exact stop.
        let n = segs.len();
        for i in 0..n {
            let is_last = i + 1 == n;
            let adjacent = !is_last && segs[i + 1].queue_index == segs[i].queue_index + 1;
            let v_junction = if is_last || !adjacent {
                0.0
            } else {
                let (a, b) = match (
                    queue.get(segs[i].queue_index),
                    queue.get(segs[i + 1].queue_index),
                ) {
                    (Some(GeoElem::Move(a)), Some(GeoElem::Move(b))) => (a, b),
                    _ => unreachable!("planned indices always point at moves"),
                };
                self.junction_velocity(a, b, limits)
                    .min(segs[i].v_cap)
                    .min(segs[i + 1].v_cap)
            };
            segs[i].v_exit = v_junction;
        }

        // Backward pass: an exit velocity must be reachable *down* from the
        // following segments' exits — braking over the next segment, so with
        // that segment's deceleration limit.
        for i in (0..n.saturating_sub(1)).rev() {
            if segs[i + 1].queue_index == segs[i].queue_index + 1 {
                let d_eff = segs[i + 1].decel * self.accel_margin;
                let reachable =
                    (segs[i + 1].v_exit.powi(2) + 2.0 * d_eff * segs[i + 1].length).sqrt();
                segs[i].v_exit = segs[i].v_exit.min(reachable);
            }
        }

        // Forward pass: and reachable *up* from the entry velocity
        // (program starts at rest), with this segment's acceleration limit.
        let mut v_entry = 0.0;
        for seg in segs.iter_mut() {
            let a_eff = seg.accel * self.accel_margin;
            let reachable = (v_entry * v_entry + 2.0 * a_eff * seg.length).sqrt();
            seg.v_exit = seg.v_exit.min(reachable);
            v_entry = seg.v_exit;
        }

        segs
    }

    /// Cruise cap of a single segment.
    fn segment_cap(&self, mv: &GeoMove, limits: &PathLimits, aux_max_velocity: &[f64]) -> f64 {
        let mut cap = mv.feed.min(limits.max_velocity);
        // Curvature cap: v ≤ sqrt(a_n / κ). Curvature is monotone over the
        // (spiral-corrected) segment, so the endpoint maximum is the segment maximum.
        let len = mv.geo.length();
        let kappa = mv.geo.curvature_at(0.0).max(mv.geo.curvature_at(len));
        if kappa > 0.0 {
            cap = cap.min((limits.max_normal_acceleration / kappa).sqrt());
        }
        // Aux caps: an aux axis travelling Δ over parameter length L moves at
        // v_aux = v · Δ/L, so v ≤ v_aux_max · L/Δ.
        let param_len = mv.param_length();
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
    fn junction_velocity(&self, a: &GeoMove, b: &GeoMove, limits: &PathLimits) -> f64 {
        let t_out = a.geo.tangent_at(a.geo.length());
        let t_in = b.geo.tangent_at(0.0);
        // A pure aux move has no tangent: the path axes stand still anyway.
        if (t_out[0] == 0.0 && t_out[1] == 0.0 && t_out[2] == 0.0)
            || (t_in[0] == 0.0 && t_in[1] == 0.0 && t_in[2] == 0.0)
        {
            return 0.0;
        }
        let cos_theta = dot(&t_out, &t_in).clamp(-1.0, 1.0);
        if cos_theta > 1.0 - 1e-12 {
            return f64::INFINITY; // colinear: other caps rule
        }
        let sin_half = ((1.0 - cos_theta) / 2.0).sqrt();
        if sin_half >= 1.0 - 1e-12 {
            return 0.0; // full reversal
        }
        (limits.max_normal_acceleration * self.junction_deviation * sin_half / (1.0 - sin_half))
            .sqrt()
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
}
