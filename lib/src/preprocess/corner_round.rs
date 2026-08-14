//! Corner rounding (the `SMC_RoundPath` analogue): replace the sharp corner
//! between two consecutive linear moves by a tangent arc, so the look-ahead
//! can carry velocity through the corner instead of slowing for it.

use super::Preprocessor;
use crate::error::PreprocessError;
use crate::geometry::{dot, norm, sub, ArcSeg, GeoElem, GeoMove, Geometry, LineSeg, Segment};
use crate::queue::GeoQueue;
use crate::types::WorkPlane;

/// Corner rounding preprocessor: fillets sharp corners between consecutive
/// linear moves with tangent arcs of (at most) the given radius.
#[derive(Debug, Clone)]
pub struct CornerRounding {
    /// Requested fillet radius; reduced when the adjoining segments are short.
    pub radius: f64,
}

impl CornerRounding {
    /// Corner rounding with the given fillet radius (internal units).
    pub fn new(radius: f64) -> Self {
        Self { radius }
    }

    /// Try to fillet the corner between two lines. Returns the shortened
    /// lines and the inserted arc.
    fn fillet(&self, a: &GeoMove, b: &GeoMove) -> Option<(GeoMove, GeoMove, GeoMove)> {
        let (Geometry::Line(la), Geometry::Line(lb)) = (&a.geo, &b.geo) else {
            return None;
        };
        if a.is_rapid || b.is_rapid {
            return None;
        }
        let len_a = la.length();
        let len_b = lb.length();
        if len_a <= 0.0 || len_b <= 0.0 {
            return None;
        }
        // Find a work plane both lines lie in (zero direction component along
        // the plane normal): our arcs are plane-aligned.
        let plane = [WorkPlane::Xy, WorkPlane::Zx, WorkPlane::Yz]
            .into_iter()
            .find(|p| {
                let (_, _, n) = p.axes();
                (la.p1[n] - la.p0[n]).abs() < 1e-12 && (lb.p1[n] - lb.p0[n]).abs() < 1e-12
            })?;
        let (i0, i1, _) = plane.axes();

        let t1 = la.tangent_at(len_a);
        let t2 = lb.tangent_at(0.0);
        let cos_theta = dot(&t1, &t2).clamp(-1.0, 1.0);
        if !((-1.0 + 1e-9)..=(1.0 - 1e-9)).contains(&cos_theta) {
            return None; // colinear or reversal: nothing to round
        }
        let theta = cos_theta.acos(); // turn angle
        let tan_half = (theta / 2.0).tan();
        // Trim distance for radius r is r·tan(θ/2); keep at most 45% of
        // either segment so the residual lines stay usable.
        let d_max = (0.45 * len_a).min(0.45 * len_b);
        let r = self.radius.min(d_max / tan_half);
        if r <= 1e-9 {
            return None;
        }
        let d = r * tan_half;

        let corner = la.p1;
        let mut tp1 = corner; // tangent point on incoming line
        let mut tp2 = corner; // tangent point on outgoing line
        for i in 0..3 {
            tp1[i] -= t1[i] * d;
            tp2[i] += t2[i] * d;
        }
        // Turn side: z-component of the in-plane cross product
        let cross = t1[i0] * t2[i1] - t1[i1] * t2[i0];
        let ccw = cross > 0.0;
        // Arc center: from tp1, perpendicular to t1, toward the turn side
        let mut center = tp1;
        let side = if ccw { 1.0 } else { -1.0 };
        center[i0] += -t1[i1] * side * r;
        center[i1] += t1[i0] * side * r;

        let arc = ArcSeg::from_center(plane, tp1, tp2, center, ccw, 1e-6 + r * 1e-9).ok()?;

        // Split the aux interpolation at the tangent points
        let ua = (len_a - d) / len_a;
        let ub = d / len_b;
        let lerp = |s: &[f64], e: &[f64], u: f64| -> Vec<f64> {
            s.iter().zip(e).map(|(s, e)| s + (e - s) * u).collect()
        };
        let aux_tp1 = lerp(&a.aux_start, &a.aux_end, ua);
        let aux_tp2 = lerp(&b.aux_start, &b.aux_end, ub);

        let short_a = GeoMove {
            geo: Geometry::Line(LineSeg::new(la.p0, tp1)),
            aux_end: aux_tp1.clone(),
            ..a.clone()
        };
        let arc_move = GeoMove {
            geo: Geometry::Arc(arc),
            feed: a.feed.min(b.feed),
            is_rapid: false,
            aux_start: aux_tp1,
            aux_end: aux_tp2.clone(),
            trc: a.trc,
            src_line: a.src_line,
            obj_no: a.obj_no,
        };
        let short_b = GeoMove {
            geo: Geometry::Line(LineSeg::new(tp2, lb.p1)),
            aux_start: aux_tp2,
            ..b.clone()
        };
        Some((short_a, arc_move, short_b))
    }
}

impl Preprocessor for CornerRounding {
    fn name(&self) -> &'static str {
        "CornerRounding"
    }

    fn process(&mut self, queue: GeoQueue) -> Result<GeoQueue, PreprocessError> {
        let elems: Vec<GeoElem> = queue.into_iter().collect();
        let mut out: Vec<GeoElem> = Vec::with_capacity(elems.len());
        for elem in elems {
            let GeoElem::Move(b) = elem else {
                out.push(elem);
                continue;
            };
            // Corner = previous output element is a line move ending where b starts
            let roundable = matches!(out.last(), Some(GeoElem::Move(a))
                if matches!(a.geo, Geometry::Line(_))
                    && norm(&sub(&a.geo.end(), &b.geo.start())) < 1e-9);
            if roundable {
                let Some(GeoElem::Move(a)) = out.pop() else {
                    unreachable!()
                };
                if let Some((short_a, arc, short_b)) = self.fillet(&a, &b) {
                    out.push(GeoElem::Move(short_a));
                    out.push(GeoElem::Move(arc));
                    out.push(GeoElem::Move(short_b));
                } else {
                    out.push(GeoElem::Move(a));
                    out.push(GeoElem::Move(b));
                }
            } else {
                out.push(GeoElem::Move(b));
            }
        }
        Ok(out.into_iter().collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::{Decoder, DecoderConfig};
    use crate::geometry::Segment;

    #[test]
    fn right_angle_fillet_is_tangent() {
        let q = Decoder::decode_source(DecoderConfig::default(), "G1 F600 X10\nY10\n").unwrap();
        let out = CornerRounding::new(2.0).process(q).unwrap();
        let moves: Vec<_> = out
            .iter()
            .filter_map(|e| match e {
                GeoElem::Move(m) => Some(m),
                _ => None,
            })
            .collect();
        assert_eq!(moves.len(), 3);
        // Tangent continuity at both junctions (G1 continuity of the fillet)
        let a = &moves[0];
        let arc = &moves[1];
        let b = &moves[2];
        let t_ae = a.geo.tangent_at(a.geo.length());
        let t_as = arc.geo.tangent_at(0.0);
        let t_arc_end = arc.geo.tangent_at(arc.geo.length());
        let t_bs = b.geo.tangent_at(0.0);
        for i in 0..3 {
            assert!((t_ae[i] - t_as[i]).abs() < 1e-9, "start tangent mismatch");
            assert!(
                (t_arc_end[i] - t_bs[i]).abs() < 1e-9,
                "end tangent mismatch"
            );
        }
        // Endpoints connect
        for i in 0..3 {
            assert!((a.geo.end()[i] - arc.geo.start()[i]).abs() < 1e-9);
            assert!((arc.geo.end()[i] - b.geo.start()[i]).abs() < 1e-9);
        }
        // 90° corner with radius 2: trim distance = 2·tan(45°) = 2
        assert!((a.geo.end()[0] - 8.0).abs() < 1e-9);
        assert!((b.geo.start()[1] - 2.0).abs() < 1e-9);
        // Arc length = r·θ = 2·(π/2)
        assert!((arc.geo.length() - core::f64::consts::PI).abs() < 1e-9);
    }

    #[test]
    fn short_segments_shrink_radius_and_events_block() {
        // 1mm legs: radius clamps to 0.45·1/tan(45°) = 0.45
        let q = Decoder::decode_source(DecoderConfig::default(), "G1 F600 X1\nY1\n").unwrap();
        let out = CornerRounding::new(100.0).process(q).unwrap();
        assert_eq!(out.len(), 3);
        // An event between moves prevents rounding
        let q = Decoder::decode_source(DecoderConfig::default(), "G1 F600 X10 M8\nY10\n").unwrap();
        let out = CornerRounding::new(2.0).process(q).unwrap();
        assert_eq!(out.len(), 3); // move, event, move - unchanged
    }
}
