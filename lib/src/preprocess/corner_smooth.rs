//! Corner smoothing with a tolerance (the `G64 P…` idea, CODESYS'
//! `SMC_SmoothPath`): replace the sharp corner between two consecutive linear
//! moves by a quintic spline that stays within a given distance of the
//! programmed corner and joins both lines with continuous curvature.
//!
//! Compared with [`CornerRounding`](super::CornerRounding), which inserts a
//! circular arc, the blend has no step in curvature where it meets the lines:
//! the centripetal acceleration builds up and decays smoothly instead of
//! switching on and off at the tangent points. It also works for corners in
//! any spatial orientation, not just inside one work plane.

use super::Preprocessor;
use crate::error::PreprocessError;
use crate::geometry::{
    dot, min_opt, norm, sub, GeoElem, GeoMove, Geometry, LineSeg, Segment, SplineSeg,
};
use crate::queue::GeoQueue;

const STAGE: &str = "CornerSmoothing";

/// Distance of the blend's apex from the corner, per unit of the trim
/// distance `d` and of `sin(φ/2)` (φ = deflection angle of the corner).
///
/// With the control points at `d`, `2d/3` and `d/3` from the corner `C` on
/// each leg, the curve's midpoint is
/// `B(½) = C + (1 + 5·⅔ + 10·⅓)/32 · d · (t_out − t_in)`, and
/// `|t_out − t_in| = 2·sin(φ/2)`. The midpoint is the point of the blend
/// nearest to the corner.
const APEX_FACTOR: f64 = 23.0 / 48.0;

/// Share of a leg the blend may take at most, so the blends of the corners
/// at the two ends of one line never overlap.
const MAX_LEG_SHARE: f64 = 0.45;

/// Corner smoothing preprocessor: blends the corners between consecutive
/// linear feed moves with quintic splines that deviate at most
/// [`tolerance`](Self::tolerance) from the programmed corner point.
///
/// The blend is a quintic Bézier curve whose control points lie on the two
/// legs, at `d`, `2d/3` and `d/3` from the corner: it leaves and rejoins the
/// lines with their direction and with zero curvature, and passes the corner
/// at a distance of `(23/48)·d·sin(φ/2)`, where φ is the corner's deflection
/// angle. `d` follows from the tolerance and is limited to 45 % of either
/// leg, so a short leg gets a tighter blend (and a smaller deviation) rather
/// than an overlap with the next corner.
///
/// Rapid moves, colinear junctions and exact reversals are left as they are.
/// Run this stage *after* [`ToolRadiusCorrection`](super::ToolRadiusCorrection):
/// the radius correction does not offset splines.
#[derive(Debug, Clone)]
pub struct CornerSmoothing {
    /// Largest allowed distance between the programmed corner and the
    /// smoothed path (internal units). `0` leaves the path unchanged.
    pub tolerance: f64,
}

impl CornerSmoothing {
    /// Corner smoothing with the given path tolerance (internal units).
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance }
    }

    /// Try to blend the corner between two lines. Returns the shortened
    /// lines and the blend between them.
    fn blend(&self, a: &GeoMove, b: &GeoMove) -> Option<(GeoMove, GeoMove, GeoMove)> {
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
        let t_in = la.tangent_at(len_a);
        let t_out = lb.tangent_at(0.0);
        let cos_phi = dot(&t_in, &t_out).clamp(-1.0, 1.0);
        if !((-1.0 + 1e-9)..=(1.0 - 1e-9)).contains(&cos_phi) {
            return None; // colinear or reversal: nothing to smooth
        }
        // sin(φ/2) of the deflection angle φ
        let sin_half = ((1.0 - cos_phi) / 2.0).sqrt();
        let d_max = MAX_LEG_SHARE * len_a.min(len_b);
        let d = (self.tolerance / (APEX_FACTOR * sin_half)).min(d_max);
        if d.is_nan() || d <= 1e-9 {
            return None;
        }

        let corner = la.p1;
        let on_leg = |t: &[f64; 3], dist: f64| {
            [
                corner[0] + t[0] * dist,
                corner[1] + t[1] * dist,
                corner[2] + t[2] * dist,
            ]
        };
        let tp1 = on_leg(&t_in, -d); // where the blend leaves the incoming line
        let tp2 = on_leg(&t_out, d); // where it joins the outgoing line
        let spline = SplineSeg::bezier(&[
            tp1,
            on_leg(&t_in, -d * 2.0 / 3.0),
            on_leg(&t_in, -d / 3.0),
            on_leg(&t_out, d / 3.0),
            on_leg(&t_out, d * 2.0 / 3.0),
            tp2,
        ])
        // (a corner so close to a reversal that the blend would stand still
        // at its apex is left sharp)
        .ok()?;

        // Split the aux interpolation at the ends of the blend
        let ua = (len_a - d) / len_a;
        let ub = d / len_b;
        let lerp = |s: &[f64], e: &[f64], u: f64| -> Vec<f64> {
            s.iter().zip(e).map(|(s, e)| s + (e - s) * u).collect()
        };
        let aux_tp1 = lerp(&a.aux_start, &a.aux_end, ua);
        let aux_tp2 = lerp(&b.aux_start, &b.aux_end, ub);

        let mut short_a = GeoMove {
            geo: Geometry::Line(LineSeg::new(la.p0, tp1)),
            aux_end: aux_tp1.clone(),
            ..a.clone()
        };
        let blend = GeoMove {
            geo: Geometry::Spline(Box::new(spline)),
            feed: a.feed.min(b.feed),
            is_rapid: false,
            aux_start: aux_tp1,
            aux_end: aux_tp2.clone(),
            trc: a.trc,
            // The blend replaces the corner both segments' dynamics met at.
            accel: min_opt(a.accel, b.accel),
            decel: min_opt(a.decel, b.decel),
            // Switch points stay on the elements they were programmed on.
            switch_points: Vec::new(),
            src_line: a.src_line,
            obj_no: a.obj_no,
        };
        let mut short_b = GeoMove {
            geo: Geometry::Line(LineSeg::new(tp2, lb.p1)),
            aux_start: aux_tp2,
            ..b.clone()
        };
        // The corner is cut off the end of a and the start of b: a's switch
        // points keep their distance to its start, b's move up with its start.
        short_a.shift_switch_points(0.0, len_a - d);
        short_b.shift_switch_points(d, len_b - d);
        Some((short_a, blend, short_b))
    }
}

impl Preprocessor for CornerSmoothing {
    fn name(&self) -> &'static str {
        STAGE
    }

    fn process(&mut self, queue: GeoQueue) -> Result<GeoQueue, PreprocessError> {
        if self.tolerance.is_nan() || self.tolerance < 0.0 {
            return Err(PreprocessError::new(
                STAGE,
                queue.front().map_or(0, GeoElem::src_line),
                format!(
                    "corner smoothing tolerance must be a number greater than or equal to zero (got {})",
                    self.tolerance
                ),
            ));
        }
        if self.tolerance == 0.0 {
            return Ok(queue);
        }
        let elems: Vec<GeoElem> = queue.into_iter().collect();
        let mut out: Vec<GeoElem> = Vec::with_capacity(elems.len());
        for elem in elems {
            let GeoElem::Move(b) = elem else {
                out.push(elem);
                continue;
            };
            // Corner = previous output element is a line move ending where b starts
            let candidate = matches!(out.last(), Some(GeoElem::Move(a))
                if matches!(a.geo, Geometry::Line(_))
                    && norm(&sub(&a.geo.end(), &b.geo.start())) < 1e-9);
            if candidate {
                let Some(GeoElem::Move(a)) = out.pop() else {
                    unreachable!()
                };
                if let Some((short_a, blend, short_b)) = self.blend(&a, &b) {
                    out.push(GeoElem::Move(short_a));
                    out.push(GeoElem::Move(blend));
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
    use crate::lookahead::LookAhead;
    use crate::types::{PathLimits, PathPoint};

    fn smooth(src: &str, tolerance: f64) -> Vec<GeoMove> {
        let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        moves(&CornerSmoothing::new(tolerance).process(q).unwrap())
    }

    fn moves(q: &GeoQueue) -> Vec<GeoMove> {
        q.iter()
            .filter_map(|e| match e {
                GeoElem::Move(m) => Some(m.clone()),
                _ => None,
            })
            .collect()
    }

    fn dist(a: &PathPoint, b: &PathPoint) -> f64 {
        norm(&sub(a, b))
    }

    /// Smallest distance from `p` to the blend (sampled finely).
    fn nearest(blend: &Geometry, p: &PathPoint) -> f64 {
        let len = blend.length();
        (0..=4000)
            .map(|i| dist(&blend.point_at(len * i as f64 / 4000.0), p))
            .fold(f64::INFINITY, f64::min)
    }

    /// Distance from `p` to the segment `a`–`b`.
    fn seg_dist(p: &PathPoint, a: &PathPoint, b: &PathPoint) -> f64 {
        let ab = sub(b, a);
        let t = (dot(&sub(p, a), &ab) / dot(&ab, &ab)).clamp(0.0, 1.0);
        dist(p, &[a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t])
    }

    #[test]
    fn deviation_matches_the_tolerance_at_every_angle() {
        // Corners from gentle to sharp, in the plane and in space
        for deflection in [5.0f64, 30.0, 60.0, 90.0, 120.0, 150.0, 175.0] {
            for z in [0.0, 7.0] {
                let phi = deflection.to_radians();
                let src = format!(
                    "G1 F20 X100\nX{:.9} Y{:.9} Z{z}\n",
                    100.0 + 100.0 * phi.cos(),
                    100.0 * phi.sin()
                );
                let tol = 0.2;
                let m = smooth(&src, tol);
                assert_eq!(m.len(), 3, "{deflection}°: not smoothed");
                let corner = [100.0, 0.0, 0.0];
                let Geometry::Spline(s) = &m[1].geo else {
                    panic!("{deflection}°: no spline");
                };
                // The apex is the nearest point to the corner and sits at
                // exactly the tolerance (the legs are long enough here).
                let apex = s.point_at(s.length() / 2.0);
                let dev = nearest(&m[1].geo, &corner);
                assert!(
                    (dev - tol).abs() < 1e-6 && (dist(&apex, &corner) - tol).abs() < 1e-9,
                    "{deflection}° z{z}: deviation {dev}, apex {}",
                    dist(&apex, &corner)
                );
                // (23/48)·d·sin(φ/2) with d the trim distance
                let d = dist(&m[0].geo.end(), &corner);
                let t_in = m[0].geo.tangent_at(0.0);
                let t_out = m[2].geo.tangent_at(0.0);
                let sin_half = ((1.0 - dot(&t_in, &t_out)) / 2.0).sqrt();
                assert!((APEX_FACTOR * d * sin_half - tol).abs() < 1e-9);
                assert!((dist(&m[2].geo.start(), &corner) - d).abs() < 1e-9);
                // No point of the blend is further from the programmed path
                // than the tolerance either.
                let (p0, p2) = (m[0].geo.start(), m[2].geo.end());
                let len = s.length();
                for i in 0..=400 {
                    let p = s.point_at(len * i as f64 / 400.0);
                    let off = seg_dist(&p, &p0, &corner).min(seg_dist(&p, &corner, &p2));
                    assert!(off <= tol + 1e-9, "{deflection}°: {off} off the path");
                }
            }
        }
    }

    #[test]
    fn blend_joins_the_lines_with_continuous_curvature() {
        let m = smooth("G1 F20 X10\nX14 Y9 Z-3\n", 0.1);
        assert_eq!(m.len(), 3);
        let (a, s, b) = (&m[0].geo, &m[1].geo, &m[2].geo);
        // position
        assert!(dist(&a.end(), &s.start()) < 1e-12);
        assert!(dist(&s.end(), &b.start()) < 1e-12);
        // direction
        assert!(dist(&a.tangent_at(a.length()), &s.tangent_at(0.0)) < 1e-9);
        assert!(dist(&s.tangent_at(s.length()), &b.tangent_at(0.0)) < 1e-9);
        // curvature: zero at both ends, like the lines', and bounded between
        assert!(s.curvature_at(0.0).abs() < 1e-9);
        assert!(s.curvature_at(s.length()).abs() < 1e-9);
        let peak = s.max_curvature();
        assert!(peak.is_finite() && peak > 0.0);
        // ... and the curvature changes smoothly in between: symmetric about
        // the apex, never above the reported maximum, no steps
        let n = 400;
        let ks: Vec<f64> = (0..=n)
            .map(|i| s.curvature_at(s.length() * i as f64 / n as f64))
            .collect();
        for i in 0..=n {
            assert!(ks[i] <= peak * (1.0 + 1e-9));
            assert!(
                (ks[i] - ks[n - i]).abs() < 1e-6 * peak,
                "not symmetric at {i}"
            );
        }
        for w in ks.windows(2) {
            assert!((w[1] - w[0]).abs() < 0.05 * peak, "curvature step: {w:?}");
        }
        assert!(ks.iter().cloned().fold(0.0, f64::max) > 0.999 * peak);
        // the untouched ends of the programmed path
        assert_eq!(a.start(), [0.0, 0.0, 0.0]);
        assert_eq!(b.end(), [14.0, 9.0, -3.0]);
    }

    #[test]
    fn blend_stays_inside_the_corner_triangle() {
        let m = smooth("G1 F20 X10\nX4 Y8\n", 0.3);
        let Geometry::Spline(s) = &m[1].geo else {
            panic!("no spline");
        };
        let (p0, c, p1) = (s.start(), [10.0, 0.0, 0.0], s.end());
        // barycentric coordinates in the triangle (p0, corner, p1)
        let (e0, e1) = (sub(&p0, &c), sub(&p1, &c));
        let det = e0[0] * e1[1] - e0[1] * e1[0];
        for i in 0..=200 {
            let p = sub(&s.point_at(s.length() * i as f64 / 200.0), &c);
            let u = (p[0] * e1[1] - p[1] * e1[0]) / det;
            let v = (e0[0] * p[1] - e0[1] * p[0]) / det;
            assert!(
                u >= -1e-9 && v >= -1e-9 && u + v <= 1.0 + 1e-9,
                "outside the triangle at sample {i}: u {u}, v {v}"
            );
        }
    }

    #[test]
    fn short_legs_tighten_the_blend() {
        // 1 mm legs: the blend may take 0.45 mm of each, however large the
        // tolerance, and the deviation shrinks accordingly.
        let m = smooth("G1 F20 X1\nY1\n", 5.0);
        assert_eq!(m.len(), 3);
        assert!((m[0].geo.length() - 0.55).abs() < 1e-9);
        let dev = nearest(&m[1].geo, &[1.0, 0.0, 0.0]);
        let expected = APEX_FACTOR * 0.45 * std::f64::consts::FRAC_1_SQRT_2;
        assert!((dev - expected).abs() < 1e-6, "{dev} vs {expected}");
        // A chain of corners: every line keeps a piece between its blends.
        let m = smooth("G1 F20 X1\nY1\nX2\nY2\nX3\n", 5.0);
        assert_eq!(m.len(), 5 + 4);
        for w in m.windows(2) {
            assert!(dist(&w[0].geo.end(), &w[1].geo.start()) < 1e-12);
        }
        for mv in &m {
            assert!(mv.geo.length() > 0.05, "vanishing element");
        }
    }

    #[test]
    fn untouched_cases() {
        let plain =
            |src: &str| moves(&Decoder::decode_source(DecoderConfig::default(), src).unwrap());
        // colinear, exact reversal, a rapid on one side, an arc on one side
        for src in [
            "G1 F20 X10\nX20\n",
            "G1 F20 X10\nX0\n",
            "G0 X10\nG1 F20 Y10\n",
            "G1 F20 X10\nG0 Y10\n",
            "G1 F20 X10\nG3 X20 Y10 I0 J10\n",
        ] {
            assert_eq!(smooth(src, 0.5), plain(src), "{src:?}");
        }
        // an event between the moves, and a zero tolerance
        let q = Decoder::decode_source(DecoderConfig::default(), "G1 F20 X10 M8\nY10\n").unwrap();
        assert_eq!(CornerSmoothing::new(0.5).process(q.clone()).unwrap(), q);
        let q = Decoder::decode_source(DecoderConfig::default(), "G1 F20 X10\nY10\n").unwrap();
        assert_eq!(CornerSmoothing::new(0.0).process(q.clone()).unwrap(), q);
        // a tolerance that is not a distance
        for bad in [f64::NAN, -0.1] {
            let err = CornerSmoothing::new(bad).process(q.clone()).unwrap_err();
            assert_eq!(err.stage, "CornerSmoothing");
        }
        // an unlimited one just uses what the legs allow
        let out = CornerSmoothing::new(f64::INFINITY).process(q).unwrap();
        assert_eq!(out.len(), 3);
        assert!(moves(&out).iter().all(|m| m.geo.length().is_finite()));
    }

    #[test]
    fn auxiliary_axes_and_switch_points_follow_the_cut() {
        let cfg = DecoderConfig {
            aux_letters: vec!['A'],
            start_aux: vec![0.0],
            ..Default::default()
        };
        // switch points near the start, in the corner zone and at the end of
        // both lines
        let src = "G1 F20 E300 X10 A10 H1 L1 H2 L9.9 H-1 O1\nE-200 F15 Y10 A30 H3 L0.05 H-3 L9\n";
        let q = Decoder::decode_source(cfg, src).unwrap();
        let out = CornerSmoothing::new(0.2).process(q).unwrap();
        let m = moves(&out);
        assert_eq!(m.len(), 3);
        let d = 10.0 - m[0].geo.length();
        assert!(d > 0.3 && d < 1.0, "trim distance {d}");
        // the auxiliary axis is continuous and split in proportion
        assert_eq!(m[0].aux_start, vec![0.0]);
        assert!((m[0].aux_end[0] - (10.0 - d)).abs() < 1e-9);
        assert_eq!(m[0].aux_end, m[1].aux_start);
        assert_eq!(m[1].aux_end, m[2].aux_start);
        assert!((m[2].aux_start[0] - (10.0 + 2.0 * d)).abs() < 1e-9);
        assert_eq!(m[2].aux_end, vec![30.0]);
        // the blend runs at the tighter of the two moves' dynamics
        assert_eq!(m[1].feed, 15.0);
        assert_eq!(m[1].accel, Some(300.0));
        assert_eq!(m[1].decel, Some(200.0));
        assert_eq!((m[1].src_line, m[1].obj_no), (m[0].src_line, m[0].obj_no));
        assert!(m[1].switch_points.is_empty());
        // switch points: all still there, in order, inside their elements
        let ids = |mv: &GeoMove| {
            mv.switch_points
                .iter()
                .map(|s| s.number)
                .collect::<Vec<_>>()
        };
        assert_eq!(ids(&m[0]), vec![1, 2, -1]);
        assert_eq!(ids(&m[2]), vec![3, -3]);
        for mv in [&m[0], &m[2]] {
            let len = mv.geo.length();
            for w in mv.switch_points.windows(2) {
                assert!(w[0].position <= w[1].position);
            }
            for sp in &mv.switch_points {
                assert!((0.0..=len).contains(&sp.position));
            }
        }
        // one clear of the corner keeps its place on the programmed path ...
        assert!((m[0].switch_points[0].position - 1.0).abs() < 1e-12);
        assert!((m[2].switch_points[1].position - (9.0 - d)).abs() < 1e-12);
        // ... one inside the cut-off piece lands on the new element boundary
        assert!((m[0].switch_points[1].position - (10.0 - d)).abs() < 1e-12);
        assert_eq!(m[2].switch_points[0].position, 0.0);
    }

    #[test]
    fn planner_runs_through_the_blend() {
        let limits = PathLimits::new(50.0, 500.0, 5000.0, 500.0);
        let la = LookAhead::default();
        let src = "G1 F50 X20\nY20\n";
        let q = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        let sharp = la.plan(&q, &limits, &[]);
        let tol = la.junction_deviation;
        let q = CornerSmoothing::new(tol).process(q).unwrap();
        let plan = la.plan(&q, &limits, &[]);
        assert_eq!(plan.len(), 3);
        // Both joints are passed in motion, and the blend's speed is what
        // its tightest point allows within the centripetal budget.
        let GeoElem::Move(blend) = q.get(1).unwrap() else {
            panic!("no blend");
        };
        let v_blend = (limits.max_normal_acceleration / blend.geo.max_curvature()).sqrt();
        assert!((plan[1].v_cap - v_blend).abs() < 1e-9 * v_blend);
        for seg in &plan[..2] {
            assert!(
                seg.v_exit.is_finite() && seg.v_exit > 0.0 && seg.v_exit <= v_blend * (1.0 + 1e-9),
                "joint velocity {}",
                seg.v_exit
            );
        }
        assert_eq!(plan[2].v_exit, 0.0);
        // The same deviation as the sharp corner's junction rule gives about
        // the same corner speed (within 10 %): the gain is the continuous
        // acceleration, not a faster corner.
        let v_sharp = sharp[0].v_exit;
        assert!(
            v_blend > 0.9 * v_sharp && v_blend < 1.1 * v_sharp,
            "blend {v_blend} vs sharp corner {v_sharp}"
        );
    }
}
