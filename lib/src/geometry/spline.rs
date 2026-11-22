//! Polynomial spline segment: a Bézier curve of degree 1…5 in path space,
//! re-parameterized by arc length.
//!
//! The decoder builds cubic pieces from `G5` spline points, and
//! [`CornerSmoothing`](crate::preprocess::CornerSmoothing) builds quintic
//! corner blends that join their neighbours with continuous curvature.

// Index loops over the three coordinates read better than iterator chains here.
#![allow(clippy::needless_range_loop)]

use super::{dot, norm, sub, Segment};
use crate::types::PathPoint;

/// Highest supported Bézier degree.
const MAX_DEGREE: usize = 5;
/// Intervals of the arc-length table.
const TABLE: usize = 32;
/// Samples used for the curvature maximum and the bounding box.
const SAMPLES: usize = 128;

/// 5-point Gauss–Legendre nodes and weights on `[-1, 1]`.
const GL_X: [f64; 5] = [
    -0.906_179_845_938_664,
    -0.538_469_310_105_683_1,
    0.0,
    0.538_469_310_105_683_1,
    0.906_179_845_938_664,
];
const GL_W: [f64; 5] = [
    0.236_926_885_056_189_08,
    0.478_628_670_499_366_47,
    0.568_888_888_888_888_9,
    0.478_628_670_499_366_47,
    0.236_926_885_056_189_08,
];

/// A Bézier curve of degree 1…5, parameterized by arc length.
///
/// Construction integrates the curve once (arc-length table, curvature
/// maximum); evaluation inverts the table with a few Newton steps, so
/// [`point_at`](Segment::point_at) is exact to rounding and allocation-free.
#[derive(Debug, Clone, PartialEq)]
pub struct SplineSeg {
    ctrl: [PathPoint; MAX_DEGREE + 1],
    degree: usize,
    /// Arc length from the start to parameter `i / TABLE`.
    table: [f64; TABLE + 1],
    kappa_max: f64,
}

impl SplineSeg {
    /// A Bézier curve through the given control points (2 to 6 of them: the
    /// degree is one less).
    ///
    /// Refused: non-finite coordinates, a curve without length, and a curve
    /// that stands still at one of its ends or anywhere in between (a zero
    /// derivative: the tangent would be undefined there).
    pub fn bezier(points: &[PathPoint]) -> Result<SplineSeg, String> {
        if !(2..=MAX_DEGREE + 1).contains(&points.len()) {
            return Err(format!(
                "a spline segment takes 2 to {} control points (got {})",
                MAX_DEGREE + 1,
                points.len()
            ));
        }
        if points.iter().flatten().any(|c| !c.is_finite()) {
            return Err("spline control point is not finite".into());
        }
        let mut ctrl = [[0.0; 3]; MAX_DEGREE + 1];
        ctrl[..points.len()].copy_from_slice(points);
        let mut seg = SplineSeg {
            ctrl,
            degree: points.len() - 1,
            table: [0.0; TABLE + 1],
            kappa_max: 0.0,
        };
        let h = 1.0 / TABLE as f64;
        for i in 0..TABLE {
            let u0 = i as f64 * h;
            seg.table[i + 1] = seg.table[i] + seg.arc(u0, u0 + h);
        }
        let length = seg.table[TABLE];
        if !length.is_finite() || length <= 0.0 {
            return Err("spline segment has no length".into());
        }
        // The curve must keep moving: speed is measured against the mean
        // speed `length` (per unit of the parameter).
        let mut slowest = f64::INFINITY;
        let mut kappa_max: f64 = 0.0;
        let mut at = 0;
        for i in 0..=SAMPLES {
            let u = i as f64 / SAMPLES as f64;
            slowest = slowest.min(norm(&seg.d1(u)));
            let k = seg.curvature_u(u);
            if k > kappa_max {
                (kappa_max, at) = (k, i);
            }
        }
        if slowest <= 1e-9 * length {
            return Err("spline segment is degenerate (it stands still at a point)".into());
        }
        // A curve that turns straight back on itself (control points on a
        // line, out and back) stands still at one point *between* the
        // samples. Its direction flips there: look for the slowest point
        // wherever the derivative turns by more than a right angle from one
        // sample to the next.
        for i in 0..SAMPLES {
            let (mut lo, mut hi) = (i as f64 / SAMPLES as f64, (i + 1) as f64 / SAMPLES as f64);
            if dot(&seg.d1(lo), &seg.d1(hi)) > 0.0 {
                continue;
            }
            const GOLD: f64 = 0.618_033_988_749_894_9;
            let mut slowest = f64::INFINITY;
            for _ in 0..60 {
                let (a, b) = (hi - GOLD * (hi - lo), lo + GOLD * (hi - lo));
                let (sa, sb) = (norm(&seg.d1(a)), norm(&seg.d1(b)));
                slowest = slowest.min(sa).min(sb);
                if sa < sb {
                    hi = b;
                } else {
                    lo = a;
                }
            }
            if slowest <= 1e-6 * length {
                return Err("spline segment turns back on itself (a cusp)".into());
            }
        }
        // Refine the curvature maximum around the best sample.
        let (mut lo, mut hi) = (
            at.saturating_sub(1) as f64 / SAMPLES as f64,
            (at + 1).min(SAMPLES) as f64 / SAMPLES as f64,
        );
        const GOLD: f64 = 0.618_033_988_749_894_9;
        for _ in 0..40 {
            let (a, b) = (hi - GOLD * (hi - lo), lo + GOLD * (hi - lo));
            let (ka, kb) = (seg.curvature_u(a), seg.curvature_u(b));
            kappa_max = kappa_max.max(ka).max(kb);
            if ka > kb {
                hi = b;
            } else {
                lo = a;
            }
        }
        if !kappa_max.is_finite() {
            return Err("spline segment has unbounded curvature".into());
        }
        seg.kappa_max = kappa_max;
        Ok(seg)
    }

    /// The cubic piece from `p0` to `p1` with the end derivatives `m0` and
    /// `m1` (per unit of the parameter, as in a Hermite / Catmull-Rom
    /// spline).
    pub fn hermite(
        p0: PathPoint,
        m0: PathPoint,
        p1: PathPoint,
        m1: PathPoint,
    ) -> Result<SplineSeg, String> {
        let mut c1 = p0;
        let mut c2 = p1;
        for i in 0..3 {
            c1[i] += m0[i] / 3.0;
            c2[i] -= m1[i] / 3.0;
        }
        Self::bezier(&[p0, c1, c2, p1])
    }

    /// The control points.
    pub fn control_points(&self) -> &[PathPoint] {
        &self.ctrl[..=self.degree]
    }

    /// Polynomial degree (1…5).
    pub fn degree(&self) -> usize {
        self.degree
    }

    /// The highest curvature along the segment.
    pub fn max_curvature(&self) -> f64 {
        self.kappa_max
    }

    /// The same curve traversed from its end to its start.
    pub fn reversed(&self) -> SplineSeg {
        let mut points = [[0.0; 3]; MAX_DEGREE + 1];
        let n = self.degree + 1;
        for (i, p) in self.control_points().iter().enumerate() {
            points[n - 1 - i] = *p;
        }
        Self::bezier(&points[..n]).expect("a valid spline stays valid when reversed")
    }

    /// Split the curve at the curve parameter `u ∈ (0, 1)` (de Casteljau);
    /// the two pieces trace exactly the original curve.
    pub fn split(&self, u: f64) -> Result<(SplineSeg, SplineSeg), String> {
        let n = self.degree + 1;
        let mut work = self.ctrl;
        let mut left = [[0.0; 3]; MAX_DEGREE + 1];
        let mut right = [[0.0; 3]; MAX_DEGREE + 1];
        for level in 0..n {
            left[level] = work[0];
            right[n - 1 - level] = work[n - 1 - level];
            for i in 0..n - 1 - level {
                for k in 0..3 {
                    work[i][k] += (work[i + 1][k] - work[i][k]) * u;
                }
            }
        }
        Ok((Self::bezier(&left[..n])?, Self::bezier(&right[..n])?))
    }

    /// Split the curve into `pieces` parts of equal parameter range.
    pub fn subdivide(&self, pieces: usize) -> Result<Vec<SplineSeg>, String> {
        let mut out = Vec::with_capacity(pieces.max(1));
        let mut rest = self.clone();
        for k in (2..=pieces).rev() {
            let (head, tail) = rest.split(1.0 / k as f64)?;
            out.push(head);
            rest = tail;
        }
        out.push(rest);
        Ok(out)
    }

    /// Axis-aligned bounding box (min corner, max corner). Sampled, so a
    /// bulge between two samples can stick out by a small fraction of the
    /// sample spacing.
    pub fn bounding_box(&self) -> (PathPoint, PathPoint) {
        let mut lo = self.ctrl[0];
        let mut hi = self.ctrl[0];
        for i in 1..=SAMPLES {
            let p = self.eval(i as f64 / SAMPLES as f64);
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        (lo, hi)
    }

    /// Curve parameter `u ∈ [0, 1]` at arc length `s`.
    pub fn param_at(&self, s: f64) -> f64 {
        let length = self.table[TABLE];
        if s.is_nan() || s <= 0.0 {
            return 0.0;
        }
        if s >= length {
            return 1.0;
        }
        // Table interval containing s
        let (mut lo, mut hi) = (0, TABLE);
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if self.table[mid] <= s {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let h = 1.0 / TABLE as f64;
        let (u_lo, u_hi) = (lo as f64 * h, hi as f64 * h);
        let (s_lo, s_hi) = (self.table[lo], self.table[hi]);
        let mut u = u_lo + h * (s - s_lo) / (s_hi - s_lo);
        for _ in 0..8 {
            let speed = norm(&self.d1(u));
            if speed <= 0.0 {
                break;
            }
            let step = (s_lo + self.arc(u_lo, u) - s) / speed;
            u = (u - step).clamp(u_lo, u_hi);
            if step.abs() <= 1e-15 {
                break;
            }
        }
        u
    }

    /// Arc length between two curve parameters (one Gauss–Legendre panel).
    fn arc(&self, u0: f64, u1: f64) -> f64 {
        let (half, mid) = (0.5 * (u1 - u0), 0.5 * (u0 + u1));
        let mut sum = 0.0;
        for (x, w) in GL_X.iter().zip(GL_W.iter()) {
            sum += w * norm(&self.d1(mid + half * x));
        }
        sum * half
    }

    /// de Casteljau evaluation of a control polygon with `n` points.
    #[inline]
    fn casteljau(mut p: [PathPoint; MAX_DEGREE + 1], n: usize, u: f64) -> PathPoint {
        for level in 1..n {
            for i in 0..n - level {
                for k in 0..3 {
                    p[i][k] += (p[i + 1][k] - p[i][k]) * u;
                }
            }
        }
        p[0]
    }

    /// Control polygon of the derivative (`n` points → `n − 1`).
    #[inline]
    fn hodograph(p: &[PathPoint; MAX_DEGREE + 1], n: usize) -> [PathPoint; MAX_DEGREE + 1] {
        let mut d = [[0.0; 3]; MAX_DEGREE + 1];
        let scale = (n - 1) as f64;
        for i in 0..n - 1 {
            d[i] = sub(&p[i + 1], &p[i]).map(|c| c * scale);
        }
        d
    }

    #[inline]
    fn eval(&self, u: f64) -> PathPoint {
        Self::casteljau(self.ctrl, self.degree + 1, u)
    }

    /// First derivative with respect to the curve parameter.
    #[inline]
    fn d1(&self, u: f64) -> PathPoint {
        let n = self.degree + 1;
        Self::casteljau(Self::hodograph(&self.ctrl, n), n - 1, u)
    }

    /// Second derivative with respect to the curve parameter.
    #[inline]
    fn d2(&self, u: f64) -> PathPoint {
        let n = self.degree + 1;
        if n < 3 {
            return [0.0; 3];
        }
        let first = Self::hodograph(&self.ctrl, n);
        Self::casteljau(Self::hodograph(&first, n - 1), n - 2, u)
    }

    fn curvature_u(&self, u: f64) -> f64 {
        let (d, dd) = (self.d1(u), self.d2(u));
        let cross = [
            d[1] * dd[2] - d[2] * dd[1],
            d[2] * dd[0] - d[0] * dd[2],
            d[0] * dd[1] - d[1] * dd[0],
        ];
        let speed = norm(&d);
        if speed > 0.0 {
            norm(&cross) / (speed * speed * speed)
        } else {
            0.0
        }
    }

    fn unit_tangent(&self, u: f64) -> PathPoint {
        let d = self.d1(u);
        let n = norm(&d);
        if n > 0.0 {
            d.map(|c| c / n)
        } else {
            [0.0; 3]
        }
    }
}

impl Segment for SplineSeg {
    fn length(&self) -> f64 {
        self.table[TABLE]
    }

    fn point_at(&self, s: f64) -> PathPoint {
        self.eval(self.param_at(s))
    }

    fn tangent_at(&self, s: f64) -> PathPoint {
        self.unit_tangent(self.param_at(s))
    }

    fn curvature_at(&self, s: f64) -> f64 {
        self.curvature_u(self.param_at(s))
    }

    fn start(&self) -> PathPoint {
        self.ctrl[0]
    }

    fn end(&self) -> PathPoint {
        self.ctrl[self.degree]
    }

    #[inline]
    fn pose_at(&self, s: f64) -> (PathPoint, PathPoint) {
        let u = self.param_at(s);
        (self.eval(u), self.unit_tangent(u))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_curve_that_turns_straight_back_is_refused() {
        // Out to x = 10 and back to x = 5 along one line: the curve stands
        // still where it turns, between the samples.
        let back = SplineSeg::bezier(&[
            [0.0, 0.0, 0.0],
            [13.0, 0.0, 0.0],
            [11.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
        ]);
        assert!(back.is_err(), "{back:?}");
        // A tight but regular turn is fine.
        let turn = SplineSeg::bezier(&[
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [10.0, 0.5, 0.0],
            [0.0, 0.5, 0.0],
        ]);
        assert!(turn.is_ok(), "{turn:?}");
    }

    fn quarter() -> SplineSeg {
        // The usual cubic approximation of a unit quarter circle
        let k = 0.552_284_749_830_793_4;
        SplineSeg::bezier(&[
            [1.0, 0.0, 0.0],
            [1.0, k, 0.0],
            [k, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ])
        .unwrap()
    }

    #[test]
    fn line_as_a_spline() {
        let s = SplineSeg::bezier(&[[0.0, 0.0, 0.0], [3.0, 4.0, 0.0]]).unwrap();
        assert!((s.length() - 5.0).abs() < 1e-12);
        let p = s.point_at(2.5);
        assert!((p[0] - 1.5).abs() < 1e-12 && (p[1] - 2.0).abs() < 1e-12);
        assert_eq!(s.max_curvature(), 0.0);
        let t = s.tangent_at(1.0);
        assert!((t[0] - 0.6).abs() < 1e-12 && (t[1] - 0.8).abs() < 1e-12);
    }

    #[test]
    fn arc_length_parameter_is_uniform() {
        let s = quarter();
        // close to a quarter circle of radius 1
        assert!((s.length() - core::f64::consts::FRAC_PI_2).abs() < 1e-3);
        // equal steps in s are equal steps along the curve
        let n = 200;
        let step = s.length() / n as f64;
        let mut prev = s.point_at(0.0);
        for i in 1..=n {
            let p = s.point_at(i as f64 * step);
            let chord = norm(&sub(&p, &prev));
            assert!(
                (chord - step).abs() < 1e-5 * step + 1e-9 && chord <= step * (1.0 + 1e-12),
                "step {i}: chord {chord} vs {step}"
            );
            prev = p;
        }
        assert_eq!(s.point_at(0.0), s.start());
        assert_eq!(s.point_at(s.length()), s.end());
        assert_eq!(s.point_at(-1.0), s.start());
        assert_eq!(s.point_at(1e9), s.end());
    }

    #[test]
    fn tangent_is_the_derivative_of_the_position() {
        let s = quarter();
        let h = 1e-6;
        for i in 1..20 {
            let at = s.length() * i as f64 / 20.0;
            let (a, b) = (s.point_at(at - h), s.point_at(at + h));
            let t = s.tangent_at(at);
            for k in 0..3 {
                assert!(((b[k] - a[k]) / (2.0 * h) - t[k]).abs() < 1e-6);
            }
            assert!((norm(&t) - 1.0).abs() < 1e-12);
            // curvature of the near-circle is close to 1
            assert!((s.curvature_at(at) - 1.0).abs() < 0.01);
        }
        assert!(s.max_curvature() >= s.curvature_at(0.3) && s.max_curvature() < 1.01);
    }

    #[test]
    fn split_and_reverse_trace_the_same_curve() {
        let s = quarter();
        let (a, b) = s.split(0.3).unwrap();
        assert!((a.length() + b.length() - s.length()).abs() < 1e-12);
        assert_eq!(a.end(), b.start());
        let p = s.point_at(a.length() + 0.4 * b.length());
        let q = b.point_at(0.4 * b.length());
        assert!(norm(&sub(&p, &q)) < 1e-12);
        let r = s.reversed();
        let p = s.point_at(0.3);
        let q = r.point_at(s.length() - 0.3);
        assert!(norm(&sub(&p, &q)) < 1e-12);
        let pieces = s.subdivide(5).unwrap();
        assert_eq!(pieces.len(), 5);
        let total: f64 = pieces.iter().map(|p| p.length()).sum();
        assert!((total - s.length()).abs() < 1e-12);
        for w in pieces.windows(2) {
            assert_eq!(w[0].end(), w[1].start());
        }
    }

    #[test]
    fn degenerate_curves_are_refused() {
        let p = [1.0, 2.0, 3.0];
        assert!(SplineSeg::bezier(&[p]).is_err());
        assert!(SplineSeg::bezier(&[p, p]).is_err());
        assert!(SplineSeg::bezier(&[p, [f64::NAN, 0.0, 0.0]]).is_err());
        // zero end derivative
        assert!(SplineSeg::bezier(&[p, p, [2.0, 2.0, 3.0], [3.0, 3.0, 3.0]]).is_err());
        // a cusp in the middle (the curve doubles back on itself)
        assert!(
            SplineSeg::bezier(&[[0.0; 3], [1.0, 0.0, 0.0], [0.0; 3], [1.0, 0.0, 0.0]]).is_err()
        );
    }

    #[test]
    fn hermite_matches_its_end_conditions() {
        let s = SplineSeg::hermite([0.0; 3], [3.0, 0.0, 0.0], [2.0, 2.0, 0.0], [0.0, 3.0, 0.0])
            .unwrap();
        let (t0, t1) = (s.tangent_at(0.0), s.tangent_at(s.length()));
        assert!((t0[0] - 1.0).abs() < 1e-12 && t0[1].abs() < 1e-12);
        assert!(t1[0].abs() < 1e-12 && (t1[1] - 1.0).abs() < 1e-12);
        let (lo, hi) = s.bounding_box();
        assert!(lo[0] >= -1e-9 && hi[0] <= 2.0 + 1e-3 && hi[1] <= 2.0 + 1e-9);
    }
}
