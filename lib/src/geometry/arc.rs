//! Circular (and helical) arc segment in a work plane.
//!
//! The radius may differ slightly between start and end (linear "spiral
//! correction", as in LinuxCNC): CAM output often has a small center/radius
//! mismatch, and interpolating the radius keeps both endpoints exactly on
//! the programmed path so consecutive segments stay C0-continuous.

use super::Segment;
use crate::types::{PathPoint, WorkPlane};

/// A circular (or helical) arc in a work plane, with optional linear
/// radius interpolation between `r_start` and `r_end` (spiral correction).
#[derive(Debug, Clone, PartialEq)]
pub struct ArcSeg {
    /// Work plane the arc lies in.
    pub plane: WorkPlane,
    /// Full 3D center; the normal-axis component is the start point's normal coordinate.
    pub center: PathPoint,
    /// Radius at the start point.
    pub r_start: f64,
    /// Radius at the end point (differs from `r_start` only within the decoder's tolerance).
    pub r_end: f64,
    /// Start angle in the plane: `atan2(second - c, first - c)`.
    pub theta0: f64,
    /// Signed sweep angle; positive = counter-clockwise (G3) in the
    /// right-handed (first, second) plane basis. Never 0.
    pub sweep: f64,
    /// Normal-axis travel over the arc (helical G2/G3); 0 for planar arcs.
    pub helix_delta: f64,
}

impl ArcSeg {
    /// Construct from a known center (IJK format). `start`/`end` are full path
    /// points; the center's in-plane components are taken from `center`,
    /// its normal component from `start`. Returns an error message when the
    /// start/end radii differ by more than `radius_tolerance`.
    ///
    /// `full_circle`: when start and end coincide in the plane, program a
    /// full circle (DIN 66025 IJK semantics) instead of a zero-length arc.
    pub fn from_center(
        plane: WorkPlane,
        start: PathPoint,
        end: PathPoint,
        center: PathPoint,
        ccw: bool,
        radius_tolerance: f64,
    ) -> Result<ArcSeg, String> {
        let (i0, i1, i2) = plane.axes();
        let r_start = ((start[i0] - center[i0]).powi(2) + (start[i1] - center[i1]).powi(2)).sqrt();
        let r_end = ((end[i0] - center[i0]).powi(2) + (end[i1] - center[i1]).powi(2)).sqrt();
        if r_start <= 0.0 || r_end <= 0.0 {
            return Err("arc start or end coincides with its center".into());
        }
        if (r_start - r_end).abs() > radius_tolerance {
            return Err(format!(
                "arc radius mismatch: start radius {r_start:.6}, end radius {r_end:.6} (tolerance {radius_tolerance})"
            ));
        }
        let theta0 = (start[i1] - center[i1]).atan2(start[i0] - center[i0]);
        let theta1 = (end[i1] - center[i1]).atan2(end[i0] - center[i0]);
        let mut sweep = theta1 - theta0;
        const TAU: f64 = core::f64::consts::TAU;
        if ccw {
            while sweep <= 1e-12 {
                sweep += TAU;
            }
        } else {
            while sweep >= -1e-12 {
                sweep -= TAU;
            }
        }
        let mut c = center;
        c[i2] = start[i2];
        Ok(ArcSeg {
            plane,
            center: c,
            r_start,
            r_end,
            theta0,
            sweep,
            helix_delta: end[i2] - start[i2],
        })
    }

    /// Construct from the R word (radius format). Sign convention per DIN /
    /// LinuxCNC: `r > 0` selects the minor arc (≤ 180°), `r < 0` the major
    /// arc (> 180°). Full circles are invalid in radius format.
    pub fn from_radius(
        plane: WorkPlane,
        start: PathPoint,
        end: PathPoint,
        r: f64,
        ccw: bool,
    ) -> Result<ArcSeg, String> {
        let (i0, i1, _) = plane.axes();
        let dx = end[i0] - start[i0];
        let dy = end[i1] - start[i1];
        let chord = (dx * dx + dy * dy).sqrt();
        if chord <= 0.0 {
            return Err(
                "radius-format arc with coincident start and end (full circle requires IJK)".into(),
            );
        }
        let radius = r.abs();
        let half = chord / 2.0;
        if half > radius * (1.0 + 1e-9) {
            return Err(format!(
                "arc radius {radius:.6} is too small for chord length {chord:.6}"
            ));
        }
        // Distance from chord midpoint to center
        let h = (radius * radius - half * half).max(0.0).sqrt();
        // Unit vector perpendicular to the chord, to the left of travel
        let (px, py) = (-dy / chord, dx / chord);
        // CCW minor arc: center left of chord; CW minor arc: center right.
        // Negative R flips to the major arc.
        let mut side = if ccw { 1.0 } else { -1.0 };
        if r < 0.0 {
            side = -side;
        }
        let mut center = start;
        center[i0] = start[i0] + dx / 2.0 + side * h * px;
        center[i1] = start[i1] + dy / 2.0 + side * h * py;
        Self::from_center(plane, start, end, center, ccw, 1e-6 + radius * 1e-9)
    }

    /// The same arc traversed from its end to its start.
    pub fn reversed(&self) -> ArcSeg {
        let (_, _, i2) = self.plane.axes();
        let mut center = self.center;
        center[i2] += self.helix_delta;
        ArcSeg {
            plane: self.plane,
            center,
            r_start: self.r_end,
            r_end: self.r_start,
            theta0: self.theta0 + self.sweep,
            sweep: -self.sweep,
            helix_delta: -self.helix_delta,
        }
    }

    /// Signed helix pitch per radian of sweep.
    #[inline]
    fn pitch(&self) -> f64 {
        self.helix_delta / self.sweep.abs()
    }

    #[inline]
    fn radius_at_u(&self, u: f64) -> f64 {
        self.r_start + (self.r_end - self.r_start) * u
    }
}

impl Segment for ArcSeg {
    fn length(&self) -> f64 {
        let planar = self.sweep.abs() * (self.r_start + self.r_end) / 2.0;
        (planar * planar + self.helix_delta * self.helix_delta).sqrt()
    }

    fn point_at(&self, s: f64) -> PathPoint {
        let len = self.length();
        let u = if len > 0.0 {
            (s / len).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (i0, i1, i2) = self.plane.axes();
        let theta = self.theta0 + self.sweep * u;
        let r = self.radius_at_u(u);
        let mut p = [0.0; 3];
        p[i0] = self.center[i0] + r * theta.cos();
        p[i1] = self.center[i1] + r * theta.sin();
        p[i2] = self.center[i2] + self.helix_delta * u;
        p
    }

    fn tangent_at(&self, s: f64) -> PathPoint {
        let len = self.length();
        let u = if len > 0.0 {
            (s / len).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (i0, i1, i2) = self.plane.axes();
        let theta = self.theta0 + self.sweep * u;
        let r = self.radius_at_u(u);
        let dr = self.r_end - self.r_start;
        // dp/du in plane coordinates
        let d0 = dr * theta.cos() - r * theta.sin() * self.sweep;
        let d1 = dr * theta.sin() + r * theta.cos() * self.sweep;
        let d2 = self.helix_delta;
        let n = (d0 * d0 + d1 * d1 + d2 * d2).sqrt();
        let mut t = [0.0; 3];
        if n > 0.0 {
            t[i0] = d0 / n;
            t[i1] = d1 / n;
            t[i2] = d2 / n;
        }
        t
    }

    fn curvature_at(&self, s: f64) -> f64 {
        let len = self.length();
        let u = if len > 0.0 {
            (s / len).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let r = self.radius_at_u(u);
        let p = self.pitch();
        // Helix curvature; reduces to 1/r for planar arcs. The tiny spiral
        // dr term is ignored (bounded by the radius tolerance).
        r / (r * r + p * p)
    }

    fn start(&self) -> PathPoint {
        self.point_at(0.0)
    }

    fn end(&self) -> PathPoint {
        self.point_at(self.length())
    }

    #[inline]
    fn pose_at(&self, s: f64) -> (PathPoint, PathPoint) {
        let len = self.length();
        let u = if len > 0.0 {
            (s / len).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (i0, i1, i2) = self.plane.axes();
        let (sin, cos) = (self.theta0 + self.sweep * u).sin_cos();
        let r = self.radius_at_u(u);
        let mut p = [0.0; 3];
        p[i0] = self.center[i0] + r * cos;
        p[i1] = self.center[i1] + r * sin;
        p[i2] = self.center[i2] + self.helix_delta * u;
        // dp/du, as in `tangent_at`
        let dr = self.r_end - self.r_start;
        let d0 = dr * cos - r * sin * self.sweep;
        let d1 = dr * sin + r * cos * self.sweep;
        let d2 = self.helix_delta;
        let n = (d0 * d0 + d1 * d1 + d2 * d2).sqrt();
        let mut t = [0.0; 3];
        if n > 0.0 {
            t[i0] = d0 / n;
            t[i1] = d1 / n;
            t[i2] = d2 / n;
        }
        (p, t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, PI, TAU};

    #[test]
    fn quarter_circle_ccw_xy() {
        // Quarter circle from (1,0) to (0,1) around origin, CCW
        let arc = ArcSeg::from_center(
            WorkPlane::Xy,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            true,
            1e-9,
        )
        .unwrap();
        assert!((arc.length() - FRAC_PI_2).abs() < 1e-12);
        assert!((arc.sweep - FRAC_PI_2).abs() < 1e-12);
        let mid = arc.point_at(arc.length() / 2.0);
        let d = FRAC_PI_2 / 2.0;
        assert!((mid[0] - d.cos()).abs() < 1e-12);
        assert!((mid[1] - d.sin()).abs() < 1e-12);
        assert!((arc.curvature_at(0.0) - 1.0).abs() < 1e-12);
        // Tangent at start of a CCW circle from angle 0 is +Y
        let t = arc.tangent_at(0.0);
        assert!((t[0]).abs() < 1e-12 && (t[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn cw_arc_and_planes() {
        // Half circle CW in ZX plane (G18), radius 2
        let arc = ArcSeg::from_center(
            WorkPlane::Zx,
            [2.0, 0.0, 0.0], // x=2, z=0; plane first axis = Z (idx 2), second = X (idx 0)
            [-2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            false,
            1e-9,
        )
        .unwrap();
        assert!((arc.length() - 2.0 * PI).abs() < 1e-12);
        assert!(arc.sweep < 0.0);
        assert!((arc.sweep.abs() - PI).abs() < 1e-12);
    }

    #[test]
    fn full_circle_from_center() {
        let arc = ArcSeg::from_center(
            WorkPlane::Xy,
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            true,
            1e-9,
        )
        .unwrap();
        assert!((arc.sweep - TAU).abs() < 1e-12);
        assert!((arc.length() - TAU).abs() < 1e-12);
    }

    #[test]
    fn radius_format_minor_major() {
        // Quarter circle (1,0) -> (0,1), radius 1: minor CCW arc has center at origin
        let minor = ArcSeg::from_radius(WorkPlane::Xy, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0, true)
            .unwrap();
        assert!((minor.center[0]).abs() < 1e-9 && (minor.center[1]).abs() < 1e-9);
        assert!((minor.sweep - FRAC_PI_2).abs() < 1e-9);
        // Negative R selects the major (270°) arc; its center is at (1,1)
        let major =
            ArcSeg::from_radius(WorkPlane::Xy, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], -1.0, true)
                .unwrap();
        assert!((major.center[0] - 1.0).abs() < 1e-9 && (major.center[1] - 1.0).abs() < 1e-9);
        assert!((major.sweep - 3.0 * FRAC_PI_2).abs() < 1e-9);
    }

    #[test]
    fn helix() {
        // Full CCW circle radius 1 with 2.0 rise in Z
        let arc = ArcSeg::from_center(
            WorkPlane::Xy,
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 2.0],
            [0.0, 0.0, 0.0],
            true,
            1e-9,
        )
        .unwrap();
        let expected = (TAU * TAU + 4.0).sqrt();
        assert!((arc.length() - expected).abs() < 1e-12);
        let end = arc.end();
        assert!((end[2] - 2.0).abs() < 1e-12);
        // Helix curvature r/(r²+p²) with p = 2/τ
        let p = 2.0 / TAU;
        assert!((arc.curvature_at(0.0) - 1.0 / (1.0 + p * p)).abs() < 1e-12);
    }

    #[test]
    fn radius_mismatch_rejected() {
        let err = ArcSeg::from_center(
            WorkPlane::Xy,
            [1.0, 0.0, 0.0],
            [0.0, 1.1, 0.0],
            [0.0, 0.0, 0.0],
            true,
            1e-3,
        );
        assert!(err.is_err());
    }
}
