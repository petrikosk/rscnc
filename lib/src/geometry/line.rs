//! Straight line segment.

use super::{norm, sub, Segment};
use crate::types::PathPoint;

#[derive(Debug, Clone, PartialEq)]
pub struct LineSeg {
    pub p0: PathPoint,
    pub p1: PathPoint,
}

impl LineSeg {
    pub fn new(p0: PathPoint, p1: PathPoint) -> Self {
        Self { p0, p1 }
    }
}

impl Segment for LineSeg {
    fn length(&self) -> f64 {
        norm(&sub(&self.p1, &self.p0))
    }

    fn point_at(&self, s: f64) -> PathPoint {
        let len = self.length();
        let u = if len > 0.0 {
            (s / len).clamp(0.0, 1.0)
        } else {
            0.0
        };
        [
            self.p0[0] + (self.p1[0] - self.p0[0]) * u,
            self.p0[1] + (self.p1[1] - self.p0[1]) * u,
            self.p0[2] + (self.p1[2] - self.p0[2]) * u,
        ]
    }

    fn tangent_at(&self, _s: f64) -> PathPoint {
        let d = sub(&self.p1, &self.p0);
        let len = norm(&d);
        if len > 0.0 {
            [d[0] / len, d[1] / len, d[2] / len]
        } else {
            [0.0; 3]
        }
    }

    fn curvature_at(&self, _s: f64) -> f64 {
        0.0
    }

    fn start(&self) -> PathPoint {
        self.p0
    }

    fn end(&self) -> PathPoint {
        self.p1
    }
}
