//! Clamp the programmed feed of arcs to the centripetal-acceleration budget
//! (the `SMC_LimitCircularVelocity` analogue).
//!
//! The look-ahead applies the same cap with the interpolator's own budget;
//! this preprocessor lets a machine apply a separate (stricter) budget and
//! makes the cap visible in the queue.

#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::Preprocessor;
use crate::error::PreprocessError;
use crate::geometry::{GeoElem, Geometry, Segment};
use crate::queue::GeoQueue;

/// Curvature velocity limit preprocessor: clamps arc feeds so the normal
/// (centripetal) acceleration stays within the given budget.
#[derive(Debug, Clone)]
pub struct CurvatureVelocityLimit {
    /// Max normal (centripetal) acceleration: v ≤ sqrt(a_n / κ).
    pub max_normal_acceleration: f64,
}

impl CurvatureVelocityLimit {
    /// Limit with the given normal-acceleration budget (internal units/s²).
    pub fn new(max_normal_acceleration: f64) -> Self {
        Self {
            max_normal_acceleration,
        }
    }
}

impl Preprocessor for CurvatureVelocityLimit {
    fn name(&self) -> &'static str {
        "CurvatureVelocityLimit"
    }

    fn process(&mut self, mut queue: GeoQueue) -> Result<GeoQueue, PreprocessError> {
        for elem in queue.iter_mut() {
            if let GeoElem::Move(mv) = elem {
                if let Geometry::Arc(_) = &mv.geo {
                    let len = mv.geo.length();
                    let kappa = mv.geo.curvature_at(0.0).max(mv.geo.curvature_at(len));
                    if kappa > 0.0 {
                        mv.feed = mv.feed.min((self.max_normal_acceleration / kappa).sqrt());
                    }
                }
            }
        }
        Ok(queue)
    }
}
