//! Path preprocessors (the `SMC_*` function-block analogues): composable
//! queue → queue transforms applied between the decoder and the interpolator.
//!
//! - [`ToolRadiusCorrection`]: G41/G42 offsetting, with removal of contour
//!   details the tool does not fit into.
//! - [`CornerRounding`]: tangent arcs of a given radius at line–line corners.
//! - [`CornerSmoothing`]: curvature-continuous spline blends that stay within
//!   a tolerance of the programmed corner (the `G64 P…` idea).
//! - [`CurvatureVelocityLimit`]: caps the feed on curved elements.
//!
//! Order matters: radius correction first (it offsets lines and arcs only),
//! then rounding or smoothing of the corrected path.
//!
//! A preprocessor works on one whole queue. A program fed in pieces
//! ([`CncController::append_program`](crate::CncController::append_program))
//! is preprocessed piece by piece: nothing is rounded, smoothed or offset
//! across the seam between two pieces.

mod corner_round;
mod corner_smooth;
mod curvature_limit;
mod tool_radius;

pub use corner_round::CornerRounding;
pub use corner_smooth::CornerSmoothing;
pub use curvature_limit::CurvatureVelocityLimit;
pub use tool_radius::ToolRadiusCorrection;

use crate::error::PreprocessError;
use crate::queue::GeoQueue;

/// A queue → queue transform.
pub trait Preprocessor {
    /// Stage name used in error reports.
    fn name(&self) -> &'static str;
    /// Transform the queue, consuming it.
    fn process(&mut self, queue: GeoQueue) -> Result<GeoQueue, PreprocessError>;
}

/// Ordered chain of preprocessors.
#[derive(Default)]
pub struct Pipeline {
    stages: Vec<Box<dyn Preprocessor>>,
}

impl Pipeline {
    /// An empty pipeline (passes queues through unchanged).
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a stage (builder style).
    pub fn with(mut self, stage: impl Preprocessor + 'static) -> Self {
        self.stages.push(Box::new(stage));
        self
    }

    /// `true` when the pipeline has no stages.
    pub fn is_empty(&self) -> bool {
        self.stages.is_empty()
    }

    /// Run the queue through all stages in order.
    pub fn run(&mut self, mut queue: GeoQueue) -> Result<GeoQueue, PreprocessError> {
        for stage in &mut self.stages {
            queue = stage.process(queue)?;
        }
        Ok(queue)
    }
}
