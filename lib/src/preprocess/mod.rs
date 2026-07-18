//! Path preprocessors (the `SMC_*` function-block analogues): composable
//! queue → queue transforms applied between the decoder and the interpolator.
//!
//! v1 processes whole queues (the interpolator plans whole programs);
//! a streaming/windowed variant is future work.

mod corner_round;
mod curvature_limit;
mod tool_radius;

pub use corner_round::CornerRounding;
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
