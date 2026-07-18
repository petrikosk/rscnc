//! Error types for every pipeline stage.

use thiserror::Error;

/// Top-level error type; each stage's error converts into it.
#[derive(Error, Debug, Clone, PartialEq)]
pub enum CncError {
    /// Lexical / syntactic error in the G-code text.
    #[error("G-code error: {0}")]
    Gcode(#[from] GcodeError),
    /// Semantic error while decoding blocks into geometry.
    #[error("decode error: {0}")]
    Decode(#[from] DecodeError),
    /// Error in a path preprocessor stage.
    #[error("preprocess error: {0}")]
    Preprocess(#[from] PreprocessError),
    /// Runtime error in the interpolator.
    #[error("interpolator error: {0}")]
    Interpolator(String),
    /// Kinematic transform error.
    #[error("kinematics error: {0}")]
    Kinematics(#[from] KinError),
    /// Error from the rsruckig trajectory generator.
    #[error("trajectory generator error at source line {line}: {message}")]
    Otg {
        /// 1-based source line of the move being interpolated.
        line: usize,
        /// Error message from rsruckig.
        message: String,
    },
}

/// Lexical / syntactic error in the G-code text.
#[derive(Error, Debug, Clone, PartialEq)]
#[error("line {line}: {message}")]
pub struct GcodeError {
    /// 1-based source line the error occurred on.
    pub line: usize,
    /// Human-readable description.
    pub message: String,
}

impl GcodeError {
    /// New error at `line` with the given message.
    pub fn new(line: usize, message: impl Into<String>) -> Self {
        Self {
            line,
            message: message.into(),
        }
    }
}

/// Semantic error while decoding blocks into geometry.
#[derive(Error, Debug, Clone, PartialEq)]
#[error("line {line}: {message}")]
pub struct DecodeError {
    /// 1-based source line the error occurred on.
    pub line: usize,
    /// Human-readable description.
    pub message: String,
}

impl DecodeError {
    /// New error at `line` with the given message.
    pub fn new(line: usize, message: impl Into<String>) -> Self {
        Self {
            line,
            message: message.into(),
        }
    }
}

/// Error in a path preprocessor stage.
#[derive(Error, Debug, Clone, PartialEq)]
#[error("{stage} (source line {line}): {message}")]
pub struct PreprocessError {
    /// Name of the preprocessor stage that failed.
    pub stage: &'static str,
    /// 1-based source line of the offending move.
    pub line: usize,
    /// Human-readable description.
    pub message: String,
}

impl PreprocessError {
    /// New error in `stage` at source line `line` with the given message.
    pub fn new(stage: &'static str, line: usize, message: impl Into<String>) -> Self {
        Self {
            stage,
            line,
            message: message.into(),
        }
    }
}

/// Kinematic transform error (e.g. unreachable pose).
#[derive(Error, Debug, Clone, PartialEq)]
#[error("kinematics: {0}")]
pub struct KinError(pub String);
