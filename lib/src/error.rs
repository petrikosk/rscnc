//! Error types for every pipeline stage.

use thiserror::Error;

/// Top-level error type; each stage's error converts into it.
#[derive(Error, Debug, Clone, PartialEq)]
pub enum CncError {
    #[error("G-code error: {0}")]
    Gcode(#[from] GcodeError),
    #[error("decode error: {0}")]
    Decode(#[from] DecodeError),
    #[error("preprocess error: {0}")]
    Preprocess(#[from] PreprocessError),
    #[error("interpolator error: {0}")]
    Interpolator(String),
    #[error("kinematics error: {0}")]
    Kinematics(#[from] KinError),
    #[error("trajectory generator error at source line {line}: {message}")]
    Otg { line: usize, message: String },
}

/// Lexical / syntactic error in the G-code text.
#[derive(Error, Debug, Clone, PartialEq)]
#[error("line {line}: {message}")]
pub struct GcodeError {
    pub line: usize,
    pub message: String,
}

impl GcodeError {
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
    pub line: usize,
    pub message: String,
}

impl DecodeError {
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
    pub stage: &'static str,
    pub line: usize,
    pub message: String,
}

impl PreprocessError {
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
