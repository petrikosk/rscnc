//! DIN 66025 G-code front end: lexer and modal program state.

pub mod lexer;
pub mod program;

pub use lexer::{lex, ParsedBlock, Word};
pub use program::{DistanceMode, ModalState, MotionMode, TrcMode, Units};
