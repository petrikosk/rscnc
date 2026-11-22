//! Arbitrary text through the offline pipeline: decode, preprocess (corner
//! rounding, corner smoothing and tool radius correction, each alone, and
//! correction chained with rounding) and plan with the look-ahead. Every
//! stage may reject its input, but none may panic.
//!
//! The work per input is bounded: long inputs and long queues are skipped.

#![no_main]

use libfuzzer_sys::fuzz_target;
use rscnc::decoder::{Decoder, DecoderConfig};
use rscnc::lookahead::LookAhead;
use rscnc::preprocess::{
    CornerRounding, CornerSmoothing, Pipeline, Preprocessor, ToolRadiusCorrection,
};
use rscnc::queue::GeoQueue;
use rscnc::types::PathLimits;

/// Inputs longer than this are skipped.
const MAX_INPUT_BYTES: usize = 4096;
/// Queues longer than this are skipped.
const MAX_QUEUE_LEN: usize = 500;
/// Velocity limit of the one auxiliary axis.
const AUX_MAX_VELOCITY: [f64; 1] = [50.0];

fn plan(queue: &GeoQueue) {
    let limits = PathLimits::new(100.0, 1000.0, 20_000.0, 800.0);
    let _ = LookAhead::default().plan(queue, &limits, &AUX_MAX_VELOCITY);
}

fuzz_target!(|data: &[u8]| {
    if data.len() > MAX_INPUT_BYTES {
        return;
    }
    let Ok(src) = std::str::from_utf8(data) else {
        return;
    };
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        // (a loop in the program must not keep one input busy for long)
        max_executed_blocks: 2 * MAX_QUEUE_LEN,
        ..Default::default()
    };
    let Ok(queue) = Decoder::decode_source(cfg, src) else {
        return;
    };
    if queue.len() > MAX_QUEUE_LEN {
        return;
    }
    plan(&queue);

    if let Ok(rounded) = CornerRounding::new(0.5).process(queue.clone()) {
        plan(&rounded);
    }
    if let Ok(smoothed) = CornerSmoothing::new(0.05).process(queue.clone()) {
        plan(&smoothed);
    }
    if let Ok(corrected) = ToolRadiusCorrection::new(1.0).process(queue.clone()) {
        plan(&corrected);
    }
    let mut chain = Pipeline::new()
        .with(ToolRadiusCorrection::new(1.0))
        .with(CornerRounding::new(0.5));
    if let Ok(processed) = chain.run(queue) {
        plan(&processed);
    }
});
