//! Arbitrary text through the lexer and the decoder, once with the default
//! configuration and once with auxiliary axes and fly-by M functions: both
//! may reject the program, but neither may panic.

#![no_main]

use libfuzzer_sys::fuzz_target;
use rscnc::decoder::{Decoder, DecoderConfig};

/// Blocks a program may execute beyond its own length (jumps and loops).
const MAX_EXECUTED_BLOCKS: usize = 2000;

fuzz_target!(|data: &[u8]| {
    let Ok(src) = std::str::from_utf8(data) else {
        return;
    };
    // (a loop in the program must not keep one input busy for long)
    let plain = DecoderConfig {
        max_executed_blocks: MAX_EXECUTED_BLOCKS,
        ..Default::default()
    };
    let _ = Decoder::decode_source(plain, src);

    let with_aux = DecoderConfig {
        aux_letters: vec!['A', 'B'],
        start_aux: vec![0.0, 0.0],
        fly_by_m: vec![7, 8, 9],
        max_executed_blocks: MAX_EXECUTED_BLOCKS,
        ..Default::default()
    };
    let _ = Decoder::decode_source(with_aux, src);
});
