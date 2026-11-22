//! Benchmarks of the offline part of the pipeline: lexing and decoding,
//! look-ahead planning, the path preprocessors and loading a program into
//! the interpolator.
//!
//! Run with `cargo bench -p rscnc --bench pipeline` (add `-- --quick` for a
//! fast, less precise pass).

use std::f64::consts::PI;
use std::fmt::Write as _;
use std::hint::black_box;

use criterion::{criterion_group, criterion_main, BatchSize, Criterion, Throughput};
use rscnc::prelude::*;

/// Fixed-seed xorshift, so every run benchmarks the same program.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.unit()
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

/// Number of motion blocks in the generated program.
const BLOCKS: usize = 10_000;

/// A program of [`BLOCKS`] short moves the way a CAM system writes them:
/// mostly `G1` blocks of 0.05 .. 2 mm along a gently winding path with an
/// occasional sharp corner, and a `G2`/`G3` arc about every 25th block.
fn short_block_program() -> String {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let mut src = String::from("N1 G21 G90\nN2 G1 F60\n");
    let (mut x, mut y) = (0.0f64, 0.0f64);
    let mut heading = 0.0f64;
    for n in 0..BLOCKS {
        let len = rng.range(0.05, 2.0);
        heading += if rng.below(50) == 0 {
            rng.range(-PI, PI)
        } else {
            rng.range(-0.15, 0.15)
        };
        x += len * heading.cos();
        y += len * heading.sin();
        let number = n + 3;
        if rng.below(25) == 0 {
            // (any radius above half the chord fits)
            let r = len * rng.range(0.6, 4.0);
            let g = 2 + rng.below(2);
            writeln!(src, "N{number} G{g} X{x:.4} Y{y:.4} R{r:.4}").unwrap();
            writeln!(src, "G1").unwrap();
        } else {
            writeln!(src, "N{number} X{x:.4} Y{y:.4}").unwrap();
        }
    }
    src.push_str("M30\n");
    src
}

/// Number of teeth of the gear contour.
const TEETH: usize = 1000;

/// A closed gear-like contour of `2 * TEETH` straight flanks (about 6 mm
/// each, corners of about 115 degrees, alternately convex and concave), run
/// counter-clockwise from a lead-in outside it. `trc` is the correction word
/// of the lead-in block (`"G41 "`, `"G42 "` or empty).
fn gear_contour_program(trc: &str) -> String {
    let (tip, root) = (1000.0f64, 995.0f64);
    let mut src = format!("G0 X{:.4} Y0\n", tip + 10.0);
    writeln!(src, "{trc}G1 F50 X{tip:.4} Y0").unwrap();
    for k in 1..=2 * TEETH {
        let angle = k as f64 * PI / TEETH as f64;
        let r = if k % 2 == 0 { tip } else { root };
        let (x, y) = (r * angle.cos(), r * angle.sin());
        writeln!(src, "X{x:.4} Y{y:.4}").unwrap();
    }
    writeln!(src, "G40 G1 X{:.4} Y0\nM30", tip + 10.0).unwrap();
    src
}

fn limits() -> PathLimits {
    PathLimits::new(100.0, 1000.0, 20_000.0, 800.0)
}

fn decode(src: &str) -> GeoQueue {
    Decoder::decode_source(DecoderConfig::default(), src).expect("benchmark program decodes")
}

fn bench_decode(c: &mut Criterion) {
    let src = short_block_program();
    let blocks = rscnc::gcode::lex(&src)
        .expect("benchmark program lexes")
        .len();
    assert!(decode(&src).len() >= BLOCKS);

    let mut group = c.benchmark_group("decode");
    group.throughput(Throughput::Elements(blocks as u64));
    group.bench_function("lex_10k", |b| {
        b.iter(|| rscnc::gcode::lex(black_box(&src)).expect("lexes"))
    });
    group.bench_function("decode_source_10k", |b| {
        b.iter(|| {
            Decoder::decode_source(DecoderConfig::default(), black_box(&src)).expect("decodes")
        })
    });
    group.finish();
}

fn bench_lookahead(c: &mut Criterion) {
    let queue = decode(&short_block_program());
    let limits = limits();
    let lookahead = LookAhead::default();

    let mut group = c.benchmark_group("lookahead");
    group.throughput(Throughput::Elements(queue.len() as u64));
    group.bench_function("plan_10k", |b| {
        b.iter(|| lookahead.plan(black_box(&queue), &limits, &[]))
    });
    group.finish();
}

fn bench_preprocess(c: &mut Criterion) {
    let plain = decode(&gear_contour_program(""));
    let left = decode(&gear_contour_program("G41 "));
    let right = decode(&gear_contour_program("G42 "));

    // The stages must really do their work on these contours: a fillet in
    // every corner, and an offset contour that differs from the programmed
    // one.
    let rounded = CornerRounding::new(0.5)
        .process(plain.clone())
        .expect("the contour can be rounded");
    assert!(rounded.len() >= plain.len() + 2 * TEETH);
    for queue in [&left, &right] {
        let corrected = ToolRadiusCorrection::new(1.0)
            .process(queue.clone())
            .expect("the contour can be corrected");
        assert!(corrected.len() >= queue.len());
        assert_ne!(&corrected, queue);
    }

    let mut group = c.benchmark_group("preprocess");
    group.throughput(Throughput::Elements(plain.len() as u64));
    group.bench_function("corner_rounding", |b| {
        b.iter_batched(
            || plain.clone(),
            |queue| CornerRounding::new(0.5).process(queue).expect("rounds"),
            BatchSize::LargeInput,
        )
    });
    // G41 on a counter-clockwise contour offsets to the inside (the tips
    // are trimmed, the roots get transition arcs); G42 does the opposite.
    for (name, queue) in [("tool_radius_left", &left), ("tool_radius_right", &right)] {
        group.bench_function(name, |b| {
            b.iter_batched(
                || queue.clone(),
                |queue| {
                    ToolRadiusCorrection::new(1.0)
                        .process(queue)
                        .expect("corrects")
                },
                BatchSize::LargeInput,
            )
        });
    }
    group.finish();
}

fn bench_load(c: &mut Criterion) {
    let queue = decode(&short_block_program());
    let mut ipl = Interpolator::new(limits(), LookAhead::default(), Vec::new(), 0.001);

    let mut group = c.benchmark_group("interpolator");
    group.throughput(Throughput::Elements(queue.len() as u64));
    group.bench_function("load_10k", |b| {
        b.iter_batched(
            || queue.clone(),
            |queue| ipl.load(queue).expect("loads"),
            BatchSize::LargeInput,
        )
    });
    group.finish();
}

criterion_group!(
    benches,
    bench_decode,
    bench_lookahead,
    bench_preprocess,
    bench_load
);
criterion_main!(benches);
