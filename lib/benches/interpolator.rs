//! Benchmarks of the real-time part: the cost of one interpolator cycle
//! (`Interpolator::cycle_into`), averaged over a steady stream of cycles
//! through a whole program.
//!
//! Each benchmark keeps one interpolator running; whenever the program
//! finishes it is loaded again, and only the cycles themselves are timed.
//! The reported time is therefore the *mean* cycle, segment boundaries and
//! replanning included. It says nothing about the worst single cycle.
//!
//! Run with `cargo bench -p rscnc --bench interpolator` (add `-- --quick`
//! for a fast, less precise pass).

use std::f64::consts::TAU;
use std::fmt::Write as _;
use std::hint::black_box;
use std::time::{Duration, Instant};

use criterion::{criterion_group, criterion_main, Criterion};
use rscnc::prelude::*;

/// Control period of the benchmarked interpolator, seconds.
const CYCLE_TIME: f64 = 0.001;

/// A meander of 200 mm lines with 20 mm steps between them: nearly every
/// cycle only samples the running profile.
fn long_line_program() -> String {
    let mut src = String::from("G1 F80\n");
    for row in 0..20 {
        let x = if row % 2 == 0 { 200.0 } else { 0.0 };
        let y = 20.0 * row as f64;
        writeln!(src, "X{x:.1} Y{y:.1}\nY{:.1}", y + 20.0).unwrap();
    }
    src
}

/// A closed figure eight (`x = 60 sin t`, `y = 40 sin 2t`) chopped into
/// chords of 0.1 mm, about 4300 of them. At the programmed feed (0.06 mm per
/// cycle) the path crosses a segment boundary on more than half the cycles.
fn fine_curve_program() -> String {
    const STEP: f64 = 0.1;
    let mut src = String::from("G1 F60\n");
    let mut t = 0.0f64;
    while t < TAU {
        let (x, y) = (60.0 * t.sin(), 40.0 * (2.0 * t).sin());
        writeln!(src, "X{x:.4} Y{y:.4}").unwrap();
        let speed = (60.0 * t.cos()).hypot(80.0 * (2.0 * t).cos());
        t += STEP / speed;
    }
    src.push_str("X0 Y0\n");
    src
}

/// An interpolator running one program over and over.
struct Stream {
    ipl: Interpolator,
    out: IplOutput,
    queue: GeoQueue,
    /// Cycles run so far (drives the override pattern).
    tick: usize,
}

impl Stream {
    fn new(src: &str) -> Self {
        let queue = Decoder::decode_source(DecoderConfig::default(), src)
            .expect("benchmark program decodes");
        let mut ipl = Interpolator::new(
            PathLimits::new(100.0, 1000.0, 20_000.0, 800.0),
            LookAhead::default(),
            Vec::new(),
            CYCLE_TIME,
        );
        ipl.load(queue.clone()).expect("benchmark program loads");
        let out = ipl.output_buffer();
        Self {
            ipl,
            out,
            queue,
            tick: 0,
        }
    }

    /// Run `cycles` cycles and return the time spent in them. `before_cycle`
    /// runs (timed) ahead of every cycle with the running cycle count.
    /// Loading the program again after it finished is not timed.
    fn run(&mut self, cycles: u64, before_cycle: impl Fn(&mut Interpolator, usize)) -> Duration {
        let mut elapsed = Duration::ZERO;
        let mut left = cycles;
        while left > 0 {
            if self.out.state == IplState::Finished {
                self.ipl
                    .load(self.queue.clone())
                    .expect("benchmark program loads");
            }
            let start = Instant::now();
            while left > 0 {
                before_cycle(&mut self.ipl, self.tick);
                self.ipl.cycle_into(&mut self.out).expect("cycle");
                self.tick += 1;
                left -= 1;
                if self.out.state == IplState::Finished {
                    break;
                }
            }
            elapsed += start.elapsed();
            black_box(&self.out);
        }
        elapsed
    }
}

/// The feed override of cycle `tick`: a slow sweep between 50 % and 100 %
/// that differs from the previous cycle's value on every cycle.
fn override_at(tick: usize) -> f64 {
    0.75 + 0.25 * (tick as f64 * 0.01).sin()
}

fn programs() -> [(&'static str, String); 2] {
    [
        ("long_lines", long_line_program()),
        ("fine_curve", fine_curve_program()),
    ]
}

/// Plain running at 100 % override.
fn bench_cycle(c: &mut Criterion) {
    let mut group = c.benchmark_group("cycle");
    for (name, src) in programs() {
        let mut stream = Stream::new(&src);
        group.bench_function(name, |b| {
            b.iter_custom(|iters| stream.run(iters, |_, _| {}))
        });
    }
    group.finish();
}

/// A new feed override on every cycle, so every cycle retargets the profile.
fn bench_override(c: &mut Criterion) {
    let mut group = c.benchmark_group("override");
    for (name, src) in programs() {
        let mut stream = Stream::new(&src);
        group.bench_function(name, |b| {
            b.iter_custom(|iters| {
                stream.run(iters, |ipl, tick| ipl.set_override(override_at(tick)))
            })
        });
    }
    group.finish();
}

criterion_group!(benches, bench_cycle, bench_override);
criterion_main!(benches);
