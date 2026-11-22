# rscnc

**CNC motion control in Rust** — a DIN 66025 (G-code) execution pipeline on top of the
[rsruckig](https://github.com/petrikosk/rsruckig) jerk-limited trajectory generator, modeled
after the CoDeSys SoftMotion CNC architecture:

```text
G-code ──► Decoder ──► GeoQueue ──► Preprocessors ──► Interpolator ──► Kinematics ──► axis setpoints
(text)     (modal      (lines,      (tool radius      (look-ahead +    (path space
            state,      arcs,        corr., corner     Ruckig 1-DoF     to axis
            program     splines,     rounding /        s-profile)       space)
            flow)       events)      smoothing, ...)
```

The interpolator runs rsruckig as a **1-DoF planner on the path parameter `s`** (arc length):
the jerk-limited profile goes from the current `(s, ṡ, s̈)` to `(L, v_exit, 0)`, where the
junction velocity `v_exit` comes from a look-ahead pass — so consecutive segments **blend
without stopping**. Consecutive segments whose junctions do not hold the velocity back (or
hold it back evenly, as on a finely segmented curve) form one *span* that is traversed by a
single profile, so a contour chopped into thousands of short moves accelerates like one long
move. Feed-override changes, pause and resume are single profile retargets from the current
state, which is exactly what Ruckig was built for.

## Features

- **DIN 66025 decoder** — `G0 G1 G2 G3` (lines, arcs in all planes, IJK center and R radius
  format, helical arcs), `G5` splines through programmed points, `G4` dwell, `G17/G18/G19`
  planes, `G40/G41/G42` tool radius correction modes, `G43/G49` tool length compensation,
  `G53` and `G54`…`G59` coordinate systems, `G25/G26` working area, `G70/G71`
  (+ `G20/G21`) units, `G90/G91`, `G92` preset, `F S T M` words, `N` numbers, comments,
  block delete, `M2/M30` program end. Modal state machine, strict semantic errors with
  source line numbers.
- **Variables and program flow** in the style of Fanuc Macro B — `#1 = [#2 + 3] * SIN[30]`,
  computed words (`X[#1 * 2]`), `GOTO`, `IF … GOTO` / `IF … THEN`, `WHILE … DO … END`,
  subprograms (`M98 P… K…` / `M99`), with a guard against programs that never end.
- **CODESYS-compatible dynamics words** — `F` is the path velocity in units/**second**, `E`
  the maximum path acceleration (positive) or deceleration (negative), `FF`/`EF` the same
  pair for `G0` rapids, and `F<axis>`/`E<axis>` the limits of one auxiliary axis. All are
  modal, and the per-segment values reach both the look-ahead and the jerk-limited profile.
- **Switch points (`H` functions)** — `H` with `L` (distance from the element's start, or
  from its end when negative) or `O` (relative position in `[0, 1]`) puts a switch point
  *inside* a path element; a positive `H` switches its function on, a negative one off.
  Several fit in one block, they survive tool radius correction and corner rounding, and
  the interpolator reports each one in the cycle it is crossed.
- **Generic N-axis** — 2 or 3 geometric path axes plus any number of auxiliary synchronized
  axes (`A B C U V W ...`), which interpolate linearly over the path.
- **Per-axis limits** — velocity, acceleration and jerk limits of every machine axis are
  kept on top of the path limits (a move's limits follow from the share each axis has in
  it, the centripetal acceleration on curves included), plus an optional limit on the
  velocity *step* an axis may take at a junction, and software end positions that refuse a
  program before it runs.
- **Look-ahead planner** — per-segment velocity caps (feed, machine cap, centripetal
  `v ≤ √(a_n/κ)`, aux-axis caps) plus Grbl-style junction-deviation corner velocities
  (auxiliary axes included; a polyline sampling a curve obeys the same centripetal budget
  as the arc it approximates) with backward/forward reachability passes that use the exact
  jerk-limited ramp distance. Where the velocity limit changes in many small steps — a
  curve cut into pieces of different curvature, smoothed corners, a run of feed changes —
  the plan **carries acceleration across span boundaries**, so the path brakes or
  accelerates in one ramp instead of one S-curve per step.
- **Jerk-limited interpolation** — one rsruckig `calculate()` per span, sampled with
  `Trajectory::at_time` under exact time bookkeeping (no cycle-quantization error at
  segment boundaries). Position, velocity and acceleration are continuous everywhere.
- **Feed override & control** — override changeable every cycle (0…150 %+; above 100 % it
  never lifts a move past the machine, curvature, junction or aux-axis limits), override 0 /
  `halt()` decelerates to a jerk-limited pause (velocity-interface stop that may safely
  cross blended segment boundaries), `resume()` retargets back, single-step mode stops at
  every segment end.
- **Backward motion** — `set_reverse(true)` stops the path and runs it back along what it
  has traversed, through the same look-ahead and with override, halt and single-step;
  forward again, the program resumes without repeating what it has already reported.
- **Streaming** — more program can be appended while the path runs (`append_program`,
  `Interpolator::append`): the look-ahead is extended over it and the path does not stop
  at the seam. `discard_history` drops what lies behind, so a program fed in pieces does
  not grow without bound.
- **Synchronized events** — M functions, spindle `S` and tool `T` changes fire exactly once
  when the path crosses their program position. They stop the path there, except the M
  functions listed as *fly-by* (`DecoderConfig::fly_by_m`), which are reported while the
  path runs through.
- **Preprocessors** (composable `GeoQueue → GeoQueue` transforms):
  - `ToolRadiusCorrection` — 2D G41/G42 offsetting with exact line/line, line/arc, arc/arc
    inner-corner intersection trimming, transition arcs on outer corners, lead-in/out moves,
    and removal of contour details the tool does not fit into (a slot narrower than the
    tool is passed over instead of cut into).
  - `CornerRounding` — inserts tangent fillet arcs between linear moves so the look-ahead
    can carry speed through corners.
  - `CornerSmoothing` — the `G64 P…` idea: quintic blends that stay within a tolerance of
    the programmed corner and join the lines with continuous curvature.
  - `CurvatureVelocityLimit` — clamps the feed on curved elements to a
    centripetal-acceleration budget.
- **Simulation-first** — `CncController` facade + `SimDriver` log for tests and SVG plots;
  real hardware goes behind the `AxisDriver` trait (fieldbus integration is future work).

## Quick start

```rust
use rscnc::prelude::*;

fn main() -> Result<(), CncError> {
    let axis_config = AxisConfig::<3>::cartesian(
        ["X", "Y", "Z"],
        [60.0; 3],    // mm/s
        [800.0; 3],   // mm/s²
        [10000.0; 3], // mm/s³
    );
    let limits = PathLimits::new(60.0, 800.0, 10000.0, 800.0);
    let mut cnc = CncController::new(
        axis_config,
        limits,
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        0.001, // 1 ms control cycle
    );

    let mut pipeline = Pipeline::new(); // optionally .with(CornerRounding::new(1.0)) ...
    // F is in mm/s
    cnc.load_program("G1 F25 X20\nY20\nX0\nY0\nM30\n", DecoderConfig::default(), &mut pipeline)?;

    let mut driver = SimDriver::<3>::new(); // or your own AxisDriver impl
    loop {
        let out = cnc.cycle(&mut driver)?;
        for event in &out.events {
            println!("line {}: M{:?}", event.src_line, event.m);
        }
        if out.state == IplState::Finished {
            break;
        }
        // driver received the axis position/velocity setpoints for this cycle
    }
    Ok(())
}
```

Runtime control on the interpolator: `set_override(0.4)`, `halt()` / `resume()`,
`set_single_step(true)` + `step()`, `set_reverse(true)`.

### Feeding a program in pieces

```rust,ignore
cnc.load_program(first_piece, DecoderConfig::default(), &mut pipeline)?;
loop {
    let out = cnc.cycle(&mut driver)?;
    if cnc.interpolator.remaining_length() < 50.0 {
        if let Some(text) = next_piece() {
            // decoded with the modal state the previous piece ended in
            cnc.append_program(text, &mut pipeline)?;
        }
        cnc.interpolator.discard_history(100.0); // keep 100 mm for backward motion
    }
    if out.state == IplState::Finished { break; }
}
```

The path does not stop at the seam between two pieces as long as the next one arrives
before the path brakes for the end of the previous one. Preprocessors see each piece on its
own (no rounding, smoothing or radius correction across a seam), and a ramp that crosses a
seam does so in two S-curves. `append_program` and `discard_history` plan and allocate:
call them beside the control cycle, not in it.

### Real-time use

`cycle()` allocates a fresh `IplOutput` on every call. In a control loop, keep one buffer
and refill it — a steady-state cycle (segment boundaries included) then performs **zero**
heap allocations:

```rust,ignore
let mut out = cnc.interpolator.output_buffer(); // sized for the loaded program
loop {
    cnc.cycle_into(&mut driver, &mut out)?;     // or interpolator.cycle_into(&mut out)
    if out.state == IplState::Finished { break; }
}
```

The planned program's own facts are readable from the interpolator without walking the
queue: `total_length()`, `start_position()` / `start_aux()` (valid before the first cycle)
and `current_is_rapid()` (whether the path element under the current output is a `G0`).

Errors keep their pipeline stage: `Decoder::decode_source` reports a lexical failure as
`CncError::Gcode` and a semantic one as `CncError::Decode`, each with its 1-based source
line. Nothing reachable through the public API panics — degenerate limits or a degenerate
cycle time are refused by `Interpolator::load`, which matters on `wasm32-unknown-unknown`
where a panic is an unrecoverable trap.

## Examples

Each example writes SVG plots into the working directory:

```bash
cargo run --release --bin example_square    # sharp corners: junction slowdown
cargo run --release --bin example_arcs      # tangent arcs: blending + curvature cap
cargo run --release --bin example_override  # 100% → 40% → 0% (pause) → 150%
cargo run --release --bin example_file      # full pipeline incl. G41 on a .nc file
cargo run --release --bin example_file -- path/to/program.nc
```

## Units and conventions

Internally everything is **mm, s**. `F` and `E` are programmed in units per second and per
second squared, as in CODESYS SoftMotion — *not* the units/min of many CAM dialects.
Any consistent unit system works — see rsruckig's numerical guidance (state values are
exact to ~1e-8 in the chosen unit). Auxiliary axis words are not unit-scaled (typically
rotary degrees). Arcs follow DIN/LinuxCNC conventions: IJK are incremental offsets from
the start point; `R > 0` selects the minor, `R < 0` the major arc; a small start/end radius
mismatch (CAM output) is absorbed by linear spiral correction within `radius_tolerance`.

## Testing

```bash
cargo test
```

- unit tests: lexer, modal decoder, expressions and program flow, arc and spline geometry,
  look-ahead rules, corner rounding tangency, corner smoothing tolerance, tool radius
  offsets/trims/loop removal (including random profiles that must never be cut into)
- integration tests: full programs through the pipeline asserting kinematic limits via
  finite differences, exact endpoints (≤ 1e-8 mm), junction blending, override/pause/resume,
  event/dwell timing, single-step, aux-axis caps
- property tests: random polylines and arcs — limits, geometry adherence (every sampled
  point on the programmed path), exact endpoints
- regression tests (`lib/tests/regression.rs`): one test per defect fixed so far — stalls,
  jolts, limit violations, rejected programs
- feature tests: per-axis limits and fly-by events (`axis_limits.rs`), the G-code dialect
  (`dialect.rs`), splines and smoothing (`spline.rs`), streaming (`streaming.rs`), backward
  motion (`reverse.rs`), carried acceleration (`carry.rs`), and the zero-allocation cycle
  (`allocation.rs`)
- stress test (`lib/tests/stress.rs`): random programs — streamed in pieces or loaded whole
  — under random override / halt / single-step / reverse actions across several limit sets
  and cycle times; the path motion must stay one continuous jerk-limited profile within the
  programmed `E` limits, the velocity caps and the per-axis limits, report every switch
  point once, and end exactly at the program end. `RSCNC_STRESS_CASES=100000 cargo test
  --release --test stress` runs a longer soak (`RSCNC_STRESS_FIRST` moves it to other seeds)
- `cargo bench -p rscnc` runs the Criterion benchmarks; `fuzz/` holds cargo-fuzz targets
  for the lexer, the decoder and the offline pipeline

## Scope and future work

Implemented against the CoDeSys SoftMotion CNC feature model; not included (backlog):
ellipses and NURBS words, 3D tool radius correction, preprocessing across the seam between
two streamed pieces, fieldbus drivers (EtherCAT/CiA402), `no_std`.

Behavior notes:

- Junction reachability uses the exact jerk-limited ramp distance, derated by a configurable
  margin on acceleration and jerk (`LookAhead::accel_margin`, default 0.8) as headroom for
  override changes. A retarget (override, single-step) that the current state cannot follow
  any more — typically in the last ramp before a junction — keeps the running profile and
  takes effect at the next span; profiles never reverse or stall along the path.
- Carried acceleration (`LookAhead::carry_acceleration`, on by default) only ever adds to
  the zero-acceleration plan, which is still planned and followed wherever no profile leads
  straight to a carried state. Inside a carried braking ramp that plan can be out of reach;
  should the ramp not be continuable there, the path brakes to a standstill and goes on
  from rest by itself. Switch it off to get a profile per span with zero acceleration at
  every span boundary.
- Junction deviation limits the *centripetal* effect of a corner, not the jump itself: at a
  corner taken at speed the axis velocities change direction within one cycle (tangential
  velocity, acceleration and jerk stay continuous and limited). Set
  `AxisConfig::with_velocity_jump` to bound the step per axis, lower
  `LookAhead::junction_deviation`, or insert `CornerRounding` / `CornerSmoothing`, where the
  drives need less.
- `CornerSmoothing` makes the path curvature-continuous, but a blend is confined to the two
  moves it joins: on a contour of very short moves the blends are tight, and the smoothed
  path can run slower than the unsmoothed one under the junction-deviation rule (which
  allows a velocity step at each corner).
- An M event forces an exact stop at its program position (CoDeSys default for
  synchronized M functions) unless it is configured as fly-by.
- A pause (override → 0) brakes time-optimally and may cross blended segment boundaries;
  it never crosses an exact-stop boundary.
- Backward motion goes back as far as the program start, or as far as `discard_history`
  has left; dwells, events and switch points are not executed on the way back.
- Tool radius correction removes contour details locally (see the module docs for the
  limits): a bottleneck between two distant parts of a contour is not detected.

## License

MIT — see `LICENSE.txt`. Depends on [rsruckig](https://github.com/petrikosk/rsruckig) (MIT).
