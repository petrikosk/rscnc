# rscnc

**CNC motion control in Rust** — a DIN 66025 (G-code) execution pipeline on top of the
[rsruckig](https://github.com/petrikosk/rsruckig) jerk-limited trajectory generator, modeled
after the CoDeSys SoftMotion CNC architecture:

```text
G-code ──► Decoder ──► GeoQueue ──► Preprocessors ──► Interpolator ──► Kinematics ──► axis setpoints
(text)     (modal      (lines,      (corner round,    (look-ahead +    (path space
            state)      arcs,        tool radius       Ruckig 1-DoF     to axis
                        events)      corr., ...)       s-profile)       space)
```

The interpolator runs rsruckig as a **1-DoF planner on the path parameter `s`** (arc length):
per segment, the jerk-limited profile goes from the current `(s, ṡ, s̈)` to `(L, v_exit, 0)`,
where the junction velocity `v_exit` comes from a look-ahead pass — so consecutive segments
**blend without stopping**. Feed-override changes, pause and resume are single profile
retargets from the current state, which is exactly what Ruckig was built for.

## Features

- **DIN 66025 decoder** — `G0 G1 G2 G3` (lines, arcs in all planes, IJK center and R radius
  format, helical arcs), `G4` dwell, `G17/G18/G19` planes, `G40/G41/G42` tool radius
  correction modes, `G70/G71` (+ `G20/G21`) units, `G90/G91`, `G92` preset, `F S T M H`
  words, `N` numbers, comments, block delete, `M2/M30` program end. Modal state machine,
  strict semantic errors with source line numbers.
- **Generic N-axis** — 2 or 3 geometric path axes plus any number of auxiliary synchronized
  axes (`A B C U V W ...`), which interpolate linearly over the path and cap the path
  velocity so their own limits are never exceeded.
- **Look-ahead planner** — per-segment velocity caps (feed, machine cap, centripetal
  `v ≤ √(a_n/κ)`, aux-axis caps) plus Grbl-style junction-deviation corner velocities with
  backward/forward reachability passes.
- **Jerk-limited interpolation** — one rsruckig `calculate()` per segment, sampled with
  `Trajectory::at_time` under exact time bookkeeping (no cycle-quantization error at
  segment boundaries). Position, velocity and acceleration are continuous everywhere.
- **Feed override & control** — override changeable every cycle (0…150 %+), override 0 /
  `halt()` decelerates to a jerk-limited pause (velocity-interface stop that may safely
  cross blended segment boundaries), `resume()` retargets back, single-step mode stops at
  every segment end.
- **Synchronized events** — M functions (with H parameter), spindle `S` and tool `T`
  changes fire exactly once when the path crosses their program position (exact stop in v1).
- **Preprocessors** (composable `GeoQueue → GeoQueue` transforms):
  - `ToolRadiusCorrection` — 2D G41/G42 offsetting with exact line/line, line/arc, arc/arc
    inner-corner intersection trimming, transition arcs on outer corners, lead-in/out moves
    and gouge detection.
  - `CornerRounding` — inserts tangent fillet arcs between linear moves so the look-ahead
    can carry speed through corners.
  - `CurvatureVelocityLimit` — clamps arc feeds to a centripetal-acceleration budget.
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
    cnc.load_program("G1 F1500 X20\nY20\nX0\nY0\nM30\n", DecoderConfig::default(), &mut pipeline)?;

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
`set_single_step(true)` + `step()`.

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

Internally everything is **mm, s** (feeds `F` are programmed in units/min and converted).
Any consistent unit system works — see rsruckig's numerical guidance (state values are
exact to ~1e-8 in the chosen unit). Auxiliary axis words are not unit-scaled (typically
rotary degrees). Arcs follow DIN/LinuxCNC conventions: IJK are incremental offsets from
the start point; `R > 0` selects the minor, `R < 0` the major arc; a small start/end radius
mismatch (CAM output) is absorbed by linear spiral correction within `radius_tolerance`.

## `no_std` support

The library is `no_std`-compatible (an allocator is still required — the queue,
decoder and error types use `alloc`). Disable default features and enable `libm`
for float math:

```toml
[dependencies]
rscnc = { version = "0.1", default-features = false, features = ["libm"] }
```

```bash
cargo build --no-default-features --features libm --target thumbv7em-none-eabihf
```

Either `std` (default) or `libm` must be enabled.

`qemu-demo/` is a bare-metal smoke test that runs the full pipeline on an
emulated Cortex-M3 without FPU (QEMU `mps2-an385`, semihosting output):

```bash
cd qemu-demo && cargo run --release   # requires qemu-system-arm on PATH
```

## Testing

```bash
cargo test
```

- unit tests: lexer, modal decoder, arc geometry (all planes, helix, R format), look-ahead
  rules, corner rounding tangency, tool radius offsets/trims/gouge detection
- integration tests: full programs through the pipeline asserting kinematic limits via
  finite differences, exact endpoints (≤ 1e-8 mm), junction blending, override/pause/resume,
  event/dwell timing, single-step, aux-axis caps
- property tests: random polylines and arcs — limits, geometry adherence (every sampled
  point on the programmed path), exact endpoints

## Scope and future work

Implemented against the CoDeSys SoftMotion CNC feature model; not yet included (backlog):
splines (G5), ellipses, 3D tool radius correction, subprograms/jumps, spline smoothing of
corners, non-stopping ("fly-by") M events, streaming/windowed preprocessing for unbounded
programs, backward move on path, fieldbus drivers (EtherCAT/CiA402).

Known v1 behavior notes:

- Junction reachability uses an acceleration formula with a configurable margin
  (`LookAhead::accel_margin`, default 0.8) to leave headroom for jerk; the interpolator
  additionally clamps each segment's exit velocity to what is reachable from the current
  state, so profiles never reverse along the path.
- An M event forces an exact stop at its program position (CoDeSys default for
  synchronized M functions).
- A pause (override → 0) brakes time-optimally and may cross blended segment boundaries;
  it never crosses an exact-stop boundary.

## License

MIT — see `LICENSE.txt`. Depends on [rsruckig](https://github.com/petrikosk/rsruckig) (MIT).
