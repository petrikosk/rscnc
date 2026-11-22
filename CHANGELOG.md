# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[semantic versioning](https://semver.org/) (pre-1.0: a `0.x` series stays
API-compatible).

## [0.3.0]

The motion layer grows from "whole programs of lines and arcs" towards what a
real control needs: per-axis limits, splines, a larger G-code dialect with
variables and program flow, programs fed in pieces while the path runs,
backward motion, software end positions — and a review pass over the planner
and the interpolator that fixed a number of defects in 0.2.0.

### Changed — breaking

- **`Geometry`, `PlannedSegment` and `IplOutput` are `#[non_exhaustive]`.**
  A `match` on `Geometry` needs a wildcard arm, and the two structs can no
  longer be built with a literal (`IplOutput::default()`,
  `Interpolator::output_buffer()`). This is what lets later versions add to
  them without another breaking release.
- **New variants and fields.** `Geometry::Spline` and `MotionMode::Spline`
  (`G5`); `DecoderConfig` gained `fly_by_m`, `work_offsets`, `tool_lengths`,
  `work_area_min`/`work_area_max`, `variables` and `max_executed_blocks`;
  `AxisConfig` gained `max_velocity_jump`, `soft_limit_min` and
  `soft_limit_max`; `LookAhead` gained `carry_acceleration`; `SyncEvent`
  gained `fly_by`; `ModalState` gained `work_offset` and `tool_length`;
  `ToolRadiusCorrection` gained `remove_loops`; `PlannedSegment` gained
  `v_junction`, `jerk`, `v_carry` and `a_carry`; `IplOutput` gained
  `reverse`; `CncError` gained `SoftLimit`. Struct literals of the
  configuration types need the new fields or `..Default::default()`.
- **`ModalState::position` is the machine position** (what the queue's
  geometry is in). With work offsets and tool length compensation the
  programmed position is no longer a single shift away from it.
- **`CncController` applies the per-axis limits of `AxisConfig`.** The
  acceleration and jerk limits of the axes were only handed to the driver
  side; now they bound every move the axis takes part in (see *Per-axis
  limits* below). A machine whose axis limits are tighter than its
  `PathLimits` runs slower than under 0.2.0 — and within its limits.
- **The planned motion changes.** Spans (one profile over many short moves),
  exact jerk-limited reachability, the corrected junction velocity and
  carried acceleration all change the velocity profile of a given program;
  see *Fixed* and *Changed*.
- **Tool radius correction removes contour details the tool does not fit
  into** instead of refusing the program (`without_loop_removal()` restores
  the strict behaviour), and refuses offset contours that cannot be closed
  without cutting into the programmed one.
- **rsruckig 3.1.2 is the minimum** (was 3.0).

### Added

Motion planning and interpolation:

- **Per-axis limits** (`AxisLimit`, `AxisLimits`,
  `Interpolator::with_axis_limits`, `LookAhead::plan_with_axis_limits`).
  Velocity, acceleration and jerk limits of the path coordinates and of the
  auxiliary axes are kept on top of the path limits: a move's limits become
  `min(path limit, axis limit / share)`, where the share is the axis' part
  of the path tangent (or an auxiliary axis' travel per path length), and
  the centripetal acceleration on arcs and splines is held within the
  acceleration limits of the axes it points along. `CncController` takes
  them from `AxisConfig`.
- **Velocity-jump limit per axis** (`AxisLimit::max_velocity_jump`,
  `AxisConfig::with_velocity_jump`): the junction velocity is kept low
  enough that no axis' velocity steps by more than its limit where the path
  direction (or an auxiliary axis' rate) changes abruptly.
- **Acceleration carried across span boundaries**
  (`LookAhead::carry_acceleration`, on by default). A chain of short spans
  with stepwise changing velocity limits — a run of small feed changes, a
  curve cut into pieces of different curvature, smoothed corners — is
  braked or accelerated in one ramp instead of one S-curve per span: the
  plan gives such span ends a velocity *and* an acceleration
  (`PlannedSegment::v_carry`, `a_carry`). A feed staircase of forty 0.2 mm
  blocks from 50 down to 5 mm/s: 1666 → 1280 cycles at 1 ms, in 2 ramps
  instead of 28; a 10 000-block CAM-style program with smoothed corners
  runs 5 % shorter. The zero-acceleration plan is still planned and is what
  the interpolator follows wherever it finds no profile straight to a
  carried state. Planning costs more with it (1.0 → 1.4 ms for 10 000
  blocks, up to 6× where nearly every span end carries).
- **Fly-by M functions** (`DecoderConfig::fly_by_m`, `SyncEvent::fly_by`):
  the listed M functions no longer stop the path; the moves on their two
  sides are blended and the event is reported in the cycle that crosses the
  junction.
- **Streaming** (`Interpolator::append`, `append_pending`,
  `remaining_length`, `discard_history`; `CncController::append_program`,
  `decoder_mut`; `Decoder::decode_chunk`). More program can be appended
  while the path runs; the look-ahead is extended over it, so the path does
  not stop where the previous piece ended. `discard_history` drops what
  lies behind, so a program fed in pieces does not grow without bound.
- **Backward motion** (`Interpolator::set_reverse`, `is_reverse`,
  `IplOutput::reverse`): the path stops, then runs back along the moves it
  has traversed, through the same look-ahead, with halt, override and
  single-step; forward again, it resumes the program without repeating
  events, dwells and switch points it has already reported.
- **Software end positions** (`AxisConfig::with_soft_limits`,
  `CncController::check_soft_limits`, `CncError::SoftLimit`): a program
  that would move an axis out of its range is refused when it is loaded or
  appended, with the source line of the move. Arc bulges and spline sweeps
  are included, through the kinematics.
- `Interpolator::planned_segments`, `PlannedSegment::v_junction` and
  `jerk`; `LookAhead::velocity_limit` — a move's velocity ceiling
  independent of the programmed feed: what an override above 100 % can
  raise the move to.

Geometry and preprocessing:

- **Splines.** `Geometry::Spline` / `SplineSeg`: polynomial (Bézier)
  segments with an arc-length table, curvature-limited like arcs, usable
  everywhere a line or an arc is (look-ahead, per-axis limits, soft limits,
  reverse). `G5` programs a curve through points (see below).
- **`CornerSmoothing`** — the `G64 P…` idea: the corner between two linear
  moves is replaced by a quintic blend that stays within a tolerance of the
  programmed corner and joins both lines with continuous curvature, in any
  spatial orientation.
- **Loop removal in `ToolRadiusCorrection`**: a slot narrower than the tool,
  a step lower than its radius or an inside arc smaller than the tool is
  passed over without cutting into the contour; switch points and
  auxiliary travel of the removed elements are handed on. The module docs
  list what it does not detect (bottlenecks between distant parts of a
  contour).
- `Segment::pose_at(s)` (point and tangent in one call);
  `Geometry::max_curvature`, `reversed`, `bounding_box`, `axis_extents` and
  `for_each_extreme`; `ArcSeg::reversed`, `GeoMove::reversed`,
  `GeoElem::is_stop`.

G-code dialect (see the `decoder` module docs):

- **Variables and program flow** in the style of Fanuc Macro B: `#1 = …`,
  computed words (`X[#1 + 2]`), expressions with `+ - * / MOD`,
  comparisons, `AND OR XOR` and the usual functions; `GOTO`, `IF … GOTO`,
  `IF … THEN`, `WHILE … DO … END`; subprograms with `M98 P… K…` / `M99`.
  `DecoderConfig::variables` presets variables, `Decoder::variable` /
  `set_variable` read and write them, `DecoderConfig::max_executed_blocks`
  stops a program that never ends.
- **Work coordinate systems** `G54`…`G59` (`DecoderConfig::work_offsets`)
  and `G53` (machine coordinates for one block).
- **Tool length compensation** `G43` / `G49`
  (`DecoderConfig::tool_lengths`).
- **Working area** `G25` / `G26` (`DecoderConfig::work_area_min`/`max`,
  `Decoder::work_area`): a path element outside it is refused with its
  block.
- **`G5`** — a Catmull-Rom spline through the programmed points, with
  natural ends.
- **Chunked decoding**: `Decoder::decode_chunk` keeps the modal state (and
  the variables) from one piece of text to the next; `Decoder::resume`
  clears the program end set by `M2`/`M30`, and `flush` / `finish` close a
  program decoded block by block.

Project:

- `lib/tests/stress.rs` — random programs (short and long segments, arcs,
  splines, rapids, dwells, events, auxiliary moves, `E` limits, switch
  points, per-axis limits) under random override / halt / single-step /
  reverse actions and streamed in pieces, across several limit sets and
  cycle times. `RSCNC_STRESS_CASES` and `RSCNC_STRESS_FIRST` widen the run
  or move it to other seeds.
- Criterion benchmarks (`cargo bench -p rscnc`), cargo-fuzz targets for the
  lexer, the decoder and the offline pipeline (`fuzz/`), and a CI workflow
  (format, clippy, tests, docs, a long stress run, a fuzz smoke test).

### Fixed

Motion planning and interpolation (each pinned by a test in
`lib/tests/regression.rs`):

- **Junction velocity formula was inverted.** The junction-deviation rule
  needs sin(θ/2) of the angle *between* the segments; the code used the half
  deflection angle instead, so the gentler a corner, the slower it was taken
  (a 0.5° kink: 0.33 mm/s) while a near-reversal was taken fastest. A 720-gon
  approximating a circle crawled; it now runs in the same time as the `G3`
  circle (3270 vs 3269 cycles at 1 ms for a 180-gon of radius 20 mm). A
  polyline that samples a curve is additionally held to the centripetal
  budget of that curve (v² ≤ a_n·R with R ≈ (L₁+L₂)/2φ).
- **Reachability ignored the jerk limit.** The backward/forward passes assumed
  constant-acceleration ramps (v² = v₀² + 2·a·d), which a jerk-limited profile
  cannot follow on a short segment: a 0.5 mm stub at the end of a 50 mm run
  was entered far too fast and the profile was cut off at its end
  (17 500 mm/s² on a 500 mm/s² machine). The passes now use the exact
  jerk-limited ramp distance; `LookAhead::accel_margin` derates acceleration
  and jerk alike and is pure headroom.
- **Lowering the override could freeze the path.** A profile left above its
  new velocity limit made the trajectory generator brake through zero
  velocity: the path stood still (~170 ms in the test case) and then jumped
  back to speed. The interpolator now brakes to the new cap with a
  velocity-controlled ramp and continues from there.
- **A retarget in a profile's final ramp could pick an unreachable exit
  velocity** (override change, single-step toggle), with the same freeze-and-
  jump result. Under a jerk limit the reachable exit velocities are not an
  interval — a full stop can need less distance than slowing down a little —
  which the clamp assumed. If the new target is out of reach, the running
  profile is now kept and the change takes effect at the next span.
- **Override above 100 % exceeded the machine limits.** It scaled the
  segment's velocity cap as a whole, lifting moves past
  `PathLimits::max_velocity`, the curvature cap of arcs and the auxiliary axis
  velocity limits. It now scales the programmed feed only, up to those limits.
- **Auxiliary axis velocity jumped at junctions.** The junction velocity only
  looked at the path tangent; an auxiliary axis that reversed or started at a
  geometrically straight junction did so at full feed. Junctions are now
  evaluated on the generalized tangent including the auxiliary rates.
- **Single-step:** `step()` outside `IplState::StepHold` was remembered and
  silently released the next hold; switching single-step off during a hold
  left the interpolator stuck in it; switching it on too late to stop at the
  end of the current move cut the profile off there (it now stops at the
  first move end it can still reach).
- **A stop that ran on through several blended moves** kept reporting the
  source line / object number of the move the halt was issued in.
- **`G4` dwell could last one cycle too long** (rounding crumb of the
  repeated cycle-time subtraction; `G4 P1.1` took 1101 cycles at 1 ms).
- **`Interpolator::load` accepted moves it could not interpolate** (non-finite
  length, zero velocity cap). They are refused with the source line, and the
  previously loaded program stays intact.
- **A stop that ran into a move with another deceleration limit** (`E-`)
  kept braking with the limit of the move the halt was issued in. Running on
  into a move planned for harder braking, it overran that move's planned
  stop: the position froze at the end point and the velocity jumped to zero.
  The stop is re-planned with each move's own limit as it gets there.
- **A plan without acceleration margin could not be followed.** With
  `LookAhead::accel_margin` at 1.0 the planned ramps used every last bit of
  their spans, while the interpolator keeps a hair of slack on each profile.
  Each braking span was therefore left a trace too fast, and a halt on the
  stop ramp at the end of such a chain ran micrometres over the end. The plan
  now leaves that slack itself (2 ppm of a span's length).
- **The trajectory generator found no profile for a state exactly on the
  boundary between two profile shapes**, and the cycle failed with an error:
  a halt in the very cycle a ramp reaches its acceleration limit, or
  single-step switched on during the last jerk phase of a move that ends at a
  very low velocity. The request is repeated a hair off the boundary; a
  retarget that still finds nothing keeps the running profile, and a new
  trajectory is only ever calculated beside the running one, which a failure
  leaves intact.
- **`IplOutput::switch_points` could grow inside a cycle.** Its capacity was
  a fixed four; a cycle crossing several short moves with switch points of
  their own reallocated it. `Interpolator::load` now sizes it for the most
  switch points one cycle can cross in the loaded program.

- **A halted stop took the limits of a move it ran into one cycle late.**
  The stop is re-planned with each move's own deceleration and jerk limit;
  it did so at the end of the cycle that crossed the junction. On a plan
  without margin that cycle was enough to overrun a planned stop by a
  fraction of a micrometre (position frozen at the end point, velocity
  jumping to zero). The cycle is now split at the junction.
- **The trajectory generator found no profile for some valid requests**, and
  the cycle failed: a rest-to-rest move of exactly the length at which an
  acceleration limit is just reached (`2·a³/j²`), moves with separate
  acceleration and deceleration limits (`E`) of some thousands, and some
  re-plans from a running profile. These are fixed in rsruckig 3.1.2,
  which is now required.

Decoding and preprocessing:

- **`CornerRounding` accepted a NaN or negative radius** and wrote it into
  the arc geometry. It is refused.
- **Tool radius correction of a contour that turns straight back** (out and
  back along the same line) had no corner rule for the exact reversal; the
  two offset lines are joined by a half circle around the tip.
- **`CurvatureVelocityLimit` looked at the curvature at the two ends of an
  arc only**; it uses the highest curvature along the element (which
  matters for spiral arcs and splines).
- **A block with an invalid `T` number** was refused only after its motion
  had been put into the queue.
- **Full circle without axis words** (`G2 I-5`) was rejected, and as a modal
  follow-up block (`I-5` alone after a `G2`) silently skipped.
- **A number too long for an `f64`** (hundreds of digits) parsed to infinity
  and travelled on into the geometry. The lexer reports it as a `GcodeError`,
  as it does a block number beyond `u32`.
- **Tool radius correction dropped auxiliary-only moves** inside a `G41`/`G42`
  chain (the axis then jumped at the next move). They are kept, at the
  corrected position.
- **`CartesianKinematics::new` disagreed with `CncController::new`** about
  which machine axes are auxiliary when the path axes are not the leading
  ones (`PathAxes::Two([1, 2])`): the controller took the velocity limit from
  axis 0, the kinematics wrote the position to axis 2. Both now use the
  machine axes not carrying the path, in ascending order.

### Changed

- **Motion planning.** Consecutive moves with the same velocity cap and the
  same `E` limits whose junctions do not hold the velocity back — or hold it
  back evenly, as on a finely segmented tight curve — are planned as one
  *span* and traversed by a single jerk-limited profile, instead of one
  profile per move with zero acceleration forced at every junction. Finely
  segmented contours therefore accelerate like one long move (a 50 m contour
  of 0.25 mm segments at `F50`: simulated run time 11 170 s → 1 016 s).
  `PlannedSegment::v_exit` of a junction inside a span is the upper bound of
  the velocity there, and `PlannedSegment::v_cap` of a junction-limited span
  is that junction limit.
- **Performance** (200 000-block program, release build): lexing 247 → 127
  ns/line (numbers are parsed straight from the source text, one `Vec` per
  block instead of two plus a `String` per word), lex + decode 326 → 202
  ns/line, interpolation cycle 49–64 → 32–47 ns, `CncController::run_to_end`
  97 → 64 ns/cycle (no per-cycle output allocation). Tool radius correction
  re-scanned the whole chain for every move: a 40 000-move chain took 1.18 s
  and now takes 8 ms. The look-ahead pass does more (exact ramps, spans) and
  went from 41 to 55 ns/segment. (Measured on the 0.1.1 code base, before
  the 0.2.0 words.)
- **Dependency.** rsruckig 3.1.2 is now the minimum (was 3.0). The 3.1
  series fixes, among others, step-1 failures for states beyond the
  velocity or acceleration limits, at the borders between profile types
  and with high or asymmetric acceleration limits, and the jerk signs of
  `UDUD` profiles in the velocity interface, all of which the interpolator
  relies on.

## [0.2.0]

The DIN 66025 dialect grows the CODESYS SoftMotion words a real program uses
for dynamics and switching — and corrects the meaning of `F`. A program that
runs under `SMC_NCDecoder` now decodes here, at the same speed.

### Changed — breaking

- **`F` is units per second, not units per minute.** CODESYS SoftMotion
  documents the DIN 66025 `F` word as "Velocity (in [u/sec])", and rscnc
  divided it by 60. Every program written for a SoftMotion control therefore
  ran 60× too slow, silently. `F` (and the new `E`) are now taken as
  units/second and units/second², scaled only by the active `G70`/`G71` unit
  mode. **Programs written against 0.1.x must divide their `F` values by 60.**
- **`SyncEvent::h` is gone.** `H` is not an M-function parameter but a switch
  point on the path element, and is reported as such (see below).
- **`Word` carries a `suffix`.** `E`/`F` may take a qualifying second letter
  (`FF`, `EF`, `FA`), so the lexer's `Word` gained a field; `Word` literals
  need `suffix: None`.
- **`GeoMove` carries `accel`, `decel` and `switch_points`**, and
  `PlannedSegment` carries `accel` and `decel`; literals of either need them.

### Added

- **`E` — path acceleration and deceleration.** Positive sets the maximum path
  acceleration, negative the maximum deceleration, both in units/s² and both
  modal, exactly as CODESYS reads them (`F100 E100 E-200`). The values reach
  the look-ahead's reachability passes *and* the jerk-limited profile, where an
  asymmetric pair becomes rsruckig's `min_acceleration`.
- **`FF` / `EF` — the same two for `G0` rapids**, overriding
  `DecoderConfig::rapid_feed` when programmed.
- **`F<axis>` / `E<axis>` — one auxiliary axis' own velocity and acceleration
  limits.** An auxiliary axis moving Δ over an element of length L runs at
  `v·Δ/L`, so a programmed axis limit folds into that element's path limits at
  decode time.
- **`H` / `L` / `O` — switch points inside a path element.** `H<n>` names the
  switch function (positive on, negative off) and `L`/`O` place it: `L` is the
  distance from the element's start, or from its end when negative; `O` is the
  relative position in `[0, 1]`. Several fit in one block
  (`N100 G1 X100 H2 L20 H-2 L-10`), an `H` in a block without geometry rides on
  the next element, and one that resolves outside its element is refused with
  its source line. `GeoMove::switch_points` survives tool radius correction
  (rescaled) and corner rounding (shifted with the trim), and
  `IplOutput::switch_points` reports each one in the cycle the interpolator
  crosses it — including across a pause that stops short of the point. Steady
  state stays allocation-free: `lib/tests/allocation.rs` now runs two switch
  points per element inside the measured window.

## [0.1.1]

Four fixes found while building a real-time consumer (and a wasm bridge) on
0.1.0. All changes are additive: no existing signature, type or variant
changed, so 0.1.0 code keeps compiling.

### Fixed

- **The interpolation cycle no longer touches the heap.** `update_pose` built
  two fresh `Vec`s per cycle and `make_output` cloned both into the returned
  `IplOutput` — four allocations per cycle with auxiliary axes, plus more at
  every segment boundary (measured downstream: 8014 allocations over 2000
  cycles with one aux axis). The aux buffers are now refilled in place, sized
  once for the program's worst case in `Interpolator::load`, and handed to the
  caller's own output buffer by capacity-reusing copies. A steady-state cycle
  with auxiliary axes, segment boundaries included, now performs **zero** heap
  allocations; `lib/tests/allocation.rs` asserts this with a counting global
  allocator over 2000 consecutive cycles.
- **Decoding keeps the stage that failed.** `Decoder::decode_program`
  collapsed lexer errors into `DecodeError`, so a caller could not tell "bad
  token on line N" from "bad block on line N" without driving `lex()` and
  `decode_block()` by hand. The new `Decoder::decode_source` returns
  `CncError`, reporting a lexical failure as `CncError::Gcode` and a semantic
  one as `CncError::Decode`, each carrying its 1-based source line.
  `CncController::load_program` now reports lexer failures as `CncError::Gcode`
  as well (it previously relabelled them as decode errors).
- **No input reachable through the public API panics.** The interpolator drove
  rsruckig through `ThrowErrorHandler`; rscnc now installs its own
  non-panicking handler, so the choice cannot silently change with the
  dependency. On top of that, `Interpolator::load` validates the configuration
  before it reaches the trajectory generator and refuses a non-finite, zero or
  negative cycle time (previously accepted, then stalled at `s = 0` forever),
  the path velocity/acceleration/jerk limits, the normal-acceleration budget,
  the auxiliary axis limits and the look-ahead parameters — as
  `CncError::Interpolator`. A NaN feed override is treated as 0 (pause)
  instead of being propagated into the profile. This matters most on
  `wasm32-unknown-unknown`, where a panic is an unrecoverable trap and no
  unwinding exists.

### Added

- `Interpolator::cycle_into(&mut IplOutput)` — one control cycle into an
  existing buffer, reusing its capacity. `cycle()` is now a thin wrapper that
  allocates the buffer for you.
- `Interpolator::output_buffer()` — a per-cycle buffer sized for the loaded
  program and seeded with the start pose, so the first `cycle_into` has
  nothing left to allocate.
- `IplOutput::with_aux_capacity(n_aux)` and `impl Default for IplOutput`.
- `CncController::cycle_into(driver, &mut IplOutput)` — the same for the
  facade.
- `Interpolator::total_length()` — the planned path's total parameter length
  (the denominator behind `IplOutput::progress`).
- `Interpolator::start_position()` / `Interpolator::start_aux()` — the planned
  path's start pose, available after `load` and before the first cycle. Note
  this is the *path's* start, which preprocessing (tool radius correction
  above all) may move away from the configured machine start.
- `Interpolator::current_is_rapid()` — whether the path element the current
  output belongs to is a `G0` rapid. Exposed as an accessor rather than an
  `IplOutput` field: `IplOutput` has only public fields and is not
  `#[non_exhaustive]`, so adding a field would break struct-literal
  construction — a major change this release does not need.
- `Decoder::decode_source` (see above); `GcodeError` and `DecodeError` joined
  the prelude.

### Deprecated

- `Decoder::decode_program` — use `Decoder::decode_source`. It still works and
  still returns `DecodeError`; it just cannot tell you which stage failed.

[0.1.1]: https://github.com/petrikosk/rscnc/compare/v0.1.0...v0.1.1
