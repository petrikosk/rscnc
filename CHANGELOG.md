# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[semantic versioning](https://semver.org/) (pre-1.0: a `0.x` series stays
API-compatible).

## [0.2.1] — unreleased

A review pass over the whole motion layer, on top of 0.2.0. All changes are
additive: no existing signature, type or variant changed, so 0.2.0 code keeps
compiling. The planned velocities do change — see *Motion planning* below.

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

Decoding and preprocessing:

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
- **Dependency.** rsruckig 3.1 is now the minimum (was 3.0). It fixes, among
  others, step-1 failures for states beyond the velocity or acceleration
  limits and the jerk signs of `UDUD` profiles in the velocity interface,
  both of which the interpolator relies on when it brakes down to a lowered
  override.

### Added

- `LookAhead::velocity_limit` — a move's velocity ceiling independent of the
  programmed feed (path limit, curvature cap, auxiliary axis caps): what a
  feed override above 100 % can raise the move to.
- `Segment::pose_at(s)` — point and tangent in one call (a provided method;
  lines and arcs specialize it).
- `lib/tests/stress.rs` — random programs (short and long segments, arcs,
  rapids, dwells, events, auxiliary moves, `E` limits, switch points) under
  random override / halt / single-step actions, across several limit sets
  and cycle times. `RSCNC_STRESS_CASES` and `RSCNC_STRESS_FIRST` widen the
  run or move it to other seeds.

## [0.2.0] — unreleased

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
