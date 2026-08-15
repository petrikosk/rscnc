# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[semantic versioning](https://semver.org/) (pre-1.0: `0.1.x` stays
API-compatible).

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
