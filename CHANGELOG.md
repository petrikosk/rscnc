# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[semantic versioning](https://semver.org/) (pre-1.0: `0.1.x` stays
API-compatible).

## [0.1.1] — unreleased

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
