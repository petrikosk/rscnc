//! Decoder (the `SMC_NCDecoder` analogue): program text + modal state →
//! geometry queue.
//!
//! Supported words: `N G0 G1 G2 G3 G4 G5 G17 G18 G19 G25 G26 G40 G41 G42 G43
//! G49 G53 G54…G59 G70/G71 (G20/G21 aliases) G90 G91 G92, X Y Z I J K R F E S
//! T M H L O, P` (dwell time for G4, seconds; subprogram number for M98) plus
//! the configured auxiliary axis letters and their `F<axis>`/`E<axis>`
//! limits. `M2`/`M30` end the program, `M98`/`M99` call and leave a
//! subprogram. All other M codes become synchronized [`SyncEvent`]s; unknown
//! G codes are errors.
//!
//! Dynamics follow CODESYS SoftMotion's DIN 66025 reading: `F` is the path
//! velocity in units/**second**, `E` is the maximum path acceleration when
//! positive and the maximum deceleration when negative (units/s²), and
//! `FF`/`EF` are the same two for `G0` rapids. All four are modal.
//!
//! `H` programs a switch point on the path element, positioned by `L`
//! (distance from the element's start, or from its end when negative) or `O`
//! (relative position in `[0, 1]`); a positive `H` switches its function on, a
//! negative one off. Several fit in one block (`H2 L20 H-2 L-10`), and the
//! interpolator reports each one as it crosses it.
//!
//! Internal units are mm and seconds: axis words, `F` and `E` are scaled by
//! the active unit mode. Auxiliary axis words are *not* unit-scaled (they are
//! typically rotary degrees), and neither are `F<axis>`/`E<axis>`.
//!
//! # Coordinate systems
//!
//! The queue's geometry is in machine coordinates. A programmed coordinate
//! becomes a machine coordinate as
//!
//! ```text
//! machine = programmed + work offset − G92 shift (+ tool length, Z only)
//! ```
//!
//! * `G54`…`G59` (modal, `G54` at start) select the work offset from
//!   [`DecoderConfig::work_offsets`]. All zeros by default, so a program that
//!   never leaves `G54` is programmed in machine coordinates.
//! * `G92 X.. Y.. Z..` gives the current point the programmed coordinates
//!   written in the block. Its shift lies on top of the work offset and stays
//!   in force when another work offset is selected.
//! * `G53` (one block only) reads the block's `X Y Z` and auxiliary axis
//!   words as absolute machine coordinates: no work offset, no `G92` shift,
//!   no tool length, also under `G91`.
//! * `G43` switches tool length compensation on, `G49` off. `G43` latches the
//!   length of the active tool — the `T` word of the same block if it has
//!   one, else the last `T` programmed — from [`DecoderConfig::tool_lengths`];
//!   a later `T` word does not change it until the next `G43`. The length is
//!   added to programmed absolute Z coordinates, so neither code moves the
//!   axis by itself: the change takes effect with the next block that
//!   programs Z.
//!
//! # Working area
//!
//! `G25 X.. Y.. Z..` sets the lower corner of the working area and
//! `G26 X.. Y.. Z..` the upper one, for the axes written (programmed
//! coordinates of the active system, or machine coordinates with `G53`); the
//! block does not move. Every path element decoded afterwards must lie
//! inside the area — an arc's bulge and a spline's sweep included — or its
//! block is refused with an error. [`DecoderConfig::work_area_min`] and
//! [`DecoderConfig::work_area_max`] give the area at program start.
//!
//! # Splines
//!
//! `G5` (modal) makes the programmed points the points of a spline: the
//! curve starts at the current position and passes through every point of the
//! `G5` blocks that follow, with a continuous tangent. The run ends with the
//! first block that selects another motion mode, or carries `G4`, `G92`,
//! `G25`/`G26` or an event (`M`, `S`, `T`; the block's own point still
//! belongs to the run), and at the end of the program text. Each span between
//! two points becomes one [`Geometry::Spline`] element with the feed, limits,
//! auxiliary axis targets and switch points of the block that programmed its
//! end point.
//!
//! The curve is a Catmull-Rom spline with chord-length parameterization:
//! the tangent at an inner point is the average of the two chord directions
//! there, the shorter chord counting more (for evenly spaced points this is
//! the classic (pᵢ₊₁ − pᵢ₋₁)/2). The two ends are *natural*: the curve
//! starts and ends with zero curvature. A run of a single point is a
//! straight line; a point repeated in place is skipped. Points that turn
//! straight back on the way they came (`X10`, then `X0`) are refused, since
//! no smooth curve does that without standing still.
//!
//! # Variables and program flow
//!
//! In the style of Fanuc Macro B:
//!
//! | Form                              | Meaning                               |
//! |-----------------------------------|---------------------------------------|
//! | `#1 = <expr>`                     | assignment (several fit in a line)    |
//! | `X#1`, `X-#1`, `X[#1 + 2]`        | a word whose value is computed        |
//! | `GOTO n`                          | jump to the block numbered `N n`      |
//! | `IF [<expr>] GOTO n`              | conditional jump                      |
//! | `IF [<expr>] THEN #1 = <expr>`    | conditional assignment                |
//! | `WHILE [<expr>] DO m` … `END m`   | loop (nestable)                       |
//! | `M98 P<n> K<count>`               | call the subprogram at block `N<n>`, `count` times (once without `K`) |
//! | `M99`                             | return from the subprogram            |
//!
//! Expressions know `+ - * /` and `MOD`, the comparisons `EQ NE GT GE LT LE`
//! (1 when true, 0 when false), the logical `AND OR XOR` (anything non-zero
//! is true) and the functions `SIN COS TAN ASIN ACOS ATAN SQRT ABS ROUND FIX
//! FUP`, angles in degrees; `ATAN[y]/[x]` is the two-argument arc tangent.
//! Brackets group, since parentheses are comments. Precedence, tightest
//! first: unary sign, `* / MOD`, `+ -`, comparisons, `AND OR XOR`.
//! `#[<expr>]` addresses a variable by a computed number.
//!
//! Variables live in the [`Decoder`] and keep their values from one
//! [`decode_chunk`](Decoder::decode_chunk) to the next. Reading a variable
//! that was never assigned is an error, as are a division by zero and a
//! jump to a block number that does not exist (the first block with the
//! number counts when several have it). A subprogram is part of the same
//! program text — put it after the `M2`/`M30` of the main program — and
//! `M98`/`M99` stand alone in their blocks. Calls nest
//! [`MAX_CALL_DEPTH`] deep. A program that keeps jumping is stopped with an
//! error after [`DecoderConfig::max_executed_blocks`].
//!
//! [`Decoder::decode_block`] sees none of this: it takes one plain, already
//! lexed block. The statements above are handled by
//! [`Decoder::decode_chunk`] and [`Decoder::decode_source`].

use crate::error::{CncError, DecodeError};
use crate::gcode::lexer::ParsedBlock;
use crate::gcode::macros::{self, Assign, Expr, Line, Stmt, Then, Variables};
use crate::gcode::program::{DistanceMode, ModalState, MotionMode, TrcMode, Units};
use crate::geometry::{
    dot, norm, sub, ArcSeg, GeoElem, GeoMove, Geometry, LineSeg, Segment, SplineSeg, SwitchPoint,
    SyncEvent,
};
use crate::queue::GeoQueue;
use crate::types::{PathPoint, WorkPlane};
use std::collections::HashMap;

/// How deep subprogram calls (`M98`) may nest.
pub const MAX_CALL_DEPTH: usize = 16;

/// Slack of the working-area check, internal units.
const AREA_TOLERANCE: f64 = 1e-9;

const AXIS_NAMES: [char; 3] = ['X', 'Y', 'Z'];

/// Decoder configuration: machine start pose, auxiliary axis letters,
/// coordinate systems, tool table, working area and tolerances.
#[derive(Debug, Clone)]
pub struct DecoderConfig {
    /// Letters of the auxiliary (non-path) axes, in machine order.
    pub aux_letters: Vec<char>,
    /// Feed used for G0 rapids, internal units/s.
    pub rapid_feed: f64,
    /// Allowed start/end radius mismatch for IJK arcs, internal units.
    pub radius_tolerance: f64,
    /// Moves shorter than this are elided.
    pub min_segment_length: f64,
    /// Machine position of the path axes at program start.
    pub start_position: [f64; 3],
    /// Machine position of the auxiliary axes at program start.
    pub start_aux: Vec<f64>,
    /// M functions the path does not stop for (*fly-by* M functions). A
    /// block's event is a fly-by event when all its M codes are listed here
    /// and it carries no tool change (`T`); the interpolator then reports it
    /// while the path runs through the junction. All other events force an
    /// exact stop, as before.
    pub fly_by_m: Vec<u32>,
    /// Work coordinate systems `G54`…`G59`: the machine coordinates of each
    /// system's zero point, internal units. All zeros by default.
    pub work_offsets: [[f64; 3]; 6],
    /// Tool table for `G43`: (tool number, tool length in internal units).
    /// `G43` with a tool that has no entry is an error. Empty by default.
    pub tool_lengths: Vec<(u32, f64)>,
    /// Lower corner of the working area at program start, machine
    /// coordinates; `-∞` leaves an axis unlimited (the default). `G25`
    /// changes it from the program.
    pub work_area_min: [f64; 3],
    /// Upper corner of the working area at program start, machine
    /// coordinates; `+∞` leaves an axis unlimited (the default). `G26`
    /// changes it from the program.
    pub work_area_max: [f64; 3],
    /// Initial values of variables: (variable number, value). Empty by
    /// default.
    pub variables: Vec<(u32, f64)>,
    /// Guard against programs that never end: decoding one program text
    /// stops with an error once it has executed this many blocks more than
    /// the text has lines (a text without jumps and loops never does).
    /// 1 000 000 by default.
    pub max_executed_blocks: usize,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            aux_letters: vec![],
            rapid_feed: 100.0,
            radius_tolerance: 0.01,
            min_segment_length: 1e-9,
            start_position: [0.0; 3],
            start_aux: vec![],
            fly_by_m: vec![],
            work_offsets: [[0.0; 3]; 6],
            tool_lengths: vec![],
            work_area_min: [f64::NEG_INFINITY; 3],
            work_area_max: [f64::INFINITY; 3],
            variables: vec![],
            max_executed_blocks: 1_000_000,
        }
    }
}

/// The DIN 66025 decoder: turns program text into geometry queue elements
/// while tracking modal state. Use [`Decoder::decode_source`] for the
/// common whole-program case and [`Decoder::decode_chunk`] to feed a program
/// piece by piece.
#[derive(Debug, Clone)]
pub struct Decoder {
    cfg: DecoderConfig,
    state: ModalState,
    obj_no: u32,
    ended: bool,
    /// Switch points programmed in blocks that produced no path element of
    /// their own; they ride on the next element that is created.
    pending_switches: Vec<PendingSwitch>,
    variables: Variables,
    /// Working area (machine coordinates).
    area_min: [f64; 3],
    area_max: [f64; 3],
    /// `G5` points waiting for the end of their run.
    spline: Option<SplineRun>,
}

/// Where an `L`/`O` word puts a switch point, in programmed units.
#[derive(Debug, Clone, Copy, PartialEq)]
enum SwitchPos {
    /// `L`: distance from the element's start, or from its end when negative.
    Absolute(f64),
    /// `O`: relative position in `[0, 1]`.
    Relative(f64),
}

/// An `H` word waiting for the path element it belongs to.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PendingSwitch {
    number: i32,
    pos: Option<SwitchPos>,
    src_line: usize,
}

/// A run of `G5` points that is still open.
#[derive(Debug, Clone)]
struct SplineRun {
    /// Where the curve starts (machine coordinates).
    start: PathPoint,
    aux_start: Vec<f64>,
    points: Vec<SplinePoint>,
}

/// One `G5` point with everything its span takes from its block.
#[derive(Debug, Clone)]
struct SplinePoint {
    target: PathPoint,
    aux_end: Vec<f64>,
    feed: f64,
    accel: Option<f64>,
    decel: Option<f64>,
    trc: TrcMode,
    line: usize,
    /// Unit scale of the block (for `L` words).
    scale: f64,
    switches: Vec<PendingSwitch>,
    aux_feed: Vec<Option<f64>>,
    aux_accel: Vec<Option<f64>>,
    aux_decel: Vec<Option<f64>>,
}

/// A subprogram call in progress.
#[derive(Debug, Clone, Copy)]
struct Call {
    /// Line index to continue at after the last return.
    ret: usize,
    /// Line index of the subprogram's first block.
    target: usize,
    /// Runs left, this one included.
    remaining: u32,
    /// Source line of the `M98`.
    src_line: usize,
}

/// What a program text's jumps need while it runs.
#[derive(Debug, Default)]
struct Flow {
    /// Block number → line index, built when the first jump needs it.
    labels: Option<HashMap<u32, usize>>,
    calls: Vec<Call>,
}

impl Flow {
    /// Line index of the first block numbered `number`.
    fn label(&mut self, number: u32, lines: &[Line]) -> Option<usize> {
        self.labels
            .get_or_insert_with(|| {
                let mut map = HashMap::new();
                for (i, line) in lines.iter().enumerate() {
                    if let Some(n) = line.number {
                        map.entry(n).or_insert(i);
                    }
                }
                map
            })
            .get(&number)
            .copied()
    }
}

/// Which corner of the working area a block sets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Corner {
    /// `G25`
    Lower,
    /// `G26`
    Upper,
}

/// `value` as a whole number in `0..=u32::MAX`.
fn whole(value: f64) -> Option<u32> {
    (value >= 0.0 && value.fract() == 0.0 && value <= f64::from(u32::MAX)).then_some(value as u32)
}

impl Decoder {
    /// New decoder starting from the configured machine pose with default
    /// modal state.
    pub fn new(cfg: DecoderConfig) -> Self {
        let mut state = ModalState::new(cfg.aux_letters.len());
        state.position = cfg.start_position;
        for (i, v) in cfg.start_aux.iter().enumerate() {
            if i < state.aux_position.len() {
                state.aux_position[i] = *v;
            }
        }
        Self {
            variables: cfg.variables.iter().copied().collect(),
            area_min: cfg.work_area_min,
            area_max: cfg.work_area_max,
            cfg,
            state,
            obj_no: 0,
            ended: false,
            pending_switches: Vec::new(),
            spline: None,
        }
    }

    /// Convenience: lex + decode a whole program, keeping the pipeline stage
    /// that failed.
    ///
    /// A lexical problem comes back as [`CncError::Gcode`] and a semantic one
    /// as [`CncError::Decode`], each carrying the 1-based source line, so a
    /// caller can tell "bad token on line N" from "bad block on line N"
    /// without driving the lexer and [`decode_block`](Self::decode_block) by
    /// hand.
    ///
    /// The same as [`Decoder::new`], one [`decode_chunk`](Self::decode_chunk)
    /// and [`finish`](Self::finish).
    ///
    /// ```
    /// use rscnc::prelude::*;
    ///
    /// let err = Decoder::decode_source(DecoderConfig::default(), "G1 F600 X1\nG1 X#\n")
    ///     .unwrap_err();
    /// assert!(matches!(err, CncError::Gcode(ref e) if e.line == 2));
    /// ```
    pub fn decode_source(cfg: DecoderConfig, source: &str) -> Result<GeoQueue, CncError> {
        let mut decoder = Decoder::new(cfg);
        let queue = decoder.decode_chunk(source)?;
        decoder.finish()?;
        Ok(queue)
    }

    /// Decode one piece of program text, keeping everything modal — motion
    /// mode, feed, position, offsets, working area, variables — for the
    /// piece that follows. This is how a program is fed in parts (streaming,
    /// MDI): every call returns the path elements of its own text.
    ///
    /// What does not reach across the seam between two pieces:
    ///
    /// * jump targets: `GOTO`, `M98` and `WHILE`/`END` pairs are resolved
    ///   inside one piece of text, and a subprogram has to return in it;
    /// * a `G5` spline: the run is closed at the end of the text (`G5` itself
    ///   stays modal, so the next piece starts a new curve).
    ///
    /// A switch point (`H`) programmed after the last move of the text is
    /// kept and rides on the first element of the next piece;
    /// [`finish`](Self::finish) reports one that is left over at the very end.
    ///
    /// After `M2`/`M30` further text is ignored (an empty queue is returned)
    /// until [`resume`](Self::resume).
    ///
    /// The call is all or nothing: on an error the decoder is exactly as it
    /// was before the call, so a corrected text can be decoded in its place.
    /// Line numbers in errors count from the start of `source`.
    ///
    /// ```
    /// use rscnc::prelude::*;
    ///
    /// let mut decoder = Decoder::new(DecoderConfig::default());
    /// let first = decoder.decode_chunk("#1 = 10\nG1 F50 X#1\n")?;
    /// let second = decoder.decode_chunk("Y[#1 / 2]\n")?; // still G1 F50, #1 known
    /// decoder.finish()?;
    /// assert_eq!((first.len(), second.len()), (1, 1));
    /// # Ok::<(), CncError>(())
    /// ```
    pub fn decode_chunk(&mut self, source: &str) -> Result<GeoQueue, CncError> {
        if self.ended {
            return Ok(GeoQueue::new());
        }
        let mut lines = macros::parse_program(source)?;
        let backup = self.clone();
        match self.run(&mut lines) {
            Ok(queue) => Ok(queue),
            Err(e) => {
                *self = backup;
                Err(e.into())
            }
        }
    }

    /// End-of-program check: a switch point (`H`) still waiting for a path
    /// element to ride on is an error. Users of
    /// [`decode_block`](Self::decode_block) call [`flush`](Self::flush) first.
    pub fn finish(&self) -> Result<(), DecodeError> {
        match self.pending_switches.first() {
            Some(p) => Err(DecodeError::new(
                p.src_line,
                format!(
                    "switch point H{} has no path element to ride on (no move follows it)",
                    p.number
                ),
            )),
            None => Ok(()),
        }
    }

    /// Convenience: lex + decode a whole program, reporting every failure as
    /// a [`DecodeError`].
    ///
    /// This flattens lexer errors into decode errors and so loses the stage
    /// distinction; use [`decode_source`](Self::decode_source) instead.
    #[deprecated(
        since = "0.1.1",
        note = "use `Decoder::decode_source`, which preserves the lex/decode stage distinction"
    )]
    pub fn decode_program(cfg: DecoderConfig, source: &str) -> Result<GeoQueue, DecodeError> {
        Self::decode_source(cfg, source).map_err(|e| match e {
            CncError::Decode(e) => e,
            CncError::Gcode(e) => DecodeError::new(e.line, e.message),
            other => DecodeError::new(0, other.to_string()),
        })
    }

    /// Current modal state (mainly for tests and diagnostics).
    pub fn modal_state(&self) -> &ModalState {
        &self.state
    }

    /// True after M2/M30 was decoded.
    pub fn program_ended(&self) -> bool {
        self.ended
    }

    /// Clear the program-end flag set by `M2`/`M30`, so that further text is
    /// decoded again. The modal state and the variables are kept.
    pub fn resume(&mut self) {
        self.ended = false;
    }

    /// Value of variable `#number`, or `None` if it was never assigned.
    pub fn variable(&self, number: u32) -> Option<f64> {
        self.variables.get(&number).copied()
    }

    /// Assign variable `#number` from outside the program.
    pub fn set_variable(&mut self, number: u32, value: f64) {
        self.variables.insert(number, value);
    }

    /// The working area in force: (lower corner, upper corner) in machine
    /// coordinates, `±∞` where an axis is unlimited.
    pub fn work_area(&self) -> ([f64; 3], [f64; 3]) {
        (self.area_min, self.area_max)
    }

    /// Run the lines of one program text.
    fn run(&mut self, lines: &mut [Line]) -> Result<GeoQueue, DecodeError> {
        macros::link_loops(lines).map_err(|(line, msg)| DecodeError::new(line, msg))?;
        let lines = &*lines;
        let mut out = GeoQueue::new();
        let mut flow = Flow::default();
        let budget = lines.len().saturating_add(self.cfg.max_executed_blocks);
        let mut executed = 0usize;
        let mut pc = 0;
        while let Some(line) = lines.get(pc) {
            if self.ended {
                break;
            }
            let at = line.src_line;
            let fail = |msg: String| DecodeError::new(at, msg);
            executed += 1;
            if executed > budget {
                return Err(fail(format!(
                    "the program does not end: it ran {} blocks more than its {} lines \
                     (an endless loop? see DecoderConfig::max_executed_blocks)",
                    self.cfg.max_executed_blocks,
                    lines.len()
                )));
            }
            let mut next = pc + 1;
            match &line.stmt {
                Stmt::Plain(block) => {
                    if let Some(to) = self.block(block, pc, lines, &mut flow, &mut out)? {
                        next = to;
                    }
                }
                Stmt::Block(words) => {
                    let block = macros::evaluate_block(words, line.number, at, &self.variables)
                        .map_err(fail)?;
                    if let Some(to) = self.block(&block, pc, lines, &mut flow, &mut out)? {
                        next = to;
                    }
                }
                Stmt::Assign(list) => self.assign(list).map_err(fail)?,
                Stmt::Goto(target) => {
                    next = self.jump_target(target, lines, &mut flow).map_err(fail)?;
                }
                Stmt::If { cond, then } => {
                    if cond.eval(&self.variables).map_err(fail)? != 0.0 {
                        match then {
                            Then::Goto(target) => {
                                next = self.jump_target(target, lines, &mut flow).map_err(fail)?;
                            }
                            Then::Assign(list) => self.assign(list).map_err(fail)?,
                        }
                    }
                }
                Stmt::While { cond, end, .. } => {
                    if cond.eval(&self.variables).map_err(fail)? == 0.0 {
                        next = end + 1;
                    }
                }
                Stmt::End { start, .. } => next = *start,
            }
            pc = next;
        }
        if !self.ended {
            if let Some(call) = flow.calls.last() {
                return Err(DecodeError::new(
                    call.src_line,
                    "the subprogram called here does not return (no M99 before the end of \
                     the program text)",
                ));
            }
        }
        self.flush(&mut out)?;
        Ok(out)
    }

    fn assign(&mut self, list: &[Assign]) -> Result<(), String> {
        for a in list {
            let number = macros::variable_number(a.target.eval(&self.variables)?)?;
            let value = a.value.eval(&self.variables)?;
            self.variables.insert(number, value);
        }
        Ok(())
    }

    /// Line index of the block a jump goes to.
    fn jump_target(&self, target: &Expr, lines: &[Line], flow: &mut Flow) -> Result<usize, String> {
        let value = target.eval(&self.variables)?;
        let number =
            whole(value).ok_or_else(|| format!("{value} is not a block number to jump to"))?;
        flow.label(number, lines)
            .ok_or_else(|| format!("jump to N{number}, but no block has that number"))
    }

    /// Execute one plain block: a subprogram call or return, or a block for
    /// [`decode_block`](Self::decode_block). Returns the line to continue at
    /// when that is not the next one.
    fn block(
        &mut self,
        block: &ParsedBlock,
        pc: usize,
        lines: &[Line],
        flow: &mut Flow,
        out: &mut GeoQueue,
    ) -> Result<Option<usize>, DecodeError> {
        let has_m = |code: f64| {
            block
                .words
                .iter()
                .any(|w| w.letter == 'M' && w.suffix.is_none() && w.value == code)
        };
        let err = |msg: String| DecodeError::new(block.src_line, msg);
        if has_m(98.0) {
            let (mut target, mut count) = (None, 1);
            for w in &block.words {
                match (w.letter, w.suffix) {
                    ('M', None) if w.value == 98.0 => {}
                    ('P', None) => {
                        target = Some(whole(w.value).ok_or_else(|| {
                            err(format!("M98: P{} is not a block number", w.value))
                        })?);
                    }
                    ('K', None) => {
                        count = whole(w.value).ok_or_else(|| {
                            err(format!("M98: K{} is not a repeat count", w.value))
                        })?;
                    }
                    _ => {
                        return Err(err(format!(
                            "M98 stands alone with its P (and K) word, found '{}'",
                            w.name()
                        )))
                    }
                }
            }
            let number =
                target.ok_or_else(|| err("M98 requires a P word (the block number)".into()))?;
            let target = flow
                .label(number, lines)
                .ok_or_else(|| err(format!("M98 P{number}: no block is numbered N{number}")))?;
            if count == 0 {
                return Ok(None);
            }
            if flow.calls.len() >= MAX_CALL_DEPTH {
                return Err(err(format!(
                    "subprogram calls nested deeper than {MAX_CALL_DEPTH}"
                )));
            }
            flow.calls.push(Call {
                ret: pc + 1,
                target,
                remaining: count,
                src_line: block.src_line,
            });
            Ok(Some(target))
        } else if has_m(99.0) {
            if block.words.len() != 1 {
                return Err(err("M99 stands alone in its block".into()));
            }
            match flow.calls.last_mut() {
                Some(call) if call.remaining > 1 => {
                    call.remaining -= 1;
                    Ok(Some(call.target))
                }
                Some(call) => {
                    let ret = call.ret;
                    flow.calls.pop();
                    Ok(Some(ret))
                }
                None => Err(err(
                    "M99 outside a subprogram call (end the main program with M2/M30 before \
                     its subprograms)"
                        .into(),
                )),
            }
        } else {
            self.decode_block(block, out)?;
            Ok(None)
        }
    }

    /// Decode one plain block, appending 0..k elements to `out`.
    ///
    /// This is the layer below [`decode_chunk`](Self::decode_chunk): no
    /// variables, no jumps, and `M98`/`M99` are ordinary M functions here.
    /// `G5` points are held back until their run ends, so call
    /// [`flush`](Self::flush) after the last block.
    pub fn decode_block(
        &mut self,
        block: &ParsedBlock,
        out: &mut GeoQueue,
    ) -> Result<(), DecodeError> {
        if self.ended {
            return Ok(());
        }
        let line = block.src_line;
        let err = |msg: String| DecodeError::new(line, msg);

        // Collect the block's words by letter
        let mut g_codes: Vec<i32> = Vec::new();
        let mut m_codes: Vec<u32> = Vec::new();
        let mut axis: [Option<f64>; 3] = [None; 3];
        let mut aux: Vec<Option<f64>> = vec![None; self.cfg.aux_letters.len()];
        let mut ijk: [Option<f64>; 3] = [None; 3];
        let (mut r_word, mut f_word, mut s_word, mut t_word, mut p_word, mut ff_word) =
            (None, None, None, None, None, None);
        // E / EF values in programmed order, still in programmed units.
        let mut e_words: Vec<f64> = Vec::new();
        let mut ef_words: Vec<f64> = Vec::new();
        // `H` words in programmed order, each with the `L`/`O` word that
        // positions it; resolved against the element length below.
        let mut switches: Vec<PendingSwitch> = Vec::new();

        for w in &block.words {
            match (w.letter, w.suffix) {
                ('G', None) => {
                    if w.value.fract() != 0.0 {
                        return Err(err(format!("unsupported decimal G code G{}", w.value)));
                    }
                    g_codes.push(w.value as i32);
                }
                ('M', None) => {
                    if w.value.fract() != 0.0 || w.value < 0.0 {
                        return Err(err(format!("invalid M code M{}", w.value)));
                    }
                    m_codes.push(w.value as u32);
                }
                ('X', None) => axis[0] = Some(w.value),
                ('Y', None) => axis[1] = Some(w.value),
                ('Z', None) => axis[2] = Some(w.value),
                ('I', None) => ijk[0] = Some(w.value),
                ('J', None) => ijk[1] = Some(w.value),
                ('K', None) => ijk[2] = Some(w.value),
                ('R', None) => r_word = Some(w.value),
                ('F', None) => f_word = Some(w.value),
                ('S', None) => s_word = Some(w.value),
                ('T', None) => t_word = Some(w.value),
                ('P', None) => p_word = Some(w.value),
                // E: maximum path acceleration (> 0) / deceleration (< 0).
                // Scaled with the block's unit mode, so kept raw until the
                // G70/G71 of this same block has been applied.
                ('E', None) => e_words.push(w.value),
                // FF / EF: the same for G0 rapids.
                ('F', Some('F')) => {
                    if w.value < 0.0 {
                        return Err(err(format!("negative rapid feed FF{}", w.value)));
                    }
                    ff_word = Some(w.value);
                }
                ('E', Some('F')) => ef_words.push(w.value),
                // F<axis> / E<axis>: one auxiliary axis' own limits. Auxiliary
                // axes carry their own units (typically rotary degrees), so
                // these are not unit-scaled.
                ('F' | 'E', Some(a)) => {
                    let name = w.name();
                    let Some(i) = self.cfg.aux_letters.iter().position(|&l| l == a) else {
                        return Err(err(format!(
                            "word '{name}' names auxiliary axis '{a}', which this machine \
                             does not have"
                        )));
                    };
                    if w.letter == 'F' {
                        if w.value < 0.0 {
                            return Err(err(format!("negative feed {name}{}", w.value)));
                        }
                        self.state.aux_feed[i] = Some(w.value);
                    } else if w.value >= 0.0 {
                        self.state.aux_accel[i] = Some(w.value);
                    } else {
                        self.state.aux_decel[i] = Some(-w.value);
                    }
                }
                // H / L / O: a switch point and its position on the element.
                ('H', None) => {
                    if w.value.fract() != 0.0 || w.value == 0.0 {
                        return Err(err(format!(
                            "invalid switch point H{} (expected a non-zero whole number)",
                            w.value
                        )));
                    }
                    switches.push(PendingSwitch {
                        number: w.value as i32,
                        pos: None,
                        src_line: line,
                    });
                }
                ('L' | 'O', None) => {
                    let pos = if w.letter == 'L' {
                        SwitchPos::Absolute(w.value)
                    } else {
                        if !(0.0..=1.0).contains(&w.value) {
                            return Err(err(format!(
                                "switch point position O{} is outside [0, 1]",
                                w.value
                            )));
                        }
                        SwitchPos::Relative(w.value)
                    };
                    match switches.last_mut() {
                        Some(PendingSwitch {
                            pos: slot @ None, ..
                        }) => *slot = Some(pos),
                        _ => {
                            return Err(err(format!(
                                "{}{} has no switch point to position (an H word must come first)",
                                w.letter, w.value
                            )))
                        }
                    }
                }
                (c, None) => {
                    if let Some(i) = self.cfg.aux_letters.iter().position(|&l| l == c) {
                        aux[i] = Some(w.value);
                    } else {
                        return Err(err(format!(
                            "unsupported word letter '{c}' (this dialect knows \
                             N G M X Y Z I J K R F E H L O P S T{}{})",
                            if self.cfg.aux_letters.is_empty() {
                                ""
                            } else {
                                " and the auxiliary axes "
                            },
                            self.cfg
                                .aux_letters
                                .iter()
                                .map(char::to_string)
                                .collect::<Vec<_>>()
                                .join(" ")
                        )));
                    }
                }
                (c, Some(s)) => {
                    return Err(err(format!(
                        "unsupported word '{c}{s}' (only FF, EF, F<axis> and E<axis> \
                         take a second letter)"
                    )))
                }
            }
        }

        // Apply non-motion modal G codes first (units before axis-word scaling)
        let mut dwell = false;
        let mut set_position = false;
        // G53: this block's coordinates are machine coordinates
        let mut machine = false;
        // G25 / G26: the block sets a corner of the working area
        let mut area: Option<Corner> = None;
        // G43: latch the active tool's length
        let mut length_comp = false;
        for &g in &g_codes {
            match g {
                0 => self.state.motion = MotionMode::Rapid,
                1 => self.state.motion = MotionMode::Linear,
                2 => self.state.motion = MotionMode::ArcCw,
                3 => self.state.motion = MotionMode::ArcCcw,
                4 => dwell = true,
                5 => self.state.motion = MotionMode::Spline,
                17 => self.state.plane = WorkPlane::Xy,
                18 => self.state.plane = WorkPlane::Zx,
                19 => self.state.plane = WorkPlane::Yz,
                25 | 26 => {
                    if area.is_some() {
                        return Err(err("G25 and G26 need a block each".into()));
                    }
                    area = Some(if g == 25 {
                        Corner::Lower
                    } else {
                        Corner::Upper
                    });
                }
                40 => self.state.trc = TrcMode::Off,
                41 => self.state.trc = TrcMode::Left,
                42 => self.state.trc = TrcMode::Right,
                43 => length_comp = true,
                49 => {
                    length_comp = false;
                    self.state.tool_length = 0.0;
                }
                53 => machine = true,
                54..=59 => self.state.work_offset = (g - 54) as usize,
                70 | 20 => self.state.units = Units::Inch,
                71 | 21 => self.state.units = Units::Mm,
                90 => self.state.distance = DistanceMode::Absolute,
                91 => self.state.distance = DistanceMode::Incremental,
                92 => set_position = true,
                other => return Err(err(format!("unsupported G code G{other}"))),
            }
        }
        if area.is_some() && (dwell || set_position) {
            return Err(err("G25/G26 cannot share a block with G4 or G92".into()));
        }

        let scale = self.state.units.to_mm();
        if let Some(f) = f_word {
            if f < 0.0 {
                return Err(err(format!("negative feed F{f}")));
            }
            self.state.feed = f * scale;
        }
        if let Some(f) = ff_word {
            self.state.rapid_feed = Some(f * scale);
        }
        // A positive E sets the acceleration, a negative one the deceleration;
        // a block may carry one of each (`F100 E100 E-200`).
        for (name, values) in [("E", &e_words), ("EF", &ef_words)] {
            for &e in values {
                if e == 0.0 {
                    return Err(err(format!(
                        "{name}0 is not a limit (E sets acceleration when positive and \
                         deceleration when negative)"
                    )));
                }
                let (accel, decel) = match name {
                    "E" => (&mut self.state.accel, &mut self.state.decel),
                    _ => (&mut self.state.rapid_accel, &mut self.state.rapid_decel),
                };
                if e > 0.0 {
                    *accel = Some(e * scale);
                } else {
                    *decel = Some(-e * scale);
                }
            }
        }

        let tool = match t_word {
            Some(t) => Some(whole(t).ok_or_else(|| err(format!("invalid tool number T{t}")))?),
            None => None,
        };
        if length_comp {
            // The block's own T word names the tool whose length is meant.
            let active = tool.unwrap_or(self.state.tool);
            let Some(&(_, length)) = self.cfg.tool_lengths.iter().find(|(n, _)| *n == active)
            else {
                return Err(err(format!(
                    "G43: tool T{active} has no entry in the tool length table \
                     (DecoderConfig::tool_lengths)"
                )));
            };
            self.state.tool_length = length;
        }

        let has_axis = axis.iter().any(Option::is_some) || aux.iter().any(Option::is_some);
        let arc_data = r_word.is_some() || ijk.iter().any(Option::is_some);
        // A spline run goes on only through blocks that stay in G5 and do
        // nothing but add a point or set something modal.
        if self.state.motion != MotionMode::Spline || dwell || set_position || area.is_some() {
            self.flush(out)?;
        }

        if dwell {
            let seconds = p_word.ok_or_else(|| err("G4 requires a P word (seconds)".into()))?;
            if seconds < 0.0 || !seconds.is_finite() {
                return Err(err(format!("invalid dwell time P{seconds}")));
            }
            out.push(GeoElem::Dwell {
                seconds,
                src_line: line,
            });
        } else if let Some(corner) = area {
            if aux.iter().any(Option::is_some) {
                return Err(err(
                    "G25/G26 limit the path axes X Y Z only (auxiliary axis word found)".into(),
                ));
            }
            if axis.iter().all(Option::is_none) {
                return Err(err("G25/G26 require at least one of X Y Z".into()));
            }
            // Always absolute: a limit is a place, not a distance.
            for (i, v) in axis.iter().enumerate() {
                let Some(v) = v else { continue };
                let limit = if machine {
                    v * scale
                } else {
                    self.to_machine(i, v * scale)
                };
                match corner {
                    Corner::Lower => self.area_min[i] = limit,
                    Corner::Upper => self.area_max[i] = limit,
                }
            }
        } else if set_position {
            // G92: the current point gets the given programmed coordinates,
            // i.e. `to_machine` of them is the current position from here on.
            for (i, v) in axis.iter().enumerate() {
                if let Some(v) = v {
                    self.state.g92_offset[i] =
                        v * scale + self.system_offset(i) - self.state.position[i];
                }
            }
            for (i, v) in aux.iter().enumerate() {
                if let Some(v) = v {
                    self.state.aux_g92_offset[i] = v - self.state.aux_position[i];
                }
            }
        } else if has_axis {
            self.pending_switches.append(&mut switches);
            if self.state.motion == MotionMode::Spline {
                if arc_data {
                    return Err(err("a G5 spline point takes no I/J/K or R word".into()));
                }
                self.spline_point(line, &axis, &aux, machine, out)?;
            } else {
                self.decode_motion(line, &axis, &aux, &ijk, r_word, machine, out)?;
            }
        } else if matches!(self.state.motion, MotionMode::ArcCw | MotionMode::ArcCcw) && arc_data {
            // Arc data without axis words: the end point is the start point,
            // i.e. a full circle (`G2 I-5`), also on a modal follow-up block.
            self.pending_switches.append(&mut switches);
            self.decode_motion(line, &axis, &aux, &ijk, r_word, machine, out)?;
        } else if g_codes.contains(&2) || g_codes.contains(&3) {
            return Err(err("arc move without axis words or I/J/K".into()));
        } else {
            // A block with no path element of its own: its switch points ride
            // on the next element, at its start.
            self.pending_switches.append(&mut switches);
        }

        let has_event = !m_codes.is_empty() || s_word.is_some() || tool.is_some();
        if has_event {
            // An event (or the program end) closes a spline run, after the
            // block's own point.
            self.flush(out)?;
        }

        // Program end
        if m_codes.iter().any(|&m| m == 2 || m == 30) {
            self.ended = true;
            m_codes.retain(|&m| m != 2 && m != 30);
        }

        // Synchronized events after the block's motion (CoDeSys default)
        if !m_codes.is_empty() || s_word.is_some() || tool.is_some() {
            if let Some(s) = s_word {
                self.state.spindle_speed = s;
            }
            if let Some(t) = tool {
                self.state.tool = t;
            }
            let fly_by = !m_codes.is_empty()
                && tool.is_none()
                && m_codes.iter().all(|m| self.cfg.fly_by_m.contains(m));
            out.push(GeoElem::Event(SyncEvent {
                m: m_codes,
                spindle: s_word,
                tool,
                src_line: line,
                fly_by,
            }));
        }

        Ok(())
    }

    /// What the active coordinate system adds to a programmed coordinate of
    /// path axis `i`, the `G92` shift aside: the work offset, and on Z the
    /// tool length.
    fn system_offset(&self, i: usize) -> f64 {
        let tool = if i == 2 { self.state.tool_length } else { 0.0 };
        self.cfg.work_offsets[self.state.work_offset][i] + tool
    }

    /// Machine coordinate of the programmed (absolute, already unit-scaled)
    /// coordinate `v` of path axis `i`.
    fn to_machine(&self, i: usize, v: f64) -> f64 {
        v - self.state.g92_offset[i] + self.system_offset(i)
    }

    /// End point of a block in machine coordinates: path axes and auxiliary
    /// axes. `machine`: the block's words are machine coordinates (G53).
    fn targets(
        &self,
        axis: &[Option<f64>; 3],
        aux: &[Option<f64>],
        machine: bool,
    ) -> (PathPoint, Vec<f64>) {
        let scale = self.state.units.to_mm();
        let mut target = self.state.position;
        for i in 0..3 {
            if let Some(v) = axis[i] {
                target[i] = if machine {
                    v * scale
                } else {
                    match self.state.distance {
                        DistanceMode::Absolute => self.to_machine(i, v * scale),
                        DistanceMode::Incremental => target[i] + v * scale,
                    }
                };
            }
        }
        let mut aux_end = self.state.aux_position.clone();
        for (i, v) in aux.iter().enumerate() {
            if let Some(v) = v {
                aux_end[i] = if machine {
                    *v
                } else {
                    match self.state.distance {
                        DistanceMode::Absolute => v - self.state.aux_g92_offset[i],
                        DistanceMode::Incremental => aux_end[i] + v,
                    }
                };
            }
        }
        (target, aux_end)
    }

    /// A path element has to stay inside the working area.
    fn check_area(&self, geo: &Geometry, line: usize) -> Result<(), DecodeError> {
        let unlimited = self.area_min.iter().all(|v| *v == f64::NEG_INFINITY)
            && self.area_max.iter().all(|v| *v == f64::INFINITY);
        if unlimited || geo.length() < 1e-12 {
            return Ok(());
        }
        let (lo, hi) = geo.bounding_box();
        for i in 0..3 {
            let name = AXIS_NAMES[i];
            let (min, max) = (self.area_min[i], self.area_max[i]);
            if min > max {
                return Err(DecodeError::new(
                    line,
                    format!(
                        "the working area is empty: the lower limit of {name} ({min:.6}, G25) \
                         is above its upper limit ({max:.6}, G26)"
                    ),
                ));
            }
            let (value, limit, which) = if lo[i] < min - AREA_TOLERANCE {
                (lo[i], min, "lower")
            } else if hi[i] > max + AREA_TOLERANCE {
                (hi[i], max, "upper")
            } else {
                continue;
            };
            return Err(DecodeError::new(
                line,
                format!(
                    "the move leaves the working area: {name} reaches {value:.6}, beyond its \
                     {which} limit {limit:.6} (machine coordinates)"
                ),
            ));
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn decode_motion(
        &mut self,
        line: usize,
        axis: &[Option<f64>; 3],
        aux: &[Option<f64>],
        ijk: &[Option<f64>; 3],
        r_word: Option<f64>,
        machine: bool,
        out: &mut GeoQueue,
    ) -> Result<(), DecodeError> {
        let err = |msg: String| DecodeError::new(line, msg);
        let scale = self.state.units.to_mm();
        let start = self.state.position;
        let (target, aux_end) = self.targets(axis, aux, machine);

        let is_rapid = self.state.motion == MotionMode::Rapid;
        let (feed, accel, decel) = if is_rapid {
            (
                self.state.rapid_feed.unwrap_or(self.cfg.rapid_feed),
                self.state.rapid_accel,
                self.state.rapid_decel,
            )
        } else {
            (self.state.feed, self.state.accel, self.state.decel)
        };
        if feed <= 0.0 {
            return Err(err(
                "no feed rate set (F word required before G1/G2/G3)".into()
            ));
        }

        let geo = match self.state.motion {
            // (Of a spline run only a move of the auxiliary axes alone comes
            // here: there is no curve through a point that stands still.)
            MotionMode::Rapid | MotionMode::Linear | MotionMode::Spline => {
                Geometry::Line(LineSeg::new(start, target))
            }
            MotionMode::ArcCw | MotionMode::ArcCcw => {
                let ccw = self.state.motion == MotionMode::ArcCcw;
                let arc = if let Some(r) = r_word {
                    ArcSeg::from_radius(self.state.plane, start, target, r * scale, ccw)
                } else if ijk.iter().any(Option::is_some) {
                    // IJK are incremental offsets from the start point
                    let mut center = start;
                    for i in 0..3 {
                        if let Some(v) = ijk[i] {
                            center[i] = start[i] + v * scale;
                        }
                    }
                    ArcSeg::from_center(
                        self.state.plane,
                        start,
                        target,
                        center,
                        ccw,
                        self.cfg.radius_tolerance,
                    )
                } else {
                    return Err(err("arc move requires I/J/K or R".into()));
                };
                Geometry::Arc(arc.map_err(err)?)
            }
        };
        self.check_area(&geo, line)?;

        self.state.position = target;
        let aux_start = core::mem::replace(&mut self.state.aux_position, aux_end.clone());

        let mut mv = GeoMove {
            geo,
            feed,
            is_rapid,
            aux_start,
            aux_end,
            trc: self.state.trc,
            accel,
            decel,
            switch_points: Vec::new(),
            src_line: line,
            obj_no: self.obj_no,
        };
        let length = mv.param_length();
        if length < self.cfg.min_segment_length {
            // Elided zero-length move: its switch points stay pending and ride
            // on the next element that survives.
            return Ok(());
        }

        fold_aux_limits(
            &mut mv,
            length,
            &self.state.aux_feed,
            &self.state.aux_accel,
            &self.state.aux_decel,
        );
        if mv.feed <= 0.0 {
            return Err(err(
                "an auxiliary axis feed limit reduces this move's feed to zero".into(),
            ));
        }
        mv.switch_points = resolve_switch_points(self.pending_switches.drain(..), length, scale)?;

        self.obj_no += 1;
        out.push(GeoElem::Move(mv));
        Ok(())
    }

    /// Take a `G5` block's point into the open spline run.
    fn spline_point(
        &mut self,
        line: usize,
        axis: &[Option<f64>; 3],
        aux: &[Option<f64>],
        machine: bool,
        out: &mut GeoQueue,
    ) -> Result<(), DecodeError> {
        if self.state.feed <= 0.0 {
            return Err(DecodeError::new(
                line,
                "no feed rate set (F word required before G5)",
            ));
        }
        let start = self.state.position;
        let (target, aux_end) = self.targets(axis, aux, machine);
        if norm(&sub(&target, &start)) < self.cfg.min_segment_length.max(1e-12) {
            if aux_end == self.state.aux_position {
                // The point repeats the previous one: skipped. Its switch
                // points stay pending and ride on the next span.
                return Ok(());
            }
            // The auxiliary axes move alone: a move of its own between two
            // curves.
            self.flush(out)?;
            return self.decode_motion(line, axis, aux, &[None; 3], None, machine, out);
        }
        let aux_start = &self.state.aux_position;
        let run = self.spline.get_or_insert_with(|| SplineRun {
            start,
            aux_start: aux_start.clone(),
            points: Vec::new(),
        });
        run.points.push(SplinePoint {
            target,
            aux_end: aux_end.clone(),
            feed: self.state.feed,
            accel: self.state.accel,
            decel: self.state.decel,
            trc: self.state.trc,
            line,
            scale: self.state.units.to_mm(),
            switches: core::mem::take(&mut self.pending_switches),
            aux_feed: self.state.aux_feed.clone(),
            aux_accel: self.state.aux_accel.clone(),
            aux_decel: self.state.aux_decel.clone(),
        });
        self.state.position = target;
        self.state.aux_position = aux_end;
        Ok(())
    }

    /// Close the open `G5` spline run, if there is one, and append its
    /// elements to `out`. [`decode_chunk`](Self::decode_chunk) and
    /// [`decode_source`](Self::decode_source) do this themselves; it is for
    /// users of [`decode_block`](Self::decode_block), after their last block.
    pub fn flush(&mut self, out: &mut GeoQueue) -> Result<(), DecodeError> {
        let Some(run) = self.spline.take() else {
            return Ok(());
        };
        let n = run.points.len();
        let mut pts: Vec<PathPoint> = Vec::with_capacity(n + 1);
        pts.push(run.start);
        pts.extend(run.points.iter().map(|p| p.target));
        let chord: Vec<PathPoint> = pts.windows(2).map(|w| sub(&w[1], &w[0])).collect();
        let len: Vec<f64> = chord.iter().map(norm).collect();
        // A smooth curve cannot turn straight back at a point: it would have
        // to stand still there.
        for k in 1..n {
            if dot(&chord[k - 1], &chord[k]) <= -(1.0 - 1e-12) * len[k - 1] * len[k] {
                return Err(DecodeError::new(
                    run.points[k - 1].line,
                    "G5 spline: the path turns straight back at this point; end the spline \
                     there (with a G1 block, for one)",
                ));
            }
        }
        // Derivative per unit of chord length at the inner point `k`: the
        // two chord directions there, the shorter chord counting more.
        let slope = |k: usize| -> PathPoint {
            let (la, lb) = (len[k - 1], len[k]);
            core::array::from_fn(|i| {
                (chord[k - 1][i] / la * lb + chord[k][i] / lb * la) / (la + lb)
            })
        };
        // The end condition without curvature: the derivative at one end of a
        // span with the chord `c` that makes p'' vanish there, given the
        // derivative `other` at the span's other end.
        let natural = |c: &PathPoint, other: &PathPoint| -> PathPoint {
            core::array::from_fn(|i| (3.0 * c[i] - other[i]) / 2.0)
        };

        let mut aux_start = run.aux_start;
        for (k, pt) in run.points.into_iter().enumerate() {
            let line = pt.line;
            let fail = move |msg: String| DecodeError::new(line, msg);
            let geo = if n == 1 {
                Geometry::Line(LineSeg::new(pts[0], pts[1]))
            } else {
                // Span k runs from pts[k] to pts[k + 1]
                let l = len[k];
                let (m0, m1) = if k == 0 {
                    let m1 = slope(1).map(|x| x * l);
                    (natural(&chord[k], &m1), m1)
                } else if k + 1 == n {
                    let m0 = slope(k).map(|x| x * l);
                    (m0, natural(&chord[k], &m0))
                } else {
                    (slope(k).map(|x| x * l), slope(k + 1).map(|x| x * l))
                };
                let seg = SplineSeg::hermite(pts[k], m0, pts[k + 1], m1)
                    .map_err(|e| fail(format!("G5 spline to this point: {e}")))?;
                Geometry::Spline(Box::new(seg))
            };
            self.check_area(&geo, line)?;
            let mut mv = GeoMove {
                geo,
                feed: pt.feed,
                is_rapid: false,
                aux_start: core::mem::replace(&mut aux_start, pt.aux_end.clone()),
                aux_end: pt.aux_end,
                trc: pt.trc,
                accel: pt.accel,
                decel: pt.decel,
                switch_points: Vec::new(),
                src_line: line,
                obj_no: self.obj_no,
            };
            let length = mv.param_length();
            fold_aux_limits(&mut mv, length, &pt.aux_feed, &pt.aux_accel, &pt.aux_decel);
            if mv.feed <= 0.0 {
                return Err(fail(
                    "an auxiliary axis feed limit reduces this move's feed to zero".into(),
                ));
            }
            mv.switch_points = resolve_switch_points(pt.switches, length, pt.scale)?;
            self.obj_no += 1;
            out.push(GeoElem::Move(mv));
        }
        Ok(())
    }
}

/// A programmed `F<axis>`/`E<axis>` limits the axis' own motion; the axis
/// moves Δ over the parameter length L, so it folds into a path limit.
/// `length` is the move's parameter length; the three slices are the
/// limits per auxiliary axis.
fn fold_aux_limits(
    mv: &mut GeoMove,
    length: f64,
    aux_feed: &[Option<f64>],
    aux_accel: &[Option<f64>],
    aux_decel: &[Option<f64>],
) {
    let (mut feed, mut accel, mut decel) = (mv.feed, mv.accel, mv.decel);
    for (i, (a0, a1)) in mv.aux_start.iter().zip(mv.aux_end.iter()).enumerate() {
        let delta = (a1 - a0).abs();
        if delta <= 0.0 {
            continue;
        }
        let ratio = length / delta;
        if let Some(v) = aux_feed[i] {
            feed = feed.min(v * ratio);
        }
        if let Some(a) = aux_accel[i] {
            accel = Some(accel.map_or(a * ratio, |x: f64| x.min(a * ratio)));
        }
        if let Some(a) = aux_decel[i] {
            decel = Some(decel.map_or(a * ratio, |x: f64| x.min(a * ratio)));
        }
    }
    mv.feed = feed;
    mv.accel = accel;
    mv.decel = decel;
}

/// Resolve the switch points waiting for a path element against that
/// element's length, in programmed order. `scale`: unit scale of `L` words.
fn resolve_switch_points(
    pending: impl IntoIterator<Item = PendingSwitch>,
    length: f64,
    scale: f64,
) -> Result<Vec<SwitchPoint>, DecodeError> {
    let mut points = Vec::new();
    for p in pending {
        let position = match p.pos {
            // No L/O word: the switch point sits at the element's start.
            None => 0.0,
            Some(SwitchPos::Relative(o)) => o * length,
            // L is a distance from the start, or from the end when negative.
            Some(SwitchPos::Absolute(l)) if l >= 0.0 => l * scale,
            Some(SwitchPos::Absolute(l)) => length + l * scale,
        };
        if !(0.0..=length).contains(&position) {
            return Err(DecodeError::new(
                p.src_line,
                format!(
                    "switch point H{} falls outside its {length:.6} long path element \
                     (resolved to {position:.6})",
                    p.number
                ),
            ));
        }
        points.push(SwitchPoint {
            number: p.number,
            position,
            src_line: p.src_line,
        });
    }
    points.sort_by(|a, b| a.position.total_cmp(&b.position));
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Segment;

    fn decode(src: &str) -> GeoQueue {
        Decoder::decode_source(DecoderConfig::default(), src).unwrap()
    }

    /// The decoded moves of a program, owned, for tests that do not need to
    /// keep the queue alive.
    fn move_list(src: &str) -> Vec<GeoMove> {
        moves(&decode(src)).into_iter().cloned().collect()
    }

    fn moves(q: &GeoQueue) -> Vec<&GeoMove> {
        q.iter()
            .filter_map(|e| match e {
                GeoElem::Move(m) => Some(m),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn modal_motion_and_feed() {
        let q = decode("G1 F10 X10\nY5\nG0 X0 Y0\n");
        let m = moves(&q);
        assert_eq!(m.len(), 3);
        // F is units/second, as in CODESYS SoftMotion
        assert!((m[0].feed - 10.0).abs() < 1e-12);
        assert!(!m[0].is_rapid);
        assert_eq!(m[0].geo.end(), [10.0, 0.0, 0.0]);
        // modal G1 continues on the next line
        assert_eq!(m[1].geo.end(), [10.0, 5.0, 0.0]);
        assert!(m[2].is_rapid);
        assert!((m[2].feed - 100.0).abs() < 1e-12);
    }

    #[test]
    fn incremental_and_units() {
        let q = decode("G91 G1 F1 X10\nX10\nG70 X1\n");
        let m = moves(&q);
        assert_eq!(m[1].geo.end(), [20.0, 0.0, 0.0]);
        // G70: inches - 1 inch increment = 25.4 mm
        assert!((m[2].geo.end()[0] - 45.4).abs() < 1e-12);
        // F1 in inch mode would be 25.4 mm/s, but F was set in mm mode: 1 mm/s
        assert!((m[2].feed - 1.0).abs() < 1e-12);
    }

    #[test]
    fn arc_ijk_and_r() {
        // Quarter circle (0,0) -> (10,10), center (0,10)
        let q = decode("G1 F600 X0 Y0\nG3 X10 Y10 I0 J10\nG2 X0 Y0 R10\n");
        let m = moves(&q);
        assert_eq!(m.len(), 2); // first move is zero-length, elided
        match &m[0].geo {
            Geometry::Arc(a) => {
                assert!((a.r_start - 10.0).abs() < 1e-9);
                assert!(a.sweep > 0.0);
                let end = a.end();
                assert!((end[0] - 10.0).abs() < 1e-9 && (end[1] - 10.0).abs() < 1e-9);
            }
            _ => panic!("expected arc"),
        }
        match &m[1].geo {
            Geometry::Arc(a) => assert!(a.sweep < 0.0),
            _ => panic!("expected arc"),
        }
    }

    #[test]
    fn g92_offset() {
        let q = decode("G1 F600 X10\nG92 X0\nX5\n");
        let m = moves(&q);
        // After G92 X0 at machine X=10, programmed X5 = machine 15
        assert_eq!(m[1].geo.start(), [10.0, 0.0, 0.0]);
        assert_eq!(m[1].geo.end(), [15.0, 0.0, 0.0]);
    }

    #[test]
    fn events_and_program_end() {
        let q = decode("G1 F600 X5 M8\nM5 M9\nS1200 T2\nM30\nG1 X99\n");
        let elems: Vec<_> = q.iter().collect();
        // move, event(M8), event(M5,M9), event(S,T)
        assert_eq!(elems.len(), 4);
        match elems[1] {
            GeoElem::Event(e) => assert_eq!(e.m, vec![8]),
            _ => panic!(),
        }
        match elems[3] {
            GeoElem::Event(e) => {
                assert_eq!(e.spindle, Some(1200.0));
                assert_eq!(e.tool, Some(2));
            }
            _ => panic!(),
        }
        // G1 X99 after M30 must not decode
        assert!(moves(&q).len() == 1);
    }

    #[test]
    fn e_word_sets_acceleration_and_deceleration() {
        // The CODESYS reference block: F is the feed, +E the acceleration,
        // -E the deceleration, and all three are modal.
        let m = move_list("N10 G01 X100 Y100 E100 F100 E-200\nX150\n");
        assert_eq!(m[0].feed, 100.0);
        assert_eq!(m[0].accel, Some(100.0));
        assert_eq!(m[0].decel, Some(200.0));
        assert_eq!((m[1].accel, m[1].decel), (Some(100.0), Some(200.0)));
        // Unit mode scales E, and it is the block's own G70 that decides
        assert_eq!(move_list("G70 G1 F1 E2 X1\n")[0].accel, Some(2.0 * 25.4));
        assert!(Decoder::decode_source(DecoderConfig::default(), "G1 F10 E0 X1\n").is_err());
    }

    #[test]
    fn rapid_feed_and_acceleration_words() {
        let m = move_list("G0 FF200 EF1000 EF-500 X10\n");
        assert!(m[0].is_rapid);
        assert_eq!(m[0].feed, 200.0);
        assert_eq!((m[0].accel, m[0].decel), (Some(1000.0), Some(500.0)));
        // G0 dynamics do not leak into the feed moves
        let m = move_list("G0 FF200 EF1000 X10\nG1 F5 X20\n");
        assert_eq!((m[1].feed, m[1].accel), (5.0, None));
    }

    #[test]
    fn switch_points_position_on_the_element() {
        // Absolute from the start, absolute from the end, and relative
        let m = move_list("G1 F50 X100 H2 L20 H-2 L-10\n");
        let sp = &m[0].switch_points;
        assert_eq!(sp.len(), 2);
        assert_eq!((sp[0].number, sp[0].position), (2, 20.0));
        assert!(sp[0].is_on() && sp[0].id() == 2);
        assert_eq!((sp[1].number, sp[1].position), (-2, 90.0));
        assert!(!sp[1].is_on());
        let m = move_list("G1 F50 X100 H-1 O0.8\n");
        assert_eq!(m[0].switch_points[0].position, 80.0);
        // An H with no L/O sits at the element's start
        let m = move_list("G1 F50 X100 H7\n");
        assert_eq!(m[0].switch_points[0].position, 0.0);
    }

    #[test]
    fn switch_point_errors() {
        let bad = |src: &str| Decoder::decode_source(DecoderConfig::default(), src).is_err();
        // Beyond the element it was programmed on
        assert!(bad("G1 F50 X10 H1 L20\n"));
        assert!(bad("G1 F50 X10 H1 O1.5\n"));
        // A position word with no switch point to position
        assert!(bad("G1 F50 X10 L5\n"));
        // H must be a non-zero whole number
        assert!(bad("G1 F50 X10 H0\n"));
        assert!(bad("G1 F50 X10 H1.5\n"));
        // Nothing left to ride on
        assert!(bad("G1 F50 X10\nH5 O0.5\n"));
    }

    #[test]
    fn switch_point_from_a_block_without_geometry_rides_the_next_element() {
        let m = move_list("H5 O0.5\nG1 F50 X10\n");
        assert_eq!(m[0].switch_points.len(), 1);
        assert_eq!(m[0].switch_points[0].position, 5.0);
        // Same for a block whose move is elided as zero-length
        let m = move_list("G1 F50 X0 H5 O0.5\nX10\n");
        assert_eq!(m[0].switch_points[0].position, 5.0);
    }

    #[test]
    fn auxiliary_axis_limits_fold_into_the_path_limits() {
        let cfg = || DecoderConfig {
            aux_letters: vec!['A'],
            start_aux: vec![0.0],
            ..Default::default()
        };
        // A moves 20 while the path moves 10, so it runs at twice the path
        // speed: FA5 caps the path at 2.5 and EA100 the path accel at 50.
        let q = Decoder::decode_source(cfg(), "G1 F100 E400 FA5 EA100 X10 A20\n").unwrap();
        let m = moves(&q);
        assert_eq!(m[0].feed, 2.5);
        assert_eq!(m[0].accel, Some(50.0));
        // An axis that does not move in this block does not constrain it
        let q = Decoder::decode_source(cfg(), "G1 F100 FA5 X10\n").unwrap();
        assert_eq!(moves(&q)[0].feed, 100.0);
        // A letter that is not an auxiliary axis of this machine is refused
        assert!(Decoder::decode_source(cfg(), "G1 F100 FB5 X10\n").is_err());
    }

    #[test]
    fn codesys_program_with_e_h_l_and_o_decodes() {
        // A program that runs in CODESYS SoftMotion, verbatim.
        let src = "N000 G01 X82.9 Y8.9 F50 E30 E-30 \n\
                   N010 G02 X84.9 Y84.6 R100 H12 L10 \n\
                   N020 G01 X6.6 Y25.7 H-1 O0.8 \n\
                   N030 G03 X54.6 Y49.7 R100 \n\
                   N040 G01 X54.914 Y49.571 Z0 \n\
                   N050 G01 X0 Y0 Z0 \n";
        let m = move_list(src);
        assert_eq!(m.len(), 6);
        assert_eq!(
            (m[0].feed, m[0].accel, m[0].decel),
            (50.0, Some(30.0), Some(30.0))
        );
        assert_eq!(m[1].switch_points[0].number, 12);
        assert_eq!(m[1].switch_points[0].position, 10.0);
        let sp = m[2].switch_points[0];
        assert_eq!(sp.number, -1);
        assert!((sp.position - 0.8 * m[2].geo.length()).abs() < 1e-9);
    }

    #[test]
    fn aux_axes() {
        let cfg = DecoderConfig {
            aux_letters: vec!['A'],
            start_aux: vec![0.0],
            ..Default::default()
        };
        let q = Decoder::decode_source(cfg, "G1 F600 X10 A90\nA180\n").unwrap();
        let m = moves(&q);
        assert_eq!(m.len(), 2);
        assert_eq!(m[0].aux_end, vec![90.0]);
        // aux-only move: param length from aux displacement
        assert!((m[1].param_length() - 90.0).abs() < 1e-12);
        assert!(m[1].geo.length() < 1e-12);
    }

    #[test]
    fn dwell_and_errors() {
        let q = decode("G4 P0.5\n");
        match q.get(0).unwrap() {
            GeoElem::Dwell { seconds, .. } => assert!((seconds - 0.5).abs() < 1e-12),
            _ => panic!(),
        }
        assert!(Decoder::decode_source(Default::default(), "G1 X10\n").is_err()); // no feed
        assert!(Decoder::decode_source(Default::default(), "G33 X10\n").is_err()); // unsupported
        assert!(Decoder::decode_source(Default::default(), "G2 F600 X10 Y10\n").is_err());
        // no IJK/R
    }

    #[test]
    fn lex_and_decode_errors_keep_their_stage_and_line() {
        // Stage 1: the lexer chokes on line 3 ('#' is not a word letter).
        let lex_err = Decoder::decode_source(DecoderConfig::default(), "G1 F600 X10\nY5\nX1 #5\n")
            .unwrap_err();
        match &lex_err {
            CncError::Gcode(e) => assert_eq!(e.line, 3, "lex error line: {e}"),
            other => panic!("expected a G-code (lex) error, got {other:?}"),
        }

        // Stage 2: the block lexes fine but has no meaning, on line 2.
        let decode_err =
            Decoder::decode_source(DecoderConfig::default(), "G1 F600 X10\nG33 X1\n").unwrap_err();
        match &decode_err {
            CncError::Decode(e) => assert_eq!(e.line, 2, "decode error line: {e}"),
            other => panic!("expected a decode error, got {other:?}"),
        }

        // The two stages are distinguishable, which is the whole point.
        assert_ne!(
            core::mem::discriminant(&lex_err),
            core::mem::discriminant(&decode_err)
        );

        // An unterminated comment is a lex error too, on its own line.
        let comment_err =
            Decoder::decode_source(DecoderConfig::default(), "G1 F600 X10\nX1 (oops\n")
                .unwrap_err();
        assert!(matches!(comment_err, CncError::Gcode(e) if e.line == 2));
    }
}
