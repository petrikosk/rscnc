//! Decoder (the `SMC_NCDecoder` analogue): parsed blocks + modal state →
//! geometry queue.
//!
//! Supported words (v1): `N G0 G1 G2 G3 G4 G17 G18 G19 G40 G41 G42
//! G70/G71 (G20/G21 aliases) G90 G91 G92, X Y Z I J K R F E S T M H L O, P`
//! (dwell time for G4, seconds) plus the configured auxiliary axis letters and
//! their `F<axis>`/`E<axis>` limits. `M2`/`M30` end the program. All other M
//! codes become synchronized [`SyncEvent`]s; unknown G codes are errors.
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

use crate::error::{CncError, DecodeError};
use crate::gcode::lexer::{lex, ParsedBlock};
use crate::gcode::program::{DistanceMode, ModalState, MotionMode, TrcMode, Units};
use crate::geometry::{ArcSeg, GeoElem, GeoMove, Geometry, LineSeg, SwitchPoint, SyncEvent};
use crate::queue::GeoQueue;
use crate::types::WorkPlane;

/// Decoder configuration: machine start pose, auxiliary axis letters and
/// tolerances.
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
        }
    }
}

/// The DIN 66025 decoder: turns parsed blocks into geometry queue elements
/// while tracking modal state. Use [`Decoder::decode_source`] for the
/// common whole-program case.
pub struct Decoder {
    cfg: DecoderConfig,
    state: ModalState,
    obj_no: u32,
    ended: bool,
    /// Switch points programmed in blocks that produced no path element of
    /// their own; they ride on the next element that is created.
    pending_switches: Vec<PendingSwitch>,
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
            cfg,
            state,
            obj_no: 0,
            ended: false,
            pending_switches: Vec::new(),
        }
    }

    /// Convenience: lex + decode a whole program, keeping the pipeline stage
    /// that failed.
    ///
    /// A lexical problem comes back as [`CncError::Gcode`] and a semantic one
    /// as [`CncError::Decode`], each carrying the 1-based source line, so a
    /// caller can tell "bad token on line N" from "bad block on line N"
    /// without driving [`lex`] and [`decode_block`](Self::decode_block) by
    /// hand.
    ///
    /// ```
    /// use rscnc::prelude::*;
    ///
    /// let err = Decoder::decode_source(DecoderConfig::default(), "G1 F600 X1\nG1 X#\n")
    ///     .unwrap_err();
    /// assert!(matches!(err, CncError::Gcode(ref e) if e.line == 2));
    /// ```
    pub fn decode_source(cfg: DecoderConfig, source: &str) -> Result<GeoQueue, CncError> {
        let blocks = lex(source)?;
        let mut decoder = Decoder::new(cfg);
        let mut queue = GeoQueue::new();
        for block in &blocks {
            decoder.decode_block(block, &mut queue)?;
            if decoder.ended {
                break;
            }
        }
        if let Some(p) = decoder.pending_switches.first() {
            return Err(DecodeError::new(
                p.src_line,
                format!(
                    "switch point H{} has no path element to ride on (no move follows it)",
                    p.number
                ),
            )
            .into());
        }
        Ok(queue)
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

    /// Decode one block, appending 0..k elements to `out`.
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
        for &g in &g_codes {
            match g {
                0 => self.state.motion = MotionMode::Rapid,
                1 => self.state.motion = MotionMode::Linear,
                2 => self.state.motion = MotionMode::ArcCw,
                3 => self.state.motion = MotionMode::ArcCcw,
                4 => dwell = true,
                17 => self.state.plane = WorkPlane::Xy,
                18 => self.state.plane = WorkPlane::Zx,
                19 => self.state.plane = WorkPlane::Yz,
                40 => self.state.trc = TrcMode::Off,
                41 => self.state.trc = TrcMode::Left,
                42 => self.state.trc = TrcMode::Right,
                70 | 20 => self.state.units = Units::Inch,
                71 | 21 => self.state.units = Units::Mm,
                90 => self.state.distance = DistanceMode::Absolute,
                91 => self.state.distance = DistanceMode::Incremental,
                92 => set_position = true,
                other => return Err(err(format!("unsupported G code G{other}"))),
            }
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

        if dwell {
            let seconds = p_word.ok_or_else(|| err("G4 requires a P word (seconds)".into()))?;
            if seconds < 0.0 || !seconds.is_finite() {
                return Err(err(format!("invalid dwell time P{seconds}")));
            }
            out.push(GeoElem::Dwell {
                seconds,
                src_line: line,
            });
        } else if set_position {
            // G92: the current point gets the given programmed coordinates.
            for (i, v) in axis.iter().enumerate() {
                if let Some(v) = v {
                    self.state.g92_offset[i] = v * scale - self.state.position[i];
                }
            }
            for (i, v) in aux.iter().enumerate() {
                if let Some(v) = v {
                    self.state.aux_g92_offset[i] = v - self.state.aux_position[i];
                }
            }
        } else if axis.iter().any(Option::is_some) || aux.iter().any(Option::is_some) {
            self.pending_switches.append(&mut switches);
            self.decode_motion(line, &axis, &aux, &ijk, r_word, out)?;
        } else if matches!(self.state.motion, MotionMode::ArcCw | MotionMode::ArcCcw)
            && (r_word.is_some() || ijk.iter().any(Option::is_some))
        {
            // Arc data without axis words: the end point is the start point,
            // i.e. a full circle (`G2 I-5`), also on a modal follow-up block.
            self.pending_switches.append(&mut switches);
            self.decode_motion(line, &axis, &aux, &ijk, r_word, out)?;
        } else if g_codes.contains(&2) || g_codes.contains(&3) {
            return Err(err("arc move without axis words or I/J/K".into()));
        } else {
            // A block with no path element of its own: its switch points ride
            // on the next element, at its start.
            self.pending_switches.append(&mut switches);
        }

        // Program end
        if m_codes.iter().any(|&m| m == 2 || m == 30) {
            self.ended = true;
            m_codes.retain(|&m| m != 2 && m != 30);
        }

        // Synchronized events after the block's motion (CoDeSys default)
        if !m_codes.is_empty() || s_word.is_some() || t_word.is_some() {
            let tool = match t_word {
                Some(t) if t >= 0.0 && t.fract() == 0.0 => Some(t as u32),
                Some(t) => return Err(err(format!("invalid tool number T{t}"))),
                None => None,
            };
            if let Some(s) = s_word {
                self.state.spindle_speed = s;
            }
            if let Some(t) = tool {
                self.state.tool = t;
            }
            out.push(GeoElem::Event(SyncEvent {
                m: m_codes,
                spindle: s_word,
                tool,
                src_line: line,
            }));
        }

        Ok(())
    }

    fn decode_motion(
        &mut self,
        line: usize,
        axis: &[Option<f64>; 3],
        aux: &[Option<f64>],
        ijk: &[Option<f64>; 3],
        r_word: Option<f64>,
        out: &mut GeoQueue,
    ) -> Result<(), DecodeError> {
        let err = |msg: String| DecodeError::new(line, msg);
        let scale = self.state.units.to_mm();
        let start = self.state.position;

        // Target in machine coordinates
        let mut target = start;
        for i in 0..3 {
            if let Some(v) = axis[i] {
                target[i] = match self.state.distance {
                    DistanceMode::Absolute => v * scale - self.state.g92_offset[i],
                    DistanceMode::Incremental => start[i] + v * scale,
                };
            }
        }
        let mut aux_end = self.state.aux_position.clone();
        for (i, v) in aux.iter().enumerate() {
            if let Some(v) = v {
                aux_end[i] = match self.state.distance {
                    DistanceMode::Absolute => v - self.state.aux_g92_offset[i],
                    DistanceMode::Incremental => aux_end[i] + v,
                };
            }
        }

        let is_rapid = self.state.motion == MotionMode::Rapid;
        let (mut feed, mut accel, mut decel) = if is_rapid {
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
            MotionMode::Rapid | MotionMode::Linear => Geometry::Line(LineSeg::new(start, target)),
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

        // A programmed F<axis>/E<axis> limits the axis' own motion; the axis
        // moves Δ over the parameter length L, so it folds into a path limit.
        for (i, (a0, a1)) in mv.aux_start.iter().zip(mv.aux_end.iter()).enumerate() {
            let delta = (a1 - a0).abs();
            if delta <= 0.0 {
                continue;
            }
            let ratio = length / delta;
            if let Some(v) = self.state.aux_feed[i] {
                feed = feed.min(v * ratio);
            }
            if let Some(a) = self.state.aux_accel[i] {
                accel = Some(accel.map_or(a * ratio, |x: f64| x.min(a * ratio)));
            }
            if let Some(a) = self.state.aux_decel[i] {
                decel = Some(decel.map_or(a * ratio, |x: f64| x.min(a * ratio)));
            }
        }
        if feed <= 0.0 {
            return Err(err(
                "an auxiliary axis feed limit reduces this move's feed to zero".into(),
            ));
        }
        mv.feed = feed;
        mv.accel = accel;
        mv.decel = decel;
        mv.switch_points = self.take_switch_points(length, scale)?;

        self.obj_no += 1;
        out.push(GeoElem::Move(mv));
        Ok(())
    }

    /// Resolve the switch points waiting for a path element against that
    /// element's length, in programmed order.
    fn take_switch_points(
        &mut self,
        length: f64,
        scale: f64,
    ) -> Result<Vec<SwitchPoint>, DecodeError> {
        if self.pending_switches.is_empty() {
            return Ok(Vec::new());
        }
        let mut points = Vec::with_capacity(self.pending_switches.len());
        for p in self.pending_switches.drain(..) {
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
        assert!(Decoder::decode_source(Default::default(), "G5 X10\n").is_err()); // unsupported
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
            Decoder::decode_source(DecoderConfig::default(), "G1 F600 X10\nG5 X1\n").unwrap_err();
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
