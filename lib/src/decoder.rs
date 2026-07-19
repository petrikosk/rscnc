//! Decoder (the `SMC_NCDecoder` analogue): parsed blocks + modal state →
//! geometry queue.
//!
//! Supported words (v1): `N G0 G1 G2 G3 G4 G17 G18 G19 G40 G41 G42
//! G70/G71 (G20/G21 aliases) G90 G91 G92, X Y Z I J K R F S T M H, P` (dwell
//! time for G4, seconds) plus configurable auxiliary axis letters
//! (`A B C U V W` by default). `M2`/`M30` end the program. All other M codes
//! become synchronized [`SyncEvent`]s; unknown G codes are errors.
//!
//! Internal units are mm and seconds: axis words are scaled by the active
//! unit mode, `F` (units/min) is converted to units/s. Auxiliary axis words
//! are *not* unit-scaled (they are typically rotary degrees).

use alloc::{format, string::String, vec, vec::Vec};

#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::error::DecodeError;
use crate::gcode::lexer::{lex, ParsedBlock};
use crate::gcode::program::{DistanceMode, ModalState, MotionMode, TrcMode, Units};
use crate::geometry::{ArcSeg, GeoElem, GeoMove, Geometry, LineSeg, SyncEvent};
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
/// while tracking modal state. Use [`Decoder::decode_program`] for the
/// common whole-program case.
pub struct Decoder {
    cfg: DecoderConfig,
    state: ModalState,
    obj_no: u32,
    ended: bool,
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
        }
    }

    /// Convenience: lex + decode a whole program.
    pub fn decode_program(cfg: DecoderConfig, source: &str) -> Result<GeoQueue, DecodeError> {
        let blocks = lex(source).map_err(|e| DecodeError::new(e.line, e.message))?;
        let mut decoder = Decoder::new(cfg);
        let mut queue = GeoQueue::new();
        for block in &blocks {
            decoder.decode_block(block, &mut queue)?;
            if decoder.ended {
                break;
            }
        }
        Ok(queue)
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
        let (mut r_word, mut f_word, mut s_word, mut t_word, mut h_word, mut p_word) =
            (None, None, None, None, None, None);

        for w in &block.words {
            match w.letter {
                'G' => {
                    if w.value.fract() != 0.0 {
                        return Err(err(format!("unsupported decimal G code G{}", w.value)));
                    }
                    g_codes.push(w.value as i32);
                }
                'M' => {
                    if w.value.fract() != 0.0 || w.value < 0.0 {
                        return Err(err(format!("invalid M code M{}", w.value)));
                    }
                    m_codes.push(w.value as u32);
                }
                'X' => axis[0] = Some(w.value),
                'Y' => axis[1] = Some(w.value),
                'Z' => axis[2] = Some(w.value),
                'I' => ijk[0] = Some(w.value),
                'J' => ijk[1] = Some(w.value),
                'K' => ijk[2] = Some(w.value),
                'R' => r_word = Some(w.value),
                'F' => f_word = Some(w.value),
                'S' => s_word = Some(w.value),
                'T' => t_word = Some(w.value),
                'H' => h_word = Some(w.value),
                'P' => p_word = Some(w.value),
                c => {
                    if let Some(i) = self.cfg.aux_letters.iter().position(|&l| l == c) {
                        aux[i] = Some(w.value);
                    } else {
                        return Err(err(format!("unsupported word letter '{c}'")));
                    }
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
            self.state.feed = f * scale / 60.0;
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
            self.decode_motion(line, &axis, &aux, &ijk, r_word, out)?;
        } else if g_codes.contains(&2) || g_codes.contains(&3) {
            return Err(err("arc move without axis words".into()));
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
                h: h_word,
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
        let aux_start = self.state.aux_position.clone();
        let mut aux_end = aux_start.clone();
        for (i, v) in aux.iter().enumerate() {
            if let Some(v) = v {
                aux_end[i] = match self.state.distance {
                    DistanceMode::Absolute => v - self.state.aux_g92_offset[i],
                    DistanceMode::Incremental => aux_start[i] + v,
                };
            }
        }

        let is_rapid = self.state.motion == MotionMode::Rapid;
        let feed = if is_rapid {
            self.cfg.rapid_feed
        } else {
            self.state.feed
        };
        if feed <= 0.0 {
            return Err(err("no feed rate set (F word required before G1/G2/G3)".into()));
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
        self.state.aux_position = aux_end.clone();

        let mv = GeoMove {
            geo,
            feed,
            is_rapid,
            aux_start,
            aux_end,
            trc: self.state.trc,
            src_line: line,
            obj_no: self.obj_no,
        };
        if mv.param_length() < self.cfg.min_segment_length {
            return Ok(()); // elide zero-length move
        }
        self.obj_no += 1;
        out.push(GeoElem::Move(mv));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Segment;

    fn decode(src: &str) -> GeoQueue {
        Decoder::decode_program(DecoderConfig::default(), src).unwrap()
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
        let q = decode("G1 F600 X10\nY5\nG0 X0 Y0\n");
        let m = moves(&q);
        assert_eq!(m.len(), 3);
        // F600 mm/min = 10 mm/s
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
        let q = decode("G91 G1 F60 X10\nX10\nG70 X1\n");
        let m = moves(&q);
        assert_eq!(m[1].geo.end(), [20.0, 0.0, 0.0]);
        // G70: inches - 1 inch increment = 25.4 mm
        assert!((m[2].geo.end()[0] - 45.4).abs() < 1e-12);
        // F60 in inch mode would be 60*25.4/60, but F was set in mm mode: 1 mm/s
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
    fn aux_axes() {
        let cfg = DecoderConfig {
            aux_letters: vec!['A'],
            start_aux: vec![0.0],
            ..Default::default()
        };
        let q = Decoder::decode_program(cfg, "G1 F600 X10 A90\nA180\n").unwrap();
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
        assert!(Decoder::decode_program(Default::default(), "G1 X10\n").is_err()); // no feed
        assert!(Decoder::decode_program(Default::default(), "G5 X10\n").is_err()); // unsupported
        assert!(Decoder::decode_program(Default::default(), "G2 F600 X10 Y10\n").is_err()); // no IJK/R
    }
}
