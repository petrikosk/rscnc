//! The decoder's dialect beyond plain DIN 66025 moves: variables and program
//! flow, work offsets, tool length compensation, the working area, `G5`
//! splines and decoding a program piece by piece.

use rscnc::prelude::*;

fn decode(src: &str) -> GeoQueue {
    Decoder::decode_source(DecoderConfig::default(), src).expect("program decodes")
}

fn decode_with(cfg: &DecoderConfig, src: &str) -> GeoQueue {
    Decoder::decode_source(cfg.clone(), src).expect("program decodes")
}

fn moves(q: &GeoQueue) -> Vec<&GeoMove> {
    q.iter()
        .filter_map(|e| match e {
            GeoElem::Move(m) => Some(m),
            _ => None,
        })
        .collect()
}

/// End points of the decoded moves.
fn ends(q: &GeoQueue) -> Vec<PathPoint> {
    moves(q).iter().map(|m| m.geo.end()).collect()
}

fn assert_point(got: PathPoint, want: PathPoint) {
    for i in 0..3 {
        assert!(
            (got[i] - want[i]).abs() < 1e-9,
            "point {got:?}, expected {want:?}"
        );
    }
}

fn assert_points(got: &[PathPoint], want: &[PathPoint]) {
    assert_eq!(got.len(), want.len(), "points {got:?}, expected {want:?}");
    for (g, w) in got.iter().zip(want) {
        assert_point(*g, *w);
    }
}

/// The error of a program that must not decode: (is a decode-stage error,
/// line, message).
fn failure_with(cfg: &DecoderConfig, src: &str) -> (bool, usize, String) {
    match Decoder::decode_source(cfg.clone(), src) {
        Ok(_) => panic!("program decoded, expected an error:\n{src}"),
        Err(CncError::Decode(e)) => (true, e.line, e.message),
        Err(CncError::Gcode(e)) => (false, e.line, e.message),
        Err(other) => panic!("unexpected error {other}"),
    }
}

/// A decode-stage error: (line, message).
fn decode_error_with(cfg: &DecoderConfig, src: &str) -> (usize, String) {
    let (decode_stage, line, message) = failure_with(cfg, src);
    assert!(
        decode_stage,
        "expected a decode error, got a lexical one: {message}"
    );
    (line, message)
}

fn decode_error(src: &str) -> (usize, String) {
    decode_error_with(&DecoderConfig::default(), src)
}

/// A lexical / syntax error: (line, message).
fn syntax_error(src: &str) -> (usize, String) {
    let (decode_stage, line, message) = failure_with(&DecoderConfig::default(), src);
    assert!(
        !decode_stage,
        "expected a syntax error, got a decode error: {message}"
    );
    (line, message)
}

// ---------------------------------------------------------------------------
// Variables and expressions
// ---------------------------------------------------------------------------

#[test]
fn variables_give_word_values() {
    let q = decode(
        "#1 = 10\n\
         #2 = [#1 * 2 + 1] (21)\n\
         G1 F[#1 * 5] X#1 Y-#2\n\
         X[#1 + 2.5] Y[SQRT[#2 - 5] + ABS[-1]]\n\
         #3 = 3 #4 = #3 * #3\n\
         G1 X#4 Y0\n",
    );
    let m = moves(&q);
    assert_points(
        &ends(&q),
        &[[10.0, -21.0, 0.0], [12.5, 5.0, 0.0], [9.0, 0.0, 0.0]],
    );
    assert!((m[0].feed - 50.0).abs() < 1e-12);
    // every element carries the source line of its own block
    assert_eq!(m.iter().map(|m| m.src_line).collect::<Vec<_>>(), [3, 4, 6]);
}

#[test]
fn expressions_follow_the_documented_precedence() {
    // comparisons bind looser than arithmetic, the logical operators loosest
    let q = decode(
        "#1 = 2 + 3 * 4\n\
         #2 = [2 + 3] * 4\n\
         #3 = 7 MOD 4 + 1\n\
         #4 = 1 + 1 EQ 2\n\
         #5 = 1 LT 2 AND 3 GT 4\n\
         #6 = 1 LT 2 OR 3 GT 4\n\
         #7 = -#1 + 20\n\
         #8 = ATAN[1]/[1] + ROUND[2.5] + FIX[-2.7] + FUP[2.1]\n\
         G1 F1 X#1 Y#2 Z#3\n\
         X#4 Y#5 Z#6\n\
         X#7 Y#8 Z0\n",
    );
    assert_points(
        &ends(&q),
        &[[14.0, 20.0, 4.0], [1.0, 0.0, 1.0], [6.0, 49.0, 0.0]],
    );
}

#[test]
fn variables_are_seeded_from_the_configuration_and_readable() {
    let cfg = DecoderConfig {
        variables: vec![(100, 12.5), (101, 2.0)],
        ..DecoderConfig::default()
    };
    let mut d = Decoder::new(cfg);
    let q = d.decode_chunk("#102 = #100 * #101\nG1 F1 X#102\n").unwrap();
    assert_points(&ends(&q), &[[25.0, 0.0, 0.0]]);
    assert_eq!(d.variable(102), Some(25.0));
    assert_eq!(d.variable(7), None);
    d.set_variable(7, 3.0);
    let q = d.decode_chunk("X#7\n").unwrap();
    assert_points(&ends(&q), &[[3.0, 0.0, 0.0]]);
}

#[test]
fn indirect_variables() {
    let q = decode("#1 = 5\n#5 = 42\n#[#1 + 1] = 7\nG1 F1 X#[#1] Y#6 Z##1\n");
    assert_points(&ends(&q), &[[42.0, 7.0, 42.0]]);
}

#[test]
fn an_undefined_variable_is_an_error_on_its_line() {
    let (line, msg) = decode_error("G1 F100 X1\nG1 X#5\n");
    assert_eq!(line, 2);
    assert!(msg.contains("#5"), "{msg}");
    // also on the right-hand side of an assignment and in a condition
    assert_eq!(decode_error("#1 = 1\n\n#2 = #3 + 1\n").0, 3);
    assert_eq!(decode_error("#1 = 1\nIF [#9 GT 0] GOTO 1\n").0, 2);
}

#[test]
fn bad_expressions_are_syntax_errors_on_their_line() {
    // the old contract: a '#' without a number is a lexical error
    assert_eq!(syntax_error("G1 F600 X1\nG1 X#\n").0, 2);
    assert_eq!(syntax_error("#1 = [1 + \n").0, 1);
    assert_eq!(syntax_error("G1 F1 X1\n\n#1 = 1 + * 2\n").0, 3);
    assert_eq!(syntax_error("#1 = 2 ]\n").0, 1);
    assert_eq!(syntax_error("G1 X[1 + 2\n").0, 1);
    assert_eq!(syntax_error("#1 = NOSUCH[2]\n").0, 1);
    assert_eq!(syntax_error("IF [1] GOTO\n").0, 1);
    assert_eq!(syntax_error("WHILE [1]\n").0, 1);
}

#[test]
fn arithmetic_failures_are_decode_errors_on_their_line() {
    let (line, msg) = decode_error("#1 = 0\n#2 = 1 / #1\n");
    assert_eq!(line, 2);
    assert!(msg.to_lowercase().contains("zero"), "{msg}");
    assert_eq!(decode_error("#1 = SQRT[-1]\n").0, 1);
    // a variable number must be a whole number
    assert_eq!(decode_error("#1 = 1.5\n#[#1] = 2\n").0, 2);
}

// ---------------------------------------------------------------------------
// Program flow
// ---------------------------------------------------------------------------

#[test]
fn while_loop_repeats_its_body() {
    let q = decode(
        "#1 = 0\n\
         WHILE [#1 LT 5] DO 1\n\
         G1 F100 X[#1 * 10 + 10]\n\
         #1 = #1 + 1\n\
         END 1\n\
         G1 Y1\n",
    );
    let m = moves(&q);
    assert_eq!(m.len(), 6);
    for (i, mv) in m[..5].iter().enumerate() {
        assert_point(mv.geo.end(), [10.0 * (i as f64 + 1.0), 0.0, 0.0]);
        assert_eq!(mv.src_line, 3);
        assert_eq!(mv.obj_no, i as u32);
    }
    assert_point(m[5].geo.end(), [50.0, 1.0, 0.0]);
}

#[test]
fn while_loops_nest_and_a_false_condition_skips_the_body() {
    let q = decode(
        "G91 G1 F100\n\
         #1 = 0\n\
         WHILE [#1 LT 3] DO 1\n\
         #2 = 0\n\
         WHILE [#2 LT 2] DO 2\n\
         X1\n\
         #2 = #2 + 1\n\
         END 2\n\
         Y1\n\
         #1 = #1 + 1\n\
         END 1\n\
         WHILE [#1 LT 0] DO 1\n\
         Z99\n\
         END 1\n",
    );
    assert_eq!(moves(&q).len(), 9);
    assert_point(*ends(&q).last().unwrap(), [6.0, 3.0, 0.0]);
}

#[test]
fn goto_and_if_goto_jump_to_block_numbers() {
    // forward: skip a block
    let q = decode("G1 F100 X1\nGOTO 20\nX99\nN20 X2\n");
    assert_points(&ends(&q), &[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);

    // backward: a counted loop
    let q = decode(
        "#1 = 0\n\
         N10 G91 G1 F100 X1\n\
         #1 = #1 + 1\n\
         IF [#1 LT 4] GOTO 10\n\
         G90 Y5\n",
    );
    assert_eq!(moves(&q).len(), 5);
    assert_point(*ends(&q).last().unwrap(), [4.0, 5.0, 0.0]);

    // the target may be computed
    let q = decode("#1 = 30\nG1 F100 X1\nGOTO #1\nX99\nN30 X2\n");
    assert_points(&ends(&q), &[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
}

#[test]
fn if_then_assigns_conditionally() {
    let q = decode(
        "#1 = 5\n\
         IF [#1 GT 3] THEN #2 = 1\n\
         IF [#1 GT 9] THEN #2 = 99\n\
         IF [#1 EQ 5] THEN #3 = #2 + 1\n\
         G1 F1 X#2 Y#3\n",
    );
    assert_points(&ends(&q), &[[1.0, 2.0, 0.0]]);
}

#[test]
fn flow_errors_name_their_line() {
    // unknown jump target
    let (line, msg) = decode_error("G1 F100 X1\nGOTO 50\nX2\n");
    assert_eq!(line, 2);
    assert!(msg.contains("N50"), "{msg}");
    assert_eq!(decode_error("#1 = 1\nIF [#1] GOTO 7\n").0, 2);
    assert_eq!(decode_error("GOTO 1.5\nN1 G4 P0\n").0, 1);

    // a loop that is not closed is reported on its WHILE
    let (line, msg) = decode_error("G1 F100 X1\n#1 = 0\nWHILE [#1 LT 2] DO 1\nX2\n");
    assert_eq!(line, 3);
    assert!(msg.contains("END"), "{msg}");
    // an END without its WHILE
    assert_eq!(decode_error("G1 F100 X1\nEND 1\n").0, 2);
    // loops must nest
    assert_eq!(
        decode_error("WHILE [0] DO 1\nWHILE [0] DO 2\nEND 1\nEND 2\n").0,
        3
    );
}

#[test]
fn a_program_that_never_ends_is_stopped() {
    let cfg = DecoderConfig {
        max_executed_blocks: 1000,
        ..DecoderConfig::default()
    };
    let (line, msg) = decode_error_with(&cfg, "G1 F100 X1\nN10 X2\nX3\nGOTO 10\n");
    assert!((2..=4).contains(&line), "line {line}");
    assert!(msg.contains("does not end"), "{msg}");
    let (line, _) = decode_error_with(&cfg, "#1 = 1\nWHILE [#1] DO 1\nEND 1\n");
    assert!((2..=3).contains(&line), "line {line}");

    // the limit counts repeated blocks only: a long program without jumps
    // decodes whatever the limit
    let cfg = DecoderConfig {
        max_executed_blocks: 0,
        ..DecoderConfig::default()
    };
    let long: String = (1..=500).map(|i| format!("G1 F100 X{i}\n")).collect();
    assert_eq!(decode_with(&cfg, &long).len(), 500);
    // …and the default lets a real loop run
    let q = decode("#1 = 0\nWHILE [#1 LT 20000] DO 1\n#1 = #1 + 1\nEND 1\nG1 F1 X#1\n");
    assert_points(&ends(&q), &[[20000.0, 0.0, 0.0]]);
}

// ---------------------------------------------------------------------------
// Subprograms
// ---------------------------------------------------------------------------

#[test]
fn m98_calls_a_local_subprogram_and_m99_returns() {
    let q = decode(
        "G91 G1 F100\n\
         M98 P100 K3\n\
         G90 G1 X0\n\
         M30\n\
         N100 X5\n\
         M98 P200\n\
         M99\n\
         N200 Y1\n\
         M99\n",
    );
    assert_points(
        &ends(&q),
        &[
            [5.0, 0.0, 0.0],
            [5.0, 1.0, 0.0],
            [10.0, 1.0, 0.0],
            [10.0, 2.0, 0.0],
            [15.0, 2.0, 0.0],
            [15.0, 3.0, 0.0],
            [0.0, 3.0, 0.0],
        ],
    );
    // M98 / M99 are flow control, not events for the machine
    assert!(q.iter().all(|e| matches!(e, GeoElem::Move(_))));
}

#[test]
fn m98_details() {
    // K0 skips the call; the target may be computed
    let q = decode("#1 = 10\nG1 F100 X1\nM98 P#1 K0\nM98 P#1\nM2\nN10 X2\nM99\n");
    assert_points(&ends(&q), &[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]);
    // P stays the dwell time in a G4 block
    let q = decode("G4 P0.5\n#1 = 2\nG4 P#1\n");
    let dwells: Vec<f64> = q
        .iter()
        .filter_map(|e| match e {
            GeoElem::Dwell { seconds, .. } => Some(*seconds),
            _ => None,
        })
        .collect();
    assert_eq!(dwells, [0.5, 2.0]);
}

#[test]
fn subprogram_errors() {
    let (line, msg) = decode_error("G1 F100 X1\nM98\n");
    assert_eq!(line, 2);
    assert!(msg.contains('P'), "{msg}");
    let (line, msg) = decode_error("G1 F100 X1\nM98 P77\nM2\n");
    assert_eq!(line, 2);
    assert!(msg.contains("N77"), "{msg}");
    assert_eq!(decode_error("M98 P1.5\nM2\nN1 M99\n").0, 1);
    assert_eq!(decode_error("M98 P1 K-1\nM2\nN1 M99\n").0, 1);
    assert_eq!(decode_error("M98 P1 X5\nM2\nN1 M99\n").0, 1);
    // a return without a call: the main program ran into its subprogram
    let (line, msg) = decode_error("G1 F100 X1\nN10 X2\nM99\n");
    assert_eq!(line, 3);
    assert!(msg.contains("M99"), "{msg}");
    // a subprogram that does not return
    assert_eq!(decode_error("G1 F100 X1\nM98 P10\nN10 X2\n").0, 2);
    // endless recursion is cut off at the nesting limit
    let (line, msg) = decode_error("N1 M98 P1\n");
    assert_eq!(line, 1);
    assert!(msg.contains("nested"), "{msg}");
}

#[test]
fn calls_nest_to_the_documented_depth() {
    // sub k calls sub k + 1; the deepest one moves
    let chain = |depth: usize| {
        let mut src = String::from("G91 G1 F100\nM98 P1\nM2\n");
        for k in 1..depth {
            src += &format!("N{k} M98 P{}\nM99\n", k + 1);
        }
        src += &format!("N{depth} X1\nM99\n");
        src
    };
    let depth = rscnc::decoder::MAX_CALL_DEPTH;
    let q = Decoder::decode_source(DecoderConfig::default(), &chain(depth)).unwrap();
    assert_eq!(moves(&q).len(), 1);
    assert!(Decoder::decode_source(DecoderConfig::default(), &chain(depth + 1)).is_err());
}

// ---------------------------------------------------------------------------
// Plain programs are untouched
// ---------------------------------------------------------------------------

#[test]
fn plain_programs_decode_as_block_by_block() {
    let src = "%\n\
               N10 G1 F100 E500 E-800 X10 Y5 (comment) ; tail\n\
               N20 G3 X20 Y15 I0 J10 H1 O0.5 H-1 L-2\n\
               / G1 X999\n\
               G 0 1 X 3 0 . 5\n\
               G0 FF200 X0 Y0 M8 S1200\n\
               G4 P0.25\n\
               G92 X100\n\
               G91 G1 X5 T3\n\
               G2 I-5\n\
               M30\n\
               G1 X777\n";
    let direct = decode(src);
    let mut d = Decoder::new(DecoderConfig::default());
    let mut by_block = GeoQueue::new();
    for block in lex(src).unwrap() {
        d.decode_block(&block, &mut by_block).unwrap();
    }
    d.flush(&mut by_block).unwrap();
    assert_eq!(format!("{direct:?}"), format!("{by_block:?}"));
    assert_eq!(direct.len(), 9);
}

// ---------------------------------------------------------------------------
// Work offsets, G53, G92
// ---------------------------------------------------------------------------

fn offsets_cfg() -> DecoderConfig {
    let mut cfg = DecoderConfig::default();
    cfg.work_offsets[1] = [100.0, 50.0, 10.0]; // G55
    cfg.work_offsets[5] = [-20.0, 0.0, 5.0]; // G59
    cfg
}

#[test]
fn work_offsets_are_modal() {
    let cfg = offsets_cfg();
    let q = decode_with(
        &cfg,
        "G1 F100 X10 Y10\n\
         G55 X10 Y10 Z0\n\
         X20\n\
         G59 X0 Y0 Z0\n\
         G54 X1\n",
    );
    assert_points(
        &ends(&q),
        &[
            [10.0, 10.0, 0.0],
            [110.0, 60.0, 10.0],
            [120.0, 60.0, 10.0],
            [-20.0, 0.0, 5.0],
            [1.0, 0.0, 5.0],
        ],
    );
    // selecting an offset is not a move
    let mut d = Decoder::new(cfg);
    assert!(d.decode_chunk("G56\nG55\n").unwrap().is_empty());
    assert_eq!(d.modal_state().work_offset, 1);
}

#[test]
fn work_offsets_are_internal_units_and_incremental_moves_ignore_them() {
    let cfg = offsets_cfg();
    // inch programming: the offset stays 100 mm
    let q = decode_with(&cfg, "G70 G55 G1 F1 X1\n");
    assert_points(&ends(&q), &[[125.4, 0.0, 0.0]]);
    let q = decode_with(&cfg, "G91 G1 F100 X10\nG55 X10\n");
    assert_points(&ends(&q), &[[10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]);
}

#[test]
fn g53_is_machine_coordinates_for_one_block() {
    let cfg = offsets_cfg();
    let q = decode_with(&cfg, "G55 G1 F100 X1\nG53 X5\nX1\nG53 G0 X0 Y0\nY2\n");
    assert_points(
        &ends(&q),
        &[
            [101.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [101.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 52.0, 0.0],
        ],
    );
    // absolute also under G91, which stays in force afterwards
    let q = decode_with(&cfg, "G91 G1 F100 X10\nG53 X3\nX1\n");
    assert_points(
        &ends(&q),
        &[[10.0, 0.0, 0.0], [3.0, 0.0, 0.0], [4.0, 0.0, 0.0]],
    );
}

#[test]
fn g92_shift_lies_on_top_of_the_work_offset() {
    let cfg = offsets_cfg();
    let q = decode_with(
        &cfg,
        "G55 G1 F100 X10\n\
         G92 X0\n\
         X5\n\
         G54 X5\n\
         G53 X0\n\
         G55 X0\n",
    );
    assert_points(
        &ends(&q),
        &[
            [110.0, 0.0, 0.0], // G55: 100 + 10
            [115.0, 0.0, 0.0], // the point at machine 110 is now X0
            [15.0, 0.0, 0.0],  // the shift (+10) goes along into G54
            [0.0, 0.0, 0.0],   // machine coordinates ignore both
            [110.0, 0.0, 0.0],
        ],
    );
    // without work offsets G92 is what it was
    let q = decode("G1 F100 X10\nG92 X0\nX5\n");
    assert_points(&ends(&q), &[[10.0, 0.0, 0.0], [15.0, 0.0, 0.0]]);
}

#[test]
fn g53_reaches_the_auxiliary_axes_too() {
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        ..DecoderConfig::default()
    };
    let q = decode_with(&cfg, "G1 F100 X10 A90\nG92 A0\nX20 A10\nG53 X30 A10\n");
    let m = moves(&q);
    assert_eq!(m[1].aux_end, [100.0]);
    assert_eq!(m[2].aux_end, [10.0]);
}

// ---------------------------------------------------------------------------
// Tool length compensation
// ---------------------------------------------------------------------------

fn tools_cfg() -> DecoderConfig {
    DecoderConfig {
        tool_lengths: vec![(1, 50.0), (2, 75.5)],
        ..DecoderConfig::default()
    }
}

#[test]
fn g43_adds_the_latched_tool_length_along_z() {
    let cfg = tools_cfg();
    let q = decode_with(
        &cfg,
        "T1\n\
         G43\n\
         G1 F100 X5 Z10\n\
         T2\n\
         Z20\n\
         G43 Z20\n\
         G49 Z20\n\
         G43 T1 Z0\n",
    );
    assert_points(
        &ends(&q),
        &[
            [5.0, 0.0, 60.0], // T1: 10 + 50
            [5.0, 0.0, 70.0], // T2 selected, but T1's length is latched
            [5.0, 0.0, 95.5], // G43 again: T2's length
            [5.0, 0.0, 20.0], // cancelled
            [5.0, 0.0, 50.0], // the T word of the G43 block itself counts
        ],
    );
}

#[test]
fn g43_does_not_move_by_itself_and_leaves_g91_and_g53_alone() {
    let mut d = Decoder::new(tools_cfg());
    let q = d.decode_chunk("T2\nG43\n").unwrap();
    assert!(moves(&q).is_empty());
    assert_eq!(d.modal_state().tool_length, 75.5);
    let q = d
        .decode_chunk("G91 G1 F100 Z5\nG90 G53 Z1\nZ1\nG49\n")
        .unwrap();
    assert_points(
        &ends(&q),
        &[[0.0, 0.0, 5.0], [0.0, 0.0, 1.0], [0.0, 0.0, 76.5]],
    );
    assert_eq!(d.modal_state().tool_length, 0.0);
}

#[test]
fn g43_needs_a_tool_table_entry() {
    let cfg = tools_cfg();
    let (line, msg) = decode_error_with(&cfg, "G1 F100 X1\nT7\nG43 Z5\n");
    assert_eq!(line, 3);
    assert!(msg.contains("T7"), "{msg}");
    // no tool selected at all, no table at all
    assert_eq!(decode_error_with(&cfg, "G43\n").0, 1);
    assert_eq!(decode_error("T1\nG43\n").0, 2);
    assert_eq!(decode_error_with(&cfg, "G1 F1 X1\nG43 T3\n").0, 2);
}

#[test]
fn g92_accounts_for_the_tool_length() {
    // with T1 (50) active the tool tip at machine Z60 is at programmed Z10;
    // G92 Z0 there makes the tip's place Z0
    let q = decode_with(&tools_cfg(), "T1\nG43\nG1 F100 Z10\nG92 Z0\nZ5\nG49 Z5\n");
    assert_points(
        &ends(&q),
        &[[0.0, 0.0, 60.0], [0.0, 0.0, 65.0], [0.0, 0.0, 15.0]],
    );
}

// ---------------------------------------------------------------------------
// Working area
// ---------------------------------------------------------------------------

#[test]
fn configured_working_area_limits_every_move() {
    let cfg = DecoderConfig {
        work_area_min: [0.0, 0.0, -50.0],
        work_area_max: [200.0, 100.0, 0.0],
        ..DecoderConfig::default()
    };
    assert_eq!(
        decode_with(&cfg, "G1 F100 X200 Y100 Z-50\nG0 X0 Y0 Z0\n").len(),
        2
    );
    let (line, msg) = decode_error_with(&cfg, "G1 F100 X100 Y50\nX250\n");
    assert_eq!(line, 2);
    assert!(
        msg.contains('X') && msg.contains("250") && msg.contains("200"),
        "{msg}"
    );
    assert!(msg.contains("upper"), "{msg}");
    let (line, msg) = decode_error_with(&cfg, "G1 F100 X100 Y50\n\nG0 Z-50.5\n");
    assert_eq!(line, 3);
    assert!(
        msg.contains('Z') && msg.contains("-50.5") && msg.contains("lower"),
        "{msg}"
    );
    // the default is no limit at all
    assert_eq!(decode("G1 F100 X1e9 Y-1e9\n").len(), 1);
}

#[test]
fn g25_and_g26_set_the_area_from_the_program() {
    let mut d = Decoder::new(DecoderConfig::default());
    let q = d
        .decode_chunk("G25 X0 Y0\nG26 X100 Y100\nG1 F100 X50 Y50\n")
        .unwrap();
    // the limit blocks do not move
    assert_eq!(q.len(), 1);
    let (lo, hi) = d.work_area();
    assert_eq!(lo, [0.0, 0.0, f64::NEG_INFINITY]);
    assert_eq!(hi, [100.0, 100.0, f64::INFINITY]);
    let err = d.decode_chunk("X80\nX101\n").unwrap_err();
    match err {
        CncError::Decode(e) => {
            assert_eq!(e.line, 2);
            assert!(e.message.contains('X'), "{}", e.message);
        }
        other => panic!("unexpected error {other}"),
    }
    // widening the area lets the same move through; Z was never limited
    let q = d.decode_chunk("G26 X150\nX101 Z-1000\n").unwrap();
    assert_eq!(q.len(), 1);
}

#[test]
fn the_area_check_sees_an_arcs_bulge() {
    // both ends of the arc lie on Y0, inside; the CCW half circle from
    // (10,0) to (30,0) dips to Y-10
    let src = "G25 Y-1\nG1 F100 X10\nG3 X30 Y0 R10\n";
    let (line, msg) = decode_error(src);
    assert_eq!(line, 3);
    assert!(msg.contains('Y') && msg.contains("-10"), "{msg}");
    // the CW one bulges the other way and passes
    assert_eq!(decode("G25 Y-1\nG1 F100 X10\nG2 X30 Y0 R10\n").len(), 2);
    // a line that ends outside
    assert_eq!(decode_error("G25 Y-1\nG1 F100 X10\nX30 Y-1.5\n").0, 3);
}

#[test]
fn limits_are_programmed_in_the_active_coordinate_system() {
    let cfg = offsets_cfg();
    let mut d = Decoder::new(cfg);
    d.decode_chunk("G55\nG25 X0 Y-5\nG26 X50\n").unwrap();
    let (lo, hi) = d.work_area();
    assert_eq!((lo[0], lo[1], hi[0]), (100.0, 45.0, 150.0));
    // with G53 the values are machine coordinates; inches are converted
    d.decode_chunk("G53 G25 X5\nG70 G53 G26 X10\nG71\n")
        .unwrap();
    let (lo, hi) = d.work_area();
    assert_eq!(lo[0], 5.0);
    assert!((hi[0] - 254.0).abs() < 1e-9);
    // G91 does not make a limit a distance
    d.decode_chunk("G91\nG53 G26 X300\nG90\n").unwrap();
    assert_eq!(d.work_area().1[0], 300.0);
}

#[test]
fn working_area_block_errors() {
    assert_eq!(decode_error("G1 F100 X1\nG25\n").0, 2);
    assert_eq!(decode_error("G25 X0 G26 X10\n").0, 1);
    assert_eq!(decode_error("G25 X0 G4 P1\n").0, 1);
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        ..DecoderConfig::default()
    };
    assert_eq!(decode_error_with(&cfg, "G26 X10 A5\n").0, 1);
    // an area turned inside out refuses the next move
    let (line, msg) = decode_error("G25 X10\nG26 X5\nG1 F100 X7\n");
    assert_eq!(line, 3);
    assert!(msg.contains("empty"), "{msg}");
}

// ---------------------------------------------------------------------------
// G5 splines
// ---------------------------------------------------------------------------

fn is_spline(m: &GeoMove) -> bool {
    matches!(m.geo, Geometry::Spline(_))
}

fn dot(a: PathPoint, b: PathPoint) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The spans of one spline run: they chain through the given points, and the
/// tangent is continuous where two spans meet.
fn assert_spline_through(spans: &[&GeoMove], start: PathPoint, points: &[PathPoint]) {
    assert_eq!(spans.len(), points.len());
    let mut from = start;
    for (k, (span, to)) in spans.iter().zip(points).enumerate() {
        assert!(is_spline(span), "span {k} is {:?}", span.geo);
        assert_point(span.geo.start(), from);
        assert_point(span.geo.end(), *to);
        assert_point(span.geo.point_at(0.0), from);
        assert_point(span.geo.point_at(span.geo.length()), *to);
        // a curve is never shorter than its chord
        let chord = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
        assert!(span.geo.length() >= dot(chord, chord).sqrt() - 1e-9);
        from = *to;
    }
    for (k, w) in spans.windows(2).enumerate() {
        let t_out = w[0].geo.tangent_at(w[0].geo.length());
        let t_in = w[1].geo.tangent_at(0.0);
        assert!(
            dot(t_out, t_in) > 1.0 - 1e-9,
            "tangent breaks after span {k}: {t_out:?} → {t_in:?}"
        );
    }
}

#[test]
fn g5_runs_a_spline_through_the_programmed_points() {
    let points = [
        [10.0, 0.0, 0.0],
        [20.0, 10.0, 0.0],
        [30.0, 0.0, 2.0],
        [40.0, 10.0, 2.0],
        [50.0, 0.0, 0.0],
    ];
    let q = decode(
        "G5 F50 X10 Y0\n\
         X20 Y10\n\
         X30 Y0 Z2\n\
         X40 Y10\n\
         X50 Y0 Z0\n\
         G1 X60\n",
    );
    let m = moves(&q);
    assert_eq!(m.len(), 6);
    assert_spline_through(&m[..5], [0.0; 3], &points);
    assert!(matches!(m[5].geo, Geometry::Line(_)));
    for (k, span) in m[..5].iter().enumerate() {
        assert_eq!(span.src_line, k + 1);
        assert_eq!(span.obj_no, k as u32);
        assert!(!span.is_rapid);
        assert!((span.feed - 50.0).abs() < 1e-12);
    }
    // natural ends: the curve starts and ends without curvature…
    assert!(m[0].geo.curvature_at(0.0) < 1e-9);
    assert!(m[4].geo.curvature_at(m[4].geo.length()) < 1e-9);
    // …and it does curve in between, most at the peaks of the zigzag
    assert!(m[1].geo.curvature_at(m[1].geo.length()) > 1e-2);
    assert!(m[2].geo.curvature_at(0.0) > 1e-2);
    // the tangent at an inner point of evenly spaced points is the chord
    // of its two neighbours (the classic Catmull-Rom tangent): along X at
    // the peaks (20,10) and (30,0) of a zigzag in the plane
    let q = decode("G5 F50 X10 Y0\nX20 Y10\nX30 Y0\nX40 Y10\n");
    let m = moves(&q);
    for k in [1, 2] {
        let t = m[k].geo.tangent_at(m[k].geo.length());
        assert!(dot(t, [1.0, 0.0, 0.0]) > 1.0 - 1e-9, "span {k}: {t:?}");
    }
    // with unequal chords the two directions are weighted, the shorter chord
    // counting more: a (3,4) chord of 5 and a (10,0) chord of 10 meet with
    // the direction (2·(0.6, 0.8) + (1, 0)) / 3
    let q = decode("G5 F50 X3 Y4\nX13 Y4\nX20 Y0\n");
    let m = moves(&q);
    let t = m[0].geo.tangent_at(m[0].geo.length());
    let want = [2.2 / 3.0, 1.6 / 3.0, 0.0];
    let n = dot(want, want).sqrt();
    assert!(dot(t, want) / n > 1.0 - 1e-9, "{t:?}");
}

#[test]
fn g5_keeps_the_tangent_continuous_with_uneven_point_spacing() {
    let points = [
        [1.0, 0.0, 0.0],
        [1.5, 0.5, 0.0],
        [30.0, 1.0, 0.0],
        [30.0, 40.0, 5.0],
        [29.0, 40.5, 5.0],
        [-10.0, 20.0, 0.0],
    ];
    let src: String = points
        .iter()
        .map(|p| format!("X{} Y{} Z{}\n", p[0], p[1], p[2]))
        .collect();
    let q = decode(&format!("G5 F20\n{src}"));
    let m = moves(&q);
    assert_spline_through(&m, [0.0; 3], &points);
    // chord-length tangents keep a short span next to a long one from
    // overshooting: no span is wildly longer than its chord
    for (span, w) in m.iter().zip(
        [[0.0; 3]]
            .iter()
            .chain(&points)
            .collect::<Vec<_>>()
            .windows(2),
    ) {
        let c = [w[1][0] - w[0][0], w[1][1] - w[0][1], w[1][2] - w[0][2]];
        assert!(span.geo.length() < 1.3 * dot(c, c).sqrt(), "{:?}", span.geo);
    }
}

#[test]
fn a_single_g5_point_is_a_line() {
    let q = decode("G5 F50 X10 Y5\n");
    let m = moves(&q);
    assert_eq!(m.len(), 1);
    assert!(matches!(m[0].geo, Geometry::Line(_)));
    assert_point(m[0].geo.end(), [10.0, 5.0, 0.0]);
    // two points are a curve already
    let q = decode("G5 F50 X10 Y5\nX20 Y0\n");
    assert!(moves(&q).iter().all(|m| is_spline(m)));
}

#[test]
fn a_spline_run_ends_at_other_elements() {
    // another motion mode, a dwell, a G92 and a working-area block each close
    // the run: the curves on their two sides are separate
    for breaker in ["G1 X25 Y5", "G4 P0.1", "G92 X20", "G26 X1000"] {
        let q = decode(&format!(
            "G5 F50 X10\nX20 Y5\n{breaker}\nG5 X30 Y0\nX40 Y5\nX50 Y0\n"
        ));
        let m: Vec<_> = moves(&q).into_iter().filter(|m| is_spline(m)).collect();
        assert_eq!(m.len(), 5, "{breaker}");
        // a run's first and last span are straight at the outer ends
        for (span, at_end) in [(m[0], false), (m[1], true), (m[2], false), (m[4], true)] {
            let s = if at_end { span.geo.length() } else { 0.0 };
            assert!(span.geo.curvature_at(s) < 1e-9, "{breaker}");
        }
    }
    // modal words alone do not: a bare F between two points keeps one curve
    let q = decode("G5 F50 X10\nX20 Y5\nF20\nX30 Y0\nX40 Y5\n");
    let m = moves(&q);
    assert_spline_through(
        &m,
        [0.0; 3],
        &[
            [10.0, 0.0, 0.0],
            [20.0, 5.0, 0.0],
            [30.0, 0.0, 0.0],
            [40.0, 5.0, 0.0],
        ],
    );
    assert_eq!(
        m.iter().map(|m| m.feed).collect::<Vec<_>>(),
        [50.0, 50.0, 20.0, 20.0]
    );
}

#[test]
fn an_event_in_a_g5_block_follows_its_point() {
    let q = decode("G5 F50 X10\nX20 Y5 M8\nX30\nX40 Y5\nM30\n");
    let kinds: Vec<&str> = q
        .iter()
        .map(|e| match e {
            GeoElem::Move(m) if is_spline(m) => "spline",
            GeoElem::Move(_) => "line",
            GeoElem::Event(_) => "event",
            GeoElem::Dwell { .. } => "dwell",
        })
        .collect();
    assert_eq!(kinds, ["spline", "spline", "event", "spline", "spline"]);
    let m = moves(&q);
    assert_point(m[1].geo.end(), [20.0, 5.0, 0.0]);
    assert_eq!(
        m.iter().map(|m| m.src_line).collect::<Vec<_>>(),
        [1, 2, 3, 4]
    );
}

#[test]
fn g5_spans_carry_their_blocks_words() {
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        ..DecoderConfig::default()
    };
    let q = decode_with(
        &cfg,
        "G5 F50 E100 X10 A10 H1 O0.5\n\
         H7\n\
         X20 Y5 A30 E-200 H-1 L-1\n\
         F20 FA1 X30 Y0 A31 H2 L2\n",
    );
    let m = moves(&q);
    assert_eq!(m.len(), 3);
    // auxiliary axes run linearly over each span
    assert_eq!(
        (m[0].aux_start.clone(), m[0].aux_end.clone()),
        (vec![0.0], vec![10.0])
    );
    assert_eq!(
        (m[1].aux_start.clone(), m[1].aux_end.clone()),
        (vec![10.0], vec![30.0])
    );
    assert_eq!(
        (m[2].aux_start.clone(), m[2].aux_end.clone()),
        (vec![30.0], vec![31.0])
    );
    // limits as they stood at each block
    assert_eq!((m[0].accel, m[0].decel), (Some(100.0), None));
    assert_eq!((m[1].accel, m[1].decel), (Some(100.0), Some(200.0)));
    // FA1: one unit of A per second over the span's length
    assert!(
        (m[2].feed - m[2].geo.length()).abs() < 1e-9,
        "{}",
        m[2].feed
    );
    // switch points sit on the span of their block, measured along the curve;
    // one from a block without a point rides on the next span, at its start
    let sp = |m: &GeoMove| -> Vec<(i32, f64)> {
        m.switch_points
            .iter()
            .map(|s| (s.number, s.position))
            .collect()
    };
    let len = |k: usize| m[k].geo.length();
    assert_eq!(sp(m[0]), [(1, 0.5 * len(0))]);
    assert_eq!(sp(m[1]), [(7, 0.0), (-1, len(1) - 1.0)]);
    assert_eq!(sp(m[2]), [(2, 2.0)]);
}

#[test]
fn repeated_g5_points_are_skipped() {
    let q = decode("G5 F50 X0 Y0\nX10\nX10\nX20 Y5\nX20 Y5\n");
    let m = moves(&q);
    assert_spline_through(&m, [0.0; 3], &[[10.0, 0.0, 0.0], [20.0, 5.0, 0.0]]);
}

#[test]
fn an_auxiliary_move_in_place_splits_a_spline_run() {
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        ..DecoderConfig::default()
    };
    let q = decode_with(&cfg, "G5 F50 X10\nX20 Y5\nA90\nX30 Y0\nX40 Y5\n");
    let m = moves(&q);
    assert_eq!(m.len(), 5);
    assert!(is_spline(m[0]) && is_spline(m[1]) && is_spline(m[3]) && is_spline(m[4]));
    assert!(m[2].geo.length() < 1e-12);
    assert_eq!(
        (m[2].aux_start.clone(), m[2].aux_end.clone()),
        (vec![0.0], vec![90.0])
    );
}

#[test]
fn g5_points_can_come_from_a_loop() {
    // eight points on a circle of radius 10 around the start
    let q = decode(
        "G1 F50 X10\n\
         G5\n\
         #1 = 0\n\
         WHILE [#1 LT 8] DO 1\n\
         #1 = #1 + 1\n\
         X[10 * COS[#1 * 45]] Y[10 * SIN[#1 * 45]]\n\
         END 1\n",
    );
    let m = moves(&q);
    assert_eq!(m.len(), 9);
    let spans = &m[1..];
    let points: Vec<PathPoint> = (1..=8)
        .map(|k| {
            let a = (k as f64 * 45.0).to_radians();
            [10.0 * a.cos(), 10.0 * a.sin(), 0.0]
        })
        .collect();
    assert_spline_through(spans, [10.0, 0.0, 0.0], &points);
    // an inner span stays close to the circle it samples
    let mid = spans[3].geo.point_at(spans[3].geo.length() / 2.0);
    let r = (mid[0] * mid[0] + mid[1] * mid[1]).sqrt();
    assert!((r - 10.0).abs() < 0.1, "radius {r}");
}

#[test]
fn g5_respects_offsets_and_the_working_area() {
    let mut cfg = offsets_cfg();
    let q = decode_with(&cfg, "G55 G5 F50 X0 Y0 Z0\nX10 Y10\nG53 X0 Y0 Z0\n");
    assert_points(
        &ends(&q),
        &[[100.0, 50.0, 10.0], [110.0, 60.0, 10.0], [0.0, 0.0, 0.0]],
    );
    // the points are inside, but the curve swings out between them
    cfg.work_area_min = [-0.5, -100.0, -100.0];
    let src = "G5 F50 X0 Y10\nX10 Y20\nX0 Y30\nX10 Y40\n";
    assert_eq!(decode(src).len(), 4);
    let (line, msg) = decode_error_with(&cfg, src);
    assert!((1..=4).contains(&line));
    assert!(msg.contains('X') && msg.contains("lower"), "{msg}");
    // a point outside is refused as well
    cfg.work_area_min = [-1000.0; 3];
    cfg.work_area_max = [1000.0, 35.0, 1000.0];
    let (line, msg) = decode_error_with(&cfg, src);
    assert_eq!(line, 4);
    assert!(msg.contains('Y') && msg.contains("upper"), "{msg}");
}

#[test]
fn g5_block_errors() {
    // no feed
    let (line, msg) = decode_error("G5 X10\n");
    assert_eq!(line, 1);
    assert!(msg.contains("feed"), "{msg}");
    // arc words mean nothing on a spline point
    assert_eq!(decode_error("G5 F50 X10\nX20 I5\n").0, 2);
    assert_eq!(decode_error("G5 F50 X10 R5\n").0, 1);
    // a curve cannot turn straight back; the error names the turning point
    let (line, msg) = decode_error("G5 F50 X10\nX20\nX5\nX30 Y1\n");
    assert_eq!(line, 2);
    assert!(msg.contains("turns straight back"), "{msg}");
    assert_eq!(decode_error("G5 F50 X10\nX0\n").0, 1);
    // …while a hairpin is a curve, if a tight one
    let q = decode("G5 F50 X10\nX10 Y0.001\nX0 Y0.001\n");
    assert_eq!(moves(&q).len(), 3);
}

#[test]
fn g5_points_on_a_line_give_straight_spans() {
    let q = decode("G5 F50 X10 Y5\nX20 Y10\nX50 Y25\n");
    let m = moves(&q);
    assert_eq!(m.len(), 3);
    for (span, len) in m.iter().zip([125f64.sqrt(), 125f64.sqrt(), 1125f64.sqrt()]) {
        assert!((span.geo.length() - len).abs() < 1e-9);
        assert!(span.geo.max_curvature() < 1e-9);
    }
}

#[test]
fn decode_block_users_flush_the_spline_themselves() {
    let mut d = Decoder::new(DecoderConfig::default());
    let mut q = GeoQueue::new();
    for block in lex("G5 F50 X10\nX20 Y5\nX30 Y0\n").unwrap() {
        d.decode_block(&block, &mut q).unwrap();
    }
    // the points wait for the end of their run
    assert!(q.is_empty());
    assert_eq!(d.modal_state().position, [30.0, 0.0, 0.0]);
    d.flush(&mut q).unwrap();
    assert_eq!(q.len(), 3);
    d.flush(&mut q).unwrap();
    assert_eq!(q.len(), 3);
    d.finish().unwrap();
}

// ---------------------------------------------------------------------------
// Chunked decoding
// ---------------------------------------------------------------------------

#[test]
fn chunks_continue_the_modal_state() {
    let mut d = Decoder::new(offsets_cfg());
    let a = d.decode_chunk("G55 G91 G1 F50 X10\n#1 = 5\n").unwrap();
    let b = d.decode_chunk("X#1\n").unwrap();
    let c = d.decode_chunk("G90 X0 Y0\n(nothing)\n").unwrap();
    let d_ = d.decode_chunk("").unwrap();
    assert_points(&ends(&a), &[[10.0, 0.0, 0.0]]);
    assert_points(&ends(&b), &[[15.0, 0.0, 0.0]]);
    assert_points(&ends(&c), &[[100.0, 50.0, 0.0]]);
    assert!(d_.is_empty());
    // feed and object numbers run on
    let all: Vec<&GeoMove> = moves(&a)
        .into_iter()
        .chain(moves(&b))
        .chain(moves(&c))
        .collect();
    assert!(all.iter().all(|m| (m.feed - 50.0).abs() < 1e-12));
    assert_eq!(all.iter().map(|m| m.obj_no).collect::<Vec<_>>(), [0, 1, 2]);
    d.finish().unwrap();

    // in one piece or in three: the same elements
    let whole = decode_with(
        &offsets_cfg(),
        "G55 G91 G1 F50 X10\n#1 = 5\nX#1\nG90 X0 Y0\n",
    );
    assert_points(&ends(&whole), &[ends(&a), ends(&b), ends(&c)].concat());
}

#[test]
fn a_failed_chunk_changes_nothing() {
    let mut d = Decoder::new(DecoderConfig::default());
    d.decode_chunk("#1 = 1\nG1 F50 X10\n").unwrap();
    let err = d
        .decode_chunk("#1 = 2\nG91 X20 H3\nG5 X30\nG33\n")
        .unwrap_err();
    // line numbers count from the start of the chunk
    assert!(
        matches!(err, CncError::Decode(ref e) if e.line == 4),
        "{err}"
    );
    assert_eq!(d.variable(1), Some(1.0));
    assert_eq!(d.modal_state().position, [10.0, 0.0, 0.0]);
    // still G90 G1, nothing pending: the corrected text decodes from X10
    let q = d.decode_chunk("X20\n").unwrap();
    let m = moves(&q);
    assert_eq!(m.len(), 1);
    assert_point(m[0].geo.start(), [10.0, 0.0, 0.0]);
    assert_point(m[0].geo.end(), [20.0, 0.0, 0.0]);
    assert_eq!(m[0].obj_no, 1);
    assert!(m[0].switch_points.is_empty());
    d.finish().unwrap();
    // a syntax error anywhere refuses the whole chunk, too
    let err = d.decode_chunk("X30\nX[1 +\n").unwrap_err();
    assert!(
        matches!(err, CncError::Gcode(ref e) if e.line == 2),
        "{err}"
    );
    assert_eq!(d.modal_state().position, [20.0, 0.0, 0.0]);
}

#[test]
fn a_chunk_closes_its_spline_and_g5_stays_modal() {
    let mut d = Decoder::new(DecoderConfig::default());
    let a = d.decode_chunk("G5 F50 X10\nX20 Y5\n").unwrap();
    let m = moves(&a);
    assert_eq!(m.len(), 2);
    assert!(m.iter().all(|m| is_spline(m)));
    // closed: the last span ends straight
    assert!(m[1].geo.curvature_at(m[1].geo.length()) < 1e-9);
    // the next piece starts a new curve from where the last one ended
    let b = d.decode_chunk("X30 Y0\nX40 Y5\n").unwrap();
    let m = moves(&b);
    assert_spline_through(&m, [20.0, 5.0, 0.0], &[[30.0, 0.0, 0.0], [40.0, 5.0, 0.0]]);
    assert!(m[0].geo.curvature_at(0.0) < 1e-9);
}

#[test]
fn a_switch_point_waits_across_the_seam() {
    let mut d = Decoder::new(DecoderConfig::default());
    assert_eq!(d.decode_chunk("G1 F50 X10\nH4\n").unwrap().len(), 1);
    // nothing to ride on yet
    assert!(d.finish().is_err());
    let q = d.decode_chunk("X20\n").unwrap();
    let m = moves(&q);
    assert_eq!(m[0].switch_points.len(), 1);
    assert_eq!(
        (m[0].switch_points[0].number, m[0].switch_points[0].position),
        (4, 0.0)
    );
    d.finish().unwrap();
    // decode_source keeps reporting a switch point nothing follows
    let (line, msg) = decode_error("G1 F50 X10\nH4\n");
    assert_eq!(line, 2);
    assert!(msg.contains("H4"), "{msg}");
}

#[test]
fn text_after_the_program_end_is_ignored_until_resume() {
    let mut d = Decoder::new(DecoderConfig::default());
    let q = d.decode_chunk("#1 = 7\nG1 F50 X10\nM30\nX20\n").unwrap();
    assert_eq!(moves(&q).len(), 1);
    assert!(d.program_ended());
    assert!(d.decode_chunk("X30\n").unwrap().is_empty());
    assert_eq!(d.modal_state().position, [10.0, 0.0, 0.0]);
    d.resume();
    assert!(!d.program_ended());
    let q = d.decode_chunk("X#1\n").unwrap();
    assert_points(&ends(&q), &[[7.0, 0.0, 0.0]]);
}

#[test]
fn jump_targets_are_local_to_a_chunk() {
    let mut d = Decoder::new(DecoderConfig::default());
    d.decode_chunk("N10 G1 F50 X1\n").unwrap();
    let err = d.decode_chunk("X2\nGOTO 10\n").unwrap_err();
    assert!(
        matches!(err, CncError::Decode(ref e) if e.line == 2),
        "{err}"
    );
    let err = d.decode_chunk("M98 P10\n").unwrap_err();
    assert!(
        matches!(err, CncError::Decode(ref e) if e.line == 1),
        "{err}"
    );
    // a loop cannot be left open for the next chunk
    assert!(d.decode_chunk("WHILE [1] DO 1\n").is_err());
    // …but the same numbers can be used again in every chunk
    let q = d.decode_chunk("GOTO 10\nX99\nN10 X3\n").unwrap();
    assert_points(&ends(&q), &[[3.0, 0.0, 0.0]]);
}

// ---------------------------------------------------------------------------
// Through the pipeline
// ---------------------------------------------------------------------------

#[test]
fn a_looped_program_runs_through_the_interpolator() {
    // a zigzag from a loop and a subprogram, with a work offset
    let mut cfg = DecoderConfig::default();
    cfg.work_offsets[1] = [5.0, 5.0, 0.0];
    let queue = Decoder::decode_source(
        cfg,
        "G55 G1 F40 X0 Y0\n\
         #1 = 0\n\
         WHILE [#1 LT 4] DO 1\n\
         M98 P100\n\
         #1 = #1 + 1\n\
         END 1\n\
         G53 G0 X0 Y0\n\
         M30\n\
         N100 G91 G1 X5 Y[2 - 4 * [#1 MOD 2]]\n\
         G90\n\
         M99\n",
    )
    .unwrap();
    assert_eq!(queue.len(), 6);
    let limits = PathLimits::new(50.0, 500.0, 5000.0, 500.0);
    let mut ipl = Interpolator::new(limits, LookAhead::default(), Vec::new(), 0.001);
    ipl.load(queue).unwrap();
    let mut furthest: f64 = 0.0;
    let mut last = [f64::NAN; 3];
    for _ in 0..200_000 {
        let out = ipl.cycle().unwrap();
        furthest = furthest.max(out.path_position[0]);
        last = out.path_position;
        if out.state == IplState::Finished {
            // (sampled once a cycle, so the far corner is met only closely)
            assert!((furthest - 25.0).abs() < 0.01, "furthest X {furthest}");
            assert_point(last, [0.0, 0.0, 0.0]);
            return;
        }
    }
    panic!("program did not finish, last position {last:?}");
}
