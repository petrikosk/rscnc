//! Feeding a program in pieces: `Interpolator::append`,
//! `Interpolator::discard_history` and `CncController::append_program`.

use rscnc::prelude::*;

const DT: f64 = 0.001;
const A_MAX: f64 = 500.0;
const J_MAX: f64 = 5000.0;

fn limits() -> PathLimits {
    PathLimits::new(50.0, A_MAX, J_MAX, 500.0)
}

fn interpolator() -> Interpolator {
    Interpolator::new(limits(), LookAhead::default(), Vec::new(), DT)
}

/// Decodes a program piece by piece, keeping the modal state.
struct Feeder(Decoder);

impl Feeder {
    fn new() -> Self {
        Feeder(Decoder::new(DecoderConfig::default()))
    }

    fn piece(&mut self, text: &str) -> GeoQueue {
        self.0.decode_chunk(text).expect("piece decodes")
    }
}

/// The dynamic limits hold from one cycle to the next and the path never
/// runs backward.
fn check_motion(outputs: &[IplOutput]) {
    for (i, w) in outputs.windows(2).enumerate() {
        let (a, b) = (&w[0], &w[1]);
        assert!(b.path_velocity >= 0.0, "cycle {i}: negative velocity");
        assert!(
            b.path_velocity <= 50.0 * (1.0 + 1e-9),
            "cycle {i}: velocity {}",
            b.path_velocity
        );
        assert!(
            b.path_acceleration.abs() <= A_MAX * (1.0 + 1e-6),
            "cycle {i}: acceleration {}",
            b.path_acceleration
        );
        let jerk = (b.path_acceleration - a.path_acceleration).abs() / DT;
        assert!(jerk <= J_MAX * (1.0 + 1e-6), "cycle {i}: jerk {jerk}");
        assert!(
            b.s_total >= a.s_total - 1e-12,
            "cycle {i}: path position went back"
        );
        // the position follows the velocity (no jump in the path position)
        let step = b.s_total - a.s_total;
        let bound = a.path_velocity.max(b.path_velocity) * DT + A_MAX * DT * DT;
        assert!(step <= bound, "cycle {i}: step {step} at {bound}");
    }
}

fn run_to_end(ipl: &mut Interpolator, outputs: &mut Vec<IplOutput>) {
    for _ in 0..400_000 {
        let out = ipl.cycle().expect("cycle succeeds");
        let done = out.state == IplState::Finished;
        outputs.push(out);
        if done {
            return;
        }
    }
    panic!("program did not finish");
}

#[test]
fn pieces_appended_ahead_run_without_stopping_at_the_seams() {
    let mut feed = Feeder::new();
    let mut ipl = interpolator();
    ipl.load(feed.piece("G1 F20 X10\n")).unwrap();
    ipl.append(feed.piece("X20\nX25 Y5\n")).unwrap();
    ipl.append(feed.piece("X40 Y5\n")).unwrap();
    assert!(!ipl.append_pending(), "taken into the plan at standstill");
    assert_eq!(ipl.planned_segments().len(), 4);
    assert!((ipl.total_length() - (20.0 + 50f64.sqrt() + 15.0)).abs() < 1e-9);
    // the straight seam is crossed at full feed
    assert!((ipl.planned_segments()[0].v_exit - 20.0).abs() < 1e-9);

    let mut outputs = Vec::new();
    run_to_end(&mut ipl, &mut outputs);
    check_motion(&outputs);
    let last = outputs.last().unwrap();
    assert!((last.path_position[0] - 40.0).abs() < 1e-9);
    assert!((last.path_position[1] - 5.0).abs() < 1e-9);
    assert!((last.progress - 1.0).abs() < 1e-9);
    // Across the seam at X10 the path keeps its feed.
    for out in outputs.iter().filter(|o| {
        let x = o.path_position[0];
        o.path_position[1] == 0.0 && (5.0..15.0).contains(&x)
    }) {
        assert!(
            out.path_velocity > 20.0 - 1e-6,
            "slowed down to {} at X{}",
            out.path_velocity,
            out.path_position[0]
        );
    }
}

#[test]
fn a_piece_appended_in_motion_is_picked_up_without_a_stop() {
    let mut feed = Feeder::new();
    let mut ipl = interpolator();
    ipl.load(feed.piece("G1 F40 X30\n")).unwrap();
    let mut outputs = Vec::new();
    while outputs.last().is_none_or(|o: &IplOutput| o.s_total < 8.0) {
        outputs.push(ipl.cycle().unwrap());
    }
    assert!(outputs.last().unwrap().path_velocity > 39.9);
    ipl.append(feed.piece("X60\n")).unwrap();
    assert!(!ipl.append_pending(), "cruising: the new plan is in use");
    assert!((ipl.remaining_length() - (60.0 - outputs.last().unwrap().s_total)).abs() < 1e-9);
    let appended_at = outputs.len();
    run_to_end(&mut ipl, &mut outputs);
    check_motion(&outputs);
    assert!((outputs.last().unwrap().path_position[0] - 60.0).abs() < 1e-9);
    // No slowing down for the old end of the program at X30.
    for out in &outputs[appended_at..] {
        if out.path_position[0] < 50.0 {
            assert!(out.path_velocity > 39.9, "{}", out.path_velocity);
        }
    }
}

#[test]
fn a_piece_appended_at_any_moment_of_the_final_ramp_is_safe() {
    // The old plan brakes to a stop at X12. Whenever the piece arrives, the
    // path continues to X40: over to the new plan when that can be reached
    // from the state the path is in, and otherwise through the stop.
    let whole = {
        let mut ipl = interpolator();
        ipl.load(Feeder::new().piece("G1 F40 X12\n")).unwrap();
        let mut outputs = Vec::new();
        run_to_end(&mut ipl, &mut outputs);
        outputs.len()
    };
    let mut stopped_runs = 0;
    for at in (1..whole + 30).step_by(7) {
        let mut feed = Feeder::new();
        let mut ipl = interpolator();
        ipl.load(feed.piece("G1 F40 X12\n")).unwrap();
        let mut outputs = Vec::new();
        for _ in 0..at {
            outputs.push(ipl.cycle().unwrap());
        }
        ipl.append(feed.piece("X40\n")).unwrap();
        run_to_end(&mut ipl, &mut outputs);
        check_motion(&outputs);
        assert!(!ipl.append_pending());
        let last = outputs.last().unwrap();
        assert!(
            (last.path_position[0] - 40.0).abs() < 1e-9,
            "append at {at}"
        );
        assert!((last.s_total - 40.0).abs() < 1e-9);
        let stopped = outputs[at..]
            .iter()
            .any(|o| o.path_velocity < 0.5 && o.s_total > 1.0 && o.s_total < 39.0);
        stopped_runs += usize::from(stopped);
        if at < whole / 2 {
            assert!(!stopped, "append at {at} (of {whole}) stopped the path");
        }
    }
    assert!(stopped_runs > 0, "no late append was exercised");
}

#[test]
fn appending_to_a_finished_program_starts_it_again() {
    let mut feed = Feeder::new();
    let mut ipl = interpolator();
    ipl.load(feed.piece("G1 F30 X5\n")).unwrap();
    let mut outputs = Vec::new();
    run_to_end(&mut ipl, &mut outputs);
    assert_eq!(ipl.remaining_length(), 0.0);
    ipl.append(feed.piece("M8\nX10\nG4 P0.05\nX12\n")).unwrap();
    assert_eq!(ipl.state(), IplState::Running);
    let from = outputs.len();
    run_to_end(&mut ipl, &mut outputs);
    check_motion(&outputs);
    assert!((outputs.last().unwrap().path_position[0] - 12.0).abs() < 1e-9);
    let events: Vec<_> = outputs[from..]
        .iter()
        .flat_map(|o| o.events.iter().flat_map(|e| e.m.clone()))
        .collect();
    assert_eq!(events, [8]);
    assert!(outputs[from..]
        .iter()
        .any(|o| o.state == IplState::Dwelling));
}

#[test]
fn switch_points_of_a_piece_appended_after_a_turn_at_the_end_are_reported() {
    // The path runs to the end of what it has, goes back a little and
    // forward again; the switch points of the last move, reported on the
    // first pass, must not count against the first move of the next piece.
    let mut feed = Feeder::new();
    let mut ipl = interpolator();
    ipl.load(feed.piece("G1 F30 X5 H1 L1 H2 L2 H3 L3\n"))
        .unwrap();
    let mut outputs = Vec::new();
    run_to_end(&mut ipl, &mut outputs);
    ipl.set_reverse(true);
    for _ in 0..400_000 {
        let out = ipl.cycle().expect("cycle succeeds");
        let back = out.reverse && out.s_total < 3.5;
        outputs.push(out);
        if back {
            break;
        }
    }
    assert!(outputs.last().unwrap().reverse);
    ipl.set_reverse(false);
    run_to_end(&mut ipl, &mut outputs);
    ipl.append(feed.piece("X10 H4 L0.5 H5 L2 H6 L4\nX15 H7 L1\n"))
        .unwrap();
    run_to_end(&mut ipl, &mut outputs);
    assert!((outputs.last().unwrap().path_position[0] - 15.0).abs() < 1e-9);
    let h: Vec<i32> = outputs
        .iter()
        .flat_map(|o| o.switch_points.iter().map(|sp| sp.number))
        .collect();
    assert_eq!(h, [1, 2, 3, 4, 5, 6, 7]);
}

#[test]
fn a_piece_that_does_not_connect_is_refused() {
    let mut ipl = interpolator();
    ipl.load(Feeder::new().piece("G1 F30 X5\n")).unwrap();
    let before = ipl.planned_segments().len();
    // decoded on its own, this piece starts at the origin
    let stray = Decoder::decode_source(
        DecoderConfig {
            start_position: [7.0, 0.0, 0.0],
            ..Default::default()
        },
        "G1 F30 X9\n",
    )
    .unwrap();
    let err = ipl.append(stray).unwrap_err();
    assert!(matches!(err, CncError::Interpolator(_)), "{err}");
    assert_eq!(ipl.planned_segments().len(), before);
    assert!(!ipl.append_pending());
    let mut outputs = Vec::new();
    run_to_end(&mut ipl, &mut outputs);
    assert!((outputs.last().unwrap().path_position[0] - 5.0).abs() < 1e-9);
}

fn stream_blocks() -> Vec<String> {
    let mut blocks = vec!["G1 F45 X4".to_string()];
    for k in 0..120 {
        let x = 4.0 + 3.0 * (k + 1) as f64;
        match k % 4 {
            0 => blocks.push(format!("X{x} Y1")),
            1 => blocks.push(format!("G2 X{x} Y0 R4")),
            2 => blocks.push(format!("G1 X{x} Y0 M8")),
            _ => blocks.push(format!("G1 X{x} Y-0.5")),
        }
    }
    blocks
}

/// A long zig-zag with arcs, fed block by block a little ahead of the path.
fn stream(discard: Option<f64>) -> (Vec<IplOutput>, usize) {
    let blocks = stream_blocks();
    let mut feed = Feeder::new();
    let mut ipl = interpolator();
    let mut next = 0;
    let mut outputs = Vec::new();
    let mut longest = 0;
    loop {
        // keep 12 mm of path ahead
        while next < blocks.len() && (next == 0 || ipl.remaining_length() < 12.0) {
            let piece = feed.piece(&format!("{}\n", blocks[next]));
            if next == 0 {
                ipl.load(piece).unwrap();
            } else {
                ipl.append(piece).unwrap();
            }
            next += 1;
        }
        if let Some(keep) = discard {
            ipl.discard_history(keep);
        }
        longest = longest.max(ipl.planned_segments().len());
        let out = ipl.cycle().unwrap();
        let done = out.state == IplState::Finished && next == blocks.len();
        outputs.push(out);
        if done {
            break;
        }
        assert!(outputs.len() < 400_000, "did not finish");
    }
    (outputs, longest)
}

#[test]
fn a_stream_runs_through_and_discarding_history_changes_nothing() {
    let (plain, plain_len) = stream(None);
    check_motion(&plain);
    let last = plain.last().unwrap();
    assert!((last.path_position[0] - 364.0).abs() < 1e-9);
    assert_eq!(
        plain.iter().map(|o| o.events.len()).sum::<usize>(),
        30,
        "every M8 once"
    );
    // Fed 12 mm ahead, one block at a time, the path is not starved: the
    // run takes about as long as the same program loaded in one piece.
    let whole = {
        let mut ipl = interpolator();
        let text = stream_blocks().join("\n");
        ipl.load(Decoder::decode_source(DecoderConfig::default(), &text).unwrap())
            .unwrap();
        let mut outputs = Vec::new();
        run_to_end(&mut ipl, &mut outputs);
        outputs.len()
    };
    eprintln!("streamed {} cycles, whole {whole}", plain.len());
    assert!(plain.len() >= whole && plain.len() < whole + whole / 5);

    let (trimmed, trimmed_len) = stream(Some(5.0));
    assert_eq!(plain_len, 121);
    assert!(trimmed_len < 30, "history kept: {trimmed_len} moves");
    assert_eq!(plain.len(), trimmed.len());
    for (i, (a, b)) in plain.iter().zip(&trimmed).enumerate() {
        assert_eq!(a.path_position, b.path_position, "cycle {i}");
        assert_eq!(a.path_velocity, b.path_velocity, "cycle {i}");
        assert_eq!(a.s_total, b.s_total, "cycle {i}");
        assert_eq!(a.state, b.state, "cycle {i}");
    }
}

#[test]
fn discarding_keeps_what_was_asked_for() {
    let mut ipl = interpolator();
    let src = "G1 F30 X10\nY10\nX0\nY0\nX10\nY10\n";
    ipl.load(Decoder::decode_source(DecoderConfig::default(), src).unwrap())
        .unwrap();
    let mut out = ipl.output_buffer();
    while out.s_total < 34.0 {
        ipl.cycle_into(&mut out).unwrap();
    }
    // Nothing behind the path further than 100 mm: nothing to drop.
    assert_eq!(ipl.discard_history(100.0), 0);
    // Keep 15 mm: the path is 4 mm into the fourth move, so the third and
    // the second (10..30) stay as well.
    assert_eq!(ipl.discard_history(15.0), 1);
    assert_eq!(ipl.planned_segments().len(), 5);
    assert_eq!(ipl.start_position(), [10.0, 0.0, 0.0]);
    // Keep nothing: everything before the move the path is in goes.
    assert_eq!(ipl.discard_history(0.0), 2);
    assert_eq!(ipl.discard_history(0.0), 0);
    assert_eq!(ipl.start_position(), [0.0, 10.0, 0.0]);
    assert!((ipl.total_length() - 60.0).abs() < 1e-9);
    while out.state != IplState::Finished {
        ipl.cycle_into(&mut out).unwrap();
    }
    assert!((out.s_total - 60.0).abs() < 1e-9);
    assert!((out.path_position[0] - 10.0).abs() < 1e-9);
    assert!((out.path_position[1] - 10.0).abs() < 1e-9);
}

#[test]
fn the_controller_appends_program_text_with_its_modal_state() {
    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [50.0; 3], [500.0; 3], [5000.0; 3])
            .with_soft_limits([f64::NEG_INFINITY; 3], [100.0, 20.0, 100.0]);
    let mut cnc = CncController::new(
        axis_config,
        limits(),
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        DT,
    );
    let mut pipeline = Pipeline::new();
    let mut driver = SimDriver::<3>::new();
    // nothing to append to yet
    assert!(cnc.append_program("G1 F30 X5\n", &mut pipeline).is_err());

    cnc.load_program("G91 G1 F30 X5\n", DecoderConfig::default(), &mut pipeline)
        .unwrap();
    // G91, G1 and the feed carry over.
    cnc.append_program("X5\nY5\n", &mut pipeline).unwrap();
    // Refused pieces (a syntax error, a soft limit) leave no trace ...
    assert!(cnc.append_program("X5\nG1 Q\n", &mut pipeline).is_err());
    let err = cnc.append_program("X1\nY30\n", &mut pipeline).unwrap_err();
    assert!(matches!(err, CncError::SoftLimit { line: 2, .. }), "{err}");
    // ... so this one continues from X10 Y5.
    cnc.append_program("X-10\n", &mut pipeline).unwrap();
    cnc.run_to_end(&mut driver, 100_000).unwrap();
    let pos = driver.log.position.last().unwrap();
    assert!(
        (pos[0] - 0.0).abs() < 1e-9 && (pos[1] - 5.0).abs() < 1e-9,
        "{pos:?}"
    );
    assert!((cnc.interpolator.total_length() - 25.0).abs() < 1e-9);
}
