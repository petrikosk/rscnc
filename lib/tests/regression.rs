//! Regression tests for defects found in review: each test reproduces a
//! concrete program / operator action that used to stall, jolt, exceed a
//! limit or be rejected, and pins the corrected behavior.

use rscnc::gcode::TrcMode;
use rscnc::prelude::*;

const DT: f64 = 0.001;

fn limits() -> PathLimits {
    PathLimits::new(50.0, 500.0, 5000.0, 500.0)
}

fn interpolator(src: &str) -> Interpolator {
    let queue = Decoder::decode_source(DecoderConfig::default(), src).expect("program decodes");
    let mut ipl = Interpolator::new(limits(), LookAhead::default(), Vec::new(), DT);
    ipl.load(queue).expect("program loads");
    ipl
}

/// Run to completion, invoking `hook(cycle_index, &mut interpolator)` before
/// every cycle.
fn run_with(src: &str, mut hook: impl FnMut(usize, &mut Interpolator)) -> Vec<IplOutput> {
    let mut ipl = interpolator(src);
    let mut outputs = Vec::new();
    for i in 0..400_000 {
        hook(i, &mut ipl);
        let out = ipl.cycle().expect("cycle succeeds");
        let done = out.state == IplState::Finished;
        outputs.push(out);
        if done {
            return outputs;
        }
    }
    panic!("program did not finish");
}

fn run(src: &str) -> Vec<IplOutput> {
    run_with(src, |_, _| {})
}

/// The motion along the path must be one continuous jerk-limited profile:
/// within the limits, never backward, and — the signature of a mis-planned
/// trajectory — never frozen in place while the velocity says it moves
/// (each cycle's progress must match the mean of its end velocities).
fn assert_smooth(outputs: &[IplOutput]) {
    let lim = limits();
    let ds_tol = 0.2 * lim.max_jerk * DT * DT * DT + 1e-9;
    for (i, w) in outputs.windows(2).enumerate() {
        let (p, o) = (&w[0], &w[1]);
        let i = i + 1;
        assert!(
            o.path_velocity >= 0.0 && o.path_velocity <= lim.max_velocity * (1.0 + 1e-9),
            "cycle {i}: velocity {}",
            o.path_velocity
        );
        let accel = (o.path_velocity - p.path_velocity) / DT;
        assert!(
            accel.abs() <= lim.max_acceleration * (1.0 + 1e-6),
            "cycle {i}: acceleration {accel}"
        );
        let jerk = (o.path_acceleration - p.path_acceleration) / DT;
        assert!(
            jerk.abs() <= lim.max_jerk * (1.0 + 1e-6),
            "cycle {i}: jerk {jerk}"
        );
        let ds = o.s_total - p.s_total;
        let expected = 0.5 * (p.path_velocity + o.path_velocity) * DT;
        assert!(
            ds >= 0.0 && (ds - expected).abs() <= ds_tol,
            "cycle {i}: moved {ds} at mean velocity {} (expected {expected})",
            expected / DT
        );
    }
}

fn polygon(radius: f64, corners: usize) -> String {
    let mut src = format!("G1 F50 X{radius} Y0\n");
    for k in 1..=corners {
        let th = std::f64::consts::TAU * k as f64 / corners as f64;
        src += &format!("X{:.6} Y{:.6}\n", radius * th.cos(), radius * th.sin());
    }
    src
}

#[test]
fn polygon_runs_as_fast_as_the_circle_it_approximates() {
    // The junction formula was inverted: the gentler the corner, the slower
    // it was taken, and a 720-gon crawled around in minutes.
    let circle = run("G1 F50 X20 Y0\nG3 X20 Y0 I-20 J0\n");
    for corners in [36, 180, 720] {
        let poly = run(&polygon(20.0, corners));
        assert_smooth(&poly);
        let (n, n_circle) = (poly.len() as f64, circle.len() as f64);
        assert!(
            (n - n_circle).abs() < 0.02 * n_circle,
            "{corners}-gon took {n} cycles, the circle {n_circle}"
        );
    }
}

#[test]
fn tight_polygon_cruises_at_the_centripetal_limit() {
    // 1° chords on a 2 mm radius: v = √(a_n·r) ≈ 31.6 mm/s, reached with
    // one profile over all the chords rather than chord by chord.
    let r = 2.0;
    let mut src = format!("G1 F50 X{r} Y0\n");
    for k in 1..=1440 {
        let th = (k as f64).to_radians();
        src += &format!("X{:.6} Y{:.6}\n", r * th.cos(), r * th.sin());
    }
    let out = run(&src);
    assert_smooth(&out);
    let v_arc = (500.0f64 * r).sqrt();
    let v_peak = out.iter().map(|o| o.path_velocity).fold(0.0, f64::max);
    assert!(
        v_peak <= v_arc * 1.01 && v_peak >= v_arc * 0.9,
        "peak {v_peak} vs centripetal limit {v_arc}"
    );
    // and it holds that speed for most of the way
    let cruising = out
        .iter()
        .filter(|o| o.path_velocity > 0.9 * v_peak)
        .count();
    assert!(
        cruising * 2 > out.len(),
        "{cruising} of {} cycles",
        out.len()
    );
}

#[test]
fn short_last_segment_stops_within_the_limits() {
    // 0.5 mm at the end of a 50 mm run: the reachability pass assumed a
    // constant-acceleration ramp, entered the stub far too fast for a
    // jerk-limited stop and was cut off at its end (17 500 mm/s²).
    let out = run("G1 F20 X50\nX50.5\n");
    assert_smooth(&out);
    assert!((out.last().unwrap().path_position[0] - 50.5).abs() < 1e-9);
}

#[test]
fn short_segments_ramp_like_one_long_move() {
    // Five 0.2 mm segments in front of a long one accelerate straight
    // through, exactly like the same line programmed in one block.
    let chopped = run("G1 F50 X0.2\nX0.4\nX0.6\nX0.8\nX1.0\nX50\n");
    let whole = run("G1 F50 X50\n");
    assert_smooth(&chopped);
    assert_eq!(chopped.len(), whole.len());
}

#[test]
fn override_drop_never_freezes_the_path() {
    // Lowering the override left the profile above its new velocity limit;
    // the trajectory generator then braked past zero velocity and the path
    // stood still for ~170 ms before jumping back to speed.
    let src = "G1 F20 X50\nX50.5\nX51\nX100\n";
    for at in [300, 1000, 2400, 2500, 2550, 2600, 2650] {
        let out = run_with(src, |i, ipl| {
            if i == at {
                ipl.set_override(0.3);
            }
        });
        assert_smooth(&out);
        // it settles at 30 % of F20
        let v_late = out[out.len() / 2 + 2000].path_velocity;
        assert!((v_late - 6.0).abs() < 1e-6, "override at {at}: {v_late}");
    }
}

#[test]
fn override_changes_at_any_moment_keep_the_profile_smooth() {
    // Sweep the moment of an override change over a corner and over the
    // final ramp of a move, where the running profile leaves no slack.
    let src = "G1 F40 X8\nY8\nX0\n";
    for at in (0..1500).step_by(7) {
        for factor in [0.2, 0.6, 1.7] {
            let out = run_with(src, |i, ipl| {
                if i == at {
                    ipl.set_override(factor);
                }
            });
            assert_smooth(&out);
            let end = out.last().unwrap().path_position;
            assert!(end[0].abs() < 1e-9 && (end[1] - 8.0).abs() < 1e-9);
        }
    }
}

#[test]
fn override_above_100_percent_respects_the_machine_limits() {
    // F50 is exactly the machine's 50 mm/s; the 2 mm arc allows
    // √(500·2) ≈ 31.6 mm/s. 180 % override must not lift either.
    let out = run_with("G1 F50 X40\nG2 X44 Y0 R2\nG1 X80\n", |i, ipl| {
        if i == 0 {
            ipl.set_override(1.8);
        }
    });
    assert_smooth(&out);
    let v_arc_limit = (500.0f64 * 2.0).sqrt();
    for o in &out {
        assert!(o.path_velocity <= 50.0 * (1.0 + 1e-9));
        if o.src_line == 2 {
            assert!(
                o.path_velocity <= v_arc_limit * (1.0 + 1e-9),
                "{} on the arc",
                o.path_velocity
            );
        }
    }
    // ... while a feed below the limits is still raised
    let slow = run_with("G1 F10 X40\n", |i, ipl| {
        if i == 0 {
            ipl.set_override(1.8);
        }
    });
    let v_peak = slow.iter().map(|o| o.path_velocity).fold(0.0, f64::max);
    assert!((v_peak - 18.0).abs() < 1e-6, "{v_peak}");
}

#[test]
fn override_above_100_percent_respects_an_aux_axis_limit() {
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        ..Default::default()
    };
    let queue = Decoder::decode_source(cfg, "G1 F10 X20 A40\n").unwrap();
    // A may do 15 units/s and travels 2 units per mm: path limit 7.5 mm/s
    let mut ipl = Interpolator::new(limits(), LookAhead::default(), vec![15.0], DT);
    ipl.load(queue).unwrap();
    ipl.set_override(2.0);
    let mut out = ipl.output_buffer();
    let mut aux_peak = 0.0f64;
    for _ in 0..100_000 {
        ipl.cycle_into(&mut out).unwrap();
        aux_peak = aux_peak.max(out.aux_velocity[0].abs());
        if out.state == IplState::Finished {
            assert!((aux_peak - 15.0).abs() < 1e-6, "aux peak {aux_peak}");
            return;
        }
    }
    panic!("did not finish");
}

#[test]
fn aux_axis_reversal_slows_the_junction() {
    // X runs straight on while A turns around: at full feed the A axis
    // velocity would flip from +20 to −20 units/s within one cycle.
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        ..Default::default()
    };
    let queue = Decoder::decode_source(cfg, "G1 F20 X20 A20\nX40 A0\n").unwrap();
    let mut ipl = Interpolator::new(limits(), LookAhead::default(), vec![100.0], DT);
    ipl.load(queue).unwrap();
    let mut out = ipl.output_buffer();
    let mut prev_aux_v = 0.0;
    let mut worst_step = 0.0f64;
    for _ in 0..100_000 {
        ipl.cycle_into(&mut out).unwrap();
        worst_step = worst_step.max((out.aux_velocity[0] - prev_aux_v).abs());
        prev_aux_v = out.aux_velocity[0];
        if out.state == IplState::Finished {
            assert!((out.aux[0]).abs() < 1e-9);
            // (it was 40 units/s in one cycle)
            assert!(worst_step < 12.0, "aux velocity step {worst_step}");
            return;
        }
    }
    panic!("did not finish");
}

#[test]
fn dwell_lasts_exactly_its_programmed_time() {
    // n subtractions of the cycle time can leave a rounding crumb, which
    // used to cost one more cycle.
    for (seconds, cycles) in [(0.1, 100), (0.3, 300), (0.7, 700), (1.1, 1100), (2.3, 2300)] {
        let out = run(&format!("G1 F10 X1\nG4 P{seconds}\nX2\n"));
        let dwelling = out.iter().filter(|o| o.state == IplState::Dwelling).count();
        assert_eq!(dwelling, cycles, "G4 P{seconds}");
    }
}

#[test]
fn step_outside_a_hold_is_not_remembered() {
    // A `step()` while running used to be latched and silently released the
    // next hold.
    let mut ipl = interpolator("G1 F20 X5\nY5\nX0\n");
    ipl.set_single_step(true);
    let mut holds = 0;
    for i in 0..100_000 {
        if i == 10 {
            assert_eq!(ipl.state(), IplState::Running);
            ipl.step(); // ignored
        }
        let out = ipl.cycle().unwrap();
        if out.state == IplState::StepHold {
            holds += 1;
            if holds == 1 {
                // still held a while later: the early step() did not count
                for _ in 0..50 {
                    assert_eq!(ipl.cycle().unwrap().state, IplState::StepHold);
                }
            }
            ipl.step();
        }
        if out.state == IplState::Finished {
            assert_eq!(holds, 2);
            return;
        }
    }
    panic!("did not finish");
}

#[test]
fn leaving_single_step_mode_releases_the_hold() {
    let mut ipl = interpolator("G1 F20 X5\nY5\n");
    ipl.set_single_step(true);
    let mut released = false;
    for _ in 0..100_000 {
        let out = ipl.cycle().unwrap();
        if out.state == IplState::StepHold && !released {
            ipl.set_single_step(false); // no step(): used to hang here forever
            released = true;
        }
        if out.state == IplState::Finished {
            assert!(released);
            assert!((out.path_position[1] - 5.0).abs() < 1e-9);
            return;
        }
    }
    panic!("did not finish");
}

#[test]
fn single_step_switched_on_at_any_moment_stops_smoothly() {
    // Too late to stop at the end of the current move, the profile used to
    // be cut off there; now it stops at the first move end it can reach.
    let src = "G1 F50 X0.5\nX1\nX1.5\nX2\nX2.5\nX3\nX30\nX60\n";
    for at in (0..400).step_by(3) {
        let mut ipl = interpolator(src);
        let mut outputs = Vec::new();
        let mut held_at = None;
        for i in 0..100_000 {
            if i == at {
                ipl.set_single_step(true);
            }
            let out = ipl.cycle().unwrap();
            if out.state == IplState::StepHold {
                held_at.get_or_insert(out.path_position[0]);
                ipl.set_single_step(false);
            }
            let done = out.state == IplState::Finished;
            outputs.push(out);
            if done {
                break;
            }
        }
        assert_smooth(&outputs);
        assert!((outputs.last().unwrap().path_position[0] - 60.0).abs() < 1e-9);
        // the hold is exactly at a programmed end point
        let x = held_at.expect("held once");
        assert!(
            [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 30.0]
                .iter()
                .any(|p| (x - p).abs() < 1e-9),
            "switched on at {at}: held at {x}"
        );
    }
}

#[test]
fn halt_across_short_segments_reports_the_segment_it_stops_in() {
    // The stop ramp runs on through several blended moves; the reported
    // source line stayed on the move the halt was issued in.
    let mut src = String::from("G1 F50\n");
    for k in 1..=100 {
        src += &format!("X{:.1}\n", k as f64 * 0.5);
    }
    let out = run_with(&src, |i, ipl| {
        if i == 300 {
            ipl.halt();
        }
        if i == 1000 {
            ipl.resume();
        }
    });
    assert_smooth(&out);
    let paused = out
        .iter()
        .find(|o| o.state == IplState::Paused)
        .expect("paused");
    // line k+1 ends at x = k/2
    let x = paused.path_position[0];
    let line = (x / 0.5).ceil() as usize + 1;
    assert_eq!(paused.src_line, line, "paused at x = {x}");
    assert!(out[300].src_line < paused.src_line);
}

#[test]
fn halt_and_resume_at_any_moment_is_smooth() {
    let src = "G1 F50 X3\nX3.2\nX3.4 Y0.1\nG4 P0.01\nX10 Y3\nM8\nX0 Y0\n";
    for at in (0..1300).step_by(11) {
        let out = run_with(src, |i, ipl| {
            if i == at {
                ipl.halt();
            }
            if i == at + 37 {
                ipl.resume();
            }
        });
        assert_smooth(&out);
        let end = out.last().unwrap().path_position;
        assert!(end[0].abs() < 1e-9 && end[1].abs() < 1e-9);
    }
}

#[test]
fn full_circle_without_axis_words() {
    // `G2 I-5` is a full circle; it was rejected, and as a modal follow-up
    // block silently skipped.
    let queue = Decoder::decode_source(DecoderConfig::default(), "G1 F10 X10\nG2 I-5\nJ3\nG1 X0\n")
        .unwrap();
    let lengths: Vec<f64> = queue
        .iter()
        .filter_map(|e| match e {
            GeoElem::Move(m) => Some(m.geo.length()),
            _ => None,
        })
        .collect();
    assert_eq!(lengths.len(), 4);
    assert!((lengths[1] - std::f64::consts::TAU * 5.0).abs() < 1e-9);
    assert!((lengths[2] - std::f64::consts::TAU * 3.0).abs() < 1e-9);
    // a bare G2 still is an error, and I/J/K mean nothing to a G1
    assert!(Decoder::decode_source(DecoderConfig::default(), "G1 F10 X10\nG2\n").is_err());
    let line_only = Decoder::decode_source(DecoderConfig::default(), "G1 F10 X10\nI-5\n").unwrap();
    assert_eq!(line_only.len(), 1);
}

#[test]
fn overflowing_number_is_a_gcode_error() {
    // 400 digits parse to infinity, which used to travel on into the
    // geometry and the trajectory generator.
    for word in ["X", "F", "I", "R"] {
        let src = format!("G1 F10 X10\nG2 X20 {word}{}\n", "9".repeat(400));
        match Decoder::decode_source(DecoderConfig::default(), &src) {
            Err(CncError::Gcode(e)) => assert_eq!(e.line, 2),
            other => panic!("{word}: expected a G-code error, got {other:?}"),
        }
    }
}

fn line_move(to: [f64; 3], from: [f64; 3], src_line: usize) -> GeoElem {
    GeoElem::Move(GeoMove {
        geo: Geometry::Line(LineSeg::new(from, to)),
        feed: 10.0,
        is_rapid: false,
        aux_start: Vec::new(),
        aux_end: Vec::new(),
        trc: TrcMode::Off,
        accel: None,
        decel: None,
        switch_points: Vec::new(),
        src_line,
        obj_no: src_line as u32,
    })
}

#[test]
fn load_refuses_a_move_that_cannot_be_interpolated() {
    let mut ipl = interpolator("G1 F10 X10\n");
    for bad in [f64::INFINITY, f64::NAN] {
        let mut queue = GeoQueue::new();
        queue.push(line_move([5.0, 0.0, 0.0], [0.0; 3], 1));
        queue.push(line_move([bad, 0.0, 0.0], [5.0, 0.0, 0.0], 2));
        let err = ipl.load(queue).expect_err("refused");
        assert!(err.to_string().contains("line 2"), "{err}");
    }
    // the program loaded before is untouched and still runs
    assert!((ipl.total_length() - 10.0).abs() < 1e-12);
    for _ in 0..100_000 {
        let out = ipl.cycle().unwrap();
        if out.state == IplState::Finished {
            assert!((out.path_position[0] - 10.0).abs() < 1e-9);
            return;
        }
    }
    panic!("did not finish");
}

#[test]
fn kinematics_and_controller_agree_on_auxiliary_axes() {
    // Path on machine axes 1 and 2: the auxiliary axis is machine axis 0.
    // The controller took its velocity limit from there, but the kinematics
    // wrote its position to axis 2 (over the path's Y).
    let kin = CartesianKinematics::<3>::new(PathAxes::Two([1, 2]));
    assert_eq!(kin.aux_axes, vec![0]);
    let mut axis_config =
        AxisConfig::<3>::cartesian(["A", "X", "Y"], [5.0, 50.0, 50.0], [500.0; 3], [5000.0; 3]);
    axis_config.path_axes = PathAxes::Two([1, 2]);
    let mut cnc = CncController::new(axis_config, limits(), LookAhead::default(), kin, DT);
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        ..Default::default()
    };
    cnc.load_program("G1 F20 X10 Y20 A7\n", cfg, &mut Pipeline::new())
        .unwrap();
    let mut driver = SimDriver::<3>::new();
    for _ in 0..100_000 {
        let out = cnc.cycle(&mut driver).unwrap();
        // A is limited to 5 units/s
        assert!(out.aux_velocity[0].abs() <= 5.0 * (1.0 + 1e-9));
        if out.state == IplState::Finished {
            let end = driver.log.position.last().unwrap();
            assert!((end[0] - 7.0).abs() < 1e-9, "A at {}", end[0]);
            assert!((end[1] - 10.0).abs() < 1e-9, "X at {}", end[1]);
            assert!((end[2] - 20.0).abs() < 1e-9, "Y at {}", end[2]);
            return;
        }
    }
    panic!("did not finish");
}

#[test]
fn tool_radius_correction_keeps_auxiliary_only_moves() {
    // An `A` move between two corrected contour moves was dropped, and the
    // A axis jumped at the start of the next move.
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        ..Default::default()
    };
    let src = "G1 F10 X0 Y0\nG41 X10\nA5\nY10\nA9\nG40 X0 Y20\n";
    let queue = Decoder::decode_source(cfg, src).unwrap();
    let out = ToolRadiusCorrection::new(1.0).process(queue).unwrap();
    let moves: Vec<&GeoMove> = out
        .iter()
        .filter_map(|e| match e {
            GeoElem::Move(m) => Some(m),
            _ => None,
        })
        .collect();
    // position and auxiliary axis are continuous over the whole result
    for w in moves.windows(2) {
        let (end, start) = (w[0].geo.end(), w[1].geo.start());
        for k in 0..3 {
            assert!(
                (end[k] - start[k]).abs() < 1e-9,
                "gap between lines {} and {}",
                w[0].src_line,
                w[1].src_line
            );
        }
        assert_eq!(w[0].aux_end, w[1].aux_start);
    }
    assert_eq!(moves.last().unwrap().aux_end, vec![9.0]);
    // the A moves are there, at the corrected position rather than on the
    // programmed contour
    let a_moves: Vec<&&GeoMove> = moves.iter().filter(|m| m.geo.length() == 0.0).collect();
    assert_eq!(a_moves.len(), 2);
    assert_eq!((a_moves[0].aux_start[0], a_moves[0].aux_end[0]), (0.0, 5.0));
    let p = a_moves[0].geo.start();
    assert!(
        (p[0] - 9.0).abs() < 1e-9 && (p[1] - 1.0).abs() < 1e-9,
        "{p:?}"
    );
}

/// Acceleration of every cycle (v[i] − v[i−1]) / dt, with the source lines
/// the cycle started and ended on.
fn accelerations(outputs: &[IplOutput]) -> Vec<(f64, usize, usize)> {
    outputs
        .windows(2)
        .map(|w| {
            (
                (w[1].path_velocity - w[0].path_velocity) / DT,
                w[0].src_line,
                w[1].src_line,
            )
        })
        .collect()
}

#[test]
fn programmed_acceleration_limits_hold_across_short_segments() {
    // `E` limits on a contour chopped into short segments: the run of
    // segments is one profile, which must use the programmed pair — 120
    // mm/s² up, 60 mm/s² down — not the machine's 500.
    let mut src = String::from("G1 F50 E120 E-60\n");
    for k in 1..=200 {
        src += &format!("X{:.2}\n", 0.25 * f64::from(k));
    }
    let out = run(&src);
    assert_smooth(&out);
    let acc = accelerations(&out);
    let up = acc.iter().map(|a| a.0).fold(0.0, f64::max);
    let down = acc.iter().map(|a| -a.0).fold(0.0, f64::max);
    assert!(up <= 120.0 * (1.0 + 1e-6), "acceleration {up}");
    assert!(down <= 60.0 * (1.0 + 1e-6), "deceleration {down}");
    // ... and it does use them: the segments ramp like one long move
    assert!(up > 110.0 && down > 55.0, "up {up}, down {down}");
    let whole = run("G1 F50 E120 E-60 X50\n");
    assert!(
        out.len() <= whole.len() + 2,
        "{} vs {} cycles",
        out.len(),
        whole.len()
    );
}

#[test]
fn a_change_of_the_acceleration_limits_is_honoured_move_by_move() {
    // Three colinear moves with their own limits: no cycle may accelerate
    // or brake harder than the moves it touches allow.
    let src = "G1 F50 E400 E-400 X15\nE80 E-80 X30\nE400 E-150 X60\n";
    let limit = |line: usize| -> (f64, f64) {
        match line {
            1 => (400.0, 400.0),
            2 => (80.0, 80.0),
            _ => (400.0, 150.0),
        }
    };
    for ovr_at in [usize::MAX, 150, 420, 700] {
        let out = run_with(src, |i, ipl| {
            if i == ovr_at {
                ipl.set_override(0.4);
            }
            if i == ovr_at.saturating_add(250) {
                ipl.set_override(1.0);
            }
        });
        assert_smooth(&out);
        for (i, (a, from, to)) in accelerations(&out).into_iter().enumerate() {
            let (acc, dec) = (
                limit(from).0.max(limit(to).0),
                limit(from).1.max(limit(to).1),
            );
            assert!(
                a <= acc * (1.0 + 1e-6) && -a <= dec * (1.0 + 1e-6),
                "override at {ovr_at}, cycle {i}: {a} on lines {from}..{to}"
            );
        }
        assert!((out.last().unwrap().path_position[0] - 60.0).abs() < 1e-9);
    }
}

#[test]
fn halt_running_into_another_deceleration_limit_stops_in_time() {
    // The stop ramp of a halt used the deceleration limit of the move the
    // halt was requested in for all of it. Running on into a move planned
    // with a harder limit, it braked too gently and overran the program
    // end (the position froze there and the velocity jumped to zero).
    for src in [
        "G1 F50 E-60 X20\nE-500 X20.8\n",
        "G1 F50 E-500 X20\nE-60 X24\n",
        "G1 F50 E-60 X20\nE-500 X20.4\nE-90 X21\nE-500 X21.3\n",
    ] {
        let cycles = run(src).len();
        for at in (0..cycles).step_by(3) {
            let out = run_with(src, |i, ipl| {
                if i == at {
                    ipl.halt();
                }
                if i == at + 400 {
                    ipl.resume();
                }
            });
            assert_smooth(&out);
        }
    }
}

#[test]
fn switch_points_on_short_segments_are_all_reported_in_order() {
    // 0.02 mm segments at 50 mm/s: a cycle crosses two or three moves, each
    // with switch points of its own. Every one is reported exactly once,
    // in path order, and the output buffer never has to grow for them.
    let mut src = String::from("G1 F50\n");
    let mut expected = Vec::new();
    for k in 1..=600 {
        let n = k % 7 + 1;
        src += &format!("X{:.2} H{n} O0.25 H-{n} O0.75\n", 0.02 * f64::from(k));
        expected.extend([n, -n]);
    }
    for halt_at in [usize::MAX, 60, 137] {
        let mut ipl = interpolator(&src);
        let mut out = ipl.output_buffer();
        let capacity = out.switch_points.capacity();
        let (mut seen, mut most) = (Vec::new(), 0);
        for i in 0..100_000 {
            if i == halt_at {
                ipl.halt();
            }
            if i == halt_at.saturating_add(90) {
                ipl.resume();
            }
            ipl.cycle_into(&mut out).unwrap();
            most = most.max(out.switch_points.len());
            seen.extend(out.switch_points.iter().map(|sp| sp.number));
            assert_eq!(out.switch_points.capacity(), capacity, "cycle {i}");
            if out.state == IplState::Finished {
                break;
            }
        }
        assert_eq!(seen, expected, "halt at {halt_at}");
        assert!(most >= 4, "at most {most} switch points in one cycle");
    }
}

#[test]
fn halt_at_the_moment_the_acceleration_limit_is_reached() {
    // Halting in the very cycle the start ramp reaches the acceleration
    // limit asks the trajectory generator for a stop profile that touches
    // its acceleration limit in a single point - a boundary on which it
    // found no solution, so the cycle failed with an error. (3000 mm/s² at
    // 100 000 mm/s³ is reached after exactly 30 cycles of 1 ms.)
    let lim = PathLimits::new(200.0, 3000.0, 100_000.0, 2000.0);
    for halt_at in 20..45 {
        let queue = Decoder::decode_source(DecoderConfig::default(), "G1 F110 X50\n").unwrap();
        let mut ipl = Interpolator::new(lim, LookAhead::default(), Vec::new(), DT);
        ipl.load(queue).unwrap();
        let mut out = ipl.output_buffer();
        for i in 0..5000 {
            if i == halt_at {
                ipl.halt();
            }
            if i == halt_at + 200 {
                ipl.resume();
            }
            ipl.cycle_into(&mut out)
                .unwrap_or_else(|e| panic!("halt at {halt_at}, cycle {i}: {e}"));
            if out.state == IplState::Finished {
                break;
            }
        }
        assert_eq!(out.state, IplState::Finished, "halt at {halt_at}");
        assert!((out.path_position[0] - 50.0).abs() < 1e-9);
    }
}

#[test]
fn a_plan_without_acceleration_margin_can_still_be_followed() {
    // With `accel_margin: 1.0` the look-ahead planned every braking span
    // with no distance to spare at all, while the interpolator keeps a hair
    // of slack on each profile it asks for. It therefore left each such span
    // a trace too fast, the excess added up along a chain of them, and the
    // stop ramp at the end of the chain no longer fitted: a halt on that
    // ramp ran micrometres over the program end (the position froze there
    // while the velocity was still above zero).
    //
    // Every `E` word starts a span of its own here, each too short for the
    // braking it has to do.
    let src = "G1 F45 X6\nE-450 X6.5\nE-400 X6.9\nE-350 X7.2\nE-300 X7.4\n\
               E-250 X7.6\nE-200 X7.8\nE-20 X9.4\n";
    let run = |halt_at: usize| {
        let lookahead = LookAhead {
            accel_margin: 1.0,
            ..LookAhead::default()
        };
        let queue = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        let mut ipl = Interpolator::new(limits(), lookahead, Vec::new(), DT);
        ipl.load(queue).unwrap();
        let mut outputs = Vec::new();
        for i in 0..100_000 {
            if i == halt_at {
                ipl.halt();
            }
            if i == halt_at.saturating_add(600) {
                ipl.resume();
            }
            outputs.push(ipl.cycle().unwrap());
            if outputs[i].state == IplState::Finished {
                break;
            }
        }
        outputs
    };
    let cycles = run(usize::MAX).len();
    for halt_at in (0..cycles).step_by(2) {
        let out = run(halt_at);
        assert_smooth(&out);
        assert!((out.last().unwrap().path_position[0] - 9.4).abs() < 1e-9);
    }
}

#[test]
fn single_step_switched_on_in_the_last_jerk_phase_of_a_move() {
    // A move that ends at a very low but non-zero velocity (here a rapid
    // held back by its auxiliary axis, before a sharp corner) finishes with
    // one jerk phase straight into its end point. Switching single-step on
    // during that phase asks for a stop at the same point from a state that
    // lies exactly on the boundary between two profile shapes - where the
    // trajectory generator found no solution, and the cycle failed.
    let src = "G1 F28 E390\nX0.0177 Y0.0564 A-5.134\nG0 X0.1764 Y0.2894\n\
               G0 X0.1805 Y0.2962\nG1 X0.2167 Y0.3412\nG0 X0.2910 Y0.5246 A9.382\n\
               G1 X0.2940 Y0.5334 A-1.232\n";
    let run = |at: usize| {
        let cfg = DecoderConfig {
            aux_letters: vec!['A'],
            start_aux: vec![0.0],
            ..Default::default()
        };
        let queue = Decoder::decode_source(cfg, src).unwrap();
        let lim = PathLimits::new(100.0, 1000.0, 20_000.0, 800.0);
        let lookahead = LookAhead {
            junction_deviation: 0.01,
            accel_margin: 1.0,
            ..LookAhead::default()
        };
        let mut ipl = Interpolator::new(lim, lookahead, vec![30.0], 0.0005);
        ipl.load(queue).unwrap();
        let mut out = ipl.output_buffer();
        for i in 0..100_000 {
            if i == at {
                ipl.set_single_step(true);
            }
            if i > at && i % 64 == 0 {
                ipl.step();
            }
            ipl.cycle_into(&mut out)
                .unwrap_or_else(|e| panic!("single-step at {at}, cycle {i}: {e}"));
            if out.state == IplState::Finished {
                return i;
            }
        }
        panic!("single-step at {at}: did not finish");
    };
    let cycles = run(usize::MAX);
    for at in 0..cycles {
        run(at);
    }
}

#[test]
#[allow(clippy::excessive_precision)] // the inputs as they were met
fn a_move_on_the_border_between_two_profile_types_is_interpolated() {
    // A rest-to-rest move of the length 2·a³/j² just reaches the
    // acceleration limit: the border between the profiles with and without
    // a phase of constant acceleration. On it (and an ulp or so beyond it)
    // rsruckig 3.1.1 finds no profile (calculator error in step 1) unless
    // it is asked a hair off the border.
    let slow = PathLimits::new(50.0, 500.0, 5000.0, 500.0); // 2·a³/j² = 10
    let fast = PathLimits::new(200.0, 3000.0, 100_000.0, 2000.0); // 5.4
    for (limits, from, to, cycles) in [
        (&slow, 0.0, 10.000000000000004, 400),
        (&slow, 4.2315545023694767, 14.2315545023694803, 400),
        (&slow, 0.0, 10.0, 400),
        (&slow, 0.0, 9.999999999999998, 400),
        (&fast, 0.0, 5.4, 120),
        (&fast, 0.0, 5.400000000000001, 120),
        (&fast, 1.4605632494715692, 6.8605632494715758, 120),
    ] {
        let cfg = DecoderConfig {
            start_position: [from, 0.0, 0.0],
            ..Default::default()
        };
        let src = format!("G1 F3000 X{to:?}\n");
        let queue = Decoder::decode_source(cfg, &src).unwrap();
        let mut ipl = Interpolator::new(*limits, LookAhead::default(), Vec::new(), 0.001);
        ipl.load(queue)
            .unwrap_or_else(|e| panic!("{from} -> {to}: {e}"));
        let mut out = ipl.output_buffer();
        let mut n = 0;
        while out.state != IplState::Finished {
            ipl.cycle_into(&mut out).unwrap();
            n += 1;
            assert!(n < 1000);
        }
        assert!((cycles..=cycles + 2).contains(&n), "{from} -> {to}: {n}");
        assert!((out.path_position[0] - to).abs() < 1e-12);
    }
}

#[test]
fn a_halted_stop_takes_the_limits_of_a_move_from_where_it_enters_it() {
    // The jerk limit of the X axis holds a move along X to 6000 mm/s³; a
    // move at 30° to it may use 6000/cos 30° = 6928. Planned without a
    // margin, the path leaves the first move with exactly the velocity it
    // can still stop from within the second. A stop that ran on into the
    // second move kept the jerk limit of the first until the end of that
    // cycle, and came to the end of the second move a trace too fast: the
    // position froze there and the velocity jumped to zero.
    let limits = PathLimits::new(50.0, 1000.0, 10_000.0, 1000.0);
    let x_jerk = AxisLimit {
        max_jerk: 6000.0,
        ..AxisLimit::UNLIMITED
    };
    let axis_limits = AxisLimits {
        path: [x_jerk, AxisLimit::UNLIMITED, AxisLimit::UNLIMITED],
        aux: Vec::new(),
    };
    let lookahead = LookAhead {
        accel_margin: 1.0,
        ..LookAhead::default()
    };
    let (sin, cos) = 30f64.to_radians().sin_cos();
    let (j1, j2) = (6000.0, 6000.0 / cos);
    // The second move is as long as a stop from `v` takes, the first as
    // long as the ramp up to `v` (and a trace more).
    let l2 = 0.135;
    let v = (l2 * j2.sqrt()).powf(2.0 / 3.0);
    let l1 = 1.0001 * v * (v / j1).sqrt();
    let src = format!("G1 F50 X{l1:.6}\nX{:.6} Y{:.6}\n", l1 + l2 * cos, l2 * sin);
    let load = || {
        let queue = Decoder::decode_source(DecoderConfig::default(), &src).unwrap();
        let mut ipl =
            Interpolator::with_axis_limits(limits, lookahead.clone(), axis_limits.clone(), DT);
        ipl.load(queue).unwrap();
        ipl
    };
    let plan = load();
    let plan = plan.planned_segments();
    assert_eq!(plan.len(), 2);
    assert!(
        (plan[0].jerk - j1).abs() < 0.1 && (plan[1].jerk - j2).abs() < 0.1,
        "{plan:?}"
    );
    assert!((plan[0].v_exit - v).abs() < 1e-3 * v, "{}", plan[0].v_exit);

    let total = l1 + l2;
    let mut halts_in_the_first_move = 0;
    for at in 0..200 {
        let mut ipl = load();
        let mut prev = ipl.cycle().unwrap();
        let mut halted_at = None;
        for i in 1..5000 {
            if i == at {
                ipl.halt();
                halted_at = Some(prev.s_total);
            }
            let out = ipl.cycle().unwrap();
            // Progress matches the velocity: no freeze at a move end.
            let mean = 0.5 * (out.path_velocity + prev.path_velocity) * DT;
            assert!(
                (out.s_total - prev.s_total - mean).abs() <= 0.2 * 10_000.0 * DT * DT * DT + 1e-9,
                "halt at {at}, cycle {i}: step {} at a mean velocity of {}",
                out.s_total - prev.s_total,
                mean / DT
            );
            let jerk = (out.path_acceleration - prev.path_acceleration).abs() / DT;
            assert!(jerk <= j2 * (1.0 + 1e-6), "halt at {at}, cycle {i}: {jerk}");
            match out.state {
                IplState::Paused => ipl.resume(),
                IplState::Finished => {
                    assert!((out.s_total - total).abs() < 1e-6);
                    break;
                }
                _ => {}
            }
            assert!(i < 4999, "halt at {at}: the program does not finish");
            prev = out;
        }
        if halted_at.is_some_and(|s| s > 0.0 && s < l1) {
            halts_in_the_first_move += 1;
        }
    }
    assert!(halts_in_the_first_move > 20, "{halts_in_the_first_move}");
}
