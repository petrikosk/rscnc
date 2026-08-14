//! End-to-end interpolation tests: run G-code programs through the full
//! pipeline and assert kinematic limits, endpoint accuracy, blending,
//! override/pause/resume, dwell, events and single-step behavior on the
//! sampled per-cycle output.

use rscnc::prelude::*;

const DT: f64 = 0.001;

fn limits() -> PathLimits {
    PathLimits::new(50.0, 500.0, 5000.0, 500.0)
}

fn controller() -> CncController<3, CartesianKinematics<3>> {
    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [50.0; 3], [500.0; 3], [5000.0; 3]);
    CncController::new(
        axis_config,
        limits(),
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        DT,
    )
}

struct Run {
    outputs: Vec<IplOutput>,
    log: TrajectoryLog<3>,
}

fn run_program(src: &str) -> Run {
    run_with(src, |_, _| {})
}

/// Run to completion, invoking `hook(cycle_index, &mut controller)` before
/// every cycle (for override changes etc.).
fn run_with(
    src: &str,
    mut hook: impl FnMut(usize, &mut CncController<3, CartesianKinematics<3>>),
) -> Run {
    let mut cnc = controller();
    let mut pipeline = Pipeline::new();
    cnc.load_program(src, DecoderConfig::default(), &mut pipeline)
        .expect("program loads");
    let mut driver = SimDriver::<3>::new();
    let mut outputs = Vec::new();
    for i in 0..200_000 {
        hook(i, &mut cnc);
        let out = cnc.cycle(&mut driver).expect("cycle succeeds");
        let done = out.state == IplState::Finished;
        outputs.push(out);
        if done {
            return Run {
                outputs,
                log: driver.log,
            };
        }
    }
    panic!("program did not finish in 200 s");
}

/// Assert path-level kinematic limits on the sampled output via finite
/// differences. The profile is continuous, so the mean acceleration/jerk over
/// one cycle can never exceed the instantaneous limits.
fn assert_path_limits(run: &Run, lim: &PathLimits) {
    let tol_v = lim.max_velocity * 1e-9 + 1e-9;
    let tol_a = lim.max_acceleration * 1e-6 + 1e-6;
    let tol_j = lim.max_jerk * 1e-6 + 1e-6;
    for (i, out) in run.outputs.iter().enumerate() {
        assert!(
            out.path_velocity <= lim.max_velocity + tol_v,
            "cycle {i}: v {} exceeds {}",
            out.path_velocity,
            lim.max_velocity
        );
        assert!(
            out.path_velocity >= 0.0,
            "cycle {i}: negative path velocity"
        );
        if i > 0 {
            let dv = out.path_velocity - run.outputs[i - 1].path_velocity;
            assert!(
                dv.abs() / DT <= lim.max_acceleration + tol_a,
                "cycle {i}: mean accel {} exceeds {}",
                dv.abs() / DT,
                lim.max_acceleration
            );
            let da = out.path_acceleration - run.outputs[i - 1].path_acceleration;
            assert!(
                da.abs() / DT <= lim.max_jerk + tol_j,
                "cycle {i}: mean jerk {} exceeds {}",
                da.abs() / DT,
                lim.max_jerk
            );
        }
    }
}

/// Assert per-cycle position continuity: no setpoint may jump farther than
/// one cycle of max velocity.
fn assert_continuity(run: &Run, lim: &PathLimits) {
    for i in 1..run.log.position.len() {
        let p = run.log.position[i];
        let q = run.log.position[i - 1];
        let d = ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt();
        assert!(
            d <= lim.max_velocity * DT * (1.0 + 1e-6) + 1e-9,
            "cycle {i}: position jump {d}"
        );
    }
}

fn last(run: &Run) -> &IplOutput {
    run.outputs.last().unwrap()
}

#[test]
fn square_endpoint_and_limits() {
    let run = run_program("G1 F1200 X10\nY10\nX0\nY0\n");
    assert_path_limits(&run, &limits());
    assert_continuity(&run, &limits());
    let end = last(&run);
    assert!(end.path_velocity.abs() < 1e-9);
    for i in 0..3 {
        assert!(
            end.path_position[i].abs() < 1e-8,
            "endpoint axis {i}: {}",
            end.path_position[i]
        );
    }
    // s_total equals the programmed path length (perimeter 40)
    assert!((end.s_total - 40.0).abs() < 1e-8, "s_total {}", end.s_total);
    assert!((end.progress - 1.0).abs() < 1e-12);
    // Progress is monotone
    for i in 1..run.outputs.len() {
        assert!(run.outputs[i].s_total >= run.outputs[i - 1].s_total - 1e-12);
    }
}

#[test]
fn full_circle_stays_on_circle() {
    // Full CCW circle around (10, 0) starting/ending at origin
    let run = run_program("G3 F1200 X0 Y0 I10 J0\n");
    assert_path_limits(&run, &limits());
    for (i, out) in run.outputs.iter().enumerate() {
        let dx = out.path_position[0] - 10.0;
        let dy = out.path_position[1];
        let r = (dx * dx + dy * dy).sqrt();
        assert!(
            (r - 10.0).abs() < 1e-9,
            "cycle {i}: radial error {}",
            r - 10.0
        );
    }
    let end = last(&run);
    assert!(end.path_position[0].abs() < 1e-8 && end.path_position[1].abs() < 1e-8);
    // Curvature cap on r=10: v ≤ sqrt(500·10) ≈ 70.7 → feed 20 is the cap;
    // the circle is long enough to actually reach it
    let v_max_seen = run
        .outputs
        .iter()
        .map(|o| o.path_velocity)
        .fold(0.0, f64::max);
    assert!((v_max_seen - 20.0).abs() < 0.1, "v_max {v_max_seen}");
}

#[test]
fn tangent_junctions_blend_without_stopping() {
    // line → quarter arc (tangent) → line (tangent): junction velocities > 0
    let run = run_program("G1 F1200 X10\nG3 X14 Y4 I0 J4\nG1 Y14\n");
    assert_path_limits(&run, &limits());
    assert_continuity(&run, &limits());
    // At every segment transition (obj_no change) velocity stays well above zero
    let mut transitions = 0;
    for i in 1..run.outputs.len() {
        if run.outputs[i].obj_no != run.outputs[i - 1].obj_no {
            transitions += 1;
            assert!(
                run.outputs[i].path_velocity > 1.0,
                "cycle {i}: blended junction dropped to {} mm/s",
                run.outputs[i].path_velocity
            );
        }
    }
    assert_eq!(transitions, 2);
}

#[test]
fn sharp_corner_slows_down() {
    // 90° corner must be slower than cruise but the program still finishes exactly
    let run = run_program("G1 F1200 X20\nY20\n");
    assert_path_limits(&run, &limits());
    let mut corner_v = None;
    for i in 1..run.outputs.len() {
        if run.outputs[i].obj_no != run.outputs[i - 1].obj_no {
            corner_v = Some(run.outputs[i].path_velocity);
        }
    }
    let corner_v = corner_v.expect("corner crossed");
    // Junction deviation 0.05, a_n 500, 90°: v = sqrt(500·0.05·sin45/(1−sin45)) ≈ 7.7
    assert!(corner_v < 10.0, "corner velocity {corner_v}");
    assert!(corner_v > 0.5, "corner should not fully stop: {corner_v}");
    let end = last(&run);
    assert!((end.path_position[0] - 20.0).abs() < 1e-8);
    assert!((end.path_position[1] - 20.0).abs() < 1e-8);
}

#[test]
fn override_pause_resume() {
    let mut paused_seen = false;
    let run = run_with("G1 F1200 X30\n", |i, cnc| {
        match i {
            400 => cnc.interpolator.set_override(0.4),
            800 => cnc.interpolator.set_override(0.0),
            1400 => cnc.interpolator.set_override(1.2),
            _ => {}
        }
        if cnc.interpolator.state() == IplState::Paused {
            // marker checked below via outputs; nothing to do here
        }
    });
    assert_path_limits(&run, &limits());
    assert_continuity(&run, &limits());
    // While overridden to 40%, cruise velocity respects 0.4·20 = 8 mm/s
    for (i, out) in run.outputs.iter().enumerate().skip(600).take(150) {
        assert!(
            out.path_velocity <= 8.0 + 1e-6,
            "cycle {i}: override ignored, v = {}",
            out.path_velocity
        );
    }
    // The pause actually reached standstill before cycle 1400
    for out in run.outputs.iter().skip(1200).take(190) {
        if out.state == IplState::Paused {
            paused_seen = true;
            assert!(out.path_velocity == 0.0);
        }
    }
    assert!(paused_seen, "interpolator never reached Paused");
    // 120% override may exceed programmed feed but never machine limits
    let v_max_seen = run
        .outputs
        .iter()
        .map(|o| o.path_velocity)
        .fold(0.0, f64::max);
    assert!(
        v_max_seen > 20.0,
        "120% override should exceed programmed feed"
    );
    assert!(v_max_seen <= 24.0 + 1e-6);
    let end = last(&run);
    assert!((end.path_position[0] - 30.0).abs() < 1e-8);
}

#[test]
fn events_and_dwell() {
    let run = run_program("G1 F1200 X5 M8\nG4 P0.25\nX10 M9\nM30\n");
    // M8 exactly once, at the end of the first move (s ≈ 5)
    let m8: Vec<_> = run
        .outputs
        .iter()
        .enumerate()
        .filter(|(_, o)| o.events.iter().any(|e| e.m.contains(&8)))
        .collect();
    assert_eq!(m8.len(), 1, "M8 fired {} times", m8.len());
    assert!((m8[0].1.s_total - 5.0).abs() < 1e-6);
    let m9 = run
        .outputs
        .iter()
        .flat_map(|o| &o.events)
        .filter(|e| e.m.contains(&9))
        .count();
    assert_eq!(m9, 1);
    // Dwell: ~250 cycles in Dwelling state (±2 for cycle quantization)
    let dwell_cycles = run
        .outputs
        .iter()
        .filter(|o| o.state == IplState::Dwelling)
        .count();
    assert!(
        (248..=252).contains(&dwell_cycles),
        "dwell lasted {dwell_cycles} cycles"
    );
    // Velocity is zero during the dwell
    assert!(run
        .outputs
        .iter()
        .filter(|o| o.state == IplState::Dwelling)
        .all(|o| o.path_velocity == 0.0));
}

#[test]
fn single_step_holds_at_boundaries() {
    let mut cnc = controller();
    let mut pipeline = Pipeline::new();
    cnc.load_program("G1 F1200 X5\nY5\n", DecoderConfig::default(), &mut pipeline)
        .unwrap();
    // Enabling single-step mid-flight retargets the running segment to stop
    // at its end
    cnc.interpolator.set_single_step(true);
    assert_eq!(cnc.interpolator.state(), IplState::Running);
    let mut driver = SimDriver::<3>::new();
    let mut hold_positions = Vec::new();
    for _ in 0..20_000 {
        let out = cnc.cycle(&mut driver).unwrap();
        if out.state == IplState::StepHold {
            hold_positions.push(out.path_position);
            cnc.interpolator.step();
        }
        if out.state == IplState::Finished {
            // First hold was at the end of segment 1
            assert_eq!(hold_positions.len(), 1);
            assert!((hold_positions[0][0] - 5.0).abs() < 1e-8);
            assert!(hold_positions[0][1].abs() < 1e-8);
            assert!((out.path_position[1] - 5.0).abs() < 1e-8);
            return;
        }
    }
    panic!("single-step program did not finish");
}

#[test]
fn aux_axis_caps_path_velocity() {
    // 4-axis machine, slow A axis (10 units/s): X10 with A90 must crawl at
    // v ≤ 10·(10/90) ≈ 1.11 mm/s
    let axis_config = AxisConfig::<4>::cartesian(
        ["X", "Y", "Z", "A"],
        [50.0, 50.0, 50.0, 10.0],
        [500.0; 4],
        [5000.0; 4],
    );
    let mut cnc = CncController::new(
        axis_config,
        limits(),
        LookAhead::default(),
        CartesianKinematics::<4>::new(PathAxes::Three([0, 1, 2])),
        DT,
    );
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        ..Default::default()
    };
    let mut pipeline = Pipeline::new();
    cnc.load_program("G1 F1200 X10 A90\n", cfg, &mut pipeline)
        .unwrap();
    let mut driver = SimDriver::<4>::new();
    let mut v_max_seen = 0.0f64;
    let mut aux_v_max = 0.0f64;
    for _ in 0..200_000 {
        let out = cnc.cycle(&mut driver).unwrap();
        v_max_seen = v_max_seen.max(out.path_velocity);
        aux_v_max = aux_v_max.max(out.aux_velocity[0].abs());
        if out.state == IplState::Finished {
            assert!(
                (out.aux[0] - 90.0).abs() < 1e-8,
                "A ended at {}",
                out.aux[0]
            );
            assert!(
                v_max_seen <= 10.0 * 10.0 / 90.0 + 1e-6,
                "path velocity {v_max_seen} exceeds aux cap"
            );
            assert!(aux_v_max <= 10.0 + 1e-6, "aux velocity {aux_v_max}");
            // A-axis setpoint from the driver matches
            let a_end = driver.log.position.last().unwrap()[3];
            assert!((a_end - 90.0).abs() < 1e-8);
            return;
        }
    }
    panic!("aux program did not finish");
}

#[test]
fn planned_facts_are_readable_from_the_interpolator() {
    // A rapid from a non-zero machine start, then a fed move, with an aux axis
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_position: [1.0, 2.0, 0.0],
        start_aux: vec![7.0],
        ..Default::default()
    };
    let queue = Decoder::decode_source(cfg, "G0 X5 A10\nG1 F600 Y12\n").expect("decodes");
    let mut ipl = Interpolator::new(limits(), LookAhead::default(), vec![100.0], DT);
    ipl.load(queue).expect("loads");

    // (b) the start pose is available before the first cycle...
    assert_eq!(ipl.start_position(), [1.0, 2.0, 0.0]);
    assert_eq!(ipl.start_aux(), &[7.0]);
    // (c) ...as is the planned total length: rapid 4 mm + fed move 10 mm
    assert!(
        (ipl.total_length() - 14.0).abs() < 1e-9,
        "{}",
        ipl.total_length()
    );
    // (a) and the first element is the rapid
    assert!(ipl.current_is_rapid());

    // ...and the seeded output buffer carries the start pose.
    let mut out = ipl.output_buffer();
    assert_eq!(out.path_position, [1.0, 2.0, 0.0]);
    assert_eq!(out.aux, vec![7.0]);
    assert_eq!(out.state, IplState::Running);

    // The rapid flag follows the path element under the interpolator.
    let mut rapid_while_first_object = true;
    let mut saw_fed_move = false;
    for _ in 0..200_000 {
        ipl.cycle_into(&mut out).expect("cycle");
        if out.obj_no == 0 {
            rapid_while_first_object &= ipl.current_is_rapid();
        } else {
            saw_fed_move = true;
            assert!(!ipl.current_is_rapid(), "G1 move reported as a rapid");
        }
        if out.state == IplState::Finished {
            break;
        }
    }
    assert!(rapid_while_first_object, "G0 move not reported as a rapid");
    assert!(saw_fed_move, "second move never reached");
    assert_eq!(out.state, IplState::Finished);
    assert!((out.progress - 1.0).abs() < 1e-12);
    assert!((out.s_total - ipl.total_length()).abs() < 1e-8);
}

#[test]
fn cycle_into_is_identical_to_cycle() {
    let src = "G1 F1200 X10 A45 M8\nG4 P0.05\nG3 X14 Y4 I0 J4 A90\nG0 X0 Y0\n";
    let cfg = || DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        ..Default::default()
    };
    let make = || {
        let mut ipl = Interpolator::new(limits(), LookAhead::default(), vec![100.0], DT);
        ipl.load(Decoder::decode_source(cfg(), src).expect("decodes"))
            .expect("loads");
        ipl
    };
    let mut allocating = make();
    let mut reusing = make();
    let mut buf = reusing.output_buffer();

    for i in 0..200_000 {
        let a = allocating.cycle().expect("cycle");
        reusing.cycle_into(&mut buf).expect("cycle_into");
        assert_eq!(a.state, buf.state, "cycle {i}");
        assert_eq!(a.path_position, buf.path_position, "cycle {i}");
        assert_eq!(a.path_tangent, buf.path_tangent, "cycle {i}");
        assert_eq!(a.path_velocity, buf.path_velocity, "cycle {i}");
        assert_eq!(a.path_acceleration, buf.path_acceleration, "cycle {i}");
        assert_eq!(a.aux, buf.aux, "cycle {i}");
        assert_eq!(a.aux_velocity, buf.aux_velocity, "cycle {i}");
        assert_eq!(a.events, buf.events, "cycle {i}");
        assert_eq!(a.src_line, buf.src_line, "cycle {i}");
        assert_eq!(a.obj_no, buf.obj_no, "cycle {i}");
        assert_eq!(a.s_total, buf.s_total, "cycle {i}");
        assert_eq!(a.progress, buf.progress, "cycle {i}");
        if a.state == IplState::Finished {
            // The M8 event was delivered exactly once, through both roads
            return;
        }
    }
    panic!("program did not finish");
}

#[test]
fn preprocessed_rounded_square_is_faster_and_exact() {
    let src = "G1 F1200 X20\nY20\nX0\nY0\n";
    let sharp = run_program(src);

    let mut cnc = controller();
    let mut pipeline = Pipeline::new().with(CornerRounding::new(2.0));
    cnc.load_program(src, DecoderConfig::default(), &mut pipeline)
        .unwrap();
    let mut driver = SimDriver::<3>::new();
    let mut outputs = Vec::new();
    for _ in 0..200_000 {
        let out = cnc.cycle(&mut driver).unwrap();
        let done = out.state == IplState::Finished;
        outputs.push(out);
        if done {
            break;
        }
    }
    let rounded = Run {
        outputs,
        log: driver.log,
    };
    assert!(rounded.outputs.last().unwrap().state == IplState::Finished);
    assert_path_limits(&rounded, &limits());
    // Rounding shortens the path and carries speed through corners: faster
    assert!(
        rounded.outputs.len() < sharp.outputs.len(),
        "rounded {} cycles vs sharp {}",
        rounded.outputs.len(),
        sharp.outputs.len()
    );
    let end = rounded.outputs.last().unwrap();
    assert!(end.path_position[0].abs() < 1e-8 && end.path_position[1].abs() < 1e-8);
}
