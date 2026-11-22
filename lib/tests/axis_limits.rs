//! Per-axis limits on the path (acceleration, jerk, velocity jump at a
//! junction), fly-by M functions and software end positions.

use rscnc::prelude::*;

const DT: f64 = 0.001;

fn limits() -> PathLimits {
    PathLimits::new(50.0, 500.0, 5000.0, 500.0)
}

fn run_queue(queue: GeoQueue, lookahead: LookAhead, axis_limits: AxisLimits) -> Vec<IplOutput> {
    let mut ipl = Interpolator::with_axis_limits(limits(), lookahead, axis_limits, DT);
    ipl.load(queue).expect("program loads");
    let mut outputs = Vec::new();
    for _ in 0..400_000 {
        let out = ipl.cycle().expect("cycle succeeds");
        let done = out.state == IplState::Finished;
        outputs.push(out);
        if done {
            return outputs;
        }
    }
    panic!("program did not finish");
}

fn run(src: &str, cfg: DecoderConfig, axis_limits: AxisLimits) -> Vec<IplOutput> {
    let queue = Decoder::decode_source(cfg, src).expect("program decodes");
    run_queue(queue, LookAhead::default(), axis_limits)
}

fn path_limits(x: AxisLimit, y: AxisLimit) -> AxisLimits {
    AxisLimits {
        path: [x, y, AxisLimit::UNLIMITED],
        aux: Vec::new(),
    }
}

/// Largest |acceleration| and |jerk| of path coordinate `k` over the run
/// (the path is straight, so the tangent is constant between two cycles).
fn axis_peaks(outputs: &[IplOutput], k: usize) -> (f64, f64) {
    let (mut acc, mut jerk) = (0.0f64, 0.0f64);
    for w in outputs.windows(2) {
        let t = w[1].path_tangent[k].abs();
        acc = acc.max(w[1].path_acceleration.abs() * t);
        jerk = jerk.max((w[1].path_acceleration - w[0].path_acceleration).abs() / DT * t);
    }
    (acc, jerk)
}

#[test]
fn a_weak_axis_limits_the_acceleration_of_the_moves_it_takes_part_in() {
    let weak = AxisLimit::new(f64::INFINITY, 100.0, 1000.0);
    let axes = || path_limits(AxisLimit::UNLIMITED, weak);

    // Along X the weak Y axis does not move: the path limits apply.
    let along_x = run("G1 F50\nX40", DecoderConfig::default(), axes());
    let (acc, _) = axis_peaks(&along_x, 0);
    assert!(acc > 300.0 && acc <= 500.0 * (1.0 + 1e-6), "x: {acc}");

    // Along Y its limits are the path's.
    let along_y = run("G1 F50\nY40", DecoderConfig::default(), axes());
    let (acc, jerk) = axis_peaks(&along_y, 1);
    assert!(acc > 60.0 && acc <= 100.0 * (1.0 + 1e-6), "y: {acc}");
    assert!(jerk > 600.0 && jerk <= 1000.0 * (1.0 + 1e-6), "y: {jerk}");
    assert!(along_y.len() > along_x.len());

    // On a diagonal the path may accelerate by 1 / |t_y| more.
    let diagonal = run("G1 F50\nX30 Y30", DecoderConfig::default(), axes());
    let (acc, jerk) = axis_peaks(&diagonal, 1);
    assert!(acc > 60.0 && acc <= 100.0 * (1.0 + 1e-6), "diagonal: {acc}");
    assert!(jerk <= 1000.0 * (1.0 + 1e-6), "diagonal: {jerk}");
    let path_acc = diagonal
        .iter()
        .fold(0.0f64, |m, o| m.max(o.path_acceleration.abs()));
    assert!(path_acc > 100.0, "diagonal path acceleration {path_acc}");
}

#[test]
fn an_auxiliary_axis_limits_acceleration_and_jerk() {
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        ..Default::default()
    };
    let axis_limits = AxisLimits {
        path: [AxisLimit::UNLIMITED; 3],
        aux: vec![AxisLimit::new(f64::INFINITY, 200.0, 2000.0)],
    };
    // A travels ten times as far as the path.
    let outputs = run("G1 F50\nX10 A100", cfg, axis_limits);
    // The axis follows the path at ten times its rate.
    let (mut vel, mut acc, mut jerk) = (0.0f64, 0.0f64, 0.0f64);
    for w in outputs.windows(2) {
        assert!((w[1].aux_velocity[0] - 10.0 * w[1].path_velocity).abs() < 1e-9);
        vel = vel.max(w[1].aux_velocity[0].abs());
        acc = acc.max(10.0 * w[1].path_acceleration.abs());
        jerk = jerk.max(10.0 * (w[1].path_acceleration - w[0].path_acceleration).abs() / DT);
    }
    assert!(vel > 100.0, "velocity {vel}");
    assert!(
        acc > 120.0 && acc <= 200.0 * (1.0 + 1e-6),
        "acceleration {acc}"
    );
    assert!(
        jerk > 1200.0 && jerk <= 2000.0 * (1.0 + 1e-6),
        "jerk {jerk}"
    );
    let last = outputs.last().unwrap();
    assert!((last.aux[0] - 100.0).abs() < 1e-9);
}

#[test]
fn the_velocity_jump_limit_slows_a_corner() {
    let src = "G1 F50\nX20\nY20";
    let queue = || Decoder::decode_source(DecoderConfig::default(), src).unwrap();
    let corner_velocity = |axis_limits: AxisLimits| {
        let mut ipl =
            Interpolator::with_axis_limits(limits(), LookAhead::default(), axis_limits, DT);
        ipl.load(queue()).unwrap();
        ipl.planned_segments()[0].v_exit
    };
    let free = corner_velocity(AxisLimits::default());
    assert!(free > 5.0, "corner without a jump limit: {free}");

    // A right-angle corner at velocity v steps both axes by v.
    let jump = AxisLimit::UNLIMITED.with_velocity_jump(3.0);
    let limited = corner_velocity(path_limits(jump, jump));
    assert!((limited - 3.0).abs() < 1e-9, "corner velocity {limited}");

    // Only the weaker axis counts.
    let strict = AxisLimit::UNLIMITED.with_velocity_jump(1.5);
    let limited = corner_velocity(path_limits(jump, strict));
    assert!((limited - 1.5).abs() < 1e-9, "corner velocity {limited}");

    // With the corner rule off, the jump limit alone sets the corner.
    let lookahead = LookAhead {
        junction_deviation: f64::INFINITY,
        ..LookAhead::default()
    };
    let outputs = run_queue(queue(), lookahead, path_limits(jump, jump));
    let mut worst = 0.0f64;
    for w in outputs.windows(2) {
        let tangents = w[0].path_tangent.iter().zip(&w[1].path_tangent);
        for (t0, t1) in tangents.take(2) {
            let v0 = w[0].path_velocity * t0;
            let v1 = w[1].path_velocity * t1;
            // within a cycle the axis may also accelerate at its limit
            worst = worst.max((v1 - v0).abs() - 500.0 * DT);
        }
    }
    assert!(worst <= 3.0 * (1.0 + 1e-6), "axis velocity step {worst}");
    assert!(
        worst > 2.5,
        "the corner is taken at the jump limit: {worst}"
    );
}

#[test]
fn a_straight_junction_is_not_slowed_by_the_jump_limit() {
    let jump = AxisLimit::UNLIMITED.with_velocity_jump(0.5);
    let outputs = run(
        "G1 F50\nX20\nX40",
        DecoderConfig::default(),
        path_limits(jump, jump),
    );
    let at_junction = outputs
        .iter()
        .find(|o| o.s_total >= 20.0)
        .expect("passes the junction");
    assert!(at_junction.path_velocity > 49.0);
}

#[test]
fn a_fly_by_m_function_does_not_stop_the_path() {
    let cfg = DecoderConfig {
        fly_by_m: vec![8],
        ..Default::default()
    };
    let outputs = run("G1 F50\nX20\nM8\nX40\nM9\nX60", cfg, AxisLimits::default());
    let mut events = Vec::new();
    for (i, out) in outputs.iter().enumerate() {
        for event in &out.events {
            events.push((i, event.clone()));
        }
    }
    assert_eq!(events.len(), 2, "{events:?}");

    // M8 is reported once, in the cycle that crosses its junction at speed.
    let (i, m8) = &events[0];
    assert_eq!(m8.m, vec![8]);
    assert!(m8.fly_by);
    let (before, at) = (&outputs[i - 1], &outputs[*i]);
    assert!(
        before.s_total < 20.0 && at.s_total >= 20.0,
        "{} {}",
        before.s_total,
        at.s_total
    );
    assert!(at.path_velocity > 49.0, "velocity {}", at.path_velocity);
    assert_eq!(at.state, IplState::Running);

    // M9 is not listed: the path stops for it.
    let (i, m9) = &events[1];
    assert_eq!(m9.m, vec![9]);
    assert!(!m9.fly_by);
    assert!((outputs[*i].s_total - 40.0).abs() < 1e-9);
    assert!(outputs[*i].path_velocity.abs() < 1e-9);
}

#[test]
fn a_block_with_other_words_is_not_fly_by() {
    let cfg = DecoderConfig {
        fly_by_m: vec![8],
        ..Default::default()
    };
    // M8 with a tool change, and M8 next to an unlisted M function
    for src in ["G1 F50\nX20\nM8 T2\nX40", "G1 F50\nX20\nM8 M3\nX40"] {
        let outputs = run(src, cfg.clone(), AxisLimits::default());
        let stop = outputs
            .iter()
            .find(|o| !o.events.is_empty())
            .expect("the event is reported");
        assert!(!stop.events[0].fly_by, "{src}");
        assert!(stop.path_velocity.abs() < 1e-9, "{src}");
        assert!((stop.s_total - 20.0).abs() < 1e-9, "{src}");
    }
}

#[test]
fn fly_by_events_survive_halt_and_single_step() {
    let cfg = DecoderConfig {
        fly_by_m: vec![7, 8],
        ..Default::default()
    };
    let src = "G1 F50\nX5\nM8\nX10\nM7\nM8\nX15\nM8\nX15.01\nM7\nX30";
    let queue = Decoder::decode_source(cfg, src).unwrap();
    let mut ipl = Interpolator::new(limits(), LookAhead::default(), Vec::new(), DT);
    ipl.load(queue).unwrap();
    let mut seen = Vec::new();
    let mut s_prev = 0.0;
    for i in 0..200_000 {
        match i {
            120 => ipl.halt(),
            400 => ipl.resume(),
            450 => ipl.set_single_step(true),
            _ if i > 450 && i % 500 == 0 => ipl.step(),
            _ => {}
        }
        let out = ipl.cycle().unwrap();
        assert!(out.s_total >= s_prev);
        s_prev = out.s_total;
        seen.extend(out.events.iter().map(|e| e.m.clone()));
        if out.state == IplState::Finished {
            assert_eq!(
                seen,
                vec![vec![8], vec![7], vec![8], vec![8], vec![7]],
                "every event exactly once, in order"
            );
            assert!((out.s_total - 30.0).abs() < 1e-9);
            return;
        }
    }
    panic!("did not finish");
}

#[test]
fn moves_with_nearly_equal_axis_limits_share_one_ramp() {
    // A quarter circle of radius 50 in 0.5 mm chords: every chord points
    // a little differently, so the axis limits give each one slightly
    // different path limits. They must still be planned as one span — a
    // ramp that returned to zero acceleration at every chord would take
    // many times longer.
    let mut src = String::from("G1 F50\n");
    let n = 157;
    for i in 1..=n {
        let a = i as f64 / n as f64 * std::f64::consts::FRAC_PI_2;
        src += &format!("X{:.5} Y{:.5}\n", 50.0 * a.sin(), 50.0 * (1.0 - a.cos()));
    }
    let x = AxisLimit::new(f64::INFINITY, 400.0, 5000.0);
    let y = AxisLimit::new(f64::INFINITY, 300.0, 4000.0);
    let outputs = run(&src, DecoderConfig::default(), path_limits(x, y));
    let ramp: Vec<_> = outputs
        .iter()
        .take_while(|o| o.path_velocity < 40.0)
        .filter(|o| o.path_velocity > 10.0)
        .collect();
    assert!(ramp.len() > 20);
    for o in &ramp {
        assert!(
            o.path_acceleration > 100.0,
            "the ramp is interrupted at s = {}: a = {}",
            o.s_total,
            o.path_acceleration
        );
    }
    // Along the path the axes stay within their limits all the way (the
    // chord corners add their own small velocity steps on top).
    for o in &outputs {
        for (k, limit) in [(0, 400.0), (1, 300.0)] {
            let a = o.path_acceleration * o.path_tangent[k];
            assert!(
                a.abs() <= limit * (1.0 + 1e-6),
                "axis {k}: {a} at s = {}",
                o.s_total
            );
        }
    }
}

#[test]
fn a_halt_running_into_a_move_with_a_lower_jerk_limit_stops_cleanly() {
    // The second move runs along the weak Y axis: its jerk limit is a
    // fraction of the first one's. A stop that begins on the first move and
    // ends on the second arrives there braking harder than the second
    // move's jerk could take back before standstill; it must still end
    // as one continuous stop, whenever the halt comes.
    let weak = AxisLimit::new(f64::INFINITY, 400.0, 400.0);
    let src = "G1 F50\nX20\nX26 Y6\nX40 Y6";
    let lim = limits();
    let ds_tol = 0.2 * lim.max_jerk * DT * DT * DT + 1e-9;
    for halt_at in 300..520 {
        let queue = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
        let mut ipl = Interpolator::with_axis_limits(
            lim,
            LookAhead::default(),
            path_limits(AxisLimit::UNLIMITED, weak),
            DT,
        );
        ipl.load(queue).unwrap();
        let (mut s_prev, mut v_prev) = (0.0, 0.0);
        let mut finished = false;
        for i in 0..20_000 {
            if i == halt_at {
                ipl.halt();
            } else if i == halt_at + 400 {
                ipl.resume();
            }
            let out = ipl.cycle().unwrap();
            let ds = out.s_total - s_prev;
            let expected = 0.5 * (v_prev + out.path_velocity) * DT;
            assert!(
                ds >= 0.0 && (ds - expected).abs() <= ds_tol,
                "halt at {halt_at}, cycle {i}: moved {ds}, expected {expected} ({:?})",
                out.state
            );
            assert!(
                ((out.path_velocity - v_prev) / DT).abs() <= lim.max_acceleration * (1.0 + 1e-6),
                "halt at {halt_at}, cycle {i}"
            );
            (s_prev, v_prev) = (out.s_total, out.path_velocity);
            if out.state == IplState::Finished {
                finished = true;
                break;
            }
        }
        assert!(finished, "halt at {halt_at}");
    }
}

fn controller(min: [f64; 3], max: [f64; 3]) -> CncController<3, CartesianKinematics<3>> {
    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [50.0; 3], [500.0; 3], [5000.0; 3])
            .with_soft_limits(min, max);
    CncController::new(
        axis_config,
        limits(),
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        DT,
    )
}

#[test]
fn a_program_beyond_a_soft_limit_is_refused_at_load() {
    let inf = f64::INFINITY;
    let mut cnc = controller([-1.0, -1.0, -inf], [100.0, 50.0, inf]);
    let mut load =
        |src: &str| cnc.load_program(src, DecoderConfig::default(), &mut Pipeline::new());

    // Inside, and ending exactly on a limit
    load("G1 F50\nX100 Y50\nX0 Y0").expect("inside the limits");

    // An end point outside
    match load("G1 F50\nX50\nX50 Y50.5\nX0") {
        Err(CncError::SoftLimit { line, message }) => {
            assert_eq!(line, 3);
            assert!(message.contains("axis Y"), "{message}");
        }
        other => panic!("expected a soft limit error, got {other:?}"),
    }

    // An arc whose ends are inside but which bulges out: a half circle
    // from (40, 20) to (40, 40) through (50, 30) is fine, the same one
    // from (95, 20) reaches X = 105.
    load("G1 F50\nX40 Y20\nG3 X40 Y40 I0 J10").expect("the arc is inside");
    match load("G1 F50\nX95 Y20\nG3 X95 Y40 I0 J10") {
        Err(CncError::SoftLimit { line, message }) => {
            assert_eq!(line, 3);
            assert!(message.contains("axis X"), "{message}");
        }
        other => panic!("expected a soft limit error, got {other:?}"),
    }

    // The lower limits
    assert!(matches!(
        load("G1 F50\nX-1.5"),
        Err(CncError::SoftLimit { line: 2, .. })
    ));
}

#[test]
fn a_refused_program_leaves_the_loaded_one_in_place() {
    let inf = f64::INFINITY;
    let mut cnc = controller([-inf; 3], [10.0, inf, inf]);
    let mut pipeline = Pipeline::new();
    cnc.load_program("G1 F50\nX5", DecoderConfig::default(), &mut pipeline)
        .unwrap();
    let total = cnc.interpolator.total_length();
    assert!(cnc
        .load_program("G1 F50\nX50", DecoderConfig::default(), &mut pipeline)
        .is_err());
    assert_eq!(cnc.interpolator.total_length(), total);
}
