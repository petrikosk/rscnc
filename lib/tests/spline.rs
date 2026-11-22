//! Spline geometry end to end: `G5` curves and smoothed corners through the
//! look-ahead and the interpolator.

use rscnc::prelude::*;

const DT: f64 = 0.001;

fn limits() -> PathLimits {
    PathLimits::new(50.0, 500.0, 5000.0, 500.0)
}

fn run_queue(queue: GeoQueue) -> Vec<IplOutput> {
    let mut ipl = Interpolator::new(limits(), LookAhead::default(), Vec::new(), DT);
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

/// One continuous jerk-limited profile within the path limits, and a
/// position that follows it without jumps.
fn assert_smooth(outputs: &[IplOutput]) {
    let lim = limits();
    let ds_tol = 0.2 * lim.max_jerk * DT * DT * DT + 1e-9;
    for (i, w) in outputs.windows(2).enumerate() {
        let (p, o) = (&w[0], &w[1]);
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
            "cycle {i}: moved {ds}, expected {expected}"
        );
        // The point in space moves as far as the path parameter says (a
        // chord is never longer than its arc, and hardly shorter).
        let chord = (0..3)
            .map(|k| (o.path_position[k] - p.path_position[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        // (at a corner between two elements the chord cuts across)
        let straight: f64 = (0..3).map(|k| o.path_tangent[k] * p.path_tangent[k]).sum();
        assert!(
            chord <= ds * (1.0 + 1e-6) + 1e-9 && (chord >= ds * 0.999 - 1e-9 || straight < 0.999),
            "cycle {i}: chord {chord} for ds {ds}"
        );
    }
}

/// Centripetal acceleration v² κ, estimated from the turning of the tangent.
fn max_normal_acceleration(outputs: &[IplOutput]) -> f64 {
    let mut worst = 0.0f64;
    for w in outputs.windows(2) {
        let (t0, t1) = (w[0].path_tangent, w[1].path_tangent);
        let turn = (0..3).map(|k| (t1[k] - t0[k]).powi(2)).sum::<f64>().sqrt();
        // (a corner between two elements is not curvature)
        if turn > 0.05 {
            continue;
        }
        let v = 0.5 * (w[0].path_velocity + w[1].path_velocity);
        worst = worst.max(v * turn / DT);
    }
    worst
}

#[test]
fn a_g5_curve_is_interpolated_smoothly_through_its_points() {
    let src = "G1 F50\nX10\nG5 X20 Y5\nX30 Y0\nX40 Y-8\nX50 Y0\nG1 X60";
    let queue = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
    let splines = queue
        .iter()
        .filter(|e| matches!(e, GeoElem::Move(m) if matches!(m.geo, Geometry::Spline(_))))
        .count();
    assert_eq!(splines, 4);
    let outputs = run_queue(queue);
    assert_smooth(&outputs);

    // The curve passes through every programmed point ...
    for target in [[20.0, 5.0], [30.0, 0.0], [40.0, -8.0], [50.0, 0.0]] {
        let nearest = outputs
            .iter()
            .map(|o| (o.path_position[0] - target[0]).hypot(o.path_position[1] - target[1]))
            .fold(f64::INFINITY, f64::min);
        assert!(nearest < 0.06, "{target:?}: {nearest}");
    }
    // ... ends where the program ends ...
    let last = outputs.last().unwrap();
    assert!((last.path_position[0] - 60.0).abs() < 1e-9 && last.path_position[1].abs() < 1e-9);
    // ... and is slowed where it bends, to the normal-acceleration limit.
    let a_n = max_normal_acceleration(&outputs);
    assert!(a_n <= 500.0 * 1.05, "normal acceleration {a_n}");
    // The tangent is continuous along the whole curve (no corner inside).
    for w in outputs.windows(2) {
        let (t0, t1) = (w[0].path_tangent, w[1].path_tangent);
        if t0 == [0.0; 3] {
            continue;
        }
        let dot: f64 = (0..3).map(|k| t0[k] * t1[k]).sum();
        // the curve joins the lines before and after it with a corner
        let on_curve = w[0].path_position[0] > 10.01 && w[1].path_position[0] < 49.99;
        if on_curve {
            assert!(dot > 0.99, "tangent jump at {:?}", w[1].path_position);
        }
    }
}

#[test]
fn smoothed_corners_keep_the_path_moving_within_tolerance() {
    let src = "G1 F50\nX20\nY20\nX0\nY0";
    let decode = || Decoder::decode_source(DecoderConfig::default(), src).unwrap();
    let tolerance = 0.2;
    let smoothed = CornerSmoothing::new(tolerance).process(decode()).unwrap();
    let plain = run_queue(decode());
    let outputs = run_queue(smoothed);
    assert_smooth(&outputs);
    // (a blend runs at the velocity its tightest point allows, so it is
    // not faster than the sharp corner — but it never stops the axes dead)
    assert!(
        (outputs.len() as f64) < plain.len() as f64 * 1.05,
        "smoothing costs little time: {} against {} cycles",
        outputs.len(),
        plain.len()
    );

    // Never further from the programmed square than the tolerance.
    let distance_to_square = |p: &PathPoint| {
        let edge = |a: [f64; 2], b: [f64; 2]| {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let u =
                (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            (p[0] - a[0] - u * dx).hypot(p[1] - a[1] - u * dy)
        };
        let c = [[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [0.0, 20.0]];
        (0..4)
            .map(|i| edge(c[i], c[(i + 1) % 4]))
            .fold(f64::INFINITY, f64::min)
    };
    let mut worst = 0.0f64;
    for o in &outputs {
        worst = worst.max(distance_to_square(&o.path_position));
    }
    assert!(worst <= tolerance * (1.0 + 1e-6), "deviation {worst}");
    assert!(worst > 0.5 * tolerance, "the corners are cut: {worst}");

    // The corners are passed in motion, within the normal acceleration.
    let slowest = outputs[200..outputs.len() - 200]
        .iter()
        .map(|o| o.path_velocity)
        .fold(f64::INFINITY, f64::min);
    assert!(slowest > 1.0, "slowest {slowest}");
    let a_n = max_normal_acceleration(&outputs);
    assert!(a_n <= 500.0 * 1.05, "normal acceleration {a_n}");
}

#[test]
fn a_spline_can_be_halted_overridden_and_single_stepped() {
    let src = "G1 F50\nG5 X10 Y5\nX20 Y-5\nX30 Y5\nX40 Y0\nG1 X50";
    let queue = Decoder::decode_source(DecoderConfig::default(), src).unwrap();
    let mut ipl = Interpolator::new(limits(), LookAhead::default(), Vec::new(), DT);
    ipl.load(queue).unwrap();
    let mut outputs = Vec::new();
    for i in 0..400_000 {
        match i {
            150 => ipl.set_override(0.3),
            400 => ipl.halt(),
            700 => ipl.resume(),
            900 => ipl.set_override(1.7),
            1100 => ipl.set_single_step(true),
            _ if i > 1100 && i % 400 == 0 => ipl.step(),
            _ => {}
        }
        let out = ipl.cycle().unwrap();
        let done = out.state == IplState::Finished;
        outputs.push(out);
        if done {
            break;
        }
    }
    assert_smooth(&outputs);
    let last = outputs.last().unwrap();
    assert_eq!(last.state, IplState::Finished);
    assert!((last.path_position[0] - 50.0).abs() < 1e-9);
}

#[test]
fn a_spline_program_runs_through_the_controller_with_soft_limits() {
    let inf = f64::INFINITY;
    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [50.0; 3], [500.0; 3], [5000.0; 3])
            .with_soft_limits([-inf; 3], [inf, 6.5, inf]);
    let mut cnc = CncController::new(
        axis_config,
        limits(),
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        DT,
    );
    // A curve through two points at the same height bulges above them in
    // between (by about a tenth here): beyond the limit at Y = 6.5 only
    // for the taller one.
    let mut pipeline = Pipeline::new();
    cnc.load_program(
        "G1 F50\nG5 X10 Y5.5\nX20 Y5.5\nX30 Y0",
        DecoderConfig::default(),
        &mut pipeline,
    )
    .expect("inside the limits");
    let mut driver = SimDriver::<3>::new();
    cnc.run_to_end(&mut driver, 100_000).expect("runs");
    assert!(matches!(
        cnc.load_program(
            "G1 F50\nG5 X10 Y6.4\nX20 Y6.4\nX30 Y0",
            DecoderConfig::default(),
            &mut pipeline,
        ),
        Err(CncError::SoftLimit { .. })
    ));
}
