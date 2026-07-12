//! Property tests: random programs through the full pipeline must always
//! respect kinematic limits, stay on the programmed geometry and reach the
//! exact endpoint.

use proptest::prelude::*;
use rscnc::prelude::*;

const DT: f64 = 0.001;

fn limits() -> PathLimits {
    PathLimits::new(50.0, 500.0, 5000.0, 500.0)
}

fn run(src: &str) -> Vec<IplOutput> {
    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [50.0; 3], [500.0; 3], [5000.0; 3]);
    let mut cnc = CncController::new(
        axis_config,
        limits(),
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        DT,
    );
    let mut pipeline = Pipeline::new();
    cnc.load_program(src, DecoderConfig::default(), &mut pipeline)
        .expect("program loads");
    let mut driver = SimDriver::<3>::new();
    let mut outputs = Vec::new();
    for _ in 0..400_000 {
        let out = cnc.cycle(&mut driver).expect("cycle succeeds");
        let done = out.state == IplState::Finished;
        outputs.push(out);
        if done {
            return outputs;
        }
    }
    panic!("program did not finish");
}

fn assert_limits(outputs: &[IplOutput]) {
    let lim = limits();
    for (i, out) in outputs.iter().enumerate() {
        assert!(out.path_velocity <= lim.max_velocity * (1.0 + 1e-9) + 1e-9);
        assert!(out.path_velocity >= 0.0);
        if i > 0 {
            let dv = (out.path_velocity - outputs[i - 1].path_velocity).abs();
            assert!(
                dv / DT <= lim.max_acceleration * (1.0 + 1e-6) + 1e-6,
                "cycle {i}: mean accel {}",
                dv / DT
            );
            let da = (out.path_acceleration - outputs[i - 1].path_acceleration).abs();
            assert!(
                da / DT <= lim.max_jerk * (1.0 + 1e-6) + 1e-6,
                "cycle {i}: mean jerk {}",
                da / DT
            );
        }
    }
}

/// Distance from `p` to segment `a`→`b` in the XY plane.
fn dist_to_segment(p: [f64; 3], a: (f64, f64), b: (f64, f64)) -> f64 {
    let (px, py) = (p[0], p[1]);
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((px - a.0) * dx + (py - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (cx, cy) = (a.0 + t * dx, a.1 + t * dy);
    ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(16))]

    #[test]
    fn random_polyline(
        raw_pts in prop::collection::vec((-30.0..30.0f64, -30.0..30.0f64), 1..7),
        feed in 600.0..3000.0f64,
    ) {
        // Round to the precision written into the program text, so the
        // assertions compare against what was actually programmed
        let pts: Vec<(f64, f64)> = raw_pts
            .iter()
            .map(|(x, y)| ((x * 1e4).round() / 1e4, (y * 1e4).round() / 1e4))
            .collect();
        let mut src = format!("G1 F{feed:.3}\n");
        for (x, y) in &pts {
            src.push_str(&format!("X{x:.4} Y{y:.4}\n"));
        }
        let outputs = run(&src);
        assert_limits(&outputs);

        // Endpoint: the last programmed point, exactly
        let end = outputs.last().unwrap();
        let (ex, ey) = *pts.last().unwrap();
        prop_assert!((end.path_position[0] - ex).abs() < 1e-8);
        prop_assert!((end.path_position[1] - ey).abs() < 1e-8);
        prop_assert!(end.path_velocity.abs() < 1e-9);

        // Geometry adherence: every sample lies on some programmed segment
        let mut vertices = vec![(0.0, 0.0)];
        vertices.extend(pts.iter().copied());
        for (i, out) in outputs.iter().enumerate() {
            let d = vertices
                .windows(2)
                .map(|w| dist_to_segment(out.path_position, w[0], w[1]))
                .fold(f64::INFINITY, f64::min);
            prop_assert!(d < 1e-8, "cycle {}: {} off the programmed path", i, d);
        }
    }

    #[test]
    fn random_arc(
        center_angle in 0.0..core::f64::consts::TAU,
        radius in 2.0..40.0f64,
        sweep_frac in 0.05..0.95f64,
        ccw in any::<bool>(),
        raw_feed in 600.0..3000.0f64,
    ) {
        // Match the precision written into the program text
        let feed = (raw_feed * 1e3).round() / 1e3;
        // Start at origin; center at angle `center_angle` from start
        let cx = radius * center_angle.cos();
        let cy = radius * center_angle.sin();
        // End point on the circle at the given sweep from the start
        let start_angle = (0.0f64 - cy).atan2(0.0 - cx);
        let sweep = sweep_frac * core::f64::consts::TAU * if ccw { 1.0 } else { -1.0 };
        let end_angle = start_angle + sweep;
        let ex = cx + radius * end_angle.cos();
        let ey = cy + radius * end_angle.sin();
        let g = if ccw { 3 } else { 2 };
        let src = format!("G{g} F{feed:.3} X{ex:.9} Y{ey:.9} I{cx:.9} J{cy:.9}\n");

        let outputs = run(&src);
        assert_limits(&outputs);

        // Every sample stays on the circle
        for out in &outputs {
            let r = ((out.path_position[0] - cx).powi(2) + (out.path_position[1] - cy).powi(2)).sqrt();
            prop_assert!((r - radius).abs() < 1e-8, "radial error {}", r - radius);
        }
        let end = outputs.last().unwrap();
        prop_assert!((end.path_position[0] - ex).abs() < 1e-7);
        prop_assert!((end.path_position[1] - ey).abs() < 1e-7);

        // Curvature cap honored
        let v_cap = (limits().max_normal_acceleration * radius)
            .sqrt()
            .min(feed / 60.0)
            .min(limits().max_velocity);
        let v_max_seen = outputs.iter().map(|o| o.path_velocity).fold(0.0, f64::max);
        prop_assert!(v_max_seen <= v_cap + 1e-6, "v {} > cap {}", v_max_seen, v_cap);
    }
}
