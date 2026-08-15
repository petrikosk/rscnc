//! A rounded rectangle programmed with G1 lines and tangent G2 arcs:
//! because every junction is tangent-continuous, the look-ahead blends
//! through all of them and the velocity never drops to zero mid-contour.
//! The arc feed is additionally capped by the centripetal-acceleration
//! budget (v ≤ √(a_n·r)).
//!
//! Saves `example_arcs.svg` and `example_arcs_path.svg`.

mod plot_util;

use plot_util::Series;
use rscnc::prelude::*;

const DT: f64 = 0.001;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 40×20 rectangle with r=5 corners, CW contour (G2 arcs)
    let program = "\
N10 G21 G90
N20 G1 F50 X35
N30 G2 X40 Y5 I0 J5
N40 G1 Y15
N50 G2 X35 Y20 I-5 J0
N60 G1 X5
N70 G2 X0 Y15 I0 J-5
N80 G1 Y5
N90 G2 X5 Y0 I5 J0
N100 M30
";

    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [80.0; 3], [1000.0; 3], [15000.0; 3]);
    // Tight normal-acceleration budget so the arc cap is clearly visible:
    // v_arc ≤ sqrt(200·5) ≈ 31.6 mm/s < feed 50 mm/s
    let limits = PathLimits::new(80.0, 1000.0, 15000.0, 200.0);
    let mut cnc = CncController::new(
        axis_config,
        limits,
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        DT,
    );
    let mut pipeline = Pipeline::new();
    cnc.load_program(program, DecoderConfig::default(), &mut pipeline)?;

    let mut driver = SimDriver::<3>::new();
    let (mut t, mut v_path) = (Vec::new(), Vec::new());
    loop {
        let out = cnc.cycle(&mut driver)?;
        t.push(*driver.log.time.last().unwrap());
        v_path.push(out.path_velocity);
        if out.state == IplState::Finished {
            break;
        }
    }

    let x = driver.log.axis_positions(0);
    let y = driver.log.axis_positions(1);
    let t_ends = vec![t[0], *t.last().unwrap()];
    let arc_cap = vec![(200.0f64 * 5.0).sqrt(); 2];
    let feed = vec![50.0; 2];
    plot_util::plot(
        "Rounded rectangle - blended path velocity",
        "Velocity [mm/s]",
        &[
            Series::new("path velocity", &t, &v_path),
            Series::dashed("programmed feed", &t_ends, &feed),
            Series::dashed("arc curvature cap", &t_ends, &arc_cap),
        ],
    )?;
    plot_util::plot_xy(
        "Rounded rectangle - XY path",
        &[Series::new("path", &x, &y)],
    )?;
    Ok(())
}
