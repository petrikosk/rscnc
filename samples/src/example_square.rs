//! A 20×20 mm square with sharp corners: the look-ahead slows down for each
//! 90° corner (junction-deviation rule) but never fully stops, and the path
//! velocity profile stays jerk-limited throughout.
//!
//! Saves `example_square.svg` (velocities over time) and
//! `example_square_path.svg` (XY geometry).

mod plot_util;

use plot_util::Series;
use rscnc::prelude::*;

const DT: f64 = 0.001;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let program = "\
N10 G21 G90
N20 G1 F1500 X20
N30 Y20
N40 X0
N50 Y0
N60 M30
";

    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [60.0; 3], [800.0; 3], [10000.0; 3]);
    let limits = PathLimits::new(60.0, 800.0, 10000.0, 800.0);
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
    let mut t = Vec::new();
    let mut v_path = Vec::new();
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
    let feed = vec![25.0; 2];
    let t_ends = vec![t[0], *t.last().unwrap()];
    plot_util::plot(
        "Square 20x20 - path velocity and axis positions",
        "Velocity [mm/s] / Position [mm]",
        &[
            Series::new("path velocity", &t, &v_path),
            Series::new("X position", &t, &x),
            Series::new("Y position", &t, &y),
            Series::dashed("programmed feed", &t_ends, &feed),
        ],
    )?;
    plot_util::plot_xy("Square 20x20 - XY path", &[Series::new("path", &x, &y)])?;
    Ok(())
}
