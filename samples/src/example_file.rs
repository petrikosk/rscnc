//! Run a G-code file through the full pipeline (tool radius correction +
//! corner rounding preprocessors enabled) and plot the result.
//!
//! Usage: `cargo run --bin example_file [-- path/to/program.nc]`
//! (defaults to the bundled `samples/gcode/demo.nc`).
//!
//! Saves `example_file.svg` and `example_file_path.svg`.

mod plot_util;

use plot_util::Series;
use rscnc::prelude::*;

const DT: f64 = 0.001;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| concat!(env!("CARGO_MANIFEST_DIR"), "/gcode/demo.nc").to_string());
    let program = std::fs::read_to_string(&path)?;
    println!("Running '{path}'");

    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [100.0; 3], [1200.0; 3], [20000.0; 3]);
    let limits = PathLimits::new(100.0, 1200.0, 20000.0, 400.0);
    let mut cnc = CncController::new(
        axis_config,
        limits,
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        DT,
    );
    // Full preprocessing chain: 4 mm tool radius, 1 mm corner rounding,
    // and a conservative curvature-based feed clamp
    let mut pipeline = Pipeline::new()
        .with(ToolRadiusCorrection::new(4.0))
        .with(CornerRounding::new(1.0))
        .with(CurvatureVelocityLimit::new(400.0));
    let cfg = DecoderConfig {
        rapid_feed: 100.0,
        ..Default::default()
    };
    cnc.load_program(&program, cfg, &mut pipeline)?;

    let mut driver = SimDriver::<3>::new();
    let (mut t, mut v_path) = (Vec::new(), Vec::new());
    loop {
        let out = cnc.cycle(&mut driver)?;
        t.push(*driver.log.time.last().unwrap());
        v_path.push(out.path_velocity);
        for sp in &out.switch_points {
            println!(
                "t = {:7.3} s  line {:3}: switch point {} {}",
                t.last().unwrap(),
                sp.src_line,
                sp.id(),
                if sp.is_on() { "on" } else { "off" }
            );
        }
        for e in &out.events {
            println!(
                "t = {:7.3} s  line {:3}: M{:?} S{:?} T{:?}",
                t.last().unwrap(),
                e.src_line,
                e.m,
                e.spindle,
                e.tool
            );
        }
        if out.state == IplState::Finished {
            break;
        }
    }
    println!("Finished in {:.3} s", t.last().unwrap());

    let x = driver.log.axis_positions(0);
    let y = driver.log.axis_positions(1);
    plot_util::plot(
        "G-code file - path velocity",
        "Velocity [mm/s]",
        &[Series::new("path velocity", &t, &v_path)],
    )?;
    plot_util::plot_xy(
        "G-code file - XY path (tool center after G41 correction)",
        &[Series::new("tool path", &x, &y)],
    )?;
    Ok(())
}
