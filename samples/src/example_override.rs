//! Feed override during a long move: 100% → 40% → 0% (pause at standstill)
//! → 150% (resume above the programmed feed, still inside machine limits).
//! Every change simply retargets the jerk-limited s-profile from the
//! current state - that is rsruckig's instantaneous-retargeting strength.
//!
//! Saves `example_override.svg`.

mod plot_util;

use plot_util::Series;
use rscnc::prelude::*;

const DT: f64 = 0.001;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let program = "G21 G90\nG1 F1500 X100\nM30\n";

    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [80.0; 3], [500.0; 3], [8000.0; 3]);
    let limits = PathLimits::new(80.0, 500.0, 8000.0, 500.0);
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
    let (mut t, mut v_path, mut ovr_series) = (Vec::new(), Vec::new(), Vec::new());
    let mut i = 0usize;
    loop {
        match i {
            600 => cnc.interpolator.set_override(0.4),
            1400 => cnc.interpolator.set_override(0.0),
            2000 => cnc.interpolator.set_override(1.5),
            _ => {}
        }
        let out = cnc.cycle(&mut driver)?;
        t.push(*driver.log.time.last().unwrap());
        v_path.push(out.path_velocity);
        // Scale the override to mm/s of programmed feed for the plot
        ovr_series.push(cnc.interpolator.override_factor() * 25.0);
        if out.state == IplState::Finished {
            break;
        }
        i += 1;
    }

    plot_util::plot(
        "Feed override 100% -> 40% -> 0% (pause) -> 150%",
        "Velocity [mm/s]",
        &[
            Series::new("path velocity", &t, &v_path),
            Series::dashed("override x programmed feed", &t, &ovr_series),
        ],
    )?;
    Ok(())
}
