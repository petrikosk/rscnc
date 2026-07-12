//! Shared SVG plotting helper for the examples (adapted from the rsruckig
//! samples), built on the pure-Rust `plotters` crate.
//!
//! Each example includes this file as a module (`mod plot_util;`) and calls
//! [`plot`] for time series and [`plot_xy`] for the path geometry; charts are
//! written to `<bin name><suffix>.svg` in the working directory.

use plotters::prelude::*;
use plotters::series::DashedLineSeries;

/// One line in the chart. The x values are per series, so different series
/// may cover different ranges (e.g. a constant limit line).
pub struct Series<'a> {
    pub name: &'a str,
    pub x: &'a [f64],
    pub y: &'a [f64],
    pub dashed: bool,
}

impl<'a> Series<'a> {
    pub fn new(name: &'a str, x: &'a [f64], y: &'a [f64]) -> Self {
        Self {
            name,
            x,
            y,
            dashed: false,
        }
    }

    /// A dashed line, e.g. for marking a limit
    #[allow(dead_code)] // not every example draws limit lines
    pub fn dashed(name: &'a str, x: &'a [f64], y: &'a [f64]) -> Self {
        Self {
            name,
            x,
            y,
            dashed: true,
        }
    }
}

const COLORS: [RGBColor; 6] = [RED, BLUE, GREEN, MAGENTA, CYAN, BLACK];

fn ranges(series: &[Series]) -> (f64, f64, f64, f64) {
    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;
    for s in series {
        for &x in s.x {
            x_min = x_min.min(x);
            x_max = x_max.max(x);
        }
        for &y in s.y {
            y_min = y_min.min(y);
            y_max = y_max.max(y);
        }
    }
    if !(x_min.is_finite() && x_min < x_max) {
        (x_min, x_max) = (0.0, 1.0);
    }
    if !(y_min.is_finite() && y_min < y_max) {
        (y_min, y_max) = (0.0, 1.0);
    }
    (x_min, x_max, y_min, y_max)
}

fn draw(
    path: &str,
    title: &str,
    x_label: &str,
    y_label: &str,
    series: &[Series],
    equal_aspect: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let (w, h) = if equal_aspect { (900, 900) } else { (1280, 768) };
    let root = SVGBackend::new(path, (w, h)).into_drawing_area();
    root.fill(&WHITE)?;

    let (mut x_min, mut x_max, mut y_min, mut y_max) = ranges(series);
    if equal_aspect {
        // Same span on both axes so circles look like circles
        let span = (x_max - x_min).max(y_max - y_min) * 1.1;
        let (xc, yc) = ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0);
        x_min = xc - span / 2.0;
        x_max = xc + span / 2.0;
        y_min = yc - span / 2.0;
        y_max = yc + span / 2.0;
    } else {
        let m = 0.05 * (y_max - y_min);
        y_min -= m;
        y_max += m;
    }

    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 25))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(x_min..x_max, y_min..y_max)?;

    chart
        .configure_mesh()
        .x_desc(x_label)
        .y_desc(y_label)
        .label_style(("sans-serif", 15))
        .draw()?;

    for (i, s) in series.iter().enumerate() {
        let color = COLORS[i % COLORS.len()];
        let points = s.x.iter().zip(s.y.iter()).map(|(&x, &y)| (x, y));
        let annotation = if s.dashed {
            chart.draw_series(DashedLineSeries::new(points, 8, 4, color.stroke_width(1)))?
        } else {
            chart.draw_series(LineSeries::new(points, &color))?
        };
        annotation
            .label(s.name)
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], color));
    }

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.8))
        .border_style(BLACK)
        .draw()?;

    root.present()?;
    println!("Plot saved to '{path}'");
    Ok(())
}

/// Render time series as `<bin name>.svg`.
pub fn plot(
    title: &str,
    y_label: &str,
    series: &[Series],
) -> Result<(), Box<dyn std::error::Error>> {
    let path = concat!(env!("CARGO_BIN_NAME"), ".svg");
    draw(path, title, "Time [s]", y_label, series, false)
}

/// Render the XY path geometry (equal aspect) as `<bin name>_path.svg`.
#[allow(dead_code)] // not every example draws the XY path
pub fn plot_xy(title: &str, series: &[Series]) -> Result<(), Box<dyn std::error::Error>> {
    let path = concat!(env!("CARGO_BIN_NAME"), "_path.svg");
    draw(path, title, "X [mm]", "Y [mm]", series, true)
}
