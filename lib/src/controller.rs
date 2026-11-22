//! High-level facade wiring decoder → preprocessors → interpolator →
//! kinematics → axis driver.

use crate::decoder::{Decoder, DecoderConfig};
use crate::driver::AxisDriver;
use crate::error::CncError;
use crate::interpolator::{Interpolator, IplOutput, IplState};
use crate::kinematics::Kinematics;
use crate::lookahead::LookAhead;
use crate::preprocess::Pipeline;
use crate::types::{AxisConfig, PathLimits};

/// The complete CNC pipeline behind one interface: load a program with
/// [`load_program`](Self::load_program), then call [`cycle`](Self::cycle)
/// every control period. Runtime control (override, halt/resume, single
/// step) goes through the public [`interpolator`](Self::interpolator).
pub struct CncController<const N: usize, K: Kinematics<N>> {
    /// Path-space → axis-space transform.
    pub kinematics: K,
    /// The interpolator; exposes runtime control (`set_override`, `halt`, ...).
    pub interpolator: Interpolator,
    /// Machine axis configuration.
    pub axis_config: AxisConfig<N>,
    time: f64,
}

impl<const N: usize, K: Kinematics<N>> CncController<N, K> {
    /// New controller for an N-axis machine running at `cycle_time` seconds
    /// per control cycle. Axes not carrying path geometry become auxiliary
    /// synchronized axes.
    pub fn new(
        axis_config: AxisConfig<N>,
        limits: PathLimits,
        lookahead: LookAhead,
        kinematics: K,
        cycle_time: f64,
    ) -> Self {
        // Auxiliary axes are the machine axes not carrying path geometry,
        // in ascending index order (matching CartesianKinematics::new).
        let path_axes: Vec<usize> = (0..3)
            .filter_map(|i| axis_config.path_axes.axis(i))
            .collect();
        let aux_max_velocity: Vec<f64> = (0..N)
            .filter(|i| !path_axes.contains(i))
            .map(|i| axis_config.max_velocity[i])
            .collect();
        Self {
            kinematics,
            interpolator: Interpolator::new(limits, lookahead, aux_max_velocity, cycle_time),
            axis_config,
            time: 0.0,
        }
    }

    /// Decode, preprocess and load a G-code program.
    pub fn load_program(
        &mut self,
        source: &str,
        decoder_cfg: DecoderConfig,
        pipeline: &mut Pipeline,
    ) -> Result<(), CncError> {
        let queue = Decoder::decode_source(decoder_cfg, source)?;
        let queue = pipeline.run(queue)?;
        self.time = 0.0;
        self.interpolator.load(queue)
    }

    /// One control cycle: interpolate, transform, write axis setpoints.
    ///
    /// Allocates a fresh [`IplOutput`] per call; for real-time use keep one
    /// buffer from [`Interpolator::output_buffer`] and call
    /// [`cycle_into`](Self::cycle_into) instead.
    pub fn cycle(&mut self, driver: &mut impl AxisDriver<N>) -> Result<IplOutput, CncError> {
        let mut out = self.interpolator.output_buffer();
        self.cycle_into(driver, &mut out)?;
        Ok(out)
    }

    /// One control cycle, writing the interpolator output into an existing
    /// buffer (see [`Interpolator::cycle_into`]): no heap allocation in
    /// steady state.
    pub fn cycle_into(
        &mut self,
        driver: &mut impl AxisDriver<N>,
        out: &mut IplOutput,
    ) -> Result<(), CncError> {
        self.interpolator.cycle_into(out)?;
        let mut pos = [0.0; N];
        let mut vel = [0.0; N];
        self.kinematics
            .inverse(&out.path_position, &out.aux, &mut pos)?;
        self.kinematics.inverse_velocity(
            &out.path_position,
            &out.path_tangent,
            out.path_velocity,
            &out.aux_velocity,
            &mut vel,
        )?;
        self.time += self.interpolator.cycle_time();
        driver.write_setpoints(self.time, &pos, &vel);
        Ok(())
    }

    /// Run until the program finishes (simulation convenience). Returns the
    /// number of cycles executed. `max_cycles` guards against runaway loops.
    pub fn run_to_end(
        &mut self,
        driver: &mut impl AxisDriver<N>,
        max_cycles: usize,
    ) -> Result<usize, CncError> {
        let mut out = self.interpolator.output_buffer();
        for n in 0..max_cycles {
            self.cycle_into(driver, &mut out)?;
            if out.state == IplState::Finished {
                return Ok(n + 1);
            }
        }
        Err(CncError::Interpolator(format!(
            "program did not finish within {max_cycles} cycles"
        )))
    }
}
