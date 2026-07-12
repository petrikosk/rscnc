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

pub struct CncController<const N: usize, K: Kinematics<N>> {
    pub kinematics: K,
    pub interpolator: Interpolator,
    pub axis_config: AxisConfig<N>,
    time: f64,
}

impl<const N: usize, K: Kinematics<N>> CncController<N, K> {
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
        let queue = Decoder::decode_program(decoder_cfg, source)?;
        let queue = pipeline.run(queue)?;
        self.time = 0.0;
        self.interpolator.load(queue)
    }

    /// One control cycle: interpolate, transform, write axis setpoints.
    pub fn cycle(&mut self, driver: &mut impl AxisDriver<N>) -> Result<IplOutput, CncError> {
        let out = self.interpolator.cycle()?;
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
        Ok(out)
    }

    /// Run until the program finishes (simulation convenience). Returns the
    /// number of cycles executed. `max_cycles` guards against runaway loops.
    pub fn run_to_end(
        &mut self,
        driver: &mut impl AxisDriver<N>,
        max_cycles: usize,
    ) -> Result<usize, CncError> {
        for n in 0..max_cycles {
            let out = self.cycle(driver)?;
            if out.state == IplState::Finished {
                return Ok(n + 1);
            }
        }
        Err(CncError::Interpolator(format!(
            "program did not finish within {max_cycles} cycles"
        )))
    }
}
