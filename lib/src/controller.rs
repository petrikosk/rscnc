//! High-level facade wiring decoder → preprocessors → interpolator →
//! kinematics → axis driver.

use crate::decoder::{Decoder, DecoderConfig};
use crate::driver::AxisDriver;
use crate::error::CncError;
use crate::geometry::{GeoElem, Segment};
use crate::interpolator::{Interpolator, IplOutput, IplState};
use crate::kinematics::Kinematics;
use crate::lookahead::LookAhead;
use crate::preprocess::Pipeline;
use crate::queue::GeoQueue;
use crate::types::{AxisConfig, PathLimits, PathPoint};

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
    /// The decoder of the loaded program, with its modal state at the end
    /// of the text decoded so far.
    decoder: Option<Decoder>,
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
        let axis_limits = axis_config.axis_limits();
        Self {
            kinematics,
            interpolator: Interpolator::with_axis_limits(
                limits,
                lookahead,
                axis_limits,
                cycle_time,
            ),
            axis_config,
            time: 0.0,
            decoder: None,
        }
    }

    /// Decode, preprocess and load a G-code program. A refused program
    /// leaves the loaded one in place.
    pub fn load_program(
        &mut self,
        source: &str,
        decoder_cfg: DecoderConfig,
        pipeline: &mut Pipeline,
    ) -> Result<(), CncError> {
        let mut decoder = Decoder::new(decoder_cfg);
        let queue = decoder.decode_chunk(source)?;
        decoder.finish()?;
        let queue = pipeline.run(queue)?;
        self.check_soft_limits(&queue)?;
        self.interpolator.load(queue)?;
        self.time = 0.0;
        self.decoder = Some(decoder);
        Ok(())
    }

    /// Decode and preprocess more program text and append it to the running
    /// program (streaming, MDI): see [`Interpolator::append`] for how the
    /// path continues, and [`Decoder::decode_chunk`] for what the decoder
    /// carries from one piece of text to the next (everything modal: motion
    /// mode, feed, position, offsets, variables) and what it does not (jump
    /// targets, a `G5` curve).
    ///
    /// The preprocessors of `pipeline` see each piece on its own: corners
    /// are not rounded or smoothed at the seam between two pieces, and a
    /// tool-radius-corrected contour that is cut by a seam is only accepted
    /// where its two parts happen to meet (each piece is offset as a contour
    /// of its own). End a piece where the contour does.
    ///
    /// The piece is checked against the software end positions like a
    /// loaded program. A refused piece changes nothing: the next one is
    /// decoded as if it had never been given. After `M2`/`M30` the decoder
    /// ignores further text until [`Decoder::resume`] (see
    /// [`decoder_mut`](Self::decoder_mut)).
    ///
    /// Without a loaded program this is an error; start with
    /// [`load_program`](Self::load_program) (an empty text will do).
    pub fn append_program(
        &mut self,
        source: &str,
        pipeline: &mut Pipeline,
    ) -> Result<(), CncError> {
        let Some(decoder) = &self.decoder else {
            return Err(CncError::Interpolator(
                "no program loaded to append to".into(),
            ));
        };
        let mut decoder = decoder.clone();
        let queue = decoder.decode_chunk(source)?;
        let queue = pipeline.run(queue)?;
        self.check_soft_limits(&queue)?;
        self.interpolator.append(queue)?;
        self.decoder = Some(decoder);
        Ok(())
    }

    /// The decoder of the loaded program, holding the modal state at the
    /// end of the text decoded so far (`None` before the first
    /// [`load_program`](Self::load_program)).
    pub fn decoder_mut(&mut self) -> Option<&mut Decoder> {
        self.decoder.as_mut()
    }

    /// Check a (preprocessed) queue against the software end positions of
    /// the axes ([`AxisConfig::soft_limit_min`] / `soft_limit_max`).
    ///
    /// Every move is transformed to axis space at its ends, at the points
    /// where an arc or a spline reaches furthest along a path coordinate,
    /// and at a number of points in between (a non-Cartesian kinematics can
    /// bulge anywhere). The first axis position outside its range is
    /// reported as [`CncError::SoftLimit`] with the move's source line.
    /// [`load_program`](Self::load_program) runs this check itself.
    pub fn check_soft_limits(&self, queue: &GeoQueue) -> Result<(), CncError> {
        let (lo, hi) = (
            &self.axis_config.soft_limit_min,
            &self.axis_config.soft_limit_max,
        );
        if lo.iter().all(|v| *v == f64::NEG_INFINITY) && hi.iter().all(|v| *v == f64::INFINITY) {
            return Ok(());
        }
        const SAMPLES: usize = 16;
        let mut aux = Vec::new();
        for elem in queue.iter() {
            let GeoElem::Move(mv) = elem else { continue };
            let len = mv.geo.length();
            let mut first_error = None;
            let mut check = |point: PathPoint, u: f64| {
                if first_error.is_some() {
                    return;
                }
                aux.clear();
                aux.extend(
                    mv.aux_start
                        .iter()
                        .zip(mv.aux_end.iter())
                        .map(|(a0, a1)| a0 + (a1 - a0) * u),
                );
                let mut pos = [0.0; N];
                if let Err(e) = self.kinematics.inverse(&point, &aux, &mut pos) {
                    first_error = Some(CncError::Kinematics(e));
                    return;
                }
                // (a tolerance of a few ulps: a move that ends exactly on
                // its limit is inside)
                let tol = |limit: f64| 1e-9 * (1.0 + limit.abs());
                for i in 0..N {
                    let (min, max) = (lo[i], hi[i]);
                    let limit = if pos[i] < min - tol(min) {
                        min
                    } else if pos[i] > max + tol(max) {
                        max
                    } else {
                        continue;
                    };
                    first_error = Some(CncError::SoftLimit {
                        line: mv.src_line,
                        message: format!(
                            "axis {} would move to {} (limit {})",
                            self.axis_config.axis_names[i], pos[i], limit
                        ),
                    });
                    return;
                }
            };
            for k in 0..=SAMPLES {
                let u = k as f64 / SAMPLES as f64;
                check(mv.geo.point_at(u * len), u);
            }
            // The extreme points of a curve; the auxiliary axes are checked
            // at their own extremes, the two ends, above.
            mv.geo.for_each_extreme(|p| check(p, 0.0));
            if let Some(e) = first_error {
                return Err(e);
            }
        }
        Ok(())
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
