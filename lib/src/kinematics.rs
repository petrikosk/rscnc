//! Kinematic transform from path space to machine-axis space.

use crate::error::KinError;
use crate::types::{PathAxes, PathPoint};

/// Transform a path-space pose (path point + auxiliary axis values) into
/// machine-axis positions. Implement this for non-Cartesian machines
/// (gantry, delta, ...); v1 ships the identity Cartesian mapping.
pub trait Kinematics<const N: usize> {
    /// Path pose → axis positions.
    fn inverse(&self, path: &PathPoint, aux: &[f64], out: &mut [f64; N]) -> Result<(), KinError>;

    /// Path velocity (unit tangent · path speed) and auxiliary velocities →
    /// axis velocities. Default: same mapping as positions applied to the
    /// velocity vector, valid for any linear kinematics.
    fn inverse_velocity(
        &self,
        path: &PathPoint,
        tangent: &PathPoint,
        v_path: f64,
        aux_vel: &[f64],
        out: &mut [f64; N],
    ) -> Result<(), KinError> {
        let vel_point = [
            tangent[0] * v_path,
            tangent[1] * v_path,
            tangent[2] * v_path,
        ];
        let _ = path;
        self.inverse(&vel_point, aux_vel, out)
    }
}

/// Identity mapping: path coordinates go straight to the configured path
/// axes, auxiliary values fill the remaining axes in order.
#[derive(Debug, Clone)]
pub struct CartesianKinematics<const N: usize> {
    /// Which machine axes carry the path geometry.
    pub path_axes: PathAxes,
    /// Machine-axis indices of the auxiliary axes, in the order the decoder
    /// produces them.
    pub aux_axes: Vec<usize>,
}

impl<const N: usize> CartesianKinematics<N> {
    /// Path axes first (2 or 3), all remaining axes auxiliary, in order.
    pub fn new(path_axes: PathAxes) -> Self {
        let path_count = path_axes.count();
        let aux_axes = (path_count..N).collect();
        Self {
            path_axes,
            aux_axes,
        }
    }
}

impl<const N: usize> Kinematics<N> for CartesianKinematics<N> {
    fn inverse(&self, path: &PathPoint, aux: &[f64], out: &mut [f64; N]) -> Result<(), KinError> {
        for (i, p) in path.iter().enumerate().take(3) {
            if let Some(axis) = self.path_axes.axis(i) {
                if axis >= N {
                    return Err(KinError(format!("path axis index {axis} out of range")));
                }
                out[axis] = *p;
            }
        }
        for (i, &axis) in self.aux_axes.iter().enumerate() {
            if axis >= N {
                return Err(KinError(format!("aux axis index {axis} out of range")));
            }
            out[axis] = aux.get(i).copied().unwrap_or(0.0);
        }
        Ok(())
    }
}
