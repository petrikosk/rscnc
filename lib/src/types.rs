//! Shared core types: axis model, work planes, path limits.

/// A point in path space. The path geometry always lives in 3 coordinates;
/// 2D programs simply keep the third at 0. Which *machine axes* these map to
/// is decided by [`PathAxes`] / the kinematics.
pub type PathPoint = [f64; 3];

/// Work plane for arcs and tool radius correction (DIN 66025 G17/G18/G19).
///
/// Stores the path-coordinate indices of the two in-plane axes and the
/// plane normal axis, in the DIN order that keeps arcs right-handed:
/// G17 = (X,Y,Z), G18 = (Z,X,Y), G19 = (Y,Z,X).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WorkPlane {
    /// G17 - XY plane, normal Z
    #[default]
    Xy,
    /// G18 - ZX plane, normal Y
    Zx,
    /// G19 - YZ plane, normal X
    Yz,
}

impl WorkPlane {
    /// (first in-plane axis, second in-plane axis, normal axis) as indices into a [`PathPoint`].
    #[inline]
    pub fn axes(self) -> (usize, usize, usize) {
        match self {
            WorkPlane::Xy => (0, 1, 2),
            WorkPlane::Zx => (2, 0, 1),
            WorkPlane::Yz => (1, 2, 0),
        }
    }
}

/// Which machine axes carry the path geometry. The remaining axes of an
/// N-axis machine are auxiliary synchronized axes (interpolated linearly
/// over the path parameter).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PathAxes {
    /// Two geometric axes (e.g. an XY table); the third path coordinate is unused.
    Two([usize; 2]),
    /// Three geometric axes (e.g. XYZ).
    Three([usize; 3]),
}

impl PathAxes {
    /// Machine-axis index for path coordinate `i` (0..3), if it exists.
    #[inline]
    pub fn axis(&self, i: usize) -> Option<usize> {
        match self {
            PathAxes::Two(a) => a.get(i).copied(),
            PathAxes::Three(a) => a.get(i).copied(),
        }
    }

    /// Number of path axes (2 or 3).
    pub fn count(&self) -> usize {
        match self {
            PathAxes::Two(_) => 2,
            PathAxes::Three(_) => 3,
        }
    }
}

/// Per-machine axis configuration for an N-axis machine.
///
/// Units are up to the caller but must be consistent (the examples use mm,
/// mm/s, mm/s², mm/s³ internally; feeds in G-code are units/min and are
/// converted by the decoder).
#[derive(Debug, Clone)]
pub struct AxisConfig<const N: usize> {
    /// Which machine axes carry the path geometry.
    pub path_axes: PathAxes,
    /// Display names, e.g. `["X", "Y", "Z", "A"]`.
    pub axis_names: [&'static str; N],
    /// Per-axis velocity limits (used for auxiliary-axis feed caps).
    pub max_velocity: [f64; N],
    /// Per-axis acceleration limits.
    pub max_acceleration: [f64; N],
    /// Per-axis jerk limits.
    pub max_jerk: [f64; N],
}

impl<const N: usize> AxisConfig<N> {
    /// A Cartesian machine with the first 2 or 3 axes as path axes.
    pub fn cartesian(
        axis_names: [&'static str; N],
        max_velocity: [f64; N],
        max_acceleration: [f64; N],
        max_jerk: [f64; N],
    ) -> Self {
        let path_axes = if N >= 3 {
            PathAxes::Three([0, 1, 2])
        } else {
            PathAxes::Two([0, 1])
        };
        Self {
            path_axes,
            axis_names,
            max_velocity,
            max_acceleration,
            max_jerk,
        }
    }
}

/// Tangential limits along the path parameter `s`, plus the normal
/// (centripetal) acceleration budget that caps velocity on arcs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathLimits {
    /// Machine cap on path velocity; the programmed feed further limits each segment.
    pub max_velocity: f64,
    /// Max tangential acceleration along the path.
    pub max_acceleration: f64,
    /// Max tangential jerk along the path.
    pub max_jerk: f64,
    /// Max normal acceleration: on an arc of radius r, v ≤ sqrt(max_normal_acceleration · r).
    pub max_normal_acceleration: f64,
}

impl PathLimits {
    /// Path limits from the four maxima (velocity, tangential acceleration,
    /// tangential jerk, normal acceleration).
    pub fn new(
        max_velocity: f64,
        max_acceleration: f64,
        max_jerk: f64,
        max_normal_acceleration: f64,
    ) -> Self {
        Self {
            max_velocity,
            max_acceleration,
            max_jerk,
            max_normal_acceleration,
        }
    }
}
