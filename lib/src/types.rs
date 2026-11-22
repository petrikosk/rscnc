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
/// mm/s, mm/s², mm/s³; G-code feeds are units/s as well).
///
/// [`CncController`](crate::CncController) applies these limits on top of
/// the [`PathLimits`]: the limits of the machine axes named by
/// [`path_axes`](Self::path_axes) bound the path coordinates they carry, and
/// the remaining (auxiliary) axes bound the path velocity, acceleration and
/// jerk of every move they take part in. See [`AxisLimits`].
#[derive(Debug, Clone)]
pub struct AxisConfig<const N: usize> {
    /// Which machine axes carry the path geometry.
    pub path_axes: PathAxes,
    /// Display names, e.g. `["X", "Y", "Z", "A"]`.
    pub axis_names: [&'static str; N],
    /// Per-axis velocity limits.
    pub max_velocity: [f64; N],
    /// Per-axis acceleration limits.
    pub max_acceleration: [f64; N],
    /// Per-axis jerk limits.
    pub max_jerk: [f64; N],
    /// Per-axis limit on the velocity step an axis may take at a junction
    /// between two path elements (infinite by default: junctions are then
    /// ruled by the look-ahead's junction deviation alone).
    pub max_velocity_jump: [f64; N],
    /// Lower software end positions of the axes (−∞: unlimited). A program
    /// that would leave the range is refused when it is loaded.
    pub soft_limit_min: [f64; N],
    /// Upper software end positions of the axes (+∞: unlimited).
    pub soft_limit_max: [f64; N],
}

impl<const N: usize> AxisConfig<N> {
    /// A Cartesian machine with the first 2 or 3 axes as path axes, without
    /// velocity-jump limits and software end positions.
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
            max_velocity_jump: [f64::INFINITY; N],
            soft_limit_min: [f64::NEG_INFINITY; N],
            soft_limit_max: [f64::INFINITY; N],
        }
    }

    /// Set the per-axis velocity-jump limits (builder style).
    pub fn with_velocity_jump(mut self, max_velocity_jump: [f64; N]) -> Self {
        self.max_velocity_jump = max_velocity_jump;
        self
    }

    /// Set the software end positions of the axes (builder style).
    pub fn with_soft_limits(mut self, min: [f64; N], max: [f64; N]) -> Self {
        self.soft_limit_min = min;
        self.soft_limit_max = max;
        self
    }

    /// The limits of machine axis `i` in the form the look-ahead takes them.
    pub fn axis_limit(&self, i: usize) -> AxisLimit {
        AxisLimit {
            max_velocity: self.max_velocity[i],
            max_acceleration: self.max_acceleration[i],
            max_jerk: self.max_jerk[i],
            max_velocity_jump: self.max_velocity_jump[i],
        }
    }

    /// Machine-axis indices of the auxiliary axes (those not carrying path
    /// geometry), in ascending order: the order of the decoder's auxiliary
    /// axis letters.
    pub fn aux_axes(&self) -> impl Iterator<Item = usize> + '_ {
        (0..N).filter(|i| (0..3).all(|k| self.path_axes.axis(k) != Some(*i)))
    }

    /// The per-axis limits in the form the look-ahead takes them: the path
    /// coordinates in path order, then the auxiliary axes in decoder order.
    pub fn axis_limits(&self) -> AxisLimits {
        let mut path = [AxisLimit::UNLIMITED; 3];
        for (k, limit) in path.iter_mut().enumerate() {
            if let Some(axis) = self.path_axes.axis(k).filter(|a| *a < N) {
                *limit = self.axis_limit(axis);
            }
        }
        AxisLimits {
            path,
            aux: self.aux_axes().map(|i| self.axis_limit(i)).collect(),
        }
    }
}

/// Limits of one axis, as the look-ahead applies them to the path.
///
/// Every field defaults to infinity (no limit).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisLimit {
    /// Highest velocity of the axis.
    pub max_velocity: f64,
    /// Highest acceleration of the axis.
    pub max_acceleration: f64,
    /// Highest jerk of the axis.
    pub max_jerk: f64,
    /// Largest velocity step the axis may take at a junction between two
    /// path elements (where the path direction, or an auxiliary axis' rate,
    /// changes abruptly).
    pub max_velocity_jump: f64,
}

impl AxisLimit {
    /// No limits at all.
    pub const UNLIMITED: AxisLimit = AxisLimit {
        max_velocity: f64::INFINITY,
        max_acceleration: f64::INFINITY,
        max_jerk: f64::INFINITY,
        max_velocity_jump: f64::INFINITY,
    };

    /// Limits on velocity, acceleration and jerk (no velocity-jump limit).
    pub fn new(max_velocity: f64, max_acceleration: f64, max_jerk: f64) -> Self {
        Self {
            max_velocity,
            max_acceleration,
            max_jerk,
            max_velocity_jump: f64::INFINITY,
        }
    }

    /// A velocity limit only.
    pub fn velocity(max_velocity: f64) -> Self {
        Self {
            max_velocity,
            ..Self::UNLIMITED
        }
    }

    /// Add a velocity-jump limit (builder style).
    pub fn with_velocity_jump(mut self, max_velocity_jump: f64) -> Self {
        self.max_velocity_jump = max_velocity_jump;
        self
    }

    /// `true` when the axis constrains the continuous motion of a move
    /// (velocity, acceleration or jerk; the jump limit acts at junctions).
    pub(crate) fn limits_motion(&self) -> bool {
        self.max_velocity < f64::INFINITY
            || self.max_acceleration < f64::INFINITY
            || self.max_jerk < f64::INFINITY
    }
}

impl Default for AxisLimit {
    fn default() -> Self {
        Self::UNLIMITED
    }
}

/// Per-axis limits the look-ahead keeps on top of the [`PathLimits`].
///
/// On a move with unit tangent `t`, path axis `i` sees the share `|tᵢ|` of
/// the path velocity, tangential acceleration and jerk, so the move's path
/// limits become `min(path limit, axis limitᵢ / |tᵢ|)`. An auxiliary axis
/// that travels `Δ` over a move of length `L` sees the share `|Δ| / L` and
/// limits the move the same way. On arcs and splines the largest share along
/// the element counts, and the centripetal acceleration `v²·κ` is kept within
/// the acceleration limits of the axes it points along — as a budget of its
/// own next to the tangential one, just like
/// [`PathLimits::max_normal_acceleration`].
///
/// At a junction the velocity of an axis changes by `v·|Δtᵢ|` (path axes) or
/// `v·|Δrᵢ|` (auxiliary axes, `r = dA/ds`); the junction velocity is kept low
/// enough for every axis' [`max_velocity_jump`](AxisLimit::max_velocity_jump).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AxisLimits {
    /// Limits of the three path coordinates.
    pub path: [AxisLimit; 3],
    /// Limits of the auxiliary axes, in decoder order.
    pub aux: Vec<AxisLimit>,
}

impl AxisLimits {
    /// Only velocity limits on the auxiliary axes (the limits rscnc 0.2
    /// knew).
    pub fn from_aux_velocity(aux_max_velocity: &[f64]) -> Self {
        Self {
            path: [AxisLimit::UNLIMITED; 3],
            aux: aux_max_velocity
                .iter()
                .map(|v| AxisLimit::velocity(*v))
                .collect(),
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
