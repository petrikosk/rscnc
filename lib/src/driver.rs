//! Axis setpoint sink: the boundary behind which real hardware (a fieldbus
//! cycle) will live. v1 ships a simulation driver that logs everything for
//! plotting and test assertions.

/// Per-cycle axis setpoint consumer.
pub trait AxisDriver<const N: usize> {
    fn write_setpoints(&mut self, time: f64, position: &[f64; N], velocity: &[f64; N]);
}

/// Sampled trajectory data collected by [`SimDriver`].
#[derive(Debug, Clone, Default)]
pub struct TrajectoryLog<const N: usize> {
    pub time: Vec<f64>,
    pub position: Vec<[f64; N]>,
    pub velocity: Vec<[f64; N]>,
}

impl<const N: usize> TrajectoryLog<N> {
    pub fn len(&self) -> usize {
        self.time.len()
    }

    pub fn is_empty(&self) -> bool {
        self.time.is_empty()
    }

    /// Time series of one axis' positions (for plotting).
    pub fn axis_positions(&self, axis: usize) -> Vec<f64> {
        self.position.iter().map(|p| p[axis]).collect()
    }

    pub fn axis_velocities(&self, axis: usize) -> Vec<f64> {
        self.velocity.iter().map(|v| v[axis]).collect()
    }
}

/// Simulation driver: records all setpoints.
#[derive(Debug, Clone, Default)]
pub struct SimDriver<const N: usize> {
    pub log: TrajectoryLog<N>,
}

impl<const N: usize> SimDriver<N> {
    pub fn new() -> Self {
        Self {
            log: TrajectoryLog {
                time: Vec::new(),
                position: Vec::new(),
                velocity: Vec::new(),
            },
        }
    }
}

impl<const N: usize> AxisDriver<N> for SimDriver<N> {
    fn write_setpoints(&mut self, time: f64, position: &[f64; N], velocity: &[f64; N]) {
        self.log.time.push(time);
        self.log.position.push(*position);
        self.log.velocity.push(*velocity);
    }
}
