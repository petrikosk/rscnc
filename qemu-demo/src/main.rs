//! Bare-metal smoke test for the `no_std` build of rscnc.
//!
//! Runs the 20×20 mm square program (same as `example_square`) through the
//! full pipeline — decoder → look-ahead → interpolator → kinematics — on an
//! emulated Cortex-M3 without FPU (soft-float f64), and reports the final
//! position and cycle count over semihosting.
//!
//! Run: `cargo run --release` (requires `qemu-system-arm` on PATH).

#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use embedded_alloc::LlffHeap as Heap;
use panic_semihosting as _;
use rscnc::prelude::*;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 512 * 1024;

const DT: f64 = 0.001;

/// Minimal driver: keeps only the latest setpoints (no per-cycle allocation),
/// the way a real fieldbus cycle would consume them.
struct LastSetpoint<const N: usize> {
    time: f64,
    position: [f64; N],
    velocity: [f64; N],
}

impl<const N: usize> AxisDriver<N> for LastSetpoint<N> {
    fn write_setpoints(&mut self, time: f64, position: &[f64; N], velocity: &[f64; N]) {
        self.time = time;
        self.position = *position;
        self.velocity = *velocity;
    }
}

#[entry]
fn main() -> ! {
    {
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    let program = "\
N10 G21 G90
N20 G1 F1500 X20
N30 Y20
N40 X0
N50 Y0
N60 M30
";

    hprintln!("rscnc no_std smoke test on Cortex-M3 (soft-float f64)");

    let axis_config =
        AxisConfig::<3>::cartesian(["X", "Y", "Z"], [60.0; 3], [800.0; 3], [10000.0; 3]);
    let limits = PathLimits::new(60.0, 800.0, 10000.0, 800.0);
    let mut cnc = CncController::new(
        axis_config,
        limits,
        LookAhead::default(),
        CartesianKinematics::new(PathAxes::Three([0, 1, 2])),
        DT,
    );
    let mut pipeline = Pipeline::new();
    if let Err(e) = cnc.load_program(program, DecoderConfig::default(), &mut pipeline) {
        hprintln!("FAIL: load_program: {}", e);
        debug::exit(debug::EXIT_FAILURE);
    }
    hprintln!("program loaded, interpolating...");

    let mut driver = LastSetpoint::<3> {
        time: 0.0,
        position: [0.0; 3],
        velocity: [0.0; 3],
    };
    let mut cycles: u32 = 0;
    let mut v_peak: f64 = 0.0;
    loop {
        match cnc.cycle(&mut driver) {
            Ok(out) => {
                cycles += 1;
                if out.path_velocity > v_peak {
                    v_peak = out.path_velocity;
                }
                if out.state == IplState::Finished {
                    break;
                }
            }
            Err(e) => {
                hprintln!("FAIL: cycle {}: {}", cycles, e);
                debug::exit(debug::EXIT_FAILURE);
            }
        }
    }

    hprintln!(
        "finished: {} cycles ({} ms simulated), peak path velocity {} mm/s",
        cycles,
        cycles,
        v_peak
    );
    hprintln!(
        "final position: X={} Y={} Z={} (expected 0,0,0)",
        driver.position[0],
        driver.position[1],
        driver.position[2]
    );

    let err = driver.position[0].abs() + driver.position[1].abs() + driver.position[2].abs();
    if err < 1e-6 {
        hprintln!("PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("FAIL: endpoint error {}", err);
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {}
}
