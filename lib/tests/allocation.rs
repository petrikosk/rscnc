//! The real-time guarantee: a steady-state interpolation cycle with
//! auxiliary axes must not touch the heap.
//!
//! A counting global allocator is armed around the measured region only, and
//! counts on the measuring thread only (the counters are thread-local), so
//! the harness's own work cannot leak into the measurement. This file holds a
//! single test on purpose: it owns the process's global allocator.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use rscnc::prelude::*;

thread_local! {
    static ARMED: Cell<bool> = const { Cell::new(false) };
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}

struct CountingAllocator;

fn record() {
    let armed = ARMED.try_with(Cell::get).unwrap_or(false);
    if armed {
        let _ = ALLOCATIONS.try_with(|c| c.set(c.get() + 1));
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record();
        System.alloc(layout)
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        record();
        System.alloc_zeroed(layout)
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        record();
        System.realloc(ptr, layout, new_size)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout);
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

fn arm() {
    ALLOCATIONS.with(|c| c.set(0));
    ARMED.with(|a| a.set(true));
}

fn disarm() -> usize {
    ARMED.with(|a| a.set(false));
    ALLOCATIONS.with(Cell::get)
}

/// A zigzag with one auxiliary axis and two switch points per element: short
/// segments, so the measured window crosses many segment boundaries (where the
/// interpolator replans) and reports switch points along the way.
fn program() -> String {
    let mut src = String::from("G1 F10\n");
    let (mut x, mut a) = (0.0f64, 0.0f64);
    for i in 0..80 {
        x += 2.0;
        let y = if i % 2 == 0 { 2.0 } else { 0.0 };
        a += 1.0;
        src.push_str(&format!("X{x:.3} Y{y:.3} A{a:.3} H1 O0.25 H-1 O0.75\n"));
    }
    src
}

#[test]
fn steady_state_cycle_with_aux_axes_never_allocates() {
    let cfg = DecoderConfig {
        aux_letters: vec!['A'],
        start_aux: vec![0.0],
        ..Default::default()
    };
    let queue = Decoder::decode_source(cfg, &program()).expect("program decodes");
    let mut ipl = Interpolator::new(
        PathLimits::new(50.0, 500.0, 5000.0, 500.0),
        LookAhead::default(),
        vec![100.0],
        0.001,
    );
    ipl.load(queue).expect("program loads");

    // The buffer is sized for the loaded program: from here on, filling it
    // reuses its capacity.
    let mut out = ipl.output_buffer();
    assert_eq!(out.aux.len(), 1, "the buffer is seeded with the start pose");
    assert_eq!(out.path_position, ipl.start_position());

    // Warm-up: whatever the first cycles do once (lazy state anywhere below
    // us included) happens before the measurement.
    for _ in 0..50 {
        ipl.cycle_into(&mut out).expect("warm-up cycle");
    }

    // The counter itself must be able to see an allocation.
    arm();
    let canary = Vec::<u8>::with_capacity(64);
    let counted = disarm();
    drop(canary);
    assert!(
        counted > 0,
        "the counting allocator did not observe an allocation"
    );

    let mut boundaries = 0;
    let mut switches = 0;
    let mut last_obj = out.obj_no;
    let mut finished = false;

    arm();
    for _ in 0..2000 {
        ipl.cycle_into(&mut out).expect("cycle");
        switches += out.switch_points.len();
        if out.obj_no != last_obj {
            last_obj = out.obj_no;
            boundaries += 1;
        }
        if out.state == IplState::Finished {
            finished = true;
            break;
        }
    }
    let allocations = disarm();

    assert_eq!(
        allocations, 0,
        "steady-state cycles allocated {allocations} times ({boundaries} segment boundaries crossed)"
    );
    assert!(
        !finished,
        "the measured window must be steady state, not a program end"
    );
    assert!(
        boundaries >= 5,
        "expected the measured window to cross segment boundaries, saw {boundaries}"
    );
    assert!(out.aux[0] > 0.0, "the aux axis did not move");
    assert!(switches > 0, "no switch point was reported in the window");
}
