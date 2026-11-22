//! Nothing reachable through the public API may panic: degenerate machine
//! configurations and degenerate runtime inputs come back as `Err`.
//!
//! This matters beyond taste on `wasm32-unknown-unknown`, where a panic is an
//! unrecoverable trap with no unwinding: a returned error can be handled by
//! the caller, a trap kills the module.

use rscnc::prelude::*;

const DT: f64 = 0.001;

fn program() -> GeoQueue {
    Decoder::decode_source(DecoderConfig::default(), "G1 F10 X10\nY10\nG2 X20 Y0 R10\n")
        .expect("program decodes")
}

fn interpolator(limits: PathLimits, cycle_time: f64) -> Interpolator {
    Interpolator::new(limits, LookAhead::default(), vec![], cycle_time)
}

fn assert_refused(limits: PathLimits, cycle_time: f64, what: &str) {
    let mut ipl = interpolator(limits, cycle_time);
    let Err(err) = ipl.load(program()) else {
        panic!("{what} must be refused, not accepted");
    };
    // Whatever the stage, the failure is an rscnc error and not a panic.
    assert!(
        matches!(err, CncError::Interpolator(_) | CncError::Otg { .. }),
        "{what}: unexpected error {err:?}"
    );
    // And the interpolator stays usable: cycling a refused program does not
    // panic either, it just goes nowhere.
    for _ in 0..10 {
        let _ = ipl.cycle();
    }
}

#[test]
fn degenerate_acceleration_and_jerk_are_refused() {
    assert_refused(
        PathLimits::new(50.0, 0.0, 5000.0, 500.0),
        DT,
        "zero acceleration",
    );
    assert_refused(
        PathLimits::new(50.0, -1.0, 5000.0, 500.0),
        DT,
        "negative acceleration",
    );
    assert_refused(PathLimits::new(50.0, 500.0, 0.0, 500.0), DT, "zero jerk");
    assert_refused(
        PathLimits::new(50.0, 500.0, -1.0, 500.0),
        DT,
        "negative jerk",
    );
    assert_refused(
        PathLimits::new(50.0, f64::NAN, 5000.0, 500.0),
        DT,
        "NaN acceleration",
    );
    assert_refused(
        PathLimits::new(50.0, 500.0, f64::INFINITY, 500.0),
        DT,
        "infinite jerk",
    );
}

#[test]
fn degenerate_velocity_limits_are_refused() {
    assert_refused(
        PathLimits::new(0.0, 500.0, 5000.0, 500.0),
        DT,
        "zero max velocity",
    );
    assert_refused(
        PathLimits::new(-5.0, 500.0, 5000.0, 500.0),
        DT,
        "negative max velocity",
    );
    assert_refused(
        PathLimits::new(f64::NAN, 500.0, 5000.0, 500.0),
        DT,
        "NaN max velocity",
    );
    assert_refused(
        PathLimits::new(50.0, 500.0, 5000.0, f64::NAN),
        DT,
        "NaN normal acceleration",
    );
    // Auxiliary axis limits are checked as well.
    let mut ipl = Interpolator::new(
        PathLimits::new(50.0, 500.0, 5000.0, 500.0),
        LookAhead::default(),
        vec![f64::NAN],
        DT,
    );
    assert!(matches!(
        ipl.load(program()),
        Err(CncError::Interpolator(_))
    ));
}

#[test]
fn degenerate_cycle_time_is_refused() {
    let limits = PathLimits::new(50.0, 500.0, 5000.0, 500.0);
    // A zero or negative cycle time used to be accepted and then stalled
    // forever at s = 0 (the profile can never advance).
    assert_refused(limits, 0.0, "zero cycle time");
    assert_refused(limits, -0.001, "negative cycle time");
    assert_refused(limits, f64::NAN, "NaN cycle time");
    assert_refused(limits, f64::INFINITY, "infinite cycle time");
}

#[test]
fn degenerate_lookahead_is_refused() {
    let mut ipl = Interpolator::new(
        PathLimits::new(50.0, 500.0, 5000.0, 500.0),
        LookAhead {
            junction_deviation: f64::NAN,
            accel_margin: 0.8,
            ..LookAhead::default()
        },
        vec![],
        DT,
    );
    assert!(matches!(
        ipl.load(program()),
        Err(CncError::Interpolator(_))
    ));

    let mut ipl = Interpolator::new(
        PathLimits::new(50.0, 500.0, 5000.0, 500.0),
        LookAhead {
            junction_deviation: 0.05,
            accel_margin: 0.0,
            ..LookAhead::default()
        },
        vec![],
        DT,
    );
    assert!(matches!(
        ipl.load(program()),
        Err(CncError::Interpolator(_))
    ));
}

#[test]
fn degenerate_override_pauses_instead_of_poisoning_the_profile() {
    let mut ipl = interpolator(PathLimits::new(50.0, 500.0, 5000.0, 500.0), DT);
    ipl.load(program()).expect("program loads");
    for _ in 0..50 {
        ipl.cycle().expect("cycle");
    }
    // A NaN override must not reach the trajectory generator.
    ipl.set_override(f64::NAN);
    assert_eq!(ipl.override_factor(), 0.0);
    for _ in 0..2000 {
        let out = ipl.cycle().expect("cycle with a NaN override");
        assert!(out.path_velocity.is_finite());
        assert!(out.path_position.iter().all(|v| v.is_finite()));
    }
    assert_eq!(ipl.state(), IplState::Paused);
    // ...and raising it again resumes normally.
    ipl.set_override(1.0);
    for _ in 0..200_000 {
        let out = ipl.cycle().expect("cycle after resume");
        if out.state == IplState::Finished {
            return;
        }
    }
    panic!("program did not finish after resuming");
}

#[test]
fn an_empty_program_is_not_a_panic() {
    let mut ipl = interpolator(PathLimits::new(50.0, 500.0, 5000.0, 500.0), DT);
    ipl.load(GeoQueue::new()).expect("an empty queue loads");
    assert_eq!(ipl.state(), IplState::Finished);
    assert_eq!(ipl.total_length(), 0.0);
    assert_eq!(ipl.start_position(), [0.0; 3]);
    assert!(ipl.start_aux().is_empty());
    assert!(!ipl.current_is_rapid());
    for _ in 0..10 {
        let out = ipl.cycle().expect("cycle on an empty program");
        assert_eq!(out.state, IplState::Finished);
    }
}
