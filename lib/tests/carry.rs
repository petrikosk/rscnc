//! Acceleration carried across span boundaries
//! (`LookAhead::carry_acceleration`).

use rscnc::prelude::*;

const DT: f64 = 0.001;
const V_MAX: f64 = 50.0;
const A_MAX: f64 = 500.0;
const J_MAX: f64 = 5000.0;

fn limits() -> PathLimits {
    PathLimits::new(V_MAX, A_MAX, J_MAX, 500.0)
}

fn lookahead(carry: bool) -> LookAhead {
    LookAhead {
        carry_acceleration: carry,
        ..LookAhead::default()
    }
}

fn decode(src: &str) -> GeoQueue {
    Decoder::decode_source(DecoderConfig::default(), src).expect("program decodes")
}

fn interpolator(src: &str, carry: bool) -> Interpolator {
    let mut ipl = Interpolator::new(limits(), lookahead(carry), Vec::new(), DT);
    ipl.load(decode(src)).expect("program loads");
    ipl
}

fn run(ipl: &mut Interpolator) -> Vec<IplOutput> {
    let mut outputs = Vec::new();
    for _ in 0..200_000 {
        let out = ipl.cycle().expect("cycle");
        let done = out.state == IplState::Finished;
        outputs.push(out);
        if done {
            return outputs;
        }
    }
    panic!("the program does not finish");
}

/// The dynamic limits hold from one cycle to the next, the path never runs
/// backward and its position follows its velocity.
fn check_motion(outputs: &[IplOutput]) {
    for (i, w) in outputs.windows(2).enumerate() {
        let (a, b) = (&w[0], &w[1]);
        assert!(b.path_velocity >= 0.0, "cycle {i}: negative velocity");
        assert!(
            b.path_velocity <= V_MAX * (1.0 + 1e-9),
            "cycle {i}: velocity {}",
            b.path_velocity
        );
        assert!(
            b.path_acceleration.abs() <= A_MAX * (1.0 + 1e-6),
            "cycle {i}: acceleration {}",
            b.path_acceleration
        );
        let jerk = (b.path_acceleration - a.path_acceleration).abs() / DT;
        assert!(jerk <= J_MAX * (1.0 + 1e-6), "cycle {i}: jerk {jerk}");
        let step = b.s_total - a.s_total;
        let mean = 0.5 * (a.path_velocity + b.path_velocity) * DT;
        assert!(
            (step - mean).abs() <= J_MAX * DT * DT * DT,
            "cycle {i}: step {step} at a mean velocity of {}",
            mean / DT
        );
    }
}

/// Number of short blocks in a feed staircase, and their length.
const STEPS: usize = 40;
const BLOCK: f64 = 0.2;

/// Feed (mm/s) of block `k` of a staircase between 50 and 5 mm/s whose
/// steps follow a steady acceleration: the square of the feed changes by
/// the same amount from block to block.
fn stair_feed(k: usize, rising: bool) -> f64 {
    let k = if rising { k } else { STEPS - 1 - k };
    (25.0 + (2500.0 - 25.0) * k as f64 / (STEPS - 1) as f64).sqrt()
}

/// A straight run from `x0` whose feed changes from one short block to the
/// next, so that every block is a span of its own.
fn staircase(x0: f64, rising: bool) -> String {
    let mut src = String::new();
    for k in 0..STEPS {
        let x = x0 + BLOCK * (k + 1) as f64;
        src += &format!("X{x:.3} F{:.3}\n", stair_feed(k, rising));
    }
    src
}

/// Full speed, then down the staircase.
fn falling() -> String {
    format!("G1 X40 F50\n{}", staircase(40.0, false))
}

/// Up the staircase, then on at full speed.
fn rising() -> String {
    format!("G1\n{}X60 F50\n", staircase(0.0, true))
}

/// Number of times the path acceleration returns to zero while the path is
/// moving, i.e. of separate ramps.
fn ramps(outputs: &[IplOutput]) -> usize {
    outputs
        .windows(2)
        .filter(|w| {
            w[0].path_acceleration.abs() > 1.0
                && w[1].path_acceleration.abs() <= 1.0
                && w[1].path_velocity > 1.0
        })
        .count()
}

#[test]
fn a_falling_feed_staircase_is_braked_in_one_ramp() {
    let src = falling();
    let plain = run(&mut interpolator(&src, false));
    let carried = run(&mut interpolator(&src, true));
    check_motion(&plain);
    check_motion(&carried);
    assert!(
        ramps(&carried) * 3 < ramps(&plain),
        "{} ramps carried, {} plain",
        ramps(&carried),
        ramps(&plain)
    );
    assert!(
        carried.len() < plain.len(),
        "{} cycles carried, {} plain",
        carried.len(),
        plain.len()
    );
    // Both end in the same place.
    let (a, b) = (plain.last().unwrap(), carried.last().unwrap());
    assert!((a.path_position[0] - b.path_position[0]).abs() < 1e-9);
    assert!((a.s_total - b.s_total).abs() < 1e-9);
}

#[test]
fn a_rising_feed_staircase_is_accelerated_in_one_ramp() {
    let src = rising();
    let plain = run(&mut interpolator(&src, false));
    let carried = run(&mut interpolator(&src, true));
    check_motion(&plain);
    check_motion(&carried);
    assert!(
        ramps(&carried) * 3 < ramps(&plain),
        "{} ramps carried, {} plain",
        ramps(&carried),
        ramps(&plain)
    );
    assert!(
        carried.len() < plain.len(),
        "{} cycles carried, {} plain",
        carried.len(),
        plain.len()
    );
}

#[test]
fn the_feed_of_every_block_is_kept_on_the_carried_plan() {
    for (src, x0, up) in [(falling(), 40.0, false), (rising(), 0.0, true)] {
        let outputs = run(&mut interpolator(&src, true));
        for w in outputs.windows(2) {
            // A cycle that lies within one block of the staircase
            let block = |x: f64| ((x - x0) / BLOCK).floor();
            let (xa, xb) = (w[0].path_position[0], w[1].path_position[0]);
            if xa > x0 && block(xa) == block(xb) && (block(xb) as usize) < STEPS {
                let feed = stair_feed(block(xb) as usize, up);
                let v = w[1].path_velocity.min(w[0].path_velocity);
                assert!(v <= feed * (1.0 + 1e-6), "x {xb}: {v} at a feed of {feed}");
            }
        }
    }
}

#[test]
fn the_plain_plan_is_what_is_planned_without_carrying() {
    let src = falling();
    let plain = interpolator(&src, false);
    let carried = interpolator(&src, true);
    let mut differ = 0;
    for (p, c) in plain
        .planned_segments()
        .iter()
        .zip(carried.planned_segments())
    {
        assert_eq!((p.v_carry, p.a_carry), (p.v_exit, 0.0));
        // The plain plan is the same either way,
        assert_eq!(p.v_exit, c.v_exit);
        assert_eq!(p.v_cap, c.v_cap);
        // and a carried state is never slower than it, nor outside the
        // limits.
        assert!(c.v_carry >= c.v_exit);
        assert!(c.v_carry <= c.v_junction.max(c.v_exit));
        assert!(c.a_carry <= c.accel && c.a_carry >= -c.decel);
        if (c.v_carry, c.a_carry) != (c.v_exit, 0.0) {
            differ += 1;
        }
    }
    assert!(differ > 10, "only {differ} carried states");
    // (the path stops at the end of the program)
    let last = carried.planned_segments().last().unwrap();
    assert_eq!((last.v_carry, last.a_carry), (0.0, 0.0));
}

#[test]
fn halting_on_a_carried_ramp_stops_short_of_the_end() {
    let src = falling();
    let total = interpolator(&src, true).total_length();
    // Halt at every tenth cycle of the braking ramp in turn.
    let reference = run(&mut interpolator(&src, true));
    let braking: Vec<usize> = reference
        .iter()
        .enumerate()
        .filter(|(_, o)| o.path_acceleration < -1.0)
        .map(|(i, _)| i)
        .collect();
    assert!(braking.len() > 100);
    for &at in braking.iter().step_by(10) {
        let mut ipl = interpolator(&src, true);
        let mut outputs = Vec::new();
        for i in 0..200_000 {
            if i == at {
                ipl.halt();
            }
            let out = ipl.cycle().expect("cycle");
            let state = out.state;
            outputs.push(out);
            if state == IplState::Paused {
                assert!(i >= at);
                break;
            }
            assert_ne!(state, IplState::Finished, "halt at {at}");
        }
        let stopped = outputs.last().unwrap();
        assert_eq!(stopped.path_velocity, 0.0);
        assert!(stopped.s_total <= total + 1e-9, "halt at {at}: overrun");
        // ... and resumes to the end.
        ipl.resume();
        outputs.extend(run(&mut ipl));
        check_motion(&outputs);
        assert!((outputs.last().unwrap().s_total - total).abs() < 1e-9);
    }
}

#[test]
fn override_changes_on_a_carried_ramp_keep_the_limits() {
    let src = falling() + "X80 F50\n";
    let total = interpolator(&src, true).total_length();
    for (k, factor) in [0.3, 0.7, 1.2, 0.05].into_iter().enumerate() {
        for at in (700..1300).step_by(37) {
            let mut ipl = interpolator(&src, true);
            let mut outputs = Vec::new();
            for i in 0..200_000 {
                if i == at {
                    ipl.set_override(factor);
                }
                if i == at + 150 + 20 * k {
                    ipl.set_override(1.0);
                }
                let out = ipl.cycle().expect("cycle");
                let done = out.state == IplState::Finished;
                outputs.push(out);
                if done {
                    break;
                }
            }
            check_motion(&outputs);
            assert!((outputs.last().unwrap().s_total - total).abs() < 1e-9);
        }
    }
}

#[test]
fn a_smoothed_contour_runs_faster_with_carried_acceleration() {
    // A polygon with smoothed corners: the blends are spans of their own,
    // each with its own velocity cap.
    let mut src = String::from("G1 F50\n");
    let n = 24;
    for i in 1..=n {
        let phi = std::f64::consts::TAU * i as f64 / n as f64;
        src += &format!("X{:.4} Y{:.4}\n", 30.0 * phi.sin(), 30.0 - 30.0 * phi.cos());
    }
    let run_with = |carry: bool| {
        let queue = Pipeline::new()
            .with(CornerSmoothing::new(0.05))
            .run(decode(&src))
            .expect("pipeline");
        let mut ipl = Interpolator::new(limits(), lookahead(carry), Vec::new(), DT);
        ipl.load(queue).expect("program loads");
        run(&mut ipl)
    };
    let (plain, carried) = (run_with(false), run_with(true));
    check_motion(&plain);
    check_motion(&carried);
    assert!(
        carried.len() <= plain.len(),
        "{} cycles carried, {} plain",
        carried.len(),
        plain.len()
    );
}
