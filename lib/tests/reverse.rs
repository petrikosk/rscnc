//! Running the path backward: `Interpolator::set_reverse`.

use rscnc::prelude::*;

const DT: f64 = 0.001;
const A_MAX: f64 = 500.0;
const J_MAX: f64 = 5000.0;

fn interpolator(src: &str) -> Interpolator {
    let mut ipl = Interpolator::new(
        PathLimits::new(50.0, A_MAX, J_MAX, 500.0),
        LookAhead::default(),
        Vec::new(),
        DT,
    );
    let cfg = DecoderConfig {
        fly_by_m: vec![7],
        ..Default::default()
    };
    ipl.load(Decoder::decode_source(cfg, src).expect("program decodes"))
        .expect("program loads");
    ipl
}

/// Cycle until `until` holds for an output (which is included).
fn run_until(
    ipl: &mut Interpolator,
    outputs: &mut Vec<IplOutput>,
    until: impl Fn(&IplOutput) -> bool,
) {
    for _ in 0..400_000 {
        let out = ipl.cycle().expect("cycle succeeds");
        let done = until(&out);
        outputs.push(out);
        if done {
            return;
        }
    }
    panic!("condition never met");
}

/// Dynamic limits from one cycle to the next, in either direction, and a
/// path position that follows the velocity.
fn check_motion(outputs: &[IplOutput]) {
    for (i, w) in outputs.windows(2).enumerate() {
        let (a, b) = (&w[0], &w[1]);
        assert!(b.path_velocity >= 0.0, "cycle {i}: negative velocity");
        assert!(
            b.path_acceleration.abs() <= A_MAX * (1.0 + 1e-6),
            "cycle {i}: acceleration {}",
            b.path_acceleration
        );
        let jerk = (b.path_acceleration - a.path_acceleration).abs() / DT;
        assert!(jerk <= J_MAX * (1.0 + 1e-6), "cycle {i}: jerk {jerk}");
        let step = b.s_total - a.s_total;
        let bound = a.path_velocity.max(b.path_velocity) * DT + A_MAX * DT * DT;
        assert!(step.abs() <= bound, "cycle {i}: step {step} at {bound}");
        if a.reverse && b.reverse {
            assert!(step <= 1e-12, "cycle {i}: forward while reversed");
        } else if !a.reverse && !b.reverse {
            assert!(step >= -1e-12, "cycle {i}: backward while forward");
        } else {
            // the direction only changes at standstill
            assert!(
                a.path_velocity == 0.0 && b.path_velocity < J_MAX * DT * DT,
                "cycle {i}: turned at speed {}",
                a.path_velocity
            );
        }
        let jump: f64 = (0..3)
            .map(|k| (b.path_position[k] - a.path_position[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(jump <= bound, "cycle {i}: position jumped {jump}");
    }
}

/// Distance from `p` to the nearest of the recorded points.
fn distance_to(recorded: &[IplOutput], p: &PathPoint) -> f64 {
    recorded
        .iter()
        .map(|o| {
            (0..3)
                .map(|k| (o.path_position[k] - p[k]).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .fold(f64::INFINITY, f64::min)
}

const CONTOUR: &str = "G1 F40 X20\nG3 X30 Y10 I0 J10\nG1 Y25\nX10 Y30\nX0 Y30\n";

#[test]
fn the_way_back_follows_the_path_to_its_start_and_forward_finishes_it() {
    let mut ipl = interpolator(CONTOUR);
    let mut outputs = Vec::new();
    // into the fourth move
    run_until(&mut ipl, &mut outputs, |o| o.s_total > 56.0);
    let first_obj = outputs[0].obj_no;
    let turned_at = outputs.len();
    ipl.set_reverse(true);
    assert!(!ipl.is_reverse(), "not before the path stands still");
    run_until(&mut ipl, &mut outputs, |o| o.reverse);
    // (the cycle that turns the path around already starts the way back)
    assert!(outputs.last().unwrap().path_velocity < J_MAX * DT * DT);
    let furthest = outputs.last().unwrap().s_total;
    assert!(furthest > 56.0 && furthest < 62.0, "{furthest}");

    // back to the very start, where the path waits
    run_until(&mut ipl, &mut outputs, |o| {
        o.state == IplState::Paused && o.s_total < 1e-9
    });
    let back = outputs.len();
    for _ in 0..20 {
        outputs.push(ipl.cycle().unwrap());
    }
    let at_start = outputs.last().unwrap();
    assert_eq!(at_start.state, IplState::Paused);
    assert!(at_start.reverse);
    assert!(at_start.path_position.iter().all(|c| c.abs() < 1e-9));
    assert!(at_start.progress.abs() < 1e-12);
    // every point of the way back lies on the path that was run forward
    let forward = &outputs[..turned_at + 200];
    for out in &outputs[turned_at..back] {
        let d = distance_to(forward, &out.path_position);
        assert!(d < 0.05, "{d} off the path at s = {}", out.s_total);
    }
    // the way back runs at the programmed feed
    let v_back = outputs[turned_at..back]
        .iter()
        .filter(|o| o.reverse)
        .map(|o| o.path_velocity)
        .fold(0.0, f64::max);
    assert!((v_back - 40.0).abs() < 1e-6, "{v_back}");
    // object numbers and lines are those of the moves, backward
    let objs: Vec<u32> = {
        let mut seen: Vec<u32> = outputs[turned_at..back].iter().map(|o| o.obj_no).collect();
        seen.dedup();
        seen
    };
    assert_eq!(
        objs,
        [first_obj + 3, first_obj + 2, first_obj + 1, first_obj]
    );

    ipl.set_reverse(false);
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    check_motion(&outputs);
    let end = outputs.last().unwrap();
    assert!(!end.reverse);
    assert!(
        (end.path_position[0] - 0.0).abs() < 1e-9 && (end.path_position[1] - 30.0).abs() < 1e-9
    );
    assert!((end.s_total - ipl.total_length()).abs() < 1e-9);
    assert!((end.progress - 1.0).abs() < 1e-12);
}

#[test]
fn an_event_at_the_start_is_reported_before_a_turn_asked_for_at_once() {
    // M8 is fired when the program is loaded and reported by the first
    // cycle: a turn asked for before that cycle waits for it.
    let mut ipl = interpolator("M8\nG1 F40 X10\n");
    ipl.set_reverse(true);
    let first = ipl.cycle().unwrap();
    assert_eq!(first.events.len(), 1);
    assert!(!first.reverse);
    assert_eq!(first.path_velocity, 0.0);
    let mut outputs = vec![first];
    for _ in 0..50 {
        outputs.push(ipl.cycle().unwrap());
    }
    assert!(outputs[1..]
        .iter()
        .all(|o| o.reverse && o.events.is_empty() && o.s_total == 0.0));
    ipl.set_reverse(false);
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    check_motion(&outputs);
    let m: Vec<u32> = outputs
        .iter()
        .flat_map(|o| o.events.iter().flat_map(|e| e.m.clone()))
        .collect();
    assert_eq!(m, [8]);
    assert!((outputs.last().unwrap().path_position[0] - 10.0).abs() < 1e-9);
}

#[test]
fn what_was_reported_before_the_turn_is_not_reported_again() {
    // M8 stops the path, M7 is a fly-by event, H1/H2 are switch points on
    // the second and the fourth move, and there is a dwell after the third.
    let src = "G1 F40 X10\nX20 H1 L5 M8\nX30 M7\nG4 P0.05\nX40 H2 L4 H3 L8\nM9\nX50\n";
    let mut ipl = interpolator(src);
    let mut outputs = Vec::new();
    // 6 mm into the fourth move: past H2, before H3
    run_until(&mut ipl, &mut outputs, |o| o.s_total > 36.0);
    ipl.set_reverse(true);
    run_until(&mut ipl, &mut outputs, |o| o.reverse && o.s_total < 3.0);
    let on_the_way_back = outputs.iter().filter(|o| o.reverse);
    assert_eq!(
        on_the_way_back
            .map(|o| o.events.len() + o.switch_points.len())
            .sum::<usize>(),
        0
    );
    assert!(outputs
        .iter()
        .all(|o| !(o.reverse && o.state == IplState::Dwelling)));
    ipl.set_reverse(false);
    let turned = outputs.len();
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    check_motion(&outputs);

    let m: Vec<u32> = outputs
        .iter()
        .flat_map(|o| o.events.iter().flat_map(|e| e.m.clone()))
        .collect();
    assert_eq!(m, [8, 7, 9], "each M function once, in order");
    let h: Vec<i32> = outputs
        .iter()
        .flat_map(|o| o.switch_points.iter().map(|sp| sp.number))
        .collect();
    assert_eq!(h, [1, 2, 3]);
    // H3 is reported at its place on the way forward again
    let h3 = outputs
        .iter()
        .find(|o| o.switch_points.iter().any(|sp| sp.number == 3))
        .unwrap();
    assert!((h3.s_total - 38.0).abs() < 0.05, "{}", h3.s_total);
    // the dwell and the stop for M8 lie behind the turning point: the
    // second pass goes through without them
    let second_pass = &outputs[turned..];
    let dwelling = |o: &&IplOutput| o.state == IplState::Dwelling;
    assert_eq!(second_pass.iter().filter(dwelling).count(), 0);
    assert!(outputs[..turned].iter().filter(dwelling).count() >= 49);
    // (the path still comes to rest where the program stops it)
    for stop in [20.0, 30.0, 40.0] {
        assert!(
            second_pass
                .iter()
                .any(|o| (o.s_total - stop).abs() < 1e-3 && o.path_velocity < 0.1),
            "no stop at {stop}"
        );
    }
}

#[test]
fn turning_around_without_moving_changes_nothing() {
    // at the end of the program
    let mut ipl = interpolator("G1 F40 X10\nG4 P0.2\nX20\n");
    let mut outputs = Vec::new();
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    ipl.set_reverse(true);
    ipl.set_override(0.0);
    for _ in 0..5 {
        outputs.push(ipl.cycle().unwrap());
    }
    assert!(ipl.is_reverse());
    assert_eq!(outputs.last().unwrap().state, IplState::Paused);
    ipl.set_reverse(false);
    ipl.set_override(1.0);
    outputs.push(ipl.cycle().unwrap());
    let out = outputs.last().unwrap();
    assert_eq!(out.state, IplState::Finished);
    assert!(!out.reverse && (out.s_total - 20.0).abs() < 1e-9);

    // in the middle of a dwell: it goes on where it was
    let mut ipl = interpolator("G1 F40 X10\nG4 P0.2\nX20\n");
    let mut outputs = Vec::new();
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Dwelling);
    for _ in 0..49 {
        outputs.push(ipl.cycle().unwrap());
    }
    ipl.halt();
    ipl.set_reverse(true);
    for _ in 0..30 {
        outputs.push(ipl.cycle().unwrap());
    }
    assert!(ipl.is_reverse());
    ipl.set_reverse(false);
    ipl.resume();
    let from = outputs.len();
    run_until(&mut ipl, &mut outputs, |o| o.state != IplState::Dwelling);
    let rest = outputs.len() - from;
    assert!(
        (149..=151).contains(&rest),
        "dwell went on for {rest} cycles"
    );
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    check_motion(&outputs);
}

#[test]
fn a_dwell_cut_short_by_the_turn_is_run_again() {
    let mut ipl = interpolator("G1 F40 X10\nG4 P0.2\nX20\n");
    let mut outputs = Vec::new();
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Dwelling);
    for _ in 0..100 {
        outputs.push(ipl.cycle().unwrap());
    }
    ipl.set_reverse(true);
    run_until(&mut ipl, &mut outputs, |o| o.reverse && o.s_total < 5.0);
    ipl.set_reverse(false);
    let from = outputs.len();
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    check_motion(&outputs);
    let dwell = outputs[from..]
        .iter()
        .filter(|o| o.state == IplState::Dwelling)
        .count();
    assert!((199..=201).contains(&dwell), "{dwell} cycles");
}

#[test]
fn halt_override_and_single_step_work_on_the_way_back() {
    let mut ipl = interpolator(CONTOUR);
    let mut outputs = Vec::new();
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    let total = ipl.total_length();
    ipl.set_reverse(true);
    ipl.set_override(0.5);
    run_until(&mut ipl, &mut outputs, |o| {
        o.reverse && o.s_total < total - 8.0
    });
    assert!((outputs.last().unwrap().path_velocity - 20.0).abs() < 1e-6);
    ipl.halt();
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Paused);
    let held = outputs.last().unwrap().s_total;
    for _ in 0..50 {
        outputs.push(ipl.cycle().unwrap());
    }
    assert_eq!(outputs.last().unwrap().s_total, held);
    ipl.resume();
    ipl.set_override(1.0);
    ipl.set_single_step(true);
    // every move of the way back ends in a hold
    let mut holds = Vec::new();
    for _ in 0..4 {
        run_until(&mut ipl, &mut outputs, |o| o.state == IplState::StepHold);
        holds.push(outputs.last().unwrap().s_total);
        ipl.step();
    }
    let ends = [total - 10.0, 10.0 * std::f64::consts::FRAC_PI_2 + 35.0];
    assert!((holds[0] - ends[0]).abs() < 1e-9, "{holds:?}");
    assert!((holds[1] - ends[1]).abs() < 1e-9, "{holds:?}");
    // forward again from a hold on the way back
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Running);
    ipl.set_reverse(false);
    ipl.set_single_step(false);
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    check_motion(&outputs);
    assert!((outputs.last().unwrap().s_total - total).abs() < 1e-9);
}

#[test]
fn appending_and_discarding_wait_for_the_forward_direction() {
    let mut ipl = interpolator("G1 F40 X10\nY10\nX0\n");
    let mut outputs = Vec::new();
    run_until(&mut ipl, &mut outputs, |o| o.s_total > 25.0);
    ipl.set_reverse(true);
    let more = || {
        Decoder::decode_source(
            DecoderConfig {
                start_position: [0.0, 10.0, 0.0],
                ..Default::default()
            },
            "G1 F40 Y0\n",
        )
        .unwrap()
    };
    assert!(ipl.append(more()).is_err());
    assert_eq!(ipl.discard_history(0.0), 0);
    run_until(&mut ipl, &mut outputs, |o| o.reverse && o.s_total < 15.0);
    assert!(ipl.append(more()).is_err());
    ipl.set_reverse(false);
    run_until(&mut ipl, &mut outputs, |o| !o.reverse && o.s_total > 16.0);
    // history discarded: the way back ends where what is left begins
    assert_eq!(ipl.discard_history(0.0), 1);
    ipl.append(more()).unwrap();
    ipl.set_reverse(true);
    run_until(&mut ipl, &mut outputs, |o| {
        o.reverse
            && o.state == IplState::Paused
            && o.path_velocity == 0.0
            && o.s_total < 10.0 + 1e-9
    });
    let at = outputs.last().unwrap();
    assert!((at.s_total - 10.0).abs() < 1e-9, "{}", at.s_total);
    assert!((at.path_position[0] - 10.0).abs() < 1e-9 && at.path_position[1].abs() < 1e-9);
    ipl.set_reverse(false);
    run_until(&mut ipl, &mut outputs, |o| o.state == IplState::Finished);
    check_motion(&outputs);
    let end = outputs.last().unwrap();
    assert!((end.s_total - 40.0).abs() < 1e-9);
    assert!(end.path_position[0].abs() < 1e-9 && end.path_position[1].abs() < 1e-9);
}
