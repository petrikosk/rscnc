//! Randomized end-to-end stress test: random programs (long and very short
//! segments, sharp and gentle corners, arcs, splines, rapids, dwells,
//! events, auxiliary moves, `E` limits, switch points), loaded whole or
//! appended in pieces while the path runs, under random operator actions
//! (override changes, halt / resume, single-step, backward motion), across
//! several limit sets (per-axis limits included) and cycle times, with and
//! without carried acceleration.
//!
//! Whatever happens, the motion along the path must stay one continuous
//! jerk-limited profile within the limits, and the program must end exactly
//! at its end point. The generator is a fixed-seed xorshift, so a failure
//! reproduces by its case number. Programmed `E` limits must hold move by
//! move, and every switch point must be reported exactly once, in order.
//!
//! To look into a failing case: `RSCNC_STRESS_FIRST=<case>
//! RSCNC_STRESS_CASES=1` runs it alone, `RSCNC_STRESS_TRACE=<first>,<last>`
//! prints the path state of those cycles (and the plan in force at the
//! first), `RSCNC_STRESS_NO_CARRY=1` plans every case without carried
//! acceleration.

use rscnc::prelude::*;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.unit()
    }
    fn log_range(&mut self, a: f64, b: f64) -> f64 {
        self.range(a.ln(), b.ln()).exp()
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn program(rng: &mut Rng, aux: bool) -> String {
    let mut src = format!("G1 F{:.0}\n", rng.log_range(5.0, 150.0));
    let (mut x, mut y) = (0.0f64, 0.0f64);
    let mut heading = rng.range(0.0, std::f64::consts::TAU);
    // 0 mixed, 1 fine and smooth, 2 short and jagged, 3 long
    let style = rng.below(4);
    for _ in 0..2 + rng.below(40) {
        let len = match style {
            1 => rng.log_range(0.02, 1.0),
            2 => rng.log_range(0.005, 0.5),
            3 => rng.log_range(5.0, 40.0),
            _ => rng.log_range(0.01, 30.0),
        };
        heading += match style {
            1 => rng.range(-0.1, 0.1),
            _ if rng.below(4) == 0 => rng.range(-std::f64::consts::PI, std::f64::consts::PI),
            _ => rng.range(-0.3, 0.3),
        };
        let roll = rng.below(40);
        match roll {
            0 => {
                src += &format!("G4 P{:.3}\n", rng.range(0.0, 0.05));
                continue;
            }
            1 => {
                src += ["M8\n", "M8\n", "M9\n", "M8 M9\n", "M7 T3\n", "M7\nM8\n"]
                    [rng.below(6) as usize];
                continue;
            }
            2 => {
                src += &format!("F{:.0}\n", rng.log_range(5.0, 150.0));
                continue;
            }
            // E: programmed path acceleration (positive) / deceleration
            3 | 4 => {
                let sign = if rng.below(2) == 0 { "" } else { "-" };
                src += &format!("E{sign}{:.0}\n", rng.log_range(30.0, 4000.0));
                continue;
            }
            _ => {}
        }
        let (nx, ny) = (x + len * heading.cos(), y + len * heading.sin());
        let mut a = if aux && rng.below(3) == 0 {
            format!(" A{:.3}", rng.range(-20.0, 20.0))
        } else {
            String::new()
        };
        // Switch points somewhere on the element
        for _ in 0..[0, 0, 0, 0, 1, 3][rng.below(6) as usize] {
            let number = 1 + rng.below(9) as i32;
            let sign = if rng.below(2) == 0 { "" } else { "-" };
            a += &format!(" H{sign}{number} O{:.3}", rng.range(0.0, 1.0));
        }
        if roll < 8 && len > 0.05 {
            let r = len * rng.range(0.55, 5.0);
            src += &format!("G{} X{nx:.4} Y{ny:.4} R{r:.4}{a}\nG1\n", 2 + rng.below(2));
        } else if roll == 8 {
            src += &format!("G0 X{nx:.4} Y{ny:.4}{a}\nG1\n");
        } else if roll == 9 || roll == 10 {
            // A spline through a few more points
            src += &format!("G5 X{nx:.4} Y{ny:.4}{a}\n");
            let (mut px, mut py) = (nx, ny);
            for _ in 0..rng.below(5) {
                heading += rng.range(-1.2, 1.2);
                let len = rng.log_range(0.05, 20.0);
                (px, py) = (px + len * heading.cos(), py + len * heading.sin());
                src += &format!("X{px:.4} Y{py:.4}\n");
            }
            src += "G1\n";
            (x, y) = (px, py);
            continue;
        } else {
            src += &format!("X{nx:.4} Y{ny:.4}{a}\n");
        }
        (x, y) = (nx, ny);
    }
    src
}

fn run_case(case: u64) {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15u64.wrapping_mul(case + 1) | 1);
    let aux = rng.below(3) == 0;
    let (lim, dt, lookahead) = match rng.below(5) {
        0 | 1 => (
            PathLimits::new(50.0, 500.0, 5000.0, 500.0),
            0.001,
            LookAhead::default(),
        ),
        2 => (
            PathLimits::new(200.0, 3000.0, 100_000.0, 2000.0),
            0.001,
            LookAhead::default(),
        ),
        3 => (
            PathLimits::new(100.0, 1000.0, 20_000.0, 800.0),
            0.0005,
            LookAhead {
                junction_deviation: 0.01,
                accel_margin: 1.0,
                ..LookAhead::default()
            },
        ),
        _ => (
            PathLimits::new(50.0, 2000.0, 8000.0, 300.0),
            0.002,
            LookAhead {
                junction_deviation: 0.2,
                accel_margin: 0.5,
                ..LookAhead::default()
            },
        ),
    };
    // (every fourth case on the plain plan alone)
    let lookahead = LookAhead {
        carry_acceleration: case % 4 != 3 && std::env::var("RSCNC_STRESS_NO_CARRY").is_err(),
        ..lookahead
    };
    // Cycles to print (`RSCNC_STRESS_TRACE=first,last`).
    let trace = std::env::var("RSCNC_STRESS_TRACE").ok().and_then(|t| {
        let (lo, hi) = t.split_once(',')?;
        Some((lo.parse::<usize>().ok()?, hi.parse::<usize>().ok()?))
    });
    let src = program(&mut rng, aux);
    // M functions the path does not stop for
    let fly_by_m = match rng.below(3) {
        0 => vec![7, 8, 9],
        1 => vec![8],
        _ => Vec::new(),
    };
    let cfg = if aux {
        DecoderConfig {
            aux_letters: vec!['A'],
            start_aux: vec![0.0],
            fly_by_m,
            ..Default::default()
        }
    } else {
        DecoderConfig {
            fly_by_m,
            ..Default::default()
        }
    };
    const AUX_LIMIT: f64 = 30.0;
    // Limits of the single axes: none, or random ones around the path's
    let mut axis_limits = AxisLimits::from_aux_velocity(if aux { &[AUX_LIMIT] } else { &[] });
    if rng.below(3) == 0 {
        let limit = |rng: &mut Rng, velocity: f64| {
            let mut l = AxisLimit::new(
                velocity,
                lim.max_acceleration * rng.log_range(0.2, 3.0),
                lim.max_jerk * rng.log_range(0.2, 3.0),
            );
            if rng.below(2) == 0 {
                l = l.with_velocity_jump(rng.log_range(0.2, 20.0));
            }
            l
        };
        for k in 0..2 {
            axis_limits.path[k] = limit(&mut rng, f64::INFINITY);
        }
        for a in &mut axis_limits.aux {
            *a = limit(&mut rng, AUX_LIMIT);
        }
    }
    // Arcs with a random radius do not always fit their chord: such a
    // program is rejected by the decoder and skipped here.
    let Ok(mut queue) = Decoder::decode_source(cfg, &src) else {
        return;
    };
    // Sometimes with the corners smoothed by spline blends
    if rng.below(5) == 0 {
        let tolerance = rng.log_range(0.005, 1.0);
        queue = CornerSmoothing::new(tolerance)
            .process(queue)
            .unwrap_or_else(|e| panic!("case {case}: corner smoothing: {e}\n{src}"));
    }
    // Per move (by object number): its programmed acceleration and
    // deceleration limits, and its switch points.
    let cap = |e: Option<f64>| e.map_or(lim.max_acceleration, |e| e.min(lim.max_acceleration));
    let mut move_limits: Vec<(f64, f64)> = Vec::new();
    let mut expected_switches = Vec::new();
    let mut expected_events = Vec::new();
    for elem in queue.iter() {
        match elem {
            GeoElem::Move(m) => {
                // (a corner blend carries the number of a move next to it)
                let limits = (cap(m.accel), cap(m.decel));
                match move_limits.get_mut(m.obj_no as usize) {
                    Some((acc, dec)) => (*acc, *dec) = (acc.max(limits.0), dec.max(limits.1)),
                    None => {
                        assert_eq!(m.obj_no as usize, move_limits.len());
                        move_limits.push(limits);
                    }
                }
                expected_switches.extend(m.switch_points.iter().copied());
            }
            GeoElem::Event(e) => expected_events.push(e.clone()),
            _ => {}
        }
    }
    let mut seen_switches = Vec::new();
    let mut seen_events = Vec::new();
    let path_axes = axis_limits.path;
    let aux_axis = axis_limits.aux.first().copied();
    let mut ipl = Interpolator::with_axis_limits(lim, lookahead, axis_limits, dt);
    // Sometimes the program is fed in pieces while it runs
    let mut pieces = std::collections::VecDeque::new();
    if rng.below(3) == 0 {
        let mut piece = GeoQueue::new();
        for elem in queue {
            piece.push(elem);
            if rng.below(6) == 0 {
                pieces.push_back(std::mem::take(&mut piece));
            }
        }
        pieces.push_back(piece);
    } else {
        pieces.push_back(queue);
    }
    ipl.load(pieces.pop_front().unwrap())
        .unwrap_or_else(|e| panic!("case {case}: {e}\n{src}"));
    let mut out = ipl.output_buffer();
    let mut switch_capacity = out.switch_points.capacity();

    let operator = rng.below(4) != 0;
    // ... and sometimes the operator runs it back and forth
    let reversing = operator && rng.below(3) == 0;
    let mut reverse = false;
    let mut reverse_prev = false;
    // Without an operator, and with the whole program planned at once, the
    // plan says how fast each move and each junction may be taken: (path
    // position of the move's end, velocity cap, junction limit).
    let planned: Vec<(f64, f64, f64)> = if !operator && pieces.is_empty() {
        let mut end = 0.0;
        ipl.planned_segments()
            .iter()
            .map(|seg| {
                end += seg.length;
                (end, seg.v_cap, seg.v_junction)
            })
            .collect()
    } else {
        Vec::new()
    };
    let mut planned_pos = 0;
    let ds_tol = 0.2 * lim.max_jerk * dt * dt * dt + 1e-6;
    let (mut s_prev, mut v_prev, mut a_prev) = (0.0f64, 0.0f64, 0.0f64);
    let (mut obj_prev, mut state_prev) = (0usize, IplState::Idle);
    let mut resume_at = None;
    let mut paused = false;
    let mut single_step = false;
    for i in 0..2_000_000usize {
        if operator && i < 60_000 {
            match rng.below(500) {
                0 => {
                    let factor = match rng.below(5) {
                        0 => 0.0,
                        1 => rng.range(1.0, 2.5),
                        _ => rng.range(0.05, 1.2),
                    };
                    paused = factor == 0.0;
                    ipl.set_override(factor);
                }
                1 => {
                    ipl.halt();
                    resume_at = Some(i + rng.below(300) as usize);
                }
                2 => {
                    single_step = !single_step;
                    ipl.set_single_step(single_step);
                }
                3 | 4 if reversing => {
                    reverse = !reverse;
                    ipl.set_reverse(reverse);
                }
                5 => {
                    ipl.discard_history(rng.range(0.0, 40.0));
                }
                _ => {}
            }
            if resume_at.is_some_and(|at| i >= at) {
                ipl.resume();
                resume_at = None;
            }
            if paused && rng.below(200) == 0 {
                paused = false;
                ipl.set_override(rng.range(0.2, 1.5));
            }
            if single_step && rng.below(40) == 0 {
                ipl.step();
            }
        } else if i == 60_000 {
            // let it run out
            ipl.set_override(1.0);
            ipl.resume();
            ipl.set_single_step(false);
            reverse = false;
            ipl.set_reverse(false);
        }
        if !pieces.is_empty()
            && !reverse
            && !ipl.is_reverse()
            && (ipl.state() == IplState::Finished || i >= 60_000 || rng.below(400) == 0)
        {
            ipl.append(pieces.pop_front().unwrap())
                .unwrap_or_else(|e| panic!("case {case}, cycle {i}: append: {e}\n{src}"));
            // (the appended part may need more room per cycle)
            out = ipl.output_buffer();
            switch_capacity = out.switch_points.capacity();
        }

        ipl.cycle_into(&mut out)
            .unwrap_or_else(|e| panic!("case {case}, cycle {i}: {e}\n{src}"));
        let (s, v, a) = (out.s_total, out.path_velocity, out.path_acceleration);
        if trace.is_some_and(|(lo, hi)| (lo..=hi).contains(&i)) {
            eprintln!(
                "cycle {i}: s {s:.9} v {v:.6} a {a:.4} {:?} line {} override {} reverse {}",
                out.state,
                out.src_line,
                ipl.override_factor(),
                ipl.is_reverse()
            );
            if i == trace.unwrap().0 {
                let mut end = 0.0;
                for (k, p) in ipl.planned_segments().iter().enumerate() {
                    end += p.length;
                    eprintln!(
                        "  plan {k}: end {end:.6} cap {:.4} exit {:.4} junction {:.4} \
                         accel {:.1} decel {:.1} jerk {:.1} carried {:.4} {:.2}",
                        p.v_cap,
                        p.v_exit,
                        p.v_junction,
                        p.accel,
                        p.decel,
                        p.jerk,
                        p.v_carry,
                        p.a_carry
                    );
                }
            }
        }
        let fail = |what: &str| -> ! {
            panic!(
                "case {case}, cycle {i}: {what}\n  s {s_prev} -> {s}, v {v_prev} -> {v}, \
                 a {a_prev} -> {a}, state {:?}, line {}\n{src}",
                out.state, out.src_line
            )
        };
        if !(0.0..=lim.max_velocity * (1.0 + 1e-9) + 1e-9).contains(&v) {
            fail("velocity out of range");
        }
        // (the direction changes at standstill only, in a cycle that may
        // already start the way back)
        let turned = out.reverse != reverse_prev;
        if turned && v_prev != 0.0 {
            fail("turned around in motion");
        }
        if !turned && (if out.reverse { s > s_prev } else { s < s_prev }) {
            fail("moved against the direction of travel");
        }
        if ((s - s_prev).abs() - 0.5 * (v_prev + v) * dt).abs() > ds_tol {
            fail("progress does not match the velocity (stall or jump)");
        }
        if out.reverse && (!out.events.is_empty() || !out.switch_points.is_empty()) {
            fail("an event or switch point was reported on the way back");
        }
        if !planned.is_empty() {
            // The junctions passed in this cycle: somewhere between the two
            // samples the velocity was the one they were crossed at.
            let slack = lim.max_jerk * dt * dt + 1e-6;
            while planned_pos + 1 < planned.len() && planned[planned_pos].0 < s - 1e-9 {
                if v.min(v_prev) > planned[planned_pos].2 * (1.0 + 1e-6) + slack {
                    fail("junction velocity limit exceeded");
                }
                planned_pos += 1;
            }
            // The cap of the move the path is in (at a junction: of either)
            let mut cap = planned[planned_pos].1;
            if planned_pos > 0 && planned[planned_pos - 1].0 >= s - 1e-9 {
                cap = cap.max(planned[planned_pos - 1].1);
            }
            if planned_pos + 1 < planned.len() && planned[planned_pos].0 <= s + 1e-9 {
                cap = cap.max(planned[planned_pos + 1].1);
            }
            if v > cap * (1.0 + 1e-6) + 1e-6 {
                fail("velocity cap of the move exceeded");
            }
        }
        reverse_prev = out.reverse;
        if i > 0 {
            if ((v - v_prev) / dt).abs() > lim.max_acceleration * (1.0 + 1e-6) + 1e-6 {
                fail("acceleration limit exceeded");
            }
            if ((a - a_prev) / dt).abs() > lim.max_jerk * (1.0 + 1e-6) + 1e-6 {
                fail("jerk limit exceeded");
            }
        }
        // (a program may consist of dwells and events only: no axes then)
        let aux_velocity = out.aux_velocity.first().map_or(0.0, |v| v.abs());
        if aux_velocity > AUX_LIMIT * (1.0 + 1e-9) + 1e-9 {
            fail("auxiliary axis velocity limit exceeded");
        }
        // The programmed E limits of the move(s) this cycle ran over. A stop
        // that runs on into a move with a lower limit cannot shed its
        // deceleration at once, so only regular running is held to them.
        let obj = out.obj_no as usize;
        if i > 0 && out.state == IplState::Running && state_prev == IplState::Running {
            // (a program may consist of dwells and events only)
            let of = |obj: usize| move_limits.get(obj).copied().unwrap_or((0.0, 0.0));
            let (acc0, dec0) = of(obj_prev);
            let (acc1, dec1) = of(obj);
            let dv = (v - v_prev) / dt;
            if dv > acc0.max(acc1) * (1.0 + 1e-6) + 1e-6 {
                fail("programmed acceleration (E) exceeded");
            }
            if -dv > dec0.max(dec1) * (1.0 + 1e-6) + 1e-6 {
                fail("programmed deceleration (E-) exceeded");
            }
        }
        // The limits of the single axes, along the path, while one move
        // runs regularly.
        if i > 0
            && obj == obj_prev
            && out.state == IplState::Running
            && state_prev == IplState::Running
        {
            for (k, axis) in path_axes.iter().enumerate() {
                if a.abs() * out.path_tangent[k].abs() > axis.max_acceleration * (1.0 + 1e-6) + 1e-6
                {
                    fail("axis acceleration limit exceeded");
                }
            }
            if let (Some(axis), Some(rate)) = (aux_axis, out.aux_velocity.first()) {
                // (the axis follows the path at a fixed rate within a move)
                if v > 1e-3
                    && a.abs() * (rate / v).abs() > axis.max_acceleration * (1.0 + 1e-6) + 1e-6
                {
                    fail("auxiliary axis acceleration limit exceeded");
                }
            }
        }
        // An event that is not fly-by is reported in the cycle the path
        // stops for it (the rest of that cycle may already start the next
        // move from standstill).
        for event in &out.events {
            if !event.fly_by && v > 0.5 * lim.max_jerk * dt * dt * (1.0 + 1e-6) + 1e-9 {
                fail("an event that stops the path was reported in motion");
            }
        }
        seen_events.extend(out.events.iter().cloned());
        (obj_prev, state_prev) = (obj, out.state);
        seen_switches.extend(out.switch_points.iter().copied());
        if out.switch_points.capacity() != switch_capacity {
            fail("the switch point buffer had to grow");
        }
        (s_prev, v_prev, a_prev) = (s, v, a);
        if out.state == IplState::Finished && pieces.is_empty() && !reverse {
            let total = ipl.total_length();
            assert!(
                (s - total).abs() <= 1e-6,
                "case {case}: ended at s = {s} of {total}\n{src}"
            );
            // Every switch point is reported exactly once, in path order.
            assert_eq!(
                seen_switches, expected_switches,
                "case {case}: switch points\n{src}"
            );
            assert_eq!(seen_events, expected_events, "case {case}: events\n{src}");
            return;
        }
    }
    panic!("case {case}: did not finish\n{src}");
}

#[test]
fn random_programs_and_operator_actions_keep_the_profile_smooth() {
    // `RSCNC_STRESS_CASES` / `RSCNC_STRESS_FIRST` widen the run or move it
    // to other seeds (e.g. a long soak outside the normal test run).
    let var = |name: &str, default: u64| {
        std::env::var(name)
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(default)
    };
    let first = var("RSCNC_STRESS_FIRST", 0);
    for case in first..first + var("RSCNC_STRESS_CASES", 400) {
        run_case(case);
    }
}
