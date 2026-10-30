//! Commissioning procedures: self-test, gain tuning and preload test.
//!
//! All three move the hand with a low current limit and write their findings into a
//! [`HandProfile`]. They are the only parts of the driver, besides [`crate::control`], that
//! enable the motors.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use apeiron_protocol::joint::{approx_limits, JointId, NUM_JOINTS};
use apeiron_protocol::keys::rel::node;
use apeiron_protocol::messages::MitParam;

use crate::control::{Command, ControlHandle, ControlMode, Controller, Telemetry, Timing};
use crate::hand::Hand;
use crate::motion::{Segment, Trajectory};
use crate::profile::{Gains, HandProfile};
use crate::Result;

/// Joints whose joint-side encoder (enc2) reads exactly zero while the motor-side encoder
/// (enc1) does not: the signature of a frozen sensor value. The firmware does not flag this
/// state itself and would drive the joint against a phantom target. A power cycle clears it.
pub async fn frozen_encoders(hand: &Hand) -> Vec<JointId> {
    let mut out = Vec::new();
    for i in 0..NUM_JOINTS {
        let id = JointId::from_index(i).unwrap();
        let (e1, e2) = (hand.get_node_f32(id, node::POS_ENC1).await, hand.get_node_f32(id, node::POS_ENC2).await);
        if let (Ok(e1), Ok(e2)) = (e1, e2) {
            if e2 == 0.0 && e1.abs() > 0.01 {
                out.push(id);
            }
        }
    }
    out
}

// ------------------------------------------------------------------ self-test

#[derive(Debug, Clone, Default)]
pub struct JointCheck {
    pub moved: Option<bool>,
    pub returned: Option<bool>,
    pub enc1_follows: Option<bool>,
    pub enc2_follows: Option<bool>,
    pub encoders_agree: Option<bool>,
    pub crosstalk: Vec<JointId>,
    pub max_current: f32,
    pub error_code: u16,
    pub notes: Vec<String>,
}

impl JointCheck {
    pub fn ok(&self) -> bool {
        self.moved == Some(true)
            && self.returned != Some(false)
            && self.enc1_follows == Some(true)
            && self.enc2_follows == Some(true)
            && self.encoders_agree == Some(true)
            && self.crosstalk.is_empty()
            && self.notes.is_empty()
    }
}

#[derive(Debug, Clone)]
pub struct SelfCheckReport {
    pub joints: Vec<(JointId, JointCheck)>,
    pub start: [f32; NUM_JOINTS],
    pub end: [f32; NUM_JOINTS],
}

impl SelfCheckReport {
    pub fn all_ok(&self) -> bool {
        self.joints.iter().all(|(_, c)| c.ok())
    }
}

/// Deflect every joint by `delta` rad (direction chosen away from the nearer limit), one
/// at a time, and verify: the joint moves (stream), both raw encoders follow and agree,
/// the joint returns, no other joint moves, current and error code stay sane.
pub async fn self_check(hand: &Hand, gains: Gains, timing: Timing, delta: f32, secs_per_joint: f32) -> Result<SelfCheckReport> {
    let ramp = secs_per_joint * 0.27;
    let hold_out = secs_per_joint * 0.35;
    let hold_back = secs_per_joint - 2.0 * ramp - hold_out;
    let start = hand.positions().await?;

    let mut enc_base = [(f32::NAN, f32::NAN); NUM_JOINTS];
    for i in 0..NUM_JOINTS {
        let id = JointId::from_index(i)?;
        enc_base[i] = (
            hand.get_node_f32(id, node::POS_ENC1).await.unwrap_or(f32::NAN),
            hand.get_node_f32(id, node::POS_ENC2).await.unwrap_or(f32::NAN),
        );
    }
    let frozen = frozen_encoders(hand).await;

    let mut gains = gains;
    for id in &frozen {
        gains.effort_limit[id.index()] = 0.0;
    }
    let mut deltas = [0f32; NUM_JOINTS];
    let mut segments = Vec::new();
    for k in 0..NUM_JOINTS {
        let (lo, hi) = approx_limits(k)?;
        let d = if start[k] + delta <= hi && (hi - start[k]) >= (start[k] - lo) {
            delta
        } else if start[k] - delta >= lo {
            -delta
        } else {
            delta.min(hi - start[k])
        };
        deltas[k] = d;
        let mut goal = start;
        goal[k] += d;
        segments.push(Segment { goal, ramp, hold: hold_out, label: format!("{} out", JointId::from_index(k)?.label()) });
        segments.push(Segment { goal: start, ramp, hold: hold_back, label: format!("{} back", JointId::from_index(k)?.label()) });
    }
    let traj = Trajectory { start, segments };

    let controller = Controller::new(hand, gains).mode(ControlMode::Position).timing(timing);
    let (handle, join) = controller.start().await?;
    let source_handle = handle.clone();
    let traj_c = traj.clone();
    let player = tokio::spawn(async move { play(source_handle, traj_c).await });

    // Sample telemetry and read the raw encoders in the middle of every "out" hold.
    let t0 = Instant::now();
    let mut samples: Vec<Telemetry> = Vec::with_capacity(40_000);
    let mut enc_mid = [(f32::NAN, f32::NAN); NUM_JOINTS];
    let mut next = 0usize;
    let total = traj.total_secs();
    while t0.elapsed().as_secs_f32() < total + 0.3 && !handle.is_stopped() {
        if next < NUM_JOINTS && t0.elapsed().as_secs_f32() >= next as f32 * secs_per_joint + ramp + hold_out * 0.5 {
            let id = JointId::from_index(next)?;
            enc_mid[next] = (
                hand.get_node_f32(id, node::POS_ENC1).await.unwrap_or(f32::NAN),
                hand.get_node_f32(id, node::POS_ENC2).await.unwrap_or(f32::NAN),
            );
            next += 1;
        }
        if let Some(t) = handle.next_telemetry().await {
            samples.push(t);
        }
    }
    let _ = player.await;
    handle.stop();
    let _ = join.await;
    controller.disarm().await?;
    let end = hand.positions().await?;

    let window = |a: f32, b: f32| samples.iter().filter(move |s| s.t.as_secs_f32() >= a && s.t.as_secs_f32() <= b);
    let tol = 0.5 * delta;
    let mut joints = Vec::new();
    for k in 0..NUM_JOINTS {
        let id = JointId::from_index(k)?;
        let t_out = (k as f32 * secs_per_joint + ramp + 0.1, k as f32 * secs_per_joint + ramp + hold_out - 0.05);
        let t_back = ((k + 1) as f32 * secs_per_joint - hold_back + 0.05, (k + 1) as f32 * secs_per_joint - 0.02);
        let mid = median(window(t_out.0, t_out.1).map(|s| s.position[k]));
        let back = median(window(t_back.0, t_back.1).map(|s| s.position[k]));
        let mut c = JointCheck {
            max_current: window(k as f32 * secs_per_joint, (k + 1) as f32 * secs_per_joint).map(|s| s.current[k].abs()).fold(0.0, f32::max),
            error_code: window(k as f32 * secs_per_joint, (k + 1) as f32 * secs_per_joint)
                .map(|s| s.error_code[k])
                .next_back()
                .unwrap_or(0),
            ..Default::default()
        };
        if frozen.contains(&id) {
            c.notes.push("joint encoder frozen (enc2 = 0) – power-cycle the hand".into());
        }
        if let Some(m) = mid {
            c.moved = Some((m - start[k]).abs() >= tol && (m - start[k]).signum() == deltas[k].signum());
            if c.moved == Some(false) {
                c.notes.push(format!("stream moved {:+.3} rad instead of {:+.3}", m - start[k], deltas[k]));
            }
        }
        if let Some(b) = back {
            c.returned = Some((b - start[k]).abs() < 0.05);
        }
        let ((e1b, e2b), (e1m, e2m)) = (enc_base[k], enc_mid[k]);
        if [e1b, e2b, e1m, e2m].iter().all(|v| v.is_finite()) {
            c.enc1_follows = Some((e1m - e1b).abs() >= tol);
            c.enc2_follows = Some((e2m - e2b).abs() >= tol);
            c.encoders_agree = Some((e1m - e2m).abs() < 0.1);
            if c.enc1_follows == Some(false) {
                c.notes.push(format!("enc1 moved {:+.3}", e1m - e1b));
            }
            if c.enc2_follows == Some(false) {
                c.notes.push(format!("enc2 moved {:+.3}", e2m - e2b));
            }
            if c.encoders_agree == Some(false) {
                c.notes.push(format!("enc1 − enc2 = {:+.3}", e1m - e2m));
            }
        } else {
            c.notes.push("encoder registers not readable".into());
        }
        for j in 0..NUM_JOINTS {
            if j != k {
                if let Some(mj) = median(window(t_out.0, t_out.1).map(|s| s.position[j])) {
                    if (mj - start[j]).abs() > 0.05 {
                        c.crosstalk.push(JointId::from_index(j)?);
                    }
                }
            }
        }
        joints.push((id, c));
    }
    Ok(SelfCheckReport { joints, start, end })
}

/// Drive a handle along a trajectory until it ends (used by the procedures above).
async fn play(handle: ControlHandle, traj: Trajectory) {
    let t0 = Instant::now();
    let total = traj.total_secs();
    let mut tick = tokio::time::interval(Duration::from_millis(5));
    loop {
        tick.tick().await;
        let t = t0.elapsed().as_secs_f32();
        if t > total || handle.is_stopped() {
            break;
        }
        handle.set_command(Command { position: traj.at(t), velocity: traj.velocity_at(t), effort: [0.0; NUM_JOINTS] });
    }
}

fn median(iter: impl Iterator<Item = f32>) -> Option<f32> {
    let mut v: Vec<f32> = iter.filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        return None;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Some(v[v.len() / 2])
}

fn rms(v: &[f32]) -> f32 {
    if v.is_empty() {
        return 0.0;
    }
    let m = v.iter().sum::<f32>() / v.len() as f32;
    (v.iter().map(|x| (x - m) * (x - m)).sum::<f32>() / v.len() as f32).sqrt()
}

// --------------------------------------------------------------------- tuning

#[derive(Debug, Clone)]
pub struct StepMetrics {
    pub kp: f32,
    pub kd: f32,
    pub rise_ms: Option<f32>,
    pub settle_ms: Option<f32>,
    pub overshoot_pct: f32,
    pub steady_error: f32,
    pub current_rms: f32,
    pub velocity_rms: f32,
    pub max_current: f32,
}

/// Acceptance thresholds for a gain candidate.
#[derive(Debug, Clone, Copy)]
pub struct TuneCriteria {
    pub max_overshoot_pct: f32,
    /// Current ripple while holding, A (sensor noise is ≈ 0.01).
    pub max_current_rms: f32,
    /// Velocity ripple while holding, rad/s.
    pub max_velocity_rms: f32,
}

impl Default for TuneCriteria {
    fn default() -> Self {
        TuneCriteria { max_overshoot_pct: 15.0, max_current_rms: 0.03, max_velocity_rms: 0.05 }
    }
}

#[derive(Debug, Clone)]
pub struct TuneReport {
    pub joints: BTreeMap<usize, Vec<StepMetrics>>,
    /// Recommended (kp, kd) per joint: settled, little overshoot, quiet, smallest residual.
    pub recommended: BTreeMap<usize, (f32, f32)>,
}

/// Step responses for every (kp, kd) candidate on each listed joint. The joint under test
/// gets the candidate gains, all others keep `gains`; velocity feed-forward is off so only
/// the gains shape the response.
pub async fn tune(
    hand: &Hand,
    gains: Gains,
    timing: Timing,
    joints: &[usize],
    kps: &[f32],
    kds: &[f32],
    step: f32,
    secs: f32,
    criteria: TuneCriteria,
) -> Result<TuneReport> {
    let start = hand.positions().await?;
    let combos: Vec<(f32, f32)> = kps.iter().flat_map(|&kp| kds.iter().map(move |&kd| (kp, kd))).collect();
    let trial = 2.0 * secs;

    let mut segments = Vec::new();
    let mut plan: Vec<(usize, f32, f32, f32)> = Vec::new();
    let mut t = 0f32;
    for &j in joints {
        let (lo, hi) = approx_limits(j)?;
        let d = if start[j] + step <= hi { step } else { -step.min(start[j] - lo) };
        for &(kp, kd) in &combos {
            let mut goal = start;
            goal[j] += d;
            segments.push(Segment { goal, ramp: 0.0, hold: secs, label: format!("{j}:{kp}:{kd}") });
            segments.push(Segment { goal: start, ramp: 0.0, hold: secs, label: String::new() });
            plan.push((j, kp, kd, t));
            t += trial;
        }
    }
    let traj = Trajectory { start, segments };

    let controller = Controller::new(hand, gains).mode(ControlMode::Position).timing(timing);
    let mut mit = gains.mit;
    let (j0, kp0, kd0, _) = plan[0];
    mit[j0] = MitParam { kp: kp0, kd: kd0 };
    let (handle, join) = controller.start().await?;
    hand.set_mit_params(&mit).await?;
    let player = tokio::spawn(play(handle.clone(), traj.clone()));

    let t0 = Instant::now();
    let mut samples: Vec<Telemetry> = Vec::with_capacity(60_000);
    let mut next = 1usize;
    while t0.elapsed().as_secs_f32() < traj.total_secs() + 0.2 && !handle.is_stopped() {
        if next < plan.len() && t0.elapsed().as_secs_f32() >= plan[next].3 - 0.25 {
            let (j, kp, kd, _) = plan[next];
            mit = gains.mit;
            mit[j] = MitParam { kp, kd };
            let _ = hand.set_mit_params(&mit).await;
            next += 1;
        }
        if let Some(t) = handle.next_telemetry().await {
            samples.push(t);
        }
    }
    let _ = player.await;
    handle.stop();
    let _ = join.await;
    controller.disarm().await?;
    hand.set_mit_params(&gains.mit).await?;

    let mut report = TuneReport { joints: BTreeMap::new(), recommended: BTreeMap::new() };
    for &j in joints {
        let p0 = start[j];
        let mut rows = Vec::new();
        for &(pj, kp, kd, ts) in &plan {
            if pj != j {
                continue;
            }
            let w: Vec<&Telemetry> =
                samples.iter().filter(|s| s.t.as_secs_f32() >= ts && s.t.as_secs_f32() < ts + secs && s.position[j].is_finite()).collect();
            if w.len() < 50 {
                continue;
            }
            let target = traj.segments.iter().find(|s| s.label == format!("{j}:{kp}:{kd}")).map(|s| s.goal[j] - p0).unwrap_or(step);
            let norm = |p: f32| (p - p0) / target;
            let t_at = |level: f32| w.iter().find(|s| norm(s.position[j]) >= level).map(|s| (s.t.as_secs_f32() - ts) * 1000.0);
            let rise_ms = match (t_at(0.1), t_at(0.9)) {
                (Some(a), Some(b)) => Some(b - a),
                _ => None,
            };
            let settle_ms =
                match w.iter().rev().find(|s| (norm(s.position[j]) - 1.0).abs() > 0.05).map(|s| (s.t.as_secs_f32() - ts) * 1000.0) {
                    Some(x) if x < secs * 1000.0 - 50.0 => Some(x),
                    Some(_) => None,
                    None => Some(0.0),
                };
            let peak = w.iter().map(|s| norm(s.position[j])).fold(f32::MIN, f32::max);
            let tail: Vec<&&Telemetry> = w.iter().filter(|s| s.t.as_secs_f32() >= ts + secs * 0.5).collect();
            rows.push(StepMetrics {
                kp,
                kd,
                rise_ms,
                settle_ms,
                overshoot_pct: ((peak - 1.0).max(0.0) * 100.0).min(999.0),
                steady_error: tail.last().map(|s| (s.position[j] - (p0 + target)) * target.signum()).unwrap_or(f32::NAN),
                current_rms: rms(&tail.iter().map(|s| s.current[j]).collect::<Vec<_>>()),
                velocity_rms: rms(&tail.iter().map(|s| s.velocity[j]).collect::<Vec<_>>()),
                max_current: w.iter().map(|s| s.current[j].abs()).fold(0.0, f32::max),
            });
        }
        let mut candidates: Vec<&StepMetrics> = rows
            .iter()
            .filter(|r| {
                r.settle_ms.is_some()
                    && r.overshoot_pct <= criteria.max_overshoot_pct
                    && r.current_rms <= criteria.max_current_rms
                    && r.velocity_rms <= criteria.max_velocity_rms
            })
            .collect();
        candidates.sort_by(|a, b| a.steady_error.abs().partial_cmp(&b.steady_error.abs()).unwrap());
        if let Some(best) = candidates.first() {
            report.recommended.insert(j, (best.kp, best.kd));
        }
        report.joints.insert(j, rows);
    }
    Ok(report)
}

// --------------------------------------------------------------------- preload

#[derive(Debug, Clone)]
pub struct BiasReport {
    /// Per joint: (velocity rms, current rms) for bias 0, +amps, −amps.
    pub stats: [[(f32, f32); 3]; NUM_JOINTS],
    /// Joints that jitter without preload and are quiet with the given sign.
    pub recommended: BTreeMap<usize, f32>,
    /// Jittering joints that no sign could quiet.
    pub unresolved: Vec<usize>,
}

/// Hold all joints for `secs` with feed-forward effort 0, +amps and −amps and compare the
/// hold ripple. Joints with gear backlash limit-cycle around the target; a small constant
/// effort pushes the gear train against one flank and stops the cycle.
pub async fn bias_test(hand: &Hand, gains: Gains, timing: Timing, amps: f32, secs: f32) -> Result<BiasReport> {
    const QUIET_V: f32 = 0.008;
    const QUIET_I: f32 = 0.02;
    let start = hand.positions().await?;
    let mut runs = Vec::new();
    for b in [0.0f32, amps, -amps] {
        let mut g = gains;
        g.effort_bias = [b; NUM_JOINTS];
        let controller = Controller::new(hand, g).mode(ControlMode::Position).timing(timing);
        let (handle, join) = controller.start().await?;
        let t0 = Instant::now();
        let mut v: Vec<Vec<f32>> = vec![Vec::new(); NUM_JOINTS];
        let mut c: Vec<Vec<f32>> = vec![Vec::new(); NUM_JOINTS];
        while t0.elapsed().as_secs_f32() < secs {
            if let Some(t) = handle.next_telemetry().await {
                if t0.elapsed().as_secs_f32() >= secs * 0.5 {
                    for i in 0..NUM_JOINTS {
                        v[i].push(t.velocity[i]);
                        c[i].push(t.current[i]);
                    }
                }
            }
        }
        handle.stop();
        let _ = join.await;
        controller.disarm().await?;
        runs.push(std::array::from_fn::<(f32, f32), NUM_JOINTS, _>(|i| (rms(&v[i]), rms(&c[i]))));
    }
    let _ = start;
    let noisy = |(v, c): (f32, f32)| v > QUIET_V || c > QUIET_I;
    let mut report = BiasReport {
        stats: std::array::from_fn(|i| [runs[0][i], runs[1][i], runs[2][i]]),
        recommended: BTreeMap::new(),
        unresolved: Vec::new(),
    };
    for i in 0..NUM_JOINTS {
        if !noisy(runs[0][i]) {
            continue;
        }
        let score = |(v, c): (f32, f32)| v + 10.0 * c;
        let mut quiet: Vec<(f32, f32)> =
            [(amps, runs[1][i]), (-amps, runs[2][i])].into_iter().filter(|(_, s)| !noisy(*s)).map(|(b, s)| (b, score(s))).collect();
        quiet.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        match quiet.first() {
            Some((b, _)) => {
                report.recommended.insert(i, *b);
            }
            None => report.unresolved.push(i),
        }
    }
    Ok(report)
}

/// Full procedure: self-test, then tuning of joints without a profile entry (or all with
/// `retune`), then the preload test. Writes into `profile`; returns the self-test report
/// and whether the hand passed.
pub struct CheckupOptions {
    pub timing: Timing,
    pub kps: Vec<f32>,
    pub kds: Vec<f32>,
    pub step: f32,
    pub step_secs: f32,
    pub retune: bool,
    pub skip_tune: bool,
    pub skip_bias: bool,
    pub bias_amps: f32,
    pub criteria: TuneCriteria,
}

impl Default for CheckupOptions {
    fn default() -> Self {
        CheckupOptions {
            timing: Timing::default(),
            kps: vec![2.0, 3.0, 4.0, 6.0],
            kds: vec![0.15, 0.3],
            step: 0.12,
            step_secs: 0.8,
            retune: false,
            skip_tune: false,
            skip_bias: false,
            bias_amps: 0.05,
            criteria: TuneCriteria::default(),
        }
    }
}

pub struct CheckupReport {
    pub self_check: SelfCheckReport,
    pub tune: Option<TuneReport>,
    pub bias: Option<BiasReport>,
}

pub async fn checkup(
    hand: &Hand,
    profile: &mut HandProfile,
    base: Gains,
    opts: &CheckupOptions,
    mut progress: impl FnMut(&str),
) -> Result<CheckupReport> {
    progress("self-test");
    let self_check = self_check(hand, profile.gains(base), opts.timing, 0.08, 1.3).await?;
    if !self_check.all_ok() {
        return Ok(CheckupReport { self_check, tune: None, bias: None });
    }
    let mut tune_report = None;
    if !opts.skip_tune {
        let joints: Vec<usize> =
            (0..NUM_JOINTS).filter(|i| opts.retune || profile.joint(JointId::from_index(*i).unwrap()).is_none()).collect();
        if !joints.is_empty() {
            progress(&format!("tuning {} joint(s)", joints.len()));
            let r = tune(hand, profile.gains(base), opts.timing, &joints, &opts.kps, &opts.kds, opts.step, opts.step_secs, opts.criteria)
                .await?;
            for (j, (kp, kd)) in &r.recommended {
                let e = profile.joint_mut(JointId::from_index(*j)?);
                e.kp = *kp;
                e.kd = *kd;
            }
            profile.note(format!("tune step {} secs {}: {} recommendation(s)", opts.step, opts.step_secs, r.recommended.len()));
            tune_report = Some(r);
        }
    }
    let mut bias_report = None;
    if !opts.skip_bias {
        progress("preload test");
        let r = bias_test(hand, profile.gains(base), opts.timing, opts.bias_amps, 3.0).await?;
        for (j, b) in &r.recommended {
            profile.joint_mut(JointId::from_index(*j)?).effort_bias = Some(*b);
        }
        profile.note(format!("preload test {} A: {} joint(s) biased", opts.bias_amps, r.recommended.len()));
        bias_report = Some(r);
    }
    Ok(CheckupReport { self_check, tune: tune_report, bias: bias_report })
}
