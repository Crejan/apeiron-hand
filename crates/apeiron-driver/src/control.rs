//! Cyclic control with pluggable middleware.
//!
//! The hand runs a 1 kHz MIT impedance loop per joint:
//! `torque = kp·(q_des − q) + kd·(v_des − v) + effort_ff`, with `kp`/`kd` set through
//! `mit_params` and the output clamped to `effort_limit`. Every control mode offered here is
//! a parameterisation of that law:
//!
//! | mode | kp | kd | command fields used |
//! |---|---|---|---|
//! | [`ControlMode::Position`] | profile | profile | position (+ velocity feed-forward) |
//! | [`ControlMode::Velocity`] | 0 | profile | velocity |
//! | [`ControlMode::Impedance`] | per command | per command… set once | position, velocity, effort |
//! | [`ControlMode::Effort`] | 0 | 0 | effort |
//!
//! A [`Controller`] owns the real-time sender thread. Middleware interacts through a
//! [`ControlHandle`] (push the latest [`Command`], read the latest [`Telemetry`]) or by
//! implementing [`ControlSource`], which is polled once per cycle. The sender always
//! transmits the most recent command at a fixed rate; if the source stops updating, the
//! last command is held and the hand's own watchdog stops the motors once frames cease.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

use apeiron_protocol::encode;
use apeiron_protocol::joint::{clamp_to_limits, NUM_JOINTS};
use apeiron_protocol::keys::rel;
use apeiron_protocol::messages::{JointCommand, JointCommandFrame, MitParam, RealtimeStreamEntry, ScalarCommandFrame};
use zenoh::Wait;

use crate::hand::{Hand, JointSnapshot};
use crate::profile::Gains;
use crate::{Error, Result};

/// Per-joint targets. Interpretation depends on the active [`ControlMode`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Command {
    pub position: [f32; NUM_JOINTS],
    pub velocity: [f32; NUM_JOINTS],
    pub effort: [f32; NUM_JOINTS],
}

impl Command {
    pub fn hold(position: [f32; NUM_JOINTS]) -> Command {
        Command { position, velocity: [0.0; NUM_JOINTS], effort: [0.0; NUM_JOINTS] }
    }

    pub fn zero() -> Command {
        Command::hold([0.0; NUM_JOINTS])
    }
}

/// Latest joint feedback, published once per received stream frame (1 kHz).
#[derive(Debug, Clone)]
pub struct Telemetry {
    /// Host time since the controller started.
    pub t: Duration,
    pub seq: u32,
    pub timestamp_us: u64,
    pub position: [f32; NUM_JOINTS],
    pub velocity: [f32; NUM_JOINTS],
    pub effort: [f32; NUM_JOINTS],
    pub current: [f32; NUM_JOINTS],
    pub vbus: [f32; NUM_JOINTS],
    pub temperature: [f32; NUM_JOINTS],
    pub error_code: [u16; NUM_JOINTS],
}

impl Telemetry {
    fn from_snapshot(t: Duration, s: &JointSnapshot) -> Telemetry {
        let f = |g: fn(&RealtimeStreamEntry) -> f32| std::array::from_fn(|i| s.joints[i].as_ref().map(g).unwrap_or(f32::NAN));
        Telemetry {
            t,
            seq: s.seq,
            timestamp_us: s.timestamp_us,
            position: f(|j| j.pos_rad_jnt_fb),
            velocity: f(|j| j.vel_rad_s_jnt_fb),
            effort: f(|j| j.effort),
            current: f(|j| j.current),
            vbus: f(|j| j.vbus_v_fb),
            temperature: f(|j| j.mcu_temp_c_fb),
            error_code: std::array::from_fn(|i| s.joints[i].as_ref().map(|j| j.error_code_current).unwrap_or(0)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlMode {
    /// Track `position` with the profile gains; `velocity` is used as feed-forward.
    Position,
    /// Track `velocity` only (kp = 0).
    Velocity,
    /// Full MIT law with explicit gains for all joints; the command supplies
    /// position, velocity and feed-forward effort.
    Impedance { kp: f32, kd: f32 },
    /// Open-loop effort (kp = kd = 0).
    Effort,
    /// Firmware-side loops: the joints are switched to `control_mode = mode` and one command
    /// field is streamed as `ScalarCommandFrame` on `cmd/<topic>`. The firmware accepts modes
    /// 0–4; 1 is the MIT law used by all other variants. Observed so far: with `cmd/position`,
    /// modes 0, 3 and 4 do not react and mode 2 drives all joints hard – the mapping between
    /// modes and topics is not established. Use with `--only` and small values. Mode 1 is
    /// restored on disarm.
    Firmware { mode: u8, topic: FirmwareTopic },
}

/// Topic (and command field) used by [`ControlMode::Firmware`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirmwareTopic {
    Position,
    Velocity,
    Torque,
    Current,
}

impl FirmwareTopic {
    fn key(self) -> &'static str {
        match self {
            FirmwareTopic::Position => rel::CMD_POSITION,
            FirmwareTopic::Velocity => rel::CMD_VELOCITY,
            FirmwareTopic::Torque => rel::CMD_TORQUE,
            FirmwareTopic::Current => rel::CMD_CURRENT,
        }
    }
}

impl ControlMode {
    fn gains(self, profile: &Gains) -> [MitParam; NUM_JOINTS] {
        match self {
            ControlMode::Position | ControlMode::Firmware { .. } => profile.mit,
            ControlMode::Velocity => std::array::from_fn(|i| MitParam { kp: 0.0, kd: profile.mit[i].kd }),
            ControlMode::Impedance { kp, kd } => [MitParam { kp, kd }; NUM_JOINTS],
            ControlMode::Effort => [MitParam { kp: 0.0, kd: 0.0 }; NUM_JOINTS],
        }
    }

    /// Firmware `control_mode` value this mode needs on every joint.
    fn firmware_mode(self) -> u8 {
        match self {
            ControlMode::Firmware { mode, .. } => mode,
            _ => MIT_CONTROL_MODE,
        }
    }
}

/// `control_mode` value under which the firmware applies the MIT law to `joint_command`.
pub const MIT_CONTROL_MODE: u8 = 1;

/// Timing options of the sender thread.
#[derive(Debug, Clone, Copy)]
pub struct Timing {
    /// Command rate in Hz (200..=1000).
    pub rate_hz: u32,
    /// SCHED_FIFO priority for the sender thread (Linux), 0 = none.
    pub rt_priority: i32,
    /// Pin the sender thread to this CPU.
    pub cpu: Option<usize>,
}

impl Default for Timing {
    fn default() -> Self {
        Timing { rate_hz: 200, rt_priority: 0, cpu: None }
    }
}

/// Safety envelope applied to every command before it is sent.
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    /// Clamp positions to the mechanical range.
    pub clamp_positions: bool,
    /// Maximum |velocity| target, rad/s.
    pub max_velocity: f32,
    /// Maximum |effort| target, A.
    pub max_effort: f32,
    /// Read both encoders of every joint before enabling and disable joints with a frozen
    /// joint encoder (costs about a second).
    pub check_encoders: bool,
    /// Host-side position correction used with `ControlMode::Firmware` on the velocity topic:
    /// `v_cmd = v_des + tracking_gain · (q_des − q)`, 1/s. 0 disables the correction.
    pub tracking_gain: f32,
}

impl Default for Limits {
    fn default() -> Self {
        Limits { clamp_positions: true, max_velocity: 3.0, max_effort: 0.3, check_encoders: true, tracking_gain: 2.0 }
    }
}

/// Timing statistics of a finished run plus per-joint motion quality.
#[derive(Debug, Clone, Default)]
pub struct RunStats {
    pub sent: u64,
    pub elapsed: Duration,
    pub late_mean_us: f64,
    pub late_p99_us: f64,
    pub late_max_us: f64,
    pub overruns: u64,
    /// Filled by [`Controller::run`]; empty for loops started with [`Controller::start`].
    pub joints: Vec<JointQuality>,
}

/// Motion quality of one joint over a run, from the 1 kHz telemetry.
#[derive(Debug, Clone, Copy, Default)]
pub struct JointQuality {
    /// Samples in which the target velocity exceeded the dead band.
    pub moving_samples: u32,
    /// Mean target velocity while moving, rad/s.
    pub target_velocity: f32,
    /// RMS of (measured − target velocity) while moving, rad/s. Smooth tracking stays well
    /// below the target velocity; stick-slip and gear roughness show up as values of the
    /// same order as the target velocity.
    pub velocity_ripple: f32,
    /// RMS of the current around its mean while moving, A.
    pub current_ripple: f32,
    /// Peak |current| over the whole run, A.
    pub peak_current: f32,
    /// RMS of (measured − target position) while moving, rad. Stick-slip shows up as a
    /// sawtooth of amplitude ≈ breakaway effort / kp.
    pub position_error: f32,
    /// RMS of the measured velocity while the target velocity is zero, rad/s: the sensor
    /// noise floor that `velocity_ripple` cannot fall below.
    pub rest_velocity_noise: f32,
}

/// Shared state between middleware and the sender thread.
struct Shared {
    command: RwLock<Command>,
    telemetry: RwLock<Option<Telemetry>>,
    bias: RwLock<[f32; NUM_JOINTS]>,
    friction_ff: [f32; NUM_JOINTS],
    stop: AtomicBool,
    frames: AtomicU64,
    telemetry_version: AtomicU64,
}

/// Handle for middleware: push commands, read telemetry, stop the loop.
#[derive(Clone)]
pub struct ControlHandle {
    shared: Arc<Shared>,
    notify: Arc<tokio::sync::Notify>,
}

impl ControlHandle {
    /// Replace the command that is transmitted every cycle.
    pub fn set_command(&self, command: Command) {
        *self.shared.command.write().unwrap() = command;
    }

    pub fn command(&self) -> Command {
        *self.shared.command.read().unwrap()
    }

    /// Latest telemetry, `None` until the first frame arrived.
    pub fn telemetry(&self) -> Option<Telemetry> {
        self.shared.telemetry.read().unwrap().clone()
    }

    /// Wait for the next telemetry frame (1 kHz).
    pub async fn next_telemetry(&self) -> Option<Telemetry> {
        let before = self.shared.telemetry_version.load(Ordering::Acquire);
        loop {
            if self.shared.stop.load(Ordering::Acquire) {
                return None;
            }
            if self.shared.telemetry_version.load(Ordering::Acquire) != before {
                return self.telemetry();
            }
            self.notify.notified().await;
        }
    }

    pub fn frames_sent(&self) -> u64 {
        self.shared.frames.load(Ordering::Relaxed)
    }

    pub fn stop(&self) {
        self.shared.stop.store(true, Ordering::SeqCst);
        self.notify.notify_waiters();
    }

    pub fn is_stopped(&self) -> bool {
        self.shared.stop.load(Ordering::Acquire)
    }
}

/// Callback-style source, polled once per command cycle on the sender thread.
/// Keep it fast and allocation-free; it runs at up to 1 kHz.
pub trait ControlSource: Send + 'static {
    /// Produce the command for cycle time `t`; `None` keeps the previous command.
    fn command(&mut self, t: Duration, telemetry: Option<&Telemetry>) -> Option<Command>;
    /// Return `true` to end the run.
    fn finished(&self, _t: Duration) -> bool {
        false
    }
}

impl<F> ControlSource for F
where
    F: FnMut(Duration, Option<&Telemetry>) -> Option<Command> + Send + 'static,
{
    fn command(&mut self, t: Duration, telemetry: Option<&Telemetry>) -> Option<Command> {
        self(t, telemetry)
    }
}

/// Runs the hand: enables the motors, streams commands at a fixed rate, disables on exit.
pub struct Controller<'a> {
    hand: &'a Hand,
    gains: Gains,
    mode: ControlMode,
    timing: Timing,
    limits: Limits,
    /// Leave the motors enabled when the run ends.
    pub keep_enabled: bool,
}

impl<'a> Controller<'a> {
    pub fn new(hand: &'a Hand, gains: Gains) -> Controller<'a> {
        Controller { hand, gains, mode: ControlMode::Position, timing: Timing::default(), limits: Limits::default(), keep_enabled: false }
    }

    pub fn mode(mut self, mode: ControlMode) -> Self {
        self.mode = mode;
        self
    }

    pub fn timing(mut self, timing: Timing) -> Self {
        self.timing = timing;
        self
    }

    pub fn limits(mut self, limits: Limits) -> Self {
        self.limits = limits;
        self
    }

    /// Write gains and current limits for the active mode, clear faults and enable. Joints
    /// whose joint-side encoder is frozen (see [`crate::commissioning::frozen_encoders`]) get a
    /// zero current limit so that the controller cannot push against a phantom target.
    pub async fn arm(&self) -> Result<()> {
        let hand = self.hand;
        let mut limits: Vec<f32> = self.gains.effort_limit.iter().map(|e| e.min(self.limits.max_effort)).collect();
        if self.limits.check_encoders {
            for id in crate::commissioning::frozen_encoders(hand).await {
                tracing::warn!("{}: joint encoder frozen (enc2 = 0) – current limit set to 0; power-cycle the hand", id.label());
                limits[id.index()] = 0.0;
            }
        }
        check(hand.set_effort_limit(&limits).await?, "effort_limit")?;
        check(hand.set_mit_params(&self.mode.gains(&self.gains)).await?, "mit_params")?;
        check(hand.set(rel::CONTROL_MODE, encode(&vec![self.mode.firmware_mode(); NUM_JOINTS])?).await?, "control_mode")?;
        check(hand.clear_fault().await?, "clear_fault")?;
        check(hand.motor_enable().await?, "motor_enable")?;
        Ok(())
    }

    /// Switch the control mode while running (rewrites `mit_params` and `control_mode`).
    /// Switching between a `Firmware` mode and the MIT modes also changes the command topic,
    /// so it only takes effect for loops started afterwards.
    pub async fn switch_mode(&mut self, mode: ControlMode) -> Result<()> {
        self.mode = mode;
        check(self.hand.set_mit_params(&mode.gains(&self.gains)).await?, "mit_params")?;
        check(self.hand.set(rel::CONTROL_MODE, encode(&vec![mode.firmware_mode(); NUM_JOINTS])?).await?, "control_mode")
    }

    /// Disable the motors and put the joints back into MIT mode.
    pub async fn disarm(&self) -> Result<()> {
        check(self.hand.motor_disable().await?, "motor_disable")?;
        if self.mode.firmware_mode() != MIT_CONTROL_MODE {
            check(self.hand.set(rel::CONTROL_MODE, encode(&vec![MIT_CONTROL_MODE; NUM_JOINTS])?).await?, "control_mode")?;
        }
        Ok(())
    }

    /// Run `source` until it reports `finished` or the handle is stopped. Arms before and
    /// disarms after (unless `keep_enabled`). Returns timing statistics.
    pub async fn run<S: ControlSource>(&self, source: S) -> Result<RunStats> {
        let (handle, stats) = self.spawn(Some(Box::new(source))).await?;
        let quality = tokio::spawn(collect_quality(handle.clone()));
        let mut stats = stats.await.map_err(|e| Error::Other(format!("sender thread: {e}")))??;
        handle.stop();
        if let Ok(q) = quality.await {
            stats.joints = q;
        }
        if !self.keep_enabled {
            self.disarm().await?;
        }
        Ok(stats)
    }

    /// Start the loop and return a handle for push-style middleware. The loop runs until
    /// [`ControlHandle::stop`] is called; await the returned join handle for statistics.
    pub async fn start(&self) -> Result<(ControlHandle, tokio::task::JoinHandle<Result<RunStats>>)> {
        self.spawn(None).await
    }

    async fn spawn(&self, source: Option<Box<dyn ControlSource>>) -> Result<(ControlHandle, tokio::task::JoinHandle<Result<RunStats>>)> {
        let start = self.hand.positions().await?;
        self.arm().await?;
        let shared = Arc::new(Shared {
            command: RwLock::new(Command::hold(start)),
            telemetry: RwLock::new(None),
            bias: RwLock::new(self.gains.effort_bias),
            friction_ff: self.gains.friction_ff,
            stop: AtomicBool::new(false),
            frames: AtomicU64::new(0),
            telemetry_version: AtomicU64::new(0),
        });
        let notify = Arc::new(tokio::sync::Notify::new());
        let handle = ControlHandle { shared: shared.clone(), notify: notify.clone() };

        // Telemetry task: stream → shared state.
        let stream = self.hand.realtime_stream().await?;
        let t0 = Instant::now();
        {
            let shared = shared.clone();
            tokio::spawn(async move {
                while !shared.stop.load(Ordering::Acquire) {
                    match tokio::time::timeout(Duration::from_millis(200), stream.recv()).await {
                        Ok(Ok((_, Ok(frame)))) => {
                            let snap = JointSnapshot::from(frame);
                            *shared.telemetry.write().unwrap() = Some(Telemetry::from_snapshot(t0.elapsed(), &snap));
                            shared.telemetry_version.fetch_add(1, Ordering::Release);
                            notify.notify_waiters();
                        }
                        Ok(Err(_)) => break,
                        _ => {}
                    }
                }
            });
        }

        let firmware = match self.mode {
            ControlMode::Firmware { topic, .. } => Some(topic),
            _ => None,
        };
        let sender = Sender {
            session: self.hand.session().clone(),
            key: self.hand.key(firmware.map(FirmwareTopic::key).unwrap_or(rel::JOINT_COMMAND)),
            firmware,
            shared,
            source: Mutex::new(source),
            timing: self.timing,
            limits: self.limits,
            t0,
        };
        let join = tokio::task::spawn_blocking(move || sender.run());
        Ok((handle, join))
    }
}

/// Accumulate [`JointQuality`] from telemetry until the handle stops.
async fn collect_quality(handle: ControlHandle) -> Vec<JointQuality> {
    const DEAD_BAND: f32 = 0.02;
    let mut n = [0u32; NUM_JOINTS];
    let mut sum_v = [0f64; NUM_JOINTS];
    let mut sum_e2 = [0f64; NUM_JOINTS];
    let mut sum_i = [0f64; NUM_JOINTS];
    let mut sum_i2 = [0f64; NUM_JOINTS];
    let mut sum_q2 = [0f64; NUM_JOINTS];
    let mut n_rest = [0u32; NUM_JOINTS];
    let mut sum_rest2 = [0f64; NUM_JOINTS];
    let mut peak = [0f32; NUM_JOINTS];
    while let Some(t) = handle.next_telemetry().await {
        let cmd = handle.command();
        for i in 0..NUM_JOINTS {
            peak[i] = peak[i].max(t.current[i].abs());
            if !t.velocity[i].is_finite() {
                continue;
            }
            if cmd.velocity[i].abs() > DEAD_BAND {
                n[i] += 1;
                sum_v[i] += cmd.velocity[i].abs() as f64;
                sum_e2[i] += ((t.velocity[i] - cmd.velocity[i]) as f64).powi(2);
                sum_q2[i] += ((t.position[i] - cmd.position[i]) as f64).powi(2);
                sum_i[i] += t.current[i] as f64;
                sum_i2[i] += (t.current[i] as f64).powi(2);
            } else if cmd.velocity[i] == 0.0 {
                n_rest[i] += 1;
                sum_rest2[i] += (t.velocity[i] as f64).powi(2);
            }
        }
    }
    (0..NUM_JOINTS)
        .map(|i| {
            let k = n[i].max(1) as f64;
            let mean_i = sum_i[i] / k;
            JointQuality {
                moving_samples: n[i],
                target_velocity: (sum_v[i] / k) as f32,
                velocity_ripple: (sum_e2[i] / k).sqrt() as f32,
                current_ripple: (sum_i2[i] / k - mean_i * mean_i).max(0.0).sqrt() as f32,
                peak_current: peak[i],
                position_error: (sum_q2[i] / k).sqrt() as f32,
                rest_velocity_noise: (sum_rest2[i] / n_rest[i].max(1) as f64).sqrt() as f32,
            }
        })
        .collect()
}

fn check(reply: apeiron_protocol::messages::SetReply, what: &str) -> Result<()> {
    if reply.all_ok() {
        Ok(())
    } else {
        Err(Error::Other(format!("{what}: {} joint(s) failed: {:?}", reply.failures().len(), reply.failures())))
    }
}

struct Sender {
    session: zenoh::Session,
    key: String,
    /// Publish one command field as `ScalarCommandFrame` on `cmd/<topic>` instead of MIT frames.
    firmware: Option<FirmwareTopic>,
    shared: Arc<Shared>,
    source: Mutex<Option<Box<dyn ControlSource>>>,
    timing: Timing,
    limits: Limits,
    t0: Instant,
}

impl Sender {
    fn run(self) -> Result<RunStats> {
        for note in rt::setup(self.timing.rt_priority, self.timing.cpu) {
            tracing::warn!("{note}");
        }
        // express: send immediately; RealTime priority; drop instead of block under congestion.
        let publisher = self
            .session
            .declare_publisher(self.key.clone())
            .express(true)
            .priority(zenoh::qos::Priority::RealTime)
            .congestion_control(zenoh::qos::CongestionControl::Drop)
            .wait()?;
        let rate = self.timing.rate_hz.clamp(200, 1000) as i64;
        let period_ns = 1_000_000_000 / rate;
        let mut late = Vec::with_capacity(60_000);
        let mut overruns = 0u64;
        let mut seq: u32 = 0;
        let mut source = self.source.lock().unwrap().take();
        let mut next = rt::now_ns();
        loop {
            if self.shared.stop.load(Ordering::Acquire) {
                break;
            }
            let t = self.t0.elapsed();
            if let Some(src) = source.as_mut() {
                if src.finished(t) {
                    break;
                }
                let telemetry = self.shared.telemetry.read().unwrap().clone();
                if let Some(cmd) = src.command(t, telemetry.as_ref()) {
                    *self.shared.command.write().unwrap() = cmd;
                }
            }
            let cmd = *self.shared.command.read().unwrap();
            let bias = *self.shared.bias.read().unwrap();
            let joints: [JointCommand; NUM_JOINTS] = std::array::from_fn(|i| {
                let position = if self.limits.clamp_positions {
                    clamp_to_limits(i, cmd.position[i]).map(|c| c.0).unwrap_or(cmd.position[i])
                } else {
                    cmd.position[i]
                };
                let velocity = cmd.velocity[i].clamp(-self.limits.max_velocity, self.limits.max_velocity);
                // Coulomb friction feed-forward follows the sign of the target velocity; a small
                // dead band keeps it off while holding.
                let friction = if velocity.abs() > 0.02 { self.shared.friction_ff[i] * velocity.signum() } else { 0.0 };
                JointCommand {
                    position,
                    velocity,
                    effort: (cmd.effort[i] + bias[i] + friction).clamp(-self.limits.max_effort, self.limits.max_effort),
                }
            });
            seq = seq.wrapping_add(1);
            let bytes = match self.firmware {
                Some(FirmwareTopic::Velocity) => {
                    // Firmware velocity loop plus a soft position correction from telemetry.
                    let q = self.shared.telemetry.read().unwrap().as_ref().map(|t| t.position);
                    let v: Vec<f32> = joints
                        .iter()
                        .enumerate()
                        .map(|(i, j)| {
                            let corr = match q {
                                Some(q) if q[i].is_finite() => self.limits.tracking_gain * (j.position - q[i]),
                                _ => 0.0,
                            };
                            (j.velocity + corr).clamp(-self.limits.max_velocity, self.limits.max_velocity)
                        })
                        .collect();
                    encode(&ScalarCommandFrame { seq, joints: v })?
                }
                Some(topic) => {
                    let field = |j: &JointCommand| match topic {
                        FirmwareTopic::Position => j.position,
                        FirmwareTopic::Torque | FirmwareTopic::Current => j.effort,
                        FirmwareTopic::Velocity => unreachable!(),
                    };
                    encode(&ScalarCommandFrame { seq, joints: joints.iter().map(field).collect() })?
                }
                None => encode(&JointCommandFrame::new(seq, joints))?,
            };
            publisher.put(bytes).wait()?;
            self.shared.frames.fetch_add(1, Ordering::Relaxed);
            next += period_ns;
            let now = rt::now_ns();
            if now > next + period_ns {
                overruns += 1;
                next = now;
            }
            rt::sleep_until_ns(next);
            late.push((rt::now_ns() - next) as f64 * 1e-3);
        }
        self.shared.stop.store(true, Ordering::SeqCst);
        let mut sorted = late.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p99 = sorted.get(sorted.len().saturating_sub(1).min((sorted.len() as f64 * 0.99) as usize)).copied().unwrap_or(0.0);
        Ok(RunStats {
            sent: seq as u64,
            elapsed: self.t0.elapsed(),
            late_mean_us: if late.is_empty() { 0.0 } else { late.iter().sum::<f64>() / late.len() as f64 },
            late_p99_us: p99,
            late_max_us: late.iter().cloned().fold(0.0, f64::max),
            overruns,
            joints: Vec::new(),
        })
    }
}

/// A [`ControlSource`] that plays a [`crate::motion::Trajectory`] with velocity feed-forward.
pub struct TrajectorySource {
    pub trajectory: crate::motion::Trajectory,
    pub velocity_feedforward: bool,
    pub endless: bool,
}

impl ControlSource for TrajectorySource {
    fn command(&mut self, t: Duration, _: Option<&Telemetry>) -> Option<Command> {
        let total = self.trajectory.total_secs();
        let mut t = t.as_secs_f32();
        if self.endless && total > 0.0 {
            t %= total;
        }
        Some(Command {
            position: self.trajectory.at(t),
            velocity: if self.velocity_feedforward { self.trajectory.velocity_at(t) } else { [0.0; NUM_JOINTS] },
            effort: [0.0; NUM_JOINTS],
        })
    }

    fn finished(&self, t: Duration) -> bool {
        !self.endless && t.as_secs_f32() > self.trajectory.total_secs()
    }
}

/// Real-time helpers (absolute sleeps, scheduling).
pub mod rt {
    #[cfg(target_os = "linux")]
    pub fn setup(rt_priority: i32, cpu: Option<usize>) -> Vec<String> {
        let mut notes = Vec::new();
        unsafe {
            if let Some(c) = cpu {
                let mut set: libc::cpu_set_t = std::mem::zeroed();
                libc::CPU_SET(c, &mut set);
                if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) != 0 {
                    notes.push(format!("cpu affinity {c}: {}", std::io::Error::last_os_error()));
                }
            }
            if rt_priority > 0 {
                let param = libc::sched_param { sched_priority: rt_priority };
                if libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) != 0 {
                    notes.push(format!("SCHED_FIFO {rt_priority}: {} (check ulimit -r)", std::io::Error::last_os_error()));
                }
                if libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) != 0 {
                    notes.push(format!("mlockall: {}", std::io::Error::last_os_error()));
                }
            }
        }
        notes
    }

    // `time_t`/`c_long` are i64 on 64-bit Linux, so these casts look redundant there but are
    // needed on 32-bit targets.
    #[cfg(target_os = "linux")]
    #[allow(clippy::unnecessary_cast)]
    pub fn now_ns() -> i64 {
        let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        ts.tv_sec as i64 * 1_000_000_000 + ts.tv_nsec as i64
    }

    #[cfg(target_os = "linux")]
    #[allow(clippy::unnecessary_cast)]
    pub fn sleep_until_ns(t: i64) {
        let ts = libc::timespec { tv_sec: (t / 1_000_000_000) as libc::time_t, tv_nsec: (t % 1_000_000_000) as libc::c_long };
        unsafe { while libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &ts, std::ptr::null_mut()) == libc::EINTR {} }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn setup(rt_priority: i32, cpu: Option<usize>) -> Vec<String> {
        if rt_priority > 0 || cpu.is_some() {
            vec!["real-time scheduling and CPU pinning are only available on Linux".into()]
        } else {
            Vec::new()
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn now_ns() -> i64 {
        use std::sync::OnceLock;
        static T0: OnceLock<std::time::Instant> = OnceLock::new();
        T0.get_or_init(std::time::Instant::now).elapsed().as_nanos() as i64
    }

    #[cfg(not(target_os = "linux"))]
    pub fn sleep_until_ns(t: i64) {
        let now = now_ns();
        if t > now {
            std::thread::sleep(std::time::Duration::from_nanos((t - now) as u64));
        }
    }
}
