//! UDP/JSON adapter for the control loop.
//!
//! A reference for docking an external middleware onto [`apeiron_driver::control`]: the
//! adapter owns the connection and the controller, forwards telemetry as JSON datagrams and
//! accepts JSON command datagrams. Calibration is deliberately out of scope; only the cyclic
//! control interface is exposed.
//!
//! Command datagram (any subset of fields; arrays have 20 entries in flat joint order):
//!
//! ```json
//! {"position":[...],"velocity":[...],"effort":[...]}
//! {"mode":"position"}            {"mode":"velocity"}
//! {"mode":"impedance","kp":3.0,"kd":0.2}   {"mode":"effort"}
//! {"stop":true}
//! ```
//!
//! Telemetry datagram (every `decimation`-th frame of the 1 kHz stream):
//!
//! ```json
//! {"t":1.234,"seq":42,"position":[...],"velocity":[...],"effort":[...],
//!  "current":[...],"error_code":[...],"error_name":[...]}
//! ```

use std::future::Future;
use std::net::SocketAddr;

use apeiron_driver::control::{Command, ControlHandle, ControlMode, Controller, Limits, RunStats, Telemetry, Timing};
use apeiron_driver::profile::Gains;
use apeiron_driver::{Error, Hand, Result};
use apeiron_protocol::error_codes;
use apeiron_protocol::joint::NUM_JOINTS;
use serde::{Deserialize, Serialize};
use tokio::net::UdpSocket;

#[derive(Debug, Clone)]
pub struct Config {
    /// Where command datagrams are received.
    pub bind: SocketAddr,
    /// Where telemetry datagrams are sent.
    pub telemetry_to: SocketAddr,
    /// Forward every n-th telemetry frame (1 = 1 kHz, 10 = 100 Hz).
    pub decimation: u32,
    pub mode: ControlMode,
    pub timing: Timing,
    pub limits: Limits,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            bind: "0.0.0.0:7700".parse().unwrap(),
            telemetry_to: "127.0.0.1:7701".parse().unwrap(),
            decimation: 10,
            mode: ControlMode::Position,
            timing: Timing::default(),
            limits: Limits::default(),
        }
    }
}

/// Incoming datagram. Unknown fields are ignored.
#[derive(Debug, Default, Deserialize)]
pub struct CommandMessage {
    pub position: Option<Vec<f32>>,
    pub velocity: Option<Vec<f32>>,
    pub effort: Option<Vec<f32>>,
    pub mode: Option<String>,
    pub kp: Option<f32>,
    pub kd: Option<f32>,
    #[serde(default)]
    pub stop: bool,
}

/// Outgoing datagram.
#[derive(Debug, Serialize)]
pub struct TelemetryMessage<'a> {
    pub t: f64,
    pub seq: u32,
    pub position: &'a [f32],
    pub velocity: &'a [f32],
    pub effort: &'a [f32],
    pub current: &'a [f32],
    pub error_code: &'a [u16],
    pub error_name: Vec<String>,
}

impl<'a> TelemetryMessage<'a> {
    pub fn from_telemetry(t: &'a Telemetry) -> Self {
        TelemetryMessage {
            t: t.t.as_secs_f64(),
            seq: t.seq,
            position: &t.position,
            velocity: &t.velocity,
            effort: &t.effort,
            current: &t.current,
            error_code: &t.error_code,
            error_name: t.error_code.iter().map(|c| error_codes::name(*c)).collect(),
        }
    }
}

/// Parse a mode request such as `{"mode":"impedance","kp":3,"kd":0.2}`.
pub fn parse_mode(msg: &CommandMessage) -> Result<Option<ControlMode>> {
    let Some(mode) = msg.mode.as_deref() else { return Ok(None) };
    let mode = match mode {
        "position" => ControlMode::Position,
        "velocity" => ControlMode::Velocity,
        "effort" => ControlMode::Effort,
        "impedance" => match (msg.kp, msg.kd) {
            (Some(kp), Some(kd)) if kp.is_finite() && kd.is_finite() && kp >= 0.0 && kd >= 0.0 => ControlMode::Impedance { kp, kd },
            _ => return Err(Error::Other("impedance mode needs finite, non-negative kp and kd".into())),
        },
        other => return Err(Error::Other(format!("unknown mode '{other}'"))),
    };
    Ok(Some(mode))
}

/// Merge the message's target arrays into `current`. Fails on wrong length or NaN.
pub fn apply_targets(msg: &CommandMessage, current: &mut Command) -> Result<bool> {
    let mut changed = false;
    for (field, name, target) in [
        (&msg.position, "position", &mut current.position),
        (&msg.velocity, "velocity", &mut current.velocity),
        (&msg.effort, "effort", &mut current.effort),
    ] {
        if let Some(values) = field {
            if values.len() != NUM_JOINTS {
                return Err(Error::JointCount { expected: NUM_JOINTS, got: values.len() });
            }
            if values.iter().any(|v| !v.is_finite()) {
                return Err(Error::Other(format!("{name} contains a non-finite value")));
            }
            target.copy_from_slice(values);
            changed = true;
        }
    }
    Ok(changed)
}

pub struct UdpAdapter {
    hand: Hand,
    gains: Gains,
    config: Config,
}

impl UdpAdapter {
    pub fn new(hand: Hand, gains: Gains, config: Config) -> UdpAdapter {
        UdpAdapter { hand, gains, config }
    }

    pub fn hand(&self) -> &Hand {
        &self.hand
    }

    /// Arm the hand and serve until `{"stop":true}` arrives or `shutdown` completes.
    /// The motors are disabled before returning.
    pub async fn run(self, shutdown: impl Future<Output = ()>) -> Result<RunStats> {
        let socket = UdpSocket::bind(self.config.bind).await?;
        let mut controller =
            Controller::new(&self.hand, self.gains).mode(self.config.mode).timing(self.config.timing).limits(self.config.limits);
        let (handle, join) = controller.start().await?;
        tracing::info!(bind = %self.config.bind, telemetry_to = %self.config.telemetry_to, mode = ?self.config.mode, "adapter running");

        let telemetry = tokio::spawn(forward_telemetry(handle.clone(), self.config.telemetry_to, self.config.decimation.max(1)));

        let mut buf = vec![0u8; 16 * 1024];
        tokio::pin!(shutdown);
        let result: Result<()> = loop {
            tokio::select! {
                _ = &mut shutdown => break Ok(()),
                recv = socket.recv_from(&mut buf) => {
                    let (n, from) = match recv {
                        Ok(r) => r,
                        Err(e) => break Err(e.into()),
                    };
                    match handle_datagram(&buf[..n], &handle, &mut controller).await {
                        Ok(Action::Continue) => {}
                        Ok(Action::Stop) => break Ok(()),
                        Err(e) => tracing::warn!(%from, "rejected command: {e}"),
                    }
                }
            }
        };

        handle.stop();
        telemetry.abort();
        let stats = join.await.map_err(|e| Error::Other(format!("sender thread: {e}")))?;
        controller.disarm().await?;
        result?;
        stats
    }
}

enum Action {
    Continue,
    Stop,
}

async fn handle_datagram(bytes: &[u8], handle: &ControlHandle, controller: &mut Controller<'_>) -> Result<Action> {
    // Several newline-delimited messages per datagram are allowed.
    for line in bytes.split(|b| *b == b'\n').filter(|l| !l.iter().all(u8::is_ascii_whitespace)) {
        let msg: CommandMessage = serde_json::from_slice(line)?;
        if msg.stop {
            return Ok(Action::Stop);
        }
        if let Some(mode) = parse_mode(&msg)? {
            controller.switch_mode(mode).await?;
            tracing::info!(?mode, "mode switched");
        }
        let mut cmd = handle.command();
        if apply_targets(&msg, &mut cmd)? {
            handle.set_command(cmd);
        }
    }
    Ok(Action::Continue)
}

async fn forward_telemetry(handle: ControlHandle, to: SocketAddr, decimation: u32) {
    let Ok(socket) = UdpSocket::bind(("0.0.0.0", 0)).await else { return };
    let mut n = 0u32;
    while let Some(t) = handle.next_telemetry().await {
        n += 1;
        if n % decimation != 0 {
            continue;
        }
        if let Ok(json) = serde_json::to_vec(&TelemetryMessage::from_telemetry(&t)) {
            let _ = socket.send_to(&json, to).await;
        }
    }
}
