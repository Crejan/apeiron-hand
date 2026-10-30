//! `apeiron-udp-adapter`: expose the hand's control loop as UDP/JSON.

use std::net::SocketAddr;
use std::path::PathBuf;

use apeiron_adapter_udp::{Config, UdpAdapter};
use apeiron_driver::control::{ControlMode, Limits, Timing};
use apeiron_driver::profile::Gains;
use apeiron_driver::{Endpoint, Hand, HandProfile};
use clap::Parser;

#[derive(Parser)]
#[command(version, about = "Drive a Wuji Hand 2 over UDP with newline-delimited JSON")]
struct Args {
    #[arg(long, default_value = "udp/192.168.1.111:7447")]
    endpoint: String,
    /// Serial number; discovered via liveliness when omitted
    #[arg(long)]
    serial: Option<String>,
    /// Address to receive command datagrams on
    #[arg(long, default_value = "0.0.0.0:7700")]
    bind: SocketAddr,
    /// Address to send telemetry datagrams to
    #[arg(long, default_value = "127.0.0.1:7701")]
    telemetry_to: SocketAddr,
    /// Forward every n-th telemetry frame (1 = 1 kHz)
    #[arg(long, default_value_t = 10)]
    decimation: u32,
    /// Initial control mode: position, velocity, effort or impedance:<kp>:<kd>
    #[arg(long, default_value = "position")]
    mode: String,
    /// Directory with per-hand profiles (profiles/<serial>.json)
    #[arg(long, default_value = "profiles")]
    profiles_dir: PathBuf,
    /// Command rate in Hz (200..=1000)
    #[arg(long, default_value_t = 200)]
    rate: u32,
    /// SCHED_FIFO priority of the sender thread (Linux), 0 = off
    #[arg(long, default_value_t = 0)]
    rt_prio: i32,
    /// Pin the sender thread to this CPU
    #[arg(long)]
    cpu: Option<usize>,
    /// Current limit applied to all joints, A
    #[arg(long, default_value_t = 0.3)]
    max_effort: f32,
}

fn parse_mode(s: &str) -> Result<ControlMode, String> {
    match s {
        "position" => Ok(ControlMode::Position),
        "velocity" => Ok(ControlMode::Velocity),
        "effort" => Ok(ControlMode::Effort),
        _ => {
            let parts: Vec<&str> = s.split(':').collect();
            match parts.as_slice() {
                ["impedance", kp, kd] => Ok(ControlMode::Impedance {
                    kp: kp.parse().map_err(|_| format!("bad kp '{kp}'"))?,
                    kd: kd.parse().map_err(|_| format!("bad kd '{kd}'"))?,
                }),
                _ => Err(format!("unknown mode '{s}' (position | velocity | effort | impedance:<kp>:<kd>)")),
            }
        }
    }
}

#[tokio::main]
async fn main() -> apeiron_driver::Result<()> {
    tracing_subscriber::fmt().with_env_filter(tracing_subscriber::EnvFilter::from_default_env()).with_writer(std::io::stderr).init();
    let args = Args::parse();
    let mode = parse_mode(&args.mode).map_err(apeiron_driver::Error::Other)?;

    let hand = Hand::connect(&Endpoint::parse(&args.endpoint)?, args.serial).await?;
    let profile = HandProfile::load_or_new(&args.profiles_dir, hand.serial());
    let gains = profile.gains(Gains::default());
    let config = Config {
        bind: args.bind,
        telemetry_to: args.telemetry_to,
        decimation: args.decimation,
        mode,
        timing: Timing { rate_hz: args.rate, rt_priority: args.rt_prio, cpu: args.cpu },
        limits: Limits { max_effort: args.max_effort, ..Limits::default() },
    };

    println!(
        "hand {} · commands on udp://{} · telemetry to udp://{} every {} frame(s) · mode {:?} · profile entries {}",
        hand.serial(),
        config.bind,
        config.telemetry_to,
        config.decimation,
        config.mode,
        profile.joints.len()
    );
    println!("example: echo '{{\"position\":[0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0]}}' | nc -u -w0 127.0.0.1 {}   (Ctrl-C disables the motors)", config.bind.port());

    let adapter = UdpAdapter::new(hand, gains, config);
    let stats = adapter
        .run(async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await?;
    println!(
        "stopped: {} frames in {:.1} s, wake-up lateness mean {:.0} µs / p99 {:.0} µs / max {:.0} µs, overruns {}",
        stats.sent,
        stats.elapsed.as_secs_f64(),
        stats.late_mean_us,
        stats.late_p99_us,
        stats.late_max_us,
        stats.overruns
    );
    Ok(())
}
