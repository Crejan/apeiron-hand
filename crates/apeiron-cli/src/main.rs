//! `apeiron` – commission, calibrate, debug and drive a Wuji Hand 2 from the command line.
#![allow(clippy::needless_range_loop, clippy::too_many_arguments, clippy::print_literal, clippy::write_literal)]

mod pcap;
mod render;

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use apeiron_driver::commissioning::{self, CheckupOptions, TuneCriteria};
use apeiron_driver::control::{ControlMode, Controller, Limits, Timing, TrajectorySource};
use apeiron_driver::hand::liveliness;
use apeiron_driver::motion::{self, Pose, Segment, Trajectory};
use apeiron_driver::profile::{Gains, HandProfile, DEFAULT_EFFORT_LIMIT, DEFAULT_KD, DEFAULT_KP};
use apeiron_driver::protocol::capability::access;
use apeiron_driver::protocol::joint::{clamp_to_limits, JointId, NUM_JOINTS};
use apeiron_driver::protocol::keys::rel;
use apeiron_driver::protocol::{error_codes, messages::*};
use apeiron_driver::{Endpoint, Error, Hand, Result};
use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "apeiron", version, about = "Open tooling for the Wuji Hand 2")]
struct Cli {
    /// Zenoh endpoint of the hand
    #[arg(long, global = true, default_value = "udp/192.168.1.111:7447")]
    endpoint: String,
    /// Serial number (skips liveliness discovery)
    #[arg(long, global = true)]
    serial: Option<String>,
    #[arg(long, global = true)]
    poses_dir: Option<PathBuf>,
    #[arg(long, global = true)]
    profiles_dir: Option<PathBuf>,
    /// Required by every command that enables the motors
    #[arg(long, global = true)]
    allow_motion: bool,
    /// Print the trajectory instead of driving it
    #[arg(long, global = true)]
    dry_run: bool,
    /// Current limit written before enabling, A
    #[arg(long, global = true, default_value_t = DEFAULT_EFFORT_LIMIT)]
    max_effort: f32,
    /// Maximum ramp speed, rad/s
    #[arg(long, global = true, default_value_t = 1.0)]
    max_speed: f32,
    /// Command rate, Hz (200..=1000)
    #[arg(long, global = true, default_value_t = 200)]
    rate: u32,
    /// SCHED_FIFO priority of the sender thread (Linux)
    #[arg(long, global = true, default_value_t = 0)]
    rt_prio: i32,
    /// Pin the sender thread to a CPU
    #[arg(long, global = true)]
    cpu: Option<usize>,
    /// Gains `KP:KD` for all joints (ignores the profile gains)
    #[arg(long, global = true, value_name = "KP:KD")]
    gains: Option<String>,
    /// Ignore the hand profile
    #[arg(long, global = true)]
    no_profile: bool,
    /// Per-joint current limit, e.g. `index_S2=0`
    #[arg(long, global = true, value_name = "JOINT=A")]
    effort_override: Vec<String>,
    /// Per-joint feed-forward preload, e.g. `index_S4=-0.05`
    #[arg(long, global = true, value_name = "JOINT=A")]
    bias: Vec<String>,
    /// Leave the motors enabled after the motion
    #[arg(long, global = true)]
    keep_enabled: bool,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Hands announcing themselves on the endpoint, plus their declarations
    Scan,
    /// Firmware, voltage, gains, limits and current joint positions
    Status,
    /// Live joint table (or JSON lines with --json)
    Telemetry {
        #[arg(long, default_value_t = 100)]
        interval_ms: u64,
        #[arg(long)]
        once: bool,
        #[arg(long)]
        json: bool,
        #[arg(long)]
        imu: bool,
        #[arg(long, default_value_t = 0)]
        secs: u64,
    },
    /// Record all streams as JSON lines
    Record {
        #[arg(long, default_value_t = 5)]
        secs: u64,
        #[arg(long, default_value = "record.jsonl")]
        out: String,
    },
    /// Device self-description: summary, resources, nodes
    Capabilities {
        #[arg(long)]
        resources: bool,
        #[arg(long)]
        nodes: bool,
    },
    /// Raw GET of a resource (hex and, where known, decoded)
    Get {
        path: String,
    },
    /// SET with a hex payload
    Set {
        path: String,
        hex: String,
    },
    /// Action (SET with empty payload), e.g. clear_fault
    Action {
        path: String,
    },
    Enable,
    Disable,
    ClearFault,
    /// Hold the current position
    Hold {
        #[arg(long, default_value_t = 3.0)]
        secs: f32,
    },
    /// Move joints: `index_S1=0.5`, `4=20deg`, `all=0`
    Move {
        targets: Vec<String>,
        #[arg(long, default_value_t = 2.0)]
        secs: f32,
        #[arg(long, default_value_t = 1.0)]
        hold: f32,
    },
    /// Drive to a pose from the pose directory (or a file)
    Pose {
        name: String,
        #[arg(long)]
        secs: Option<f32>,
        #[arg(long)]
        hold: Option<f32>,
    },
    /// Play a sequence of poses
    Sequence {
        name: String,
        /// Repetitions (0 = until Ctrl-C)
        #[arg(long, name = "loop")]
        loops: Option<u32>,
    },
    /// List poses and sequences
    Poses,
    /// Save the current joint positions as a pose
    SavePose {
        name: String,
        #[arg(long, default_value = "")]
        description: String,
    },
    /// Deflect every joint a little and verify motion, encoders and crosstalk
    Selfcheck {
        #[arg(long, default_value_t = 0.08)]
        delta: f32,
        #[arg(long, default_value_t = 1.3)]
        secs_per_joint: f32,
    },
    /// Step responses over a kp/kd grid with a recommendation per joint
    Tune {
        #[arg(long = "joint")]
        joints: Vec<String>,
        #[arg(long, default_value = "1.5,2,3,4,6,8")]
        kp_list: String,
        #[arg(long, default_value = "0.1,0.2,0.3")]
        kd_list: String,
        #[arg(long, default_value_t = 0.15)]
        step: f32,
        #[arg(long, default_value_t = 1.0)]
        secs: f32,
        #[arg(long)]
        save_profile: bool,
    },
    /// Preload test: hold with effort 0, +A, −A and detect joints that need a bias
    Bias {
        #[arg(long, default_value_t = 0.05)]
        amps: f32,
        #[arg(long, default_value_t = 3.0)]
        secs: f32,
        #[arg(long)]
        no_save: bool,
    },
    /// Self-test, tuning of untuned joints and preload test; writes the profile
    Checkup {
        #[arg(long)]
        retune: bool,
        #[arg(long)]
        skip_tune: bool,
        #[arg(long)]
        skip_bias: bool,
        #[arg(long, default_value = "2,3,4,6")]
        kp_list: String,
        #[arg(long, default_value = "0.15,0.3")]
        kd_list: String,
    },
    /// Show the hand's IP address, or set a new one (takes effect after `--reboot`)
    Ip {
        address: Option<String>,
        /// Reboot the hand after setting the address
        #[arg(long)]
        reboot: bool,
    },
    /// Show or edit the hand profile
    Profile {
        joint: Option<String>,
        #[arg(long)]
        kp: Option<f32>,
        #[arg(long)]
        kd: Option<f32>,
        /// Per-joint current limit in A (0 removes it)
        #[arg(long)]
        effort_limit: Option<f32>,
        /// Feed-forward preload in A (negative values allowed, 0 removes it)
        #[arg(long, allow_negative_numbers = true)]
        effort_bias: Option<f32>,
    },
    /// Decode a pcap/pcapng capture of hand traffic
    Pcap(pcap::PcapArgs),
}

fn parse_list(l: &str) -> Result<Vec<f32>> {
    l.split(',').map(|x| x.trim().parse::<f32>().map_err(|_| Error::Other(format!("bad list '{l}'")))).collect()
}

fn parse_angle(s: &str) -> Result<f32> {
    let s = s.trim();
    let (num, deg) = match s.strip_suffix("deg") {
        Some(n) => (n, true),
        None => (s.strip_suffix("rad").unwrap_or(s), false),
    };
    let v: f32 = num.trim().parse().map_err(|_| Error::Other(format!("bad angle '{s}'")))?;
    Ok(if deg { v.to_radians() } else { v })
}

fn parse_overrides(list: &[String]) -> Result<Vec<(usize, f32)>> {
    list.iter()
        .map(|o| {
            let (j, v) = o.split_once('=').ok_or_else(|| Error::Other(format!("expected JOINT=VALUE, got '{o}'")))?;
            Ok((JointId::parse(j)?.index(), v.parse().map_err(|_| Error::Other(format!("bad value in '{o}'")))?))
        })
        .collect()
}

fn needs_motion(cmd: &Cmd) -> bool {
    matches!(
        cmd,
        Cmd::Hold { .. }
            | Cmd::Move { .. }
            | Cmd::Pose { .. }
            | Cmd::Sequence { .. }
            | Cmd::Selfcheck { .. }
            | Cmd::Tune { .. }
            | Cmd::Bias { .. }
            | Cmd::Checkup { .. }
            | Cmd::Enable
    )
}

struct Ctx {
    cli: Cli,
    poses_dir: PathBuf,
    profiles_dir: PathBuf,
}

impl Ctx {
    fn timing(&self) -> Timing {
        Timing { rate_hz: self.cli.rate, rt_priority: self.cli.rt_prio, cpu: self.cli.cpu }
    }

    fn limits(&self) -> Limits {
        Limits { max_effort: self.cli.max_effort, ..Limits::default() }
    }

    /// Gains: CLI overrides > profile > defaults.
    fn gains(&self, hand: &Hand) -> Result<(Gains, String)> {
        let cli_gains = match &self.cli.gains {
            Some(s) => {
                let (kp, kd) = s.split_once(':').ok_or_else(|| Error::Other(format!("--gains expects KP:KD, got '{s}'")))?;
                Some((
                    kp.parse::<f32>().map_err(|_| Error::Other(format!("bad kp '{kp}'")))?,
                    kd.parse::<f32>().map_err(|_| Error::Other(format!("bad kd '{kd}'")))?,
                ))
            }
            None => None,
        };
        let (kp, kd) = cli_gains.unwrap_or((DEFAULT_KP, DEFAULT_KD));
        let base = Gains::uniform(kp, kd, self.cli.max_effort);
        let (mut g, mut desc) = if self.cli.no_profile || cli_gains.is_some() {
            (base, format!("kp {} / kd {}", base.mit[0].kp, base.mit[0].kd))
        } else {
            match HandProfile::load(&HandProfile::path_in(&self.profiles_dir, hand.serial())) {
                Ok(p) => (p.gains(base), format!("profile {} ({} joints, {})", p.serial, p.joints.len(), p.created)),
                Err(_) => (base, format!("defaults kp {} / kd {}", base.mit[0].kp, base.mit[0].kd)),
            }
        };
        for (i, a) in parse_overrides(&self.cli.effort_override)? {
            g.effort_limit[i] = a;
            desc.push_str(&format!(", effort {}={a}", JointId::from_index(i)?.label()));
        }
        for (i, b) in parse_overrides(&self.cli.bias)? {
            g.effort_bias[i] = b;
            desc.push_str(&format!(", bias {}={b:+}", JointId::from_index(i)?.label()));
        }
        Ok((g, desc))
    }

    fn controller<'a>(&self, hand: &'a Hand) -> Result<Controller<'a>> {
        let (gains, desc) = self.gains(hand)?;
        println!("gains: {desc}");
        let mut c = Controller::new(hand, gains).mode(ControlMode::Position).timing(self.timing()).limits(self.limits());
        c.keep_enabled = self.cli.keep_enabled;
        Ok(c)
    }

    async fn drive(&self, hand: &Hand, mut traj: Trajectory, endless: bool) -> Result<()> {
        traj.limit_speed(self.cli.max_speed);
        render::trajectory(&traj);
        if self.cli.dry_run {
            return Ok(());
        }
        let controller = self.controller(hand)?;
        let stats = controller.run(TrajectorySource { trajectory: traj, velocity_feedforward: true, endless }).await?;
        println!(
            "{} frames in {:.2} s, wake-up lateness mean {:.0} µs / p99 {:.0} µs / max {:.0} µs, overruns {}",
            stats.sent,
            stats.elapsed.as_secs_f64(),
            stats.late_mean_us,
            stats.late_p99_us,
            stats.late_max_us,
            stats.overruns
        );
        println!("reached:");
        render::positions(&hand.positions().await?);
        Ok(())
    }
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt().with_env_filter(tracing_subscriber::EnvFilter::from_default_env()).with_writer(std::io::stderr).init();
    let cli = Cli::parse();
    let ctx = Ctx { poses_dir: motion::poses_dir(cli.poses_dir.as_deref()), profiles_dir: profiles_dir(cli.profiles_dir.as_deref()), cli };
    if let Err(e) = run(ctx).await {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn profiles_dir(explicit: Option<&std::path::Path>) -> PathBuf {
    if let Some(p) = explicit {
        return p.to_path_buf();
    }
    if let Ok(p) = std::env::var("APEIRON_PROFILES") {
        return PathBuf::from(p);
    }
    ["profiles", "../profiles"].iter().map(PathBuf::from).find(|p| p.is_dir()).unwrap_or_else(|| PathBuf::from("profiles"))
}

async fn run(ctx: Ctx) -> Result<()> {
    // Commands that need no connection.
    match &ctx.cli.cmd {
        Cmd::Pcap(args) => return pcap::run(args.clone()),
        Cmd::Poses => {
            for p in motion::list_poses(&ctx.poses_dir)? {
                println!("{:<12} {}", p.name, p.description);
            }
            for s in motion::list_sequences(&ctx.poses_dir) {
                println!("{:<12} {} ({} steps, loop {})", s.name, s.description, s.steps.len(), s.loops);
            }
            return Ok(());
        }
        Cmd::Profile { .. } if ctx.cli.serial.is_some() => return profile_cmd(&ctx, ctx.cli.serial.clone().unwrap()),
        _ if ctx.cli.dry_run && needs_motion(&ctx.cli.cmd) => {
            let traj = build_trajectory(&ctx, &[0.0; NUM_JOINTS])?;
            return ctx.drive_dry(traj);
        }
        _ => {}
    }

    let endpoint = Endpoint::parse(&ctx.cli.endpoint)?;
    if let Cmd::Scan = ctx.cli.cmd {
        return scan(&endpoint).await;
    }
    let hand = Hand::connect(&endpoint, ctx.cli.serial.clone()).await?;
    println!("hand {} at {}", hand.serial(), endpoint);
    if needs_motion(&ctx.cli.cmd) && !ctx.cli.allow_motion {
        hand.close().await?;
        return Err(Error::Other("this command moves the hand; add --allow-motion (hand free, nothing in the workspace)".into()));
    }
    let result = dispatch(&ctx, &hand).await;
    hand.close().await?;
    result
}

impl Ctx {
    fn drive_dry(&self, mut traj: Trajectory) -> Result<()> {
        traj.limit_speed(self.cli.max_speed);
        render::trajectory(&traj);
        Ok(())
    }
}

async fn scan(endpoint: &Endpoint) -> Result<()> {
    let session = apeiron_driver::transport::open(endpoint).await?;
    let serials = liveliness(&session, Duration::from_secs(3)).await?;
    if serials.is_empty() {
        println!("no hand announced itself within 3 s");
    }
    for sn in &serials {
        println!("hand {sn} ({})", if sn.chars().nth(3) == Some('K') { "right" } else { "left" });
    }
    let replies = session.get("@/*/router/**").timeout(Duration::from_secs(2)).await?;
    while let Ok(reply) = replies.recv_async().await {
        if let Ok(sample) = reply.result() {
            let key = sample.key_expr().to_string();
            if let Some(kind) = key.split("/router/").nth(1) {
                if kind.starts_with("subscriber/") || kind.starts_with("queryable/") || kind.starts_with("publisher/") {
                    println!("  {kind}");
                }
            }
        }
    }
    session.close().await?;
    Ok(())
}

async fn dispatch(ctx: &Ctx, hand: &Hand) -> Result<()> {
    match &ctx.cli.cmd {
        Cmd::Scan | Cmd::Poses | Cmd::Pcap(_) => unreachable!(),
        Cmd::Status => {
            println!("hw_version    {}", hand.hw_version().await?);
            println!("handedness    {}", hand.handedness().await?);
            println!("input_voltage {:.2} V", hand.input_voltage().await?);
            let mit = hand.mit_params().await?;
            println!("kp            {:?}", mit.iter().map(|m| m.kp).collect::<Vec<_>>());
            println!("kd            {:?}", mit.iter().map(|m| m.kd).collect::<Vec<_>>());
            println!("effort_limit  {:?} A", hand.effort_limit().await?);
            println!("control_mode  {:?}", hand.control_mode().await?);
            let frozen = commissioning::frozen_encoders(hand).await;
            if !frozen.is_empty() {
                println!(
                    "WARNING frozen joint encoder (enc2 = 0): {} – power-cycle the hand",
                    frozen.iter().map(|j| j.label()).collect::<Vec<_>>().join(", ")
                );
            }
            println!("positions:");
            let snap = hand.snapshot().await?;
            render::positions(&snap.positions());
            let errors: Vec<String> = (0..NUM_JOINTS)
                .filter_map(|i| {
                    snap.joints[i]
                        .filter(|j| j.error_code_current != 0)
                        .map(|j| format!("{}={}", JointId::from_index(i).unwrap().label(), error_codes::name(j.error_code_current)))
                })
                .collect();
            if !errors.is_empty() {
                println!("error codes: {}", errors.join(", "));
            }
        }
        Cmd::Telemetry { interval_ms, once, json, imu, secs } => render::telemetry(hand, *interval_ms, *once, *json, *imu, *secs).await?,
        Cmd::Record { secs, out } => record(hand, *secs, out).await?,
        Cmd::Capabilities { resources, nodes } => {
            let s = hand.capability_summary().await?;
            println!("device_id {} serial {} firmware {} uptime {} s", s.device_id, s.serial_number, s.firmware_version, s.uptime_s);
            if *nodes {
                for n in hand.capability_nodes().await? {
                    println!("node {:>2} type {} {}", n.node_id, n.type_id, n.name);
                }
            }
            if *resources {
                println!("{:>6} {:>5} {:<6} {:>6} {}", "id", "scope", "access", "schema", "name");
                for (scope, r) in hand.capability_resources().await? {
                    let a = [(access::GET, 'G'), (access::SET, 'S'), (access::SUB, 'U'), (access::PUB, 'P'), (access::ACTION, 'A')]
                        .iter()
                        .map(|(f, c)| if r.can(*f) { *c } else { '-' })
                        .collect::<String>();
                    println!("{:>6} {:>5} {:<6} {:>6} {}", r.resource_id, scope, a, r.schema_id, r.name);
                }
            }
        }
        Cmd::Get { path } => {
            let b = hand.get_raw(path).await?;
            println!("{} bytes: {}", b.len(), apeiron_driver::hand::hex(&b));
            if let Some(text) = render::decode_known(path, &b) {
                println!("{text}");
            }
        }
        Cmd::Set { path, hex } => {
            let bytes = (0..hex.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).map_err(|_| Error::Other("bad hex".into())))
                .collect::<Result<Vec<u8>>>()?;
            render::set_reply(path, &hand.set(path, bytes).await?);
        }
        Cmd::Action { path } => render::set_reply(path, &hand.action(path).await?),
        Cmd::Ip { address, reboot } => {
            let current: String = hand.get("ip_address").await?;
            let port: u32 = hand.get("data_port").await?;
            println!("ip_address {current}  data_port {port}");
            if let Some(a) = address {
                a.parse::<std::net::Ipv4Addr>().map_err(|_| Error::Other(format!("'{a}' is not an IPv4 address")))?;
                render::set_reply("ip_address", &hand.set("ip_address", apeiron_driver::protocol::encode(a)?).await?);
                println!("new address stored; it becomes active after a reboot – reconnect with --endpoint udp/{a}:7447");
                if *reboot {
                    match hand.action("reboot").await {
                        Ok(r) => render::set_reply("reboot", &r),
                        Err(e) => println!("reboot: {e} (the hand may already be restarting)"),
                    }
                }
            }
        }
        Cmd::Enable => render::set_reply("motor_enable", &hand.motor_enable().await?),
        Cmd::Disable => render::set_reply("motor_disable", &hand.motor_disable().await?),
        Cmd::ClearFault => render::set_reply("clear_fault", &hand.clear_fault().await?),
        Cmd::SavePose { name, description } => {
            let pos = hand.positions().await?;
            let pose = Pose::from_positions(name, description, &pos);
            std::fs::create_dir_all(&ctx.poses_dir)?;
            let path = ctx.poses_dir.join(format!("{name}.json"));
            pose.save(&path)?;
            println!("saved {}", path.display());
            render::positions(&pos);
        }
        Cmd::Profile { .. } => profile_cmd(ctx, hand.serial().to_string())?,
        Cmd::Hold { .. } | Cmd::Move { .. } | Cmd::Pose { .. } | Cmd::Sequence { .. } => {
            let start = hand.positions().await?;
            println!("start:");
            render::positions(&start);
            let traj = build_trajectory(ctx, &start)?;
            let endless = match &ctx.cli.cmd {
                Cmd::Sequence { name, loops } => {
                    loops.unwrap_or_else(|| motion::find_sequence(&ctx.poses_dir, name).map(|s| s.loops).unwrap_or(1)) == 0
                }
                _ => false,
            };
            ctx.drive(hand, traj, endless).await?;
        }
        Cmd::Selfcheck { delta, secs_per_joint } => {
            let (gains, _) = ctx.gains(hand)?;
            let report = commissioning::self_check(hand, gains, ctx.timing(), *delta, *secs_per_joint).await?;
            render::self_check(&report);
            if !report.all_ok() {
                std::process::exit(2);
            }
        }
        Cmd::Tune { joints, kp_list, kd_list, step, secs, save_profile } => {
            let js: Vec<usize> = if joints.is_empty() {
                vec![4]
            } else {
                joints.iter().map(|j| JointId::parse(j).map(|j| j.index())).collect::<std::result::Result<_, _>>()?
            };
            let (gains, _) = ctx.gains(hand)?;
            let report = commissioning::tune(
                hand,
                gains,
                ctx.timing(),
                &js,
                &parse_list(kp_list)?,
                &parse_list(kd_list)?,
                *step,
                *secs,
                TuneCriteria::default(),
            )
            .await?;
            render::tune(&report);
            if *save_profile {
                let mut profile = HandProfile::load_or_new(&ctx.profiles_dir, hand.serial());
                for (j, (kp, kd)) in &report.recommended {
                    let e = profile.joint_mut(JointId::from_index(*j)?);
                    e.kp = *kp;
                    e.kd = *kd;
                }
                profile.note(format!("tune step {step} secs {secs}"));
                println!("profile saved: {}", profile.save(&ctx.profiles_dir)?.display());
            }
        }
        Cmd::Bias { amps, secs, no_save } => {
            let (gains, _) = ctx.gains(hand)?;
            let report = commissioning::bias_test(hand, gains, ctx.timing(), *amps, *secs).await?;
            render::bias(&report, *amps);
            if !*no_save && !report.recommended.is_empty() {
                let mut profile = HandProfile::load_or_new(&ctx.profiles_dir, hand.serial());
                for (j, b) in &report.recommended {
                    profile.joint_mut(JointId::from_index(*j)?).effort_bias = Some(*b);
                }
                profile.note(format!("preload test {amps} A"));
                println!("profile saved: {}", profile.save(&ctx.profiles_dir)?.display());
            }
        }
        Cmd::Checkup { retune, skip_tune, skip_bias, kp_list, kd_list } => {
            let mut profile = HandProfile::load_or_new(&ctx.profiles_dir, hand.serial());
            let opts = CheckupOptions {
                timing: ctx.timing(),
                kps: parse_list(kp_list)?,
                kds: parse_list(kd_list)?,
                retune: *retune,
                skip_tune: *skip_tune,
                skip_bias: *skip_bias,
                ..Default::default()
            };
            let base = Gains::uniform(DEFAULT_KP, DEFAULT_KD, ctx.cli.max_effort);
            let report = commissioning::checkup(hand, &mut profile, base, &opts, |step| println!("== {step}")).await?;
            render::self_check(&report.self_check);
            if !report.self_check.all_ok() {
                println!("self-test failed – tuning skipped");
                std::process::exit(2);
            }
            if let Some(t) = &report.tune {
                render::tune(t);
            }
            if let Some(b) = &report.bias {
                render::bias(b, opts.bias_amps);
            }
            println!("profile saved: {}", profile.save(&ctx.profiles_dir)?.display());
        }
    }
    Ok(())
}

fn profile_cmd(ctx: &Ctx, serial: String) -> Result<()> {
    let Cmd::Profile { joint, kp, kd, effort_limit, effort_bias } = &ctx.cli.cmd else { unreachable!() };
    let mut profile = HandProfile::load_or_new(&ctx.profiles_dir, &serial);
    if let Some(j) = joint {
        let id = JointId::parse(j)?;
        let e = profile.joint_mut(id);
        if let Some(v) = kp {
            e.kp = *v;
        }
        if let Some(v) = kd {
            e.kd = *v;
        }
        if let Some(v) = effort_limit {
            e.max_effort = (*v > 0.0).then_some(*v);
        }
        if let Some(v) = effort_bias {
            e.effort_bias = (*v != 0.0).then_some(*v);
        }
        profile.note(format!("profile set {}", id.label()));
        println!("saved {}", profile.save(&ctx.profiles_dir)?.display());
    }
    println!("profile {} ({})", profile.serial, profile.created);
    for (label, e) in &profile.joints {
        println!(
            "  {:<10} kp {:<5} kd {:<5}{}{}",
            label,
            e.kp,
            e.kd,
            e.max_effort.map(|v| format!(" max_effort {v}")).unwrap_or_default(),
            e.effort_bias.map(|v| format!(" effort_bias {v:+}")).unwrap_or_default()
        );
    }
    Ok(())
}

fn build_trajectory(ctx: &Ctx, start: &[f32; NUM_JOINTS]) -> Result<Trajectory> {
    let mut segments = Vec::new();
    match &ctx.cli.cmd {
        Cmd::Hold { secs } => segments.push(Segment { goal: *start, ramp: 0.0, hold: *secs, label: "hold".into() }),
        Cmd::Move { targets, secs, hold } => {
            let mut goal = *start;
            for t in targets {
                let (lhs, rhs) = t.split_once('=').ok_or_else(|| Error::Other(format!("target '{t}' is not JOINT=ANGLE")))?;
                let angle = parse_angle(rhs)?;
                if lhs.eq_ignore_ascii_case("all") {
                    goal = [angle; NUM_JOINTS];
                } else {
                    goal[JointId::parse(lhs)?.index()] = angle;
                }
            }
            for (i, g) in goal.iter_mut().enumerate() {
                let (c, clamped) = clamp_to_limits(i, *g)?;
                if clamped {
                    println!("note: {} clamped to {c:+.3} rad", JointId::from_index(i)?.label());
                    *g = c;
                }
            }
            segments.push(Segment { goal, ramp: *secs, hold: *hold, label: "move".into() });
        }
        Cmd::Pose { name, secs, hold } => {
            let pose = motion::find_pose(&ctx.poses_dir, name)?;
            segments.push(segment_from_pose(&pose, start, *secs, *hold)?);
        }
        Cmd::Sequence { name, loops } => {
            let seq = motion::find_sequence(&ctx.poses_dir, name)?;
            let reps = loops.unwrap_or(seq.loops).max(1);
            let mut from = *start;
            for _ in 0..reps {
                for step in &seq.steps {
                    let pose = motion::find_pose(&ctx.poses_dir, &step.pose)?;
                    let seg = segment_from_pose(&pose, &from, step.secs, step.hold)?;
                    from = seg.goal;
                    segments.push(seg);
                }
            }
        }
        _ => return Err(Error::Other("not a motion command".into())),
    }
    Ok(Trajectory { start: *start, segments })
}

fn segment_from_pose(pose: &Pose, from: &[f32; NUM_JOINTS], secs: Option<f32>, hold: Option<f32>) -> Result<Segment> {
    let (goal, clamped) = pose.resolve(from)?;
    for (i, wanted, got) in clamped {
        println!("note: {} clamped to {got:+.3} rad (pose asked for {wanted:+.3})", JointId::from_index(i)?.label());
    }
    Ok(Segment { goal, ramp: secs.unwrap_or(pose.secs), hold: hold.unwrap_or(pose.hold), label: pose.name.clone() })
}

async fn record(hand: &Hand, secs: u64, out: &str) -> Result<()> {
    use std::io::Write;
    let mut file: Box<dyn Write> =
        if out == "-" { Box::new(std::io::stdout()) } else { Box::new(std::io::BufWriter::new(std::fs::File::create(out)?)) };
    let sub = hand.session().declare_subscriber(hand.key("**")).await?;
    let t0 = Instant::now();
    let deadline = t0 + Duration::from_secs(secs);
    let mut stats: BTreeMap<String, (u64, u64)> = BTreeMap::new();
    while Instant::now() < deadline {
        let Ok(Ok(sample)) = tokio::time::timeout(deadline - Instant::now(), sub.recv_async()).await else { break };
        let key = sample.key_expr().to_string();
        let relpath = apeiron_driver::protocol::keys::rel_from_key(&key).unwrap_or("").to_string();
        let bytes = sample.payload().to_bytes();
        let decoded = match relpath.as_str() {
            rel::REALTIME_STREAM => {
                apeiron_driver::protocol::decode::<RealtimeStreamFrame>(&bytes).ok().map(|f| serde_json::to_value(f).unwrap())
            }
            rel::IMU => apeiron_driver::protocol::decode::<ImuData>(&bytes).ok().map(|f| serde_json::to_value(f).unwrap()),
            _ => None,
        };
        let e = stats.entry(relpath).or_default();
        e.0 += 1;
        e.1 += bytes.len() as u64;
        let line = serde_json::json!({ "t": t0.elapsed().as_secs_f64(), "key": key, "len": bytes.len(), "decoded": decoded, "hex": apeiron_driver::hand::hex(&bytes[..bytes.len().min(64)]) });
        writeln!(file, "{line}")?;
    }
    file.flush()?;
    let secs = t0.elapsed().as_secs_f64();
    for (k, (n, b)) in stats {
        eprintln!("{k:<24} {n:>7} frames {:>7.1} Hz {b:>10} bytes", n as f64 / secs);
    }
    Ok(())
}
