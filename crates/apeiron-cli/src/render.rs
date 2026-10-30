//! Terminal output.

use std::io::Write;
use std::time::{Duration, Instant};

use apeiron_driver::commissioning::{BiasReport, SelfCheckReport, TuneReport};
use apeiron_driver::hand::JointSnapshot;
use apeiron_driver::motion::Trajectory;
use apeiron_driver::protocol::joint::{Finger, JointId, NUM_JOINTS};
use apeiron_driver::protocol::keys::rel;
use apeiron_driver::protocol::messages::*;
use apeiron_driver::protocol::{decode, error_codes};
use apeiron_driver::{Hand, Result};

pub fn positions(pos: &[f32; NUM_JOINTS]) {
    for f in Finger::ALL {
        let b = f as usize * 4;
        println!("  {:<7} S1 {:+.3}  S2 {:+.3}  S3 {:+.3}  S4 {:+.3}  rad", f.name(), pos[b], pos[b + 1], pos[b + 2], pos[b + 3]);
    }
}

pub fn trajectory(traj: &Trajectory) {
    println!("trajectory: {} segment(s), {:.1} s", traj.segments.len(), traj.total_secs());
    let mut t = 0f32;
    for seg in &traj.segments {
        println!("  t={t:6.1}s  {:<14} ramp {:.2} s, hold {:.2} s", seg.label, seg.ramp, seg.hold);
        positions(&seg.goal);
        t += seg.ramp + seg.hold;
    }
}

pub fn set_reply(what: &str, r: &SetReply) {
    if r.all_ok() {
        println!("{what}: ok ({}/{} joints)", r.ok, r.total);
    } else {
        println!("{what}: {} failure(s): {:?}", r.failures().len(), r.failures());
    }
}

/// Decode well-known GET payloads for `apeiron get`.
pub fn decode_known(path: &str, b: &[u8]) -> Option<String> {
    let leaf = path.rsplit('/').next().unwrap_or(path);
    match leaf {
        rel::HW_VERSION => decode::<HwVersion>(b).ok().map(|v| format!("hw_version {v}")),
        rel::MIT_PARAMS => decode::<Vec<MitParam>>(b).ok().map(|v| format!("{v:?}")),
        rel::EFFORT_LIMIT => decode::<Vec<f32>>(b).ok().map(|v| format!("{v:?} A")),
        rel::HANDEDNESS => b.first().map(|v| Handedness::from(*v).to_string()),
        rel::CONTROL_MODE if b.len() > 1 => decode::<Vec<u8>>(b).ok().map(|v| format!("{v:?}")),
        _ if b.len() == 4 => {
            Some(format!("f32 {} / u32 {}", f32::from_le_bytes([b[0], b[1], b[2], b[3]]), u32::from_le_bytes([b[0], b[1], b[2], b[3]])))
        }
        _ if b.len() == 1 => Some(format!("u8 {}", b[0])),
        _ if b.len() == 2 => Some(format!("u16 {}", u16::from_le_bytes([b[0], b[1]]))),
        _ => std::str::from_utf8(b).ok().filter(|s| s.starts_with('{')).map(str::to_string),
    }
}

pub async fn telemetry(hand: &Hand, interval_ms: u64, once: bool, json: bool, show_imu: bool, secs: u64) -> Result<()> {
    let hw = hand.hw_version().await.map(|v| v.to_string()).unwrap_or_else(|_| "?".into());
    let handed = hand.handedness().await.map(|h| h.to_string()).unwrap_or_else(|_| "?".into());
    let rt = hand.realtime_stream().await?;
    let imu = hand.imu().await?;
    let t0 = Instant::now();
    let mut snap: Option<JointSnapshot> = None;
    let mut last_imu: Option<ImuData> = None;
    let (mut frames, mut imu_frames, mut gaps, mut last_seq) = (0u64, 0u64, 0u64, None::<u32>);
    let mut out = std::io::stdout();
    let mut next_render = Instant::now();
    let (mut frames_at, mut imu_at, mut t_at) = (0u64, 0u64, Instant::now());
    loop {
        tokio::select! {
            r = rt.recv() => if let Ok((_, Ok(f))) = r {
                if let Some(p) = last_seq { gaps += f.header.seq.wrapping_sub(p.wrapping_add(1)) as u64; }
                last_seq = Some(f.header.seq);
                frames += 1;
                let s = JointSnapshot::from(f);
                match &mut snap {
                    None => snap = Some(s),
                    Some(m) => { for (slot, j) in m.joints.iter_mut().zip(s.joints) { if j.is_some() { *slot = j; } } m.seq = s.seq; m.timestamp_us = s.timestamp_us; }
                }
            },
            r = imu.recv() => if let Ok((_, Ok(f))) = r { last_imu = Some(f); imu_frames += 1; },
            _ = tokio::time::sleep_until(next_render.into()) => {
                next_render += Duration::from_millis(interval_ms);
                let dt = t_at.elapsed().as_secs_f64().max(1e-6);
                let rt_hz = (frames - frames_at) as f64 / dt;
                let imu_hz = (imu_frames - imu_at) as f64 / dt;
                frames_at = frames; imu_at = imu_frames; t_at = Instant::now();
                let Some(s) = &snap else { continue };
                if json {
                    let line = serde_json::json!({
                        "t": t0.elapsed().as_secs_f64(), "seq": s.seq, "timestamp_us": s.timestamp_us, "rt_hz": rt_hz, "imu_hz": imu_hz, "seq_gaps": gaps,
                        "joints": (0..NUM_JOINTS).filter_map(|i| s.joints[i].map(|j| serde_json::json!({
                            "label": JointId::from_index(i).unwrap().label(), "nid": j.nid, "position": j.pos_rad_jnt_fb, "velocity": j.vel_rad_s_jnt_fb,
                            "effort": j.effort, "current": j.current, "vbus": j.vbus_v_fb, "temperature": j.mcu_temp_c_fb,
                            "error_code": j.error_code_current, "error": error_codes::name(j.error_code_current), "status": j.status_word }))).collect::<Vec<_>>(),
                        "imu": last_imu.as_ref().map(|m| serde_json::json!({ "q": [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w],
                            "gyro": [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z], "acc": [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] })),
                    });
                    writeln!(out, "{line}")?;
                } else {
                    if !once { write!(out, "\x1b[2J\x1b[H")?; }
                    table(&mut out, hand.serial(), &hw, &handed, s, rt_hz, imu_hz, gaps, show_imu.then_some(last_imu.as_ref()).flatten(), t0.elapsed())?;
                }
                out.flush()?;
                if once { break; }
                if secs > 0 && t0.elapsed() >= Duration::from_secs(secs) { break; }
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn table(
    out: &mut impl Write,
    serial: &str,
    hw: &str,
    handed: &str,
    s: &JointSnapshot,
    rt_hz: f64,
    imu_hz: f64,
    gaps: u64,
    imu: Option<&ImuData>,
    elapsed: Duration,
) -> std::io::Result<()> {
    writeln!(
        out,
        "hand {serial}  hw {hw}  {handed}  t={:.1}s   realtime_stream {rt_hz:6.1} Hz seq {} gaps {gaps}   imu {imu_hz:5.1} Hz",
        elapsed.as_secs_f64(),
        s.seq
    )?;
    writeln!(out)?;
    writeln!(
        out,
        "{:<10} {:>3} {:>8} {:>7} {:>8} {:>7} {:>7} {:>6} {:>5}  {}",
        "joint", "nid", "pos rad", "pos °", "vel r/s", "eff A", "cur A", "Vbus", "°C", "error"
    )?;
    for i in 0..NUM_JOINTS {
        let label = JointId::from_index(i).unwrap().label();
        match s.joints[i] {
            Some(j) => writeln!(
                out,
                "{:<10} {:>3} {:>+8.4} {:>+7.1} {:>+8.3} {:>+7.3} {:>+7.3} {:>6.2} {:>5.1}  {}",
                label,
                j.nid,
                j.pos_rad_jnt_fb,
                j.pos_rad_jnt_fb.to_degrees(),
                j.vel_rad_s_jnt_fb,
                j.effort,
                j.current,
                j.vbus_v_fb,
                j.mcu_temp_c_fb,
                if j.error_code_current == 0 { String::new() } else { error_codes::name(j.error_code_current) }
            )?,
            None => writeln!(out, "{label:<10}   -   (no data)")?,
        }
        if i % 4 == 3 && i + 1 < NUM_JOINTS {
            writeln!(out)?;
        }
    }
    if let Some(m) = imu {
        writeln!(out)?;
        writeln!(
            out,
            "imu q=({:+.3},{:+.3},{:+.3},{:+.3}) gyro=({:+.3},{:+.3},{:+.3}) rad/s acc=({:+.2},{:+.2},{:+.2}) m/s²",
            m.orientation.x,
            m.orientation.y,
            m.orientation.z,
            m.orientation.w,
            m.angular_velocity.x,
            m.angular_velocity.y,
            m.angular_velocity.z,
            m.linear_acceleration.x,
            m.linear_acceleration.y,
            m.linear_acceleration.z
        )?;
    }
    Ok(())
}

pub fn self_check(r: &SelfCheckReport) {
    let yn = |o: Option<bool>| match o {
        Some(true) => "ok",
        Some(false) => "NO",
        None => "?",
    };
    println!(
        "{:<10} {:>6} {:>5} {:>5} {:>5} {:>5} {:>6} {:>6}  {:<22} {}",
        "joint", "result", "moved", "back", "enc1", "enc2", "e1=e2", "I max", "error", "notes"
    );
    for (id, c) in &r.joints {
        let mut notes = c.notes.clone();
        if !c.crosstalk.is_empty() {
            notes.push(format!("moves with: {}", c.crosstalk.iter().map(|j| j.label()).collect::<Vec<_>>().join(",")));
        }
        println!(
            "{:<10} {:>6} {:>5} {:>5} {:>5} {:>5} {:>6} {:>6.3}  {:<22} {}",
            id.label(),
            if c.ok() { "OK" } else { "FAIL" },
            yn(c.moved),
            yn(c.returned),
            yn(c.enc1_follows),
            yn(c.enc2_follows),
            yn(c.encoders_agree),
            c.max_current,
            error_codes::name(c.error_code),
            notes.join("; ")
        );
    }
    let ok = r.joints.iter().filter(|(_, c)| c.ok()).count();
    let drift = r.end.iter().zip(&r.start).map(|(a, b)| (a - b).abs()).fold(0f32, f32::max);
    println!("{ok}/20 joints OK, end-to-start drift {drift:.3} rad");
}

pub fn tune(r: &TuneReport) {
    for (j, rows) in &r.joints {
        println!("\n{}:", JointId::from_index(*j).unwrap().label());
        println!(
            "{:>5} {:>5} {:>8} {:>9} {:>7} {:>9} {:>7} {:>7} {:>6}",
            "kp", "kd", "rise ms", "settle ms", "over %", "err rad", "I rms", "v rms", "I max"
        );
        for m in rows {
            println!(
                "{:>5} {:>5} {:>8} {:>9} {:>7.1} {:>+9.4} {:>7.3} {:>7.3} {:>6.3}",
                m.kp,
                m.kd,
                m.rise_ms.map(|x| format!("{x:.0}")).unwrap_or("-".into()),
                m.settle_ms.map(|x| format!("{x:.0}")).unwrap_or("never".into()),
                m.overshoot_pct,
                m.steady_error,
                m.current_rms,
                m.velocity_rms,
                m.max_current
            );
        }
        match r.recommended.get(j) {
            Some((kp, kd)) => println!("recommended: kp {kp} / kd {kd}"),
            None => println!("no candidate met the criteria"),
        }
    }
}

pub fn bias(r: &BiasReport, amps: f32) {
    println!("{:<10} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}  result", "joint", "v0", "v+", "v-", "I0", "I+", "I-");
    for i in 0..NUM_JOINTS {
        let s = r.stats[i];
        let verdict = match r.recommended.get(&i) {
            Some(b) => format!("jitters without preload → bias {b:+.3} A"),
            None if r.unresolved.contains(&i) => "jitters, no sign is quiet".to_string(),
            None => "quiet".to_string(),
        };
        println!(
            "{:<10} {:>8.4} {:>8.4} {:>8.4} {:>8.3} {:>8.3} {:>8.3}  {verdict}",
            JointId::from_index(i).unwrap().label(),
            s[0].0,
            s[1].0,
            s[2].0,
            s[0].1,
            s[1].1,
            s[2].1
        );
    }
    let _ = amps;
}
