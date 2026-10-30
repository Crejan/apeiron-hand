//! Poses, sequences and smooth trajectories.
//!
//! A pose file (`poses/<name>.json`) holds four values per finger (S1..S4, rad); `null`
//! keeps the joint where it is. A sequence file (`poses/sequences/<name>.json`) lists poses
//! with ramp and hold times and an optional loop count (0 = endless).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use apeiron_protocol::joint::{clamp_to_limits, Finger, JointId, JOINTS_PER_FINGER, NUM_JOINTS};
use serde::{Deserialize, Serialize};

use crate::{Error, Result};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub name: String,
    #[serde(default)]
    pub description: String,
    pub joints: BTreeMap<String, [Option<f32>; JOINTS_PER_FINGER]>,
    #[serde(default = "default_secs")]
    pub secs: f32,
    #[serde(default = "default_hold")]
    pub hold: f32,
}

fn default_secs() -> f32 {
    1.5
}

fn default_hold() -> f32 {
    1.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SequenceStep {
    pub pose: String,
    pub secs: Option<f32>,
    pub hold: Option<f32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Sequence {
    pub name: String,
    #[serde(default)]
    pub description: String,
    /// Repetitions, 0 = endless.
    #[serde(default = "default_loop", rename = "loop")]
    pub loops: u32,
    pub steps: Vec<SequenceStep>,
}

fn default_loop() -> u32 {
    1
}

/// A clamped joint: (index, requested, applied).
pub type Clamped = (usize, f32, f32);

impl Pose {
    /// Resolve against `current` (for `null` entries), clamping to the joint limits.
    pub fn resolve(&self, current: &[f32; NUM_JOINTS]) -> Result<([f32; NUM_JOINTS], Vec<Clamped>)> {
        let mut goal = *current;
        let mut clamped = Vec::new();
        for (finger, values) in &self.joints {
            let f = Finger::from_name(finger).ok_or_else(|| Error::Other(format!("pose {}: unknown finger {finger}", self.name)))?;
            for (s, v) in values.iter().enumerate() {
                if let Some(v) = v {
                    let i = f as usize * JOINTS_PER_FINGER + s;
                    let (c, was) = clamp_to_limits(i, *v)?;
                    if was {
                        clamped.push((i, *v, c));
                    }
                    goal[i] = c;
                }
            }
        }
        Ok((goal, clamped))
    }

    pub fn from_positions(name: &str, description: &str, pos: &[f32; NUM_JOINTS]) -> Pose {
        let round = |v: f32| Some((v * 1000.0).round() / 1000.0);
        let joints = Finger::ALL
            .iter()
            .map(|f| {
                let b = *f as usize * JOINTS_PER_FINGER;
                (f.name().to_string(), [round(pos[b]), round(pos[b + 1]), round(pos[b + 2]), round(pos[b + 3])])
            })
            .collect();
        Pose { name: name.into(), description: description.into(), joints, secs: default_secs(), hold: default_hold() }
    }

    pub fn load(path: &Path) -> Result<Pose> {
        Ok(serde_json::from_str(&std::fs::read_to_string(path)?)?)
    }

    pub fn save(&self, path: &Path) -> Result<()> {
        Ok(std::fs::write(path, serde_json::to_string_pretty(self)? + "\n")?)
    }
}

impl Sequence {
    pub fn load(path: &Path) -> Result<Sequence> {
        Ok(serde_json::from_str(&std::fs::read_to_string(path)?)?)
    }
}

/// Pose directory: explicit, `$APEIRON_POSES`, `./poses`, `../poses`.
pub fn poses_dir(explicit: Option<&Path>) -> PathBuf {
    if let Some(p) = explicit {
        return p.to_path_buf();
    }
    if let Ok(p) = std::env::var("APEIRON_POSES") {
        return PathBuf::from(p);
    }
    ["poses", "../poses"].iter().map(PathBuf::from).find(|p| p.is_dir()).unwrap_or_else(|| PathBuf::from("poses"))
}

/// `name` is a path to a pose file or a pose name inside `dir`.
pub fn find_pose(dir: &Path, name: &str) -> Result<Pose> {
    if Path::new(name).is_file() {
        return Pose::load(Path::new(name));
    }
    let p = dir.join(format!("{name}.json"));
    if p.is_file() {
        return Pose::load(&p);
    }
    Err(Error::Other(format!("pose '{name}' not found ({})", p.display())))
}

pub fn find_sequence(dir: &Path, name: &str) -> Result<Sequence> {
    if Path::new(name).is_file() {
        return Sequence::load(Path::new(name));
    }
    for p in [dir.join("sequences").join(format!("{name}.json")), dir.join(format!("{name}.json"))] {
        if p.is_file() {
            return Sequence::load(&p);
        }
    }
    Err(Error::Other(format!("sequence '{name}' not found in {}", dir.join("sequences").display())))
}

pub fn list_poses(dir: &Path) -> Result<Vec<Pose>> {
    let mut out = Vec::new();
    for entry in std::fs::read_dir(dir)?.flatten() {
        let p = entry.path();
        if p.extension().is_some_and(|e| e == "json") && p.is_file() {
            if let Ok(pose) = Pose::load(&p) {
                out.push(pose);
            }
        }
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(out)
}

pub fn list_sequences(dir: &Path) -> Vec<Sequence> {
    let mut out = Vec::new();
    if let Ok(rd) = std::fs::read_dir(dir.join("sequences")) {
        for entry in rd.flatten() {
            if let Ok(s) = Sequence::load(&entry.path()) {
                out.push(s);
            }
        }
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    out
}

/// Smoothstep ramp to `goal` over `ramp` seconds, then hold for `hold` seconds.
#[derive(Debug, Clone)]
pub struct Segment {
    pub goal: [f32; NUM_JOINTS],
    pub ramp: f32,
    pub hold: f32,
    pub label: String,
}

/// Piecewise trajectory starting at `start`.
#[derive(Debug, Clone)]
pub struct Trajectory {
    pub start: [f32; NUM_JOINTS],
    pub segments: Vec<Segment>,
}

impl Trajectory {
    pub fn hold(start: [f32; NUM_JOINTS], secs: f32) -> Trajectory {
        Trajectory { start, segments: vec![Segment { goal: start, ramp: 0.0, hold: secs, label: "hold".into() }] }
    }

    pub fn total_secs(&self) -> f32 {
        self.segments.iter().map(|s| s.ramp + s.hold).sum()
    }

    /// Stretch ramps so that no joint exceeds `max_speed` rad/s.
    pub fn limit_speed(&mut self, max_speed: f32) {
        let mut from = self.start;
        for seg in &mut self.segments {
            let max_delta = from.iter().zip(&seg.goal).map(|(a, b)| (b - a).abs()).fold(0f32, f32::max);
            seg.ramp = seg.ramp.max(max_delta / max_speed.max(1e-3));
            from = seg.goal;
        }
    }

    /// Target positions at time `t` (seconds since start; clamps to the final goal).
    pub fn at(&self, t: f32) -> [f32; NUM_JOINTS] {
        let mut from = self.start;
        let mut t0 = 0f32;
        for seg in &self.segments {
            let end = t0 + seg.ramp + seg.hold;
            if t < end {
                let a = if seg.ramp > 0.0 { ((t - t0) / seg.ramp).clamp(0.0, 1.0) } else { 1.0 };
                let s = a * a * (3.0 - 2.0 * a);
                return std::array::from_fn(|i| from[i] + (seg.goal[i] - from[i]) * s);
            }
            from = seg.goal;
            t0 = end;
        }
        from
    }

    /// Target velocities at `t` (central difference, 5 ms).
    pub fn velocity_at(&self, t: f32) -> [f32; NUM_JOINTS] {
        let h = 0.005f32;
        let a = self.at((t - h).max(0.0));
        let b = self.at(t + h);
        std::array::from_fn(|i| (b[i] - a[i]) / (2.0 * h))
    }

    pub fn segment_at(&self, t: f32) -> Option<&Segment> {
        let mut t0 = 0f32;
        self.segments.iter().find(|seg| {
            t0 += seg.ramp + seg.hold;
            t < t0
        })
    }
}

/// Label of joint `i` for messages.
pub fn label(i: usize) -> String {
    JointId::from_index(i).map(|j| j.label()).unwrap_or_default()
}
