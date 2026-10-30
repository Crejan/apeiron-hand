//! Per-hand tuning stored as `profiles/<serial>.json`.
//!
//! A profile is produced by the commissioning procedures and applied by the control loop
//! before the motors are enabled. Missing joints fall back to the defaults.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use apeiron_protocol::joint::{JointId, NUM_JOINTS};
use apeiron_protocol::messages::MitParam;
use serde::{Deserialize, Serialize};

use crate::Result;

/// Gains that run quietly on the tested hardware (the firmware defaults 12 / 0.3 excite
/// gear backlash audibly).
pub const DEFAULT_KP: f32 = 3.0;
pub const DEFAULT_KD: f32 = 0.2;
/// Conservative per-joint current limit in A.
pub const DEFAULT_EFFORT_LIMIT: f32 = 0.3;

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct HandProfile {
    pub serial: String,
    #[serde(default)]
    pub created: String,
    #[serde(default)]
    pub notes: Vec<String>,
    /// Joint label (`index_S1`) → settings.
    #[serde(default)]
    pub joints: BTreeMap<String, JointProfile>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointProfile {
    pub kp: f32,
    pub kd: f32,
    /// Current limit in A, if lower than the global one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_effort: Option<f32>,
    /// Constant feed-forward effort in A. Preloads the gear train against one flank and
    /// suppresses limit cycles in joints with backlash.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub effort_bias: Option<f32>,
}

impl Default for JointProfile {
    fn default() -> Self {
        JointProfile { kp: DEFAULT_KP, kd: DEFAULT_KD, max_effort: None, effort_bias: None }
    }
}

/// Settings resolved for all 20 joints.
#[derive(Debug, Clone, Copy)]
pub struct Gains {
    pub mit: [MitParam; NUM_JOINTS],
    pub effort_limit: [f32; NUM_JOINTS],
    pub effort_bias: [f32; NUM_JOINTS],
}

impl Gains {
    pub fn uniform(kp: f32, kd: f32, effort_limit: f32) -> Gains {
        Gains { mit: [MitParam { kp, kd }; NUM_JOINTS], effort_limit: [effort_limit; NUM_JOINTS], effort_bias: [0.0; NUM_JOINTS] }
    }
}

impl Default for Gains {
    fn default() -> Self {
        Gains::uniform(DEFAULT_KP, DEFAULT_KD, DEFAULT_EFFORT_LIMIT)
    }
}

impl HandProfile {
    pub fn new(serial: &str) -> HandProfile {
        HandProfile { serial: serial.to_string(), created: timestamp(), ..Default::default() }
    }

    pub fn path_in(dir: &Path, serial: &str) -> PathBuf {
        dir.join(format!("{serial}.json"))
    }

    pub fn load(path: &Path) -> Result<HandProfile> {
        Ok(serde_json::from_str(&std::fs::read_to_string(path)?)?)
    }

    /// Load `dir/<serial>.json` or start an empty profile.
    pub fn load_or_new(dir: &Path, serial: &str) -> HandProfile {
        HandProfile::load(&HandProfile::path_in(dir, serial)).unwrap_or_else(|_| HandProfile::new(serial))
    }

    pub fn save(&self, dir: &Path) -> Result<PathBuf> {
        std::fs::create_dir_all(dir)?;
        let path = HandProfile::path_in(dir, &self.serial);
        std::fs::write(&path, serde_json::to_string_pretty(self)? + "\n")?;
        Ok(path)
    }

    pub fn joint(&self, id: JointId) -> Option<&JointProfile> {
        self.joints.get(&id.label())
    }

    pub fn joint_mut(&mut self, id: JointId) -> &mut JointProfile {
        self.joints.entry(id.label()).or_default()
    }

    /// Resolve gains: profile entries over `base`, effort limits capped by `base`.
    pub fn gains(&self, base: Gains) -> Gains {
        let mut g = base;
        for (label, jp) in &self.joints {
            if let Ok(id) = JointId::parse(label) {
                let i = id.index();
                g.mit[i] = MitParam { kp: jp.kp, kd: jp.kd };
                if let Some(e) = jp.max_effort {
                    g.effort_limit[i] = g.effort_limit[i].min(e);
                }
                g.effort_bias[i] = jp.effort_bias.unwrap_or(0.0);
            }
        }
        g
    }

    pub fn note(&mut self, text: impl Into<String>) {
        self.notes.push(text.into());
    }
}

/// `YYYY-MM-DDTHH:MMZ` without pulling in a date crate.
pub fn timestamp() -> String {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let days = (secs / 86400) as i64;
    let (h, m) = ((secs % 86400) / 3600, (secs % 3600) / 60);
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy + 1 - (153 * mp + 2) / 5;
    let mo = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(mo <= 2);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{m:02}Z")
}
