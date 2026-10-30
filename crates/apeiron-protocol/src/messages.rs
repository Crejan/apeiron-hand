//! Message layouts (postcard).
//!
//! postcard wire rules: `u8`/`bool` one byte, `u16`/`u32`/`u64` LEB128 varints, `f32`/`f64`
//! little-endian, `String`/`Vec` length-prefixed, fixed arrays without prefix, struct fields
//! in declaration order. A wrong field type shifts everything after it, so the layouts below
//! are kept exactly as verified on the device.

use serde::{Deserialize, Serialize};

use crate::joint::NUM_JOINTS;

/// Header of the stream frames.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct FrameHeader {
    pub seq: u32,
    pub timestamp_us: u64,
    /// `r_wrist` / `l_wrist` on the joint stream, empty on the IMU stream.
    pub frame_id: String,
}

// ------------------------------------------------------------------ streams

/// `realtime_stream`, 1 kHz. One entry per online joint (tactile nodes are omitted).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct RealtimeStreamFrame {
    pub header: FrameHeader,
    pub num_joints: u8,
    pub joints: Vec<RealtimeStreamEntry>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct RealtimeStreamEntry {
    pub nid: u8,
    pub status_word: u32,
    /// Joint position, rad.
    pub pos_rad_jnt_fb: f32,
    /// Joint velocity, rad/s.
    pub vel_rad_s_jnt_fb: f32,
    /// Commanded effort, A.
    pub effort: f32,
    /// Measured phase current, A.
    pub current: f32,
    pub vbus_v_fb: f32,
    pub mcu_temp_c_fb: f32,
    pub error_code_current: u16,
}

/// `imu`, 100 Hz. Layout of ROS `sensor_msgs/Imu`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImuData {
    pub header: FrameHeader,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

/// `joint/states` as defined by the vendor SDK. Not published by firmware 2.2.2; kept for
/// decoding recordings made with other firmware.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct JointStateFrame {
    pub header: FrameHeader,
    pub num_joints: u8,
    pub joints: Vec<JointStateEntry>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct JointStateEntry {
    pub nid: u8,
    pub position: f32,
    pub velocity: f32,
    pub effort: f32,
}

/// `joint_diagnostics` stream (layout from the SDK binary, not observed on firmware 2.2.2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct JointDiagnosticsFrame {
    pub header: FrameHeader,
    pub num_joints: u8,
    pub joints: Vec<JointDiagnosticsEntry>,
    pub comm: Vec<CommSummary>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct JointDiagnosticsEntry {
    pub nid: u8,
    pub status_word: u32,
    pub current: f32,
    pub vbus_v_fb: f32,
    pub mcu_temp_c_fb: f32,
    pub error_code_current: u16,
    pub comm_response_rate_pct: u8,
    pub comm_timeout_total: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct CommSummary {
    pub age_ms: u16,
    pub tactile_online_mask: u8,
    pub e2e_received: u32,
    pub e2e_lost: u32,
    pub e2e_reordered: u16,
    pub e2e_duplicates: u16,
    pub e2e_window_loss_x100: u16,
    pub rpc_total: u32,
    pub rpc_retries: u16,
    pub rpc_timeouts: u16,
    pub comm_get_failures: u16,
    pub tactile_response_rate_pct: [u8; 5],
    pub tactile_timeout_total: [u32; 5],
    pub sdk_dropped: u32,
}

// ----------------------------------------------------------------- commands

/// One joint of a [`JointCommandFrame`]. The firmware applies the MIT control law
/// `torque = kp·(position − pos) + kd·(velocity − vel) + effort` with `kp`/`kd` from
/// [`MitParam`] (set via `mit_params`) and clamps the result to `effort_limit`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct JointCommand {
    /// Target position, rad.
    pub position: f32,
    /// Target velocity, rad/s.
    pub velocity: f32,
    /// Feed-forward effort, A.
    pub effort: f32,
}

/// `joint_command`: sequence number followed by exactly 20 commands in flat joint order,
/// **without** a length prefix (241 bytes while `seq < 128`). Send at 200 Hz to 1 kHz;
/// the hand stops the motors when the stream pauses.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointCommandFrame {
    pub seq: u32,
    pub joints: [JointCommand; NUM_JOINTS],
}

impl JointCommandFrame {
    pub fn new(seq: u32, joints: [JointCommand; NUM_JOINTS]) -> Self {
        Self { seq, joints }
    }

    pub fn from_slice(seq: u32, joints: &[JointCommand]) -> crate::Result<Self> {
        let joints: [JointCommand; NUM_JOINTS] = joints.try_into().map_err(|_| crate::Error::InvalidJointIndex(joints.len()))?;
        Ok(Self { seq, joints })
    }
}

/// `cmd/position` and `cmd/velocity` (from the SDK binary; not exercised on the device).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct ScalarCommandFrame {
    pub seq: u32,
    pub joints: Vec<f32>,
}

// --------------------------------------------------------------- parameters

/// MIT gains of one joint (`mit_params` is `Vec<MitParam>` with 20 entries).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct MitParam {
    pub kp: f32,
    pub kd: f32,
}

/// `hw_version`: three bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct HwVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

impl std::fmt::Display for HwVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

/// Reply to every SET or action: one three-byte result per joint.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SetReply {
    pub total: u8,
    pub ok: u8,
    pub results: Vec<SetResult>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SetResult {
    pub nid: u8,
    pub flag: u8,
    pub code: u8,
}

impl SetResult {
    pub fn is_ok(&self) -> bool {
        self.flag == 0 && self.code == 0
    }
}

impl SetReply {
    pub fn all_ok(&self) -> bool {
        self.results.iter().all(SetResult::is_ok)
    }

    pub fn failures(&self) -> Vec<SetResult> {
        self.results.iter().copied().filter(|r| !r.is_ok()).collect()
    }
}

/// Handedness as reported by `handedness`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Handedness {
    Left,
    Right,
    Unknown(u8),
}

impl From<u8> for Handedness {
    fn from(v: u8) -> Self {
        match v {
            1 => Handedness::Left,
            2 => Handedness::Right,
            o => Handedness::Unknown(o),
        }
    }
}

impl std::fmt::Display for Handedness {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Handedness::Left => write!(f, "left"),
            Handedness::Right => write!(f, "right"),
            Handedness::Unknown(v) => write!(f, "unknown({v})"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{decode, encode};

    #[test]
    fn joint_command_frame_has_no_length_prefix() {
        let cmds = [JointCommand { position: 0.1, velocity: 0.0, effort: 0.0 }; NUM_JOINTS];
        let bytes = encode(&JointCommandFrame::new(7, cmds)).unwrap();
        assert_eq!(bytes.len(), 241);
        assert_eq!(bytes[0], 7);
        assert_eq!(&bytes[1..5], &0.1f32.to_le_bytes());
        assert_eq!(encode(&JointCommandFrame::new(500, cmds)).unwrap().len(), 242);
    }

    #[test]
    fn realtime_entry_decodes_captured_bytes() {
        // nid 1, status 0x24620001, pos 0.1526, vel ~0, effort 0, current 0, vbus 12.49, temp 47.7, err 6
        let hex = "01 8180 88a3 02 6830 1c3e 6d81 0838 00000000 00000000 c8c7 4741 c8ae 3e42 06";
        let bytes: Vec<u8> = hex
            .split_whitespace()
            .flat_map(|h| (0..h.len()).step_by(2).map(move |i| u8::from_str_radix(&h[i..i + 2], 16).unwrap()))
            .collect();
        let e: RealtimeStreamEntry = decode(&bytes).unwrap();
        assert_eq!(e.nid, 1);
        assert_eq!(e.status_word, 0x2462_0001);
        assert!((e.pos_rad_jnt_fb - 0.1526).abs() < 1e-3);
        assert!((e.vbus_v_fb - 12.49).abs() < 0.01);
        assert_eq!(e.error_code_current, 6);
    }

    #[test]
    fn set_reply_decodes_captured_bytes() {
        let mut bytes = vec![0x14u8, 0x14, 0x14];
        for nid in [1u8, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24] {
            bytes.extend_from_slice(&[nid, 0, 0]);
        }
        let r: SetReply = decode(&bytes).unwrap();
        assert_eq!((r.total, r.ok, r.results.len()), (20, 20, 20));
        assert!(r.all_ok());
    }
}
