//! Key expressions.
//!
//! Every resource of a hand lives under `wuji/<serial>/`. Device-level resources use a
//! plain name (`realtime_stream`, `mit_params`); per-joint resources are prefixed with the
//! joint's node path (`index/S1/control_mode`). Streams are published by the hand, commands
//! are subscribed by the hand, everything else is served by a single queryable on
//! `wuji/<serial>/**` (GET = query without payload, SET/action = query with payload).

pub const ROOT: &str = "wuji";
/// Liveliness token announced by every hand.
pub const ALIVE_ALL: &str = "wuji/*/@alive";
pub const ALL: &str = "wuji/**";

/// Resource paths relative to `wuji/<serial>/`.
pub mod rel {
    // Streams published by the hand
    /// 1 kHz joint state stream (`RealtimeStreamFrame`).
    pub const REALTIME_STREAM: &str = "realtime_stream";
    /// 100 Hz IMU stream (`ImuData`).
    pub const IMU: &str = "imu";

    // Commands subscribed by the hand
    /// `JointCommandFrame`, MIT control law.
    pub const JOINT_COMMAND: &str = "joint_command";
    pub const MOTOR_COMMAND: &str = "motor_command";
    pub const CMD_POSITION: &str = "cmd/position";
    pub const CMD_VELOCITY: &str = "cmd/velocity";
    pub const CMD_TORQUE: &str = "cmd/torque";
    pub const CMD_CURRENT: &str = "cmd/current";

    // GET
    pub const HW_VERSION: &str = "hw_version";
    pub const HANDEDNESS: &str = "handedness";
    pub const INPUT_VOLTAGE: &str = "input_voltage";
    pub const COMM_DIAG: &str = "comm_diag/hand";
    pub const CONTROL_MODE: &str = "control_mode";
    // GET + SET
    pub const MIT_PARAMS: &str = "mit_params";
    pub const EFFORT_LIMIT: &str = "effort_limit";
    // Actions (SET with empty payload)
    pub const MOTOR_ENABLE: &str = "motor_enable";
    pub const MOTOR_DISABLE: &str = "motor_disable";
    pub const CLEAR_FAULT: &str = "clear_fault";
    pub const EMERGENCY_STOP: &str = "emergency_stop";
    /// Self-description (`?kind=summary`, `?kind=node_types&index=N`, `?kind=nodes&index=N`).
    pub const CAPABILITY: &str = "@capability";
    pub const ALIVE: &str = "@alive";

    /// Per-joint registers (relative to the joint's node path).
    pub mod node {
        pub const CONTROL_MODE: &str = "control_mode";
        pub const ERROR_STATUS: &str = "error_status";
        pub const POS_ENC1: &str = "od/pos_rad_jnt_enc1";
        pub const POS_ENC2: &str = "od/pos_rad_jnt_enc2";
        pub const MIT_THETA_ERR: &str = "od/mit_theta_rad_err";
        pub const MIT_TAU_OUT: &str = "od/mit_tau_out_nm";
        pub const ENC2_CALIB_RESULT: &str = "enc2_calib_result";
    }
}

/// `wuji/<serial>/<rel>`
pub fn device_key(serial: &str, rel: &str) -> String {
    format!("{ROOT}/{serial}/{rel}")
}

/// `wuji/<serial>/<node_path>/<rel>`
pub fn node_key(serial: &str, node_path: &str, rel: &str) -> String {
    format!("{ROOT}/{serial}/{node_path}/{rel}")
}

/// Serial number from a key `wuji/<serial>/...`.
pub fn serial_from_key(key: &str) -> Option<&str> {
    let mut parts = key.split('/');
    if parts.next()? != ROOT {
        return None;
    }
    let sn = parts.next()?;
    (!sn.is_empty() && sn != "*" && !sn.starts_with('@')).then_some(sn)
}

/// Relative path after `wuji/<serial>/`.
pub fn rel_from_key(key: &str) -> Option<&str> {
    let rest = key.strip_prefix(ROOT)?.strip_prefix('/')?;
    let (_, rel) = rest.split_once('/')?;
    (!rel.is_empty()).then_some(rel)
}
