# Message layouts

Application payloads are Rust structs serialized with
[postcard](https://docs.rs/postcard). The layouts below are the ones implemented in
`apeiron-protocol::messages` and verified on the device.

## postcard rules

| Type | Encoding |
|---|---|
| `u8`, `bool` | one byte |
| `u16`, `u32`, `u64` | LEB128 varint (1–5/10 bytes) |
| `i32` | zigzag varint |
| `f32`, `f64` | 4 / 8 bytes little-endian |
| `String`, `Vec<T>` | varint length, then contents |
| `[T; N]` | contents only, **no** length |
| `Option<T>` | `0x00` or `0x01` + value |
| struct | fields in declaration order, no names, no padding |

Nothing is self-describing: a wrong field type shifts every following field.

## Joint addressing

Motor boards are addressed by node id `nid = bus · 5 + node`, bus 0..5 = thumb, index,
middle, ring, pinky; node 1..4 = segment S1..S4; node 5 = tactile sensor. Host APIs use a
flat, finger-major index 0..20 (thumb 0–3, index 4–7, …). Tactile nids (5, 10, 15, 20,
25) never appear in joint data.

## FrameHeader

| Field | Type |
|---|---|
| `seq` | u32 |
| `timestamp_us` | u64 |
| `frame_id` | String (`r_wrist` / `l_wrist`; empty on `imu`) |

## RealtimeStreamFrame — `realtime_stream`, 1 kHz

| Field | Type |
|---|---|
| `header` | FrameHeader |
| `num_joints` | u8 |
| `joints` | Vec\<RealtimeStreamEntry\> |

### RealtimeStreamEntry

| Field | Type | Unit |
|---|---|---|
| `nid` | u8 | |
| `status_word` | u32 | opaque bit field (`0x24620001` while idle) |
| `pos_rad_jnt_fb` | f32 | rad |
| `vel_rad_s_jnt_fb` | f32 | rad/s |
| `effort` | f32 | A, commanded |
| `current` | f32 | A, measured |
| `vbus_v_fb` | f32 | V |
| `mcu_temp_c_fb` | f32 | °C |
| `error_code_current` | u16 | see [Errors](errors.md) |

Captured entry (nid 1):

```text
01 | 81 80 88 a3 02 | 68 30 1c 3e | 6d 81 08 38 | 00 00 00 00 | 00 00 00 00 | c8 c7 47 41 | c8 ae 3e 42 | 06
nid  status 0x24620001  pos 0.1526    vel ≈ 0       effort 0      current 0     vbus 12.49    47.7 °C       error 6
```

A full frame with 20 joints is 637 bytes (636 while `seq < 2^21`).

## ImuData — `imu`, 100 Hz

Layout of ROS `sensor_msgs/Imu`; 305 bytes.

| Field | Type |
|---|---|
| `header` | FrameHeader (`frame_id` empty) |
| `orientation` | Quaternion `{x, y, z, w: f64}` |
| `orientation_covariance` | [f64; 9] |
| `angular_velocity` | Vector3 `{x, y, z: f64}` |
| `angular_velocity_covariance` | [f64; 9] |
| `linear_acceleration` | Vector3 |
| `linear_acceleration_covariance` | [f64; 9] |

## JointCommandFrame — `joint_command`

| Field | Type |
|---|---|
| `seq` | u32 (varint) |
| `joints` | **[JointCommand; 20]** — fixed array, no length prefix |

### JointCommand

| Field | Type | Unit |
|---|---|---|
| `position` | f32 | rad, target |
| `velocity` | f32 | rad/s, target |
| `effort` | f32 | A, feed-forward |

Entries are in flat joint order (thumb S1 first, pinky S4 last). The frame is 241 bytes
while `seq < 128` and 242 bytes afterwards. **Do not send a `Vec`:** the extra length byte
shifts every position by one byte and the firmware drives the joints to 0 or to their
mechanical stops. Send at 200–1000 Hz; the hand stops its motors when frames cease.

## ScalarCommandFrame — `cmd/position`, `cmd/velocity`

| Field | Type |
|---|---|
| `seq` | u32 |
| `joints` | Vec\<f32\> |

From the SDK's type list; not exercised on the device.

## MitParam — `mit_params`

GET and SET carry `Vec<MitParam>` with 20 entries (161 bytes).

| Field | Type |
|---|---|
| `kp` | f32 |
| `kd` | f32 |

`effort_limit` is `Vec<f32>` with 20 entries in A (81 bytes).

## HwVersion — `hw_version`

| Field | Type |
|---|---|
| `major` | u8 |
| `minor` | u8 |
| `patch` | u8 |

## SetReply — reply to every SET and action

| Field | Type |
|---|---|
| `total` | u8 |
| `ok` | u8 |
| `results` | Vec\<SetResult\> |

### SetResult

| Field | Type |
|---|---|
| `nid` | u8 |
| `flag` | u8 |
| `code` | u8 |

63 bytes for 20 joints; `flag = code = 0` means success for that joint.

## Legacy and diagnostic types

`JointStateFrame`/`JointStateEntry` (`{nid, position, velocity, effort}`),
`JointDiagnosticsFrame`/`JointDiagnosticsEntry` and `CommSummary` follow the vendor SDK's
definitions and are implemented for decoding recordings; firmware 2.2.2 does not publish
them.
