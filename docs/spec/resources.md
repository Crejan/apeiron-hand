# Resources and key expressions

All resources of one hand live under `wuji/<serial>/`. Three interaction patterns exist:

| Pattern | Zenoh primitive | Direction |
|---|---|---|
| Stream | the hand publishes under `wuji/<serial>/**` | hand → host |
| Command | the hand subscribes to specific keys | host → hand |
| GET / SET / action | one queryable on `wuji/<serial>/**` | request/reply |

## Streams

| Key | Rate | Payload | Size |
|---|---|---|---|
| `realtime_stream` | 1000 Hz | `RealtimeStreamFrame` | 637 B (20 joints) |
| `imu` | 100 Hz | `ImuData` | 305 B |

These are published unconditionally as soon as the session is up. `joint/states`,
`joint_diagnostics` and `hms_codes` from the SDK's resource list are not published by
firmware 2.2.2. There is no rate control on the device; `rate/request` answers
`no such resource`. Decimate on the host.

## Commands

The hand declares subscribers for:

| Key | Payload | Note |
|---|---|---|
| `joint_command` | `JointCommandFrame` | MIT control law, used by Apeiron |
| `motor_command` | unknown | |
| `cmd/position` | `ScalarCommandFrame` | not exercised |
| `cmd/velocity` | `ScalarCommandFrame` | not exercised |
| `cmd/torque` | unknown | not exercised |
| `cmd/current` | unknown | not exercised |

Sending to any other key is silently dropped; there is no subscriber.

## GET, SET and actions

All of these go through the single queryable. The *kind* of request is encoded in the
query, not in the key:

| Kind | Query | Reply |
|---|---|---|
| GET | query without payload | resource value (postcard) |
| SET | query with payload | `SetReply` |
| action | query with an **empty** payload | `SetReply` |

An unknown path produces a reply **error** with the text `no such resource`. A GET on a
stream key returns a short descriptor, not a sample.

### Device-level resources

| Path | Access | Value |
|---|---|---|
| `hw_version` | GET | `HwVersion` (3 × u8) |
| `handedness` | GET | u8, 1 = left, 2 = right |
| `input_voltage` | GET | f32, V |
| `mit_params` | GET, SET | `Vec<MitParam>`, 20 entries |
| `effort_limit` | GET, SET | `Vec<f32>`, A, 20 entries |
| `control_mode` | GET, SET | `Vec<u8>`, 20 entries (1 on this firmware) |
| `comm_diag/hand` | GET | 533 B communication statistics |
| `motor_enable` | action | enables all joints |
| `motor_disable` | action | disables all joints |
| `clear_fault` | action | clears manually clearable errors |
| `emergency_stop` | action | |
| `@capability` | GET with parameters | see below |

### Per-joint registers

Each joint is a node with its own namespace `wuji/<serial>/<finger>/S<k>/<name>` where
`<finger>` is `thumb`, `index`, `middle`, `ring`, `pinky` and `k` is 1..4. The firmware
exposes several hundred registers there (`od/...`, "object dictionary"). Useful ones:

| Register | Type | Meaning |
|---|---|---|
| `control_mode` | u8 | per-joint control mode |
| `error_status` | u32 | active error code, see [Errors](errors.md) |
| `od/pos_rad_jnt_enc1` | f32 | joint angle derived from the motor-side encoder |
| `od/pos_rad_jnt_enc2` | f32 | joint angle from the joint-side encoder |
| `od/mit_theta_rad_err` | f32 | MIT position error (target − actual) |
| `od/mit_tau_out_nm` | f32 | MIT torque output |
| `od/mit_kp_term_nm`, `od/mit_kd_term_nm` | f32 | controller terms |
| `soft_pos_min`, `soft_pos_max`, `soft_limit_enabled` | f32, f32, u8 | firmware soft limits (0 / disabled by default) |
| `enc2_calib_result` | 7 B | encoder-2 calibration record |

Scalar registers are returned as raw little-endian values (`f32` = 4 bytes, `u8` = 1
byte); they are not postcard structs.

## Capability pages

`@capability` is the device's self-description. It must be queried with selector
parameters:

| Selector | Payload |
|---|---|
| `@capability?kind=summary` | JSON `{"device_id":2,"serial_number":"…","firmware_version":"2.2.2","uptime_s":…,"flag":2}` |
| `@capability?kind=node_types&index=N` | postcard page of resource entries |
| `@capability?kind=nodes&index=N` | postcard page of node entries |

Without parameters the reply is the error `capability parameters required`. Pages are
fetched by incrementing `index` until it equals `total`. Firmware 2.2.2 serves 75
`node_types` pages with 587 resource entries and 5 `nodes` pages with 25 nodes
(`/`, then `/<finger>/S1`..`S4` and `/<finger>/tactile` per finger).

Page layouts:

```text
node_types page: status u8, total varint, index varint, type_id varint, scope varint,
                 entries: Vec<Resource>
nodes page:      status u8, total varint, index varint, entries: Vec<Node>

Resource: resource_id varint, name String, access u8, schema_id varint,
          format varint, version varint
Node:     node_id varint, type_id varint, name String
```

`scope` tells where a resource lives: 0 = device, 1 = every joint node, 2 = every tactile
node. `access` is a bit mask: `0x01` GET, `0x02` SET, `0x04` SUB (stream), `0x08` PUB
(command the hand subscribes to), `0x10` action. `schema_id` identifies the payload type
on the vendor side; `format` and `version` are opaque.

## Network configuration

`ip_address` (GET/SET, postcard `String`, e.g. `"192.168.1.111"`) holds the static IPv4
address; a new value is stored by the firmware and becomes active after `reboot` (action).
`data_port` (GET, varint `u32`, factory value 50001) is the port of the vendor's legacy UDP
protocol and is not used by this driver. There is no netmask or gateway resource; the hand
answers on its link only.
