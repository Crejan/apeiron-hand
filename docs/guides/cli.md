# CLI reference

```text
apeiron [GLOBAL FLAGS] <COMMAND> [ARGS]
```

## Global flags

| Flag | Default | Meaning |
|---|---|---|
| `--endpoint <EP>` | `udp/192.168.1.111:7447` | Zenoh endpoint of the hand |
| `--serial <SN>` | from liveliness | select a hand when several answer |
| `--poses-dir <DIR>` | `$APEIRON_POSES`, `./poses`, `../poses` | pose and sequence files |
| `--profiles-dir <DIR>` | `$APEIRON_PROFILES`, `./profiles`, `../profiles` | hand profiles |
| `--allow-motion` | off | required by every command that enables the motors |
| `--dry-run` | off | print the trajectory, do not connect |
| `--max-effort <A>` | 0.3 | current limit written before enabling |
| `--max-speed <rad/s>` | 1.0 | ramps are stretched to respect this |
| `--rate <Hz>` | 200 | command rate, 200–1000 |
| `--rt-prio <N>` | 0 | SCHED_FIFO priority of the command thread (Linux) |
| `--cpu <N>` | – | pin the command thread to a CPU |
| `--gains <KP:KD>`, `--kd <KD>` | profile / 3, 0.2 | gains for all joints, override the profile |
| `--no-profile` | off | ignore `profiles/<serial>.json` |
| `--keep-enabled` | off | leave the motors enabled after a motion command |

## Read-only commands

| Command | Description |
|---|---|
| `scan` | hands announcing themselves and what they declared (subscribers, publishers, queryable) |
| `status` | firmware, handedness, voltage, gains, limits, joint positions |
| `telemetry [--imu] [--once] [--json] [--interval-ms N]` | live table or JSON lines of the 1 kHz stream |
| `record [--secs N] [--out FILE]` | all streams as JSON lines with decoded frames (`--out -` for stdout) |
| `capabilities [--resources] [--nodes]` | the device's self-description: summary, optionally the resource table and node list |
| `get <PATH>` | GET any resource, e.g. `get index/S1/od/pos_rad_jnt_enc2`; scalars are decoded |
| `poses` | list poses and sequences |
| `profile [JOINT --kp --kd --effort-limit --effort-bias]` | show or edit the hand profile |

## State-changing commands without motion

| Command | Description |
|---|---|
| `set <PATH> <HEX>` | SET with a raw payload |
| `action <PATH>` | action (SET with empty payload) |
| `enable`, `disable`, `clear-fault` | the three common actions |
| `ip [ADDRESS] [--reboot]` | show the stored IPv4 address and data port; with an address, store a new one (active after reboot) |
| `save-pose <NAME> [--description TEXT]` | store the current joint positions as a pose file |

`enable` on its own produces no torque; the hand needs a command stream for that.

## Motion commands (`--allow-motion`)

| Command | Description |
|---|---|
| `hold [--secs N]` | hold the current position |
| `move <JOINT=ANGLE>... [--secs N] [--hold N]` | move joints by index or label (`index_S1=20deg`, `all=0`) |
| `pose <NAME> [--secs N] [--hold N]` | drive to a pose file |
| `sequence <NAME> [--loop N]` | play a sequence (0 = until Ctrl-C) |
| `selfcheck [--delta RAD] [--secs-per-joint S]` | mechanical and encoder self-test |
| `tune [--joint J]... [--kp-list] [--kd-list] [--step] [--secs] [--save-profile]` | step-response gain tuning |
| `bias [--amps A] [--secs S] [--no-save]` | preload test |
| `checkup [--retune] [--skip-tune] [--skip-bias] [--kp-list] [--kd-list]` | self-test, tuning, preload, profile |

Per-joint overrides for one run: `--effort-override index_S2=0.1` and `--bias index_S4=-0.05`
(both repeatable); per-joint gains live in the profile (`apeiron profile index_S4 --kp 2 --kd 0.15`).
`--gains KP:KD` overrides the gains of all joints and ignores the profile. After every motion
command the tool prints the timing statistics of the command loop and the reached positions.

## Offline

| Command | Description |
|---|---|
| `pcap <FILE.pcapng> [--summary] [--filter KEY] [--no-transport]` | decode a packet capture of Zenoh traffic into JSON lines |

## Pose and sequence files

```json
{
  "name": "victory",
  "description": "Index and middle finger extended and spread",
  "joints": {
    "thumb":  [0.5, -0.9, 0.8, 0.8],
    "index":  [0.0, -0.4, 0.0, 0.0],
    "middle": [0.0, 0.4, 0.0, 0.0],
    "ring":   [1.2, 0.0, 1.3, 1.2],
    "pinky":  [1.2, 0.1, 1.3, 1.2]
  },
  "secs": 1.5,
  "hold": 1.0
}
```

Four values per finger (S1..S4, rad); `null` keeps a joint where it is; omitted fingers
stay as well. Values outside the mechanical range are clamped and reported.

```json
{
  "name": "demo",
  "loop": 1,
  "steps": [
    { "pose": "open",      "secs": 1.5, "hold": 1.0 },
    { "pose": "thumbs_up", "secs": 1.5, "hold": 1.5 }
  ]
}
```

Sequences live in `<poses-dir>/sequences/`; `secs` and `hold` default to the pose's own
values.
