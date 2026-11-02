# Control

## The control law in the hand

Every joint board runs a 1 kHz MIT-style impedance loop:

```text
torque = kp · (q_des − q) + kd · (v_des − v) + effort_ff
```

- `q_des`, `v_des`, `effort_ff` come from the latest `JointCommandFrame`
  (`position`, `velocity`, `effort`).
- `kp`, `kd` are set per joint with `mit_params`. Firmware default 12 / 0.3; Apeiron
  defaults to **3 / 0.2**, which is markedly quieter on the tested hardware.
- The output is clamped to the per-joint current limit `effort_limit` (firmware default
  1.5 A; Apeiron defaults to **0.3 A**).
- Units: the hand reports `effort` in amperes of motor current. The motor constant
  (`od/ucfg/kt_torque`) is 0.01 Nm/A and the gear ratio (`od/ucfg/gear_ratio`) 50.4, so one
  ampere is ≈ 0.5 Nm at the joint. `kp` is therefore in A/rad and `kd` in A/(rad/s).

Motors produce torque only after `motor_enable` **and** while command frames arrive. An
enabled hand without commands is limp and silent; when the stream pauses, the hand's
watchdog disables the motors and `motor_enable` is required again.

## Control modes

The driver maps its modes onto that one law by choosing the gains it writes to
`mit_params` and which command fields it fills:

| Mode | kp | kd | Command fields |
|---|---|---|---|
| `Position` | profile | profile | `position`, `velocity` as feed-forward |
| `Velocity` | 0 | profile | `velocity` |
| `Impedance { kp, kd }` | given | given | `position`, `velocity`, `effort` |
| `Effort` | 0 | 0 | `effort` |

Switching modes rewrites `mit_params`; it is allowed while the loop runs. Gains and
limits are written once before `motor_enable` ("arming"), the command itself is sent every
cycle.

## Command cycle

Apeiron transmits the most recent command at a fixed rate (200–1000 Hz) from a dedicated
thread using absolute monotonic sleeps. On PREEMPT_RT hosts the thread can be given a
SCHED_FIFO priority and pinned to an isolated CPU; measured wake-up jitter is then below
15 µs, and below 70 µs without any real-time configuration. Jitter at this level has no
audible or measurable effect on the hand.

Before every frame the driver applies a safety envelope: positions clamped to the
mechanical range, `|velocity| ≤ max_velocity` (3 rad/s), `|effort| ≤ max_effort` (0.3 A).

## Trajectories and feed-forward

Poses and sequences are executed as smoothstep trajectories, ramps stretched so that no
joint exceeds `max_speed` (1 rad/s by default). The trajectory velocity is sent as
`velocity` so the `kd` term does not brake the intended motion.

## Preload against backlash

Fingertip joints have enough gear backlash to limit-cycle around a target, which is
audible as a buzz. Holding the fingertip lightly stops it; so does a small constant
feed-forward effort (`effort_bias`, typically ±0.05 A) that keeps the gear train pressed
against one flank. The preload test in commissioning finds the sign per joint and stores
it in the profile. Side effects: a static offset of `bias / kp` (≈ 1° at 0.05 A and
kp 3) and correspondingly less headroom in the opposite direction.

## Profiles

A profile (`profiles/<serial>.json`) stores per joint `kp`, `kd`, an optional lower
`max_effort` and an optional `effort_bias`. The driver merges it over the defaults
before arming; commissioning produces it.

## Feed-forward terms added by the driver

The driver fills the `effort` field of every command with up to three host-side terms,
all clamped to the current limit:

- `effort` from the command itself (impedance and effort modes),
- `effort_bias`: a constant preload per joint that keeps the gear train on one flank and
  suppresses backlash limit cycles,
- `friction_ff · sign(v_des)`: Coulomb friction compensation while the target moves
  (dead band 0.02 rad/s). The breakaway current of a base joint is 0.05–0.08 A
  (`apeiron effort JOINT=A` finds it); a value just below it reduces the lag at the start
  of a motion. It does not change the velocity ripple of a motion, see
  [Motion quality](../guides/commissioning.md#motion-quality).

Both per-joint values live in the hand profile (`effort_bias`, `friction_ff`) and can be
overridden for one run (`--bias`, `--friction`, `--friction-ff`).

## Firmware control modes

Every joint has a `control_mode` register (device-level `control_mode` is the vector of all
20; the register also mirrors into `od/mode_of_operation_display`). The firmware accepts the
values 0–4 and rejects anything above with `InvalidData`. Observed on firmware 2.2.2:

| mode | consumes | behaviour |
|---|---|---|
| 1 | `joint_command` | MIT law described above; factory state and what this driver uses |
| 2 | `cmd/position`, `cmd/velocity` | cascaded position → velocity → current loops of the firmware (`od/ucfg/pos_loop_*`, `vel_loop_*`, `cur_loop_*`) |
| 0, 3, 4 | not `cmd/position` | no reaction to position frames; presumably idle, torque and current modes fed by `cmd/torque` / `cmd/current` (not verified) |

The cascade in mode 2 is a pure P position loop (`pos_loop_kp` 35, no I/D, output limit 8)
driving a PI velocity loop (`vel_loop_kp` 0.005, `vel_loop_ki` 0.05, output limit 1 A). With
the conservative current limits used here it limit-cycles instead of settling, and because
its gains are persistent joint configuration rather than a per-command parameter, this
driver does not tune it. `ControlMode::Firmware { mode, topic }` (`apeiron --mode
firmware:N:topic`) exposes the mechanism for experiments: it writes `control_mode` before
enabling, streams one command field as `ScalarCommandFrame` on `cmd/<topic>`, and restores
mode 1 on disarm. Combine with `--only JOINT` and a low `--max-effort`.
