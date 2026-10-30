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
