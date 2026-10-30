# Commissioning

Commissioning turns an unknown hand into one with a verified mechanism and a tuned
profile. It consists of three procedures that build on each other and one wrapper that
runs them all. Everything here moves the hand; place it so that all fingers can move
freely and keep a hand on the power switch.

Safety defaults for all procedures:

- `--allow-motion` is mandatory; without it the command refuses to arm the motors.
- The current limit is set to `--max-effort` (0.3 A) on every joint before enabling.
- Gains are written from the profile, or 3 / 0.2 if the joint has no entry.
- Before enabling, every joint is checked for a frozen joint encoder (see
  [Errors](../spec/errors.md)); affected joints are driven with 0 A and reported.
- Motors are disabled at the end, also on errors.

## Self-test — `apeiron selfcheck`

```sh
apeiron --allow-motion selfcheck                  # ±0.08 rad, 1.3 s per joint, ≈ 30 s
```

Each joint in turn is deflected by 0.08 rad (direction away from the nearer limit), held,
and returned while all others hold position. Per joint the report shows:

| Column | Check |
|---|---|
| moved | stream position reached ≥ 50 % of the deflection in the right direction |
| returned | back within 0.05 rad of the start |
| enc1, enc2 | both raw encoder registers followed by ≥ 50 % of the deflection |
| e1=e2 | encoders agree within 0.1 rad |
| crosstalk | no other joint left its position by more than 0.05 rad |
| I max | peak current during the joint's slot |
| err | error name at the end of the slot |

Exit code 2 if any joint fails. Typical peak currents are 0.1–0.3 A; a joint that only
moves with the limit saturated is worth a look.

## Gain tuning — `apeiron tune`

```sh
apeiron --allow-motion tune --joint index_S1 --kp-list 1.5,2,3,4,6 --kd-list 0.1,0.2,0.3 --save-profile
```

For every (kp, kd) candidate the joint receives a position step of `--step` (0.15 rad),
holds for `--secs` (1 s) and steps back. From the step response the tool computes rise
time (10–90 %), settling time (±5 %), overshoot, residual error, and the current and
velocity ripple in the second half of the hold. The recommendation is the candidate with
the smallest residual error among those that

- settled,
- overshoot by at most 15 %,
- show current ripple ≤ 0.03 A and velocity ripple ≤ 0.05 rad/s while holding.

`--save-profile` writes the recommendation into `profiles/<serial>.json`. Several
`--joint` options may be given. Joints with no acceptable candidate are reported and left
unchanged.

## Preload test — `apeiron bias`

```sh
apeiron --allow-motion bias --amps 0.05 --secs 3
```

All joints hold their position three times: with feed-forward effort 0, +0.05 A and
−0.05 A. A joint that jitters without preload (velocity ripple > 0.008 rad/s or current
ripple > 0.02 A) and is quiet with one sign gets that sign stored as `effort_bias`. This
removes the limit cycle that gear backlash causes in some fingertip joints.

## Everything — `apeiron checkup`

```sh
apeiron --allow-motion --rt-prio 80 --cpu 10 checkup
```

1. Self-test; stops with exit code 2 if it fails.
2. Tuning of all joints that have **no** profile entry yet, with a compact grid
   (kp 2, 3, 4, 6 × kd 0.15, 0.3; `--retune` includes joints already in the profile).
3. Preload test.
4. Writes `profiles/<serial>.json`, which is loaded automatically from then on.

About five minutes in total. Re-running is safe; existing entries survive unless
`--retune` is given.

## Profiles

```sh
apeiron profile                                  # show
apeiron profile index_S4 --kp 3 --kd 0.1 --effort-bias -0.05 --effort-limit 0.25
```

The profile directory is `--profiles-dir`, `$APEIRON_PROFILES`, `./profiles` or
`../profiles`, in that order. `--kp/--kd` on the command line or `--no-profile` override
the profile for one run.

## Interpreting results

- Residual errors of 1–2° at kp 3 are expected; stiffer gains trade quietness for
  accuracy.
- A joint that limit-cycles harder at a *lower* current limit is saturating; do not go
  below 0.3 A for such joints.
- `enc2` following `enc1` only partially, or an `e1=e2` failure, points at the
  joint-side encoder or its calibration record (`enc2_calib_result`).
- A frozen encoder warning is cleared by power-cycling the hand.
