# Poses and sequences

Pose files hold four target angles per finger (S1..S4, radians; `null` keeps the joint where
it is) plus default ramp (`secs`) and hold times. Sequence files list poses with per-step
timing and a loop count (`0` = until Ctrl-C). Targets are clamped to the mechanical range when
driven. Segment semantics: S1 base flexion (+ = flex), S2 abduction (+ = away from the thumb),
S3 middle joint, S4 fingertip.

```sh
apeiron poses
apeiron --allow-motion pose victory
apeiron --allow-motion sequence count --loop 2
apeiron --allow-motion save-pose my_grasp --description "..."
```
