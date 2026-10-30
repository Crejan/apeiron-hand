# Kinematics

Each finger has four actuated segments. Positions are joint angles in radians, zero at
the neutral, extended hand.

| Segment | Joint | Positive direction |
|---|---|---|
| S1 | base flexion (MCP) | flex, towards the palm |
| S2 | base abduction | away from the thumb |
| S3 | middle joint (PIP) | flex |
| S4 | fingertip (DIP) | flex |

Index to pinky behave identically; the thumb has different rest positions and ranges in
S1 and S2.

## Mechanical range

Ranges measured on a right hand, reduced by a small margin as used by
`apeiron_protocol::joint::approx_limits`:

| Segment | Fingers | Thumb |
|---|---|---|
| S1 | −1.00 … +1.40 rad | −1.15 … +0.75 rad |
| S2 | −0.60 … +0.60 rad | −1.25 … +0.15 rad |
| S3 | −1.00 … +1.50 rad | −1.00 … +1.50 rad |
| S4 | −0.85 … +1.50 rad | −0.85 … +1.50 rad |

The firmware's own soft limits (`soft_pos_min`/`soft_pos_max`) are zero and disabled on
the tested unit, so the driver clamps commands to the table above.

## Two encoders per joint

Every joint reports two angles: `od/pos_rad_jnt_enc1`, derived from the motor-side
encoder through the gear ratio, and `od/pos_rad_jnt_enc2`, the joint-side absolute
encoder. They differ by 10–50 mrad in normal operation (backlash and compliance). The
realtime stream and the controller use the joint-side value.
