# Apeiron

Apeiron is an open implementation of the Wuji Hand 2 protocol: a Rust driver, a command
line tool for commissioning, calibration, debugging and control, and this specification.
It talks to the hand over plain Zenoh 1.x and does not depend on the vendor SDK.

## What is in here

| Part | Purpose |
|---|---|
| [Specification](spec/transport.md) | Transport, resources, message layouts, control law, error codes, kinematics |
| [Guides](guides/getting-started.md) | Bring-up, commissioning, CLI reference, attaching a middleware, debugging |
| [Crates](crates.md) | How the workspace is layered |

## Status

Verified against firmware 2.2.2 / hardware revision 0.2.0 on a right hand: connection,
all streams and GET/SET resources, motor enable/disable, the command frame layout, cyclic
position control at 200–1000 Hz, self-test, gain tuning and preload tuning. Tactile
sensors, firmware update and factory-mode resources are listed but not exercised.

## Safety

The hand moves with real force. Everything that moves it requires `--allow-motion` on the
command line and runs with a current limit of 0.3 A per joint unless raised explicitly.
Keep the fingers free and unloaded during commissioning. The hand stops its motors by
itself when the command stream pauses.

## License

MIT OR Apache-2.0. Apeiron is not affiliated with or endorsed by the manufacturer of the
hand.
