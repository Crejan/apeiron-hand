# Apeiron

Open driver, command-line tooling and protocol specification for the **Wuji Hand 2**.
Plain Zenoh 1.x, no vendor SDK, Rust.

- Drive the hand from your own middleware through a small control API (position,
  velocity, impedance and effort modes at up to 1 kHz).
- Commission and calibrate it from the command line: self-test, gain tuning, preload
  tuning, per-hand profiles.
- Debug it: live telemetry, recordings, register access, packet-capture decoding, error
  codes resolved to names.

Documentation: `docs/`, built with [Zensical](https://zensical.org) (`uv run --with zensical zensical serve`) and published to GitHub Pages by `.github/workflows/docs.yml`.

## Status

Verified on firmware 2.2.2 / hardware 0.2.0 (right hand): transport, all streams and
GET/SET resources, enable/disable, command frame layout, cyclic control, commissioning
procedures. Tactile, firmware-update and factory resources are catalogued but not
exercised. Left hands differ only in address and serial number.

## Quick start

```sh
cargo build --release
ip addr add 192.168.1.50/24 dev <iface>            # hand: 192.168.1.111 (right) / .110 (left)
target/release/apeiron scan
target/release/apeiron status
target/release/apeiron telemetry
target/release/apeiron --allow-motion checkup       # self-test + tuning → profiles/<serial>.json
target/release/apeiron --allow-motion sequence demo
```

Anything that moves the hand needs `--allow-motion` and runs with a 0.3 A current limit
by default. Keep the fingers free.

## Workspace

| Crate | Contents |
|---|---|
| `apeiron-protocol` | message layouts, keys, joint mapping, capability pages, error codes |
| `apeiron-driver` | Zenoh session, `Hand`, control loop with middleware API, commissioning, profiles |
| `apeiron-adapter-udp` | reference middleware adapter (JSON over UDP) |
| `apeiron-cli` | the `apeiron` binary |

## License

MIT OR Apache-2.0, at your option. Apeiron is an independent project and is not
affiliated with, endorsed by or supported by the manufacturer of the hand.
