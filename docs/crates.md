# Crates

```text
apeiron-protocol          wire format only, no I/O
      ▲
apeiron-driver            Zenoh session, Hand, control loop, commissioning, profiles
      ▲            ▲
apeiron-adapter-udp   apeiron-cli
```

## apeiron-protocol

Message layouts (`messages`), key expressions (`keys`), joint addressing and mechanical
limits (`joint`), capability page parsing (`capability`) and the error-code table
(`error_codes`). Depends on `postcard`, `serde` and `serde_json` only. Use it to decode
captures, to build a driver on another transport stack, or to generate bindings.

## apeiron-driver

- `transport`: Zenoh configuration (router without listeners) and session open.
- `hand`: `Hand` — discovery through liveliness, typed streams, GET/SET/actions,
  capability download, admin-space declarations.
- `profile`: `HandProfile` and resolved `Gains`.
- `motion`: poses, sequences, smoothstep trajectories with velocity feed-forward.
- `control`: `Controller`, `ControlMode`, `ControlHandle`, `ControlSource`, the real-time
  sender thread and the safety envelope.
- `commissioning`: `self_check`, `tune`, `bias_test`, `checkup`.

The driver is the only crate a middleware integration needs.

## apeiron-adapter-udp

Reference adapter: JSON commands in, JSON telemetry out, over UDP. Small enough to read
in one sitting and copy for another middleware.

## apeiron-cli

The `apeiron` binary: inventory, status, telemetry, recording, capability dump,
raw GET/SET, poses and sequences, commissioning, profiles and the offline `pcap`
decoder. Everything in the guides is implemented on top of the driver's public API;
nothing in the CLI talks to Zenoh directly.

## What belongs where

| Change | Crate |
|---|---|
| A new message type or key | protocol |
| A new error code | protocol (`error_codes`) |
| A new resource accessor, stream or action | driver (`hand`) |
| A new control mode or safety rule | driver (`control`) |
| A new commissioning check | driver (`commissioning`) |
| A new output format or interactive command | cli |
| A new middleware | new crate depending on driver |
