# Debugging

## Live telemetry

```sh
apeiron telemetry --imu              # table, 10 Hz refresh
apeiron telemetry --json > log.jsonl  # one JSON object per refresh
```

The table shows per joint: position (rad and degrees), velocity, commanded effort,
measured current, bus voltage, MCU temperature, the error **name** and the raw status
word; the header shows stream rates, sequence gaps and decode errors. Idle joints read
±0.000 A of current; anything above 0.02 A ripple while holding is a controller limit
cycle, a constant offset is the controller pushing against something.

## Recording

```sh
apeiron record --secs 10 --out rec.jsonl
```

Every stream sample becomes a line with time, key, length and the decoded frame. The
same format is produced by `apeiron pcap`, so plotting and analysis tools need only one
reader.

## Packet captures

The hand speaks plain Zenoh over UDP. Capture on the host that runs the driver:

```sh
sudo tcpdump -i <iface> -s 0 -w hand.pcapng 'host 192.168.1.111'
apeiron pcap hand.pcapng --summary          # keys, rates, lengths
apeiron pcap hand.pcapng --filter joint_command --no-transport | head
```

`pcap` reassembles Zenoh 1.x batches, resolves wire expressions through the `Declare`
messages and decodes the payloads with the layouts from the specification. For a vendor
SDK run, capture on the machine that runs the SDK; a third machine on a switch sees
nothing but the discovery broadcast. Wireshark with the Zenoh dissector (port 7447) is an
alternative view of the same traffic.

## Per-joint registers

Every joint node exposes its internals as GET resources:

```sh
apeiron get index/S4/od/pos_rad_jnt_enc1     # motor-side encoder, rad
apeiron get index/S4/od/pos_rad_jnt_enc2     # joint-side encoder, rad
apeiron get index/S4/od/mit_theta_rad_err    # controller position error
apeiron get index/S4/od/mit_tau_out_nm       # controller output
apeiron get index/S4/error_status
apeiron get index/S4/control_mode
```

`apeiron capabilities` lists all 587 resources with their access rights and scope, which
is the quickest way to find a register name.

Two readings worth knowing:

- `enc1` and `enc2` normally differ by 10–50 mrad. `enc2 == 0.0` while `enc1` moves is a
  frozen encoder value; power-cycle the hand.
- `od/mit_theta_rad_err` only updates while the motors are enabled; with motors off it
  holds the last value.

## Error names

`error_code_current` in the stream and `error_status` per node use the table in
[Errors](../spec/errors.md). `apeiron_protocol::error_codes::describe(code)` returns name,
severity, clearing rule, description and remedy. Code 6 (`BusFrameLossHigh`) after boot is
normal; immediate-stop codes (`0x20xx`) explain why a joint went limp.

## Command-loop timing

Every motion command prints the sender statistics: wake-up lateness (mean, p99, max) and
overruns. Values below 100 µs are normal without real-time configuration, below 15 µs
with `--rt-prio` and `--cpu`. Overruns indicate a starved host, not a problem on the hand.

## Noise

Buzzing or rattling with electrically quiet joints (current ripple ≈ 0.01 A) is gear
backlash in individual joints, typically the fingertip segments. Find the joint with
`telemetry` (velocity ripple) or `bias`, then store a preload for it. Lowering the
current limit below 0.3 A makes such joints worse, not better.
