# Attaching a middleware

The driver crate is the integration point for ROS 2, LCM, a simulator bridge or any other
stack. A middleware adapter does two things: it feeds commands into the control loop and
publishes telemetry out of it. It never calibrates — commissioning stays with the CLI and
writes the profile the adapter then loads.

## The pieces

```rust
use apeiron_driver::{Hand, Endpoint, HandProfile};
use apeiron_driver::control::{Controller, ControlMode, Timing, Command, ControlHandle};
use apeiron_driver::profile::Gains;

let hand = Hand::connect(&Endpoint::parse("udp/192.168.1.111:7447")?, None).await?;
let profile = HandProfile::load_or_new(std::path::Path::new("profiles"), hand.serial());
let controller = Controller::new(&hand, profile.gains(Gains::default()))
    .mode(ControlMode::Position)
    .timing(Timing { rate_hz: 500, rt_priority: 80, cpu: Some(10) });
```

`Hand` is the connection: streams, GET/SET, actions. `Controller` owns the real-time
sender thread; constructing it does nothing, arming happens when a run starts. Gains,
limits and preload come from the profile merged over `Gains::default()` (3 / 0.2, 0.3 A).

## Two ways to drive the loop

### Push: `start()` and a `ControlHandle`

For middleware that receives commands asynchronously (a ROS 2 subscription, a socket):

```rust
let (handle, join) = controller.start().await?;      // arms and starts sending
handle.set_command(Command::hold(positions));         // from any thread, at any rate
while let Some(t) = handle.next_telemetry().await {   // 1 kHz, latest frame
    publish(t.position, t.velocity, t.current, t.error_code);
}
handle.stop();
let stats = join.await??;
controller.disarm().await?;
```

The sender transmits the most recent `Command` every cycle. If the middleware stops
updating, the last command is held; if the host process dies, the hand's own watchdog
stops the motors.

### Pull: `run()` with a `ControlSource`

For loops that compute the command from the current state at the cycle rate:

```rust
let stats = controller.run(|t: Duration, telemetry: Option<&Telemetry>| {
    let q = telemetry?.position;
    Some(Command { position: planner.step(q, t), velocity: [0.0; 20], effort: [0.0; 20] })
}).await?;
```

The closure (or any `ControlSource`) is polled on the sender thread once per cycle; keep
it allocation-free. Returning `None` keeps the previous command; `finished()` ends the
run, after which `run()` disarms unless `keep_enabled` is set.

## Control modes

| Mode | What the middleware sends |
|---|---|
| `Position` | target positions; optional velocity feed-forward |
| `Velocity` | target velocities (kp = 0) |
| `Impedance { kp, kd }` | positions, velocities and feed-forward effort with explicit gains |
| `Effort` | feed-forward effort only |

`controller.switch_mode(mode)` rewrites the gains while the loop keeps running, so a
teleoperation stack can move between position tracking and compliant holding without
re-arming. Every command passes the safety envelope (`Limits`: position clamp, maximum
velocity, maximum effort) before it is sent.

## Threading and latency

- The sender thread uses absolute monotonic sleeps; with `rt_priority` and a dedicated
  CPU on a PREEMPT_RT kernel the wake-up jitter stays below 15 µs.
- `ControlHandle` is `Clone + Send`; `set_command` takes a write lock for a 240-byte copy.
- `next_telemetry()` wakes once per received frame. A middleware that publishes at 100 Hz
  can simply read `telemetry()` on its own timer instead.
- Zenoh runs on its own Tokio runtime; the driver's async API needs a multi-threaded
  Tokio runtime in the host process.

## Worked example: the UDP adapter

`apeiron-adapter-udp` is a complete, deliberately small adapter that any language can
talk to. It opens a UDP socket, pushes every received command into a `ControlHandle` and
sends telemetry back to the last sender.

Commands (JSON, one datagram each):

```json
{"position": [20 floats], "velocity": [20 floats], "effort": [20 floats]}
{"mode": "position" | "velocity" | "effort" | {"impedance": {"kp": 3.0, "kd": 0.2}}}
{"stop": true}
```

`velocity` and `effort` are optional and default to zero. Telemetry (JSON, 100 Hz by
default):

```json
{"t": 1.234, "seq": 12001, "position": [...], "velocity": [...], "effort": [...],
 "current": [...], "error_code": [0, 6, ...], "error_name": ["none", "BusFrameLossHigh", ...]}
```

```sh
apeiron-udp-adapter --bind 0.0.0.0:7700 --telemetry-to 127.0.0.1:7701 --decimation 10 \
                    --mode position --rt-prio 80 --cpu 10
```

The adapter arms with the hand's profile (`--profiles-dir`), zeroes the current limit of any
joint whose encoder is frozen, keeps transmitting the last valid command, and disarms on
`{"stop": true}` or Ctrl-C. Several newline-separated messages per datagram are accepted;
malformed ones (wrong length, NaN) are logged and ignored.

## Sketch: a ROS 2 node

A `rclrs` or C++ node holding a `ControlHandle` would subscribe to
`sensor_msgs/JointState` (or a custom command message) and call `set_command` in the
callback, run a 100 Hz timer that reads `telemetry()` and publishes `JointState` with the
20 joint names (`thumb_S1` … `pinky_S4`), expose `switch_mode` as a service, and map
`error_code` to diagnostics using `apeiron_protocol::error_codes::name`. Use the flat
joint order of `apeiron_protocol::joint` everywhere; node ids never leave the driver.
