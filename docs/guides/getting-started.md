# Getting started

## Build

Rust 1.80 or newer. The workspace pins Zenoh 1.9.0, the version the hand was verified
against.

```sh
git clone https://github.com/apeiron/apeiron
cd apeiron
cargo build --release
```

Binaries land in `target/release/`: `apeiron` (CLI) and `apeiron-udp-adapter`.

## Network

The hand has a static address, `192.168.1.110` for a left and `192.168.1.111` for a right
hand, and does not run DHCP. Give the host interface an address in the same subnet:

```sh
# NetworkManager
nmcli con add type ethernet ifname <iface> con-name hand ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv6.method disabled
nmcli con up hand
# or plain iproute2
ip addr add 192.168.1.50/24 dev <iface>
```

`ping 192.168.1.111` must answer before anything else will.

## First contact

```sh
apeiron scan                                   # liveliness, streams, what the hand declares
apeiron --endpoint udp/192.168.1.111:7447 status
apeiron telemetry --imu                        # live table, Ctrl-C to leave
```

`scan` connects, lists the serial numbers announced under `wuji/*/@alive`, the keys that
are being published with their rates and sizes, and the subscribers and queryables the
hand declared. `status` prints firmware version, handedness, input voltage, the current
`mit_params` and `effort_limit`, and a snapshot of all joint positions. `telemetry`
renders the 1 kHz stream as a table with positions in rad and degrees, velocity, effort,
current, bus voltage, temperature and the error name of every joint.

All commands default to `udp/192.168.1.111:7447` (right hand); pass `--endpoint` for the
left hand or a custom address. If more than one hand answers, select one with
`--serial`.

## Real-time hosts

On a PREEMPT_RT kernel with isolated CPUs, give the command loop a fixed priority and a
dedicated core:

```sh
apeiron --allow-motion --rt-prio 80 --cpu 10 hold --secs 5
```

Requires `ulimit -r` to allow the priority. Without it the loop still runs with
well under 100 µs of jitter, which the hand does not notice.

## Next

- [Commissioning](commissioning.md): self-test and tuning before the first real use.
- [CLI reference](cli.md): every command and flag.
- [Middleware](middleware.md): drive the hand from your own stack.

## Finding a hand with an unknown address

The hand is silent on the wire until addressed: no DHCP, mDNS, LLDP, gratuitous ARP or
Zenoh scouting. It does answer a limited-broadcast ping, which works from any host on the
same link:

```sh
ping -b -I <iface> -c 3 255.255.255.255      # reply comes from the hand's address
sudo arp-scan --interface <iface> 192.168.1.0/24   # alternative: ARP sweep of a candidate range
```

The MAC address starts with `02:10:00`, which identifies the hand in a switch's address
table. Factory addresses are 192.168.1.110 (left) and 192.168.1.111 (right); `apeiron ip`
shows and changes the stored address once connected.
