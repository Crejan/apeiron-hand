# Transport

## Physical layer

| | |
|---|---|
| Link | 100BASE-TX Ethernet, static IPv4 |
| Address | `192.168.1.110` (left hand), `192.168.1.111` (right hand) |
| Handedness | fourth character of the serial number: `J` = left, `K` = right |
| Power | 12 V via XT30 |

The host needs an address in `192.168.1.0/24`; `192.168.1.50/24` is used throughout the
guides. The hand does not run DHCP and does not answer ICMP-reachable TCP ports.

## Zenoh session

The hand is a **Zenoh 1.x client** listening on `udp/<ip>:7447`. No TCP, no port 50001
(that port belongs to the vendor SDK's own router, not to the hand). Because Zenoh 1.x
refuses client-to-client links ("north-north client-client configuration"), a driver has to
present itself as a **router** that connects to the hand:

```json5
{
  mode: "router",
  listen: { endpoints: [] },
  connect: { endpoints: ["udp/192.168.1.111:7447"] },
  scouting: { multicast: { enabled: false }, gossip: { enabled: false } },
  adminspace: { enabled: true },
}
```

- No listeners: nothing else on the network can attach to the driver's router.
- Scouting off: the session only ever talks to the configured hand.
- Admin space on: the hand's declarations become visible under
  `@/<zid>/router/subscriber/**`, `@/<zid>/router/publisher/**` and
  `@/<zid>/router/queryable/**`, which is the quickest way to see what a firmware offers.

The hand's `InitAck` reports protocol version 0 and `whatami = client`, consistent with a
zenoh-pico implementation. Lease is 10 s, batches are plain (no compression, no
low-latency transport).

## Discovery

Every hand declares a liveliness token `wuji/<serial>/@alive`. A liveliness query on
`wuji/*/@alive` returns the serial numbers of all reachable hands; the driver uses this to
pick the serial when exactly one hand answers.

The vendor SDK additionally broadcasts a proprietary discovery datagram to UDP port 50000.
The hand opens that port but the exchange is not required to operate it, so Apeiron does
not implement it.

## Quality of service

Command frames are published with `express = true`, priority `RealTime` and congestion
control `Drop`: a stale set-point is worthless, and batching would add latency. Streams
arrive as individual `Push` messages, one per frame.
