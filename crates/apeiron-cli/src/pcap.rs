//! Offline decoder for pcap/pcapng captures of the Zenoh traffic between a host and a hand.
//!
//! Pipeline: pcap block → Ethernet/SLL/Null → IPv4 → TCP/UDP → Zenoh batch (TCP: 2-byte LE
//! length prefix, UDP: one batch per datagram) → `TransportMessage`s (Frame/Fragment/Init/
//! Open/KeepAlive/Close) → `NetworkMessage`s (Declare/Push/Request/Response) → key
//! resolution → postcard decoding with `apeiron_protocol::messages`. Output: JSON Lines on
//! stdout, `--summary` on stderr.
//!
//! Not supported: compressed batches (`ext_compression`), TLS/QUIC, IPv6, IP fragments,
//! TCP retransmission recovery beyond simple reordering.

#![allow(clippy::collapsible_match)]

use std::collections::{BTreeMap, HashMap};
use std::fmt;
use std::io::{Read, Write};
use std::net::Ipv4Addr;
use std::path::PathBuf;

use apeiron_driver::Error;
use apeiron_protocol::keys::rel_from_key;
use apeiron_protocol::messages::*;
use clap::Args;
use pcap_parser::{create_reader, Block, Linktype, PcapBlockOwned, PcapError};
use serde_json::{json, Value};
use zenoh_codec::{RCodec, Zenoh080, Zenoh080Reliability};
use zenoh_protocol::core::{Reliability, WireExpr};
use zenoh_protocol::network::{DeclareBody, Mapping, NetworkBody, NetworkMessage};
use zenoh_protocol::transport::{TransportBody, TransportMessage};
use zenoh_protocol::zenoh::{PushBody, RequestBody, ResponseBody};

/// Options of `apeiron pcap`.
#[derive(Args, Debug, Clone)]
pub struct PcapArgs {
    /// pcap or pcapng file
    pub file: PathBuf,
    /// Only print events whose key contains this substring
    #[arg(long)]
    pub filter: Option<String>,
    /// Print per-key statistics on stderr at the end
    #[arg(long)]
    pub summary: bool,
    /// Suppress transport handshake/keep-alive events
    #[arg(long)]
    pub no_transport: bool,
}

/// Decode `args.file`, writing one JSON line per event to stdout.
pub fn run(args: PcapArgs) -> apeiron_driver::Result<()> {
    let file = std::fs::File::open(&args.file).map_err(|e| Error::Other(format!("{}: {e}", args.file.display())))?;
    let mut decoder = Decoder::new();
    let stdout = std::io::stdout();
    let mut out = std::io::BufWriter::new(stdout.lock());
    let result = run_pcap(std::io::BufReader::new(file), &mut decoder, |e| {
        if args.no_transport && e.kind == "transport" {
            return;
        }
        if let Some(f) = &args.filter {
            if !e.key.contains(f.as_str()) {
                return;
            }
        }
        let _ = writeln!(out, "{}", event_json(e));
    });
    let _ = out.flush();
    result.map_err(Error::Other)?;
    if args.summary {
        print_summary(&decoder.stats);
    }
    Ok(())
}

// ---------------------------------------------------------------------------------------------
// Link / network / transport layer
// ---------------------------------------------------------------------------------------------

/// One direction of a flow (source → destination).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Dir {
    pub src: (Ipv4Addr, u16),
    pub dst: (Ipv4Addr, u16),
}

impl Dir {
    fn reverse(self) -> Self {
        Dir { src: self.dst, dst: self.src }
    }
}

impl fmt::Display for Dir {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{} -> {}:{}", self.src.0, self.src.1, self.dst.0, self.dst.1)
    }
}

#[derive(Debug)]
pub enum L4<'a> {
    Tcp { dir: Dir, seq: u32, syn: bool, payload: &'a [u8] },
    Udp { dir: Dir, payload: &'a [u8] },
}

/// Strip the link-layer header and return the IPv4 packet.
fn strip_link(linktype: Linktype, data: &[u8]) -> Option<&[u8]> {
    match linktype {
        Linktype::ETHERNET => {
            if data.len() < 14 {
                return None;
            }
            let mut off = 12;
            let mut ethertype = u16::from_be_bytes([data[off], data[off + 1]]);
            // 802.1Q VLAN tags
            while ethertype == 0x8100 || ethertype == 0x88a8 {
                off += 4;
                if data.len() < off + 2 {
                    return None;
                }
                ethertype = u16::from_be_bytes([data[off], data[off + 1]]);
            }
            (ethertype == 0x0800).then(|| &data[off + 2..])
        }
        Linktype::LINUX_SLL => {
            if data.len() < 16 {
                return None;
            }
            let proto = u16::from_be_bytes([data[14], data[15]]);
            (proto == 0x0800).then(|| &data[16..])
        }
        Linktype::LINUX_SLL2 => {
            if data.len() < 20 {
                return None;
            }
            let proto = u16::from_be_bytes([data[0], data[1]]);
            (proto == 0x0800).then(|| &data[20..])
        }
        Linktype::NULL => {
            if data.len() < 4 {
                return None;
            }
            // BSD loopback: 4-byte address family in host byte order, AF_INET = 2
            let fam_le = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            let fam_be = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
            (fam_le == 2 || fam_be == 2).then(|| &data[4..])
        }
        Linktype::RAW | Linktype::IPV4 => Some(data),
        _ => None,
    }
}

/// Parse IPv4 + TCP/UDP. IPv6, IP fragments and other protocols are ignored.
pub fn parse_l4(linktype: Linktype, frame: &[u8]) -> Option<L4<'_>> {
    let ip = strip_link(linktype, frame)?;
    if ip.len() < 20 || ip[0] >> 4 != 4 {
        return None;
    }
    let ihl = usize::from(ip[0] & 0x0f) * 4;
    let total = usize::from(u16::from_be_bytes([ip[2], ip[3]]));
    let flags_frag = u16::from_be_bytes([ip[6], ip[7]]);
    if flags_frag & 0x1fff != 0 || flags_frag & 0x2000 != 0 {
        return None; // IP fragment
    }
    let proto = ip[9];
    let src = Ipv4Addr::new(ip[12], ip[13], ip[14], ip[15]);
    let dst = Ipv4Addr::new(ip[16], ip[17], ip[18], ip[19]);
    let end = total.min(ip.len()).max(ihl);
    let l4 = &ip[ihl..end];
    match proto {
        6 => {
            if l4.len() < 20 {
                return None;
            }
            let sport = u16::from_be_bytes([l4[0], l4[1]]);
            let dport = u16::from_be_bytes([l4[2], l4[3]]);
            let seq = u32::from_be_bytes([l4[4], l4[5], l4[6], l4[7]]);
            let doff = usize::from(l4[12] >> 4) * 4;
            let syn = l4[13] & 0x02 != 0;
            if l4.len() < doff {
                return None;
            }
            Some(L4::Tcp { dir: Dir { src: (src, sport), dst: (dst, dport) }, seq, syn, payload: &l4[doff..] })
        }
        17 => {
            if l4.len() < 8 {
                return None;
            }
            let sport = u16::from_be_bytes([l4[0], l4[1]]);
            let dport = u16::from_be_bytes([l4[2], l4[3]]);
            let len = usize::from(u16::from_be_bytes([l4[4], l4[5]])).min(l4.len());
            Some(L4::Udp { dir: Dir { src: (src, sport), dst: (dst, dport) }, payload: &l4[8..len] })
        }
        _ => None,
    }
}

/// Minimal TCP reassembly per direction: sort segments by sequence number and append them
/// without gaps. On a gap we wait until the missing segment arrives or too many segments
/// have piled up (then the gap is skipped).
#[derive(Default)]
struct TcpStream {
    next_seq: Option<u32>,
    /// The start sequence was guessed from the first segment seen (no SYN captured).
    guessed: bool,
    pending: BTreeMap<u32, Vec<u8>>,
    buf: Vec<u8>,
    gaps: usize,
}

impl TcpStream {
    const MAX_PENDING: usize = 64;

    fn push(&mut self, seq: u32, syn: bool, payload: &[u8]) {
        if syn {
            self.next_seq = Some(seq.wrapping_add(1));
            return;
        }
        if payload.is_empty() {
            return;
        }
        match self.next_seq {
            None => {
                self.next_seq = Some(seq);
                self.guessed = true;
            }
            Some(next) if self.guessed => {
                // Capture started mid-stream and an older segment shows up: push the buffer
                // back into the queue and restart before it (as long as no batch was taken).
                let buf_seq = next.wrapping_sub(self.buf.len() as u32);
                let back = buf_seq.wrapping_sub(seq);
                if back != 0 && back < 65_536 {
                    if !self.buf.is_empty() {
                        let old = std::mem::take(&mut self.buf);
                        self.pending.insert(buf_seq, old);
                    }
                    self.next_seq = Some(seq);
                }
            }
            Some(_) => {}
        }
        self.pending.entry(seq).or_insert_with(|| payload.to_vec());
        self.drain();
        if self.pending.len() > Self::MAX_PENDING {
            // Skip the gap: continue at the smallest pending segment.
            if let Some(&first) = self.pending.keys().next() {
                self.gaps += 1;
                self.next_seq = Some(first);
                self.buf.clear(); // a partial batch cannot be decoded anyway
                self.drain();
            }
        }
    }

    fn drain(&mut self) {
        let Some(mut next) = self.next_seq else { return };
        loop {
            // The segment starting exactly at `next`, or an older one overlapping it.
            let candidate = self.pending.range(..=next).next_back().map(|(&s, v)| (s, v.len()));
            match candidate {
                Some((s, len)) => {
                    let data = self.pending.remove(&s).unwrap();
                    let skip = next.wrapping_sub(s) as usize;
                    if skip < len {
                        self.buf.extend_from_slice(&data[skip..]);
                        next = s.wrapping_add(len as u32);
                    }
                    // otherwise: pure retransmission, drop it
                }
                None => break,
            }
        }
        self.next_seq = Some(next);
    }

    /// Take complete Zenoh batches (2-byte LE length + content).
    fn take_batches(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let mut off = 0;
        while self.buf.len() >= off + 2 {
            let len = usize::from(u16::from_le_bytes([self.buf[off], self.buf[off + 1]]));
            if self.buf.len() < off + 2 + len {
                break;
            }
            out.push(self.buf[off + 2..off + 2 + len].to_vec());
            off += 2 + len;
        }
        self.buf.drain(..off);
        if !out.is_empty() {
            self.guessed = false;
        }
        out
    }
}

// ---------------------------------------------------------------------------------------------
// Zenoh layer
// ---------------------------------------------------------------------------------------------

/// One decoded event (one JSON line).
#[derive(Debug, Clone)]
pub struct Event {
    pub t: f64,
    pub dir: Dir,
    pub kind: &'static str,
    pub key: String,
    pub len: usize,
    pub payload: Vec<u8>,
    pub info: Value,
}

#[derive(Default)]
struct Session {
    /// Key expressions declared by the sender of this direction (expr id → full key)
    keyexprs: HashMap<u16, String>,
    /// Fragment buffer per reliability
    frag: HashMap<bool, (u32, Vec<u8>)>,
}

#[derive(Default)]
pub struct Stats {
    pub packets: usize,
    pub batches: usize,
    pub transport_msgs: usize,
    pub decode_errors: usize,
    pub tcp_gaps: usize,
    pub per_key: BTreeMap<String, KeyStats>,
}

#[derive(Default)]
pub struct KeyStats {
    pub count: usize,
    pub bytes: usize,
    pub first_t: f64,
    pub last_t: f64,
    pub lengths: BTreeMap<usize, usize>,
}

/// Decoder state across all flows.
pub struct Decoder {
    tcp: HashMap<Dir, TcpStream>,
    sessions: HashMap<Dir, Session>,
    pub stats: Stats,
    pub events: Vec<Event>,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    pub fn new() -> Self {
        Self { tcp: HashMap::new(), sessions: HashMap::new(), stats: Stats::default(), events: Vec::new() }
    }

    /// Process one link-layer packet. TCP payload is reassembled into length-prefixed
    /// batches; every UDP datagram is one batch (the hand speaks Zenoh over udp/7447).
    pub fn feed_packet(&mut self, t: f64, linktype: Linktype, frame: &[u8]) {
        self.stats.packets += 1;
        match parse_l4(linktype, frame) {
            Some(L4::Tcp { dir, seq, syn, payload }) => {
                let stream = self.tcp.entry(dir).or_default();
                stream.push(seq, syn, payload);
                let batches = stream.take_batches();
                for b in batches {
                    self.feed_batch(t, dir, &b);
                }
            }
            Some(L4::Udp { dir, payload }) => {
                if !payload.is_empty() {
                    self.feed_batch(t, dir, payload);
                }
            }
            None => {}
        }
    }

    /// A Zenoh batch is a sequence of transport messages.
    pub fn feed_batch(&mut self, t: f64, dir: Dir, batch: &[u8]) {
        self.stats.batches += 1;
        let codec = Zenoh080::new();
        let mut reader = batch;
        while !reader.is_empty() {
            let msg: TransportMessage = match codec.read(&mut reader) {
                Ok(m) => m,
                Err(_) => {
                    self.stats.decode_errors += 1;
                    break;
                }
            };
            self.stats.transport_msgs += 1;
            self.handle_transport(t, dir, msg);
        }
    }

    fn handle_transport(&mut self, t: f64, dir: Dir, msg: TransportMessage) {
        match msg.body {
            TransportBody::Frame(f) => {
                for nm in f.payload {
                    self.handle_network(t, dir, nm);
                }
            }
            TransportBody::Fragment(fr) => {
                let reliable = fr.reliability == Reliability::Reliable;
                let session = self.sessions.entry(dir).or_default();
                let entry = session.frag.entry(reliable).or_insert_with(|| (fr.sn, Vec::new()));
                if fr.ext_first.is_some() || entry.1.is_empty() {
                    entry.0 = fr.sn;
                    entry.1.clear();
                } else if fr.sn != entry.0.wrapping_add(1) {
                    // gap in the fragment sequence → discard
                    entry.1.clear();
                    self.stats.decode_errors += 1;
                }
                entry.0 = fr.sn;
                entry.1.extend_from_slice(fr.payload.as_slice());
                if fr.ext_drop.is_some() {
                    entry.1.clear();
                    return;
                }
                if !fr.more {
                    let data = std::mem::take(&mut entry.1);
                    let rcodec = Zenoh080Reliability::new(fr.reliability);
                    let mut reader = data.as_slice();
                    match rcodec.read(&mut reader) {
                        Ok(nm) => self.handle_network(t, dir, nm),
                        Err(_) => self.stats.decode_errors += 1,
                    }
                }
            }
            TransportBody::InitSyn(i) => self.transport_event(
                t,
                dir,
                "InitSyn",
                json!({"version": i.version, "whatami": i.whatami.to_string(), "zid": i.zid.to_string(),
                       "batch_size": i.batch_size, "compression": i.ext_compression.is_some(),
                       "lowlatency": i.ext_lowlatency.is_some()}),
            ),
            TransportBody::InitAck(i) => self.transport_event(
                t,
                dir,
                "InitAck",
                json!({"version": i.version, "whatami": i.whatami.to_string(), "zid": i.zid.to_string(),
                       "batch_size": i.batch_size, "compression": i.ext_compression.is_some(),
                       "lowlatency": i.ext_lowlatency.is_some()}),
            ),
            TransportBody::OpenSyn(o) => {
                self.transport_event(t, dir, "OpenSyn", json!({"lease_ms": o.lease.as_millis() as u64, "initial_sn": o.initial_sn}))
            }
            TransportBody::OpenAck(o) => {
                self.transport_event(t, dir, "OpenAck", json!({"lease_ms": o.lease.as_millis() as u64, "initial_sn": o.initial_sn}))
            }
            TransportBody::KeepAlive(_) => self.transport_event(t, dir, "KeepAlive", Value::Null),
            TransportBody::Close(c) => self.transport_event(t, dir, "Close", json!({"reason": c.reason, "session": c.session})),
            TransportBody::Join(_) => self.transport_event(t, dir, "Join", Value::Null),
            TransportBody::OAM(o) => self.transport_event(t, dir, "OAM", json!({"id": o.id})),
        }
    }

    fn transport_event(&mut self, t: f64, dir: Dir, what: &str, info: Value) {
        self.events.push(Event { t, dir, kind: "transport", key: what.to_string(), len: 0, payload: Vec::new(), info });
    }

    /// Resolve a wire expression against the key table of the owning direction.
    fn resolve(&self, dir: Dir, we: &WireExpr<'_>) -> String {
        if we.scope == 0 {
            return we.suffix.to_string();
        }
        let owner = match we.mapping {
            Mapping::Sender => dir,
            Mapping::Receiver => dir.reverse(),
        };
        let lookup = |d: Dir| self.sessions.get(&d).and_then(|s| s.keyexprs.get(&we.scope).cloned());
        match lookup(owner).or_else(|| lookup(owner.reverse())) {
            Some(prefix) => format!("{prefix}{}", we.suffix),
            None => format!("<{}>{}", we.scope, we.suffix),
        }
    }

    fn handle_network(&mut self, t: f64, dir: Dir, nm: NetworkMessage) {
        match nm.body {
            NetworkBody::Declare(d) => {
                let (what, key, info) = match &d.body {
                    DeclareBody::DeclareKeyExpr(k) => {
                        let full = self.resolve(dir, &k.wire_expr);
                        self.sessions.entry(dir).or_default().keyexprs.insert(k.id, full.clone());
                        ("DeclareKeyExpr", full, json!({"expr_id": k.id}))
                    }
                    DeclareBody::UndeclareKeyExpr(k) => ("UndeclareKeyExpr", String::new(), json!({"expr_id": k.id})),
                    DeclareBody::DeclareSubscriber(s) => ("DeclareSubscriber", self.resolve(dir, &s.wire_expr), json!({"id": s.id})),
                    DeclareBody::UndeclareSubscriber(s) => ("UndeclareSubscriber", String::new(), json!({"id": s.id})),
                    DeclareBody::DeclareQueryable(q) => (
                        "DeclareQueryable",
                        self.resolve(dir, &q.wire_expr),
                        json!({"id": q.id, "complete": q.ext_info.complete, "distance": q.ext_info.distance}),
                    ),
                    DeclareBody::UndeclareQueryable(q) => ("UndeclareQueryable", String::new(), json!({"id": q.id})),
                    DeclareBody::DeclareToken(k) => ("DeclareToken", self.resolve(dir, &k.wire_expr), json!({"id": k.id})),
                    DeclareBody::UndeclareToken(k) => ("UndeclareToken", String::new(), json!({"id": k.id})),
                    DeclareBody::DeclareFinal(_) => ("DeclareFinal", String::new(), Value::Null),
                };
                let mut info = info;
                info["declare"] = Value::String(what.to_string());
                if let Some(iid) = d.interest_id {
                    info["interest_id"] = json!(iid);
                }
                self.events.push(Event { t, dir, kind: "declare", key, len: 0, payload: Vec::new(), info });
            }
            NetworkBody::Interest(i) => {
                let key = i.wire_expr.as_ref().map(|w| self.resolve(dir, w)).unwrap_or_default();
                self.events.push(Event {
                    t,
                    dir,
                    kind: "interest",
                    key,
                    len: 0,
                    payload: Vec::new(),
                    info: json!({"id": i.id, "mode": format!("{:?}", i.mode)}),
                });
            }
            NetworkBody::Push(p) => {
                let key = self.resolve(dir, &p.wire_expr);
                let (payload, info) = match p.payload {
                    PushBody::Put(put) => {
                        let bytes = put.payload.to_zslice().as_slice().to_vec();
                        (bytes, json!({"encoding": encoding_str(&put.encoding), "timestamp": put.timestamp.map(|ts| ts.to_string())}))
                    }
                    PushBody::Del(_) => (Vec::new(), json!({"del": true})),
                };
                self.record(t, dir, "push", key, payload, info);
            }
            NetworkBody::Request(r) => {
                let key = self.resolve(dir, &r.wire_expr);
                let RequestBody::Query(q) = r.payload;
                let (payload, encoding) = match q.ext_body {
                    Some(b) => (b.payload.to_zslice().as_slice().to_vec(), encoding_str(&b.encoding)),
                    None => (Vec::new(), String::new()),
                };
                self.record(
                    t,
                    dir,
                    "request",
                    key,
                    payload,
                    json!({"rid": r.id, "parameters": q.parameters, "encoding": encoding, "target": format!("{:?}", r.ext_target)}),
                );
            }
            NetworkBody::Response(r) => {
                let key = self.resolve(dir, &r.wire_expr);
                let (payload, info) = match r.payload {
                    ResponseBody::Reply(rep) => match rep.payload {
                        PushBody::Put(put) => {
                            let bytes = put.payload.to_zslice().as_slice().to_vec();
                            (bytes, json!({"rid": r.rid, "encoding": encoding_str(&put.encoding)}))
                        }
                        PushBody::Del(_) => (Vec::new(), json!({"rid": r.rid, "del": true})),
                    },
                    ResponseBody::Err(e) => {
                        let bytes = e.payload.to_zslice().as_slice().to_vec();
                        (bytes, json!({"rid": r.rid, "error": true, "encoding": encoding_str(&e.encoding)}))
                    }
                };
                self.record(t, dir, "response", key, payload, info);
            }
            NetworkBody::ResponseFinal(r) => {
                self.events.push(Event {
                    t,
                    dir,
                    kind: "response_final",
                    key: String::new(),
                    len: 0,
                    payload: Vec::new(),
                    info: json!({"rid": r.rid}),
                });
            }
            NetworkBody::OAM(o) => {
                self.events.push(Event { t, dir, kind: "oam", key: String::new(), len: 0, payload: Vec::new(), info: json!({"id": o.id}) });
            }
        }
    }

    fn record(&mut self, t: f64, dir: Dir, kind: &'static str, key: String, payload: Vec<u8>, info: Value) {
        let ks = self.stats.per_key.entry(format!("{kind} {key}")).or_default();
        if ks.count == 0 {
            ks.first_t = t;
        }
        ks.count += 1;
        ks.bytes += payload.len();
        ks.last_t = t;
        *ks.lengths.entry(payload.len()).or_default() += 1;
        self.events.push(Event { t, dir, kind, key, len: payload.len(), payload, info });
    }
}

fn encoding_str(e: &zenoh_protocol::core::Encoding) -> String {
    match &e.schema {
        Some(s) => format!("{}:{}", e.id, String::from_utf8_lossy(s.as_slice())),
        None => e.id.to_string(),
    }
}

// ---------------------------------------------------------------------------------------------
// Payload decoding
// ---------------------------------------------------------------------------------------------

/// Decode a payload by the resource name of its key, falling back to a length heuristic.
pub fn decode_payload(key: &str, bytes: &[u8]) -> Option<Value> {
    if bytes.is_empty() {
        return None;
    }
    fn try_as<T: serde::de::DeserializeOwned + serde::Serialize>(b: &[u8]) -> Option<Value> {
        apeiron_protocol::decode::<T>(b).ok().and_then(|v| serde_json::to_value(v).ok())
    }
    let rel = rel_from_key(key).unwrap_or(key);
    let by_key = match rel {
        "realtime_stream" => try_as::<RealtimeStreamFrame>(bytes),
        "imu" => try_as::<ImuData>(bytes),
        "joint_command" => try_as::<JointCommandFrame>(bytes),
        "joint/states" | "joint_states" => try_as::<JointStateFrame>(bytes),
        "joint_diagnostics" => try_as::<JointDiagnosticsFrame>(bytes),
        "cmd/position" | "cmd/velocity" => try_as::<ScalarCommandFrame>(bytes),
        "mit_params" => try_as::<Vec<MitParam>>(bytes),
        "effort_limit" => try_as::<Vec<f32>>(bytes),
        "hw_version" => try_as::<HwVersion>(bytes),
        _ => None,
    };
    by_key.or_else(|| {
        // Length heuristics: a command frame is 241 bytes (seq < 128) or 242 bytes.
        if bytes.len() == 241 || bytes.len() == 242 {
            try_as::<JointCommandFrame>(bytes)
        } else {
            try_as::<RealtimeStreamFrame>(bytes).filter(|v| v.get("num_joints").and_then(Value::as_u64).is_some_and(|n| n <= 20)).or_else(
                || try_as::<JointStateFrame>(bytes).filter(|v| v.get("num_joints").and_then(Value::as_u64).is_some_and(|n| n <= 20)),
            )
        }
    })
}

fn hex(bytes: &[u8], max: usize) -> String {
    bytes.iter().take(max).map(|b| format!("{b:02x}")).collect::<Vec<_>>().join("")
}

pub fn event_json(e: &Event) -> Value {
    let decoded = if e.kind == "transport" || e.kind == "declare" { None } else { decode_payload(&e.key, &e.payload) };
    json!({
        "t": e.t,
        "src": format!("{}:{}", e.dir.src.0, e.dir.src.1),
        "dst": format!("{}:{}", e.dir.dst.0, e.dir.dst.1),
        "kind": e.kind,
        "key": e.key,
        "len": e.len,
        "info": e.info,
        "decoded": decoded,
        "hex": if e.payload.is_empty() { Value::Null } else { Value::String(hex(&e.payload, 32)) },
    })
}

// ---------------------------------------------------------------------------------------------
// pcap reading
// ---------------------------------------------------------------------------------------------

/// Read a pcap/pcapng source and feed the decoder. Events are delivered through `sink` in
/// arrival order (the decoder does not keep them).
pub fn run_pcap<R: Read + Send>(source: R, decoder: &mut Decoder, mut sink: impl FnMut(&Event)) -> Result<(), String> {
    let mut reader = create_reader(1 << 24, source).map_err(|e| format!("open pcap: {e:?}"))?;
    let mut legacy_linktype = Linktype::ETHERNET;
    let mut legacy_nanos = false;
    // pcapng: per interface (linktype, ts_offset, ts_resolution)
    let mut ifaces: Vec<(Linktype, u64, u64)> = Vec::new();
    let mut last_t = 0.0;
    loop {
        match reader.next() {
            Ok((offset, block)) => {
                match block {
                    PcapBlockOwned::LegacyHeader(h) => {
                        legacy_linktype = h.network;
                        legacy_nanos = matches!(h.magic_number, 0xa1b2_3c4d | 0x4d3c_b2a1);
                    }
                    PcapBlockOwned::Legacy(p) => {
                        let frac = if legacy_nanos { 1e-9 } else { 1e-6 };
                        let t = f64::from(p.ts_sec) + f64::from(p.ts_usec) * frac;
                        last_t = t;
                        decoder.feed_packet(t, legacy_linktype, p.data);
                    }
                    PcapBlockOwned::NG(Block::SectionHeader(_)) => ifaces.clear(),
                    PcapBlockOwned::NG(Block::InterfaceDescription(idb)) => {
                        let res = idb.ts_resolution().unwrap_or(1_000_000);
                        ifaces.push((idb.linktype, idb.if_tsoffset.max(0) as u64, res));
                    }
                    PcapBlockOwned::NG(Block::EnhancedPacket(epb)) => {
                        let (lt, off, res) = ifaces.get(epb.if_id as usize).copied().unwrap_or((Linktype::ETHERNET, 0, 1_000_000));
                        let t = epb.decode_ts_f64(off, res);
                        last_t = t;
                        decoder.feed_packet(t, lt, epb.data);
                    }
                    PcapBlockOwned::NG(Block::SimplePacket(spb)) => {
                        let lt = ifaces.first().map(|i| i.0).unwrap_or(Linktype::ETHERNET);
                        decoder.feed_packet(last_t, lt, spb.data);
                    }
                    PcapBlockOwned::NG(_) => {}
                }
                for e in decoder.events.drain(..) {
                    sink(&e);
                }
                reader.consume(offset);
            }
            Err(PcapError::Eof) => break,
            Err(PcapError::Incomplete(_)) => {
                reader.refill().map_err(|e| format!("read pcap: {e:?}"))?;
            }
            Err(e) => return Err(format!("read pcap: {e:?}")),
        }
    }
    for s in decoder.tcp.values() {
        decoder.stats.tcp_gaps += s.gaps;
    }
    Ok(())
}

fn print_summary(stats: &Stats) {
    let mut err = std::io::stderr().lock();
    let _ = writeln!(
        err,
        "# packets {}  batches {}  transport messages {}  decode errors {}  tcp gaps {}",
        stats.packets, stats.batches, stats.transport_msgs, stats.decode_errors, stats.tcp_gaps
    );
    let _ = writeln!(err, "# {:<60} {:>8} {:>10} {:>9}  lengths", "key", "count", "bytes", "rate/Hz");
    for (key, ks) in &stats.per_key {
        let span = ks.last_t - ks.first_t;
        let rate = if ks.count > 1 && span > 0.0 { (ks.count - 1) as f64 / span } else { 0.0 };
        let lens: Vec<String> = ks.lengths.iter().take(6).map(|(l, n)| format!("{l}×{n}")).collect();
        let _ = writeln!(err, "# {:<60} {:>8} {:>10} {:>9.1}  {}", key, ks.count, ks.bytes, rate, lens.join(" "));
    }
}

// ---------------------------------------------------------------------------------------------
// Tests with synthetic captures
// ---------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use apeiron_protocol::encode;
    use std::io::Cursor;
    use zenoh_buffers::ZBuf;
    use zenoh_codec::WCodec;
    use zenoh_protocol::core::Encoding;
    use zenoh_protocol::network::{ext as next, Push};
    use zenoh_protocol::transport::{frame::ext as fext, Frame};
    use zenoh_protocol::zenoh::Put;

    const KEY: &str = "wuji/TEST/joint/states";

    fn sample_frame() -> JointStateFrame {
        JointStateFrame {
            header: FrameHeader { seq: 42, timestamp_us: 1_700_000_000_123_456, frame_id: "l_wrist".into() },
            num_joints: 2,
            joints: vec![
                JointStateEntry { nid: 1, position: 0.5, velocity: -0.1, effort: 0.02 },
                JointStateEntry { nid: 24, position: 1.5, velocity: 0.0, effort: -0.3 },
            ],
        }
    }

    /// Build a Zenoh batch with one frame containing a push on KEY.
    fn zenoh_batch(payload: Vec<u8>) -> Vec<u8> {
        let push = Push {
            wire_expr: WireExpr { scope: 0, suffix: KEY.into(), mapping: Mapping::Sender },
            ext_qos: next::QoSType::DEFAULT,
            ext_tstamp: None,
            ext_nodeid: next::NodeIdType::DEFAULT,
            payload: PushBody::Put(Put { payload: ZBuf::from(payload), encoding: Encoding::empty(), ..Default::default() }),
        };
        let nm = NetworkMessage { body: NetworkBody::Push(push), reliability: Reliability::Reliable };
        let frame = Frame { reliability: Reliability::Reliable, sn: 1, ext_qos: fext::QoSType::DEFAULT, payload: vec![nm] };
        let tm = TransportMessage { body: TransportBody::Frame(frame) };
        let mut buf: Vec<u8> = Vec::new();
        Zenoh080::new().write(&mut buf, &tm).expect("write batch");
        buf
    }

    /// Ethernet + IPv4 + TCP segment with the given TCP payload.
    fn tcp_packet(seq: u32, tcp_payload: &[u8]) -> Vec<u8> {
        let mut p = Vec::new();
        p.extend_from_slice(&[0x02, 0, 0, 0, 0, 0x02]); // dst MAC
        p.extend_from_slice(&[0x02, 0, 0, 0, 0, 0x01]); // src MAC
        p.extend_from_slice(&0x0800u16.to_be_bytes());
        let total = 20 + 20 + tcp_payload.len();
        let mut ip = vec![0x45, 0];
        ip.extend_from_slice(&(total as u16).to_be_bytes());
        ip.extend_from_slice(&[0, 1, 0x40, 0, 64, 6, 0, 0]);
        ip.extend_from_slice(&[192, 168, 1, 50]);
        ip.extend_from_slice(&[192, 168, 1, 110]);
        p.extend_from_slice(&ip);
        let mut tcp = Vec::new();
        tcp.extend_from_slice(&40000u16.to_be_bytes());
        tcp.extend_from_slice(&50001u16.to_be_bytes());
        tcp.extend_from_slice(&seq.to_be_bytes());
        tcp.extend_from_slice(&0u32.to_be_bytes());
        tcp.extend_from_slice(&[0x50, 0x18, 0xff, 0xff, 0, 0, 0, 0]);
        p.extend_from_slice(&tcp);
        p.extend_from_slice(tcp_payload);
        p
    }

    /// Minimal legacy pcap (linktype 1) with the given frames.
    fn pcap_file(frames: &[(f64, Vec<u8>)]) -> Vec<u8> {
        let mut f = Vec::new();
        f.extend_from_slice(&0xa1b2_c3d4u32.to_le_bytes());
        f.extend_from_slice(&2u16.to_le_bytes());
        f.extend_from_slice(&4u16.to_le_bytes());
        f.extend_from_slice(&0i32.to_le_bytes());
        f.extend_from_slice(&0u32.to_le_bytes());
        f.extend_from_slice(&65535u32.to_le_bytes());
        f.extend_from_slice(&1u32.to_le_bytes());
        for (t, data) in frames {
            let sec = t.trunc() as u32;
            let usec = ((t - t.trunc()) * 1e6).round() as u32;
            f.extend_from_slice(&sec.to_le_bytes());
            f.extend_from_slice(&usec.to_le_bytes());
            f.extend_from_slice(&(data.len() as u32).to_le_bytes());
            f.extend_from_slice(&(data.len() as u32).to_le_bytes());
            f.extend_from_slice(data);
        }
        f
    }

    #[test]
    fn decodes_push_from_synthetic_pcap() {
        let frame = sample_frame();
        let payload = encode(&frame).unwrap();
        let batch = zenoh_batch(payload.clone());
        let mut tcp_payload = (batch.len() as u16).to_le_bytes().to_vec();
        tcp_payload.extend_from_slice(&batch);
        let pcap = pcap_file(&[(1700000000.25, tcp_packet(1000, &tcp_payload))]);

        let mut dec = Decoder::new();
        let mut events = Vec::new();
        run_pcap(Cursor::new(pcap), &mut dec, |e| events.push(e.clone())).unwrap();

        assert_eq!(dec.stats.batches, 1);
        assert_eq!(dec.stats.decode_errors, 0);
        assert_eq!(events.len(), 1);
        let e = &events[0];
        assert_eq!(e.kind, "push");
        assert_eq!(e.key, KEY);
        assert_eq!(e.payload, payload);
        assert_eq!(e.dir.src, (Ipv4Addr::new(192, 168, 1, 50), 40000));
        assert_eq!(e.dir.dst, (Ipv4Addr::new(192, 168, 1, 110), 50001));
        assert!((e.t - 1700000000.25).abs() < 1e-3);

        let j = event_json(e);
        assert_eq!(j["decoded"]["header"]["seq"], 42);
        assert_eq!(j["decoded"]["num_joints"], 2);
        assert_eq!(j["decoded"]["joints"][1]["nid"], 24);
        assert_eq!(dec.stats.per_key["push wuji/TEST/joint/states"].count, 1);
    }

    #[test]
    fn batch_split_over_two_tcp_segments_and_reordered() {
        let payload = encode(&sample_frame()).unwrap();
        let batch = zenoh_batch(payload);
        let mut stream = (batch.len() as u16).to_le_bytes().to_vec();
        stream.extend_from_slice(&batch);
        let (a, b) = stream.split_at(stream.len() / 2);
        // second segment appears first in the capture
        let pcap = pcap_file(&[(1.0, tcp_packet(1000 + a.len() as u32, b)), (1.001, tcp_packet(1000, a))]);
        let mut dec = Decoder::new();
        let mut n = 0;
        run_pcap(Cursor::new(pcap), &mut dec, |e| {
            assert_eq!(e.key, KEY);
            n += 1;
        })
        .unwrap();
        assert_eq!(n, 1);
        assert_eq!(dec.stats.decode_errors, 0);
    }

    #[test]
    fn declared_keyexpr_is_resolved_for_scoped_push() {
        use zenoh_protocol::network::{declare::DeclareKeyExpr, Declare};
        let decl = Declare {
            interest_id: None,
            ext_qos: next::QoSType::DEFAULT,
            ext_tstamp: None,
            ext_nodeid: next::NodeIdType::DEFAULT,
            body: DeclareBody::DeclareKeyExpr(DeclareKeyExpr {
                id: 7,
                wire_expr: WireExpr { scope: 0, suffix: "wuji/TEST".into(), mapping: Mapping::Sender },
            }),
        };
        let push = Push {
            wire_expr: WireExpr { scope: 7, suffix: "/imu".into(), mapping: Mapping::Sender },
            ext_qos: next::QoSType::DEFAULT,
            ext_tstamp: None,
            ext_nodeid: next::NodeIdType::DEFAULT,
            payload: PushBody::Put(Put { payload: ZBuf::from(vec![1u8, 2, 3]), ..Default::default() }),
        };
        let frame = Frame {
            reliability: Reliability::Reliable,
            sn: 1,
            ext_qos: fext::QoSType::DEFAULT,
            payload: vec![
                NetworkMessage { body: NetworkBody::Declare(decl), reliability: Reliability::Reliable },
                NetworkMessage { body: NetworkBody::Push(push), reliability: Reliability::Reliable },
            ],
        };
        let mut batch: Vec<u8> = Vec::new();
        Zenoh080::new().write(&mut batch, &TransportMessage { body: TransportBody::Frame(frame) }).unwrap();
        let dir = Dir { src: (Ipv4Addr::new(192, 168, 1, 110), 50001), dst: (Ipv4Addr::new(192, 168, 1, 50), 40000) };
        let mut dec = Decoder::new();
        dec.feed_batch(0.0, dir, &batch);
        assert_eq!(dec.events.len(), 2);
        assert_eq!(dec.events[0].kind, "declare");
        assert_eq!(dec.events[1].key, "wuji/TEST/imu");
    }
}
