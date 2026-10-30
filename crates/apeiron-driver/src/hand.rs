//! Connection to one hand: discovery, streams, GET/SET, actions.

use std::collections::BTreeMap;
use std::time::Duration;

use apeiron_protocol::capability::{self, Node, Page, Resource, Summary};
use apeiron_protocol::joint::{JointId, NUM_JOINTS};
use apeiron_protocol::keys::{self, rel};
use apeiron_protocol::messages::*;
use apeiron_protocol::{decode, encode};
use serde::de::DeserializeOwned;
use zenoh::pubsub::Subscriber;
use zenoh::sample::Sample;
use zenoh::Session;

use crate::transport::{self, Endpoint};
use crate::{Error, Result};

pub struct Hand {
    session: Session,
    serial: String,
    endpoint: Endpoint,
}

/// Joint feedback of one stream frame, indexed by flat joint index.
#[derive(Debug, Clone)]
pub struct JointSnapshot {
    pub seq: u32,
    pub timestamp_us: u64,
    pub joints: [Option<RealtimeStreamEntry>; NUM_JOINTS],
}

impl JointSnapshot {
    pub fn positions(&self) -> [f32; NUM_JOINTS] {
        std::array::from_fn(|i| self.joints[i].map(|j| j.pos_rad_jnt_fb).unwrap_or(f32::NAN))
    }

    pub fn complete(&self) -> bool {
        self.joints.iter().all(Option::is_some)
    }
}

impl From<RealtimeStreamFrame> for JointSnapshot {
    fn from(f: RealtimeStreamFrame) -> Self {
        let mut joints = [None; NUM_JOINTS];
        for j in f.joints {
            if let Ok(id) = JointId::from_nid(j.nid) {
                joints[id.index()] = Some(j);
            }
        }
        JointSnapshot { seq: f.header.seq, timestamp_us: f.header.timestamp_us, joints }
    }
}

impl Hand {
    /// Connect to `endpoint`. Without `serial` the hand is identified through its liveliness
    /// token; exactly one hand must be reachable.
    pub async fn connect(endpoint: &Endpoint, serial: Option<String>) -> Result<Hand> {
        let session = transport::open(endpoint).await?;
        let serial = match serial {
            Some(s) => s,
            None => {
                let mut found = liveliness(&session, Duration::from_secs(3)).await?;
                found.sort();
                found.dedup();
                match found.as_slice() {
                    [one] => one.clone(),
                    [] => return Err(Error::NoHand),
                    many => return Err(Error::SeveralHands(many.to_vec())),
                }
            }
        };
        tracing::info!(%serial, "hand connected");
        Ok(Hand { session, serial, endpoint: endpoint.clone() })
    }

    pub fn serial(&self) -> &str {
        &self.serial
    }

    pub fn endpoint(&self) -> &Endpoint {
        &self.endpoint
    }

    pub fn session(&self) -> &Session {
        &self.session
    }

    pub fn key(&self, rel: &str) -> String {
        keys::device_key(&self.serial, rel)
    }

    pub fn node_key(&self, joint: JointId, rel: &str) -> String {
        keys::node_key(&self.serial, &joint.node_path(), rel)
    }

    // ---------------------------------------------------------------- streams

    pub async fn subscribe<T: DeserializeOwned>(&self, rel: &str) -> Result<Stream<T>> {
        let inner = self.session.declare_subscriber(self.key(rel)).await?;
        Ok(Stream { inner, _t: std::marker::PhantomData })
    }

    pub async fn realtime_stream(&self) -> Result<Stream<RealtimeStreamFrame>> {
        self.subscribe(rel::REALTIME_STREAM).await
    }

    pub async fn imu(&self) -> Result<Stream<ImuData>> {
        self.subscribe(rel::IMU).await
    }

    /// One complete joint snapshot from the realtime stream.
    pub async fn snapshot(&self) -> Result<JointSnapshot> {
        let stream = self.realtime_stream().await?;
        let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
        let mut merged: Option<JointSnapshot> = None;
        while tokio::time::Instant::now() < deadline {
            let (_, frame) = stream.recv_timeout(Duration::from_millis(500)).await?;
            let Ok(frame) = frame else { continue };
            let snap = JointSnapshot::from(frame);
            match &mut merged {
                None => merged = Some(snap),
                Some(m) => {
                    for (slot, j) in m.joints.iter_mut().zip(snap.joints) {
                        if j.is_some() {
                            *slot = j;
                        }
                    }
                    m.seq = snap.seq;
                    m.timestamp_us = snap.timestamp_us;
                }
            }
            if merged.as_ref().is_some_and(JointSnapshot::complete) {
                break;
            }
        }
        merged.filter(JointSnapshot::complete).ok_or(Error::Timeout("incomplete joint snapshot"))
    }

    pub async fn positions(&self) -> Result<[f32; NUM_JOINTS]> {
        Ok(self.snapshot().await?.positions())
    }

    // -------------------------------------------------------------------- GET

    pub async fn get_raw(&self, rel: &str) -> Result<Vec<u8>> {
        self.query(rel, None).await
    }

    pub async fn get<T: DeserializeOwned>(&self, rel: &str) -> Result<T> {
        Ok(decode(&self.get_raw(rel).await?)?)
    }

    /// Little-endian f32 register of a joint node, e.g. `od/pos_rad_jnt_enc2`.
    pub async fn get_node_f32(&self, joint: JointId, rel: &str) -> Result<f32> {
        let b = self.query(&format!("{}/{rel}", joint.node_path()), None).await?;
        let arr: [u8; 4] = b.as_slice().try_into().map_err(|_| Error::Other(format!("{rel}: expected 4 bytes, got {}", b.len())))?;
        Ok(f32::from_le_bytes(arr))
    }

    pub async fn hw_version(&self) -> Result<HwVersion> {
        self.get(rel::HW_VERSION).await
    }

    pub async fn handedness(&self) -> Result<Handedness> {
        Ok(Handedness::from(self.get::<u8>(rel::HANDEDNESS).await?))
    }

    pub async fn input_voltage(&self) -> Result<f32> {
        self.get(rel::INPUT_VOLTAGE).await
    }

    pub async fn mit_params(&self) -> Result<Vec<MitParam>> {
        self.get(rel::MIT_PARAMS).await
    }

    pub async fn effort_limit(&self) -> Result<Vec<f32>> {
        self.get(rel::EFFORT_LIMIT).await
    }

    pub async fn control_mode(&self) -> Result<Vec<u8>> {
        self.get(rel::CONTROL_MODE).await
    }

    // -------------------------------------------------------------------- SET

    /// SET = query with payload; the queryable answers with a [`SetReply`].
    pub async fn set_raw(&self, rel: &str, payload: Vec<u8>) -> Result<Vec<u8>> {
        self.query(rel, Some(payload)).await
    }

    pub async fn set(&self, rel: &str, payload: Vec<u8>) -> Result<SetReply> {
        Ok(decode(&self.set_raw(rel, payload).await?)?)
    }

    /// Action = SET with an empty payload.
    pub async fn action(&self, rel: &str) -> Result<SetReply> {
        self.set(rel, Vec::new()).await
    }

    pub async fn motor_enable(&self) -> Result<SetReply> {
        self.action(rel::MOTOR_ENABLE).await
    }

    pub async fn motor_disable(&self) -> Result<SetReply> {
        self.action(rel::MOTOR_DISABLE).await
    }

    pub async fn clear_fault(&self) -> Result<SetReply> {
        self.action(rel::CLEAR_FAULT).await
    }

    pub async fn set_effort_limit(&self, limits: &[f32]) -> Result<SetReply> {
        expect_joints(limits.len())?;
        self.set(rel::EFFORT_LIMIT, encode(&limits.to_vec())?).await
    }

    pub async fn set_mit_params(&self, params: &[MitParam]) -> Result<SetReply> {
        expect_joints(params.len())?;
        self.set(rel::MIT_PARAMS, encode(&params.to_vec())?).await
    }

    // ------------------------------------------------------------- capability

    pub async fn capability_summary(&self) -> Result<Summary> {
        Ok(capability::decode_summary(&self.get_raw(&capability::selector("summary", None)).await?)?)
    }

    /// All resources (`node_types` pages), with the page scope attached.
    pub async fn capability_resources(&self) -> Result<Vec<(u32, Resource)>> {
        let mut out = Vec::new();
        let mut index = 0;
        loop {
            let page = Page::<Resource>::decode(&self.get_raw(&capability::selector("node_types", Some(index))).await?)?;
            out.extend(page.entries.into_iter().map(|r| (page.scope, r)));
            index += 1;
            if index >= page.total {
                break;
            }
        }
        Ok(out)
    }

    pub async fn capability_nodes(&self) -> Result<Vec<Node>> {
        let mut out = Vec::new();
        let mut index = 0;
        loop {
            let page = Page::<Node>::decode(&self.get_raw(&capability::selector("nodes", Some(index))).await?)?;
            out.extend(page.entries);
            index += 1;
            if index >= page.total {
                break;
            }
        }
        Ok(out)
    }

    /// Subscribers and queryables the hand declared, from the router admin space.
    pub async fn declarations(&self) -> Result<BTreeMap<String, String>> {
        let replies = self.session.get("@/*/router/**").timeout(Duration::from_secs(2)).await?;
        let mut out = BTreeMap::new();
        while let Ok(reply) = replies.recv_async().await {
            if let Ok(sample) = reply.result() {
                let key = sample.key_expr().to_string();
                if let Some(kind) = key.split("/router/").nth(1) {
                    if kind.starts_with("subscriber/") || kind.starts_with("queryable/") || kind.starts_with("publisher/") {
                        out.insert(kind.to_string(), sample.payload().try_to_string().map(|s| s.into_owned()).unwrap_or_default());
                    }
                }
            }
        }
        Ok(out)
    }

    pub async fn close(self) -> Result<()> {
        self.session.close().await?;
        Ok(())
    }

    async fn query(&self, rel: &str, payload: Option<Vec<u8>>) -> Result<Vec<u8>> {
        let key = self.key(rel);
        let builder = self.session.get(&key).timeout(Duration::from_secs(2));
        let replies = match payload {
            Some(p) => builder.payload(p).await?,
            None => builder.await?,
        };
        let mut last_error = None;
        while let Ok(reply) = replies.recv_async().await {
            match reply.result() {
                Ok(sample) => return Ok(sample.payload().to_bytes().to_vec()),
                Err(e) => {
                    last_error = Some(e.payload().try_to_string().map(|s| s.into_owned()).unwrap_or_else(|_| hex(&e.payload().to_bytes())))
                }
            }
        }
        match last_error {
            Some(message) => Err(Error::DeviceError { resource: rel.to_string(), message }),
            None => Err(Error::Timeout("query without reply")),
        }
    }
}

/// Serial numbers announced through `wuji/*/@alive`.
pub async fn liveliness(session: &Session, timeout: Duration) -> Result<Vec<String>> {
    let replies = session.liveliness().get(keys::ALIVE_ALL).timeout(timeout).await?;
    let mut out = Vec::new();
    while let Ok(reply) = replies.recv_async().await {
        if let Ok(sample) = reply.result() {
            if let Some(sn) = keys::serial_from_key(sample.key_expr().as_str()) {
                out.push(sn.to_string());
            }
        }
    }
    Ok(out)
}

/// Typed subscriber: every sample is returned as raw bytes plus the decoding result.
pub struct Stream<T> {
    inner: Subscriber<zenoh::handlers::FifoChannelHandler<Sample>>,
    _t: std::marker::PhantomData<T>,
}

impl<T: DeserializeOwned> Stream<T> {
    pub async fn recv(&self) -> Result<(Vec<u8>, Result<T>)> {
        let sample = self.inner.recv_async().await.map_err(|_| Error::Other("subscriber closed".into()))?;
        let bytes = sample.payload().to_bytes().to_vec();
        let value = decode(&bytes).map_err(Error::from);
        Ok((bytes, value))
    }

    pub async fn recv_timeout(&self, timeout: Duration) -> Result<(Vec<u8>, Result<T>)> {
        tokio::time::timeout(timeout, self.recv()).await.map_err(|_| Error::Timeout("stream"))?
    }

    pub fn key_expr(&self) -> String {
        self.inner.key_expr().to_string()
    }
}

fn expect_joints(n: usize) -> Result<()> {
    if n == NUM_JOINTS {
        Ok(())
    } else {
        Err(Error::JointCount { expected: NUM_JOINTS, got: n })
    }
}

pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}
