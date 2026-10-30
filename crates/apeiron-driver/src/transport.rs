//! Zenoh session setup.
//!
//! The hand runs a Zenoh 1.x client on `udp/<ip>:7447`. Zenoh does not allow two clients to
//! talk to each other directly, so the driver opens its session in router mode without any
//! listener and connects to the hand. Multicast scouting is disabled so that the session
//! never attaches to anything else on the lab network.

use std::fmt;

use zenoh::{Config, Session};

use crate::{Error, Result};

pub const DEFAULT_PORT: u16 = 7447;
pub const DEFAULT_PROTOCOL: &str = "udp";
/// Factory addresses: fourth serial character `J` = left, `K` = right.
pub const LEFT_IP: &str = "192.168.1.110";
pub const RIGHT_IP: &str = "192.168.1.111";

/// A Zenoh endpoint such as `udp/192.168.1.111:7447`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Endpoint {
    pub protocol: String,
    pub host: String,
    pub port: u16,
}

impl Endpoint {
    pub fn udp(host: impl Into<String>, port: u16) -> Self {
        Self { protocol: DEFAULT_PROTOCOL.into(), host: host.into(), port }
    }

    /// Accepts `udp/1.2.3.4:7447`, `tcp/...`, `1.2.3.4:7447` or a bare host.
    pub fn parse(s: &str) -> Result<Self> {
        let (protocol, rest) = match s.split_once('/') {
            Some((p, r)) => (p.to_string(), r),
            None => (DEFAULT_PROTOCOL.to_string(), s),
        };
        let (host, port) = match rest.rsplit_once(':') {
            Some((h, p)) if !h.contains(':') || h.starts_with('[') => {
                (h.to_string(), p.parse::<u16>().map_err(|_| Error::Other(format!("invalid port in {s}")))?)
            }
            _ => (rest.to_string(), DEFAULT_PORT),
        };
        if host.is_empty() {
            return Err(Error::Other(format!("empty host in {s}")));
        }
        Ok(Self { protocol, host, port })
    }
}

impl fmt::Display for Endpoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}:{}", self.protocol, self.host, self.port)
    }
}

/// Configuration that connects to exactly one endpoint.
pub fn config(endpoint: &Endpoint) -> Result<Config> {
    let mut config = Config::default();
    let mut set =
        |key: &str, json: &str| -> Result<()> { config.insert_json5(key, json).map_err(|e| Error::Zenoh(format!("config {key}: {e}"))) };
    set("mode", "\"router\"")?;
    set("listen/endpoints", "[]")?;
    set("connect/endpoints", &format!("[\"{endpoint}\"]"))?;
    set("scouting/multicast/enabled", "false")?;
    set("scouting/gossip/enabled", "false")?;
    // Exposes what the hand declared (subscribers, queryable) under @/<zid>/router/**.
    set("adminspace/enabled", "true")?;
    set("connect/timeout_ms", "5000")?;
    set("connect/exit_on_failure", "true")?;
    Ok(config)
}

pub async fn open(endpoint: &Endpoint) -> Result<Session> {
    let session = zenoh::open(config(endpoint)?).await?;
    tracing::info!(%endpoint, zid = %session.zid(), "session open");
    Ok(session)
}
