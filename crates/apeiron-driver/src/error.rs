use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("zenoh: {0}")]
    Zenoh(String),
    #[error(transparent)]
    Protocol(#[from] apeiron_protocol::Error),
    #[error("device replied with an error for {resource}: {message}")]
    DeviceError { resource: String, message: String },
    #[error("timeout: {0}")]
    Timeout(&'static str),
    #[error("no hand found (no liveliness token under wuji/*/@alive)")]
    NoHand,
    #[error("several hands found: {0:?}; select one by serial number")]
    SeveralHands(Vec<String>),
    #[error("expected {expected} joints, got {got}")]
    JointCount { expected: usize, got: usize },
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
    #[error("{0}")]
    Other(String),
}

impl From<zenoh::Error> for Error {
    fn from(e: zenoh::Error) -> Self {
        Error::Zenoh(e.to_string())
    }
}

pub type Result<T> = std::result::Result<T, Error>;
