//! Wire protocol of the Wuji Hand 2, independent of any transport.
//!
//! The hand is a Zenoh node; application payloads are `postcard`-encoded Rust structs.
//! This crate contains everything needed to produce and consume those payloads:
//!
//! - [`messages`]: the message layouts (streams, commands, parameters, replies)
//! - [`keys`]: key expressions under `wuji/<serial>/…`
//! - [`joint`]: mapping between firmware node ids and the flat joint index 0..20
//! - [`capability`]: parsing of the device's self-description pages
//! - [`error_codes`]: names and descriptions of the joint error codes
//!
//! All layouts were recovered from the vendor SDK binary and verified against a
//! hand running firmware 2.2.2 (hardware revision 0.2.0).

pub mod capability;
pub mod error_codes;
pub mod joint;
pub mod keys;
pub mod messages;

use serde::{de::DeserializeOwned, Serialize};

/// Errors produced while encoding or decoding payloads.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("postcard: {0}")]
    Postcard(#[from] postcard::Error),
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
    #[error("invalid node id {0} (joints are bus*5+node with node in 1..=4)")]
    InvalidNid(u8),
    #[error("invalid joint index {0} (expected 0..20)")]
    InvalidJointIndex(usize),
    #[error("malformed capability page: {0}")]
    Capability(&'static str),
}

pub type Result<T> = std::result::Result<T, Error>;

/// Encode a message with postcard (the hand's default serialization).
pub fn encode<T: Serialize>(value: &T) -> Result<Vec<u8>> {
    Ok(postcard::to_stdvec(value)?)
}

/// Decode a postcard payload.
pub fn decode<T: DeserializeOwned>(bytes: &[u8]) -> Result<T> {
    Ok(postcard::from_bytes(bytes)?)
}
