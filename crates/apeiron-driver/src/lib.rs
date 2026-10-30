//! Driver for the Wuji Hand 2.
//!
//! Layers, bottom up:
//!
//! - [`transport`]: opens the Zenoh session (the hand is a Zenoh *client* on
//!   `udp/<ip>:7447`; the driver connects as a router without listeners).
//! - [`Hand`]: connection to one hand – streams, GET/SET, actions.
//! - [`profile`]: per-hand tuning (gains, effort limits, preload) stored as JSON.
//! - [`motion`]: poses, sequences and smooth trajectories.
//! - [`control`]: cyclic control loop with selectable [`control::ControlMode`]; middleware
//!   plugs in through [`control::ControlHandle`] or [`control::ControlSource`].
//! - [`commissioning`]: self-test, gain tuning and preload test that produce a profile.
//!
//! Anything that moves the hand lives in `control` and `commissioning`; everything else
//! is read-only or changes parameters only.

#![allow(clippy::needless_range_loop, clippy::too_many_arguments)]

pub mod commissioning;
pub mod control;
pub mod error;
pub mod hand;
pub mod motion;
pub mod profile;
pub mod transport;

pub use apeiron_protocol as protocol;
pub use error::{Error, Result};
pub use hand::Hand;
pub use profile::HandProfile;
pub use transport::Endpoint;
