//! Joint addressing.
//!
//! The firmware addresses motor boards by node id `nid = bus * 5 + node`, where bus 0..5 is
//! the finger (thumb, index, middle, ring, pinky) and node 1..=4 is the joint segment;
//! node 5 is the finger's tactile sensor and never appears in joint data. Host-side APIs
//! use a flat, finger-major index 0..20 instead (thumb 0..4, index 4..8, …).
//!
//! Segment semantics (verified by hand on a right hand):
//!
//! | segment | joint | positive direction |
//! |---|---|---|
//! | S1 | base flexion (MCP) | flex (inwards) |
//! | S2 | base abduction | away from the thumb |
//! | S3 | middle joint (PIP) | flex |
//! | S4 | fingertip (DIP) | flex |

use crate::{Error, Result};

pub const NUM_JOINTS: usize = 20;
pub const NUM_FINGERS: usize = 5;
pub const JOINTS_PER_FINGER: usize = 4;
const NODES_PER_BUS: u8 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Finger {
    Thumb = 0,
    Index = 1,
    Middle = 2,
    Ring = 3,
    Pinky = 4,
}

impl Finger {
    pub const ALL: [Finger; NUM_FINGERS] = [Finger::Thumb, Finger::Index, Finger::Middle, Finger::Ring, Finger::Pinky];

    pub fn name(self) -> &'static str {
        match self {
            Finger::Thumb => "thumb",
            Finger::Index => "index",
            Finger::Middle => "middle",
            Finger::Ring => "ring",
            Finger::Pinky => "pinky",
        }
    }

    pub fn from_bus(bus: u8) -> Option<Finger> {
        Finger::ALL.get(bus as usize).copied()
    }

    pub fn from_name(name: &str) -> Option<Finger> {
        Finger::ALL.iter().copied().find(|f| f.name().eq_ignore_ascii_case(name))
    }
}

/// A joint identified by finger and segment (1..=4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct JointId {
    pub finger: Finger,
    pub segment: u8,
}

impl JointId {
    /// Human-readable label such as `index_S1`.
    pub fn label(self) -> String {
        format!("{}_S{}", self.finger.name(), self.segment)
    }

    /// Resource path prefix of the joint's node, e.g. `index/S1`.
    pub fn node_path(self) -> String {
        format!("{}/S{}", self.finger.name(), self.segment)
    }

    pub fn index(self) -> usize {
        self.finger as usize * JOINTS_PER_FINGER + (self.segment as usize - 1)
    }

    pub fn nid(self) -> u8 {
        self.finger as u8 * NODES_PER_BUS + self.segment
    }

    pub fn from_index(index: usize) -> Result<JointId> {
        if index >= NUM_JOINTS {
            return Err(Error::InvalidJointIndex(index));
        }
        Ok(JointId {
            finger: Finger::from_bus((index / JOINTS_PER_FINGER) as u8).expect("bus < 5"),
            segment: (index % JOINTS_PER_FINGER) as u8 + 1,
        })
    }

    pub fn from_nid(nid: u8) -> Result<JointId> {
        let (bus, node) = (nid / NODES_PER_BUS, nid % NODES_PER_BUS);
        match Finger::from_bus(bus) {
            Some(finger) if (1..=4).contains(&node) => Ok(JointId { finger, segment: node }),
            _ => Err(Error::InvalidNid(nid)),
        }
    }

    /// Parse an index (`"4"`) or a label (`"index_S1"`, case-insensitive).
    pub fn parse(s: &str) -> Result<JointId> {
        if let Ok(i) = s.parse::<usize>() {
            return JointId::from_index(i);
        }
        let (finger, seg) = s.rsplit_once("_S").ok_or(Error::InvalidJointIndex(usize::MAX))?;
        let finger = Finger::from_name(finger).ok_or(Error::InvalidJointIndex(usize::MAX))?;
        let segment: u8 = seg.parse().map_err(|_| Error::InvalidJointIndex(usize::MAX))?;
        if !(1..=4).contains(&segment) {
            return Err(Error::InvalidJointIndex(usize::MAX));
        }
        Ok(JointId { finger, segment })
    }
}

/// Firmware node id → flat joint index.
pub fn nid_to_index(nid: u8) -> Result<usize> {
    JointId::from_nid(nid).map(JointId::index)
}

/// Flat joint index → firmware node id.
pub fn index_to_nid(index: usize) -> Result<u8> {
    JointId::from_index(index).map(JointId::nid)
}

/// True for the tactile-sensor slot of a finger (node 5).
pub fn is_tactile_nid(nid: u8) -> bool {
    nid % NODES_PER_BUS == 0 && (NODES_PER_BUS..=NUM_FINGERS as u8 * NODES_PER_BUS).contains(&nid)
}

/// Approximate mechanical range of a joint in radians, measured on a right hand, with a
/// small safety margin. Thumb S1/S2 differ from the other fingers.
pub fn approx_limits(index: usize) -> Result<(f32, f32)> {
    let id = JointId::from_index(index)?;
    let thumb = id.finger == Finger::Thumb;
    Ok(match id.segment {
        1 if thumb => (-1.15, 0.75),
        1 => (-1.0, 1.4),
        2 if thumb => (-1.25, 0.15),
        2 => (-0.6, 0.6),
        3 => (-1.0, 1.5),
        _ => (-0.85, 1.5),
    })
}

/// Clamp `target` into [`approx_limits`]; returns the clamped value and whether it changed.
pub fn clamp_to_limits(index: usize, target: f32) -> Result<(f32, bool)> {
    let (lo, hi) = approx_limits(index)?;
    let c = target.clamp(lo, hi);
    Ok((c, c != target))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nid_roundtrip() {
        let nids: [u8; 20] = [1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24];
        for (i, nid) in nids.iter().enumerate() {
            assert_eq!(index_to_nid(i).unwrap(), *nid);
            assert_eq!(nid_to_index(*nid).unwrap(), i);
        }
        for t in [5u8, 10, 15, 20, 25] {
            assert!(is_tactile_nid(t));
            assert!(nid_to_index(t).is_err());
        }
    }
}
