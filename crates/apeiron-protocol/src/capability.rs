//! Self-description of the hand (`wuji/<serial>/@capability`).
//!
//! Three query kinds exist; `kind` and `index` are passed as selector parameters:
//!
//! | selector | payload |
//! |---|---|
//! | `@capability?kind=summary` | JSON [`Summary`] |
//! | `@capability?kind=node_types&index=N` | postcard [`Page`] of [`Resource`] entries |
//! | `@capability?kind=nodes&index=N` | postcard [`Page`] of [`Node`] entries |
//!
//! Pages are fetched by incrementing `index` until `index == total`. A query without
//! parameters is answered with the reply error `capability parameters required`.

use serde::{Deserialize, Serialize};

use crate::{Error, Result};

/// `?kind=summary`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Summary {
    pub device_id: u32,
    pub serial_number: String,
    pub firmware_version: String,
    pub uptime_s: u64,
    #[serde(default)]
    pub flag: u32,
}

/// Access rights of a resource (bit mask in [`Resource::access`]).
pub mod access {
    pub const GET: u8 = 0x01;
    pub const SET: u8 = 0x02;
    pub const SUB: u8 = 0x04;
    pub const PUB: u8 = 0x08;
    pub const ACTION: u8 = 0x10;
}

/// One resource entry of a `node_types` page.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Resource {
    pub resource_id: u32,
    pub name: String,
    pub access: u8,
    pub schema_id: u32,
    pub format: u32,
    pub version: u32,
}

impl Resource {
    pub fn can(&self, flag: u8) -> bool {
        self.access & flag != 0
    }
}

/// One entry of a `nodes` page.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Node {
    pub node_id: u32,
    pub type_id: u32,
    /// `/`, `/thumb/S1`, `/thumb/tactile`, …
    pub name: String,
}

/// Page header followed by entries. `node_types` pages carry `type_id` and `scope`
/// (0 = device, 1 = joint node, 2 = tactile node); `nodes` pages do not (both 0).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Page<T> {
    pub status: u8,
    pub total: u32,
    pub index: u32,
    pub type_id: u32,
    pub scope: u32,
    pub entries: Vec<T>,
}

#[derive(Deserialize)]
struct RawResourcePage {
    status: u8,
    total: u32,
    index: u32,
    type_id: u32,
    scope: u32,
    entries: Vec<Resource>,
}

#[derive(Deserialize)]
struct RawNodePage {
    status: u8,
    total: u32,
    index: u32,
    entries: Vec<Node>,
}

impl Page<Resource> {
    pub fn decode(bytes: &[u8]) -> Result<Page<Resource>> {
        let r: RawResourcePage = postcard::from_bytes(bytes).map_err(|_| Error::Capability("node_types page"))?;
        Ok(Page { status: r.status, total: r.total, index: r.index, type_id: r.type_id, scope: r.scope, entries: r.entries })
    }
}

impl Page<Node> {
    pub fn decode(bytes: &[u8]) -> Result<Page<Node>> {
        let r: RawNodePage = postcard::from_bytes(bytes).map_err(|_| Error::Capability("nodes page"))?;
        Ok(Page { status: r.status, total: r.total, index: r.index, type_id: 0, scope: 0, entries: r.entries })
    }
}

pub fn decode_summary(bytes: &[u8]) -> Result<Summary> {
    Ok(serde_json::from_slice(bytes)?)
}

/// Selector for a page query, e.g. `@capability?kind=nodes&index=3`.
pub fn selector(kind: &str, index: Option<u32>) -> String {
    match index {
        Some(i) => format!("{}?kind={kind}&index={i}", crate::keys::rel::CAPABILITY),
        None => format!("{}?kind={kind}", crate::keys::rel::CAPABILITY),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_nodes_page() {
        // First `nodes` page as captured from a right hand.
        let hex = "00050006000001 2f 0101092f7468756d622f5331 0201092f7468756d622f5332 0301092f7468756d622f5333 0401092f7468756d622f5334 05020e2f7468756d622f74616374696c65";
        let bytes: Vec<u8> = hex
            .split_whitespace()
            .flat_map(|h| (0..h.len()).step_by(2).map(move |i| u8::from_str_radix(&h[i..i + 2], 16).unwrap()))
            .collect();
        let page = Page::<Node>::decode(&bytes).unwrap();
        assert_eq!((page.total, page.index, page.entries.len()), (5, 0, 6));
        assert_eq!(page.entries[0].name, "/");
        assert_eq!(page.entries[1], Node { node_id: 1, type_id: 1, name: "/thumb/S1".into() });
        assert_eq!(page.entries[5], Node { node_id: 5, type_id: 2, name: "/thumb/tactile".into() });
    }

    #[test]
    fn decodes_resource_entries() {
        let hex = "004b00010002 0110 6669726d776172655f76657273696f6e 010c0200 05 15 6d61737465725f76657273696f6e5f64657461696c 01110200";
        let bytes: Vec<u8> = hex
            .split_whitespace()
            .flat_map(|h| (0..h.len()).step_by(2).map(move |i| u8::from_str_radix(&h[i..i + 2], 16).unwrap()))
            .collect();
        let page = Page::<Resource>::decode(&bytes).unwrap();
        assert_eq!(page.total, 75);
        assert_eq!(
            page.entries[0],
            Resource { resource_id: 1, name: "firmware_version".into(), access: 1, schema_id: 12, format: 2, version: 0 }
        );
        assert!(page.entries[0].can(access::GET));
        assert_eq!(page.entries[1].name, "master_version_detail");
    }
}
