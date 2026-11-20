//! The cable page model: the section arithmetic and the 30-column row
//! grids, kws-x1's firmware/src/pd/cable.rs transcribed onto the data the
//! pd_task publishes (one `CableData` copy instead of their live
//! Observation). The renderer in ui.rs draws the grids verbatim; encoded
//! values are hexadecimal claims, not electrical measurements.
//!
//! Deviations from kws-x1, both forced by fnb58 facts: the meter-plug /
//! cable-test branch is absent (fnb58 has no emarker boot test), and the
//! ask hint reads OK (a press) instead of HOLD OK (fnb58 has no long-press;
//! the r226 energy-reset precedent).

use pd_protocol::pd_cable::{CableReport, CableStatus};
use pd_protocol::pd_cable_modes::Discovery;
use pd_protocol::pd_spr::Supply;

use crate::cable_summary;
use crate::pd_task::CableData;

/// One cable page section, wide rows so summary values fit on one line
/// (kws-x1's WIDE_COLUMNS; 30 cells at scale 1 = 180 px on this panel).
pub const COLUMNS: usize = 30;
pub type Lines = [[u8; COLUMNS]; 8];

/// USB PD vSafe5V nominal, in mV: cable reads run only on a fixed 5V
/// contract.
const VSAFE5V_MV: u16 = 5000;
/// The charger's PD major revision a cable query needs, as
/// `peer_revision` records it (3 = PD 3.0).
const PD3: u8 = 3;
/// Rows below a section's heading.
const SECTION_ROWS: usize = 7;
/// Sections for the readable summary ahead of the raw identity, enough for
/// every item, the alt modes and the maker name.
const SUMMARY_SECTIONS: usize =
    (cable_summary::MAX_ITEMS + 1 + MAKER_ROWS).div_ceil(SECTION_ROWS);
/// Summary values start one space after the longest label.
const SUMMARY_VALUE: usize = 9;
/// USB PD R3.2's Manufacturer Info Data Block: VID and PID, then a
/// manufacturer string of at most 22 bytes, `Not Supported` when the cable
/// has none to give.
const MAKER_START: usize = 4;
const MAKER_UNSUPPORTED: &[u8] = b"Not Supported";
/// Rows a 22-byte maker name takes beside its label.
const MAKER_ROWS: usize = 1;

/// Sections of the cable list: the readable summary, identity fields six
/// per section, the latest SVID reply in two, two per SVID for its modes,
/// then manufacturer info and both cable statuses; one section while
/// there is no identity. (kws-x1's `cable_sections`, minus the
/// passthrough/meter-plug branch the caller handles.)
pub fn sections(data: &CableData) -> u32 {
    use pd_protocol::pd_cable_details;
    let Some(report) = data.report else {
        return 1;
    };
    match pd_cable_details::field_count(&report) {
        0 => 1,
        fields => {
            (SUMMARY_SECTIONS
                + fields.div_ceil(6)
                + 2
                + usize::from(data.modes.count) * 2
                + 3) as u32
        }
    }
}

/// Selected SVID's ascending-set index for the section on view; no bitmap
/// is copied into the data (kws-x1's `cable_svid_index`).
pub fn svid_index(data: &CableData, page: u32) -> Option<u16> {
    use pd_protocol::pd_cable_details;
    let report = data.report?;
    let fields = pd_cable_details::field_count(&report);
    if fields == 0 {
        return None;
    }
    let start = SUMMARY_SECTIONS + fields.div_ceil(6) + 2;
    let end = start + usize::from(data.modes.count) * 2;
    let page = page as usize;
    (start..end)
        .contains(&page)
        .then(|| ((page - start) / 2) as u16)
}

/// Section `page` of [`sections`]: readable summary, raw identity, latest
/// SVID reply and selected mode, plus decoded identity fields. Encoded
/// values use hexadecimal. A section that continues the previous one has
/// no heading. `svid`: the store's SVID at the mode cursor.
pub fn lines(data: &CableData, page: u32, svid: Option<u16>, svids: &dyn pd_protocol::pd_svids::Store) -> Lines {
    use pd_protocol::{pd_cable_details, pd_cable_modes::Status};
    let mut rows = [[b' '; COLUMNS]; 8];
    put(&mut rows[0], b"E-MARKER DETAILS");
    if !data.spr {
        // No SPR observation at all (kws-x1's `let Some(spr) =
        // observation.spr else`): nothing has run, whatever was asked.
        put(&mut rows[1], b"NO DISCOVERY YET");
        return rows;
    }
    let Some(report) = data.report else {
        // A cable is only asked under a PD 3.0 contract (the PD task drops
        // the query otherwise); once asked, say why nothing could start.
        let refused = match data.peer_revision {
            _ if !data.asked => None,
            // No PD contract on this port.
            None => Some(&b"NO PD ON THIS PORT"[..]),
            Some(revision) if revision < PD3 => Some(&b"PD 2.0 ON THIS PORT"[..]),
            Some(_) => None,
        };
        if data.failed {
            put(&mut rows[1], b"PD HALTED SEE PD PAGE");
        } else if data.active {
            put(&mut rows[1], b"DISCOVERY IN PROGRESS");
        } else if let Some(refused) = refused {
            put(&mut rows[1], refused);
            put(&mut rows[2], b"E-MARKER NOT READ");
            put(&mut rows[3], b"NEEDS A PD 3.0 PORT");
        } else {
            // fnb58 has no long-press (the r226 energy-reset precedent):
            // the ask is the browse mode's OK.
            put(&mut rows[1], b"OK TO ASK AT 5V");
        }
        return rows;
    };
    let fields = pd_cable_details::field_count(&report);
    if fields == 0 {
        if report.status == CableStatus::Identity {
            put(&mut rows[1], b"NO VALID IDENTITY");
        } else {
            let (reason, hint) = failure(report.status, data.modes.sync);
            put(&mut rows[1], reason);
            put(&mut rows[2], hint);
            // The last NAK's VDM Header and the revision it came back at,
            // so a rejected request can be told apart from the photo.
            if report.status == CableStatus::Nak && report.count > 0 {
                put(&mut rows[2], b"VDM 00000000 PD?");
                hex(&mut rows[2], 4, (report.objects[0] >> 16) as u16);
                hex(&mut rows[2], 8, report.objects[0] as u16);
                rows[2][15] = match report.revision {
                    1 => b'2',
                    2 => b'3',
                    _ => b'?',
                };
            }
        }
        return rows;
    }
    let identity_pages = fields.div_ceil(6);
    let discovery = &data.modes;
    let svid_pages = 2; // latest raw reply, not a transcript of every list packet
    let extended_start = identity_pages + svid_pages + usize::from(discovery.count) * 2;
    let page = page as usize;
    if page < SUMMARY_SECTIONS {
        return summary_lines(&report, discovery, svids, page);
    }
    let page = page - SUMMARY_SECTIONS;
    if page < identity_pages {
        if page > 0 {
            rows[0].fill(b' ');
        }
        for row in 0..6 {
            if let Some(field) = pd_cable_details::field(&report, page * 6 + row) {
                let name = field.name.as_bytes();
                put(&mut rows[row + 1], &name[..name.len().min(15)]);
                hex(&mut rows[row + 1], 16, (field.value >> 16) as u16);
                hex(&mut rows[row + 1], 20, field.value as u16);
            }
        }
    } else if page >= extended_start {
        let manufacturer = page == extended_start;
        let second = page == extended_start + 2;
        let info = if manufacturer {
            discovery.manufacturer
        } else if second {
            discovery.second_status
        } else {
            discovery.cable_status
        };
        rows[0].fill(b' ');
        put(
            &mut rows[0],
            if manufacturer {
                b"MANUFACTURER INFO"
            } else if second {
                b"SOP2 CABLE STATUS"
            } else {
                b"ACTIVE CABLE STATUS"
            },
        );
        if second {
            rows[7].fill(b' ');
            put(&mut rows[7], b"SYNC:");
            let caption = status_caption(discovery.second_sync);
            rows[7][5..5 + caption.len()].copy_from_slice(caption);
            if info.status != Status::Ack {
                put(
                    &mut rows[2],
                    match pd_protocol::pd_cable::second_controller(&report) {
                        Some(false) => b"NO SOP2 ADVERTISED",
                        None => b"LAYOUT/PRESENCE UNKNOWN",
                        Some(true) if report.revision != fusb302_pd::header::REVISION_3 => {
                            b"PD2 NO GET_STATUS"
                        }
                        Some(true) if data.failed => b"PD HALTED SEE PD PAGE",
                        Some(true) => b"SECOND PLUG NO DATA",
                    },
                );
            }
        }
        put(&mut rows[1], status_caption(info.status));
        hex(&mut rows[1], 20, info.header);
        if info.status == Status::Ack {
            if manufacturer {
                put(&mut rows[2], b"VID:---- PID:----");
                hex(
                    &mut rows[2],
                    4,
                    u16::from_le_bytes([info.data[0], info.data[1]]),
                );
                hex(
                    &mut rows[2],
                    13,
                    u16::from_le_bytes([info.data[2], info.data[3]]),
                );
            } else {
                put(
                    &mut rows[2],
                    match info.data[0] {
                        0 => b"T:N/A SHUTDOWN:-",
                        1 => b"T:LOW SHUTDOWN:-",
                        _ => b"T:--C SHUTDOWN:-",
                    },
                );
                if info.data[0] >= 2 {
                    rows[2][2] = b"0123456789ABCDEF"[(info.data[0] >> 4) as usize];
                    rows[2][3] = b"0123456789ABCDEF"[(info.data[0] & 15) as usize];
                }
                rows[2][15] = b'0' + (info.data[1] & 1);
            }
            for (index, &byte) in info.data[..info.count as usize].iter().enumerate() {
                let row = 3 + index / 8;
                let column = 8 + index % 8 * 2;
                rows[row][column] = b"0123456789ABCDEF"[(byte >> 4) as usize];
                rows[row][column + 1] = b"0123456789ABCDEF"[(byte & 15) as usize];
            }
        }
    } else {
        let page = page - identity_pages;
        rows[0].fill(b' ');
        let (status, objects, count, offset) = if page < svid_pages {
            if page == 0 {
                put(&mut rows[0], b"LAST SVID REPLY");
                hex(&mut rows[0], 20, discovery.count);
            }
            (
                discovery.status,
                discovery.svid_objects.as_slice(),
                discovery.svid_count,
                page * 5,
            )
        } else {
            let mode = &discovery.mode;
            if page % 2 == 1 {
                // Continues the previous section.
            } else if let Some(svid) = svid {
                put(&mut rows[0], b"SVID:---- MODES SORTED");
                hex(&mut rows[0], 5, svid);
            } else {
                put(&mut rows[0], b"SVID SET REFRESHING");
            }
            let cached = svid.is_some_and(|svid| svid == mode.svid);
            let status = if cached {
                mode.status
            } else {
                Status::NotQueried
            };
            (
                status,
                mode.objects.as_slice(),
                if cached { mode.count } else { 0 },
                ((page - svid_pages) % 2) * 5,
            )
        };
        // Every list takes two sections; the second continues the words.
        let first = page.is_multiple_of(2);
        if first {
            put(&mut rows[1], status_caption(status));
        }
        if first && page >= svid_pages && status == Status::NotQueried {
            put(
                &mut rows[1],
                if discovery.status != Status::Ack {
                    b"LIST NOT COMPLETE"
                } else if data.failed {
                    b"PD HALTED SEE PD PAGE"
                } else if data.active
                    || data.supply == Supply::Fixed && data.target_mv == VSAFE5V_MV
                {
                    // At fixed 5V the read for this SVID is queued or running.
                    b"READ IN PROGRESS"
                } else {
                    b"READ NEEDS SAFE FIXED5V"
                },
            );
        }
        // Five words per page leave a status row; no returned word is omitted.
        for row in 0..5 {
            let index = offset + row;
            if index < count as usize && index < objects.len() {
                put(&mut rows[row + 2], b"VDO:--");
                hex(&mut rows[row + 2], 4, index as u16);
                hex(&mut rows[row + 2], 16, (objects[index] >> 16) as u16);
                hex(&mut rows[row + 2], 20, objects[index] as u16);
            }
        }
    }
    rows
}

/// Section `page` of the readable summary: one labelled item per row, the
/// alt modes once the cable's SVID list is complete, then the maker's own
/// name when the cable gave one.
fn summary_lines(
    report: &CableReport,
    discovery: &Discovery,
    svids: &dyn pd_protocol::pd_svids::Store,
    page: usize,
) -> Lines {
    const { assert!(SUMMARY_VALUE + cable_summary::VALUE_WIDTH <= COLUMNS) };
    let summary = cable_summary::summary(report);
    let items = summary
        .iter()
        .flat_map(|summary| summary.items())
        .map(|(label, value)| (label.as_bytes(), value.as_slice()));
    let modes = modes(discovery, svids);
    let modes = (discovery.status == pd_protocol::pd_cable_modes::Status::Ack)
        .then_some((&b"MODES"[..], &modes[..]));
    let maker = maker(&discovery.manufacturer)
        .chunks(COLUMNS - SUMMARY_VALUE)
        .enumerate()
        .map(|(index, chunk)| (if index == 0 { &b"MAKER"[..] } else { &[] }, chunk));
    let mut rows = [[b' '; COLUMNS]; 8];
    if page == 0 {
        put(&mut rows[0], b"CABLE SUMMARY");
    }
    for (row, (label, value)) in rows[1..]
        .iter_mut()
        .zip(items.chain(modes).chain(maker).skip(page * SECTION_ROWS))
    {
        put(row, label);
        row[SUMMARY_VALUE..][..value.len()].copy_from_slice(value);
    }
    rows
}

/// The cable's alt modes in ascending SVID order: `TBT` and `DP` by name,
/// others in hexadecimal, then `+n` for the ones past the value's width;
/// `NONE` for an empty list.
fn modes(
    discovery: &Discovery,
    svids: &dyn pd_protocol::pd_svids::Store,
) -> [u8; cable_summary::VALUE_WIDTH] {
    /// Room kept for " +nn" while more names follow.
    const MORE: usize = 4;
    fn write(value: &mut [u8; cable_summary::VALUE_WIDTH], used: &mut usize, text: &[u8]) {
        value[*used..][..text.len()].copy_from_slice(text);
        *used += text.len();
    }
    let mut value = [b' '; cable_summary::VALUE_WIDTH];
    let mut used = 0;
    let count = discovery.count;
    if count == 0 {
        write(&mut value, &mut used, b"NONE");
    }
    for index in 0..count {
        let Some(svid) = svids.at(index) else { break };
        let hex: [u8; 4] = core::array::from_fn(|digit| {
            b"0123456789ABCDEF"[usize::from((svid >> (12 - 4 * digit)) & 15)]
        });
        let name = cable_summary::mode_name(svid).map_or(&hex[..], str::as_bytes);
        let separator: &[u8] = if index > 0 { b" " } else { b"" };
        let after = if index + 1 < count { MORE } else { 0 };
        if used + separator.len() + name.len() + after > value.len() {
            let left = (count - index).min(99);
            write(&mut value, &mut used, b" +");
            if left >= 10 {
                write(&mut value, &mut used, &[b'0' + (left / 10) as u8]);
            }
            write(&mut value, &mut used, &[b'0' + (left % 10) as u8]);
            break;
        }
        write(&mut value, &mut used, separator);
        write(&mut value, &mut used, name);
    }
    value
}

/// The maker's name from the cable's manufacturer info: printable text up
/// to its terminator, empty when the cable gave none.
fn maker(manufacturer: &pd_protocol::pd_cable_modes::Extended) -> &[u8] {
    if manufacturer.status != pd_protocol::pd_cable_modes::Status::Ack {
        return &[];
    }
    let data = manufacturer
        .data
        .get(MAKER_START..usize::from(manufacturer.count))
        .unwrap_or(&[]);
    let text = &data[..data
        .iter()
        .position(|&byte| byte == 0)
        .unwrap_or(data.len())];
    if text == MAKER_UNSUPPORTED
        || !text
            .iter()
            .all(|byte| byte.is_ascii_graphic() || *byte == b' ')
    {
        return &[];
    }
    text
}

/// Why an inspection returned no identity, and an optional hint row. The
/// SOP' Soft_Reset result separates a silent cable from an unanswered
/// request (kws-x1's `cable_failure`).
pub fn failure(
    status: CableStatus,
    sync: pd_protocol::pd_cable_modes::Sync,
) -> (&'static [u8], &'static [u8]) {
    use pd_protocol::pd_cable_modes::Sync;
    match (status, sync) {
        (CableStatus::Nak, _) => (b"CABLE ID NAK", b""),
        (CableStatus::Busy, _) => (b"CABLE ID BUSY NO RETRY", b""),
        // Sections 7.2/7.3: SinkTxNG or charger traffic; nothing was asked.
        (CableStatus::Deferred, _) => (b"CABLE: CHARGER BUSY", b""),
        // VCONN is only routed to CC2, so nothing is sent with CC2 active.
        (_, Sync::NotSent) => (b"CABLE ON CC2: NOT ASKED", b"TURN CABLE OVER"),
        (_, Sync::NoGoodCrc) => (b"CABLE NO GOODCRC", b"NO E-MARKER OR NO VCONN"),
        (_, Sync::NoAccept) => (b"CABLE RESET NO ACCEPT", b""),
        (_, Sync::Accepted) => (b"CABLE NO ID RESPONSE", b""),
    }
}

fn status_caption(status: pd_protocol::pd_cable_modes::Status) -> &'static [u8] {
    use pd_protocol::pd_cable_modes::Status;
    match status {
        Status::NotQueried => b"NOT QUERIED",
        Status::Ack => b"ACK",
        Status::Nak => b"NAK",
        Status::Busy => b"BUSY NO RETRY",
        Status::Timeout => b"TIMEOUT NO RETRY",
        Status::Continuing => b"LIST INCOMPLETE",
        Status::NotSupported => b"NOT SUPPORTED",
    }
}

fn put<const N: usize>(row: &mut [u8; N], text: &[u8]) {
    row[..text.len()].copy_from_slice(text);
}

fn hex<const N: usize>(row: &mut [u8; N], offset: usize, value: u16) {
    for i in 0..4 {
        row[offset + i] = b"0123456789ABCDEF"[((value >> (12 - 4 * i)) & 15) as usize];
    }
}
