//! Standard scan measurement node parsing.
//!
//! During a standard scan the device streams 5-byte nodes:
//!
//! ```text
//! byte 0:  [quality:6][!S:1][S:1]    S = start-of-rotation flag, !S its inverse
//! byte 1:  [angle_low:7][C:1]        C = check bit, always 1
//! byte 2:  [angle_high:8]
//! byte 3:  [distance_low:8]
//! byte 4:  [distance_high:8]
//! ```
//!
//! The `S`/`!S` pair and the check bit exist purely for synchronization: if
//! either invariant fails, the reader has lost frame alignment and must
//! resynchronize by discarding bytes one at a time (see
//! [`could_start_node`]).

use super::ProtocolError;

/// Length in bytes of one standard scan node.
pub const SCAN_NODE_LEN: usize = 5;

/// One raw measurement from a standard scan.
///
/// Fields hold the wire fixed-point values; use the accessor methods for
/// unit conversions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanNode {
    /// `true` on the first node of each new 360° rotation.
    pub start_flag: bool,
    /// Reflected-signal quality, 0..=63. Higher is better; 0 usually
    /// accompanies an invalid measurement.
    pub quality: u8,
    /// Angle in fixed-point Q6 format (1/64 degree units), 15 bits.
    pub angle_q6: u16,
    /// Distance in fixed-point Q2 format (1/4 millimeter units).
    /// `0` means no return (invalid measurement).
    pub distance_q2: u16,
}

impl ScanNode {
    /// Angle in degrees, `0.0..512.0` nominally (`0.0..360.0` in practice).
    #[must_use]
    pub fn angle_deg(&self) -> f32 {
        f32::from(self.angle_q6) / 64.0
    }

    /// Distance in millimeters. `0.0` means no return.
    #[must_use]
    pub fn distance_mm(&self) -> f32 {
        f32::from(self.distance_q2) / 4.0
    }

    /// `true` when the node carries a real measurement (non-zero distance).
    #[must_use]
    pub const fn is_valid_measurement(&self) -> bool {
        self.distance_q2 != 0
    }
}

/// `true` when `byte0` could plausibly begin a node: the start flag and its
/// inverse must differ. Used to resynchronize a desynchronized stream
/// cheaply before attempting a full parse.
#[must_use]
pub const fn could_start_node(byte0: u8) -> bool {
    let start = byte0 & 0x01;
    let inverse = (byte0 >> 1) & 0x01;
    start != inverse
}

/// Parses one 5-byte scan node.
///
/// # Errors
///
/// [`ProtocolError::InvalidScanNode`] when the `S`/`!S` pair or the check
/// bit is inconsistent — the stream is desynchronized; discard one byte and
/// retry.
pub fn parse_scan_node(bytes: &[u8; SCAN_NODE_LEN]) -> Result<ScanNode, ProtocolError> {
    let check_bit = bytes[1] & 0x01;
    if !could_start_node(bytes[0]) || check_bit != 1 {
        return Err(ProtocolError::InvalidScanNode {
            byte0: bytes[0],
            byte1: bytes[1],
        });
    }
    Ok(ScanNode {
        start_flag: bytes[0] & 0x01 == 1,
        quality: bytes[0] >> 2,
        angle_q6: (u16::from(bytes[2]) << 7) | u16::from(bytes[1] >> 1),
        distance_q2: (u16::from(bytes[4]) << 8) | u16::from(bytes[3]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a wire node from human units, for tests.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn encode_node(start: bool, quality: u8, angle_deg: f32, distance_mm: f32) -> [u8; 5] {
        let angle_q6 = (angle_deg * 64.0) as u16;
        let distance_q2 = (distance_mm * 4.0) as u16;
        let s: u8 = u8::from(start);
        [
            (quality << 2) | ((1 - s) << 1) | s,
            (((angle_q6 & 0x7F) as u8) << 1) | 0x01,
            (angle_q6 >> 7) as u8,
            (distance_q2 & 0xFF) as u8,
            (distance_q2 >> 8) as u8,
        ]
    }

    #[test]
    fn parses_a_start_node() {
        // 90°, 1000 mm, quality 15, start of rotation.
        let node = parse_scan_node(&encode_node(true, 15, 90.0, 1000.0)).unwrap();
        assert!(node.start_flag);
        assert_eq!(node.quality, 15);
        assert_eq!(node.angle_q6, 5760);
        assert!((node.angle_deg() - 90.0).abs() < 1.0 / 64.0);
        assert_eq!(node.distance_q2, 4000);
        assert!((node.distance_mm() - 1000.0).abs() < 0.25);
        assert!(node.is_valid_measurement());
    }

    #[test]
    fn parses_a_non_start_node_with_zero_distance() {
        let node = parse_scan_node(&encode_node(false, 0, 359.9, 0.0)).unwrap();
        assert!(!node.start_flag);
        assert_eq!(node.quality, 0);
        assert_eq!(node.distance_q2, 0);
        assert!(!node.is_valid_measurement());
        assert!((node.angle_deg() - 359.9).abs() < 1.0 / 32.0);
    }

    #[test]
    fn known_wire_bytes_decode_exactly() {
        // Hand-computed: quality 40, not start (S=0, !S=1), angle_q6 = 0x1234
        // (72.8125°), distance_q2 = 0x0FA0 (1000 mm).
        // byte0 = 40<<2 | 0b10 = 0xA2
        // byte1 = (0x34 & 0x7F) << 1 | 1 = 0x69
        // byte2 = 0x1234 >> 7 = 0x24
        let node = parse_scan_node(&[0xA2, 0x69, 0x24, 0xA0, 0x0F]).unwrap();
        assert_eq!(node.quality, 40);
        assert!(!node.start_flag);
        assert_eq!(node.angle_q6, 0x1234);
        assert_eq!(node.distance_q2, 0x0FA0);
    }

    #[test]
    fn rejects_bad_start_flag_pair() {
        // S == !S == 0 (byte0 low bits 0b00).
        let err = parse_scan_node(&[0b0000_0000, 0x01, 0, 0, 0]).unwrap_err();
        assert!(matches!(err, ProtocolError::InvalidScanNode { .. }));
        // S == !S == 1 (0b11).
        let err = parse_scan_node(&[0b0000_0011, 0x01, 0, 0, 0]).unwrap_err();
        assert!(matches!(err, ProtocolError::InvalidScanNode { .. }));
    }

    #[test]
    fn rejects_clear_check_bit() {
        let err = parse_scan_node(&[0b0000_0001, 0b0000_0000, 0, 0, 0]).unwrap_err();
        assert_eq!(
            err,
            ProtocolError::InvalidScanNode {
                byte0: 0x01,
                byte1: 0x00
            }
        );
    }

    #[test]
    fn could_start_node_matches_parser() {
        for byte0 in 0..=u8::MAX {
            let plausible = could_start_node(byte0);
            let parses = parse_scan_node(&[byte0, 0x01, 0, 0, 0]).is_ok();
            assert_eq!(plausible, parses, "byte0={byte0:#04x}");
        }
    }
}
