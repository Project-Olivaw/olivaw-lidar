//! Response descriptor parsing.
//!
//! Every response the device sends is preceded by a 7-byte descriptor:
//!
//! ```text
//! ┌──────┬──────┬───────────────────────────────┬───────────┐
//! │ 0xA5 │ 0x5A │ len[29:0] + mode[1:0], u32 LE │ data type │
//! └──────┴──────┴───────────────────────────────┴───────────┘
//! ```
//!
//! The third field packs a 30-bit payload length (low bits) and a 2-bit send
//! mode (high bits) into a little-endian `u32`.

use super::{ProtocolError, SYNC_BYTE};

/// Length in bytes of a response descriptor.
pub const DESCRIPTOR_LEN: usize = 7;

/// Second sync byte of a response descriptor (the first is [`SYNC_BYTE`]).
pub const RESP_SYNC_BYTE: u8 = 0x5A;

/// Data type of a `GET_INFO` response (20-byte payload, single).
pub const DATA_TYPE_DEVINFO: u8 = 0x04;
/// Data type of a `GET_HEALTH` response (3-byte payload, single).
pub const DATA_TYPE_DEVHEALTH: u8 = 0x06;
/// Data type of a `GET_SAMPLERATE` response (4-byte payload, single).
pub const DATA_TYPE_SAMPLE_RATE: u8 = 0x15;
/// Data type of a standard `SCAN` response (5-byte nodes, multi).
pub const DATA_TYPE_MEASUREMENT: u8 = 0x81;

/// How many responses follow a descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendMode {
    /// Exactly one response of the announced length follows.
    Single,
    /// An endless stream of responses of the announced length follows,
    /// until the device is stopped. Used by scan commands.
    Multi,
}

/// A parsed 7-byte response descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResponseDescriptor {
    /// Payload length in bytes of each following response (30-bit value).
    pub len: u32,
    /// Whether one or many responses follow.
    pub send_mode: SendMode,
    /// Payload type identifier, e.g. [`DATA_TYPE_DEVINFO`].
    pub data_type: u8,
}

/// Parses a 7-byte response descriptor.
///
/// # Errors
///
/// - [`ProtocolError::BadSync`] if the first two bytes are not `0xA5 0x5A`
///   (the stream is desynchronized).
/// - [`ProtocolError::ReservedSendMode`] if the send-mode bits hold a
///   reserved value (`0x2`/`0x3`).
///
/// ```
/// use olivaw_lidar::protocol::descriptor::{self, SendMode};
///
/// // GET_INFO descriptor: 20-byte single response, data type 0x04.
/// let desc = descriptor::parse_descriptor(&[0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04])?;
/// assert_eq!(desc.len, 20);
/// assert_eq!(desc.send_mode, SendMode::Single);
/// assert_eq!(desc.data_type, 0x04);
/// # Ok::<(), olivaw_lidar::protocol::ProtocolError>(())
/// ```
pub fn parse_descriptor(bytes: &[u8; DESCRIPTOR_LEN]) -> Result<ResponseDescriptor, ProtocolError> {
    if bytes[0] != SYNC_BYTE || bytes[1] != RESP_SYNC_BYTE {
        return Err(ProtocolError::BadSync {
            actual: [bytes[0], bytes[1]],
        });
    }
    let packed = u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
    let len = packed & 0x3FFF_FFFF;
    let send_mode = match packed >> 30 {
        0 => SendMode::Single,
        1 => SendMode::Multi,
        mode => {
            // `mode` is 2 or 3 here; the cast cannot truncate.
            #[allow(clippy::cast_possible_truncation)]
            return Err(ProtocolError::ReservedSendMode(mode as u8));
        }
    };
    Ok(ResponseDescriptor {
        len,
        send_mode,
        data_type: bytes[6],
    })
}

impl ResponseDescriptor {
    /// Validates this descriptor against what a request expects.
    ///
    /// # Errors
    ///
    /// [`ProtocolError::WrongDataType`], [`ProtocolError::WrongLength`] or
    /// [`ProtocolError::WrongSendMode`] naming the first mismatch found.
    pub fn expect(self, data_type: u8, len: u32, send_mode: SendMode) -> Result<(), ProtocolError> {
        if self.data_type != data_type {
            return Err(ProtocolError::WrongDataType {
                expected: data_type,
                actual: self.data_type,
            });
        }
        if self.len != len {
            return Err(ProtocolError::WrongLength {
                expected: len,
                actual: self.len,
            });
        }
        if self.send_mode != send_mode {
            return Err(ProtocolError::WrongSendMode);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_devinfo_descriptor() {
        let desc = parse_descriptor(&[0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04]).unwrap();
        assert_eq!(
            desc,
            ResponseDescriptor {
                len: 20,
                send_mode: SendMode::Single,
                data_type: DATA_TYPE_DEVINFO
            }
        );
    }

    #[test]
    fn parses_health_descriptor() {
        let desc = parse_descriptor(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06]).unwrap();
        assert_eq!(desc.len, 3);
        assert_eq!(desc.send_mode, SendMode::Single);
        assert_eq!(desc.data_type, DATA_TYPE_DEVHEALTH);
    }

    #[test]
    fn parses_scan_descriptor_multi_mode() {
        // len = 5, mode bits = 0b01 → byte 5 (bits 30..32 of the LE u32) = 0x40.
        let desc = parse_descriptor(&[0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81]).unwrap();
        assert_eq!(desc.len, 5);
        assert_eq!(desc.send_mode, SendMode::Multi);
        assert_eq!(desc.data_type, DATA_TYPE_MEASUREMENT);
    }

    #[test]
    fn parses_full_30_bit_length() {
        // All 30 length bits set, mode 0.
        let desc = parse_descriptor(&[0xA5, 0x5A, 0xFF, 0xFF, 0xFF, 0x3F, 0x00]).unwrap();
        assert_eq!(desc.len, 0x3FFF_FFFF);
        assert_eq!(desc.send_mode, SendMode::Single);
    }

    #[test]
    fn rejects_bad_sync() {
        let err = parse_descriptor(&[0xA5, 0x00, 0x14, 0x00, 0x00, 0x00, 0x04]).unwrap_err();
        assert_eq!(
            err,
            ProtocolError::BadSync {
                actual: [0xA5, 0x00]
            }
        );
        let err = parse_descriptor(&[0x00, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04]).unwrap_err();
        assert_eq!(
            err,
            ProtocolError::BadSync {
                actual: [0x00, 0x5A]
            }
        );
    }

    #[test]
    fn rejects_reserved_send_modes() {
        // Mode bits 0b10 → byte 5 = 0x80; 0b11 → 0xC0.
        let err = parse_descriptor(&[0xA5, 0x5A, 0x05, 0x00, 0x00, 0x80, 0x81]).unwrap_err();
        assert_eq!(err, ProtocolError::ReservedSendMode(2));
        let err = parse_descriptor(&[0xA5, 0x5A, 0x05, 0x00, 0x00, 0xC0, 0x81]).unwrap_err();
        assert_eq!(err, ProtocolError::ReservedSendMode(3));
    }

    #[test]
    fn expect_validates_each_field() {
        let desc = ResponseDescriptor {
            len: 20,
            send_mode: SendMode::Single,
            data_type: DATA_TYPE_DEVINFO,
        };
        assert!(desc.expect(DATA_TYPE_DEVINFO, 20, SendMode::Single).is_ok());
        assert_eq!(
            desc.expect(DATA_TYPE_DEVHEALTH, 20, SendMode::Single),
            Err(ProtocolError::WrongDataType {
                expected: DATA_TYPE_DEVHEALTH,
                actual: DATA_TYPE_DEVINFO
            })
        );
        assert_eq!(
            desc.expect(DATA_TYPE_DEVINFO, 3, SendMode::Single),
            Err(ProtocolError::WrongLength {
                expected: 3,
                actual: 20
            })
        );
        assert_eq!(
            desc.expect(DATA_TYPE_DEVINFO, 20, SendMode::Multi),
            Err(ProtocolError::WrongSendMode)
        );
    }
}
