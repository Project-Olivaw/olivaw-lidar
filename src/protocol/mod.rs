//! Pure RPLIDAR protocol encoding and decoding.
//!
//! Bytes in, typed values out. This module performs **no I/O**, depends only
//! on `core`, and is fully unit-testable with no hardware attached. All
//! device communication is layered on top of it by the `transport` and
//! `device` modules (enabled with the `std` feature).
//!
//! Wire format reference: the SLAMTEC RPLIDAR public protocol specification.
//! Every request starts with the sync byte [`SYNC_BYTE`]; every response is
//! preceded by a 7-byte descriptor.

mod command;

pub use command::{Command, MAX_REQUEST_LEN, MOTOR_PWM_MAX, SYNC_BYTE};

/// A protocol-level decoding failure.
///
/// These errors indicate that bytes received from the device do not match
/// the RPLIDAR wire format — typically a desynchronized stream, a corrupted
/// transfer, or an unexpected response to a request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum ProtocolError {
    /// A response descriptor did not start with the `0xA5 0x5A` sync bytes.
    ///
    /// The stream is desynchronized; flush the input and retry the request.
    #[error("bad descriptor sync: expected [0xa5, 0x5a], got {actual:02x?}")]
    BadSync {
        /// The two bytes actually received where the sync bytes were expected.
        actual: [u8; 2],
    },

    /// A response descriptor used a reserved send-mode value (`0x2`/`0x3`).
    #[error("reserved response send mode {0:#04x}")]
    ReservedSendMode(u8),

    /// The response descriptor announced a different data type than the one
    /// the request expects.
    #[error("unexpected response data type: expected {expected:#04x}, got {actual:#04x}")]
    WrongDataType {
        /// Data type required by the request that was sent.
        expected: u8,
        /// Data type announced by the device.
        actual: u8,
    },

    /// The response descriptor announced a different payload length than the
    /// one the request expects.
    #[error("unexpected response length: expected {expected}, got {actual}")]
    WrongLength {
        /// Payload length in bytes required by the request that was sent.
        expected: u32,
        /// Payload length announced by the device.
        actual: u32,
    },

    /// The response descriptor announced a different send mode (single vs.
    /// multi) than the one the request expects.
    #[error("unexpected response send mode")]
    WrongSendMode,

    /// A `GET_HEALTH` response carried a status byte outside `0..=2`.
    #[error("invalid health status byte {0:#04x}")]
    InvalidHealthStatus(u8),

    /// A checksum-protected payload failed verification.
    ///
    /// Reserved for express-scan responses; standard responses carry no
    /// checksum.
    #[error("bad checksum: expected {expected:#04x}, got {actual:#04x}")]
    Checksum {
        /// Checksum computed over the received bytes.
        expected: u8,
        /// Checksum byte actually received.
        actual: u8,
    },
}
