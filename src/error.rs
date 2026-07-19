//! The crate-level error type.

use crate::protocol::ProtocolError;
use crate::transport::TransportError;

/// Any failure while talking to a lidar.
///
/// Variants are specific enough to act on: a [`LidarError::Timeout`] names
/// what was being waited for, a [`LidarError::Protocol`] means the byte
/// stream broke, and a [`LidarError::Serial`] points at the port itself
/// (wrong path, permissions, unplugged device).
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum LidarError {
    /// The serial port could not be opened or configured. Check the device
    /// path, that the lidar is plugged in, and (on Linux) that the user is
    /// in the `dialout` group.
    #[error("serial port error: {0}")]
    Serial(#[from] serialport::Error),

    /// Received bytes did not match the RPLIDAR wire format.
    #[error(transparent)]
    Protocol(#[from] ProtocolError),

    /// A transport failure other than a timeout or a serial-port error
    /// (those are upgraded to [`LidarError::Timeout`] and
    /// [`LidarError::Serial`] with more context).
    #[error("transport error: {0}")]
    Transport(TransportError),

    /// The device did not answer in time. Check that the correct port was
    /// selected and that the baud rate matches the model.
    #[error("timed out waiting for {what} after {ms}ms")]
    Timeout {
        /// What was being waited for, e.g. `"device info descriptor"`.
        what: &'static str,
        /// The timeout that elapsed, in milliseconds.
        ms: u64,
    },

    /// The measurement stream lost frame synchronization.
    #[error("protocol desync: {0}")]
    Desync(String),

    /// The device reported an internal error status; the payload is the
    /// device-specific error code. Try power-cycling the unit.
    #[error("device reported error status: {0:#06x}")]
    DeviceError(u16),

    /// The connected device reported a model byte this crate does not
    /// support.
    #[error("unsupported model: {0:#04x}")]
    UnsupportedModel(u8),
}
