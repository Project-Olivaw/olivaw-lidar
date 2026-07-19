//! Trait-based byte I/O between the driver and a lidar.
//!
//! The [`Transport`] trait is the seam that keeps protocol logic testable
//! without hardware: [`SerialTransport`] talks to a real device, while
//! replay and mock transports feed recorded or scripted bytes through the
//! exact same code paths.

use core::time::Duration;

mod mock;
mod replay;
mod serial;

pub use mock::MockTransport;
pub use replay::ReplayTransport;
pub use serial::{SerialTransport, auto_detect_port, prefer_callout_device};

/// A transport-level failure, before any protocol interpretation.
///
/// Deliberately context-free: the device layer knows *what* it was waiting
/// for and upgrades [`TransportError::Timeout`] into a
/// [`LidarError::Timeout`](crate::LidarError::Timeout) that names it.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum TransportError {
    /// An operating-system I/O error.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// A serial-port-specific error (open, configuration, enumeration).
    #[error("serial port error: {0}")]
    Serial(#[from] serialport::Error),

    /// No byte arrived within the allowed time.
    #[error("read timed out")]
    Timeout,

    /// The byte source is exhausted. Never produced by real hardware; a
    /// replay transport reports this when the recording runs out.
    #[error("end of stream")]
    Eof,
}

/// Blocking byte-stream transport to a lidar unit.
///
/// Implementations move bytes and nothing else — framing, checksums and
/// parsing all live in [`crate::protocol`]. All methods are object-safe, so
/// `Box<dyn Transport>` is a valid transport too.
pub trait Transport {
    /// Writes all of `bytes`, blocking until the OS has accepted them.
    ///
    /// # Errors
    ///
    /// Any [`TransportError`] from the underlying byte sink.
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError>;

    /// Fills `buf` completely, or fails.
    ///
    /// # Errors
    ///
    /// [`TransportError::Timeout`] if `buf` could not be filled within
    /// `timeout`, [`TransportError::Eof`] if the byte source ended, or any
    /// underlying I/O error.
    fn read_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), TransportError>;

    /// Reads *at least one* byte (up to `buf.len()`) and returns how many
    /// arrived. Never returns `Ok(0)`.
    ///
    /// # Errors
    ///
    /// [`TransportError::Timeout`] if no byte arrived within `timeout`,
    /// [`TransportError::Eof`] if the byte source ended, or any underlying
    /// I/O error.
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, TransportError>;

    /// Discards any bytes already received but not yet read.
    ///
    /// # Errors
    ///
    /// Any [`TransportError`] from the underlying byte source.
    fn discard_input(&mut self) -> Result<(), TransportError>;
}

impl<T: Transport + ?Sized> Transport for Box<T> {
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        (**self).write_all(bytes)
    }

    fn read_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), TransportError> {
        (**self).read_exact(buf, timeout)
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, TransportError> {
        (**self).read(buf, timeout)
    }

    fn discard_input(&mut self) -> Result<(), TransportError> {
        (**self).discard_input()
    }
}
