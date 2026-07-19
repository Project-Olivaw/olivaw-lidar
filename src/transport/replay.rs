//! Transport that replays a recorded byte stream.

use core::time::Duration;
use std::path::Path;

use super::{Transport, TransportError};

/// A [`Transport`] that reads from a recorded session instead of hardware.
///
/// Writes are accepted and discarded (a recording cannot answer), reads
/// return the recorded bytes in order, and the end of the recording
/// surfaces as [`TransportError::Eof`] — which the scan iterator treats as
/// a clean end of stream.
///
/// Point it at a file produced by `examples/record.rs` and the full driver
/// stack runs with no device attached:
///
/// ```no_run
/// use olivaw_lidar::Lidar;
/// use olivaw_lidar::transport::ReplayTransport;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let replay = ReplayTransport::from_file("tests/fixtures/c1_scan_1000_nodes.bin")?;
/// let mut lidar = Lidar::with_transport(replay);
/// lidar.start_scan()?;
/// for scan in lidar.scans() {
///     println!("{} points", scan?.len());
/// }
/// # Ok(())
/// # }
/// ```
#[derive(Debug, Clone)]
pub struct ReplayTransport {
    data: Vec<u8>,
    pos: usize,
}

impl ReplayTransport {
    /// Replays from an in-memory byte buffer.
    #[must_use]
    pub fn from_bytes(data: Vec<u8>) -> Self {
        Self { data, pos: 0 }
    }

    /// Replays the contents of a recorded file.
    ///
    /// # Errors
    ///
    /// [`TransportError::Io`] when the file cannot be read.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, TransportError> {
        Ok(Self::from_bytes(std::fs::read(path)?))
    }

    /// Bytes not yet consumed.
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }
}

impl Transport for ReplayTransport {
    fn write_all(&mut self, _bytes: &[u8]) -> Result<(), TransportError> {
        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<(), TransportError> {
        if self.remaining() < buf.len() {
            // A partial node at the end of a recording is not an error
            // condition worth distinguishing; the stream is simply over.
            self.pos = self.data.len();
            return Err(TransportError::Eof);
        }
        buf.copy_from_slice(&self.data[self.pos..self.pos + buf.len()]);
        self.pos += buf.len();
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize, TransportError> {
        if self.remaining() == 0 {
            return Err(TransportError::Eof);
        }
        let n = buf.len().min(self.remaining());
        buf[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }

    fn discard_input(&mut self) -> Result<(), TransportError> {
        // A recording has no "pending" bytes to discard: consuming the
        // stream here would destroy the data the replay exists to provide.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMEOUT: Duration = Duration::from_millis(1);

    #[test]
    fn replays_bytes_in_order_then_eof() {
        let mut t = ReplayTransport::from_bytes(vec![1, 2, 3, 4, 5]);
        assert_eq!(t.remaining(), 5);

        let mut buf = [0u8; 2];
        t.read_exact(&mut buf, TIMEOUT).unwrap();
        assert_eq!(buf, [1, 2]);

        let mut rest = [0u8; 4];
        let n = t.read(&mut rest, TIMEOUT).unwrap();
        assert_eq!(n, 3);
        assert_eq!(&rest[..n], &[3, 4, 5]);

        assert!(matches!(
            t.read(&mut rest, TIMEOUT),
            Err(TransportError::Eof)
        ));
    }

    #[test]
    fn short_tail_is_eof_not_partial() {
        let mut t = ReplayTransport::from_bytes(vec![1, 2, 3]);
        let mut buf = [0u8; 5];
        assert!(matches!(
            t.read_exact(&mut buf, TIMEOUT),
            Err(TransportError::Eof)
        ));
        assert_eq!(t.remaining(), 0);
    }

    #[test]
    fn writes_and_discards_are_inert() {
        let mut t = ReplayTransport::from_bytes(vec![9]);
        t.write_all(&[0xA5, 0x20]).unwrap();
        t.discard_input().unwrap();
        assert_eq!(t.remaining(), 1, "discard_input must not eat replay data");
    }
}
