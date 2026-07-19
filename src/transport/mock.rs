//! Scriptable test-double transport.

use core::time::Duration;

use super::{Transport, TransportError};

/// A [`Transport`] test double: reads come from bytes queued in advance,
/// writes are captured for assertions.
///
/// An exhausted queue reads as [`TransportError::Timeout`] — a silent
/// device — unlike `ReplayTransport`, whose exhausted recording is a clean
/// [`TransportError::Eof`].
///
/// ```
/// use olivaw_lidar::Lidar;
/// use olivaw_lidar::transport::MockTransport;
///
/// let mut mock = MockTransport::new();
/// // GET_HEALTH response: descriptor + status Good.
/// mock.queue_input(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06]);
/// mock.queue_input(&[0x00, 0x00, 0x00]);
///
/// let mut lidar = Lidar::with_transport(mock);
/// assert!(lidar.health().unwrap().is_good());
/// assert_eq!(lidar.into_transport().written(), &[0xA5, 0x52]);
/// ```
#[derive(Debug, Default)]
pub struct MockTransport {
    input: Vec<u8>,
    pos: usize,
    written: Vec<u8>,
    discards: usize,
}

impl MockTransport {
    /// A mock with nothing queued.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Queues bytes for future reads.
    pub fn queue_input(&mut self, bytes: &[u8]) {
        self.input.extend_from_slice(bytes);
    }

    /// Every byte written so far, in order.
    #[must_use]
    pub fn written(&self) -> &[u8] {
        &self.written
    }

    /// How many times [`Transport::discard_input`] was called.
    #[must_use]
    pub fn discards(&self) -> usize {
        self.discards
    }

    fn remaining(&self) -> usize {
        self.input.len() - self.pos
    }
}

impl Transport for MockTransport {
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.written.extend_from_slice(bytes);
        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<(), TransportError> {
        if self.remaining() < buf.len() {
            return Err(TransportError::Timeout);
        }
        buf.copy_from_slice(&self.input[self.pos..self.pos + buf.len()]);
        self.pos += buf.len();
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize, TransportError> {
        if self.remaining() == 0 {
            return Err(TransportError::Timeout);
        }
        let n = buf.len().min(self.remaining());
        buf[..n].copy_from_slice(&self.input[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }

    fn discard_input(&mut self) -> Result<(), TransportError> {
        // Like real hardware: whatever was pending is gone.
        self.pos = self.input.len();
        self.discards += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TIMEOUT: Duration = Duration::from_millis(1);

    #[test]
    fn records_writes_and_serves_queued_reads() {
        let mut t = MockTransport::new();
        t.queue_input(&[10, 20, 30]);
        t.write_all(&[0xA5, 0x50]).unwrap();

        let mut buf = [0u8; 3];
        t.read_exact(&mut buf, TIMEOUT).unwrap();
        assert_eq!(buf, [10, 20, 30]);
        assert_eq!(t.written(), &[0xA5, 0x50]);
    }

    #[test]
    fn empty_queue_times_out() {
        let mut t = MockTransport::new();
        let mut buf = [0u8; 1];
        assert!(matches!(
            t.read(&mut buf, TIMEOUT),
            Err(TransportError::Timeout)
        ));
    }

    #[test]
    fn discard_drops_pending_input() {
        let mut t = MockTransport::new();
        t.queue_input(&[1, 2, 3]);
        t.discard_input().unwrap();
        assert_eq!(t.discards(), 1);
        let mut buf = [0u8; 1];
        assert!(t.read_exact(&mut buf, TIMEOUT).is_err());
    }
}
