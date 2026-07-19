//! Real-hardware transport over a serial port.

use std::io::{ErrorKind, Read as _, Write as _};
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPortType, StopBits};

use super::{Transport, TransportError};

/// Timeout configured on a freshly opened port; every read replaces it.
const INITIAL_TIMEOUT: Duration = Duration::from_secs(1);

/// USB VID/PID pairs of the USB-to-UART bridges SLAMTEC ships:
/// Silicon Labs `CP210x` and WCH `CH340`.
const KNOWN_BRIDGES: [(u16, u16); 2] = [(0x10C4, 0xEA60), (0x1A86, 0x7523)];

/// A [`Transport`] over a real serial port, 8 data bits, no parity, 1 stop
/// bit, no flow control.
///
/// Reads take a per-call timeout; the port-level timeout is updated lazily
/// (a plain field write on POSIX) only when the requested value changes.
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    current_timeout: Duration,
}

impl std::fmt::Debug for SerialTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialTransport")
            .field("port", &self.port.name())
            .field("baud", &self.port.baud_rate().ok())
            .finish_non_exhaustive()
    }
}

impl SerialTransport {
    /// Opens `path` at `baud`, 8N1, no flow control.
    ///
    /// This performs no device handshake — `Lidar::open` layers the
    /// C1 recovery sequence (STOP, settle, flush) on top.
    ///
    /// Note for future A1 support: POSIX asserts DTR on open, and the A1
    /// gates its motor on DTR; explicit DTR control belongs here when that
    /// model lands. The C1 ignores DTR entirely.
    ///
    /// # Errors
    ///
    /// [`TransportError::Serial`] if the port cannot be opened or
    /// configured — wrong path, device unplugged, or missing permissions
    /// (on Linux, membership in the `dialout` group).
    pub fn open(path: &str, baud: u32) -> Result<Self, TransportError> {
        let port = serialport::new(path, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(INITIAL_TIMEOUT)
            .open()?;
        Ok(Self {
            port,
            current_timeout: INITIAL_TIMEOUT,
        })
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<(), TransportError> {
        // serialport rejects a zero timeout on some platforms; clamp up.
        let timeout = timeout.max(Duration::from_millis(1));
        if timeout != self.current_timeout {
            self.port.set_timeout(timeout)?;
            self.current_timeout = timeout;
        }
        Ok(())
    }
}

impl Transport for SerialTransport {
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.port.write_all(bytes)?;
        self.port.flush()?;
        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), TransportError> {
        let deadline = Instant::now() + timeout;
        let mut filled = 0;
        while filled < buf.len() {
            let Some(remaining) = deadline
                .checked_duration_since(Instant::now())
                .filter(|d| !d.is_zero())
            else {
                return Err(TransportError::Timeout);
            };
            self.set_timeout(remaining)?;
            match self.port.read(&mut buf[filled..]) {
                Ok(0) => return Err(TransportError::Eof),
                Ok(n) => filled += n,
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) if is_timeout(&e) => return Err(TransportError::Timeout),
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, TransportError> {
        let deadline = Instant::now() + timeout;
        loop {
            let Some(remaining) = deadline
                .checked_duration_since(Instant::now())
                .filter(|d| !d.is_zero())
            else {
                return Err(TransportError::Timeout);
            };
            self.set_timeout(remaining)?;
            match self.port.read(buf) {
                Ok(0) => return Err(TransportError::Eof),
                Ok(n) => return Ok(n),
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) if is_timeout(&e) => return Err(TransportError::Timeout),
                Err(e) => return Err(e.into()),
            }
        }
    }

    fn discard_input(&mut self) -> Result<(), TransportError> {
        self.port.clear(ClearBuffer::Input)?;
        Ok(())
    }
}

/// Timeout surfacing differs per platform; normalize both flavors.
fn is_timeout(e: &std::io::Error) -> bool {
    matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock)
}

/// Finds the serial port a SLAMTEC lidar is most likely attached to.
///
/// Preference order:
/// 1. a USB port whose VID/PID matches a bridge chip SLAMTEC ships
///    (`CP210x` `10C4:EA60`, `CH340` `1A86:7523`),
/// 2. any other USB serial port,
/// 3. a port whose name looks like a USB-serial adapter
///    (`/dev/cu.usbserial-*`, `/dev/ttyUSB*`, `/dev/ttyACM*`).
///
/// On macOS the `/dev/cu.*` (call-out) device is always preferred over its
/// `/dev/tty.*` sibling — opening the `tty.` variant can block waiting for
/// a carrier signal that USB adapters never raise.
///
/// Returns `None` when no plausible port exists (is the lidar plugged in?).
#[must_use]
pub fn auto_detect_port() -> Option<String> {
    let ports = serialport::available_ports().ok()?;
    let mut usb_fallback: Option<String> = None;
    let mut name_fallback: Option<String> = None;
    for port in ports {
        let name = prefer_callout_device(&port.port_name);
        if let SerialPortType::UsbPort(usb) = &port.port_type {
            if KNOWN_BRIDGES.contains(&(usb.vid, usb.pid)) {
                return Some(name);
            }
            usb_fallback.get_or_insert(name);
        } else if looks_like_usb_serial(&name) {
            name_fallback.get_or_insert(name);
        }
    }
    usb_fallback.or(name_fallback)
}

/// On macOS, rewrites `/dev/tty.*` to its non-blocking `/dev/cu.*` sibling.
/// Returns other names (and all names on other platforms) unchanged.
#[must_use]
pub fn prefer_callout_device(path: &str) -> String {
    if cfg!(target_os = "macos") {
        if let Some(rest) = path.strip_prefix("/dev/tty.") {
            return format!("/dev/cu.{rest}");
        }
    }
    path.to_owned()
}

fn looks_like_usb_serial(name: &str) -> bool {
    name.starts_with("/dev/cu.usbserial")
        || name.starts_with("/dev/cu.SLAB_USBtoUART")
        || name.starts_with("/dev/cu.wchusbserial")
        || name.starts_with("/dev/ttyUSB")
        || name.starts_with("/dev/ttyACM")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recognizes_usb_serial_names() {
        assert!(looks_like_usb_serial("/dev/cu.usbserial-1130"));
        assert!(looks_like_usb_serial("/dev/ttyUSB0"));
        assert!(looks_like_usb_serial("/dev/ttyACM0"));
        assert!(!looks_like_usb_serial("/dev/cu.Bluetooth-Incoming-Port"));
        assert!(!looks_like_usb_serial("/dev/ttyS0"));
    }

    #[cfg(target_os = "macos")]
    #[test]
    fn rewrites_tty_to_cu_on_macos() {
        assert_eq!(
            prefer_callout_device("/dev/tty.usbserial-1130"),
            "/dev/cu.usbserial-1130"
        );
        assert_eq!(
            prefer_callout_device("/dev/cu.usbserial-1130"),
            "/dev/cu.usbserial-1130"
        );
    }
}
