//! The high-level [`Lidar`] API.

use std::time::{Duration, Instant};

use crate::error::LidarError;
use crate::protocol::descriptor::{
    self, DATA_TYPE_DEVHEALTH, DATA_TYPE_DEVINFO, DATA_TYPE_MEASUREMENT, ResponseDescriptor,
    SendMode,
};
use crate::protocol::info::{self, DEVHEALTH_LEN, DEVINFO_LEN, DeviceInfo, HealthStatus};
use crate::protocol::scan_node::{self, SCAN_NODE_LEN, ScanNode};
use crate::protocol::{Command, MAX_REQUEST_LEN};
use crate::transport::{SerialTransport, Transport, TransportError};
use crate::types::{Point, Scan};

/// Give up resynchronizing after discarding this many bytes without finding
/// a valid node. Post-restart garbage is typically well under a kilobyte.
const MAX_DESYNC_DISCARD: usize = 4096;

/// Connection and timing parameters.
///
/// The defaults match the RPLIDAR C1. Override individual fields for other
/// models, e.g. `LidarConfig { baud_rate: 115_200, ..LidarConfig::default() }`
/// for an A1.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LidarConfig {
    /// Serial baud rate. C1: `460_800`.
    pub baud_rate: u32,
    /// Budget for each descriptor or payload read.
    pub response_timeout: Duration,
    /// Wait after `STOP` before the device is reliably idle.
    pub stop_settle: Duration,
    /// Wait after `SET_MOTOR_PWM` before the motor speed is trustworthy;
    /// starting a scan earlier can race the motor spin-up.
    pub motor_settle: Duration,
    /// Wait after `RESET` while the device reboots and prints its unframed
    /// ASCII boot banner.
    pub reset_settle: Duration,
    /// Motor PWM duty sent by [`Lidar::start_scan`] before scanning.
    /// The C1 manages its own motor, but sending a duty is harmless there
    /// and required on externally-driven A-series units. `0` skips the
    /// motor command entirely.
    pub motor_pwm: u16,
    /// Budget for the *first* scan node after [`Lidar::start_scan`].
    ///
    /// The device acknowledges `SCAN` with a descriptor immediately, but
    /// streams no measurements until the motor reaches speed — on the C1
    /// about two seconds. Subsequent nodes use the much shorter
    /// [`LidarConfig::response_timeout`].
    pub scan_startup_timeout: Duration,
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            baud_rate: 460_800,
            response_timeout: Duration::from_secs(1),
            stop_settle: Duration::from_millis(50),
            motor_settle: Duration::from_millis(50),
            reset_settle: Duration::from_millis(500),
            motor_pwm: 660,
            scan_startup_timeout: Duration::from_secs(5),
        }
    }
}

/// A connected lidar.
///
/// Owns a [`Transport`] and drives the request/response protocol over it.
/// Constructed either from a real serial port with [`Lidar::open`] or from
/// any transport implementation with [`Lidar::with_transport`].
#[derive(Debug)]
pub struct Lidar<T: Transport> {
    transport: T,
    config: LidarConfig,
}

impl Lidar<SerialTransport> {
    /// Opens the lidar on `path` with C1 defaults (460800 baud, 8N1).
    ///
    /// Also performs the recovery handshake — `STOP`, a short settle, then
    /// an input flush — so that a unit left streaming by a crashed previous
    /// run comes up clean.
    ///
    /// # Errors
    ///
    /// [`LidarError::Serial`] when the port cannot be opened (wrong path,
    /// unplugged device, missing permissions), or any error from the
    /// handshake writes.
    pub fn open(path: &str) -> Result<Self, LidarError> {
        Self::open_with_config(path, LidarConfig::default())
    }

    /// Opens the lidar on `path` with explicit parameters.
    ///
    /// # Errors
    ///
    /// Same as [`Lidar::open`].
    pub fn open_with_config(path: &str, config: LidarConfig) -> Result<Self, LidarError> {
        let transport = match SerialTransport::open(path, config.baud_rate) {
            Ok(t) => t,
            Err(TransportError::Serial(e)) => return Err(LidarError::Serial(e)),
            Err(e) => return Err(LidarError::Transport(e)),
        };
        let mut lidar = Self::with_transport_and_config(transport, config);
        lidar.stop()?;
        Ok(lidar)
    }
}

impl<T: Transport> Lidar<T> {
    /// Wraps an existing transport with default configuration.
    ///
    /// No handshake bytes are sent — appropriate for replay and mock
    /// transports.
    pub fn with_transport(transport: T) -> Self {
        Self::with_transport_and_config(transport, LidarConfig::default())
    }

    /// Wraps an existing transport with explicit configuration.
    pub fn with_transport_and_config(transport: T, config: LidarConfig) -> Self {
        Self { transport, config }
    }

    /// Requests the device model, firmware/hardware versions and serial
    /// number.
    ///
    /// # Errors
    ///
    /// [`LidarError::Timeout`] when the device does not answer (wrong port
    /// or baud rate), or [`LidarError::Protocol`] when the response does not
    /// match the wire format.
    pub fn info(&mut self) -> Result<DeviceInfo, LidarError> {
        self.request(Command::GetInfo)?;
        self.expect_descriptor(
            "device info descriptor",
            DATA_TYPE_DEVINFO,
            payload_len_u32(DEVINFO_LEN),
        )?;
        let payload: [u8; DEVINFO_LEN] = self.read_payload("device info payload")?;
        Ok(info::parse_device_info(&payload))
    }

    /// Requests the device self-test health status.
    ///
    /// A [`HealthStatus::Error`] is returned as a *value*, not an `Err`:
    /// asking for the health of an unhealthy device is a successful query.
    ///
    /// # Errors
    ///
    /// [`LidarError::Timeout`] when the device does not answer, or
    /// [`LidarError::Protocol`] when the response is malformed.
    pub fn health(&mut self) -> Result<HealthStatus, LidarError> {
        self.request(Command::GetHealth)?;
        self.expect_descriptor(
            "device health descriptor",
            DATA_TYPE_DEVHEALTH,
            payload_len_u32(DEVHEALTH_LEN),
        )?;
        let payload: [u8; DEVHEALTH_LEN] = self.read_payload("device health payload")?;
        Ok(info::parse_health(&payload)?)
    }

    /// Stops any active scan and flushes stale input.
    ///
    /// Safe to call at any time; the device ignores `STOP` when idle.
    ///
    /// # Errors
    ///
    /// Any transport failure while writing or flushing.
    pub fn stop(&mut self) -> Result<(), LidarError> {
        self.request(Command::Stop)?;
        std::thread::sleep(self.config.stop_settle);
        self.discard_input()
    }

    /// Reboots the device, waits out the boot sequence and discards the
    /// unframed ASCII boot banner it prints.
    ///
    /// # Errors
    ///
    /// Any transport failure while writing or flushing.
    pub fn reset(&mut self) -> Result<(), LidarError> {
        self.request(Command::Reset)?;
        std::thread::sleep(self.config.reset_settle);
        self.discard_input()
    }

    /// Sets the motor PWM duty (`0..=1023`, clamped; `0` stops the motor),
    /// then waits for the speed to settle.
    ///
    /// The C1 manages its motor automatically for scanning, so calling this
    /// is optional on that model; it is required on externally-driven
    /// A-series units.
    ///
    /// # Errors
    ///
    /// Any transport failure while writing.
    pub fn set_motor_pwm(&mut self, pwm: u16) -> Result<(), LidarError> {
        self.request(Command::SetMotorPwm(pwm))?;
        std::thread::sleep(self.config.motor_settle);
        Ok(())
    }

    /// Begins a standard scan.
    ///
    /// Spins the motor (see [`LidarConfig::motor_pwm`]), sends `SCAN`, and
    /// validates the response descriptor. Follow with [`Lidar::scans`] to
    /// consume the measurement stream, and [`Lidar::stop`] when done.
    ///
    /// Consider checking [`Lidar::health`] first: a device in a protection
    /// state accepts the command but streams nothing useful.
    ///
    /// # Errors
    ///
    /// [`LidarError::Timeout`] when the device does not start streaming, or
    /// [`LidarError::Protocol`] when the response descriptor is not the
    /// standard-scan one.
    pub fn start_scan(&mut self) -> Result<(), LidarError> {
        self.discard_input()?;
        if self.config.motor_pwm > 0 {
            self.set_motor_pwm(self.config.motor_pwm)?;
        }
        self.request(Command::Scan)?;
        let descriptor = self.read_descriptor("scan descriptor")?;
        descriptor.expect(
            DATA_TYPE_MEASUREMENT,
            payload_len_u32(SCAN_NODE_LEN),
            SendMode::Multi,
        )?;
        Ok(())
    }

    /// Iterates over complete 360° rotations. Call after
    /// [`Lidar::start_scan`].
    ///
    /// Each item is one assembled [`Scan`]; the iterator ends (`None`) when
    /// the byte source is exhausted, which never happens on live hardware —
    /// stop by dropping the iterator and calling [`Lidar::stop`]. Read
    /// failures and unrecoverable desyncs surface as `Some(Err(_))`.
    pub fn scans(&mut self) -> Scans<'_, T> {
        Scans {
            lidar: self,
            current: Vec::new(),
            primed: false,
            started: false,
            done: false,
        }
    }

    /// The active configuration.
    #[must_use]
    pub fn config(&self) -> &LidarConfig {
        &self.config
    }

    /// Consumes the `Lidar` and returns the underlying transport.
    pub fn into_transport(self) -> T {
        self.transport
    }

    fn request(&mut self, command: Command) -> Result<(), LidarError> {
        let mut buf = [0u8; MAX_REQUEST_LEN];
        let len = command.encode(&mut buf);
        self.transport
            .write_all(&buf[..len])
            .map_err(|e| lift(e, "request write", Duration::ZERO))
    }

    fn discard_input(&mut self) -> Result<(), LidarError> {
        self.transport
            .discard_input()
            .map_err(|e| lift(e, "input flush", Duration::ZERO))
    }

    fn expect_descriptor(
        &mut self,
        what: &'static str,
        data_type: u8,
        len: u32,
    ) -> Result<(), LidarError> {
        let descriptor = self.read_descriptor(what)?;
        descriptor.expect(data_type, len, SendMode::Single)?;
        Ok(())
    }

    fn read_descriptor(&mut self, what: &'static str) -> Result<ResponseDescriptor, LidarError> {
        let bytes: [u8; descriptor::DESCRIPTOR_LEN] = self.read_payload(what)?;
        Ok(descriptor::parse_descriptor(&bytes)?)
    }

    fn read_payload<const N: usize>(&mut self, what: &'static str) -> Result<[u8; N], LidarError> {
        let timeout = self.config.response_timeout;
        let mut bytes = [0u8; N];
        self.transport
            .read_exact(&mut bytes, timeout)
            .map_err(|e| lift(e, what, timeout))?;
        Ok(bytes)
    }

    /// Reads the next valid scan node, recovering from desync by discarding
    /// one byte at a time. `Ok(None)` means the byte source ended cleanly
    /// (replay exhausted).
    fn next_node(&mut self, timeout: Duration) -> Result<Option<ScanNode>, LidarError> {
        let mut buf = [0u8; SCAN_NODE_LEN];
        match self.transport.read_exact(&mut buf, timeout) {
            Err(TransportError::Eof) => return Ok(None),
            other => other.map_err(|e| lift(e, "scan node", timeout))?,
        }

        let mut discarded = 0;
        loop {
            match scan_node::parse_scan_node(&buf) {
                Ok(node) => return Ok(Some(node)),
                Err(_) if discarded < MAX_DESYNC_DISCARD => {
                    // Slide the window one byte and pull one more.
                    discarded += 1;
                    buf.copy_within(1.., 0);
                    let last = &mut buf[SCAN_NODE_LEN - 1..];
                    match self.transport.read_exact(last, timeout) {
                        Err(TransportError::Eof) => return Ok(None),
                        other => other.map_err(|e| lift(e, "scan node resync", timeout))?,
                    }
                }
                Err(_) => {
                    return Err(LidarError::Desync(format!(
                        "no valid scan node found after discarding {discarded} bytes; \
                         is another program reading the port?"
                    )));
                }
            }
        }
    }
}

/// Iterator over assembled 360° rotations. Created by [`Lidar::scans`].
#[derive(Debug)]
pub struct Scans<'a, T: Transport> {
    lidar: &'a mut Lidar<T>,
    current: Vec<Point>,
    /// A rotation only starts at a start-flag node; until the first one
    /// arrives, points belong to a partial rotation and are dropped.
    primed: bool,
    /// The first node gets [`LidarConfig::scan_startup_timeout`] (motor
    /// spin-up); the rest get [`LidarConfig::response_timeout`].
    started: bool,
    done: bool,
}

impl<T: Transport> Iterator for Scans<'_, T> {
    type Item = Result<Scan, LidarError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        loop {
            let timeout = if self.started {
                self.lidar.config.response_timeout
            } else {
                self.lidar.config.scan_startup_timeout
            };
            let node = match self.lidar.next_node(timeout) {
                Ok(Some(node)) => {
                    self.started = true;
                    node
                }
                Ok(None) => {
                    // Clean end of stream; a trailing partial rotation is
                    // dropped rather than yielded as a bogus "scan".
                    self.done = true;
                    return None;
                }
                Err(e) => {
                    self.done = true;
                    return Some(Err(e));
                }
            };

            if node.start_flag {
                let finished = if self.primed && !self.current.is_empty() {
                    Some(Scan::new(std::mem::take(&mut self.current), Instant::now()))
                } else {
                    None
                };
                self.primed = true;
                self.current.clear();
                self.current.push(Point::from(&node));
                if let Some(scan) = finished {
                    return Some(Ok(scan));
                }
            } else if self.primed {
                self.current.push(Point::from(&node));
            }
        }
    }
}

/// Payload lengths are tiny protocol constants; the cast cannot truncate.
#[allow(clippy::cast_possible_truncation)]
const fn payload_len_u32(len: usize) -> u32 {
    len as u32
}

/// Upgrades a context-free transport error into a `LidarError` that names
/// what the device layer was doing.
fn lift(e: TransportError, what: &'static str, timeout: Duration) -> LidarError {
    match e {
        TransportError::Timeout => LidarError::Timeout {
            what,
            ms: u64::try_from(timeout.as_millis()).unwrap_or(u64::MAX),
        },
        TransportError::Serial(e) => LidarError::Serial(e),
        other => LidarError::Transport(other),
    }
}
