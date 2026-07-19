//! Device info and health response parsing.

use core::fmt;

use super::ProtocolError;

/// Payload length of a `GET_INFO` response.
pub const DEVINFO_LEN: usize = 20;
/// Payload length of a `GET_HEALTH` response.
pub const DEVHEALTH_LEN: usize = 3;
/// Model byte reported by the RPLIDAR C1.
pub const MODEL_C1: u8 = 0x41;

/// Device identity: model, versions and serial number.
///
/// Returned by `Lidar::info` (with the `std` feature) or parsed directly
/// from wire bytes with [`parse_device_info`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceInfo {
    /// Raw model identifier byte; [`MODEL_C1`] for the C1.
    pub model: u8,
    /// Firmware version as `(major, minor)`.
    pub firmware_version: (u8, u8),
    /// Hardware revision number.
    pub hardware_version: u8,
    /// 128-bit unique serial number, as raw bytes. Use
    /// [`DeviceInfo::serial_number_hex`] for the conventional 32-character
    /// hex rendering.
    pub serial_number: [u8; 16],
}

impl DeviceInfo {
    /// The marketing name of the model, if this crate knows it.
    #[must_use]
    pub const fn model_name(&self) -> Option<&'static str> {
        match self.model {
            MODEL_C1 => Some("RPLIDAR C1"),
            _ => None,
        }
    }

    /// The serial number as a 32-character uppercase-hex [`Display`] adapter.
    ///
    /// Allocation-free, so it is usable from `no_std` code:
    ///
    /// [`Display`]: core::fmt::Display
    #[must_use]
    pub const fn serial_number_hex(&self) -> SerialNumberHex {
        SerialNumberHex(self.serial_number)
    }
}

/// [`Display`](core::fmt::Display) adapter rendering a serial number as
/// 32 uppercase hex characters. Created by [`DeviceInfo::serial_number_hex`].
#[derive(Debug, Clone, Copy)]
pub struct SerialNumberHex([u8; 16]);

impl fmt::Display for SerialNumberHex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(f, "{byte:02X}")?;
        }
        Ok(())
    }
}

/// Device self-test result reported by `GET_HEALTH`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthStatus {
    /// The device is operating normally.
    Good,
    /// The device works but flagged a potential issue; the payload is the
    /// device-specific warning code.
    Warning(u16),
    /// The device is in a protection state and will not scan; the payload is
    /// the device-specific error code. A power cycle usually clears it.
    Error(u16),
}

impl HealthStatus {
    /// `true` when the status is [`HealthStatus::Good`].
    #[must_use]
    pub const fn is_good(&self) -> bool {
        matches!(self, Self::Good)
    }
}

/// Parses a `GET_INFO` response payload.
///
/// Infallible: every bit pattern is a valid `DeviceInfo`. An unknown model
/// byte is preserved as-is (see [`DeviceInfo::model_name`]).
///
/// The firmware version is transmitted as a little-endian `u16` with the
/// **minor** version in the low byte — i.e. wire offset 1 is minor and wire
/// offset 2 is major.
#[must_use]
pub fn parse_device_info(bytes: &[u8; DEVINFO_LEN]) -> DeviceInfo {
    let mut serial_number = [0u8; 16];
    serial_number.copy_from_slice(&bytes[4..20]);
    DeviceInfo {
        model: bytes[0],
        firmware_version: (bytes[2], bytes[1]),
        hardware_version: bytes[3],
        serial_number,
    }
}

/// Parses a `GET_HEALTH` response payload.
///
/// # Errors
///
/// [`ProtocolError::InvalidHealthStatus`] if the status byte is outside
/// `0..=2`.
pub fn parse_health(bytes: &[u8; DEVHEALTH_LEN]) -> Result<HealthStatus, ProtocolError> {
    let code = u16::from_le_bytes([bytes[1], bytes[2]]);
    match bytes[0] {
        0 => Ok(HealthStatus::Good),
        1 => Ok(HealthStatus::Warning(code)),
        2 => Ok(HealthStatus::Error(code)),
        status => Err(ProtocolError::InvalidHealthStatus(status)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_device_info_with_firmware_byte_order() {
        // Firmware 1.25: wire carries minor (0x19 = 25) before major (0x01).
        let mut payload = [0u8; DEVINFO_LEN];
        payload[0] = MODEL_C1;
        payload[1] = 0x19;
        payload[2] = 0x01;
        payload[3] = 0x12;
        for (i, byte) in payload[4..20].iter_mut().enumerate() {
            // Cast is fine: i < 16.
            #[allow(clippy::cast_possible_truncation)]
            {
                *byte = 0xF0 + i as u8;
            }
        }

        let info = parse_device_info(&payload);
        assert_eq!(info.model, MODEL_C1);
        assert_eq!(info.model_name(), Some("RPLIDAR C1"));
        assert_eq!(info.firmware_version, (1, 25));
        assert_eq!(info.hardware_version, 0x12);
        assert_eq!(info.serial_number[0], 0xF0);
        assert_eq!(info.serial_number[15], 0xFF);
    }

    #[test]
    fn unknown_model_is_preserved_not_rejected() {
        let mut payload = [0u8; DEVINFO_LEN];
        payload[0] = 0x99;
        let info = parse_device_info(&payload);
        assert_eq!(info.model, 0x99);
        assert_eq!(info.model_name(), None);
    }

    #[test]
    fn serial_number_hex_renders_32_uppercase_chars() {
        let info = DeviceInfo {
            model: MODEL_C1,
            firmware_version: (1, 1),
            hardware_version: 1,
            serial_number: [
                0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x0F,
            ],
        };
        // std is available under `cargo test`; format! exercises the core::fmt impl.
        let hex = std::format!("{}", info.serial_number_hex());
        assert_eq!(hex, "ABCDEF0123456789000000000000000F");
        assert_eq!(hex.len(), 32);
    }

    #[test]
    fn parses_health_statuses() {
        assert_eq!(parse_health(&[0, 0, 0]), Ok(HealthStatus::Good));
        assert!(parse_health(&[0, 0, 0]).unwrap().is_good());
        // Code 0x0201 little-endian.
        assert_eq!(
            parse_health(&[1, 0x01, 0x02]),
            Ok(HealthStatus::Warning(0x0201))
        );
        assert_eq!(
            parse_health(&[2, 0xFF, 0x00]),
            Ok(HealthStatus::Error(0xFF))
        );
        assert!(!parse_health(&[2, 0xFF, 0x00]).unwrap().is_good());
    }

    #[test]
    fn rejects_invalid_health_status_byte() {
        assert_eq!(
            parse_health(&[3, 0, 0]),
            Err(ProtocolError::InvalidHealthStatus(3))
        );
        assert_eq!(
            parse_health(&[0xFF, 0, 0]),
            Err(ProtocolError::InvalidHealthStatus(0xFF))
        );
    }
}
