//! Request framing: command opcodes and wire encoding.
//!
//! Requests are `0xA5 <opcode>`, or for commands carrying a payload,
//! `0xA5 <opcode> <payload len> <payload…> <checksum>` where the checksum is
//! the XOR of every preceding byte in the frame.

/// First byte of every request frame and of every response descriptor.
pub const SYNC_BYTE: u8 = 0xA5;

/// Length of the longest request frame this crate can emit.
///
/// Sized with headroom for the express-scan request (5-byte payload, 9 bytes
/// framed) so adding it later does not change this constant.
pub const MAX_REQUEST_LEN: usize = 12;

/// Maximum motor PWM duty value accepted by the device.
pub const MOTOR_PWM_MAX: u16 = 1023;

/// A request that can be sent to the device.
///
/// Encode one into wire bytes with [`Command::encode`]:
///
/// ```
/// use olivaw_lidar::protocol::{Command, MAX_REQUEST_LEN};
///
/// let mut buf = [0u8; MAX_REQUEST_LEN];
/// let len = Command::GetInfo.encode(&mut buf);
/// assert_eq!(&buf[..len], &[0xA5, 0x50]);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Command {
    /// Exit scanning mode and enter idle (`0x25`). No response.
    Stop,
    /// Reboot the device (`0x40`). No framed response, but the device emits
    /// an unframed ASCII boot banner that must be discarded.
    Reset,
    /// Begin a standard scan (`0x20`). Multi-response: an endless stream of
    /// 5-byte measurement nodes follows one descriptor.
    Scan,
    /// Request the device model, firmware, hardware revision and serial
    /// number (`0x50`).
    GetInfo,
    /// Request the device self-test health status (`0x52`).
    GetHealth,
    /// Request the standard/express sample durations (`0x59`).
    GetSampleRate,
    /// Set the motor PWM duty, `0..=`[`MOTOR_PWM_MAX`] (`0xF0`).
    ///
    /// Values above [`MOTOR_PWM_MAX`] are clamped during encoding. `0` stops
    /// the motor.
    SetMotorPwm(u16),
}

impl Command {
    /// The opcode byte sent after [`SYNC_BYTE`].
    #[must_use]
    pub const fn opcode(self) -> u8 {
        match self {
            Self::Stop => 0x25,
            Self::Reset => 0x40,
            Self::Scan => 0x20,
            Self::GetInfo => 0x50,
            Self::GetHealth => 0x52,
            Self::GetSampleRate => 0x59,
            Self::SetMotorPwm(_) => 0xF0,
        }
    }

    /// Encodes the framed request into `out` and returns the frame length.
    ///
    /// Infallible: [`MAX_REQUEST_LEN`] bounds every variant by construction.
    /// Commands with a payload gain a length byte, the payload, and an XOR
    /// checksum of all preceding bytes.
    #[must_use]
    pub fn encode(self, out: &mut [u8; MAX_REQUEST_LEN]) -> usize {
        out[0] = SYNC_BYTE;
        out[1] = self.opcode();
        match self {
            Self::SetMotorPwm(pwm) => {
                let payload = pwm.min(MOTOR_PWM_MAX).to_le_bytes();
                out[2] = 2; // payload length
                out[3] = payload[0];
                out[4] = payload[1];
                out[5] = xor_checksum(&out[..5]);
                6
            }
            _ => 2,
        }
    }
}

/// XOR of all bytes; the request-frame checksum.
pub(crate) fn xor_checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |acc, b| acc ^ b)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encoded(cmd: Command) -> ([u8; MAX_REQUEST_LEN], usize) {
        let mut buf = [0u8; MAX_REQUEST_LEN];
        let len = cmd.encode(&mut buf);
        (buf, len)
    }

    #[test]
    fn payloadless_commands_are_two_bytes() {
        for (cmd, opcode) in [
            (Command::Stop, 0x25),
            (Command::Reset, 0x40),
            (Command::Scan, 0x20),
            (Command::GetInfo, 0x50),
            (Command::GetHealth, 0x52),
            (Command::GetSampleRate, 0x59),
        ] {
            let (buf, len) = encoded(cmd);
            assert_eq!(&buf[..len], &[0xA5, opcode], "{cmd:?}");
        }
    }

    #[test]
    fn set_motor_pwm_frames_with_checksum() {
        // 660 = 0x0294, little-endian payload [0x94, 0x02];
        // checksum = 0xA5 ^ 0xF0 ^ 0x02 ^ 0x94 ^ 0x02 = 0xC1.
        let (buf, len) = encoded(Command::SetMotorPwm(660));
        assert_eq!(&buf[..len], &[0xA5, 0xF0, 0x02, 0x94, 0x02, 0xC1]);
    }

    #[test]
    fn set_motor_pwm_zero() {
        // checksum = 0xA5 ^ 0xF0 ^ 0x02 = 0x57.
        let (buf, len) = encoded(Command::SetMotorPwm(0));
        assert_eq!(&buf[..len], &[0xA5, 0xF0, 0x02, 0x00, 0x00, 0x57]);
    }

    #[test]
    fn set_motor_pwm_max() {
        // 1023 = 0x03FF; checksum = 0xA5 ^ 0xF0 ^ 0x02 ^ 0xFF ^ 0x03 = 0xAB.
        let (buf, len) = encoded(Command::SetMotorPwm(1023));
        assert_eq!(&buf[..len], &[0xA5, 0xF0, 0x02, 0xFF, 0x03, 0xAB]);
    }

    #[test]
    fn set_motor_pwm_clamps_above_max() {
        assert_eq!(
            encoded(Command::SetMotorPwm(2000)),
            encoded(Command::SetMotorPwm(1023))
        );
        assert_eq!(
            encoded(Command::SetMotorPwm(u16::MAX)),
            encoded(Command::SetMotorPwm(1023))
        );
    }

    #[test]
    fn xor_checksum_basics() {
        assert_eq!(xor_checksum(&[]), 0x00);
        assert_eq!(xor_checksum(&[0xFF]), 0xFF);
        assert_eq!(xor_checksum(&[0xA5, 0xA5]), 0x00);
        assert_eq!(xor_checksum(&[0x01, 0x02, 0x04]), 0x07);
    }
}
