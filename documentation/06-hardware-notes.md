# 06 — Hardware field notes (RPLIDAR C1)

Facts learned from the physical device on the bench. Datasheets approximate;
these numbers were measured.

## The development unit

| Property | Value |
|---|---|
| Model byte | `0x41` ("RPLIDAR C1") |
| Firmware | 1.02 |
| Hardware revision | 18 |
| USB bridge | CP210x (VID `0x10C4`, PID `0xEA60`) |
| Port on the dev Mac | `/dev/cu.usbserial-1130` |
| Rotation rate | about 10 Hz (measured 9.9 - 10.2) |
| Nodes per rotation | about 508 - 516 |
| Effective sample rate | about 5.1 kHz |
| Valid returns indoors | about 410 of 512 per rotation (zeros are no-return) |
| Motor spin-up to first node | about 2 s after SCAN |

Soak result: 583 consecutive scans over 60.1 s with zero desynchronizations.

## Behaviors that drive the code

### Motor spin-up vs. instant descriptor

After `SCAN`, the descriptor arrives immediately but nodes only start once the
motor is at speed (about 2 s). This is why `LidarConfig::scan_startup_timeout`
(5 s) exists and applies only to the first node. See
[04-development-process.md](04-development-process.md) for the bug story.

### The device never stops streaming on its own

Once scanning, the C1 streams until STOP (or power loss). A crashed client
leaves it streaming, so the next `Lidar::open` performs STOP, 50 ms settle,
input flush — always. Symptom if this were missing: descriptor parsing fails
with `BadSync` on bytes that are actually mid-stream scan nodes.

### RESET prints a boot banner

`RESET` (0x40) reboots the device, which then emits an unframed ASCII banner
(not descriptor-framed). `reset()` waits 500 ms and flushes input. Never try
to parse bytes arriving during that window.

### Motor control is the PWM command, not DTR

On the C1, the motor is controlled by the accessory command `SET_MOTOR_PWM`
(0xF0) and largely self-managed during scanning. DTR does nothing. This is the
opposite of the A1/A2, whose motors are wired to the adapter's DTR pin —
that is the main hardware difference to handle when adding those models (hook
point marked with a comment in `transport/serial.rs`).

## Serial port handling by OS

| OS | Device path | Notes |
|---|---|---|
| macOS | `/dev/cu.usbserial-*` | **Always the `cu.` callout device.** The `tty.` variant can block on open waiting for carrier-detect. `transport::prefer_callout_device` rewrites `tty.` to `cu.` |
| Linux | `/dev/ttyUSB0` (CP210x) or `/dev/ttyACM*` | user must be in the `dialout` group (`sudo usermod -aG dialout $USER`, re-login) |
| Windows | `COM3` etc. | enumeration via VID/PID works the same |

Auto-detection (`transport::auto_detect_port`) preference order:

1. USB port matching a known bridge (CP210x `10C4:EA60`, CH340 `1A86:7523`)
2. any other USB serial port
3. name patterns (`cu.usbserial*`, `cu.SLAB_USBtoUART`, `cu.wchusbserial*`,
   `ttyUSB*`, `ttyACM*`)

Practical gotchas observed:

- A device plugged in through some USB hubs at boot may silently fail to
  enumerate; replug directly.
- `serialport` is used with `default-features = false`: on Linux this avoids
  linking libudev (C library) — enumeration falls back to sysfs and still
  reports VID/PID (serialport >= 4.2).

## Timeout surfacing differs per platform

The OS reports a serial read timeout in different ways: `ErrorKind::TimedOut`,
`ErrorKind::WouldBlock`, or a zero-byte read. `SerialTransport` normalizes all
of them into `TransportError::Timeout` / `Eof` so no platform difference leaks
past the transport layer.

## Sanity numbers for plausibility checks

When something looks wrong, compare against these:

- 460800 baud is about 46 KB/s; the standard scan stream is
  5 bytes x 5.1 kHz = about 25.5 KB/s — roughly half the link. This is why
  express scan is unnecessary on the C1 and mandatory on 16-32 kHz models.
- A healthy indoor rotation covers close to 360.0 degrees
  (`Scan::angular_coverage`); persistent values far below that mean dropped
  nodes or a desynchronizing link.
- Quality is 6 bits (0..=63); indoor returns on this unit cluster in the
  40s. Quality 0 accompanies invalid (zero-distance) measurements.
