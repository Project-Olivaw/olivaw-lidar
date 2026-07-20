# 07 — Future work, gaps, and improvement opportunities

The C1 is fully supported; nothing here is required to use it. This document
is the honest map of what is missing, what could be better, and where this
driver can aim to beat the reference implementations.

## The ground rule

For every item below that touches the wire protocol: **record real fixtures
from the target device first** (`examples/record.rs`), then build and test the
parser offline. This is how the C1 support was built and why it worked on the
first live run (except for one timing bug — see
[04-development-process.md](04-development-process.md)).

## Protocol features not yet implemented

### Express scan (0x82)

- What: 84-byte checksummed "capsules" of 32 measurements with delta-angle
  encoding — about half the bytes per sample of the standard 5-byte node.
- Why not yet: the C1's about 5 kHz fits comfortably through standard scan at
  460800 baud (measured about 25.5 KB/s of a 46 KB/s link). Express buys
  nothing on this hardware.
- When it becomes mandatory: A3/S-series at 16 - 32 kHz.
- Implementation sketch: `protocol/express.rs` capsule parser (stateful across
  capsules — angles are deltas from the previous capsule's start angle),
  activate the dormant `ProtocolError::Checksum` variant, request framing with
  payload (the `Command` enum and `MAX_REQUEST_LEN = 12` already leave room),
  fixtures from a device that actually streams express mode.

### GET_SAMPLERATE (0x59) device method

`Command::GetSampleRate` and the descriptor data type (`0x15`, 4-byte payload:
two u16 LE microsecond durations) exist, but `Lidar` has no `sample_rate()`
method yet. Small, self-contained addition; needs a fixture.

### GET_LIDAR_CONF (0x84)

Scan-mode discovery for A3/S-series (enumerating supported modes, their
sample rates and answer types). Required before those models can pick their
best mode the way SLAMTEC's own SDK does.

## Additional models

| Model | Baud | Work needed |
|---|---|---|
| A1 | 115200 | DTR-pin motor control — POSIX asserts DTR on open, and the A1 motor is wired to it; needs explicit control in `SerialTransport::open` (hook point commented there) plus a config preset |
| A2 | 256000 | DTR motor control; express scan recommended |
| A3 | 256000 | GET_LIDAR_CONF + express/dense scan |
| S1/S2 | 256000 | as A3; TOF ranging, different range limits |

Per model: `model_name()` byte mapping, `LidarConfig` preset, fixtures, and a
hardware soak with `basic_scan`.

## Known gaps and design debts

- **`Scan.timestamp` is set at assembly completion**, not at measurement
  time. For SLAM, per-point timestamps (interpolated across the rotation)
  would deskew motion. This matters once `olivaw-slam` does scan matching
  from a moving robot.
- **No motor speed feedback or control loop.** The C1 self-manages; A-series
  PWM is open-loop. The protocol offers no RPM telemetry on these models, but
  rotation rate can be estimated from start-flag intervals and exposed.
- **`Scans` uses a blocking iterator.** Deliberate (the spec forbids async —
  a driver is a blocking read loop), but a `std::sync::mpsc`-based background
  reader could be layered on top without touching the core, if an application
  needs non-blocking consumption.
- **Fixtures are from firmware 1.02 only.** Re-record after any firmware
  update; the recorder makes this a one-command task.
- **The 4096-byte desync budget (`MAX_DESYNC_DISCARD`) is a heuristic.**
  Post-restart garbage measured well under 1 KB; revisit if a future device
  behaves differently.

## Where this driver can be better than the alternatives

Measured against the SLAMTEC C++ SDK and typical community drivers:

1. **Testability.** The SDK has no offline replay; this driver's replay
   transport runs the full stack on recorded bytes in CI. Keep this
   property absolute — it is the moat.
2. **Recovery guarantees.** Byte-exact desync behavior is pinned by tests
   (garbage costs zero nodes, corruption costs exactly one). No other RPLIDAR
   driver we examined documents, let alone tests, its resync loss bound.
3. **Correctness details.** The firmware version byte order is decoded
   correctly (a known bug in at least one popular Python driver); errors are
   specific and actionable; timeouts name what was being awaited.
4. **Deployment.** Pure Rust, no libudev, `no_std` parser core: one
   `cargo build` on a Pi, potential ESP32 reuse of `protocol/`.
5. **Honest scope.** SLAM, transforms, and visualization live elsewhere
   (rerun for viz, future `olivaw-slam`). Compare SLAM_toolbox: that is a
   consumer of drivers like this one, not a competitor.

## Release chores

- [ ] README GIF at `.github/assets/viz.gif` (screen-record `rerun_viz`, or
  replay the saved `.rrd` and capture that)
- [ ] License decision: single MIT (current `LICENSE`) vs MIT + Apache-2.0
  dual (the Rust ecosystem default, and what the original spec's layout
  implies) — must be settled before crates.io publish
- [ ] Initial commit / commit history, then publish `0.1.0` to crates.io
- [ ] Optional: `cargo semver-checks` in CI once published
