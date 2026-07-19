# CLAUDE.md — olivaw-lidar

> Pure-Rust driver for SLAMTEC RPLIDAR devices. No C++ SDK, no FFI, no bindgen.
> Part of [Project Olivaw](https://github.com/Project-Olivaw) — tools and examples for robotics in Rust.

---

## Project intent

`olivaw-lidar` is a **pure Rust** driver for SLAMTEC RPLIDAR units, targeting the **C1** first and designed to extend to A1/A2/A3/S1/S2 later.

**Why pure Rust matters here (do not compromise on this):**

- No `build.rs` invoking a C++ toolchain
- Cross-compiles cleanly to `aarch64-unknown-linux-gnu` (Raspberry Pi) and `aarch64-apple-darwin` (M-series Mac) with zero extra setup
- No `unsafe` FFI boundary
- The parsing core is `no_std`-compatible so it can eventually run on an ESP32

If a design decision would require linking C or C++, reject it and find another way.

**Primary author's hardware for testing:** SLAMTEC C1, MacBook M3 (dev), Raspberry Pi 4 2GB (deployment), Jetson Nano 4GB (later).

---

## Non-negotiable architecture rule

**Separate protocol parsing from I/O.** This is the single most important structural decision in the crate.

```
┌─────────────────────────────────────────────────┐
│  protocol/  — PURE. bytes in, typed values out. │
│               no_std. no I/O. fully unit-tested │
│               against recorded byte fixtures.   │
└─────────────────────────────────────────────────┘
                       ▲
                       │
┌─────────────────────────────────────────────────┐
│  transport/ — trait-based I/O.                  │
│               SerialTransport (real hardware)   │
│               ReplayTransport (recorded file)   │
│               MockTransport   (unit tests)      │
└─────────────────────────────────────────────────┘
                       ▲
                       │
┌─────────────────────────────────────────────────┐
│  device/    — high-level API. Owns a transport, │
│               drives the state machine, yields  │
│               Scan values.                      │
└─────────────────────────────────────────────────┘
```

Consequences of this split, all of which we want:
- CI runs the full parser test suite with **no hardware attached**
- The scan matcher (future `olivaw-slam`) can be developed against recorded data
- Swapping to a different transport (TCP, UDP, ESP32 UART) is a new impl, not a rewrite

---

## Protocol reference

The RPLIDAR serial protocol is documented in SLAMTEC's public protocol spec and readable in the
[reference SDK](https://github.com/slamtec/rplidar_sdk). **Read those as reference; reimplement in Rust.**
Do not vendor, translate line-by-line, or copy C++ source into this repo.

### Connection parameters

| Model | Baud rate | Notes |
|---|---|---|
| C1 | **460800** | primary target |
| A1 | 115200 | later |
| A2/A3 | 256000 | later |
| S1/S2 | 256000 | later |

8 data bits, no parity, 1 stop bit. On macOS the device typically appears as `/dev/tty.usbserial-*`,
on Linux as `/dev/ttyUSB*`.

### Request frame format

```
┌────────┬────────┬─────────┬──────────┬──────────┐
│ 0xA5   │ cmd    │ payload │ payload  │ checksum │
│ start  │ byte   │ size    │ bytes    │ (if any) │
└────────┴────────┴─────────┴──────────┴──────────┘
```

Commands without payload are just `0xA5 <cmd>`. Commands with payload append a size byte,
the payload, and a checksum which is the XOR of all preceding bytes.

### Key commands to implement

| Command | Byte | Response type | Priority |
|---|---|---|---|
| `STOP` | `0x25` | none | **P0** |
| `RESET` | `0x40` | none | **P0** |
| `SCAN` | `0x20` | multi-response | **P0** |
| `EXPRESS_SCAN` | `0x82` | multi-response | **P1** |
| `GET_INFO` | `0x50` | single response | **P0** |
| `GET_HEALTH` | `0x52` | single response | **P0** |
| `GET_SAMPLERATE` | `0x59` | single response | P2 |

### Response descriptor

Every response is preceded by a 7-byte descriptor:

```
0xA5 0x5A <len[29:0] + mode[1:0] packed in 4 bytes> <data_type>
```

Parse the 30-bit length and 2-bit send-mode out of the packed field. Send mode `0x0` means a single
response; `0x1` means multiple responses follow (used by SCAN).

### Standard scan data node — 5 bytes

```
byte 0:  [quality:6][!S:1][S:1]      S = start flag, !S must be inverse of S
byte 1:  [angle_low:7][C:1]          C = check bit, must be 1
byte 2:  [angle_high:8]
byte 3:  [distance_low:8]
byte 4:  [distance_high:8]
```

- `angle` = `((byte2 << 7) | (byte1 >> 1))` / 64.0  → degrees
- `distance` = `((byte4 << 8) | byte3)` / 4.0  → millimeters
- `quality` = `byte0 >> 2`
- `start_flag` = `byte0 & 0x01` — marks the beginning of a new 360° rotation
- **Validity checks:** `S != !S` and `C == 1`. If either fails, the stream is desynchronized —
  discard bytes one at a time until sync is regained.

A `distance` of 0 means an invalid/no-return measurement. Keep these in the raw scan but
filter them in convenience methods.

### Express scan (P1, do after standard scan works)

Express mode packs data into 84-byte packets with delta-angle encoding for higher sample rates.
Implement only after the standard scan path is solid and tested.

---

## Public API shape

Design for this to feel obvious to someone who has never used a lidar before.

```rust
use olivaw_lidar::{Lidar, LidarConfig};

// Simplest possible path
let mut lidar = Lidar::open("/dev/tty.usbserial-0001")?;
println!("{:?}", lidar.info()?);
println!("{:?}", lidar.health()?);

lidar.start_scan()?;
for scan in lidar.scans() {
    let scan = scan?;
    println!("{} points, {:.1}° coverage", scan.len(), scan.angular_coverage());
    for p in scan.valid_points() {
        // p.angle_deg, p.distance_mm, p.quality
    }
}
```

### Core types

```rust
/// A single measurement.
pub struct Point {
    pub angle_deg: f32,
    pub distance_mm: f32,
    pub quality: u8,
}

impl Point {
    pub fn is_valid(&self) -> bool;          // distance > 0
    pub fn to_cartesian(&self) -> (f32, f32); // meters, x-forward y-left
}

/// One full 360° rotation, assembled from points between start flags.
pub struct Scan {
    points: Vec<Point>,
    pub timestamp: std::time::Instant,
}

impl Scan {
    pub fn len(&self) -> usize;
    pub fn valid_points(&self) -> impl Iterator<Item = &Point>;
    pub fn angular_coverage(&self) -> f32;
    pub fn to_cartesian(&self) -> Vec<(f32, f32)>;
}

pub struct DeviceInfo {
    pub model: u8,
    pub firmware_version: (u8, u8),
    pub hardware_version: u8,
    pub serial_number: [u8; 16],
}

pub enum HealthStatus { Good, Warning(u16), Error(u16) }
```

### Error type

Use `thiserror`. Be specific — errors should tell the user what to *do*.

```rust
#[derive(Debug, thiserror::Error)]
pub enum LidarError {
    #[error("serial port error: {0}")]
    Serial(#[from] serialport::Error),

    #[error("timed out waiting for {what} after {ms}ms")]
    Timeout { what: &'static str, ms: u64 },

    #[error("protocol desync: {0}")]
    Desync(String),

    #[error("bad checksum: expected {expected:#04x}, got {actual:#04x}")]
    Checksum { expected: u8, actual: u8 },

    #[error("device reported error status: {0}")]
    DeviceError(u16),

    #[error("unsupported model: {0:#04x}")]
    UnsupportedModel(u8),
}
```

---

## Repository layout

```
olivaw-lidar/
├── Cargo.toml
├── README.md
├── LICENSE-MIT
├── LICENSE-APACHE
├── CLAUDE.md                    ← this file
├── src/
│   ├── lib.rs                   ← re-exports, crate docs
│   ├── error.rs
│   ├── protocol/
│   │   ├── mod.rs
│   │   ├── command.rs           ← command bytes, request framing
│   │   ├── descriptor.rs        ← 7-byte response descriptor parsing
│   │   ├── scan_node.rs         ← 5-byte node parsing + validity
│   │   ├── express.rs           ← (P1) express scan packets
│   │   └── info.rs              ← device info / health parsing
│   ├── transport/
│   │   ├── mod.rs               ← Transport trait
│   │   ├── serial.rs            ← real hardware
│   │   ├── replay.rs            ← read from recorded byte dump
│   │   └── mock.rs              ← test double
│   ├── device.rs                ← Lidar struct, state machine, scan assembly
│   └── types.rs                 ← Point, Scan, DeviceInfo, HealthStatus
├── examples/
│   ├── info.rs                  ← print device info + health, exit
│   ├── basic_scan.rs            ← print scans to stdout
│   ├── record.rs                ← dump raw bytes to file for fixtures
│   └── rerun_viz.rs             ← live point cloud in rerun
├── tests/
│   ├── protocol_tests.rs        ← parser tests against fixtures
│   └── fixtures/
│       ├── c1_info_response.bin
│       ├── c1_health_response.bin
│       └── c1_scan_1000_nodes.bin
└── .github/workflows/ci.yml
```

---

## Dependencies

Keep this list short. Every dependency is a maintenance liability.

```toml
[dependencies]
serialport = "4"           # cross-platform serial, pure Rust
thiserror = "2"

[dev-dependencies]
rerun = "0.33"             # visualization in examples only
clap = { version = "4", features = ["derive"] }
anyhow = "1"
```

**Do not add:** tokio (not needed — the driver is a blocking read loop; async adds nothing here),
nalgebra (no linear algebra in a driver), any C-linking crate.

The `protocol` module must not depend on `serialport` or `std`. Gate `std` behind a default feature
so `no_std` users can take just the parser.

---

## Implementation order

Work in this order. Each step should end with something runnable.

### Step 1 — Skeleton + protocol constants
Cargo project, error type, command byte constants, request framing function. Unit test the framing
(checksum calculation) with hand-computed expected bytes.

### Step 2 — Descriptor + info/health parsing
Parse the 7-byte descriptor. Parse `GET_INFO` and `GET_HEALTH` responses. Unit tests with
hand-constructed byte arrays.

### Step 3 — Serial transport + `examples/info.rs`
**First hardware milestone.** Open the port, send `GET_INFO`, print the result.
When this prints a real serial number from the C1, the foundation is proven.

### Step 4 — `examples/record.rs`
Dump raw bytes from a scan session to `fixtures/`. Run this once with real hardware and you
have test data forever. **Do this before writing the scan parser** — it means the parser can be
developed and tested offline.

### Step 5 — Scan node parsing
5-byte node parser with validity checks. Test against the recorded fixture from step 4.
This is pure logic — no hardware needed.

### Step 6 — Scan assembly + `Lidar::scans()` iterator
Buffer nodes, split on start flag, yield complete `Scan` values. Handle desync recovery:
on an invalid node, advance one byte and retry rather than aborting.

### Step 7 — `examples/basic_scan.rs`
**Second hardware milestone.** Live scans printing to stdout.

### Step 8 — `examples/rerun_viz.rs`
Log points as a 2D point cloud to rerun. This is the screenshot for the README.

### Step 9 — Polish for release
Crate-level docs, README with a GIF, CI running tests on the fixtures, publish `0.1.0`.

### Later (P1/P2)
Express scan mode, motor PWM control, additional device models, `no_std` verification.

---

## Testing strategy

**Unit tests (no hardware, run in CI):** every parser function tested against byte fixtures.
Include deliberately corrupted fixtures to verify desync recovery.

**Integration tests (no hardware, run in CI):** `ReplayTransport` reading a recorded session,
asserting the expected number of scans with plausible point counts.

**Hardware tests (manual, documented in README):** the `examples/` are the hardware test suite.
Each example must print clear output that makes it obvious whether it worked.

Never gate CI on hardware. Never write a test that requires a device.

---

## Style and conventions

- Rust 2024 edition, MSRV 1.85 (state it in Cargo.toml, test it in CI)
- `#![warn(missing_docs)]` on the lib — every public item documented
- `clippy::pedantic` on, with targeted `allow`s where it's noise
- Doc comments include a runnable example where it makes sense
- No `unsafe`. If something seems to need it, it doesn't.
- Errors bubble with `?`; never `unwrap()` outside tests and examples
- Public API changes are semver-relevant — this crate is meant to be depended on

---

## What this crate is NOT

Resist scope creep hard. These belong in other Olivaw crates:

- ❌ SLAM, scan matching, occupancy grids → `olivaw-slam`
- ❌ Coordinate frame transforms → `olivaw-transforms`
- ❌ Motor control, robot base → `olivaw-base`
- ❌ Message passing / dataflow → use dora-rs
- ❌ A custom visualizer → use rerun

This crate reads a lidar and produces scans. That is the entire job.

---

## Definition of done for 0.1.0

- [ ] `cargo run --example info` prints real device info from a C1
- [ ] `cargo run --example basic_scan` streams valid scans continuously for 60s without desync
- [ ] `cargo run --example rerun_viz` shows a recognizable room outline in rerun
- [ ] All parser tests pass in CI with no hardware
- [ ] Cross-compiles to `aarch64-unknown-linux-gnu`
- [ ] README has a GIF of the rerun output and a 5-line quickstart
- [ ] Published to crates.io
