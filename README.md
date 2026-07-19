# olivaw-lidar

Pure-Rust driver for SLAMTEC RPLIDAR laser range scanners. No C++ SDK, no FFI, no `bindgen`.

Primary target: **RPLIDAR C1**. Designed to extend to the A/S series. Part of
[Project Olivaw](https://github.com/Project-Olivaw) — robotics in Rust.

<!-- TODO: GIF of examples/rerun_viz.rs here once recorded on hardware -->

## Quickstart

```rust
use olivaw_lidar::Lidar;

let mut lidar = Lidar::open("/dev/cu.usbserial-XXXX")?; // /dev/ttyUSB0 on Linux
println!("{:?}", lidar.info()?);
lidar.start_scan()?;
for scan in lidar.scans() {
    println!("{} points", scan?.len());
}
```

## Why pure Rust

- Cross-compiles cleanly to `aarch64-unknown-linux-gnu` (Raspberry Pi) and
  `aarch64-apple-darwin` with zero extra setup — no C++ toolchain, ever.
- No `unsafe` anywhere (`unsafe_code = "forbid"`).
- The `protocol` parsing core is `no_std`-compatible: disable the default `std`
  feature and take just the parsers (e.g. for an ESP32).

```toml
[dependencies]
olivaw-lidar = "0.1"                                          # full driver
olivaw-lidar = { version = "0.1", default-features = false }  # no_std parsers only
```

## Architecture

```
protocol/   PURE: bytes in, typed values out. no_std, no I/O, tested on fixtures.
transport/  Trait-based I/O: SerialTransport (hardware), ReplayTransport
            (recorded sessions), MockTransport (tests).
device/     High-level Lidar API: request/response, scan assembly, desync recovery.
```

Because parsing never touches I/O, the full test suite — including desync
recovery against corrupted streams — runs in CI with no hardware attached, and
recorded sessions replay through the exact same code path as a live device.

## Examples (the hardware test suite)

| Example | What it proves |
|---|---|
| `cargo run --example info` | Port + protocol work: prints model, firmware, serial, health |
| `cargo run --example record` | Captures real wire bytes into `tests/fixtures/` |
| `cargo run --example basic_scan` | Streams assembled 360° scans to stdout |
| `cargo run --example rerun_viz` | Live 2D point cloud in [rerun](https://rerun.io) |

All examples auto-detect the serial port (CP210x/CH340 USB bridges) on macOS,
Linux and Windows, and accept `--port <PATH>` to override. On macOS the
`/dev/cu.*` device is preferred over `/dev/tty.*`, which can block on open.

## Status

- [x] Info / health / stop / reset / motor PWM — verified on a real C1
- [x] Standard scan (0x20): node parsing, scan assembly, byte-level desync recovery
      (60 s live soak: 583 scans at ~10 Hz, ~512 points each, zero desync)
- [x] Replay + mock transports, fixture-driven test suite
- [x] Fixtures recorded from real hardware (`tests/fixtures/c1_*.bin`)
- [ ] Express scan (0x82)
- [ ] Additional models (A1/A2/A3/S1/S2)

## License

MIT. See [LICENSE](LICENSE).
