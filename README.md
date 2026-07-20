# olivaw-lidar

Pure-Rust driver for SLAMTEC RPLIDAR laser range scanners. No C++ SDK, no FFI, no `bindgen`.

Primary target: **RPLIDAR C1**. Designed to extend to the A/S series. Part of
[Project Olivaw](https://github.com/Project-Olivaw) — robotics in Rust.

![Rerun visualization of olivaw-lidar](.github/assets/viz.gif)

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

| Example                          | What it proves                                               |
| -------------------------------- | ------------------------------------------------------------ |
| `cargo run --example info`       | Port + protocol work: prints model, firmware, serial, health |
| `cargo run --example record`     | Captures real wire bytes into `tests/fixtures/`              |
| `cargo run --example basic_scan` | Streams assembled 360° scans to stdout                       |
| `cargo run --example rerun_viz`  | Live 2D point cloud in [rerun](https://rerun.io)             |

All examples auto-detect the serial port (CP210x/CH340 USB bridges) on macOS,
Linux and Windows, and accept `--port <PATH>` to override. On macOS the
`/dev/cu.*` device is preferred over `/dev/tty.*`, which can block on open.

### Visualization

`rerun_viz` has two modes:

```sh
cargo run --example rerun_viz                     # opens the rerun viewer live
cargo run --example rerun_viz -- --save room.rrd  # headless: no window, writes a file
```

The live mode needs the [rerun viewer](https://rerun.io/docs/getting-started/installing-viewer)
binary on your `PATH` (the `--save` mode needs nothing). Install it with any of:

```sh
uv tool install rerun-sdk         # fastest: prebuilt binary
pip3 install rerun-sdk            # same, via pip
cargo install rerun-cli --locked  # builds from source (slow)
```

Keep the viewer's version aligned with this crate's `rerun` dev-dependency
(currently **0.34**) or the viewer will warn about a version mismatch — e.g.
`uv tool install rerun-sdk==0.34.1`. A saved `.rrd` opens anytime with
`rerun room.rrd`.

## Status

**The RPLIDAR C1 is fully supported.** Everything the C1 offers over its serial
protocol is implemented and verified against real hardware:

- [x] Info / health / stop / reset / motor PWM — verified on a real C1
- [x] Standard scan (0x20): node parsing, scan assembly, byte-level desync recovery
      (60 s live soak: 583 scans at ~10 Hz, ~512 points each, zero desync)
- [x] Replay + mock transports, fixture-driven test suite
- [x] Fixtures recorded from real hardware (`tests/fixtures/c1_*.bin`)
- [x] `no_std` parser core, cross-checked for `aarch64-unknown-linux-gnu` (Raspberry Pi)

## Future work

None of this is needed to use a C1 — it is the roadmap for extending the driver
to the rest of the RPLIDAR family. The ground rule for all of it: **record real
wire fixtures from the target device first** (`examples/record.rs`), then build
the parser against those fixtures offline, exactly as the C1 support was built.

### Express scan (0x82) — needed for faster models, not the C1

The standard scan spends 5 bytes per measurement. Express mode packs 32
measurements into an 84-byte checksummed "capsule" with delta-angle encoding —
about half the bytes per sample. The C1's ~5 kHz sample rate fits comfortably
through standard scan at 460800 baud (measured: ~512 points × ~10 Hz), so
express buys nothing there. It becomes mandatory for A3/S-series units whose
16–32 kHz rates physically cannot fit through the 5-byte format. Implementing
it means: capsule parser in `protocol/express.rs` (stateful across capsules,
checksum-validated → the dormant `ProtocolError::Checksum` variant),
`EXPRESS_SCAN` request framing with payload, and fixtures from a device that
actually streams it.

### Additional models (A1 / A2 / A3 / S1 / S2)

The architecture is ready (trait-based transport, `LidarConfig` overrides);
each model needs a small, specific slice of work:

| Model | Baud | What it needs beyond the C1 path |
|---|---|---|
| A1 | 115200 | DTR-pin motor control (not the 0xF0 PWM command — the hook point is marked in `transport/serial.rs`) |
| A2 | 256000 | DTR motor control; express scan recommended |
| A3 | 256000 | `GET_LIDAR_CONF` (0x84) scan-mode discovery; express/dense scan for rated sample rates |
| S1/S2 | 256000 | Same as A3; TOF-specific ranges |

Plus, for every model: its `model_name()` byte mapping and a `LidarConfig`
preset, both confirmed against real hardware.

## License

MIT. See [LICENSE](LICENSE).
