# olivaw-lidar documentation

This folder is the long-term memory of the project. It exists so that anyone —
including the original authors returning after months away — can understand
what was built, why it was built that way, how every part works, and where the
known gaps and improvement opportunities are.

## Reading order

| Document | What it answers |
|---|---|
| [01-project-overview.md](01-project-overview.md) | What is this? Why pure Rust? What is in and out of scope? |
| [02-architecture.md](02-architecture.md) | How is the code organized? Why three layers? Which file does what? |
| [03-protocol.md](03-protocol.md) | The RPLIDAR wire protocol, byte by byte, as verified on real hardware |
| [04-development-process.md](04-development-process.md) | The step-by-step build order, the fixture-first methodology, and the bugs reality caught |
| [05-testing-strategy.md](05-testing-strategy.md) | How the driver is tested without hardware, and what each test tier proves |
| [06-hardware-notes.md](06-hardware-notes.md) | Field facts about the real C1: timing, ports, quirks, measured numbers |
| [07-future-work.md](07-future-work.md) | Express scan, other models, and honest gaps vs. other drivers |

## The two ideas that shaped everything

If you read nothing else, read these:

1. **Parsing is separated from I/O, absolutely.** The `protocol/` module is
   pure: bytes in, typed values out, `no_std`, no serial port anywhere. This
   is why the full test suite — including desynchronization recovery — runs in
   CI with no lidar attached, and why recorded sessions replay through the
   exact same code path as live hardware.

2. **Record real bytes before writing parsers.** `examples/record.rs` captured
   authentic wire responses from a real C1 into `tests/fixtures/` before the
   scan parser was written. Parsers are developed offline against truth, not
   against a datasheet's idea of truth. Every future protocol feature (express
   scan, new models) must follow the same rule.

## Quick facts

- Crate: `olivaw-lidar`, Rust 2024 edition, MSRV 1.85
- Primary hardware: SLAMTEC RPLIDAR C1 (460800 baud, 8N1, USB CP210x bridge)
- Zero `unsafe` (`unsafe_code = "forbid"`), zero clippy pedantic warnings
- Dependencies (library): `serialport` (without libudev, pure-Rust build) and `thiserror` — nothing else
- Verified live: 583 consecutive scans in 60 s, about 10 Hz rotation,
  about 512 nodes per rotation (about 5.1 kHz sample rate), zero desyncs
