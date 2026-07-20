# 01 — Project overview

## What this is

`olivaw-lidar` is a driver for SLAMTEC RPLIDAR laser range scanners written
entirely in Rust. It speaks the RPLIDAR serial protocol directly over a USB
serial port: no SLAMTEC C++ SDK, no FFI, no `bindgen`, no C or C++ anywhere in
the build. The first supported device is the RPLIDAR C1; the architecture is
deliberately shaped to extend to the A1/A2/A3/S1/S2 families later.

It is part of Project Olivaw, a set of robotics tools in Rust. A future
`olivaw-slam` crate will consume this driver's `Scan` values.

## Why pure Rust (do not compromise on this)

The constraint sounds ideological but is practical:

1. **Cross-compilation is trivial.** `cargo check --target aarch64-unknown-linux-gnu`
   passes with zero extra setup. Deploying to a Raspberry Pi or Jetson never
   involves a C++ cross-toolchain. Even `serialport` is used with
   `default-features = false` so Linux builds do not link libudev.
2. **No unsafe FFI boundary.** The crate forbids `unsafe` entirely
   (`unsafe_code = "forbid"` in Cargo.toml). A driver that parses untrusted
   bytes from a wire is exactly where memory safety pays for itself.
3. **The parser can go anywhere Rust goes.** The `protocol/` module is
   `no_std`-compatible (disable the default `std` feature), so the same
   parsing code can eventually run on an ESP32.

## Goals

- An API that feels obvious to someone who has never used a lidar:
  `Lidar::open`, `info()`, `health()`, `start_scan()`, `scans()`.
- Errors that tell the user what to do, not just what happened
  (`thiserror`, specific variants, named timeouts).
- A test suite that never requires hardware, driven by bytes recorded from
  real hardware.
- OS-agnostic library code. Examples may lean on macOS conveniences (that is
  where development happens), but the library itself runs anywhere Rust and
  `serialport` run: macOS, Linux, Windows.

## What this crate is NOT (scope boundaries)

Resist scope creep. These belong in other Olivaw crates or other tools:

| Not here | Where it belongs |
|---|---|
| SLAM, scan matching, occupancy grids | `olivaw-slam` (future) |
| Coordinate frame transforms | `olivaw-transforms` (future) |
| Motor control, robot base | `olivaw-base` (future) |
| Message passing / dataflow | dora-rs |
| A custom visualizer | rerun (used in an example only) |

This crate reads a lidar and produces scans. That is the entire job.

## Authoritative documents

- `CLAUDE.md` at the repo root is the original specification: architecture,
  protocol details, API shape, implementation order, and scope boundaries.
  When documentation and spec disagree, the code and this folder record what
  was actually built and why any deviation happened (see
  [04-development-process.md](04-development-process.md)).
- The SLAMTEC public protocol specification and the reference
  [rplidar_sdk](https://github.com/slamtec/rplidar_sdk) were used as wire
  format references only. Nothing was vendored or translated line by line.
