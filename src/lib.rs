//! Pure-Rust driver for SLAMTEC RPLIDAR laser range scanners.
//!
//! `olivaw-lidar` speaks the RPLIDAR serial protocol directly — no C++ SDK,
//! no FFI, no `bindgen`. The primary target is the **RPLIDAR C1**, with the
//! architecture designed to extend to the A/S series.
//!
//! The crate is split into three layers:
//!
//! - [`protocol`] — pure, `no_std`-compatible parsing and encoding: bytes in,
//!   typed values out. No I/O anywhere in this module.
//! - `transport` — trait-based byte I/O. `SerialTransport` talks to real
//!   hardware; replay and mock transports implement the same trait for
//!   offline testing.
//! - `device` — the high-level `Lidar` API that owns a transport and drives
//!   the request/response protocol.
//!
//! # `no_std`
//!
//! Disable the default `std` feature to compile only the [`protocol`] parsing
//! core, e.g. for use on a microcontroller:
//!
//! ```toml
//! olivaw-lidar = { version = "0.1", default-features = false }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![warn(missing_docs)]

pub mod protocol;

#[cfg(feature = "std")]
mod error;

#[cfg(feature = "std")]
pub use error::LidarError;
