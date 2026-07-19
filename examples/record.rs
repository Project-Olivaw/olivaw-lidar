//! Record raw wire bytes from a live device into `tests/fixtures/`.
//!
//! Run this once against real hardware and the parser test suite has
//! authentic data forever:
//!
//! ```sh
//! cargo run --example record                 # auto-detect, 1000 nodes
//! cargo run --example record -- --port /dev/cu.usbserial-1130 --nodes 2000
//! ```
//!
//! This example deliberately bypasses the `Lidar` API and drives the
//! transport with `protocol` directly: its job is verbatim capture of every
//! byte the device sends, descriptors included, which the request/response
//! methods would consume.

use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use anyhow::{Context as _, bail};
use clap::Parser;
use olivaw_lidar::protocol::descriptor::{DESCRIPTOR_LEN, parse_descriptor};
use olivaw_lidar::protocol::{Command, MAX_REQUEST_LEN};
use olivaw_lidar::transport::{
    SerialTransport, Transport, auto_detect_port, prefer_callout_device,
};

const BAUD_C1: u32 = 460_800;
const NODE_LEN: usize = 5;
const RESPONSE_TIMEOUT: Duration = Duration::from_secs(1);
const SETTLE: Duration = Duration::from_millis(50);

/// Record raw RPLIDAR responses to fixture files.
#[derive(Parser)]
struct Args {
    /// Serial port path. Auto-detected if omitted.
    #[arg(long)]
    port: Option<String>,

    /// Output directory for the .bin fixtures.
    #[arg(long, default_value = "tests/fixtures")]
    out: PathBuf,

    /// Number of 5-byte scan nodes to capture.
    #[arg(long, default_value_t = 1000)]
    nodes: usize,

    /// Give up on the scan capture after this many seconds.
    #[arg(long, default_value_t = 30)]
    seconds: u64,

    /// Motor PWM duty sent before scanning (the C1 also self-manages).
    #[arg(long, default_value_t = 660)]
    pwm: u16,
}

fn send(transport: &mut SerialTransport, command: Command) -> anyhow::Result<()> {
    let mut buf = [0u8; MAX_REQUEST_LEN];
    let len = command.encode(&mut buf);
    transport
        .write_all(&buf[..len])
        .with_context(|| format!("failed to send {command:?}"))?;
    Ok(())
}

/// Captures descriptor + fixed-size payload of a single-response command.
fn capture_single(
    transport: &mut SerialTransport,
    command: Command,
    payload_len: usize,
) -> anyhow::Result<Vec<u8>> {
    transport.discard_input().context("input flush failed")?;
    send(transport, command)?;
    let mut bytes = vec![0u8; DESCRIPTOR_LEN + payload_len];
    transport
        .read_exact(&mut bytes, RESPONSE_TIMEOUT)
        .with_context(|| format!("no response to {command:?} — wrong port or baud rate?"))?;

    // Sanity-check the capture so a bad recording is caught immediately.
    let descriptor_bytes: &[u8; DESCRIPTOR_LEN] = bytes[..DESCRIPTOR_LEN].try_into()?;
    let descriptor = parse_descriptor(descriptor_bytes)
        .with_context(|| format!("{command:?} response descriptor is malformed"))?;
    println!(
        "  {command:?}: descriptor announces {} byte(s), data type {:#04x}",
        descriptor.len, descriptor.data_type
    );
    Ok(bytes)
}

fn write_fixture(dir: &Path, name: &str, bytes: &[u8]) -> anyhow::Result<PathBuf> {
    let path = dir.join(name);
    fs::write(&path, bytes).with_context(|| format!("failed to write {}", path.display()))?;
    println!("  wrote {} ({} bytes)", path.display(), bytes.len());
    Ok(path)
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let port = match args.port {
        Some(explicit) => prefer_callout_device(&explicit),
        None => auto_detect_port().context(
            "no serial port found that looks like a lidar.\n\
             Is the device plugged in? Pass one explicitly with --port <PATH>.",
        )?,
    };
    println!("recording from {port} at {BAUD_C1} baud");

    fs::create_dir_all(&args.out)
        .with_context(|| format!("failed to create {}", args.out.display()))?;

    let mut transport =
        SerialTransport::open(&port, BAUD_C1).with_context(|| format!("failed to open {port}"))?;

    // Recovery handshake: a crashed prior run leaves the unit streaming.
    send(&mut transport, Command::Stop)?;
    std::thread::sleep(SETTLE);
    transport.discard_input().context("input flush failed")?;

    println!("capturing single-response fixtures:");
    let info = capture_single(&mut transport, Command::GetInfo, 20)?;
    write_fixture(&args.out, "c1_info_response.bin", &info)?;
    let health = capture_single(&mut transport, Command::GetHealth, 3)?;
    write_fixture(&args.out, "c1_health_response.bin", &health)?;

    println!("starting scan (motor pwm {}):", args.pwm);
    transport.discard_input().context("input flush failed")?;
    send(&mut transport, Command::SetMotorPwm(args.pwm))?;
    std::thread::sleep(SETTLE);
    send(&mut transport, Command::Scan)?;

    let target = DESCRIPTOR_LEN + args.nodes * NODE_LEN;
    let deadline = Instant::now() + Duration::from_secs(args.seconds);
    let mut stream = Vec::with_capacity(target);
    let mut chunk = [0u8; 4096];
    while stream.len() < target {
        if Instant::now() >= deadline {
            break;
        }
        match transport.read(&mut chunk, Duration::from_millis(500)) {
            Ok(n) => stream.extend_from_slice(&chunk[..n]),
            Err(e) => {
                println!("  read stalled ({e}); continuing until deadline");
            }
        }
        // Lightweight progress so a hung capture is visible.
        print!("\r  captured {} / {target} bytes", stream.len());
        std::io::stdout().flush().ok();
    }
    println!();

    // Always leave the device idle, even after a partial capture.
    send(&mut transport, Command::Stop)?;
    std::thread::sleep(SETTLE);
    send(&mut transport, Command::SetMotorPwm(0))?;
    transport.discard_input().ok();

    if stream.len() < DESCRIPTOR_LEN {
        bail!(
            "scan capture got only {} byte(s) — the device never started streaming",
            stream.len()
        );
    }
    let descriptor_bytes: &[u8; DESCRIPTOR_LEN] = stream[..DESCRIPTOR_LEN].try_into()?;
    let descriptor = parse_descriptor(descriptor_bytes)
        .context("scan response descriptor is malformed — capture desynchronized?")?;
    println!(
        "  scan descriptor announces {}-byte nodes, data type {:#04x}",
        descriptor.len, descriptor.data_type
    );

    let nodes_captured = (stream.len() - DESCRIPTOR_LEN) / NODE_LEN;
    write_fixture(&args.out, "c1_scan_1000_nodes.bin", &stream)?;
    println!();
    if nodes_captured >= args.nodes {
        println!("OK — captured {nodes_captured} nodes. Fixtures are ready to commit.");
    } else {
        println!(
            "PARTIAL — captured only {nodes_captured}/{} nodes before the deadline.",
            args.nodes
        );
    }
    Ok(())
}
