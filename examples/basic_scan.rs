//! Second hardware milestone: stream live scans to stdout.
//!
//! ```sh
//! cargo run --example basic_scan                 # auto-detect, 60 seconds
//! cargo run --example basic_scan -- --seconds 10
//! ```

use std::time::Instant;

use anyhow::{Context as _, bail};
use clap::Parser;
use olivaw_lidar::Lidar;
use olivaw_lidar::transport::{auto_detect_port, prefer_callout_device};

/// Stream live lidar scans to stdout.
#[derive(Parser)]
struct Args {
    /// Serial port path. Auto-detected if omitted.
    #[arg(long)]
    port: Option<String>,

    /// How long to stream before stopping.
    #[arg(long, default_value_t = 60)]
    seconds: u64,
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

    let mut lidar =
        Lidar::open(&port).with_context(|| format!("failed to open lidar on {port}"))?;

    let info = lidar.info().context("GET_INFO failed")?;
    println!(
        "connected to {} (serial {}) on {port}",
        info.model_name().unwrap_or("unknown RPLIDAR model"),
        info.serial_number_hex()
    );

    let health = lidar.health().context("GET_HEALTH failed")?;
    if !health.is_good() {
        bail!("device is not healthy ({health:?}); power-cycle it and retry");
    }

    println!("starting scan for {} second(s)…", args.seconds);
    lidar.start_scan().context("failed to start scanning")?;

    let started = Instant::now();
    let mut previous_scan_at: Option<Instant> = None;
    let mut total_scans = 0u64;
    for (i, scan) in lidar.scans().enumerate() {
        let scan = scan.context("scan stream broke")?;
        let hz = previous_scan_at.map_or(0.0, |prev| {
            1.0 / scan.timestamp.duration_since(prev).as_secs_f64().max(1e-9)
        });
        previous_scan_at = Some(scan.timestamp);
        total_scans += 1;

        println!(
            "scan {i:4}: {:4} points ({:4} valid), {:6.1}° coverage, {hz:5.1} Hz",
            scan.len(),
            scan.valid_points().count(),
            scan.angular_coverage(),
        );

        if started.elapsed().as_secs() >= args.seconds {
            break;
        }
    }

    lidar.stop().context("failed to stop the scan")?;
    println!(
        "OK — {total_scans} scans in {:.1}s with no desync.",
        started.elapsed().as_secs_f64()
    );
    Ok(())
}
