//! First hardware milestone: print device info and health, then exit.
//!
//! ```sh
//! cargo run --example info                       # auto-detect the port
//! cargo run --example info -- --port /dev/cu.usbserial-1130
//! ```

use anyhow::{Context as _, bail};
use clap::Parser;
use olivaw_lidar::Lidar;
use olivaw_lidar::transport::{auto_detect_port, prefer_callout_device};

/// Print RPLIDAR device info and health status.
#[derive(Parser)]
struct Args {
    /// Serial port path (e.g. /dev/cu.usbserial-1130 on macOS,
    /// /dev/ttyUSB0 on Linux, COM3 on Windows). Auto-detected if omitted.
    #[arg(long)]
    port: Option<String>,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let port = match args.port {
        Some(explicit) => {
            let preferred = prefer_callout_device(&explicit);
            if preferred != explicit {
                println!(
                    "note: using {preferred} instead of {explicit} (the tty. variant can block on open)"
                );
            }
            preferred
        }
        None => match auto_detect_port() {
            Some(detected) => {
                println!("auto-detected port: {detected}");
                detected
            }
            None => bail!(
                "no serial port found that looks like a lidar.\n\
                 Is the device plugged in? Pass one explicitly with --port <PATH>."
            ),
        },
    };

    let mut lidar =
        Lidar::open(&port).with_context(|| format!("failed to open lidar on {port}"))?;

    let info = lidar.info().context("GET_INFO failed")?;
    let health = lidar.health().context("GET_HEALTH failed")?;

    let model = info.model_name().map_or_else(
        || format!("unknown model ({:#04x})", info.model),
        String::from,
    );
    let (fw_major, fw_minor) = info.firmware_version;

    println!();
    println!("  Port:      {port}");
    println!("  Model:     {model}");
    println!("  Firmware:  {fw_major}.{fw_minor:02}");
    println!("  Hardware:  rev {}", info.hardware_version);
    println!("  Serial:    {}", info.serial_number_hex());
    println!("  Health:    {health:?}");
    println!();

    if health.is_good() {
        println!("OK — device answered and reports healthy.");
    } else {
        println!("Device answered but is not healthy: {health:?}");
        println!("A power cycle usually clears a protection state.");
    }
    Ok(())
}
