//! Live 2D point cloud in the [rerun](https://rerun.io) viewer.
//!
//! ```sh
//! cargo run --example rerun_viz                    # spawn the viewer
//! cargo run --example rerun_viz -- --save out.rrd  # record to a file instead
//! ```
//!
//! Spawning requires the `rerun` viewer binary (`cargo install rerun-cli`
//! or `pip install rerun-sdk`); `--save` needs nothing extra and the file
//! can be opened later with `rerun out.rrd`.

use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Context as _, bail};
use clap::Parser;
use olivaw_lidar::Lidar;
use olivaw_lidar::transport::{auto_detect_port, prefer_callout_device};

/// Visualize live lidar scans as a 2D point cloud in rerun.
#[derive(Parser)]
struct Args {
    /// Serial port path. Auto-detected if omitted.
    #[arg(long)]
    port: Option<String>,

    /// How long to stream before stopping.
    #[arg(long, default_value_t = 60)]
    seconds: u64,

    /// Record to a .rrd file instead of spawning the viewer.
    #[arg(long)]
    save: Option<PathBuf>,
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

    let builder = rerun::RecordingStreamBuilder::new("olivaw_lidar");
    let rec = match &args.save {
        Some(path) => builder
            .save(path)
            .with_context(|| format!("failed to open {} for recording", path.display()))?,
        None => builder
            .spawn()
            .context("failed to spawn the rerun viewer — is it installed? (cargo install rerun-cli). Alternatively use --save out.rrd")?,
    };

    let mut lidar =
        Lidar::open(&port).with_context(|| format!("failed to open lidar on {port}"))?;
    let info = lidar.info().context("GET_INFO failed")?;
    println!(
        "connected to {} on {port}",
        info.model_name().unwrap_or("unknown RPLIDAR model")
    );
    let health = lidar.health().context("GET_HEALTH failed")?;
    if !health.is_good() {
        bail!("device is not healthy ({health:?}); power-cycle it and retry");
    }

    lidar.start_scan().context("failed to start scanning")?;
    println!("streaming for {} second(s)…", args.seconds);

    let started = Instant::now();
    for scan in lidar.scans() {
        let scan = scan.context("scan stream broke")?;

        let (positions, colors): (Vec<(f32, f32)>, Vec<rerun::Color>) = scan
            .valid_points()
            .map(|p| {
                // Quality 0..=63 → dim red (weak) to bright green (strong).
                let q = f32::from(p.quality) / 63.0;
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let color =
                    rerun::Color::from_rgb((200.0 * (1.0 - q)) as u8, (220.0 * q + 35.0) as u8, 40);
                (p.to_cartesian(), color)
            })
            .unzip();

        rec.log(
            "lidar/points",
            &rerun::Points2D::new(positions)
                .with_colors(colors)
                .with_radii([0.02]),
        )
        .context("failed to log to rerun")?;

        if started.elapsed().as_secs() >= args.seconds {
            break;
        }
    }

    lidar.stop().context("failed to stop the scan")?;
    if let Some(path) = &args.save {
        println!("OK — recording saved to {}", path.display());
    } else {
        println!("OK — done streaming.");
    }
    Ok(())
}
