//! Integration tests: the full driver stack over replayed and scripted
//! byte streams, plus (when present) fixtures recorded from real hardware
//! by `examples/record.rs`.

use std::path::Path;
use std::time::Duration;

use olivaw_lidar::protocol::descriptor::{
    DATA_TYPE_DEVHEALTH, DATA_TYPE_DEVINFO, DATA_TYPE_MEASUREMENT, DESCRIPTOR_LEN, SendMode,
    parse_descriptor,
};
use olivaw_lidar::protocol::info::{DEVINFO_LEN, MODEL_C1, parse_device_info, parse_health};
use olivaw_lidar::transport::{MockTransport, ReplayTransport};
use olivaw_lidar::{Lidar, LidarConfig};

/// A config with no settle sleeps, so replay/mock tests run instantly.
fn instant_config() -> LidarConfig {
    LidarConfig {
        response_timeout: Duration::from_millis(10),
        stop_settle: Duration::ZERO,
        motor_settle: Duration::ZERO,
        reset_settle: Duration::ZERO,
        ..LidarConfig::default()
    }
}

/// Encodes a wire scan node from human units.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn node(start: bool, quality: u8, angle_deg: f32, distance_mm: f32) -> [u8; 5] {
    let angle_q6 = (angle_deg * 64.0) as u16;
    let distance_q2 = (distance_mm * 4.0) as u16;
    let s = u8::from(start);
    [
        (quality << 2) | ((1 - s) << 1) | s,
        (((angle_q6 & 0x7F) as u8) << 1) | 0x01,
        (angle_q6 >> 7) as u8,
        (distance_q2 & 0xFF) as u8,
        (distance_q2 >> 8) as u8,
    ]
}

/// The 7-byte standard-scan response descriptor.
const SCAN_DESCRIPTOR: [u8; 7] = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];

/// Builds a recorded-session byte stream: scan descriptor + `rotations`
/// full turns of `nodes_per_rotation` nodes each.
fn synthetic_session(rotations: usize, nodes_per_rotation: usize) -> Vec<u8> {
    let mut bytes = SCAN_DESCRIPTOR.to_vec();
    for _ in 0..rotations {
        for i in 0..nodes_per_rotation {
            #[allow(clippy::cast_precision_loss)]
            let angle = 360.0 * i as f32 / nodes_per_rotation as f32;
            bytes.extend_from_slice(&node(i == 0, 47, angle, 1000.0 + angle));
        }
    }
    bytes
}

#[test]
fn replayed_session_yields_complete_scans() {
    let stream = synthetic_session(4, 100);
    let mut lidar =
        Lidar::with_transport_and_config(ReplayTransport::from_bytes(stream), instant_config());
    lidar.start_scan().expect("descriptor should validate");

    let scans: Vec<_> = lidar
        .scans()
        .collect::<Result<_, _>>()
        .expect("clean stream should produce no errors");

    // 4 rotations: three complete scans are emitted at rotation boundaries;
    // the 4th has no closing start flag and is dropped at EOF.
    assert_eq!(scans.len(), 3);
    for scan in &scans {
        assert_eq!(scan.len(), 100);
        assert!(
            scan.angular_coverage() > 350.0,
            "coverage {}",
            scan.angular_coverage()
        );
        assert_eq!(scan.valid_points().count(), 100);
    }
}

/// Builds the 3-rotation stream of `corrupted_stream` tests, letting the
/// caller mutate the bytes of rotation 1's node 25.
fn three_rotations_with(corrupt: impl Fn(&mut Vec<u8>, [u8; 5])) -> Vec<u8> {
    let mut stream = SCAN_DESCRIPTOR.to_vec();
    for rotation in 0..3 {
        for i in 0u8..50 {
            let angle = 360.0 * f32::from(i) / 50.0;
            let wire = node(i == 0, 47, angle, 800.0);
            if rotation == 1 && i == 25 {
                corrupt(&mut stream, wire);
            } else {
                stream.extend_from_slice(&wire);
            }
        }
    }
    stream
}

fn replay_scans(stream: Vec<u8>) -> Vec<olivaw_lidar::Scan> {
    let mut lidar =
        Lidar::with_transport_and_config(ReplayTransport::from_bytes(stream), instant_config());
    lidar.start_scan().unwrap();
    lidar
        .scans()
        .collect::<Result<_, _>>()
        .expect("resync should recover, not error")
}

#[test]
fn garbage_between_nodes_recovers_every_node() {
    // 3 unparseable bytes injected between two nodes: the byte-at-a-time
    // resync discards exactly the garbage and loses nothing.
    let scans = replay_scans(three_rotations_with(|stream, wire| {
        stream.extend_from_slice(&[0x00, 0x00, 0x00]);
        stream.extend_from_slice(&wire);
    }));
    assert_eq!(scans.len(), 2);
    assert_eq!(scans[0].len(), 50, "garbage must cost zero real nodes");
    assert_eq!(scans[1].len(), 50);
}

#[test]
fn corrupted_node_is_dropped_and_stream_recovers() {
    // Node 25's header byte is destroyed in place (S == !S): that node is
    // unrecoverable, but exactly one node is lost and the rest realign.
    let scans = replay_scans(three_rotations_with(|stream, mut wire| {
        wire[0] = 0b0000_0011;
        stream.extend_from_slice(&wire);
    }));
    assert_eq!(scans.len(), 2);
    assert_eq!(
        scans[0].len(),
        50,
        "rotation before the corruption is intact"
    );
    assert_eq!(scans[1].len(), 49, "exactly the corrupted node is lost");
}

#[test]
fn wrong_descriptor_fails_start_scan() {
    // A GET_INFO descriptor where the scan descriptor belongs.
    let mut stream = vec![0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04];
    stream.extend_from_slice(&[0u8; 20]);
    let mut lidar =
        Lidar::with_transport_and_config(ReplayTransport::from_bytes(stream), instant_config());
    let err = lidar.start_scan().unwrap_err();
    assert!(
        matches!(err, olivaw_lidar::LidarError::Protocol(_)),
        "{err:?}"
    );
}

#[test]
fn mock_round_trip_info_and_health() {
    let mut mock = MockTransport::new();
    // GET_INFO response: descriptor + 20-byte payload (C1, firmware 1.25,
    // hardware rev 3, serial 00..0F).
    mock.queue_input(&[0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04]);
    let mut payload = vec![MODEL_C1, 0x19, 0x01, 0x03];
    payload.extend(0u8..16u8);
    mock.queue_input(&payload);
    // GET_HEALTH response: Good.
    mock.queue_input(&[0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06]);
    mock.queue_input(&[0x00, 0x00, 0x00]);

    let mut lidar = Lidar::with_transport_and_config(mock, instant_config());
    let info = lidar.info().unwrap();
    assert_eq!(info.model, MODEL_C1);
    assert_eq!(info.firmware_version, (1, 25));
    assert_eq!(info.hardware_version, 3);
    assert_eq!(info.serial_number_hex().to_string().len(), 32);
    assert!(lidar.health().unwrap().is_good());

    // The wire requests must be exactly GET_INFO then GET_HEALTH.
    assert_eq!(lidar.into_transport().written(), &[0xA5, 0x50, 0xA5, 0x52]);
}

#[test]
fn silent_device_reports_named_timeout() {
    let mut lidar = Lidar::with_transport_and_config(MockTransport::new(), instant_config());
    let err = lidar.info().unwrap_err();
    if let olivaw_lidar::LidarError::Timeout { what, ms } = &err {
        assert_eq!(*what, "device info descriptor");
        assert_eq!(*ms, 10);
    } else {
        panic!("expected a named timeout, got {err:?}");
    }
}

// ---------------------------------------------------------------------------
// Fixtures recorded from real hardware (run `cargo run --example record`).
// These tests validate against the actual C1 wire responses once captured;
// until then they succeed trivially so CI never depends on hardware.
// ---------------------------------------------------------------------------

fn fixture(name: &str) -> Option<Vec<u8>> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name);
    if let Ok(bytes) = std::fs::read(&path) {
        Some(bytes)
    } else {
        eprintln!("fixture {name} not recorded yet — skipping");
        None
    }
}

#[test]
fn real_info_fixture_parses() {
    let Some(bytes) = fixture("c1_info_response.bin") else {
        return;
    };
    assert_eq!(bytes.len(), DESCRIPTOR_LEN + DEVINFO_LEN);
    let descriptor = parse_descriptor(bytes[..DESCRIPTOR_LEN].try_into().unwrap()).unwrap();
    descriptor
        .expect(DATA_TYPE_DEVINFO, 20, SendMode::Single)
        .unwrap();
    let info = parse_device_info(bytes[DESCRIPTOR_LEN..].try_into().unwrap());
    assert_eq!(info.model, MODEL_C1, "recorded from a non-C1 device?");
    assert_ne!(info.serial_number, [0u8; 16]);
}

#[test]
fn real_health_fixture_parses() {
    let Some(bytes) = fixture("c1_health_response.bin") else {
        return;
    };
    let descriptor = parse_descriptor(bytes[..DESCRIPTOR_LEN].try_into().unwrap()).unwrap();
    descriptor
        .expect(DATA_TYPE_DEVHEALTH, 3, SendMode::Single)
        .unwrap();
    parse_health(bytes[DESCRIPTOR_LEN..].try_into().unwrap()).unwrap();
}

#[test]
fn real_scan_fixture_replays_into_scans() {
    let Some(bytes) = fixture("c1_scan_1000_nodes.bin") else {
        return;
    };
    let descriptor = parse_descriptor(bytes[..DESCRIPTOR_LEN].try_into().unwrap()).unwrap();
    descriptor
        .expect(DATA_TYPE_MEASUREMENT, 5, SendMode::Multi)
        .unwrap();

    let mut lidar =
        Lidar::with_transport_and_config(ReplayTransport::from_bytes(bytes), instant_config());
    lidar.start_scan().unwrap();
    let scans: Vec<_> = lidar
        .scans()
        .collect::<Result<_, _>>()
        .expect("recorded C1 stream should replay without errors");

    // ~1000 nodes at ~5 kHz over ~10 Hz rotations → at least one full scan.
    assert!(!scans.is_empty(), "no complete rotation in the recording");
    for scan in &scans {
        assert!(
            scan.len() > 50,
            "implausibly sparse rotation: {}",
            scan.len()
        );
        assert!(scan.angular_coverage() > 300.0);
    }
}
