//! Core value types produced by the driver.
//!
//! [`DeviceInfo`] and [`HealthStatus`] are defined in the `no_std`-capable
//! [`crate::protocol`] module (they are what the protocol parsers produce)
//! and re-exported here. [`Point`] and [`Scan`] require `std`.

pub use crate::protocol::info::{DeviceInfo, HealthStatus, SerialNumberHex};

#[cfg(feature = "std")]
pub use with_std::{Point, Scan};

#[cfg(feature = "std")]
mod with_std {
    use std::time::Instant;

    use crate::protocol::scan_node::ScanNode;

    /// A single lidar measurement.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct Point {
        /// Angle in degrees, `0.0..360.0`, increasing clockwise seen from
        /// the top of the unit. `0°` is the device's forward direction.
        pub angle_deg: f32,
        /// Distance in millimeters. `0.0` means no return (invalid).
        pub distance_mm: f32,
        /// Reflected-signal quality, `0..=63`. Higher is better.
        pub quality: u8,
    }

    impl Point {
        /// `true` when the measurement carries a real distance.
        #[must_use]
        pub fn is_valid(&self) -> bool {
            self.distance_mm > 0.0
        }

        /// Converts to Cartesian coordinates in **meters**, in a
        /// right-handed, x-forward, y-left frame (the common robotics
        /// convention): a point dead ahead is `(d, 0)`, a point 90° to the
        /// left is `(0, d)`.
        #[must_use]
        pub fn to_cartesian(&self) -> (f32, f32) {
            let r = self.distance_mm / 1000.0;
            // Lidar angles increase clockwise; y-left needs the negation.
            let theta = self.angle_deg.to_radians();
            (r * theta.cos(), -r * theta.sin())
        }
    }

    impl From<&ScanNode> for Point {
        fn from(node: &ScanNode) -> Self {
            Self {
                angle_deg: node.angle_deg(),
                distance_mm: node.distance_mm(),
                quality: node.quality,
            }
        }
    }

    /// One full 360° rotation of measurements, assembled from the points
    /// between two start flags.
    ///
    /// Invalid (no-return) points are kept, so the raw data is complete;
    /// [`Scan::valid_points`] filters them out.
    #[derive(Debug, Clone)]
    pub struct Scan {
        points: Vec<Point>,
        /// When the rotation completed.
        pub timestamp: Instant,
    }

    impl Scan {
        /// Assembles a scan from points captured over one rotation.
        #[must_use]
        pub fn new(points: Vec<Point>, timestamp: Instant) -> Self {
            Self { points, timestamp }
        }

        /// Number of measurements in the rotation, invalid ones included.
        #[must_use]
        pub fn len(&self) -> usize {
            self.points.len()
        }

        /// `true` when the scan holds no measurements.
        #[must_use]
        pub fn is_empty(&self) -> bool {
            self.points.is_empty()
        }

        /// All measurements, in capture order.
        #[must_use]
        pub fn points(&self) -> &[Point] {
            &self.points
        }

        /// Measurements that carry a real distance.
        pub fn valid_points(&self) -> impl Iterator<Item = &Point> {
            self.points.iter().filter(|p| p.is_valid())
        }

        /// Degrees of the rotation actually covered by measurements,
        /// `0.0..=360.0` — the sum of the angular steps between consecutive
        /// points. A healthy full rotation approaches `360.0`.
        #[must_use]
        pub fn angular_coverage(&self) -> f32 {
            let mut total = 0.0_f32;
            for pair in self.points.windows(2) {
                let mut step = pair[1].angle_deg - pair[0].angle_deg;
                if step < 0.0 {
                    step += 360.0;
                }
                total += step;
            }
            total.min(360.0)
        }

        /// Valid points as Cartesian `(x, y)` meters (see
        /// [`Point::to_cartesian`]).
        #[must_use]
        pub fn to_cartesian(&self) -> Vec<(f32, f32)> {
            self.valid_points().map(Point::to_cartesian).collect()
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        fn point(angle_deg: f32, distance_mm: f32) -> Point {
            Point {
                angle_deg,
                distance_mm,
                quality: 47,
            }
        }

        #[test]
        fn cartesian_convention_x_forward_y_left() {
            let ahead = point(0.0, 1000.0).to_cartesian();
            assert!((ahead.0 - 1.0).abs() < 1e-6 && ahead.1.abs() < 1e-6);

            // 90° clockwise (device right) → negative y in a y-left frame.
            let right = point(90.0, 2000.0).to_cartesian();
            assert!(right.0.abs() < 1e-6 && (right.1 + 2.0).abs() < 1e-6);

            let behind = point(180.0, 500.0).to_cartesian();
            assert!((behind.0 + 0.5).abs() < 1e-6 && behind.1.abs() < 1e-6);
        }

        #[test]
        fn invalid_points_are_kept_but_filterable() {
            let scan = Scan::new(
                vec![point(0.0, 100.0), point(1.0, 0.0), point(2.0, 200.0)],
                Instant::now(),
            );
            assert_eq!(scan.len(), 3);
            assert_eq!(scan.valid_points().count(), 2);
            assert_eq!(scan.to_cartesian().len(), 2);
        }

        #[test]
        fn angular_coverage_of_a_full_rotation() {
            let points: Vec<Point> = (0..360)
                .map(|d| {
                    #[allow(clippy::cast_precision_loss)]
                    point(d as f32, 1000.0)
                })
                .collect();
            let scan = Scan::new(points, Instant::now());
            let coverage = scan.angular_coverage();
            assert!((coverage - 359.0).abs() < 1e-3, "{coverage}");
        }

        #[test]
        fn angular_coverage_handles_wraparound() {
            // A rotation that starts at 350° and wraps through 0°.
            let scan = Scan::new(
                vec![
                    point(350.0, 1.0),
                    point(355.0, 1.0),
                    point(5.0, 1.0),
                    point(10.0, 1.0),
                ],
                Instant::now(),
            );
            assert!((scan.angular_coverage() - 20.0).abs() < 1e-3);
        }

        #[test]
        fn empty_scan_is_well_behaved() {
            let scan = Scan::new(vec![], Instant::now());
            assert!(scan.is_empty());
            assert_eq!(scan.len(), 0);
            assert!((scan.angular_coverage() - 0.0).abs() < f32::EPSILON);
        }
    }
}
