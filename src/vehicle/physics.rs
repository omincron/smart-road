pub const VEHICLE_LENGTH: f32 = 32.0;
pub const SAFETY_DISTANCE: f32 = 40.0; // minimum front-to-back gap between vehicles
pub const MIN_FOLLOWING_GAP: f32 = SAFETY_DISTANCE + VEHICLE_LENGTH; // centre-to-centre
pub const SMOOTH_ALPHA: f32 = 0.12; // exponential smoothing factor for vehicle speed

pub fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    let dx = x1 - x2;
    let dy = y1 - y2;
    (dx * dx + dy * dy).sqrt()
}

/// Returns true when two vehicles are physically nearly touching (centres < SAFETY_DISTANCE).
pub fn is_close_call(x1: f32, y1: f32, x2: f32, y2: f32) -> bool {
    distance(x1, y1, x2, y2) < SAFETY_DISTANCE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_axis_aligned() {
        assert!((distance(0.0, 0.0, 3.0, 4.0) - 5.0).abs() < 1e-5);
        assert!((distance(0.0, 0.0, 0.0, 0.0) - 0.0).abs() < 1e-5);
    }

    #[test]
    fn distance_is_symmetric() {
        let d1 = distance(1.0, 2.0, 5.0, 8.0);
        let d2 = distance(5.0, 8.0, 1.0, 2.0);
        assert!((d1 - d2).abs() < 1e-5);
    }

    #[test]
    fn close_call_true_when_within_safety_distance() {
        // Two vehicles at the same point are definitely a close call.
        assert!(is_close_call(0.0, 0.0, 0.0, 0.0));
        // One pixel apart — well within SAFETY_DISTANCE (40.0).
        assert!(is_close_call(0.0, 0.0, 1.0, 0.0));
    }

    #[test]
    fn close_call_false_when_beyond_safety_distance() {
        // SAFETY_DISTANCE + margin apart — not a close call.
        assert!(!is_close_call(0.0, 0.0, SAFETY_DISTANCE + 1.0, 0.0));
        assert!(!is_close_call(0.0, 0.0, 0.0, SAFETY_DISTANCE + 1.0));
    }

    #[test]
    fn min_following_gap_exceeds_safety_distance() {
        const { assert!(MIN_FOLLOWING_GAP > SAFETY_DISTANCE) };
    }
}
