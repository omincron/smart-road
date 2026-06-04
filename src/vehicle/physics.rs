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
