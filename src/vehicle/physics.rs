pub const VEHICLE_LENGTH: f32 = 32.0;
pub const SAFETY_DISTANCE: f32 = 40.0; // minimum gap between vehicles (front-to-back)

pub fn distance(x1: f32, y1: f32, x2: f32, y2: f32) -> f32 {
    let dx = x1 - x2;
    let dy = y1 - y2;
    (dx * dx + dy * dy).sqrt()
}

/// Returns true when two vehicles are closer than the safety threshold.
pub fn is_close_call(x1: f32, y1: f32, x2: f32, y2: f32) -> bool {
    distance(x1, y1, x2, y2) < SAFETY_DISTANCE
}
