pub mod physics;

use physics::SMOOTH_ALPHA;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    North, // travelling north, enters from south
    South, // travelling south, enters from north
    East,  // travelling east, enters from west
    West,  // travelling west, enters from east
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    Right,
    Straight,
    Left,
}

pub struct Speed;

impl Speed {
    pub const SLOW_PX: f32 = 1.5;
    pub const NORMAL_PX: f32 = 3.0;
    pub const FAST_PX: f32 = 5.0;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleState {
    Approaching, // moving down the approach lane toward the intersection
    Waiting,     // stopped at the stop line, waiting for a reservation
    Crossing,    // has a reservation, currently crossing
    Exiting,     // cleared the intersection, leaving the screen
    Done,        // off-screen, ready to be removed
}

#[derive(Debug, Clone)]
pub struct Vehicle {
    pub id: u32,
    pub x: f32,
    pub y: f32,
    pub direction: Direction,
    pub route: Route,
    pub current_speed: f32,
    pub target_speed: f32,
    pub reservation: Option<u64>, // entry_tick assigned by IntersectionManager
    pub state: VehicleState,
    pub angle_deg: f32, // clockwise degrees; 0 = pointing up (north)

    // path through the intersection — set when a reservation is granted
    pub crossing_path: Vec<(f32, f32)>,
    pub waypoint_idx: usize,

    // stats — filled in as simulation progresses
    pub detection_tick: Option<u64>, // None until vehicle first reaches the stop line
    pub exit_tick: Option<u64>,
}

impl Vehicle {
    pub fn new(id: u32, x: f32, y: f32, direction: Direction, route: Route) -> Self {
        Vehicle {
            id,
            x,
            y,
            direction,
            route,
            current_speed: Speed::NORMAL_PX,
            target_speed: Speed::NORMAL_PX,
            reservation: None,
            state: VehicleState::Approaching,
            angle_deg: direction_to_angle(direction),
            crossing_path: Vec::new(),
            waypoint_idx: 0,
            detection_tick: None,
            exit_tick: None,
        }
    }

    /// Transit time in ticks from first stop-line detection to off-screen exit.
    pub fn transit_ticks(&self) -> Option<u64> {
        match (self.detection_tick, self.exit_tick) {
            (Some(d), Some(e)) => Some(e - d),
            _ => None,
        }
    }

    /// Exponential approach toward target_speed. Call once per tick before advance().
    pub fn smooth_speed(&mut self) {
        self.current_speed += (self.target_speed - self.current_speed) * SMOOTH_ALPHA;
    }

    /// Straight-line advance along current direction (Approaching / Exiting).
    pub fn advance(&mut self) {
        let d = self.current_speed;
        match self.direction {
            Direction::North => self.y -= d,
            Direction::South => self.y += d,
            Direction::East => self.x += d,
            Direction::West => self.x -= d,
        }
    }

    /// Move one tick along the pre-computed crossing path.
    /// Returns true when the last waypoint has been reached (vehicle has exited).
    pub fn advance_crossing(&mut self) -> bool {
        if self.waypoint_idx >= self.crossing_path.len() {
            return true;
        }
        let (tx, ty) = self.crossing_path[self.waypoint_idx];
        let dx = tx - self.x;
        let dy = ty - self.y;
        let dist = (dx * dx + dy * dy).sqrt();
        let speed = self.current_speed;

        if dist <= speed {
            // Update angle to face the snapped waypoint before moving past it.
            self.angle_deg = (dx.atan2(-dy).to_degrees() + 360.0) % 360.0;
            self.x = tx;
            self.y = ty;
            self.waypoint_idx += 1;
            self.waypoint_idx >= self.crossing_path.len()
        } else {
            self.x += dx / dist * speed;
            self.y += dy / dist * speed;
            self.angle_deg = (dx.atan2(-dy).to_degrees() + 360.0) % 360.0;
            false
        }
    }
}

pub fn direction_to_angle(dir: Direction) -> f32 {
    match dir {
        Direction::North => 0.0,
        Direction::East => 90.0,
        Direction::South => 180.0,
        Direction::West => 270.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn veh(dir: Direction) -> Vehicle {
        Vehicle::new(0, 400.0, 400.0, dir, Route::Straight)
    }

    // ── direction_to_angle ────────────────────────────────────────────────────

    #[test]
    fn direction_to_angle_correct() {
        assert_eq!(direction_to_angle(Direction::North), 0.0);
        assert_eq!(direction_to_angle(Direction::East), 90.0);
        assert_eq!(direction_to_angle(Direction::South), 180.0);
        assert_eq!(direction_to_angle(Direction::West), 270.0);
    }

    #[test]
    fn new_vehicle_angle_matches_direction() {
        for (dir, expected) in [
            (Direction::North, 0.0f32),
            (Direction::East, 90.0),
            (Direction::South, 180.0),
            (Direction::West, 270.0),
        ] {
            let v = veh(dir);
            assert!(
                (v.angle_deg - expected).abs() < 1e-4,
                "{dir:?}: expected {expected}, got {}",
                v.angle_deg
            );
        }
    }

    // ── advance ───────────────────────────────────────────────────────────────

    #[test]
    fn advance_north_decreases_y() {
        let mut v = veh(Direction::North);
        let y0 = v.y;
        v.advance();
        assert!(v.y < y0, "North advance should decrease y");
        assert_eq!(v.x, 400.0, "x must not change");
    }

    #[test]
    fn advance_south_increases_y() {
        let mut v = veh(Direction::South);
        let y0 = v.y;
        v.advance();
        assert!(v.y > y0);
        assert_eq!(v.x, 400.0);
    }

    #[test]
    fn advance_east_increases_x() {
        let mut v = veh(Direction::East);
        let x0 = v.x;
        v.advance();
        assert!(v.x > x0);
        assert_eq!(v.y, 400.0);
    }

    #[test]
    fn advance_west_decreases_x() {
        let mut v = veh(Direction::West);
        let x0 = v.x;
        v.advance();
        assert!(v.x < x0);
        assert_eq!(v.y, 400.0);
    }

    #[test]
    fn advance_moves_by_current_speed() {
        let mut v = veh(Direction::South);
        v.current_speed = 7.0;
        v.advance();
        assert!((v.y - 407.0).abs() < 1e-4);
    }

    // ── smooth_speed ──────────────────────────────────────────────────────────

    #[test]
    fn smooth_speed_moves_toward_higher_target() {
        let mut v = veh(Direction::North);
        v.current_speed = 1.0;
        v.target_speed = 5.0;
        let before = v.current_speed;
        v.smooth_speed();
        assert!(v.current_speed > before);
        assert!(v.current_speed < 5.0, "must not overshoot in one step");
    }

    #[test]
    fn smooth_speed_moves_toward_lower_target() {
        let mut v = veh(Direction::North);
        v.current_speed = 5.0;
        v.target_speed = 1.0;
        let before = v.current_speed;
        v.smooth_speed();
        assert!(v.current_speed < before);
        assert!(v.current_speed > 1.0);
    }

    #[test]
    fn smooth_speed_stable_when_at_target() {
        let mut v = veh(Direction::North);
        v.current_speed = 3.0;
        v.target_speed = 3.0;
        v.smooth_speed();
        assert!((v.current_speed - 3.0).abs() < 1e-5);
    }

    // ── transit_ticks ─────────────────────────────────────────────────────────

    #[test]
    fn transit_ticks_none_without_detection() {
        let mut v = veh(Direction::North);
        v.exit_tick = Some(100);
        assert!(v.transit_ticks().is_none());
    }

    #[test]
    fn transit_ticks_none_without_exit() {
        let mut v = veh(Direction::North);
        v.detection_tick = Some(10);
        assert!(v.transit_ticks().is_none());
    }

    #[test]
    fn transit_ticks_computed_correctly() {
        let mut v = veh(Direction::North);
        v.detection_tick = Some(20);
        v.exit_tick = Some(80);
        assert_eq!(v.transit_ticks(), Some(60));
    }

    // ── advance_crossing ──────────────────────────────────────────────────────

    #[test]
    fn advance_crossing_moves_toward_waypoint() {
        let mut v = veh(Direction::North);
        // Place waypoint directly above the vehicle.
        v.crossing_path = vec![(400.0, 300.0)];
        v.waypoint_idx = 0;
        v.current_speed = Speed::NORMAL_PX;
        let y0 = v.y;
        let done = v.advance_crossing();
        assert!(!done);
        assert!(v.y < y0, "vehicle should have moved upward toward waypoint");
    }

    #[test]
    fn advance_crossing_snaps_to_waypoint_when_close() {
        let mut v = veh(Direction::North);
        // Position vehicle within one tick of the waypoint.
        v.x = 400.0;
        v.y = 401.0;
        v.crossing_path = vec![(400.0, 400.0), (400.0, 300.0)];
        v.waypoint_idx = 0;
        v.current_speed = Speed::FAST_PX;
        v.advance_crossing();
        // Should have snapped to first waypoint and advanced index.
        assert_eq!(v.x, 400.0);
        assert_eq!(v.y, 400.0);
        assert_eq!(v.waypoint_idx, 1);
    }

    #[test]
    fn advance_crossing_returns_true_after_last_waypoint() {
        let mut v = veh(Direction::North);
        v.x = 400.0;
        v.y = 400.5;
        v.crossing_path = vec![(400.0, 400.0)];
        v.waypoint_idx = 0;
        v.current_speed = Speed::FAST_PX;
        let done = v.advance_crossing();
        assert!(
            done,
            "should signal completion after reaching the last waypoint"
        );
    }

    #[test]
    fn advance_crossing_true_when_waypoints_already_exhausted() {
        let mut v = veh(Direction::North);
        v.crossing_path = vec![(400.0, 300.0)];
        v.waypoint_idx = 1; // already past the only waypoint
        let done = v.advance_crossing();
        assert!(done);
    }
}
