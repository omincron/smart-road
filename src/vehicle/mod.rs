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
