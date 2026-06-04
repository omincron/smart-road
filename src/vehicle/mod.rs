pub mod physics;

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
    pub detection_tick: u64,
    pub exit_tick: Option<u64>,
    pub distance_traveled: f32,
    pub min_gap_seen: f32,
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
            detection_tick: 0,
            exit_tick: None,
            distance_traveled: 0.0,
            min_gap_seen: f32::MAX,
        }
    }

    pub fn transit_ticks(&self) -> Option<u64> {
        self.exit_tick.map(|e| e - self.detection_tick)
    }

    /// Exponential approach toward target_speed. Call once per tick before advance().
    pub fn smooth_speed(&mut self) {
        const ALPHA: f32 = 0.12;
        self.current_speed += (self.target_speed - self.current_speed) * ALPHA;
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
        self.distance_traveled += d;
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
            self.x = tx;
            self.y = ty;
            self.waypoint_idx += 1;
            self.distance_traveled += dist;
            self.waypoint_idx >= self.crossing_path.len()
        } else {
            self.x += dx / dist * speed;
            self.y += dy / dist * speed;
            self.distance_traveled += speed;
            // keep angle aligned with movement vector
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
