use rand::Rng;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;

use crate::renderer::{LANE_W, ROAD_W, WINDOW_H, WINDOW_W};
use crate::vehicle::{Direction, Route, Vehicle};

const CENTER_X: i32 = WINDOW_W as i32 / 2;
const CENTER_Y: i32 = WINDOW_H as i32 / 2;

const RANDOM_INTERVAL: u32 = 30; // ticks between random spawns (~0.5 s at 60 fps)

pub struct InputHandler {
    pub random_mode: bool,
    random_timer: u32,
    next_id: u32,
}

impl InputHandler {
    pub fn new() -> Self {
        InputHandler { random_mode: false, random_timer: 0, next_id: 0 }
    }

    /// Handle one SDL2 event. Only processes vehicle-spawn keys; returns false
    /// when the window is closed (Quit event). Esc is handled by the caller.
    pub fn handle_event<R: Rng>(
        &mut self,
        event: &Event,
        vehicles: &mut Vec<Vehicle>,
        rng: &mut R,
    ) -> bool {
        match event {
            Event::Quit { .. } => return false,
            Event::KeyDown { keycode: Some(key), .. } => match *key {
                Keycode::Up    => self.try_spawn(Direction::North, vehicles, rng),
                Keycode::Down  => self.try_spawn(Direction::South, vehicles, rng),
                Keycode::Right => self.try_spawn(Direction::East,  vehicles, rng),
                Keycode::Left  => self.try_spawn(Direction::West,  vehicles, rng),
                Keycode::R => {
                    self.random_mode = !self.random_mode;
                    self.random_timer = 0;
                }
                _ => {}
            },
            _ => {}
        }
        true
    }

    /// Call once per tick; spawns a random vehicle when random mode is active.
    pub fn tick<R: Rng>(&mut self, vehicles: &mut Vec<Vehicle>, rng: &mut R) {
        if !self.random_mode {
            return;
        }
        self.random_timer += 1;
        if self.random_timer >= RANDOM_INTERVAL {
            self.random_timer = 0;
            let dir = random_direction(rng);
            self.try_spawn(dir, vehicles, rng);
        }
    }

    fn try_spawn<R: Rng>(&mut self, direction: Direction, vehicles: &mut Vec<Vehicle>, rng: &mut R) {
        let route = random_route(rng);
        let (sx, sy) = spawn_pos(direction, route);

        // Block if any existing vehicle is too close to the spawn point.
        let blocked = vehicles.iter().any(|v| {
            let dx = v.x - sx;
            let dy = v.y - sy;
            (dx * dx + dy * dy).sqrt() < crate::vehicle::physics::SAFETY_DISTANCE
        });

        if !blocked {
            let id = self.next_id;
            self.next_id += 1;
            vehicles.push(Vehicle::new(id, sx, sy, direction, route));
        }
    }
}

/// Entry (x, y) position for a vehicle with the given direction and route.
/// Vehicles spawn just off-screen at the correct lane centre.
pub fn spawn_pos(direction: Direction, route: Route) -> (f32, f32) {
    let idx = lane_index(direction, route);
    match direction {
        Direction::South => {
            let x = CENTER_X - ROAD_W + idx * LANE_W + LANE_W / 2;
            (x as f32, -20.0)
        }
        Direction::North => {
            let x = CENTER_X + idx * LANE_W + LANE_W / 2;
            (x as f32, WINDOW_H as f32 + 20.0)
        }
        Direction::West => {
            let y = CENTER_Y - ROAD_W + idx * LANE_W + LANE_W / 2;
            (WINDOW_W as f32 + 20.0, y as f32)
        }
        Direction::East => {
            let y = CENTER_Y + idx * LANE_W + LANE_W / 2;
            (-20.0, y as f32)
        }
    }
}

/// 0-based lane index counting from the outer edge of the road inward.
/// Multiply by LANE_W to convert to a pixel offset from the road edge.
pub fn lane_index(direction: Direction, route: Route) -> i32 {
    match direction {
        // Outer-to-inner order: right, straight, left
        Direction::South | Direction::West => match route {
            Route::Right => 0,
            Route::Straight => 1,
            Route::Left => 2,
        },
        // Outer-to-inner order: left, straight, right (mirrored for opposing traffic)
        Direction::North | Direction::East => match route {
            Route::Left => 0,
            Route::Straight => 1,
            Route::Right => 2,
        },
    }
}

fn random_direction<R: Rng>(rng: &mut R) -> Direction {
    match rng.gen_range(0..4) {
        0 => Direction::North,
        1 => Direction::South,
        2 => Direction::East,
        _ => Direction::West,
    }
}

fn random_route<R: Rng>(rng: &mut R) -> Route {
    match rng.gen_range(0..3) {
        0 => Route::Right,
        1 => Route::Straight,
        _ => Route::Left,
    }
}
