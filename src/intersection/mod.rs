use crate::renderer::{CENTER_X, CENTER_Y, ROAD_W};
use crate::vehicle::{Direction, Route};

// ── Path index encoding ───────────────────────────────────────────────────────
// path_index = direction_index * 3 + route_index
//   South=0, North=1, West=2, East=3
//   Right=0, Straight=1, Left=2
//
//  0 S_r   1 S_s   2 S_l
//  3 N_r   4 N_s   5 N_l
//  6 W_r   7 W_s   8 W_l
//  9 E_r  10 E_s  11 E_l

fn path_index(direction: Direction, route: Route) -> usize {
    let d = match direction {
        Direction::South => 0,
        Direction::North => 1,
        Direction::West => 2,
        Direction::East => 3,
    };
    let r = match route {
        Route::Right => 0,
        Route::Straight => 1,
        Route::Left => 2,
    };
    d * 3 + r
}

// ── Conflict table ────────────────────────────────────────────────────────────
// Derived from tile-overlap analysis of all 12 paths through the intersection.
// Right turns (indices 0, 3, 6, 9) are conflict-free — they stay in the corner.
#[rustfmt::skip]
const CONFLICTS: [[bool; 12]; 12] = [
    //          S_r    S_s    S_l    N_r    N_s    N_l    W_r    W_s    W_l    E_r    E_s    E_l
    /* S_r  */ [false, false, false, false, false, false, false, false, false, false, false, false],
    /* S_s  */ [false, false, false, false, false, true,  false, true,  false, false, true,  true ],
    /* S_l  */ [false, false, false, false, true,  false, false, true,  true,  false, false, true ],
    /* N_r  */ [false, false, false, false, false, false, false, false, false, false, false, false],
    /* N_s  */ [false, false, true,  false, false, false, false, true,  true,  false, true,  false],
    /* N_l  */ [false, true,  false, false, false, false, false, false, true,  false, true,  true ],
    /* W_r  */ [false, false, false, false, false, false, false, false, false, false, false, false],
    /* W_s  */ [false, true,  true,  false, true,  false, false, false, false, false, false, true ],
    /* W_l  */ [false, false, true,  false, true,  true,  false, false, false, false, true,  true ],
    /* E_r  */ [false, false, false, false, false, false, false, false, false, false, false, false],
    /* E_s  */ [false, true,  false, false, true,  true,  false, false, true,  false, false, false],
    /* E_l  */ [false, true,  true,  false, false, true,  false, true,  true,  false, false, false],
];

// ── Crossing waypoints ────────────────────────────────────────────────────────
// Pre-computed (x, y) waypoints for each (direction, route) through the box.
// Coordinates use renderer constants so they stay in sync with the road layout.
pub fn crossing_waypoints(direction: Direction, route: Route) -> Vec<(f32, f32)> {
    let cx = CENTER_X as f32;
    let cy = CENTER_Y as f32;
    let rw = ROAD_W as f32;
    let lw = rw / 3.0; // LANE_W

    // lane-centre helpers
    let sb = |idx: f32| cx - rw + idx * lw + lw / 2.0; // southbound x
    let nb = |idx: f32| cx + idx * lw + lw / 2.0;       // northbound x
    let wb = |idx: f32| cy - rw + idx * lw + lw / 2.0;  // westbound y
    let eb = |idx: f32| cy + idx * lw + lw / 2.0;       // eastbound y

    match (direction, route) {
        // ── right turns (corner clips) ────────────────────────────────────
        (Direction::South, Route::Right) => vec![(sb(0.0), cy - rw), (cx - rw, wb(0.0))],
        (Direction::North, Route::Right) => vec![(nb(2.0), cy + rw), (cx + rw, eb(0.0))],
        (Direction::West,  Route::Right) => vec![(cx + rw, wb(0.0)), (nb(2.0), cy - rw)],
        (Direction::East,  Route::Right) => vec![(cx - rw, eb(2.0)), (sb(0.0), cy + rw)],

        // ── straight paths ────────────────────────────────────────────────
        (Direction::South, Route::Straight) => vec![(sb(1.0), cy - rw), (sb(1.0), cy + rw)],
        (Direction::North, Route::Straight) => vec![(nb(1.0), cy + rw), (nb(1.0), cy - rw)],
        (Direction::West,  Route::Straight) => vec![(cx + rw, wb(1.0)), (cx - rw, wb(1.0))],
        (Direction::East,  Route::Straight) => vec![(cx - rw, eb(1.0)), (cx + rw, eb(1.0))],

        // ── left turns (bent path through centre) ────────────────────────
        (Direction::South, Route::Left) => vec![
            (sb(2.0), cy - rw),
            (sb(2.0), wb(2.0)), // turn point
            (cx + rw, wb(2.0)),
        ],
        (Direction::North, Route::Left) => vec![
            (nb(0.0), cy + rw),
            (nb(0.0), eb(0.0)), // turn point
            (cx - rw, eb(0.0)),
        ],
        (Direction::West, Route::Left) => vec![
            (cx + rw, wb(2.0)),
            (sb(2.0), wb(2.0)), // turn point
            (sb(2.0), cy + rw),
        ],
        (Direction::East, Route::Left) => vec![
            (cx - rw, eb(0.0)),
            (nb(0.0), eb(0.0)), // turn point
            (nb(0.0), cy - rw),
        ],
    }
}

/// Direction the vehicle is heading after it exits the intersection.
pub fn exit_direction(direction: Direction, route: Route) -> Direction {
    match route {
        Route::Straight => direction,
        Route::Right => match direction {
            Direction::South => Direction::West,
            Direction::North => Direction::East,
            Direction::West => Direction::North,
            Direction::East => Direction::South,
        },
        Route::Left => match direction {
            Direction::South => Direction::East,
            Direction::North => Direction::West,
            Direction::West => Direction::South,
            Direction::East => Direction::North,
        },
    }
}

// ── Stop-line detection ───────────────────────────────────────────────────────
/// True once an approaching vehicle's front has reached its stop line.
pub fn at_stop_line(direction: Direction, x: f32, y: f32) -> bool {
    let cx = CENTER_X as f32;
    let cy = CENTER_Y as f32;
    let rw = ROAD_W as f32;
    match direction {
        Direction::South => y >= cy - rw,
        Direction::North => y <= cy + rw,
        Direction::West  => x <= cx + rw,
        Direction::East  => x >= cx - rw,
    }
}

// ── Reservation manager ───────────────────────────────────────────────────────
pub struct IntersectionManager {
    active: Vec<(u32, usize)>, // (vehicle_id, path_index)
}

impl IntersectionManager {
    pub fn new() -> Self {
        IntersectionManager { active: Vec::new() }
    }

    /// Try to grant a crossing reservation. Returns true if granted.
    pub fn try_reserve(&mut self, vehicle_id: u32, direction: Direction, route: Route) -> bool {
        // Already reserved (e.g. called twice for the same vehicle)
        if self.active.iter().any(|(id, _)| *id == vehicle_id) {
            return true;
        }
        let idx = path_index(direction, route);
        let conflict = self.active.iter().any(|(_, active_idx)| CONFLICTS[idx][*active_idx]);
        if conflict {
            return false;
        }
        self.active.push((vehicle_id, idx));
        true
    }

    /// Release the reservation held by a vehicle that has finished crossing.
    pub fn release(&mut self, vehicle_id: u32) {
        self.active.retain(|(id, _)| *id != vehicle_id);
    }
}
