use crate::renderer::{CENTER_X, CENTER_Y, ROAD_W};
use crate::vehicle::{Direction, Route, Speed};

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
    /* S_l  */ [false, false, false, false, true,  true,  false, true,  true,  false, false, true ],
    /* N_r  */ [false, false, false, false, false, false, false, false, false, false, false, false],
    /* N_s  */ [false, false, true,  false, false, false, false, true,  true,  false, true,  false],
    /* N_l  */ [false, true,  true,  false, false, false, false, false, true,  false, true,  true ],
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
    let nb = |idx: f32| cx + idx * lw + lw / 2.0; // northbound x
    let wb = |idx: f32| cy - rw + idx * lw + lw / 2.0; // westbound y
    let eb = |idx: f32| cy + idx * lw + lw / 2.0; // eastbound y

    match (direction, route) {
        // ── right turns (corner clips) ────────────────────────────────────
        // Exit into the outermost lane of the cross street (nearest curb).
        (Direction::South, Route::Right) => vec![(sb(0.0), cy - rw), (cx - rw, wb(0.0))],
        (Direction::North, Route::Right) => vec![(nb(2.0), cy + rw), (cx + rw, eb(2.0))],
        (Direction::West, Route::Right) => vec![(cx + rw, wb(0.0)), (nb(2.0), cy - rw)],
        (Direction::East, Route::Right) => vec![(cx - rw, eb(2.0)), (sb(0.0), cy + rw)],

        // ── straight paths ────────────────────────────────────────────────
        (Direction::South, Route::Straight) => vec![(sb(1.0), cy - rw), (sb(1.0), cy + rw)],
        (Direction::North, Route::Straight) => vec![(nb(1.0), cy + rw), (nb(1.0), cy - rw)],
        (Direction::West, Route::Straight) => vec![(cx + rw, wb(1.0)), (cx - rw, wb(1.0))],
        (Direction::East, Route::Straight) => vec![(cx - rw, eb(1.0)), (cx + rw, eb(1.0))],

        // ── left turns (bent path through centre) ────────────────────────
        // Exit into the innermost lane of the cross street.
        // S/Left turns east  → must use eastbound y (eb), not westbound.
        // N/Left turns west  → must use westbound y (wb), not eastbound.
        (Direction::South, Route::Left) => vec![
            (sb(2.0), cy - rw),
            (sb(2.0), eb(0.0)), // turn point (past centre into eastbound half)
            (cx + rw, eb(0.0)),
        ],
        (Direction::North, Route::Left) => vec![
            (nb(0.0), cy + rw),
            (nb(0.0), wb(2.0)), // turn point (past centre into westbound half)
            (cx - rw, wb(2.0)),
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

/// Total geometric path length through the intersection for a given (direction, route).
pub fn crossing_path_length(direction: Direction, route: Route) -> f32 {
    let pts = crossing_waypoints(direction, route);
    pts.windows(2)
        .map(|w| {
            let dx = w[1].0 - w[0].0;
            let dy = w[1].1 - w[0].1;
            (dx * dx + dy * dy).sqrt()
        })
        .sum()
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
/// Remaining distance (px) to the stop line. Always positive while approaching.
pub fn dist_to_stop_line(direction: Direction, x: f32, y: f32) -> f32 {
    let cx = CENTER_X as f32;
    let cy = CENTER_Y as f32;
    let rw = ROAD_W as f32;
    match direction {
        Direction::North => y - (cy + rw),
        Direction::South => (cy - rw) - y,
        Direction::West => x - (cx + rw),
        Direction::East => (cx - rw) - x,
    }
}

/// True once an approaching vehicle's front has reached its stop line.
pub fn at_stop_line(direction: Direction, x: f32, y: f32) -> bool {
    let cx = CENTER_X as f32;
    let cy = CENTER_Y as f32;
    let rw = ROAD_W as f32;
    match direction {
        Direction::South => y >= cy - rw,
        Direction::North => y <= cy + rw,
        Direction::West => x <= cx + rw,
        Direction::East => x >= cx - rw,
    }
}

// ── Time-slot reservation manager ────────────────────────────────────────────
/// Distance from the stop line at which vehicles begin requesting reservations.
pub const RESERVATION_DIST: f32 = 300.0;

/// Vehicles cross at Normal speed; used to estimate crossing window duration.
const CROSSING_SPEED: f32 = Speed::NORMAL_PX;

/// Extra ticks added to crossing window to absorb speed-smoothing imprecision.
const GRACE_TICKS: u64 = 10;

struct TimedReservation {
    vehicle_id: u32,
    path_idx: usize,
    entry_tick: u64,
    exit_tick: u64,
}

pub struct IntersectionManager {
    reservations: Vec<TimedReservation>,
}

impl IntersectionManager {
    pub fn new() -> Self {
        IntersectionManager {
            reservations: Vec::new(),
        }
    }

    /// Purge reservations whose crossing window has fully elapsed.
    /// Call once per simulation tick before processing vehicles.
    pub fn cleanup_expired(&mut self, current_tick: u64) {
        self.reservations.retain(|r| r.exit_tick > current_tick);
    }

    /// Find the earliest free crossing window reachable from `dist_to_stop` px away.
    ///
    /// If the vehicle already holds a reservation, returns its (entry_tick, approach_speed).
    /// Otherwise searches forward in time and books the first conflict-free slot.
    /// Returns None only when the intersection is saturated for the full look-ahead range.
    pub fn try_reserve_timed(
        &mut self,
        vehicle_id: u32,
        direction: Direction,
        route: Route,
        current_tick: u64,
        dist_to_stop: f32,
    ) -> Option<(u64, f32)> {
        // If already reserved, recalculate the approach speed needed to arrive on time.
        // If the entry window has passed (vehicle was blocked by a leader), re-book.
        if let Some(pos) = self
            .reservations
            .iter()
            .position(|r| r.vehicle_id == vehicle_id)
        {
            let entry = self.reservations[pos].entry_tick;
            if current_tick <= entry + GRACE_TICKS
                && current_tick < self.reservations[pos].exit_tick
            {
                let ticks_left = (entry as i64 - current_tick as i64).max(1) as f32;
                let speed = (dist_to_stop / ticks_left).clamp(Speed::SLOW_PX, Speed::FAST_PX);
                return Some((entry, speed));
            }
            // Entry window fully passed — drop and find a new slot below.
            self.reservations.remove(pos);
        }

        let path_len = crossing_path_length(direction, route);
        let crossing_ticks = (path_len / CROSSING_SPEED).ceil() as u64 + GRACE_TICKS;
        let idx = path_index(direction, route);

        // Search window: earliest arrival at full speed, latest at crawl speed + buffer.
        let earliest = current_tick + (dist_to_stop / Speed::FAST_PX).ceil() as u64;
        let search_max = current_tick + (dist_to_stop / Speed::SLOW_PX).floor() as u64 + 400;

        let mut entry = earliest;
        while entry <= search_max {
            let exit = entry + crossing_ticks;
            // Find the earliest exit_tick among all conflicting reservations overlapping
            // [entry, exit). If any exist, jump straight to that tick — skipping the
            // entire blocked window in one step rather than iterating tick-by-tick.
            let jump_to = self
                .reservations
                .iter()
                .filter(|r| {
                    CONFLICTS[idx][r.path_idx] && entry < r.exit_tick && r.entry_tick < exit
                })
                .map(|r| r.exit_tick)
                .min();
            match jump_to {
                None => {
                    let ticks_until = (entry - current_tick).max(1) as f32;
                    let speed = (dist_to_stop / ticks_until).clamp(Speed::SLOW_PX, Speed::FAST_PX);
                    self.reservations.push(TimedReservation {
                        vehicle_id,
                        path_idx: idx,
                        entry_tick: entry,
                        exit_tick: exit,
                    });
                    return Some((entry, speed));
                }
                Some(next) => entry = next,
            }
        }
        None
    }

    /// Fallback reservation for vehicles already at the stop line (Waiting state).
    /// Grants an immediate slot if the full crossing window is free of conflicts
    /// — including future reservations from approaching vehicles.
    pub fn try_reserve(
        &mut self,
        vehicle_id: u32,
        direction: Direction,
        route: Route,
        current_tick: u64,
    ) -> bool {
        if self.reservations.iter().any(|r| r.vehicle_id == vehicle_id) {
            return true;
        }
        let idx = path_index(direction, route);
        let path_len = crossing_path_length(direction, route);
        let crossing_ticks = (path_len / CROSSING_SPEED).ceil() as u64 + GRACE_TICKS;
        let exit_tick = current_tick + crossing_ticks;

        // Full interval overlap: [current_tick, exit_tick) vs [r.entry_tick, r.exit_tick)
        let conflict = self.reservations.iter().any(|r| {
            CONFLICTS[idx][r.path_idx] && current_tick < r.exit_tick && r.entry_tick < exit_tick
        });
        if conflict {
            return false;
        }
        self.reservations.push(TimedReservation {
            vehicle_id,
            path_idx: idx,
            entry_tick: current_tick,
            exit_tick,
        });
        true
    }

    /// Release the reservation held by a vehicle that has finished crossing.
    pub fn release(&mut self, vehicle_id: u32) {
        self.reservations.retain(|r| r.vehicle_id != vehicle_id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::renderer::{CENTER_X, CENTER_Y, ROAD_W};
    use crate::vehicle::{Direction, Route};

    // ── path_index ────────────────────────────────────────────────────────────

    #[test]
    fn path_index_spans_0_to_11() {
        let mut seen = [false; 12];
        for d in [
            Direction::South,
            Direction::North,
            Direction::West,
            Direction::East,
        ] {
            for r in [Route::Right, Route::Straight, Route::Left] {
                let i = path_index(d, r);
                assert!(i < 12, "path_index out of range: {i}");
                assert!(!seen[i], "path_index collision at {i}");
                seen[i] = true;
            }
        }
        assert!(
            seen.iter().all(|&v| v),
            "not all indices 0-11 were produced"
        );
    }

    // ── conflict table ────────────────────────────────────────────────────────

    #[test]
    fn conflicts_table_is_symmetric() {
        for (i, row) in CONFLICTS.iter().enumerate() {
            for (j, &val) in row.iter().enumerate() {
                assert_eq!(val, CONFLICTS[j][i], "asymmetry at [{i}][{j}]");
            }
        }
    }

    #[test]
    fn right_turns_are_conflict_free() {
        // Right turns map to indices 0 (S_r), 3 (N_r), 6 (W_r), 9 (E_r)
        for &r in &[
            path_index(Direction::South, Route::Right),
            path_index(Direction::North, Route::Right),
            path_index(Direction::West, Route::Right),
            path_index(Direction::East, Route::Right),
        ] {
            for (other, &conflict) in CONFLICTS[r].iter().enumerate() {
                assert!(!conflict, "right turn {r} conflicts with {other}");
            }
        }
    }

    // ── crossing_path_length ──────────────────────────────────────────────────

    #[test]
    fn right_turn_shorter_than_straight() {
        let right = crossing_path_length(Direction::South, Route::Right);
        let straight = crossing_path_length(Direction::South, Route::Straight);
        assert!(
            right < straight,
            "right {right:.1} >= straight {straight:.1}"
        );
    }

    #[test]
    fn path_lengths_symmetric_for_opposing_directions() {
        let s = crossing_path_length(Direction::South, Route::Straight);
        let n = crossing_path_length(Direction::North, Route::Straight);
        assert!(
            (s - n).abs() < 1.0,
            "S/N straight lengths differ: {s:.1} vs {n:.1}"
        );

        let w = crossing_path_length(Direction::West, Route::Straight);
        let e = crossing_path_length(Direction::East, Route::Straight);
        assert!(
            (w - e).abs() < 1.0,
            "W/E straight lengths differ: {w:.1} vs {e:.1}"
        );
    }

    // ── exit_direction ────────────────────────────────────────────────────────

    #[test]
    fn exit_direction_straight_is_unchanged() {
        for d in [
            Direction::North,
            Direction::South,
            Direction::East,
            Direction::West,
        ] {
            assert_eq!(exit_direction(d, Route::Straight), d);
        }
    }

    #[test]
    fn exit_direction_right_turns() {
        assert_eq!(
            exit_direction(Direction::South, Route::Right),
            Direction::West
        );
        assert_eq!(
            exit_direction(Direction::North, Route::Right),
            Direction::East
        );
        assert_eq!(
            exit_direction(Direction::West, Route::Right),
            Direction::North
        );
        assert_eq!(
            exit_direction(Direction::East, Route::Right),
            Direction::South
        );
    }

    #[test]
    fn exit_direction_left_turns() {
        assert_eq!(
            exit_direction(Direction::South, Route::Left),
            Direction::East
        );
        assert_eq!(
            exit_direction(Direction::North, Route::Left),
            Direction::West
        );
        assert_eq!(
            exit_direction(Direction::West, Route::Left),
            Direction::South
        );
        assert_eq!(
            exit_direction(Direction::East, Route::Left),
            Direction::North
        );
    }

    // ── stop-line detection ───────────────────────────────────────────────────

    #[test]
    fn dist_to_stop_line_positive_when_approaching() {
        let cx = CENTER_X as f32;
        let cy = CENTER_Y as f32;
        let rw = ROAD_W as f32;
        assert!(dist_to_stop_line(Direction::South, cx, cy - rw - 50.0) > 0.0);
        assert!(dist_to_stop_line(Direction::North, cx, cy + rw + 50.0) > 0.0);
        assert!(dist_to_stop_line(Direction::West, cx + rw + 50.0, cy) > 0.0);
        assert!(dist_to_stop_line(Direction::East, cx - rw - 50.0, cy) > 0.0);
    }

    #[test]
    fn at_stop_line_true_past_threshold_false_before() {
        let cx = CENTER_X as f32;
        let cy = CENTER_Y as f32;
        let rw = ROAD_W as f32;
        // Just past stop line
        assert!(at_stop_line(Direction::South, cx, cy - rw + 1.0));
        assert!(at_stop_line(Direction::North, cx, cy + rw - 1.0));
        assert!(at_stop_line(Direction::West, cx + rw - 1.0, cy));
        assert!(at_stop_line(Direction::East, cx - rw + 1.0, cy));
        // Still approaching
        assert!(!at_stop_line(Direction::South, cx, cy - rw - 50.0));
        assert!(!at_stop_line(Direction::North, cx, cy + rw + 50.0));
    }

    // ── IntersectionManager ───────────────────────────────────────────────────

    #[test]
    fn reserve_and_release() {
        let mut mgr = IntersectionManager::new();
        assert!(mgr.try_reserve(0, Direction::South, Route::Straight, 0));
        mgr.release(0);
        assert!(mgr.try_reserve(0, Direction::South, Route::Straight, 0));
    }

    #[test]
    fn second_reserve_same_vehicle_is_noop() {
        let mut mgr = IntersectionManager::new();
        assert!(mgr.try_reserve(0, Direction::South, Route::Straight, 0));
        // Calling again for the same vehicle should return true without double-booking.
        assert!(mgr.try_reserve(0, Direction::South, Route::Straight, 0));
        assert_eq!(mgr.reservations.len(), 1);
    }

    #[test]
    fn conflicting_paths_block_second_vehicle() {
        let mut mgr = IntersectionManager::new();
        // S_s and N_l conflict in the CONFLICTS table.
        assert!(mgr.try_reserve(0, Direction::South, Route::Straight, 0));
        assert!(!mgr.try_reserve(1, Direction::North, Route::Left, 0));
    }

    #[test]
    fn non_conflicting_paths_both_granted() {
        let mut mgr = IntersectionManager::new();
        // Right turns never conflict.
        assert!(mgr.try_reserve(0, Direction::South, Route::Right, 0));
        assert!(mgr.try_reserve(1, Direction::North, Route::Right, 0));
        assert!(mgr.try_reserve(2, Direction::West, Route::Right, 0));
        assert!(mgr.try_reserve(3, Direction::East, Route::Right, 0));
    }

    #[test]
    fn cleanup_expired_removes_old_slots() {
        let mut mgr = IntersectionManager::new();
        mgr.try_reserve(0, Direction::South, Route::Straight, 0);

        // Slot should still block a conflict shortly after.
        mgr.cleanup_expired(1);
        assert!(!mgr.try_reserve(1, Direction::North, Route::Left, 1));

        // After the full crossing window expires, the conflict clears.
        let path_len = crossing_path_length(Direction::South, Route::Straight);
        let expire_at = (path_len / CROSSING_SPEED).ceil() as u64 + GRACE_TICKS + 50;
        mgr.cleanup_expired(expire_at);
        assert!(mgr.try_reserve(1, Direction::North, Route::Left, expire_at));
    }
}
