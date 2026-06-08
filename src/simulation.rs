use std::collections::{HashMap, HashSet};

use crate::intersection::{self, IntersectionManager};
use crate::renderer;
use crate::statistics::StatsAccumulator;
use crate::vehicle::physics::{MIN_FOLLOWING_GAP, SMOOTH_ALPHA};
use crate::vehicle::{Direction, Route, Speed, VehicleState, direction_to_angle};

// ── Per-lane O(n) leader lookup ───────────────────────────────────────────────
// Key: (Direction, Route). Value: (vehicle_id, axial_progress) sorted ascending.
// Includes vehicles that still block the approach lane, including a Crossing
// vehicle that has not yet moved off the stop line.
type LaneGroups = HashMap<(Direction, Route), Vec<(u32, f32)>>;

/// Converts a vehicle position to a scalar that increases as the vehicle approaches
/// the intersection. Values are comparable only within the same (Direction, Route) lane.
fn axial_pos(dir: Direction, x: f32, y: f32) -> f32 {
    match dir {
        Direction::North => -y,
        Direction::South => y,
        Direction::East => x,
        Direction::West => -x,
    }
}

/// Build per-lane sorted groups from the current vehicle list (O(n log n)).
fn build_lane_groups(vehicles: &[crate::vehicle::Vehicle]) -> LaneGroups {
    let mut groups: LaneGroups = HashMap::new();
    for v in vehicles {
        let at_stop = intersection::dist_to_stop_line(v.direction, v.x, v.y).abs() < 0.5;
        if matches!(v.state, VehicleState::Approaching | VehicleState::Waiting)
            || matches!(v.state, VehicleState::Crossing) && at_stop
        {
            groups
                .entry((v.direction, v.route))
                .or_default()
                .push((v.id, axial_pos(v.direction, v.x, v.y)));
        }
    }
    for group in groups.values_mut() {
        group.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    }
    groups
}

/// Centre-to-centre axial gap to the nearest leader in the same lane (O(log n)).
/// Returns f32::MAX when no leader exists.
fn lane_leader_gap(
    id: u32,
    dir: Direction,
    route: Route,
    x: f32,
    y: f32,
    groups: &LaneGroups,
) -> f32 {
    let my_pos = axial_pos(dir, x, y);
    let group = match groups.get(&(dir, route)) {
        Some(g) => g,
        None => return f32::MAX,
    };
    // Group is sorted ascending; leader = first entry strictly ahead (pos > my_pos).
    let start = group.partition_point(|&(_, pos)| pos <= my_pos);
    group[start..]
        .iter()
        .find(|&&(gid, _)| gid != id)
        .map(|&(_, leader_pos)| leader_pos - my_pos)
        .unwrap_or(f32::MAX)
}

/// Maximum safe target speed given a centre-to-centre leader gap.
/// v_max = SMOOTH_ALPHA × clearance ensures the vehicle cannot overshoot MIN_FOLLOWING_GAP.
fn max_follow_speed(gap: f32) -> f32 {
    let clearance = gap - MIN_FOLLOWING_GAP;
    (clearance * SMOOTH_ALPHA).clamp(0.0, Speed::FAST_PX)
}

/// Distance threshold (px) beyond which a free-approach vehicle travels at full speed.
const APPROACH_FAR_PX: f32 = 200.0;
/// Distance threshold (px) below which a free-approach vehicle slows to minimum speed.
const APPROACH_NEAR_PX: f32 = 80.0;
/// Extra margin beyond the window edge before an Exiting vehicle is considered off-screen.
const OFFSCREEN_MARGIN: f32 = 80.0;

/// Target speed for an approaching vehicle outside the reservation zone.
/// Fast > APPROACH_FAR_PX from stop line or leader, Normal in between, Slow < APPROACH_NEAR_PX.
fn approach_target_speed(dist_to_stop: f32, leader_gap: f32) -> f32 {
    let constraint = dist_to_stop.min(leader_gap);
    if constraint > APPROACH_FAR_PX {
        Speed::FAST_PX
    } else if constraint > APPROACH_NEAR_PX {
        Speed::NORMAL_PX
    } else {
        Speed::SLOW_PX
    }
}

pub fn step_simulation(
    tick: u64,
    vehicles: &mut Vec<crate::vehicle::Vehicle>,
    manager: &mut IntersectionManager,
    stats: &mut StatsAccumulator,
) {
    let mut exits: Vec<u64> = Vec::new();

    // Per-lane sorted groups for O(n) leader lookup — built once per tick.
    let groups = build_lane_groups(vehicles);

    manager.cleanup_expired(tick);

    for v in vehicles.iter_mut() {
        if v.state != VehicleState::Done {
            stats.record_speed(v.current_speed);
        }

        match v.state {
            VehicleState::Approaching => {
                if intersection::at_stop_line(v.direction, v.x, v.y) {
                    if v.detection_tick.is_none() {
                        v.detection_tick = Some(tick);
                    }
                    if let Some(entry_tick) = v.reservation {
                        if tick >= entry_tick {
                            v.state = VehicleState::Crossing;
                            v.crossing_path =
                                intersection::crossing_waypoints(v.direction, v.route);
                            v.waypoint_idx = 0;
                        }
                        // else: hold at stop line until entry_tick arrives
                    } else {
                        v.state = VehicleState::Waiting;
                    }
                } else {
                    // Compute lane leader gap once — used for speed, cap, and blocking.
                    let gap = lane_leader_gap(v.id, v.direction, v.route, v.x, v.y, &groups);
                    let dist = intersection::dist_to_stop_line(v.direction, v.x, v.y);

                    // 1. Desired target speed from AIM or free-flow following.
                    if dist <= intersection::RESERVATION_DIST {
                        match manager.try_reserve_timed(v.id, v.direction, v.route, tick, dist) {
                            Some((entry, spd)) => {
                                v.reservation = Some(entry);
                                v.target_speed = spd;
                            }
                            None => {
                                v.reservation = None;
                                v.target_speed = Speed::SLOW_PX;
                            }
                        }
                    } else {
                        v.target_speed = approach_target_speed(dist, gap);
                    }

                    // 2. Cap by safe following speed — overrides AIM when a leader
                    //    is close. Caps current_speed too so smooth_speed can't
                    //    carry excess momentum into this tick's advance().
                    let follow_cap = max_follow_speed(gap);
                    v.target_speed = v.target_speed.min(follow_cap);
                    v.current_speed = v.current_speed.min(follow_cap);

                    // 3. Interpolate toward target, then advance.
                    v.smooth_speed();
                    if gap >= MIN_FOLLOWING_GAP {
                        v.advance();
                    }
                }
            }
            VehicleState::Waiting => {
                if manager.try_reserve(v.id, v.direction, v.route, tick) {
                    v.state = VehicleState::Crossing;
                    v.crossing_path = intersection::crossing_waypoints(v.direction, v.route);
                    v.waypoint_idx = 0;
                    v.target_speed = Speed::NORMAL_PX;
                }
            }
            VehicleState::Crossing => {
                v.target_speed = Speed::NORMAL_PX;
                v.smooth_speed();
                if v.advance_crossing() {
                    let exit_dir = intersection::exit_direction(v.direction, v.route);
                    v.direction = exit_dir;
                    v.angle_deg = direction_to_angle(exit_dir);
                    v.state = VehicleState::Exiting;
                    v.reservation = None;
                    manager.release(v.id);
                }
            }
            VehicleState::Exiting => {
                v.target_speed = Speed::FAST_PX;
                v.smooth_speed();
                v.advance();
                let w = renderer::WINDOW_W as f32;
                let h = renderer::WINDOW_H as f32;
                if v.x < -OFFSCREEN_MARGIN
                    || v.x > w + OFFSCREEN_MARGIN
                    || v.y < -OFFSCREEN_MARGIN
                    || v.y > h + OFFSCREEN_MARGIN
                {
                    v.state = VehicleState::Done;
                    v.exit_tick = Some(tick);
                    if let Some(transit) = v.transit_ticks() {
                        exits.push(transit);
                    }
                }
            }
            VehicleState::Done => {}
        }
    }

    for transit in exits {
        stats.record_exit(transit);
    }

    // ── close-call detection ───────────────────────────────────────────────
    let mut violations: HashSet<(u32, u32)> = HashSet::new();

    // Same-lane following violations: O(n) over consecutive sorted pairs.
    for group in groups.values() {
        for w in group.windows(2) {
            let (id_a, pos_a) = w[0];
            let (id_b, pos_b) = w[1];
            let axial_gap = pos_b - pos_a;
            stats.record_gap(axial_gap);
            if axial_gap < MIN_FOLLOWING_GAP {
                violations.insert((id_a.min(id_b), id_a.max(id_b)));
            }
        }
    }

    // Physical proximity for vehicles in the intersection zone: O(k²), k small.
    let in_zone: Vec<&crate::vehicle::Vehicle> = vehicles
        .iter()
        .filter(|v| matches!(v.state, VehicleState::Crossing | VehicleState::Exiting))
        .collect();
    for i in 0..in_zone.len() {
        for j in (i + 1)..in_zone.len() {
            let (a, b) = (in_zone[i], in_zone[j]);
            let gap = crate::vehicle::physics::distance(a.x, a.y, b.x, b.y);
            stats.record_gap(gap);
            if crate::vehicle::physics::is_close_call(a.x, a.y, b.x, b.y) {
                violations.insert((a.id.min(b.id), a.id.max(b.id)));
            }
        }
    }

    stats.update_violations(violations);

    vehicles.retain(|v| v.state != VehicleState::Done);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::physics::MIN_FOLLOWING_GAP;

    fn veh(id: u32, dir: Direction, route: Route, x: f32, y: f32) -> crate::vehicle::Vehicle {
        crate::vehicle::Vehicle::new(id, x, y, dir, route)
    }

    // ── axial_pos ─────────────────────────────────────────────────────────────

    #[test]
    fn axial_pos_north_negates_y() {
        assert_eq!(axial_pos(Direction::North, 0.0, 300.0), -300.0);
    }

    #[test]
    fn axial_pos_south_returns_y() {
        assert_eq!(axial_pos(Direction::South, 0.0, 300.0), 300.0);
    }

    #[test]
    fn axial_pos_east_returns_x() {
        assert_eq!(axial_pos(Direction::East, 150.0, 0.0), 150.0);
    }

    #[test]
    fn axial_pos_west_negates_x() {
        assert_eq!(axial_pos(Direction::West, 150.0, 0.0), -150.0);
    }

    // ── build_lane_groups ─────────────────────────────────────────────────────

    #[test]
    fn build_lane_groups_separates_by_direction_and_route() {
        let a = veh(0, Direction::North, Route::Straight, 400.0, 700.0);
        let b = veh(1, Direction::North, Route::Right, 430.0, 700.0);
        let c = veh(2, Direction::North, Route::Straight, 400.0, 600.0);
        let groups = build_lane_groups(&[a, b, c]);
        assert_eq!(
            groups[&(Direction::North, Route::Straight)].len(),
            2,
            "N/Straight should have 2 vehicles"
        );
        assert_eq!(
            groups[&(Direction::North, Route::Right)].len(),
            1,
            "N/Right should have 1 vehicle"
        );
    }

    #[test]
    fn build_lane_groups_sorted_ascending_by_axial_pos() {
        // North axial_pos = -y. Vehicle at y=700 → pos=-700 (further back than y=600 → pos=-600).
        let a = veh(0, Direction::North, Route::Straight, 400.0, 700.0);
        let b = veh(1, Direction::North, Route::Straight, 400.0, 600.0);
        let groups = build_lane_groups(&[a, b]);
        let group = &groups[&(Direction::North, Route::Straight)];
        assert_eq!(group[0].0, 0, "vehicle at y=700 (pos=-700) should be first");
        assert_eq!(
            group[1].0, 1,
            "vehicle at y=600 (pos=-600) should be second"
        );
    }

    #[test]
    fn build_lane_groups_includes_waiting_vehicles() {
        let mut a = veh(0, Direction::North, Route::Straight, 400.0, 482.0);
        a.state = VehicleState::Waiting;
        let groups = build_lane_groups(&[a]);
        assert_eq!(groups[&(Direction::North, Route::Straight)].len(), 1);
    }

    #[test]
    fn build_lane_groups_includes_crossing_vehicle_at_stop_line() {
        let mut a = veh(0, Direction::North, Route::Straight, 400.0, 482.0);
        a.state = VehicleState::Crossing;
        let groups = build_lane_groups(&[a]);
        assert_eq!(groups[&(Direction::North, Route::Straight)].len(), 1);
    }

    #[test]
    fn build_lane_groups_excludes_crossing_vehicle_after_clearing_stop_line() {
        let mut a = veh(0, Direction::North, Route::Straight, 400.0, 470.0);
        a.state = VehicleState::Crossing;
        let groups = build_lane_groups(&[a]);
        assert!(!groups.contains_key(&(Direction::North, Route::Straight)));
    }

    // ── lane_leader_gap ───────────────────────────────────────────────────────

    #[test]
    fn lane_leader_gap_max_when_no_vehicles() {
        let groups = build_lane_groups(&[]);
        let gap = lane_leader_gap(0, Direction::North, Route::Straight, 400.0, 700.0, &groups);
        assert_eq!(gap, f32::MAX);
    }

    #[test]
    fn lane_leader_gap_returns_nearest_ahead() {
        // North: two vehicles ahead at 100 and 200 px axial distance.
        let a = veh(1, Direction::North, Route::Straight, 400.0, 600.0); // 100 ahead
        let b = veh(2, Direction::North, Route::Straight, 400.0, 500.0); // 200 ahead
        let groups = build_lane_groups(&[a, b]);
        let gap = lane_leader_gap(0, Direction::North, Route::Straight, 400.0, 700.0, &groups);
        assert!((gap - 100.0).abs() < 1.0, "expected 100, got {gap}");
    }

    // ── max_follow_speed ──────────────────────────────────────────────────────

    #[test]
    fn max_follow_speed_zero_when_at_min_gap() {
        let speed = max_follow_speed(MIN_FOLLOWING_GAP);
        assert_eq!(speed, 0.0);
    }

    #[test]
    fn max_follow_speed_clamped_to_fast_px() {
        let speed = max_follow_speed(f32::MAX);
        assert_eq!(speed, Speed::FAST_PX);
    }

    #[test]
    fn max_follow_speed_positive_with_clearance() {
        let speed = max_follow_speed(MIN_FOLLOWING_GAP + 100.0);
        assert!(speed > 0.0);
        assert!(speed <= Speed::FAST_PX);
    }

    // ── approach_target_speed ─────────────────────────────────────────────────

    #[test]
    fn approach_speed_fast_far_from_stop_line() {
        assert_eq!(approach_target_speed(418.0, f32::MAX), Speed::FAST_PX);
    }

    #[test]
    fn approach_speed_normal_at_medium_distance() {
        assert_eq!(approach_target_speed(118.0, f32::MAX), Speed::NORMAL_PX);
    }

    #[test]
    fn approach_speed_slow_near_stop_line() {
        assert_eq!(approach_target_speed(38.0, f32::MAX), Speed::SLOW_PX);
    }

    #[test]
    fn approach_speed_capped_by_close_leader() {
        // Leader gap 60 < 80 dominates over dist_to_stop 500 → SLOW.
        assert_eq!(approach_target_speed(500.0, 60.0), Speed::SLOW_PX);
    }

    // ── step_simulation ──────────────────────────────────────────────────────

    #[test]
    fn step_simulation_can_run_without_sdl_types() {
        let mut vehicles = vec![veh(0, Direction::South, Route::Right, 400.0, 200.0)];
        let mut manager = IntersectionManager::new();
        let mut stats = StatsAccumulator::new();

        step_simulation(0, &mut vehicles, &mut manager, &mut stats);

        assert_eq!(vehicles.len(), 1);
        assert!(stats.vehicles_passed <= 1);
    }
}
