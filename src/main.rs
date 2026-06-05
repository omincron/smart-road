mod input;
mod intersection;
mod renderer;
mod statistics;
mod vehicle;

use std::collections::{HashMap, HashSet};

use rand::thread_rng;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;

use input::InputHandler;
use intersection::IntersectionManager;
use renderer::Renderer;
use statistics::StatsAccumulator;
use vehicle::physics::{MIN_FOLLOWING_GAP, SMOOTH_ALPHA};
use vehicle::{Direction, Route, Speed, VehicleState, direction_to_angle};

// ── Per-lane O(n) leader lookup ───────────────────────────────────────────────
// Key: (Direction, Route). Value: (vehicle_id, axial_progress) sorted ascending.
// Only Approaching/Waiting vehicles are included.
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
fn build_lane_groups(vehicles: &[vehicle::Vehicle]) -> LaneGroups {
    let mut groups: LaneGroups = HashMap::new();
    for v in vehicles {
        if matches!(v.state, VehicleState::Approaching | VehicleState::Waiting) {
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

#[derive(PartialEq)]
enum SimState {
    Running,
    ShowingStats,
}

fn main() {
    let sdl = sdl2::init().expect("SDL2 init failed");
    let video = sdl.video().expect("SDL2 video init failed");

    let window = video
        .window("Smart Road", renderer::WINDOW_W, renderer::WINDOW_H)
        .position_centered()
        .build()
        .expect("window creation failed");

    let canvas = window
        .into_canvas()
        .accelerated()
        .present_vsync()
        .build()
        .expect("canvas creation failed");

    let texture_creator = canvas.texture_creator();
    let mut renderer = Renderer::new(canvas, &texture_creator);
    let mut event_pump = sdl.event_pump().expect("event pump failed");

    let ttf = sdl2::ttf::init().expect("SDL2 TTF init failed");
    let font_paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    ];
    let font = font_paths
        .iter()
        .find_map(|p| ttf.load_font(p, 20).ok())
        .expect("no usable font found; install fonts-dejavu-core");

    let mut rng = thread_rng();
    let mut input = InputHandler::new();
    let mut manager = IntersectionManager::new();
    let mut stats = StatsAccumulator::new();
    let mut vehicles: Vec<vehicle::Vehicle> = Vec::new();
    let mut tick: u64 = 0;
    let mut sim_state = SimState::Running;

    'running: loop {
        // ── input ─────────────────────────────────────────────────────────────
        for event in event_pump.poll_iter() {
            match &event {
                Event::Quit { .. } => break 'running,
                Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => match sim_state {
                    SimState::Running => sim_state = SimState::ShowingStats,
                    SimState::ShowingStats => break 'running,
                },
                _ => {
                    if sim_state == SimState::Running
                        && !input.handle_event(&event, &mut vehicles, &mut rng)
                    {
                        break 'running;
                    }
                }
            }
        }

        if sim_state == SimState::Running {
            input.tick(&mut vehicles, &mut rng);

            // ── simulation update ─────────────────────────────────────────────
            let mut exits: Vec<u64> = Vec::new();

            // Per-lane sorted groups for O(n) leader lookup — built once per tick.
            let groups = build_lane_groups(&vehicles);

            manager.cleanup_expired(tick);

            for v in vehicles.iter_mut() {
                stats.record_speed(v.current_speed);

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
                            let gap =
                                lane_leader_gap(v.id, v.direction, v.route, v.x, v.y, &groups);
                            let dist = intersection::dist_to_stop_line(v.direction, v.x, v.y);

                            // 1. Desired target speed from AIM or free-flow following.
                            if dist <= intersection::RESERVATION_DIST {
                                match manager.try_reserve_timed(
                                    v.id,
                                    v.direction,
                                    v.route,
                                    tick,
                                    dist,
                                ) {
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
                            v.crossing_path =
                                intersection::crossing_waypoints(v.direction, v.route);
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

            // ── close-call detection ──────────────────────────────────────────
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
            let in_zone: Vec<&vehicle::Vehicle> = vehicles
                .iter()
                .filter(|v| matches!(v.state, VehicleState::Crossing | VehicleState::Exiting))
                .collect();
            for i in 0..in_zone.len() {
                for j in (i + 1)..in_zone.len() {
                    let (a, b) = (in_zone[i], in_zone[j]);
                    let gap = vehicle::physics::distance(a.x, a.y, b.x, b.y);
                    stats.record_gap(gap);
                    if vehicle::physics::is_close_call(a.x, a.y, b.x, b.y) {
                        violations.insert((a.id.min(b.id), a.id.max(b.id)));
                    }
                }
            }

            stats.update_violations(violations);

            vehicles.retain(|v| v.state != VehicleState::Done);

            tick += 1;
        }

        // ── render ────────────────────────────────────────────────────────────
        renderer.clear();
        renderer.draw_road();
        renderer.draw_vehicles(&vehicles);

        if sim_state == SimState::Running {
            renderer.draw_hud(&font, vehicles.len(), stats.close_calls);
        } else {
            renderer.draw_stats_overlay(&font, &stats);
        }

        renderer.present();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vehicle::physics::MIN_FOLLOWING_GAP;
    use vehicle::{Direction, Route, Speed, VehicleState};

    fn veh(id: u32, dir: Direction, route: Route, x: f32, y: f32) -> vehicle::Vehicle {
        vehicle::Vehicle::new(id, x, y, dir, route)
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
    fn build_lane_groups_excludes_crossing_vehicles() {
        let mut a = veh(0, Direction::North, Route::Straight, 400.0, 700.0);
        a.state = VehicleState::Crossing;
        let groups = build_lane_groups(&[a]);
        assert!(!groups.contains_key(&(Direction::North, Route::Straight)));
    }

    #[test]
    fn build_lane_groups_includes_waiting_vehicles() {
        let mut a = veh(0, Direction::North, Route::Straight, 400.0, 482.0);
        a.state = VehicleState::Waiting;
        let groups = build_lane_groups(&[a]);
        assert_eq!(groups[&(Direction::North, Route::Straight)].len(), 1);
    }

    // ── lane_leader_gap ───────────────────────────────────────────────────────

    #[test]
    fn lane_leader_gap_max_when_no_vehicles() {
        let groups = build_lane_groups(&[]);
        let gap = lane_leader_gap(0, Direction::North, Route::Straight, 400.0, 700.0, &groups);
        assert_eq!(gap, f32::MAX);
    }

    #[test]
    fn lane_leader_gap_max_for_different_route() {
        // Vehicle in N/Right lane — ego asking about N/Straight → different group.
        let a = veh(1, Direction::North, Route::Right, 430.0, 600.0);
        let groups = build_lane_groups(&[a]);
        let gap = lane_leader_gap(0, Direction::North, Route::Straight, 400.0, 700.0, &groups);
        assert_eq!(gap, f32::MAX);
    }

    #[test]
    fn lane_leader_gap_ignores_self() {
        let a = veh(0, Direction::North, Route::Straight, 400.0, 600.0);
        let groups = build_lane_groups(&[a]);
        let gap = lane_leader_gap(0, Direction::North, Route::Straight, 400.0, 700.0, &groups);
        assert_eq!(gap, f32::MAX);
    }

    #[test]
    fn lane_leader_gap_ignores_vehicle_behind() {
        // For North (axial = -y), y=800 → pos=-800 is behind y=700 → pos=-700.
        let a = veh(1, Direction::North, Route::Straight, 400.0, 800.0);
        let groups = build_lane_groups(&[a]);
        let gap = lane_leader_gap(0, Direction::North, Route::Straight, 400.0, 700.0, &groups);
        assert_eq!(gap, f32::MAX);
    }

    #[test]
    fn lane_leader_gap_ignores_crossing_vehicles() {
        let mut a = veh(1, Direction::North, Route::Straight, 400.0, 600.0);
        a.state = VehicleState::Crossing;
        let groups = build_lane_groups(&[a]);
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
    fn max_follow_speed_zero_when_inside_min_gap() {
        let speed = max_follow_speed(MIN_FOLLOWING_GAP - 10.0);
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
}
