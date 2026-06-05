mod input;
mod intersection;
mod renderer;
mod statistics;
mod vehicle;

use std::collections::HashSet;

use rand::thread_rng;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;

use input::InputHandler;
use intersection::IntersectionManager;
use renderer::Renderer;
use statistics::StatsAccumulator;
use vehicle::physics::{MIN_FOLLOWING_GAP, SMOOTH_ALPHA};
use vehicle::{Direction, Speed, VehicleState, direction_to_angle};

/// Center-to-centre axial gap to the nearest vehicle ahead in the same approach lane.
/// Returns f32::MAX when no leader exists.
fn min_leader_gap_in_lane(
    id: u32,
    dir: Direction,
    x: f32,
    y: f32,
    snapshot: &[(u32, Direction, f32, f32, VehicleState)],
) -> f32 {
    snapshot
        .iter()
        .filter(|&&(oid, odir, ox, oy, ostate)| {
            if oid == id || odir != dir {
                return false;
            }
            if matches!(
                ostate,
                VehicleState::Crossing | VehicleState::Exiting | VehicleState::Done
            ) {
                return false;
            }
            let ahead = match dir {
                Direction::North => oy < y,
                Direction::South => oy > y,
                Direction::East => ox > x,
                Direction::West => ox < x,
            };
            let transverse = match dir {
                Direction::North | Direction::South => (ox - x).abs(),
                Direction::East | Direction::West => (oy - y).abs(),
            };
            ahead && transverse <= 20.0
        })
        .map(|&(_, _, ox, oy, _)| match dir {
            Direction::North | Direction::South => (oy - y).abs(),
            Direction::East | Direction::West => (ox - x).abs(),
        })
        .fold(f32::MAX, f32::min)
}

/// Maximum safe target speed given a centre-to-centre leader gap.
/// v_max = SMOOTH_ALPHA × clearance ensures the vehicle cannot overshoot MIN_FOLLOWING_GAP.
fn max_follow_speed(gap: f32) -> f32 {
    let clearance = gap - MIN_FOLLOWING_GAP;
    (clearance * SMOOTH_ALPHA).clamp(0.0, Speed::FAST_PX)
}

/// True if two Approaching/Waiting vehicles in the same lane are inside the safe following gap.
fn is_following_violation(a: &vehicle::Vehicle, b: &vehicle::Vehicle) -> bool {
    if a.direction != b.direction {
        return false;
    }
    if matches!(
        a.state,
        VehicleState::Crossing | VehicleState::Exiting | VehicleState::Done
    ) {
        return false;
    }
    if matches!(
        b.state,
        VehicleState::Crossing | VehicleState::Exiting | VehicleState::Done
    ) {
        return false;
    }
    let transverse = match a.direction {
        Direction::North | Direction::South => (a.x - b.x).abs(),
        Direction::East | Direction::West => (a.y - b.y).abs(),
    };
    if transverse > 20.0 {
        return false;
    }
    let axial = match a.direction {
        Direction::North | Direction::South => (a.y - b.y).abs(),
        Direction::East | Direction::West => (a.x - b.x).abs(),
    };
    axial < MIN_FOLLOWING_GAP
}

/// True if the nearest leader in the same lane is within the minimum following gap.
fn is_blocked_ahead(
    id: u32,
    dir: Direction,
    x: f32,
    y: f32,
    snapshot: &[(u32, Direction, f32, f32, VehicleState)],
) -> bool {
    min_leader_gap_in_lane(id, dir, x, y, snapshot) < MIN_FOLLOWING_GAP
}

/// Target speed for an approaching vehicle outside the reservation zone.
/// Fast > 200 px from stop line or leader, Normal 80–200 px, Slow < 80 px.
fn approach_target_speed(
    id: u32,
    dir: Direction,
    x: f32,
    y: f32,
    snapshot: &[(u32, Direction, f32, f32, VehicleState)],
) -> f32 {
    let dist_stop = intersection::dist_to_stop_line(dir, x, y);
    let gap = min_leader_gap_in_lane(id, dir, x, y, snapshot);
    let constraint = dist_stop.min(gap);
    if constraint > 200.0 {
        Speed::FAST_PX
    } else if constraint > 80.0 {
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

            // Snapshot used for same-lane following-distance checks.
            let snapshot: Vec<(u32, Direction, f32, f32, VehicleState)> = vehicles
                .iter()
                .map(|v| (v.id, v.direction, v.x, v.y, v.state))
                .collect();

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
                            // 1. Compute desired target speed from AIM or leader-following.
                            let dist = intersection::dist_to_stop_line(v.direction, v.x, v.y);
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
                                v.target_speed =
                                    approach_target_speed(v.id, v.direction, v.x, v.y, &snapshot);
                            }

                            // 2. Cap by safe following speed — overrides AIM when a leader
                            //    is close. Caps current_speed too so smooth_speed can't
                            //    carry excess momentum into this tick's advance().
                            let follow_cap = max_follow_speed(min_leader_gap_in_lane(
                                v.id,
                                v.direction,
                                v.x,
                                v.y,
                                &snapshot,
                            ));
                            v.target_speed = v.target_speed.min(follow_cap);
                            v.current_speed = v.current_speed.min(follow_cap);

                            // 3. Interpolate toward target, then advance.
                            v.smooth_speed();
                            if !is_blocked_ahead(v.id, v.direction, v.x, v.y, &snapshot) {
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
                        if v.x < -80.0 || v.x > w + 80.0 || v.y < -80.0 || v.y > h + 80.0 {
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
            for i in 0..vehicles.len() {
                for j in (i + 1)..vehicles.len() {
                    let (a, b) = (&vehicles[i], &vehicles[j]);
                    let gap = vehicle::physics::distance(a.x, a.y, b.x, b.y);
                    stats.record_gap(gap);
                    if vehicle::physics::is_close_call(a.x, a.y, b.x, b.y)
                        || is_following_violation(a, b)
                    {
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

        if sim_state == SimState::ShowingStats {
            renderer.draw_stats_overlay(&font, &stats);
        }

        renderer.present();
    }
}
