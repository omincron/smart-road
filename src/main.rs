mod intersection;
mod vehicle;
mod renderer;
mod input;
mod statistics;

use std::collections::HashSet;

use rand::thread_rng;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;

use input::InputHandler;
use intersection::IntersectionManager;
use renderer::Renderer;
use statistics::StatsAccumulator;
use vehicle::{VehicleState, direction_to_angle};

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

    let mut renderer = Renderer::new(canvas);
    let mut event_pump = sdl.event_pump().expect("event pump failed");
    let mut rng = thread_rng();
    let mut input = InputHandler::new();
    let mut manager = IntersectionManager::new();
    let mut stats = StatsAccumulator::new();
    let mut vehicles: Vec<vehicle::Vehicle> = Vec::new();
    let mut tick: u64 = 0;
    let mut sim_state = SimState::Running;
    let mut stats_printed = false;

    'running: loop {
        // ── input ─────────────────────────────────────────────────────────────
        for event in event_pump.poll_iter() {
            match &event {
                Event::Quit { .. } => break 'running,
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => match sim_state {
                    SimState::Running => sim_state = SimState::ShowingStats,
                    SimState::ShowingStats => break 'running,
                },
                _ => {
                    if sim_state == SimState::Running {
                        if !input.handle_event(&event, &mut vehicles, &mut rng) {
                            break 'running;
                        }
                    }
                }
            }
        }

        if sim_state == SimState::Running {
            input.tick(&mut vehicles, &mut rng);

            // ── simulation update ─────────────────────────────────────────────
            let mut exits: Vec<(u64, f32)> = Vec::new(); // (transit_ticks, speed)

            for v in vehicles.iter_mut() {
                stats.record_speed(v.speed.pixels_per_tick());

                match v.state {
                    VehicleState::Approaching => {
                        if intersection::at_stop_line(v.direction, v.x, v.y) {
                            if manager.try_reserve(v.id, v.direction, v.route) {
                                v.state = VehicleState::Crossing;
                                v.detection_tick = tick;
                                v.crossing_path =
                                    intersection::crossing_waypoints(v.direction, v.route);
                                v.waypoint_idx = 0;
                            } else {
                                v.state = VehicleState::Waiting;
                            }
                        } else {
                            v.advance();
                        }
                    }
                    VehicleState::Waiting => {
                        if manager.try_reserve(v.id, v.direction, v.route) {
                            v.state = VehicleState::Crossing;
                            v.detection_tick = tick;
                            v.crossing_path =
                                intersection::crossing_waypoints(v.direction, v.route);
                            v.waypoint_idx = 0;
                        }
                    }
                    VehicleState::Crossing => {
                        if v.advance_crossing() {
                            let exit_dir = intersection::exit_direction(v.direction, v.route);
                            v.direction = exit_dir;
                            v.angle_deg = direction_to_angle(exit_dir);
                            v.state = VehicleState::Exiting;
                            v.exit_tick = Some(tick);
                            manager.release(v.id);
                            if let Some(transit) = v.transit_ticks() {
                                exits.push((transit, v.speed.pixels_per_tick()));
                            }
                        }
                    }
                    VehicleState::Exiting => {
                        v.advance();
                    }
                    VehicleState::Done => {}
                }
            }

            for (transit, _speed) in exits {
                stats.record_exit(transit);
            }

            // ── close-call detection ──────────────────────────────────────────
            let mut violations: HashSet<(u32, u32)> = HashSet::new();
            for i in 0..vehicles.len() {
                for j in (i + 1)..vehicles.len() {
                    let (a, b) = (&vehicles[i], &vehicles[j]);
                    if vehicle::physics::is_close_call(a.x, a.y, b.x, b.y) {
                        let pair = (a.id.min(b.id), a.id.max(b.id));
                        violations.insert(pair);
                    }
                }
            }
            stats.update_violations(violations);

            // Drop vehicles that have left the screen
            vehicles.retain(|v| {
                let w = renderer::WINDOW_W as f32;
                let h = renderer::WINDOW_H as f32;
                v.x > -80.0 && v.x < w + 80.0 && v.y > -80.0 && v.y < h + 80.0
            });

            tick += 1;
        }

        // ── render ────────────────────────────────────────────────────────────
        renderer.clear();
        renderer.draw_road();
        renderer.draw_vehicles(&vehicles);

        if sim_state == SimState::ShowingStats {
            renderer.draw_stats_overlay();
            if !stats_printed {
                stats.print_to_terminal();
                stats_printed = true;
            }
        }

        renderer.present();
    }
}
