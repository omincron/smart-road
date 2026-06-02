mod intersection;
mod vehicle;
mod renderer;
mod input;
mod statistics;

use rand::thread_rng;

use input::InputHandler;
use intersection::IntersectionManager;
use renderer::Renderer;
use vehicle::{VehicleState, direction_to_angle};

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
    let mut vehicles: Vec<vehicle::Vehicle> = Vec::new();
    let mut tick: u64 = 0;

    'running: loop {
        // ── input ─────────────────────────────────────────────────────────────
        for event in event_pump.poll_iter() {
            if !input.handle_event(&event, &mut vehicles, &mut rng) {
                break 'running;
            }
        }
        input.tick(&mut vehicles, &mut rng);

        // ── simulation update ─────────────────────────────────────────────────
        for v in vehicles.iter_mut() {
            match v.state {
                VehicleState::Approaching => {
                    if intersection::at_stop_line(v.direction, v.x, v.y) {
                        if manager.try_reserve(v.id, v.direction, v.route) {
                            v.state = VehicleState::Crossing;
                            v.detection_tick = tick;
                            v.crossing_path = intersection::crossing_waypoints(v.direction, v.route);
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
                        v.crossing_path = intersection::crossing_waypoints(v.direction, v.route);
                        v.waypoint_idx = 0;
                    }
                    // vehicle holds position while waiting
                }
                VehicleState::Crossing => {
                    if v.advance_crossing() {
                        let exit_dir = intersection::exit_direction(v.direction, v.route);
                        v.direction = exit_dir;
                        v.angle_deg = direction_to_angle(exit_dir);
                        v.state = VehicleState::Exiting;
                        v.exit_tick = Some(tick);
                        manager.release(v.id);
                    }
                }
                VehicleState::Exiting => {
                    v.advance();
                }
                VehicleState::Done => {}
            }
        }

        // Drop vehicles that have left the screen
        vehicles.retain(|v| {
            let w = renderer::WINDOW_W as f32;
            let h = renderer::WINDOW_H as f32;
            v.x > -80.0 && v.x < w + 80.0 && v.y > -80.0 && v.y < h + 80.0
        });

        tick += 1;

        // ── render ────────────────────────────────────────────────────────────
        renderer.clear();
        renderer.draw_road();
        renderer.draw_vehicles(&vehicles);
        renderer.present();
    }
}
