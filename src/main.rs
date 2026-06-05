mod input;
mod intersection;
mod renderer;
mod simulation;
mod statistics;
mod vehicle;

use rand::thread_rng;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;

use input::InputHandler;
use intersection::IntersectionManager;
use renderer::Renderer;
use simulation::step_simulation;
use statistics::StatsAccumulator;

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
            // Core vehicle/state updates live in `simulation::step_simulation`;
            // this entrypoint only handles SDL input and rendering.
            step_simulation(tick, &mut vehicles, &mut manager, &mut stats);
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
