mod intersection;
mod vehicle;
mod renderer;
mod input;
mod statistics;

use rand::thread_rng;

use input::InputHandler;
use renderer::Renderer;

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
    let mut vehicles: Vec<vehicle::Vehicle> = Vec::new();

    'running: loop {
        for event in event_pump.poll_iter() {
            if !input.handle_event(&event, &mut vehicles, &mut rng) {
                break 'running;
            }
        }

        input.tick(&mut vehicles, &mut rng);

        for v in vehicles.iter_mut() {
            v.advance();
        }

        // Drop vehicles that have left the screen entirely
        vehicles.retain(|v| {
            let w = renderer::WINDOW_W as f32;
            let h = renderer::WINDOW_H as f32;
            v.x > -60.0 && v.x < w + 60.0 && v.y > -60.0 && v.y < h + 60.0
        });

        renderer.clear();
        renderer.draw_road();
        renderer.draw_vehicles(&vehicles);
        renderer.present();
    }
}
