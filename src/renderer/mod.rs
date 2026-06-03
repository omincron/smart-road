use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::surface::Surface;
use sdl2::video::{Window, WindowContext};

use crate::statistics::StatsAccumulator;
use crate::vehicle::Vehicle;

pub const WINDOW_W: u32 = 800;
pub const WINDOW_H: u32 = 800;
pub const LANE_W: i32 = 40;

pub const CENTER_X: i32 = WINDOW_W as i32 / 2; // 400
pub const CENTER_Y: i32 = WINDOW_H as i32 / 2; // 400
pub const ROAD_W: i32 = LANE_W * 3; // 120px per direction (3 lanes)

// ── colours ──────────────────────────────────────────────────────────────────
const C_BG: Color = Color { r: 34, g: 85, b: 34, a: 255 };    // grass
const C_ROAD: Color = Color { r: 50, g: 50, b: 50, a: 255 };  // asphalt
const C_MARK: Color = Color { r: 160, g: 160, b: 160, a: 255 }; // lane dashes / dividers
const C_STOP: Color = Color { r: 255, g: 255, b: 255, a: 255 }; // stop lines


// Rendered size of a vehicle sprite (portrait — car faces North by default).
const CAR_W: u32 = 24;
const CAR_H: u32 = 34;

pub struct Renderer {
    pub canvas: Canvas<Window>,
    texture_creator: TextureCreator<WindowContext>,
    // SAFETY: car_texture is declared after texture_creator so it is dropped first,
    // satisfying SDL2's requirement that textures are destroyed before their creator.
    car_texture: Texture<'static>,
}

impl Renderer {
    pub fn new(canvas: Canvas<Window>) -> Self {
        let texture_creator = canvas.texture_creator();
        let car_texture = load_car_texture(&texture_creator);
        Renderer { canvas, texture_creator, car_texture }
    }

    fn draw_text(&mut self, font: &sdl2::ttf::Font, text: &str, x: i32, y: i32, color: Color) {
        if text.is_empty() { return; }
        let Ok(surface) = font.render(text).blended(color) else { return };
        let Ok(texture) = self.texture_creator.create_texture_from_surface(&surface) else { return };
        let q = texture.query();
        let _ = self.canvas.copy(&texture, None, Some(Rect::new(x, y, q.width, q.height)));
    }

    pub fn clear(&mut self) {
        self.canvas.set_draw_color(C_BG);
        self.canvas.clear();
    }

    pub fn draw_road(&mut self) {
        self.canvas.set_draw_color(C_ROAD);

        // vertical strip (full height)
        self.canvas
            .fill_rect(Rect::new(CENTER_X - ROAD_W, 0, (ROAD_W * 2) as u32, WINDOW_H))
            .unwrap();

        // horizontal strip (full width)
        self.canvas
            .fill_rect(Rect::new(0, CENTER_Y - ROAD_W, WINDOW_W, (ROAD_W * 2) as u32))
            .unwrap();

        self.draw_centre_dividers();
        self.draw_lane_dashes();
        self.draw_stop_lines();
    }

    /// Solid centre lines separating opposing traffic on each road arm.
    fn draw_centre_dividers(&mut self) {
        self.canvas.set_draw_color(C_MARK);
        const T: i32 = 3;

        // vertical road — above and below the intersection box
        self.canvas
            .fill_rect(Rect::new(CENTER_X - T / 2, 0, T as u32, (CENTER_Y - ROAD_W) as u32))
            .unwrap();
        self.canvas
            .fill_rect(Rect::new(
                CENTER_X - T / 2,
                CENTER_Y + ROAD_W,
                T as u32,
                (WINDOW_H as i32 - CENTER_Y - ROAD_W) as u32,
            ))
            .unwrap();

        // horizontal road — left and right of the intersection box
        self.canvas
            .fill_rect(Rect::new(0, CENTER_Y - T / 2, (CENTER_X - ROAD_W) as u32, T as u32))
            .unwrap();
        self.canvas
            .fill_rect(Rect::new(
                CENTER_X + ROAD_W,
                CENTER_Y - T / 2,
                (WINDOW_W as i32 - CENTER_X - ROAD_W) as u32,
                T as u32,
            ))
            .unwrap();
    }

    /// Dashed lane dividers inside each road arm, outside the intersection box.
    fn draw_lane_dashes(&mut self) {
        self.canvas.set_draw_color(C_MARK);

        for i in 1..3_i32 {
            // southbound lanes (left half of vertical road, x: 280–400)
            let x = CENTER_X - ROAD_W + i * LANE_W;
            self.dashes_v(x, 0, CENTER_Y - ROAD_W);
            self.dashes_v(x, CENTER_Y + ROAD_W, WINDOW_H as i32);

            // northbound lanes (right half, x: 400–520)
            let x = CENTER_X + i * LANE_W;
            self.dashes_v(x, 0, CENTER_Y - ROAD_W);
            self.dashes_v(x, CENTER_Y + ROAD_W, WINDOW_H as i32);

            // westbound lanes (top half of horizontal road, y: 280–400)
            let y = CENTER_Y - ROAD_W + i * LANE_W;
            self.dashes_h(0, CENTER_X - ROAD_W, y);
            self.dashes_h(CENTER_X + ROAD_W, WINDOW_W as i32, y);

            // eastbound lanes (bottom half, y: 400–520)
            let y = CENTER_Y + i * LANE_W;
            self.dashes_h(0, CENTER_X - ROAD_W, y);
            self.dashes_h(CENTER_X + ROAD_W, WINDOW_W as i32, y);
        }
    }

    fn dashes_v(&mut self, x: i32, y0: i32, y1: i32) {
        const DASH: i32 = 12;
        const GAP: i32 = 10;
        let mut y = y0;
        while y < y1 {
            let h = DASH.min(y1 - y) as u32;
            self.canvas.fill_rect(Rect::new(x - 1, y, 2, h)).unwrap();
            y += DASH + GAP;
        }
    }

    fn dashes_h(&mut self, x0: i32, x1: i32, y: i32) {
        const DASH: i32 = 12;
        const GAP: i32 = 10;
        let mut x = x0;
        while x < x1 {
            let w = DASH.min(x1 - x) as u32;
            self.canvas.fill_rect(Rect::new(x, y - 1, w, 2)).unwrap();
            x += DASH + GAP;
        }
    }

    /// White stop lines at the entry edge of the intersection for each direction.
    fn draw_stop_lines(&mut self) {
        self.canvas.set_draw_color(C_STOP);
        const T: u32 = 3;

        // southbound — top edge, left half (x: 280–400)
        self.canvas
            .fill_rect(Rect::new(CENTER_X - ROAD_W, CENTER_Y - ROAD_W - T as i32, ROAD_W as u32, T))
            .unwrap();
        // northbound — bottom edge, right half (x: 400–520)
        self.canvas
            .fill_rect(Rect::new(CENTER_X, CENTER_Y + ROAD_W, ROAD_W as u32, T))
            .unwrap();
        // westbound — right edge, top half (y: 280–400)
        self.canvas
            .fill_rect(Rect::new(CENTER_X + ROAD_W, CENTER_Y - ROAD_W, T, ROAD_W as u32))
            .unwrap();
        // eastbound — left edge, bottom half (y: 400–520)
        self.canvas
            .fill_rect(Rect::new(CENTER_X - ROAD_W - T as i32, CENTER_Y, T, ROAD_W as u32))
            .unwrap();
    }

    pub fn draw_vehicles(&mut self, vehicles: &[Vehicle]) {
        for v in vehicles {
            self.draw_vehicle(v);
        }
    }

    fn draw_vehicle(&mut self, v: &Vehicle) {
        let dest = Rect::new(
            v.x as i32 - CAR_W as i32 / 2,
            v.y as i32 - CAR_H as i32 / 2,
            CAR_W, CAR_H,
        );
        // Sprite already faces North (0°), so angle_deg maps directly to copy_ex.
        let angle = v.angle_deg as f64;
        self.canvas.copy_ex(&self.car_texture, None, Some(dest), angle, None, false, false).unwrap();
    }

    /// Overlay shown when the simulation ends — renders the stats panel with text.
    pub fn draw_stats_overlay(&mut self, font: &sdl2::ttf::Font, stats: &StatsAccumulator) {
        const LINE_H: i32 = 28;
        const PAD: i32 = 20;

        let lines = stats.stat_lines();
        let pw: i32 = 380;
        let ph: i32 = PAD + lines.len() as i32 * LINE_H + PAD;
        let px = (WINDOW_W as i32 - pw) / 2;
        let py = (WINDOW_H as i32 - ph) / 2;

        self.canvas.set_blend_mode(sdl2::render::BlendMode::Blend);

        // Dim background
        self.canvas.set_draw_color(Color::RGBA(0, 0, 0, 170));
        self.canvas.fill_rect(Rect::new(0, 0, WINDOW_W, WINDOW_H)).unwrap();

        // Panel background
        self.canvas.set_draw_color(Color::RGBA(15, 15, 35, 235));
        self.canvas.fill_rect(Rect::new(px, py, pw as u32, ph as u32)).unwrap();

        // Panel border
        self.canvas.set_draw_color(Color::RGB(140, 160, 220));
        for t in 0..2_i32 {
            self.canvas
                .draw_rect(Rect::new(px - t, py - t, (pw + 2 * t) as u32, (ph + 2 * t) as u32))
                .unwrap();
        }

        self.canvas.set_blend_mode(sdl2::render::BlendMode::None);

        // Text lines
        for (i, line) in lines.iter().enumerate() {
            let color = if i == 0 {
                Color::RGB(200, 220, 255) // title highlight
            } else if i == lines.len() - 1 {
                Color::RGB(140, 140, 160) // footer hint
            } else {
                Color::WHITE
            };
            self.draw_text(font, line, px, py + PAD + i as i32 * LINE_H, color);
        }
    }

    pub fn present(&mut self) {
        self.canvas.present();
    }
}

/// Decode the car PNG, strip the solid background via flood-fill, and upload as an SDL2 texture.
/// Returns a Texture with a transmuted 'static lifetime — safe because the caller
/// stores it after the TextureCreator in the same struct (dropped first).
fn load_car_texture(tc: &TextureCreator<WindowContext>) -> Texture<'static> {
    let mut img = image::open("assets/car.png")
        .expect("failed to open car sprite")
        .into_rgba8();

    strip_background(&mut img);

    let (w, h) = img.dimensions();
    let mut pixels = img.into_raw();

    // RGBA32 is the endian-aware alias: ABGR8888 on little-endian, RGBA8888 on big-endian.
    // The `image` crate gives RGBA bytes in memory order, which matches RGBA32.
    let surface = Surface::from_data(&mut pixels, w, h, w * 4, PixelFormatEnum::RGBA32)
        .expect("failed to create surface from car pixels");

    let mut texture = tc.create_texture_from_surface(&surface)
        .expect("failed to upload car texture to GPU");
    texture.set_blend_mode(BlendMode::Blend);

    // SAFETY: texture_creator outlives car_texture (see Renderer field declaration order).
    unsafe { std::mem::transmute(texture) }
}

/// BFS flood-fill from all four corners, making every connected near-background pixel
/// fully transparent. This removes the solid white/grey canvas without touching the car body.
fn strip_background(img: &mut image::RgbaImage) {
    let (w, h) = img.dimensions();
    let bg = {
        let p = img.get_pixel(0, 0);
        [p[0], p[1], p[2]]
    };
    const TOL: i32 = 35;

    let matches = |p: &image::Rgba<u8>| -> bool {
        (0..3).all(|i| (p[i] as i32 - bg[i] as i32).abs() <= TOL)
    };

    let mut visited = vec![false; (w * h) as usize];
    let mut queue = std::collections::VecDeque::new();

    for &(cx, cy) in &[(0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)] {
        let idx = (cy * w + cx) as usize;
        if !visited[idx] {
            visited[idx] = true;
            queue.push_back((cx, cy));
        }
    }

    while let Some((x, y)) = queue.pop_front() {
        if !matches(img.get_pixel(x, y)) {
            continue;
        }
        img.put_pixel(x, y, image::Rgba([0, 0, 0, 0]));

        for (nx, ny) in [
            (x.wrapping_sub(1), y), (x + 1, y),
            (x, y.wrapping_sub(1)), (x, y + 1),
        ] {
            if nx < w && ny < h {
                let nidx = (ny * w + nx) as usize;
                if !visited[nidx] {
                    visited[nidx] = true;
                    queue.push_back((nx, ny));
                }
            }
        }
    }
}
