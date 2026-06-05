use std::collections::HashMap;

use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::surface::Surface;
use sdl2::video::{Window, WindowContext};

use crate::statistics::StatsAccumulator;
use crate::vehicle::Vehicle;

pub const WINDOW_W: u32 = 800;
pub const WINDOW_H: u32 = 800;

pub const CENTER_X: i32 = WINDOW_W as i32 / 2; // 400
pub const CENTER_Y: i32 = WINDOW_H as i32 / 2; // 400
pub const ROAD_W: i32 = 82; // half-width of road from center — matched to cross-road.png
pub const LANE_W: i32 = ROAD_W / 3; // 27px per lane

// ── colours ──────────────────────────────────────────────────────────────────
const C_STOP: Color = Color {
    r: 255,
    g: 255,
    b: 255,
    a: 255,
}; // stop lines

// Rendered size of a vehicle sprite (portrait — car faces North by default).
const CAR_W: u32 = 24;
const CAR_H: u32 = 34;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct TextKey {
    text: String,
    color: [u8; 4],
}

fn text_key(text: &str, color: Color) -> TextKey {
    TextKey {
        text: text.to_owned(),
        color: [color.r, color.g, color.b, color.a],
    }
}

struct TextCache<T> {
    entries: HashMap<TextKey, T>,
}

impl<T> TextCache<T> {
    fn new() -> Self {
        Self {
            entries: HashMap::new(),
        }
    }

    fn get_or_insert_with<F>(&mut self, key: TextKey, build: F) -> &T
    where
        F: FnOnce() -> T,
    {
        self.entries.entry(key).or_insert_with(build)
    }
}

pub struct Renderer<'tc> {
    pub canvas: Canvas<Window>,
    tc: &'tc TextureCreator<WindowContext>,
    road_texture: Texture<'tc>,
    car_texture: Texture<'tc>,
    text_cache: TextCache<Texture<'tc>>,
}

impl<'tc> Renderer<'tc> {
    pub fn new(canvas: Canvas<Window>, tc: &'tc TextureCreator<WindowContext>) -> Self {
        let road_texture = load_rgb_texture(tc, "assets/cross-road.png");
        let car_texture = load_car_texture(tc);
        Renderer {
            canvas,
            tc,
            road_texture,
            car_texture,
            text_cache: TextCache::new(),
        }
    }

    fn draw_text(&mut self, font: &sdl2::ttf::Font, text: &str, x: i32, y: i32, color: Color) {
        if text.is_empty() {
            return;
        }
        let key = text_key(text, color);
        if !self.text_cache.entries.contains_key(&key) {
            let Ok(surface) = font.render(text).blended(color) else {
                return;
            };
            let Ok(texture) = self.tc.create_texture_from_surface(&surface) else {
                return;
            };
            self.text_cache.entries.insert(key.clone(), texture);
        }
        let Some(texture) = self.text_cache.entries.get(&key) else {
            return;
        };
        let q = texture.query();
        let _ = self
            .canvas
            .copy(&texture, None, Some(Rect::new(x, y, q.width, q.height)));
    }

    pub fn clear(&mut self) {
        self.canvas.set_draw_color(Color::BLACK);
        self.canvas.clear();
    }

    pub fn draw_road(&mut self) {
        // Blit the cross-road PNG scaled to fill the window.
        let dest = Rect::new(0, 0, WINDOW_W, WINDOW_H);
        let _ = self.canvas.copy(&self.road_texture, None, Some(dest));

        // Draw stop lines on top so they match the simulation's lane geometry.
        self.draw_stop_lines();
    }

    /// White stop lines at the entry edge of the intersection for each direction.
    fn draw_stop_lines(&mut self) {
        self.canvas.set_draw_color(C_STOP);
        const T: u32 = 3;

        // southbound — top edge, left half (x: 280–400)
        let _ = self.canvas.fill_rect(Rect::new(
            CENTER_X - ROAD_W,
            CENTER_Y - ROAD_W - T as i32,
            ROAD_W as u32,
            T,
        ));
        // northbound — bottom edge, right half (x: 400–520)
        let _ = self
            .canvas
            .fill_rect(Rect::new(CENTER_X, CENTER_Y + ROAD_W, ROAD_W as u32, T));
        // westbound — right edge, top half (y: 280–400)
        let _ = self.canvas.fill_rect(Rect::new(
            CENTER_X + ROAD_W,
            CENTER_Y - ROAD_W,
            T,
            ROAD_W as u32,
        ));
        // eastbound — left edge, bottom half (y: 400–520)
        let _ = self.canvas.fill_rect(Rect::new(
            CENTER_X - ROAD_W - T as i32,
            CENTER_Y,
            T,
            ROAD_W as u32,
        ));
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
            CAR_W,
            CAR_H,
        );
        // Sprite already faces North (0°), so angle_deg maps directly to copy_ex.
        let angle = v.angle_deg as f64;
        let _ = self.canvas.copy_ex(
            &self.car_texture,
            None,
            Some(dest),
            angle,
            None,
            false,
            false,
        );
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
        let _ = self.canvas.fill_rect(Rect::new(0, 0, WINDOW_W, WINDOW_H));

        // Panel background
        self.canvas.set_draw_color(Color::RGBA(15, 15, 35, 235));
        let _ = self
            .canvas
            .fill_rect(Rect::new(px, py, pw as u32, ph as u32));

        // Panel border
        self.canvas.set_draw_color(Color::RGB(140, 160, 220));
        for t in 0..2_i32 {
            let _ = self.canvas.draw_rect(Rect::new(
                px - t,
                py - t,
                (pw + 2 * t) as u32,
                (ph + 2 * t) as u32,
            ));
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

    /// Small top-left HUD showing live vehicle count and accumulated close-call count.
    pub fn draw_hud(&mut self, font: &sdl2::ttf::Font, vehicle_count: usize, close_calls: u32) {
        let text = format!("  Vehicles: {vehicle_count}   Close calls: {close_calls}  ");
        self.draw_text(font, &text, 8, 8, Color::RGB(200, 230, 200));
    }

    pub fn present(&mut self) {
        self.canvas.present();
    }
}

/// Decode an RGB(A) PNG and upload it as an SDL2 texture (no background stripping).
fn load_rgb_texture<'tc>(tc: &'tc TextureCreator<WindowContext>, path: &str) -> Texture<'tc> {
    let img = image::open(path)
        .unwrap_or_else(|_| panic!("failed to open {path}"))
        .into_rgba8();
    let (w, h) = img.dimensions();
    let mut pixels = img.into_raw();
    let surface = Surface::from_data(&mut pixels, w, h, w * 4, PixelFormatEnum::RGBA32)
        .expect("failed to create surface");
    tc.create_texture_from_surface(&surface)
        .expect("failed to upload texture to GPU")
}

/// Decode the car PNG, strip the solid background via flood-fill, and upload as an SDL2 texture.
fn load_car_texture<'tc>(tc: &'tc TextureCreator<WindowContext>) -> Texture<'tc> {
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

    let mut texture = tc
        .create_texture_from_surface(&surface)
        .expect("failed to upload car texture to GPU");
    texture.set_blend_mode(BlendMode::Blend);
    texture
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

    let matches =
        |p: &image::Rgba<u8>| -> bool { (0..3).all(|i| (p[i] as i32 - bg[i] as i32).abs() <= TOL) };

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
            (x.wrapping_sub(1), y),
            (x + 1, y),
            (x, y.wrapping_sub(1)),
            (x, y + 1),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_key_distinguishes_color() {
        let a = text_key("HUD", Color::RGBA(1, 2, 3, 4));
        let b = text_key("HUD", Color::RGBA(1, 2, 3, 5));
        assert_ne!(a, b);
    }

    #[test]
    fn text_cache_reuses_existing_entries() {
        let mut cache = TextCache::new();
        let key = text_key("Speed: 42", Color::WHITE);

        let first_ptr = {
            let value = cache.get_or_insert_with(key.clone(), || String::from("first"));
            value.as_ptr()
        };

        let second_ptr = {
            let value = cache.get_or_insert_with(key, || String::from("second"));
            value.as_ptr()
        };

        assert_eq!(first_ptr, second_ptr);
        assert_eq!(cache.entries.len(), 1);
        assert_eq!(
            cache
                .entries
                .values()
                .next()
                .expect("cache should contain one entry"),
            "first"
        );
    }

    #[test]
    fn text_cache_stores_separate_entries_for_different_keys() {
        let mut cache = TextCache::new();
        let _ = cache.get_or_insert_with(text_key("a", Color::WHITE), || 1);
        let _ = cache.get_or_insert_with(text_key("b", Color::WHITE), || 2);
        assert_eq!(cache.entries.len(), 2);
    }
}
