use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::Canvas;
use sdl2::video::Window;

use crate::vehicle::{Direction, Vehicle};

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

// vehicle colours by travel direction
const C_NORTH: Color = Color { r: 220, g: 60, b: 60, a: 255 };  // heading north (red)
const C_SOUTH: Color = Color { r: 60, g: 120, b: 220, a: 255 }; // heading south (blue)
const C_EAST: Color = Color { r: 60, g: 180, b: 60, a: 255 };   // heading east  (green)
const C_WEST: Color = Color { r: 220, g: 200, b: 50, a: 255 };  // heading west  (yellow)

pub struct Renderer {
    pub canvas: Canvas<Window>,
}

impl Renderer {
    pub fn new(canvas: Canvas<Window>) -> Self {
        Renderer { canvas }
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
        let color = match v.original_direction {
            Direction::North => C_NORTH,
            Direction::South => C_SOUTH,
            Direction::East => C_EAST,
            Direction::West => C_WEST,
        };
        const SZ: i32 = 28;
        self.canvas.set_draw_color(color);
        self.canvas
            .fill_rect(Rect::new(v.x as i32 - SZ / 2, v.y as i32 - SZ / 2, SZ as u32, SZ as u32))
            .unwrap();
    }

    /// Dark semi-transparent overlay shown when the simulation ends.
    pub fn draw_stats_overlay(&mut self) {
        self.canvas.set_blend_mode(sdl2::render::BlendMode::Blend);

        // Dim the whole screen
        self.canvas.set_draw_color(Color::RGBA(0, 0, 0, 180));
        self.canvas.fill_rect(Rect::new(0, 0, WINDOW_W, WINDOW_H)).unwrap();

        // Central info panel
        let pw: u32 = 320;
        let ph: u32 = 120;
        let px = (WINDOW_W as i32 - pw as i32) / 2;
        let py = (WINDOW_H as i32 - ph as i32) / 2;

        self.canvas.set_draw_color(Color::RGBA(20, 20, 40, 230));
        self.canvas.fill_rect(Rect::new(px, py, pw, ph)).unwrap();

        // Border
        self.canvas.set_draw_color(Color::RGB(180, 180, 220));
        for t in 0..3 {
            self.canvas
                .draw_rect(Rect::new(px - t, py - t, pw + 2 * t as u32, ph + 2 * t as u32))
                .unwrap();
        }

        // Three coloured bars as a simple "stats ended" indicator
        let bar_w: u32 = 60;
        let bar_h: u32 = 20;
        let gap: i32 = 20;
        let total = (bar_w * 3) as i32 + gap * 2;
        let bx = px + (pw as i32 - total) / 2;
        let by = py + (ph as i32 - bar_h as i32) / 2;

        let colours = [
            Color::RGB(220, 80, 80),
            Color::RGB(80, 180, 80),
            Color::RGB(80, 120, 220),
        ];
        for (i, c) in colours.iter().enumerate() {
            self.canvas.set_draw_color(*c);
            self.canvas
                .fill_rect(Rect::new(bx + i as i32 * (bar_w as i32 + gap), by, bar_w, bar_h))
                .unwrap();
        }

        self.canvas.set_blend_mode(sdl2::render::BlendMode::None);
    }

    pub fn present(&mut self) {
        self.canvas.present();
    }
}
