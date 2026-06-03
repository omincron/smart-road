use std::collections::HashSet;

pub struct StatsAccumulator {
    pub vehicles_passed: u32,
    pub max_velocity: f32,
    pub min_velocity: f32,
    max_transit: Option<u64>,
    min_transit: Option<u64>,
    pub close_calls: u32,
    active_violations: HashSet<(u32, u32)>, // pairs currently inside safety distance
}

impl StatsAccumulator {
    pub fn new() -> Self {
        StatsAccumulator {
            vehicles_passed: 0,
            max_velocity: f32::MIN,
            min_velocity: f32::MAX,
            max_transit: None,
            min_transit: None,
            close_calls: 0,
            active_violations: HashSet::new(),
        }
    }

    /// Call when a vehicle finishes crossing (gets exit_tick).
    pub fn record_exit(&mut self, transit_ticks: u64) {
        self.vehicles_passed += 1;
        self.max_transit = Some(self.max_transit.unwrap_or(0).max(transit_ticks));
        self.min_transit = Some(self.min_transit.unwrap_or(u64::MAX).min(transit_ticks));
    }

    /// Call each tick with the current speed (px/tick) of every active vehicle.
    pub fn record_speed(&mut self, px_per_tick: f32) {
        if px_per_tick > self.max_velocity { self.max_velocity = px_per_tick; }
        if px_per_tick < self.min_velocity { self.min_velocity = px_per_tick; }
    }

    /// Call each tick with every pair of vehicles that violates the safety gap.
    /// Counts each NEW pair entering violation as one close-call event.
    pub fn update_violations(&mut self, current: HashSet<(u32, u32)>) {
        for pair in &current {
            if !self.active_violations.contains(pair) {
                self.close_calls += 1;
            }
        }
        self.active_violations = current;
    }

    /// Returns stat lines ready to render on screen.
    pub fn stat_lines(&self) -> Vec<String> {
        let mut v = Vec::new();
        v.push("  Simulation Statistics".to_string());
        v.push(String::new());
        v.push(format!("  Vehicles passed : {}", self.vehicles_passed));
        if self.max_velocity > f32::MIN {
            v.push(format!("  Max velocity    : {:.1} px/tick", self.max_velocity));
            v.push(format!("  Min velocity    : {:.1} px/tick",
                if self.min_velocity < f32::MAX { self.min_velocity } else { 0.0 }));
        }
        match (self.max_transit, self.min_transit) {
            (Some(max), Some(min)) => {
                v.push(format!("  Max transit     : {} ticks", max));
                v.push(format!("  Min transit     : {} ticks", min));
            }
            _ => v.push("  Transit time    : N/A".to_string()),
        }
        v.push(format!("  Close calls     : {}", self.close_calls));
        v.push(String::new());
        v.push("  Press ESC to quit".to_string());
        v
    }
}
