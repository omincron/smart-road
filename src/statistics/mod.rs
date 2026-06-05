use std::collections::HashSet;

pub struct StatsAccumulator {
    pub vehicles_passed: u32,
    pub max_velocity: f32,
    pub min_velocity: f32,
    max_transit: Option<u64>,
    min_transit: Option<u64>,
    pub close_calls: u32,
    pub min_gap_px: f32,
    active_violations: HashSet<(u32, u32)>,
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
            min_gap_px: f32::MAX,
            active_violations: HashSet::new(),
        }
    }

    pub fn record_gap(&mut self, gap: f32) {
        if gap < self.min_gap_px {
            self.min_gap_px = gap;
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
        if px_per_tick > self.max_velocity {
            self.max_velocity = px_per_tick;
        }
        if px_per_tick < self.min_velocity {
            self.min_velocity = px_per_tick;
        }
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
        const FPS: f32 = 60.0;
        let mut v = Vec::new();
        v.push("  Simulation Statistics".to_string());
        v.push(String::new());
        v.push(format!("  Vehicles passed : {}", self.vehicles_passed));
        if self.max_velocity > f32::MIN {
            v.push(format!(
                "  Max velocity    : {:.0} px/s",
                self.max_velocity * FPS
            ));
            v.push(format!(
                "  Min velocity    : {:.0} px/s",
                if self.min_velocity < f32::MAX {
                    self.min_velocity * FPS
                } else {
                    0.0
                }
            ));
        }
        match (self.max_transit, self.min_transit) {
            (Some(max), Some(min)) => {
                v.push(format!("  Max transit     : {:.2} s", max as f32 / FPS));
                v.push(format!("  Min transit     : {:.2} s", min as f32 / FPS));
            }
            _ => v.push("  Transit time    : N/A".to_string()),
        }
        v.push(format!("  Close calls     : {}", self.close_calls));
        if self.min_gap_px < f32::MAX {
            v.push(format!("  Min gap seen    : {:.1} px", self.min_gap_px));
        }
        v.push(String::new());
        v.push("  Press ESC to quit".to_string());
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_stats() -> StatsAccumulator {
        StatsAccumulator::new()
    }

    #[test]
    fn record_exit_increments_vehicles_passed() {
        let mut s = make_stats();
        assert_eq!(s.vehicles_passed, 0);
        s.record_exit(100);
        assert_eq!(s.vehicles_passed, 1);
        s.record_exit(200);
        assert_eq!(s.vehicles_passed, 2);
    }

    #[test]
    fn record_exit_tracks_min_max_transit() {
        let mut s = make_stats();
        s.record_exit(50);
        s.record_exit(200);
        s.record_exit(100);
        let lines = s.stat_lines();
        let joined = lines.join("\n");
        assert!(joined.contains("Max transit"), "missing max transit line");
        assert!(joined.contains("Min transit"), "missing min transit line");
    }

    #[test]
    fn record_speed_tracks_min_and_max() {
        let mut s = make_stats();
        s.record_speed(1.5);
        s.record_speed(5.0);
        s.record_speed(3.0);
        assert!((s.max_velocity - 5.0).abs() < f32::EPSILON);
        assert!((s.min_velocity - 1.5).abs() < f32::EPSILON);
    }

    #[test]
    fn update_violations_counts_new_pairs_only() {
        let mut s = make_stats();
        let pair = (0u32, 1u32);

        // First tick: pair enters violation — counts as 1.
        let mut current = HashSet::new();
        current.insert(pair);
        s.update_violations(current.clone());
        assert_eq!(s.close_calls, 1);

        // Second tick: same pair still active — should NOT increment.
        s.update_violations(current);
        assert_eq!(s.close_calls, 1);

        // Third tick: pair resolved — no increment.
        s.update_violations(HashSet::new());
        assert_eq!(s.close_calls, 1);

        // Fourth tick: pair re-enters — counts as a new event.
        let mut renewed = HashSet::new();
        renewed.insert(pair);
        s.update_violations(renewed);
        assert_eq!(s.close_calls, 2);
    }

    #[test]
    fn record_gap_tracks_minimum() {
        let mut s = make_stats();
        s.record_gap(80.0);
        s.record_gap(30.0);
        s.record_gap(50.0);
        assert!((s.min_gap_px - 30.0).abs() < f32::EPSILON);
    }

    #[test]
    fn stat_lines_no_panic_on_empty_stats() {
        let s = make_stats();
        let lines = s.stat_lines();
        assert!(!lines.is_empty());
    }
}
