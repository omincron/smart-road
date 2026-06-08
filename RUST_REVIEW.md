# Smart-Road Rust Code Review

Reviewed by: senior Rust engineer  
Date: 2026-06-08  
Revision: `3e497db` (HEAD on `main`)

---

## Executive Summary

Smart-Road is a well-structured, small Rust codebase with clean module boundaries and an
honestly implemented AIM (Autonomous Intersection Management) reservation algorithm. The
simulation logic is free of `unwrap` panics in hot paths, all 67 unit tests pass, and the
collision-avoidance model is coherent end-to-end. The three most significant weaknesses are
(1) the conflict table is hard-coded with no geometric derivation or test coverage verifying
its physical correctness, making silent bugs possible; (2) there is a real integer-overflow
risk in `try_reserve_timed` when `entry < current_tick`; and (3) the renderer and intersection
modules duplicate the same `CENTER_X / CENTER_Y / ROAD_W` constants instead of using a
single source of truth, making layout maintenance error-prone.

---

## 1. Architecture & Design

### Module boundaries — good overall

The split into `vehicle/`, `intersection/`, `simulation`, `renderer/`, `input/`, `statistics/`
matches the planned layout in CLAUDE.md almost exactly. Rendering, physics, and scheduling
concerns are not entangled.

### `simulation.rs` is a free function, not a struct

`step_simulation` is a 120-line free function that takes four mutable references. It
performs vehicle state dispatch, lane-group building, off-screen detection, and close-call
detection all in one body. Clippy pedantic already flags it (`too_many_lines`). Splitting
the Approaching / Crossing / Exiting arms into private helpers would improve readability
without changing the design.

### Renderer knows about game state geometry

`simulation.rs` imports `renderer::WINDOW_W` / `renderer::WINDOW_H` directly
(`src/simulation.rs:189-190`) to decide when a vehicle is off-screen. Renderer constants
bleeding into simulation logic inverts the dependency direction. A single `layout.rs` (or
constants in `intersection/`) exported to both would decouple them.

### `Speed` is a zero-size struct acting as a namespace for constants

```rust
// src/vehicle/mod.rs:20-26
pub struct Speed;
impl Speed {
    pub const SLOW_PX: f32 = 1.5;
    ...
}
```

This is unidiomatic. Use `pub mod speed` with plain `pub const` items, or at least a unit
struct with `#[allow(dead_code)]`. As written, `Speed` is never instantiated, `cargo clippy`
emits no warning only because it is `pub`, but the pattern misleads readers into thinking
`Speed` has instances.

### `IntersectionManager` has no `Default` impl

`IntersectionManager::new()` delegates to nothing special; it should `#[derive(Default)]` or
implement `Default`. Same applies to `InputHandler`, `StatsAccumulator`, and `LaneGroups`.

### `CENTER_X` / `CENTER_Y` are duplicated

`renderer/mod.rs:13-14` and `input/mod.rs:8-9` each define `CENTER_X`/`CENTER_Y` from
`WINDOW_W`/`WINDOW_H`. The renderer exports `CENTER_X` and `CENTER_Y` as `pub const`;
`input` re-derives its own local copies. `input` already imports `renderer::{LANE_W,
ROAD_W, WINDOW_H, WINDOW_W}` — it could simply re-use `renderer::CENTER_X` / `CENTER_Y`
instead of duplicating them.

---

## 2. Rust Idioms

### `let...else` not used (`simulation.rs:57-60`)

```rust
// current
let group = match groups.get(&(dir, route)) {
    Some(g) => g,
    None => return f32::MAX,
};
```

Clippy pedantic flags this. Prefer `let Some(group) = groups.get(...) else { return f32::MAX };`.

### `.map(...).unwrap_or(...)` instead of `.map_or(...)` (`simulation.rs:63-67`)

```rust
group[start..]
    .iter()
    .find(|&&(gid, _)| gid != id)
    .map(|&(_, leader_pos)| leader_pos - my_pos)
    .unwrap_or(f32::MAX)
```

Prefer `.map_or(f32::MAX, |&(_, leader_pos)| leader_pos - my_pos)`.

### `match` for single-arm + else (`simulation.rs:137-146`)

```rust
match manager.try_reserve_timed(...) {
    Some((entry, spd)) => { ... }
    None => { ... }
}
```

Should be `if let Some((entry, spd)) = ... { } else { }`.

### Unnecessary owned `String` in `stat_lines`

`stat_lines()` (`statistics/mod.rs:63-97`) allocates a `Vec<String>` every time the
overlay renders, which is once per frame while the stats screen is visible. Since the stats
are immutable at that point, returning `&'static str` slices or formatting into a pre-sized
buffer would avoid repeated heap allocation. For the target frame-rate this is minor, but
it is the hot path during stats rendering.

### Lifetime annotation on private function could be elided

`load_car_texture<'tc>` (`renderer/mod.rs:246`) — Clippy flags this as elidable:
`fn load_car_texture(tc: &TextureCreator<WindowContext>) -> Texture<'_>`.

### `f32` arithmetic on pixel geometry — precision is not a concern

The pedantic `cast_precision_loss` warnings for `i32 → f32` casts at pixel-geometry sites
are not genuine bugs (the values fit comfortably in `f32` mantissa at 800px resolution) but
should be suppressed with targeted `#[allow]` or replaced with `f32` constants in the first
place. Leaving 79 clippy warnings clutters CI output.

---

## 3. Safety & Correctness

### CRITICAL: Integer underflow in `try_reserve_timed` (`intersection/mod.rs:225`)

```rust
let ticks_left = (entry as i64 - current_tick as i64).max(1) as f32;
```

Clippy warns about the `u64 → i64` casts. When `entry` is returned from a previous call
and `current_tick` has advanced past `entry + GRACE_TICKS`, the branch at line 222 should
catch it — but it compares `current_tick <= entry + GRACE_TICKS`. If `entry` is 0 and
`GRACE_TICKS` is 10, a `current_tick` of 11 passes the guard. The subtraction at line 225
then produces a negative value, `.max(1)` clamps it to 1, and the resulting approach speed
is clamped to `FAST_PX` — an overspeed. This is unlikely in normal play but is a
correctness hole.

### CRITICAL: `transit_ticks` can underflow silently

```rust
// vehicle/mod.rs:82
(Some(d), Some(e)) => Some(e - d),
```

If `exit_tick < detection_tick` (impossible under correct operation, but no invariant
enforces it), this panics in debug mode or wraps in release. Add `e.saturating_sub(d)` or
an explicit assertion.

### HIGH: Conflict table is manually authored with no geometric proof

`intersection/mod.rs:33-47` — The `CONFLICTS` 12×12 matrix is hand-typed. Tests verify
symmetry and that right-turns are conflict-free, but there is no test that checks a
conflicting path pair actually has overlapping tiles, nor a test for every claimed
non-conflicting pair. A single wrong `false` entry can permit two vehicles to occupy the
same tile simultaneously without triggering any run-time check. The waypoint geometry in
`crossing_waypoints` should be used to derive or at least cross-validate this table.

### HIGH: `strip_background` BFS silently does nothing if the corner pixel does not match the expected background

`renderer/mod.rs:270-313` — If the asset changes to have a transparent corner or a
dark-coloured background, `strip_background` will leave all pixels untouched and the car
sprite will render with a solid rectangle around it. There is no assertion or log warning.
The function should check that at least one pixel was actually stripped, or accept the
background colour as an explicit parameter.

### MEDIUM: `try_spawn` proximity check uses Euclidean distance across all vehicles

`input/mod.rs:79-83` — The spawn-blocking check iterates all vehicles regardless of
direction or lane. A vehicle at the same position but in a perpendicular lane will block
spawning when it should not. The check should be restricted to vehicles in the same
(direction, route) lane, or at least the same direction.

### MEDIUM: `VehicleState::Done` vehicles remain in the `Vec` until end-of-tick

`simulation.rs:245`:
```rust
vehicles.retain(|v| v.state != VehicleState::Done);
```

`Done` vehicles are iterated and processed (speed recorded) in the main loop at line 111
before the `retain`. They harmlessly fall into `VehicleState::Done => {}` but the
`record_speed` call at line 112 still runs, inflating the minimum-velocity statistic toward
zero because `Done` vehicles stop smoothing and eventually have `current_speed ≈ 0`.

### MEDIUM: `at_stop_line` and `dist_to_stop_line` can give contradictory answers for the same position

`at_stop_line` uses `>=` / `<=`, while `dist_to_stop_line` returns positive values only when
still approaching. When a vehicle is exactly on the stop line, `dist_to_stop_line` returns
0.0 and the `dist <= RESERVATION_DIST` branch fires, calling `try_reserve_timed` with
`dist_to_stop = 0.0`. Dividing `0.0 / Speed::FAST_PX` gives 0.0 and `ceil() as u64` gives
0, setting `earliest = current_tick`. This is benign but surprising; a comment would help.

### LOW: `next_id` in `InputHandler` wraps at `u32::MAX`

After ~4 billion spawned vehicles, `next_id` wraps and IDs alias. Practically harmless, but
using a `u64` would be more correct.

---

## 4. Performance

### `build_lane_groups` allocates `HashMap` and `Vec`s on every tick

`simulation.rs:27-44` — This runs every tick regardless of whether the vehicle list has
changed. For the expected vehicle counts (< 50) this is not a bottleneck, but if you wanted
to profile, incremental maintenance of a persistent structure would help.

### `reservations` is a `Vec` scanned linearly in hot paths

`IntersectionManager.reservations` is searched with `.iter().position(...)` and
`.iter().any(...)` on every reservation call. For a simulation with dozens of vehicles this
is O(n) per vehicle per tick, i.e., O(n²) total. At simulation scale this is fine, but
`HashMap<u32, TimedReservation>` keyed on `vehicle_id` would be clearer and O(1).

### `crossing_path_length` re-computes waypoints on every call

`intersection/mod.rs:106-115` — `crossing_path_length` calls `crossing_waypoints` which
allocates a `Vec` of tuples. Both `try_reserve_timed` and `try_reserve` call it. The
results are deterministic for any `(direction, route)` pair; they should be pre-computed
once into a `[[f32; 12]]` table just like `CONFLICTS`.

### `stat_lines` called every rendered frame while the overlay is shown

`renderer/mod.rs:172`, `statistics/mod.rs:63` — `stat_lines` formats strings on every
render call. Since the stats are immutable once the overlay is visible, cache the formatted
`Vec<String>` on first call.

---

## 5. Error Handling

### `expect` / `panic` in renderer initialisation is acceptable

`load_rgb_texture` and `load_car_texture` call `.expect(...)` and `.unwrap_or_else(|_|
panic!(...))`. For asset-loading at startup these are acceptable: a missing PNG is a
programmer error, not a recoverable condition. Document that assets must be present in the
working directory.

### SDL2 draw calls — results silently discarded

Throughout `renderer/mod.rs`, all SDL2 `Result`-returning draw calls are discarded with
`let _ = ...`. This is standard SDL2-Rust practice because draw errors are non-recoverable,
but it would be cleaner to handle them consistently (either a macro that logs them in debug
builds, or a wrapping helper).

### No error path if `IntersectionManager::try_reserve_timed` returns `None`

`simulation.rs:136-146` — when `try_reserve_timed` returns `None` (intersection saturated),
the vehicle's target speed is set to `SLOW_PX` and `reservation` is cleared. The vehicle
will re-request every tick. This is functionally correct but the `None` case is not logged
anywhere. In stress tests it can silently hide a livelock.

---

## 6. Code Quality

### Naming

- `axial_pos` is clear. `sb` / `nb` / `wb` / `eb` inside `crossing_waypoints` are opaque
  one-letter closures (`sb = "southbound x lane centre"`). Expand them or add a comment at
  their definition site.
- `APPROACH_FAR_PX` / `APPROACH_NEAR_PX` are good. `OFFSCREEN_MARGIN` is clear.
- `C_STOP` (`renderer/mod.rs:30`) — the `C_` prefix adds nothing. `STOP_LINE_COLOR` would
  be more descriptive.

### Dead / unreachable code

- `LANE_W` is exported from `renderer/mod.rs` as `pub const` and imported in `input/mod.rs`,
  but the value `ROAD_W / 3 = 27` is also re-derived inside `crossing_waypoints` as
  `let lw = rw / 3.0`. Use `LANE_W` directly to make the coupling explicit.

- `lane_index` and `spawn_pos` in `input/mod.rs` are marked `pub` but are only called
  internally. They could be `pub(crate)` or private.

### Missing `#[must_use]` on pure query functions

`crossing_path_length`, `crossing_waypoints`, `dist_to_stop_line`, `at_stop_line`,
`exit_direction`, `stat_lines`, `transit_ticks` are all pure and their return values are
always consumed — but they lack `#[must_use]`. Adding it prevents accidental discard.

### `glyph_for` is 140-line match arm

`renderer/mod.rs:329-376` — Large but mechanical; not a problem. It would be cleaner as a
data table (`static GLYPHS: [(char, [[u8; 5]; 7]); N]`) but the current form is readable.

---

## 7. Testing

### Coverage summary

67 tests, all passing. Coverage is concentrated in:
- `vehicle/mod.rs` and `vehicle/physics.rs` — thorough unit tests for movement, smoothing, transit
- `intersection/mod.rs` — reservation logic, symmetry, expiry, timed slots
- `simulation.rs` — lane-group building, speed helpers, one integration smoke test
- `statistics/mod.rs` — all stat methods covered

### What is tested well

- `conflicts_table_is_symmetric` catches a whole class of hand-editing errors.
- `build_lane_groups_*` tests cover the tricky Crossing-at-stop-line edge case.
- `timed_reserve_conflicting_windows_do_not_overlap` verifies the key AIM invariant.
- `advance_crossing_snaps_to_waypoint_when_close` covers the snap logic.

### What is missing

1. **Conflict table geometric correctness** — no test verifies that paths marked `true` in
   `CONFLICTS` actually have overlapping waypoint bounding boxes. A geometry-driven test
   would catch a misplaced `false`.

2. **`spawn_pos` / `lane_index`** — no tests. These are pure coordinate functions and easy
   to unit-test; a regression would silently misplace every vehicle at spawn.

3. **`exit_direction` consistency with `crossing_waypoints`** — no test verifies that the
   exit waypoint of a path is in the half-road region consistent with `exit_direction`. For
   example, `(Direction::South, Route::Left)` should exit eastbound and the last waypoint's
   x should be `cx + rw`.

4. **`strip_background`** — no test. Even a trivial test with a 4×4 white pixel block would
   catch regressions.

5. **`step_simulation` end-to-end** — only one smoke test at `simulation::tests::step_simulation_can_run_without_sdl_types`. A vehicle that reaches `Done` state, records a transit time, and contributes to stats is not tested.

6. **Close-call detection** — `simulation.rs:211-243` is untested. An integration test
   placing two vehicles close together and verifying `stats.close_calls == 1` would be
   trivial.

7. **Reservation re-booking when window passes** — `try_reserve_timed` drops and re-books
   when `current_tick > entry + GRACE_TICKS`. This branch has no dedicated test.

---

## 8. Spec Compliance (against CLAUDE.md)

| Requirement | Status | Notes |
|---|---|---|
| 3 distinct speed levels | PASS | `SLOW_PX=1.5`, `NORMAL_PX=3.0`, `FAST_PX=5.0` |
| AIM controls assigned speed | PASS | `try_reserve_timed` returns `(entry_tick, speed)` |
| Safety distance strictly positive | PASS | `SAFETY_DISTANCE=40`, `MIN_FOLLOWING_GAP=72` |
| Per-vehicle physics tracking | PARTIAL | `time`/`distance` not accumulated per tick; only `detection_tick`, `exit_tick`, and current speed are stored. The spec says track `distance` per vehicle; this is not implemented. |
| No spawn on top of each other | PASS | `try_spawn` checks `MIN_FOLLOWING_GAP` radius |
| Close-call recording | PASS | `StatsAccumulator::update_violations` counts entry events |
| Arrow keys spawn vehicles | PASS | `input/mod.rs:41-44` |
| `R` toggles random generation | PASS | `input/mod.rs:45-49` |
| `Esc` shows stats then quits | PASS | `main.rs:59-64` |
| Stats: max vehicles passed | PASS | `vehicles_passed` |
| Stats: max/min velocity | PASS | `max_velocity` / `min_velocity` |
| Stats: max/min transit time | PASS | `max_transit` / `min_transit` |
| Stats: close calls | PASS | `close_calls` |
| Visual rotation during turns | PASS | `copy_ex` with `angle_deg`, updated in `advance_crossing` |
| 4-way intersection, 3 lanes per direction | PASS | 12 paths in conflict table |

**Spec gap**: CLAUDE.md requires tracking `distance` per vehicle as part of the
per-vehicle physics model. No field on `Vehicle` accumulates odometer distance; only entry
and exit ticks are stored.

---

## Prioritised Action List

### Critical

1. **`intersection/mod.rs:225` — integer underflow risk in `try_reserve_timed`.**
   The `u64 → i64` cast when computing `ticks_left` can produce unexpected negative values
   after the `.max(1)` clamp silently masks them. Rewrite as:
   ```rust
   let ticks_left = entry.saturating_sub(current_tick).max(1) as f32;
   ```
   No cast to `i64` needed.

2. **`vehicle/mod.rs:82` — `transit_ticks` subtraction can wrap.**
   Change `Some(e - d)` to `Some(e.saturating_sub(d))` and add a debug assertion that
   `e >= d`.

### High

3. **Verify or generate the `CONFLICTS` table geometrically.**
   Add a test in `intersection::tests` that, for every `(i, j)` pair where `CONFLICTS[i][j]
   == true`, verifies that the AABB of path `i` overlaps with the AABB of path `j`. Derive
   the AABBs from `crossing_waypoints`. This will detect silent `false` entries.

4. **Fix the `Done`-vehicle speed recording bias.**
   Move `stats.record_speed(v.current_speed)` inside an `if v.state != VehicleState::Done`
   guard (`simulation.rs:112`) to avoid pulling min-velocity toward zero.

5. **Consolidate `CENTER_X`, `CENTER_Y` constants.**
   Remove the re-derivation in `input/mod.rs:8-9`. Import from `renderer`. Consider also
   making `LANE_W` in `crossing_waypoints` reference `renderer::LANE_W` rather than
   computing `rw / 3.0` again.

6. **Pre-compute `crossing_path_length` per `(direction, route)`.**
   Allocating a `Vec` and iterating it in `try_reserve_timed` and `try_reserve` on every
   reservation call is wasteful. Add a `const` or `lazy_static` table of 12 `f32` values.

### Medium

7. **Add `#[must_use]` to all pure query functions** (see §6 list).

8. **Fix the spawn-blocking check to be lane-local** (`input/mod.rs:79-83`). Only compare
   against vehicles with matching `direction`.

9. **Add `distance` field to `Vehicle`** and accumulate it in `advance()` and
   `advance_crossing()` to satisfy the spec's per-vehicle physics requirement.

10. **Address the 79 pedantic clippy warnings.** Most are mechanical `as`-cast style
    fixes. Run `cargo clippy --fix` for the 19 auto-fixable ones, then add targeted
    `#[allow]` with comments for the intentional casts (pixel arithmetic).

11. **Rename `Speed` struct to a `mod` or remove the zero-size wrapper** — use plain
    `pub const` items.

### Low

12. **Add missing unit tests** for `spawn_pos`, `lane_index`, close-call integration,
    `strip_background`, `Done` → stats pipeline, and reservation re-booking (see §7).

13. **Rename `C_STOP` → `STOP_LINE_COLOR`** and expand the opaque `sb`/`nb`/`wb`/`eb`
    closure names in `crossing_waypoints`.

14. **Implement `Default` for `IntersectionManager`, `InputHandler`, `StatsAccumulator`.**

15. **`next_id` in `InputHandler`**: change type to `u64` to prevent wrap-around.

---

## Positive Findings

- **Clean AIM implementation.** The time-slot jump-scheduling in `try_reserve_timed`
  (skipping blocked windows in one step via `min(exit_tick)`) is algorithmically clean and
  avoids the naïve tick-by-tick loop that would be O(window_size × n_reservations).

- **Lane-leader lookup is O(n log n) per tick, not O(n²).** `build_lane_groups` with sorted
  insertion and `partition_point` binary search is a good choice.

- **Smooth-speed interpolation prevents teleporting.** The exponential approach
  (`smooth_speed`) with both `target_speed` and `current_speed` capping (`simulation.rs:155-156`)
  prevents a leader-gap cap from being bypassed by carried momentum — a subtle but correct
  detail.

- **Self-contained font renderer.** Removing `SDL_ttf` as a native dependency and
  implementing a compact bitmap glyph map is pragmatic and makes the binary fully
  self-contained for deployment.

- **Conflict table is symmetric and tested.** `conflicts_table_is_symmetric` and
  `right_turns_are_conflict_free` are high-value tests that guard the most safety-critical
  data structure.

- **No hot-path `unwrap` panics.** The simulation tick loop and reservation logic use
  `Option`-returning functions throughout; panic sites are confined to asset loading at
  startup where they are appropriate.

- **Excellent test naming and organisation.** Test names are descriptive sentences (`advance_crossing_snaps_to_waypoint_when_close`), each test is short, and modules are grouped with section headers. This is a high standard for a game project.
