# Test Suite — Follow the Gap Algorithm

## Overview

Standalone unit and integration tests for the Follow the Gap reactive obstacle avoidance node (`gap_follow/scripts/reactive_node.py`).

**No ROS2, no simulator, no Docker required.** All ROS2 dependencies are mocked out at import time. The algorithm logic is pure NumPy array operations and can be fully validated on any machine with Python 3 + NumPy.

### Run

```bash
python3 gap_follow/test/test_reactive_node.py
```

Or using the project venv:

```bash
.venv/bin/python gap_follow/test/test_reactive_node.py
```

**Expected output:** `Ran 35 tests in ~0.02s — OK`

---

## How the mocking works

`rclpy`, `rclpy.node`, `sensor_msgs`, and `ackermann_msgs` are injected into `sys.modules` as fake objects **before** `reactive_node` is imported. The `Node` base class is replaced with a real Python class (`_FakeNode`) that provides no-op implementations of `create_subscription`, `create_publisher`, and `get_logger`. This allows:

- The `ReactiveFollowGap` class to be instantiated any number of times (Python 3.14+ `MagicMock` as a base class breaks on the second instantiation).
- Instance methods (`preprocess_lidar`, `find_max_gap`, etc.) to be called directly without being intercepted by mock's `__getattribute__`.
- `publisher.publish()` calls to be tracked via `MagicMock` call history.

`LaserScan` messages are simulated with `MockScan`, a plain Python class that sets `angle_min`, `angle_max`, `angle_increment`, and `ranges` to match the Hokuyo UST-10LX used on the RoboRacer (270° FOV, 1080 beams).

---

## Test Suites

### 1. `TestPreprocessLidar` — 7 tests

Validates the data cleaning step that runs before any algorithm logic.

| Test | Scenario | Assertion |
|---|---|---|
| `test_inf_replaced` | All `inf` inputs | All values are finite after preprocessing |
| `test_nan_replaced` | All `nan` inputs | All values are finite after preprocessing |
| `test_values_clipped_to_max_range` | Values above `MAX_RANGE_CLIP` (3.0 m) | All values ≤ `MAX_RANGE_CLIP` |
| `test_values_clipped_to_zero` | Negative values (sensor glitch) | All values ≥ 0 |
| `test_uniform_array_unchanged_by_smoothing` | Constant array of 1.5 m — including at boundaries | All output values ≈ 1.5 m (validates edge-padding, not zero-padding) |
| `test_output_length_preserved` | 1080-element array | Output length is still 1080 |
| `test_mixed_inf_and_valid` | Mix of `inf` and valid readings | All finite, all ≤ `MAX_RANGE_CLIP` |

**Why `test_uniform_array_unchanged_by_smoothing` is important:**
Using `np.convolve(mode='same')` zero-pads at the array boundaries, which makes the first and last smoothed values smaller than they should be. The bubble radius formula `r_b / (d * angle_increment)` is inversely proportional to `d` — an artificially small `d` at the scan boundary would make the bubble span hundreds of indices and zero out all free space. Edge-padding (`np.pad(mode='edge')`) fixes this. This test caught a real bug before simulator testing.

---

### 2. `TestFindMaxGap` — 8 tests

Validates the core gap-detection algorithm with known small arrays where the correct answer can be computed by hand.

| Test | Input array | Expected `(start, end)` |
|---|---|---|
| `test_two_runs_returns_longer` | `[0,1,2,3,0,0,1,2,0]` | `(1, 3)` — longer run wins |
| `test_single_nonzero_element` | `[0,0,5,0,0]` | `(2, 2)` — single beam |
| `test_all_free_space` | All non-zero, length 10 | `(0, 9)` — full array |
| `test_gap_at_end_of_array` | `[1,0,0,0,1,2,3,4,5]` | `(4, 8)` — tail run |
| `test_gap_at_start_of_array` | `[3,2,1,0,0,1,0]` | `(0, 2)` — head run |
| `test_equal_length_gaps_first_wins` | `[1,1,1,0,2,2,2]` | `(0, 2)` — deterministic |
| `test_all_zeros_does_not_crash` | All zeros, length 50 | No exception; returns a 2-tuple |
| `test_large_realistic_array` | 1080 elements, indices 200–300 zeroed | `(300, 1079)` — correct on real LiDAR size |

---

### 3. `TestFindBestPoint` — 5 tests

Validates that the steering target is always the **center** of the gap (anti-wiggle strategy).

| Test | `(start_i, end_i)` | Expected index |
|---|---|---|
| `test_center_even_width` | `(2, 8)` | `5` |
| `test_center_odd_width` | `(1, 5)` | `3` |
| `test_single_element_gap` | `(4, 4)` | `4` |
| `test_full_array_gap` | `(0, 1079)` | `539` |
| `test_result_within_bounds` | Various pairs | Result always in `[start_i, end_i]` |

**Why center and not furthest point:**
When the car is laterally offset in a straight corridor, the two far corners alternate as "furthest point" on each LiDAR scan update (~40 Hz). Chasing the furthest point causes left-right oscillation (the "S-shape wiggle" — FTG Tweak 4 from the lecture). The center of the gap is a stable, continuously varying target that eliminates this behavior.

---

### 4. `TestSpeedChoice` — 5 tests

Validates the piecewise speed lookup based on the distance to the nearest obstacle.

| Test | Nearest obstacle | Expected speed |
|---|---|---|
| `test_far_obstacle_fast_speed` | > `FAST_THRESHOLD` (1.5 m) | `FAST_SPEED` (3.0 m/s) |
| `test_medium_distance_mid_speed` | Between thresholds | `MID_SPEED` (1.5 m/s) |
| `test_close_obstacle_slow_speed` | < `MID_THRESHOLD` (0.5 m) | `SLOW_SPEED` (0.5 m/s) |
| `test_boundary_fast_threshold` | Exactly at 1.5 m | NOT `FAST_SPEED` (boundary is strict `>`) |
| `test_boundary_mid_threshold` | Exactly at 0.5 m | NOT `MID_SPEED` (boundary is strict `>`) |

---

### 5. `TestIntegration` — 11 tests

End-to-end tests: a `MockScan` is passed into `lidar_callback()` and the published `AckermannDriveStamped` message is inspected. These test the entire pipeline — preprocess → bubble → gap → best point → steering + speed → publish — as a unit.

| Test | Scan scenario | Assertion |
|---|---|---|
| `test_obstacle_on_right_steers_left` | Right 25% at 0.5 m, rest at 3.0 m | `steering_angle > 0` (left turn) |
| `test_obstacle_on_left_steers_right` | Left 25% at 0.5 m, rest at 3.0 m | `steering_angle < 0` (right turn) |
| `test_obstacles_both_sides_steers_forward` | Both outer quarters at 0.5 m, center open | `steering_angle ≈ 0` (±0.3 rad) |
| `test_open_field_drives_fast` | All readings at `MAX_RANGE_CLIP` | `speed == FAST_SPEED` (3.0 m/s) |
| `test_close_obstacles_drives_slow` | All readings at 0.2 m | `speed == SLOW_SPEED` (0.5 m/s) |
| `test_publish_called_every_scan` | 5 scans sent | `publish()` called exactly 5 times |
| `test_no_crash_on_all_inf` | All `inf` (LiDAR startup state) | No exception |
| `test_no_crash_on_all_zeros` | All 0.0 (sensor error) | No exception |
| `test_empty_gap_fallback_steers_straight` | All 0.01 m (car fully surrounded) | `steering=0.0`, `speed=SLOW_SPEED` |
| `test_steering_within_lidar_fov` | Left two-thirds open | Angle within ±135° (LiDAR FOV) |

**Why `test_empty_gap_fallback_steers_straight` exists:**
When the safety bubble radius (in index space) exceeds the length of the scan, all values get zeroed and `find_max_gap` returns `(0, -1)`. Without a fallback, the node would compute `steering_angle = angle_min + (-1) * angle_increment`, which is a large negative number — a hard left command in a crash scenario. The fallback publishes `steering=0, speed=SLOW_SPEED` instead.

---

## LiDAR angle convention (F1TENTH)

```
index 0           →  angle_min  (~-2.356 rad / -135°)  rightmost beam
index n // 2      →  ~0 rad                             straight ahead
index n - 1       →  angle_max  (~+2.356 rad / +135°)  leftmost beam

steering_angle > 0  →  left turn
steering_angle < 0  →  right turn
```

This is critical for understanding the directional integration tests: obstacles on the **right** (low indices) → gap on the left (high indices) → positive steering angle.

---

## Bugs caught during testing

| Bug | Root cause | Fix |
|---|---|---|
| Boundary values distorted after smoothing | `np.convolve(mode='same')` zero-pads, shrinking edge values | Use `np.pad(mode='edge')` before convolution |
| Empty-gap crash → garbage steering | No check for `find_max_gap` returning `(0, -1)` | Added fallback: publish straight + slow if `end_i < start_i` |
