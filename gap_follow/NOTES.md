# Lab 4 — Follow the Gap: Project Notes

## Platform Overview

**RoboRacer = F1TENTH** (UPenn). 1/10th-scale Ackermann-steered RC car.

| Component | Detail |
|---|---|
| Chassis | Traxxas Slash 4x4 (or equivalent) |
| LiDAR | Hokuyo UST-10LX — 270° FOV, ~1080 beams, 40Hz |
| Motor controller | VESC (open-source ESC) — controls speed + servo |
| Onboard compute | NVIDIA Jetson (Xavier NX / Orin) |
| Steering model | **Ackermann (non-holonomic)** — must reposition to turn |
| Car width | ~0.31m |
| Wheelbase (L) | ~0.33m |

---

## ROS2 Topic Map

```
/scan  (sensor_msgs/LaserScan)
    └──▶  ReactiveFollowGap node  ──▶  /drive  (ackermann_msgs/AckermannDriveStamped)
```

**Simulator:** `f1tenth_gym_ros` (ROS2 bridge over f1tenth_gym Python environment)
- Launch: `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`
- Config: `f1tenth_gym_ros/config/sim.yaml` — change `map_path` here
- Maps go in: `f1tenth_gym_ros/maps/` as `.png` + `.yaml` pairs

---

## LaserScan Message Fields

```python
msg.angle_min        # radians — start angle (~-2.35 rad / -135°)
msg.angle_max        # radians — end angle  (~+2.35 rad / +135°)
msg.angle_increment  # radians per step between beams (~0.00436 rad)
msg.range_min        # minimum valid range (typ. 0.0m)
msg.range_max        # maximum valid range (typ. 30.0m in sim, ~10m real)
msg.ranges           # tuple of floats, len ~1080
                     # index 0 = angle_min (far right)
                     # index len//2 ≈ straight ahead
                     # float('inf') = no return within range_max
```

**Critical:** `inf` and `nan` must be sanitized. Never use raw ranges directly.

---

## AckermannDriveStamped Message Fields

```python
msg.drive.speed           # float — m/s, positive = forward
msg.drive.steering_angle  # float — radians, positive = left
                           # hardware clamped to ~±0.4189 rad (±24°)
```

---

## Algorithm: Follow the Gap (FTG)

### Why Not Wall Following?
Wall following tracks one wall boundary and fails when obstacles are on both sides or in the path. FTG is **reactive and map-free** — it makes decisions based only on the current LiDAR scan.

### Why the Safety Bubble (not naive "largest gap")?
The RoboRacer is **non-holonomic** (Ackermann steering). It cannot slide sideways to squeeze through tight gaps. The safety bubble converts the physical car width into zeroed-out LiDAR readings, so the algorithm only ever sees gaps that are physically wide enough to drive through.

### Core 4-Step Algorithm

```
1. preprocess_lidar(ranges)
   → Replace inf/nan → clip to max_range
   → Apply windowed mean to smooth noise

2. Restrict processing to a forward FOV cone
   → Keep only beams within ±(forward_fov_deg/2) around 0 rad

3. Find closest point (inside forward cone) → draw safety bubble of radius r_b
   → Zero out all indices within the bubble (arc-length formula)
   → Non-zero ranges are now "free space"

4. find_max_gap(free_space_ranges)
   → Linear scan for longest consecutive non-zero run
   → Returns (start_i, end_i)

5. find_best_point(start_i, end_i, ranges)
   → Return CENTER of gap (not furthest point)
   → Avoids S-shape wiggling (FTG Tweak 4)
```

### Converting Best Point Index to Steering Angle

```python
best_angle = angle_min + best_idx * angle_increment
```

The angle is relative to the LiDAR frame. Since the LiDAR is forward-facing and centered, `angle=0` means straight ahead. Positive angle = left.

### Speed Modulation

Speed now has two stages:

1) Base speed from nearest-obstacle distance (inside the forward cone):

```
min_range > 1.5m  →  3.0 m/s   (open track)
min_range > 0.5m  →  1.5 m/s   (obstacle nearby)
else              →  0.5 m/s   (very close obstacle)
```

2) Steering-aware scaling:
- As abs(steering_angle) grows, speed is reduced linearly.
- Full speed is kept for small steering.
- At large steering, speed is limited by a minimum scaling factor.

---

## Parameters and Tuning Guide

| Parameter | Default | Effect if increased | Effect if decreased |
|---|---|---|---|
| `r_b` (bubble radius) | 0.20m | More conservative, wider gaps required | Less conservative, may clip obstacles |
| `max_range` (clip) | 3.0m | Looks further ahead, smoother on straights | Tighter focus on immediate obstacles |
| Window size (smooth) | 3 | More noise filtering, slight lag | Less filtering |
| `forward_fov_deg` | 160° | More side context but can pick less-forward gaps | More forward-only behavior, may be too narrow |
| Speed thresholds | [1.5, 0.5]m | Lower thresholds = faster overall | Higher = more conservative |
| `steer_slowdown_start` | 0.1745 rad | Keeps high speed deeper into turns | Slows earlier when turning |
| `steer_slowdown_end` | 0.4189 rad | Delays max slowdown | Reaches max slowdown sooner |
| `min_steer_speed_scale` | 0.35 | Faster at high steering | Safer/slower in sharp turns |

**Tuning symptoms:**
- **Clips walls** → increase `r_b`
- **Drifts toward side openings** → decrease `forward_fov_deg`
- **S-shape oscillation** → confirm using center of gap, not furthest point
- **Stops in front of reachable gaps** → decrease `r_b` (over-conservative bubble)
- **Slow in open sections** → raise `max_range` clip and/or speed thresholds

---

## Why Gap Center (not Furthest Point)?

When the car is laterally offset, the two far corners of a straight corridor alternate as "furthest point" on each LiDAR scan. Chasing the furthest point causes the car to repeatedly oversteer left then right — the "S-shape wiggle" (FTG Tweak 4 from lecture). Using the **center of the gap** produces a stable, continuous steering target that changes smoothly as the car moves.

---

## Key Decisions Log

| Decision | Choice | Reason |
|---|---|---|
| Language | Python | Faster tuning, NumPy array ops are fast enough (<5ms) |
| Starting variant | Basic FTG + anti-wiggle | Build understanding before adding complexity |
| Best point | Gap center | Anti-wiggle; furthest point only if gap > 3m |
| Disparity Extender | Optional (Phase 9) | Only needed for 4-corner bonus |

---

## Progress Log

- **Phase 0** (complete): ROS2 not installed on macOS dev machine — expected. Sim runs on Linux/Docker environment separately. Topics confirmed from f1tenth_gym_ros docs: `/scan` (LaserScan), `/drive` (AckermannDriveStamped).
- **Phase 1** (complete): This file created.
- **Phase 2–8** (complete): `reactive_node.py` fully implemented.
- **Phase 9** (complete): Node made environment-ready for `f1tenth_gym_ros` by moving runtime implementation to package module (`gap_follow/reactive_node.py`) and adding ROS parameters for topic names and FTG tuning.
- **Phase 10** (in progress): Added forward-FOV gap filtering, steering clamp, and steering-aware speed scaling to reduce wall strikes on straight-to-corner transitions.
- **Local tests** (complete): 35 unit tests passing in `gap_follow/test/test_reactive_node.py`. Run with: `python3 gap_follow/test/test_reactive_node.py`

## Simulator Bring-Up Checklist

When you get simulator access, run this in one terminal:

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Then run your FTG node in a second terminal:

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 run gap_follow reactive_node
```

Optional: override parameters at runtime (no code edits):

```bash
ros2 run gap_follow reactive_node --ros-args \
   -p scan_topic:=/scan \
   -p drive_topic:=/drive \
   -p bubble_radius:=0.20 \
   -p max_range_clip:=3.0 \
   -p smooth_window:=3 \
   -p forward_fov_deg:=160.0 \
   -p fast_speed:=3.0 \
   -p mid_speed:=1.5 \
   -p slow_speed:=0.5 \
   -p fast_threshold:=1.5 \
   -p mid_threshold:=0.5 \
   -p steering_limit:=0.4189 \
   -p steer_slowdown_start:=0.1745 \
   -p steer_slowdown_end:=0.4189 \
   -p min_steer_speed_scale:=0.35
```

### Implementation Notes / Bugs Found During Testing

**Edge-padding in `preprocess_lidar`:**
`np.convolve(mode='same')` uses zero-padding at the array boundaries. This artificially reduces the first and last smoothed values below the true sensor reading. The bubble radius formula `r_b / (d * angle_increment)` is inversely proportional to `d` — a falsely small `d` at the scan boundary creates a massive bubble that zeroes the entire free-space array. Fix: use `np.pad(mode='edge')` before convolution so boundary values are repeated (not zero-padded).

**Empty-gap fallback:**
If the safety bubble zeros every element (possible when obstacles are within ~5cm of the LiDAR — essentially a crash scenario), `find_max_gap` returns `(0, -1)`. The callback detects `end_i < start_i` and publishes `steering=0, speed=SLOW_SPEED` (go straight, slow) rather than computing a garbage steering angle from index -1.

---

## Real Car Deployment Notes

*(To be filled in during Phase 10)*

- Use separate launch file with conservative params
- Start at 1.0 m/s max until stable
- Real LiDAR has more noise than sim → may need larger window or larger `r_b`
- Real VESC has ~20ms actuation latency → do not use very aggressive speed changes
- YouTube link goes in `SUBMISSION.md`
