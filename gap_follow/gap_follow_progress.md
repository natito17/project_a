# Follow the Gap — Session Progress & Next Steps

## Context
This file summarizes progress on running the `gap_follow` ROS2 node in the F1TENTH simulator.
The full simulation setup is documented separately in `/Users/ntryle/f1tenth_sim_cheatsheet.md`.

---

## Package Location

| Location | Path |
|---|---|
| Mac (original source) | `/Users/ntryle/programming/project_a/f1tenth_lab4_template/gap_follow/` |
| Ubuntu host | `~/f1tenth_gym_ros/gap_follow/` (copied via scp) |
| Inside container (volume mount) | `/sim_ws/src/f1tenth_gym_ros/gap_follow/` |
| Inside container (colcon sees it here) | `/sim_ws/src/gap_follow/` ← manually copied, ephemeral |

### Package structure
```
gap_follow/
├── gap_follow/
│   ├── __init__.py
│   └── reactive_node.py   ← the Python node (main implementation)
├── src/
│   └── reactive_node.cpp  ← C++ skeleton (empty template, not used)
├── scripts/
│   └── reactive_node.py
├── CMakeLists.txt
├── package.xml
└── setup.py
```

Entry point: `ros2 run gap_follow reactive_node`
ROS2 node name: `/reactive_node`
Subscribes to: `/scan` (LaserScan, ~40Hz from simulator)
Publishes to: `/drive` (AckermannDriveStamped)

---

## What Was Done This Session

### Step 1 — scp from Mac to Ubuntu host (run on Mac, not SSH)
```bash
scp -r /Users/ntryle/programming/project_a/f1tenth_lab4_template/gap_follow \
    user_124@132.68.55.154:~/f1tenth_gym_ros/
```

### Step 2 — Copy inside container (colcon limitation)
Colcon ignores packages nested inside another package. `gap_follow` inside `f1tenth_gym_ros` was invisible to colcon.
Fix: manually copy one level up inside the container:
```bash
# Inside the container:
cp -r /sim_ws/src/f1tenth_gym_ros/gap_follow /sim_ws/src/
```
**This copy is ephemeral — lost when the container restarts. See "Known Issues" below.**

### Step 3 — Build with colcon (inside container, from /sim_ws)
```bash
cd /sim_ws
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select gap_follow --symlink-install
source install/local_setup.bash
```
Build result: **SUCCESS** (warnings only — unused parameters in the C++ skeleton, harmless).

### Step 4 — Run the node
```bash
ros2 run gap_follow reactive_node
```
Node starts and appears in `ros2 node list` as `/reactive_node`.

---

## Current Status

| Check | Result |
|---|---|
| Node running | ✅ `/reactive_node` appears in `ros2 node list` |
| `/drive` publishing | ❌ `ros2 topic hz /drive` shows nothing after 60+ seconds |
| Car moving in Foxglove | ❌ No movement observed |

**The node is alive but not publishing drive commands.** Root cause unknown — needs debugging.

---

## Diagnostic Commands

All commands run **inside the container** (SSH → `docker exec -it 1e08d1 /bin/bash` → source setup files).

```bash
# Always source these first in any new container shell:
source /opt/ros/humble/setup.bash
source /sim_ws/install/local_setup.bash

# Is the node running?
ros2 node list
# Expected: /reactive_node

# Is it publishing? (wait 10 seconds for first reading)
ros2 topic hz /drive
# Expected: average rate: ~40Hz
# Actual: nothing → node is not publishing

# What is it publishing?
ros2 topic echo /drive --once

# Is the simulator publishing LiDAR?
ros2 topic hz /scan
# Expected: ~40Hz

# Full topic list (check /scan and /drive exist)
ros2 topic list

# Check what the node is subscribed/publishing to:
ros2 node info /reactive_node

# Live log output from the node:
ros2 topic echo /rosout | grep reactive_node
```

---

## Known Issues

### Issue 1: `/drive` not being published (ACTIVE — needs fix)
The node runs but sends no commands. Likely causes to investigate:
1. **`/scan` not being received** — check `ros2 topic hz /scan` is ~40Hz
2. **Speed logic returning 0** — check the speed tier logic in `reactive_node.py`
3. **Exception swallowed silently** — add `print()` statements inside `lidar_callback` to confirm it fires
4. **Topic name mismatch** — verify `/scan` and `/drive` match what the sim publishes:
   ```bash
   ros2 topic list   # confirm /scan and /drive exist
   ros2 node info /reactive_node  # confirm subscriptions
   ```

**First debugging step:** add a print at the top of `lidar_callback` to confirm it's being called:
```python
def lidar_callback(self, scan_msg):
    print("lidar_callback fired")   # add this line temporarily
    ...
```
If this never prints → the subscription isn't receiving `/scan`.
If it prints → the problem is in the algorithm logic.

### Issue 2: Container restart loses the cp (ephemeral copy)
Every time the container restarts, `/sim_ws/src/gap_follow/` disappears because it was manually copied, not volume-mounted.

**Permanent fix — add a second volume mount to the rocker command:**

Option A: move gap_follow to its own folder on Ubuntu host and add second volume:
```bash
# On Ubuntu host:
mv ~/f1tenth_gym_ros/gap_follow ~/gap_follow

# New rocker command (add --volume for gap_follow):
rocker --nvidia --x11 --network=host \
  --volume ~/f1tenth_gym_ros:/sim_ws/src/f1tenth_gym_ros \
  --volume ~/gap_follow:/sim_ws/src/gap_follow \
  -- f1tenth_gym_ros
```
This means edits to `~/gap_follow/` on the Ubuntu host are immediately live inside the container — no cp needed after restart.

Option B (simpler short-term): add the cp to a startup script or just re-run it each session:
```bash
cp -r /sim_ws/src/f1tenth_gym_ros/gap_follow /sim_ws/src/
```

---

## Editing Workflow (Recommended)

**Use VSCode Remote SSH — edit on Ubuntu host, not on Mac.**

1. Install extensions on Mac VSCode: **Remote - SSH** + **Dev Containers**
2. `Cmd+Shift+P` → **Remote-SSH: Connect to Host** → `user_124@132.68.55.154`
3. Open folder `~/f1tenth_gym_ros` (or `~/gap_follow` if you moved it per fix above)
4. Edit `gap_follow/gap_follow/reactive_node.py` directly
5. Because `--symlink-install` was used, Python edits are live immediately — no rebuild needed
6. To test: go to a container shell and run `ros2 run gap_follow reactive_node`

For full debugging with breakpoints:
- After connecting via Remote SSH, also do: `Cmd+Shift+P` → **Dev Containers: Attach to Running Container**
- VSCode reopens inside the container with full debugger support

---

## Next Steps (in order)

1. **Confirm `/scan` is being received** by the node:
   - In the container: `ros2 topic hz /scan` — should show ~40Hz
   - If not: check sim is running (`ss -tlnp | grep 8765`)

2. **Confirm `lidar_callback` fires** by adding a temporary print statement inside it and rerunning the node

3. **Fix the volume mount** (Issue 2 above) so edits survive container restarts — use the two-volume rocker command

4. **Set up VSCode Remote SSH** to edit `reactive_node.py` on Ubuntu directly without scp

5. **Debug the algorithm** — once confirmed the callback fires, step through `preprocess_lidar` → `find_max_gap` → `find_best_point` and verify the gap-finding logic produces a valid output

6. **Watch the car move** in Foxglove at `https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://132.68.55.154:8765`

---

## Key Reference

- Ubuntu host IP: `132.68.55.154`
- Ubuntu user: `user_124`
- Container ID (may change): `1e08d1` — verify with `docker ps`
- Sim cheat sheet: `/Users/ntryle/f1tenth_sim_cheatsheet.md`
- Gap follow source on Mac: `/Users/ntryle/programming/project_a/f1tenth_lab4_template/gap_follow/`

---

## Session Update (2026-06-19) — Live Runtime Verification + Root-Cause Analysis

### Why this update
Goal: verify the *actual running* map/start state from ROS2 (not just file defaults), then map observed behavior to concrete code paths in `reactive_node.py`.

### How the running container was accessed
Used Cheat Sheet "Scenario 5" flow:
```bash
docker ps
docker exec -it <container_id> /bin/bash
source /sim_ws/.venv/bin/activate
source /opt/ros/humble/setup.bash
source /sim_ws/install/local_setup.bash
```

### Live ROS2 facts confirmed in the running sim
```bash
ros2 node list
```
Returned:
- `/bridge`
- `/ego_robot_state_publisher`
- `/foxglove_bridge`
- `/lifecycle_manager_localization`
- `/map_server`

`/reactive_node` was **not** running at the moment of this check.

```bash
ros2 param get /bridge map_path
ros2 param get /bridge sx
ros2 param get /bridge sy
ros2 param get /bridge stheta
ros2 param get /bridge scan_num_beams
ros2 param get /bridge async_mode
```
Runtime values:
- `map_path = maps/levine`
- `sx = 0.0`
- `sy = 0.0`
- `stheta = 0.0`
- `scan_num_beams = 819`
- `async_mode = True`

```bash
timeout 5 ros2 topic hz /scan
```
Measured scan publication rate: **~250 Hz**.

### Decision cadence in the current algorithm
In this implementation, decisions are made in `lidar_callback`, and that callback runs once per `/scan` message.

So in the current running setup:
- Decision cadence is effectively ~250 decisions/second (limited by callback compute + scheduling).

### How steering and speed are currently handled
- Steering:
  - Best beam index is selected as center of max gap.
  - Steering command is computed directly as:
   - `best_angle = angle_min + best_idx * angle_increment`
  - This value is published directly to `/drive` (no explicit clamp/slew in node).

- Speed:
  - Uses tiered function `_choose_speed(min_range)`:
   - if `min_range > 1.5` -> `3.0 m/s`
   - elif `min_range > 0.5` -> `1.5 m/s`
   - else -> `0.5 m/s`
  - `min_range` here is computed from **non-zero** values after bubble masking.

### What could have gone wrong? (ranked list)

1. **Speed too aggressive for reactive-only steering (most likely)**
  - Why likely: controller can command 3.0 m/s whenever effective min-range looks open, but does not reduce speed by steering magnitude/curvature.
  - Symptom match: car initially moves and turns, then cannot complete correction and contacts wall.

2. **Gap search uses full 270 degree scan, so target can drift away from forward-driving intent (very likely)**
  - Why likely: max-gap center over full FOV can bias toward side/backward-open regions, especially near walls.
  - Symptom match: brief left correction then re-centering to a direction that still leads into collision.

3. **No explicit steering clamp/smoothing in the node (likely contributor)**
  - Why likely: raw index-to-angle can jump between scans, creating oscillatory steering at high update rates.
  - Symptom match: short left move followed by opposite correction/straightening.

4. **Bubble placement based on global nearest point can over-prioritize side wall beams (possible)**
  - Why possible: nearest beam is often lateral in corridor-like starts; bubble can reshape free-space topology in a way that is not best for forward progress.

5. **Purely reactive FTG without memory/prediction (possible baseline limit)**
  - Why possible: with no temporal smoothing or trajectory objective, FTG can be unstable in certain geometries.

### Most likely primary culprit
Most likely: **(1) + (2) combined**
- The controller is likely driving too fast for the steering it is choosing, and the chosen target can be influenced by non-forward beams.
- This combination commonly creates exactly the pattern you reported: starts fine, makes a local correction, then fails to avoid a wall ahead.

### Next code changes to try (in this order)
1. Restrict gap search to a forward FOV window first (e.g. +/- 70 to +/- 90 deg).
2. Add steering-aware speed scaling (reduce speed as `abs(steering_angle)` grows).
3. Clamp and smooth steering command before publish.
4. Re-test and log `/drive` steering+speed over 10-20 seconds.

### Implementation update (completed in this session)

Implemented in `gap_follow/gap_follow/reactive_node.py`:

1. **Forward-FOV gap filtering**
  - Added a forward cone parameter and limited FTG processing to that cone.
  - Default: `forward_fov_deg = 160.0` (±80° around forward).
  - Impact: avoids selecting side/rear-biased gaps.

2. **Steering-aware speed scaling**
  - Kept distance-tier base speed logic, then added steering-based downscaling.
  - Defaults:
    - `steering_limit = 0.4189`
    - `steer_slowdown_start = 0.1745`
    - `steer_slowdown_end = 0.4189`
    - `min_steer_speed_scale = 0.35`
  - Impact: speed is reduced in sharper turns, lowering wall-hit risk.

3. **Steering clamp**
  - Steering command is explicitly clamped to ±`steering_limit` before publish.

### Immediate re-test checklist

1. Launch bridge + node, then verify publisher exists:
```bash
ros2 topic info /drive -v
```
Expect at least one `/drive` publisher from `/reactive_node`.

2. Run for 20-30 seconds and observe if the initial straight-wall collision is gone.

3. If still too aggressive in turns:
```bash
ros2 run gap_follow reactive_node --ros-args \
  -p forward_fov_deg:=140.0 \
  -p min_steer_speed_scale:=0.25 \
  -p steer_slowdown_start:=0.12
```

### Environment fix applied (important)

While validating the rebuild, a source-path mismatch was found inside the running container:

- `colcon list` was building package from: `/sim_ws/src/gap_follow`
- Code edits were being made in: `/sim_ws/src/f1tenth_gym_ros/gap_follow`

Result: a normal `colcon build --packages-select gap_follow` did **not** include the newest edits.

Fix applied in container:

```bash
cp /sim_ws/src/f1tenth_gym_ros/gap_follow/gap_follow/reactive_node.py \
   /sim_ws/src/gap_follow/gap_follow/reactive_node.py

cd /sim_ws
colcon build --packages-select gap_follow
source install/local_setup.bash
```

Verification after fix:
- Installed node now contains `FORWARD_FOV_DEG = 160.0`
- Installed node now contains `_scale_speed_for_steering`

Recommendation:
- For future sessions, keep only one active `gap_follow` source path in the container (or mount `~/f1tenth_gym_ros/gap_follow` directly to `/sim_ws/src/gap_follow`) so build output always matches edited files.

### Session note: map-intent ambiguity

- Levine in this workspace has no local `*_centerline.csv`/`*_raceline.csv` reference file alongside `maps/levine.*`.
- Pure FTG therefore optimizes local free-space, not a global intended corridor, which explains wrong-branch decisions in open corner zones.
- Consolidated guidance for future sessions is documented in `LAB4_GAP_FOLLOW_REFERENCE.md`.

### Session note: topic-based debug workflow

- Added `LECTURE_5_6_ACCESS_AND_DEBUG_GUIDE.md` with a terminal-first checklist for `/scan`, `/drive`, `/ego_racecar/odom`, connectivity checks, and decision capture via rosbag.
- This is intended to reduce dependence on laggy Foxglove rendering during tuning.

### Session note: lecture-based upgrade plan

- Parsed Lecture 5 and Lecture 6 PDFs from `docs/lectures/` and updated `LECTURE_5_6_ACCESS_AND_DEBUG_GUIDE.md`.
- Added a prioritized improvement plan based on lecture guidance:
  1. turn-commitment hysteresis at corner entry,
  2. disparity-aware masking,
  3. side-beam safety guard while turning,
  4. decision telemetry for reproducible tuning.

### Update (2026-06-20, Improvement 1 implemented)

Implemented turn-commitment hysteresis in `gap_follow/gap_follow/reactive_node.py`.

What changed:
1. Front blockage detection
  - Added front-sector minimum check with threshold parameter.
2. Side commit decision
  - At blockage entry, choose `left` or `right` from side-sector score (mean valid range).
3. Hysteresis counter
  - Hold chosen side for `turn_commit_hold_scans` callbacks to avoid left/right flip-flopping.
4. Integration with existing logic
  - Commitment is applied before max-gap search by masking the opposite half.
  - Existing forward-FOV filter and steering-aware speed scaling remain active.
5. Lightweight debug output
  - Added throttled info logs with side, counter, front/left/right minimums, and side scores.

New ROS params:
- `turn_commit_front_min_threshold` (default `1.0`)
- `turn_commit_front_sector_deg` (default `30.0`)
- `turn_commit_side_sector_deg` (default `70.0`)
- `turn_commit_hold_scans` (default `8`)
- `turn_commit_debug` (default `true`)
- `turn_commit_log_stride` (default `8`)

Syntax validation done:
- Pylance syntax check reports no Python syntax errors in `reactive_node.py`.

Run commands (container):

```bash
source /opt/ros/humble/setup.bash
source /sim_ws/install/local_setup.bash

ros2 run gap_follow reactive_node --ros-args \
  -p turn_commit_front_min_threshold:=1.0 \
  -p turn_commit_front_sector_deg:=30.0 \
  -p turn_commit_side_sector_deg:=70.0 \
  -p turn_commit_hold_scans:=8 \
  -p turn_commit_debug:=true \
  -p turn_commit_log_stride:=8
```

Path mismatch safeguard (important):
- If `colcon list` shows source path `/sim_ws/src/gap_follow` while edits were made in `/sim_ws/src/f1tenth_gym_ros/gap_follow`, sync before build:

```bash
cp /sim_ws/src/f1tenth_gym_ros/gap_follow/gap_follow/reactive_node.py \
  /sim_ws/src/gap_follow/gap_follow/reactive_node.py
cd /sim_ws
colcon build --packages-select gap_follow
source install/local_setup.bash
```
