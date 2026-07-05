# MPPI Controller Progress Log

Last updated: 2026-07-05

## Goal
Track implementation status, build/run/debug workflow, simulator integration, known failures, and decisions for the `mppi_controller` package.

## Scope
- This document analyzes existing MPPI code and runtime behavior.
- No existing files inside `mppi_controller` were modified for this analysis step.

## Package Snapshot (Current)

### Files reviewed
- `mppi_controller/mppi_controller/mppi_node.py`
- `mppi_controller/setup.py`
- `mppi_controller/package.xml`
- `mppi_controller/setup.cfg`

### What the node does
- Creates ROS2 node `mppi_node`.
- Subscribes to:
  - `/scan` (`sensor_msgs/LaserScan`)
  - `/odom` (`nav_msgs/Odometry`)
- Publishes:
  - `/drive` (`ackermann_msgs/AckermannDriveStamped`)
- Runs control loop every `DT = 0.1s` (10 Hz).
- Uses `pytorch_mppi.MPPI` with bicycle dynamics (`x, y, yaw, v`) and control (`steering, acceleration`).
- Builds a local obstacle cloud from LiDAR and uses it in the running cost.
- Chooses heading target from the farthest LiDAR beam (greedy local target).

## MPPI Configuration (Current)
- State dimension `nx=4`
- `num_samples=200`
- `horizon=20`
- `lambda_=1.0`
- Control limits:
  - steering in `[-0.4, 0.4]`
  - acceleration in `[-1.5, 1.5]`
- Noise covariance:
  - steering variance ~ `0.15`
  - acceleration variance ~ `0.5`

## Running Cost Terms (Current)
- Speed reward: encourages higher velocity.
- Steering penalty: discourages large steering magnitude.
- Heading penalty: penalizes yaw error relative to target yaw.
- Stop penalty: penalizes low speed (`v < 0.2`).
- Obstacle penalty:
  - soft wall penalty under 1.5 m
  - hard critical penalty under 0.35 m

## Why the Car Can Move but Still Hit Walls
Likely contributing causes in current implementation:
1. Target selection is local-greedy (farthest beam), not map/waypoint aware.
2. High speed incentive can dominate in ambiguous geometry.
3. Obstacle penalty thresholding can react too late in sharp turns.
4. 10 Hz control may be too slow for aggressive dynamics.
5. No explicit terminal cost, raceline tracking, or progress reward.

## Direct Answers to Current Questions

### 1) Should rocker mount include `mppi_controller` as a volume?
Short answer: usually **no extra volume is needed** if you already run:
```bash
rocker --nvidia --x11 --network=host --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```
Because the whole repository is already mounted, `mppi_controller` is inside that mount.

Practical check inside container:
```bash
cd /sim_ws
source /opt/ros/humble/setup.bash
colcon list | grep mppi_controller
```
If listed, mount is sufficient.

### 2) Should we ignore `gap_follow` folders now?
Short answer: **do not remove yet**. Keep both controllers for A/B testing and fallback.

Recommended now:
- Keep `gap_follow` present.
- Choose active controller at run time (run one node at a time publishing to `/drive`).
- Later, archive deprecated controllers only after MPPI is stable and benchmarked.

### 3) Is `PYTHONPATH` issue the same as the earlier python3/shebang issue?
Short answer: **related but different layer**.

- Shebang/interpreter issue (fixed by `sed`) chooses *which Python executable* runs a script.
- `PYTHONPATH` issue chooses *where that Python looks for importable modules*.

So:
- `sed` fix solved wrong interpreter for `gym_bridge`.
- `export PYTHONPATH=...venv_mppi...` allowed imports (e.g., torch) from another environment path.

Best practice:
- Install `torch` and `pytorch-mppi` directly in `/sim_ws/.venv` used by container runtime.
- Avoid cross-venv imports via `PYTHONPATH` except as temporary workaround.

Optional consistency workaround (same style as gym_bridge):
```bash
sed -i '1s|#!/usr/bin/python3|#!/sim_ws/.venv/bin/python3|' \
  /sim_ws/install/mppi_controller/lib/mppi_controller/mppi_node
```
Use this only after confirming the file exists post-build.

### 4) About your partner's run sequence and wall collision
The sequence is mostly correct and explains why the car moved. Collision behavior is consistent with current MPPI objective design (local target + cost balance), not only a launch mistake.

Most useful immediate debug next steps are listed below.

### 5) Should you send the Hebrew summary?
**Yes.** Send it. It is valuable for reconstructing failed branches, exact error messages, and decision history. I can integrate it into this progress file as a dated troubleshooting timeline.

## Build and Run Procedure (Linux Host)

### Terminal 1: launch simulator
```bash
cd ~/f1tenth_gym_ros
rocker --nvidia --x11 --network=host --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros

cd /sim_ws
source /opt/ros/humble/setup.bash
source /sim_ws/.venv/bin/activate
colcon build --packages-select f1tenth_gym_ros mppi_controller --symlink-install
source install/setup.bash

sed -i '1s|#!/usr/bin/python3|#!/sim_ws/.venv/bin/python3|' \
  /sim_ws/install/f1tenth_gym_ros/lib/f1tenth_gym_ros/gym_bridge

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2: run MPPI node in same container
```bash
docker ps
docker exec -it <container_name_or_id> /bin/bash

cd /sim_ws
source /opt/ros/humble/setup.bash
source /sim_ws/.venv/bin/activate
source install/setup.bash

# Recommended: only if torch is installed in /sim_ws/.venv
ros2 run mppi_controller mppi_node \
  --ros-args -r odom:=/ego_racecar/odom -r scan:=/scan -r cmd_vel:=/drive
```

If torch is not in `/sim_ws/.venv`, temporary fallback:
```bash
export PYTHONPATH="/sim_ws/src/f1tenth_gym_ros/venv_mppi/lib/python3.10/site-packages:$PYTHONPATH"
ros2 run mppi_controller mppi_node \
  --ros-args -r odom:=/ego_racecar/odom -r scan:=/scan -r cmd_vel:=/drive
```

Note: Current node publishes directly to `/drive` in code, so the `cmd_vel:=/drive` remap is not used by this node as written.

## Build and Run Procedure (Mac via SSH)

### Mac Terminal 1: simulator on remote host
```bash
ssh user_124@<ubuntu-ip>
cd ~/f1tenth_gym_ros
rocker --nvidia --x11 --network=host --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros

cd /sim_ws
source /opt/ros/humble/setup.bash
source /sim_ws/.venv/bin/activate
colcon build --packages-select f1tenth_gym_ros mppi_controller --symlink-install
source install/setup.bash

sed -i '1s|#!/usr/bin/python3|#!/sim_ws/.venv/bin/python3|' \
  /sim_ws/install/f1tenth_gym_ros/lib/f1tenth_gym_ros/gym_bridge

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Mac Terminal 2: attach and run MPPI
```bash
ssh user_124@<ubuntu-ip>
docker ps
docker exec -it <container_name_or_id> /bin/bash

cd /sim_ws
source /opt/ros/humble/setup.bash
source /sim_ws/.venv/bin/activate
source install/setup.bash

ros2 run mppi_controller mppi_node \
  --ros-args -r odom:=/ego_racecar/odom -r scan:=/scan -r cmd_vel:=/drive
```

Foxglove URL from Mac browser:
```text
https://app.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://132.68.55.154:8765
```

## Fast Debug Checklist
Run in container terminal:
```bash
source /opt/ros/humble/setup.bash
source /sim_ws/.venv/bin/activate
source /sim_ws/install/setup.bash
```

Then check:
```bash
ros2 node list
ros2 node info /mppi_node
ros2 topic hz /scan
ros2 topic echo /ego_racecar/odom --once
ros2 topic info /drive -v
ros2 topic echo /drive --once
```

If imports fail:
```bash
python3 -c "import torch; import pytorch_mppi; print('ok')"
```

If node runs but drives into wall:
- Reduce speed incentive weight.
- Increase near-obstacle penalties and safety radius.
- Increase control rate (e.g., `DT=0.05`), if stable.
- Replace farthest-beam target with waypoint/progress target.
- Add steering-rate penalty and action smoothing.

## Known Technical Risks in Current MPPI Package
1. Missing runtime dependency declarations in `mppi_controller/package.xml` for required ROS messages and `rclpy`.
2. External Python dependencies (`torch`, `pytorch_mppi`) not encoded in packaging files.
3. Hardcoded topics in code reduce flexibility unless remapping is explicitly used in code paths.
4. No parameterization of major MPPI/cost constants through ROS parameters.
5. No unit/integration tests for controller behavior.

## Decision Log Template (Use Going Forward)
### Date
### Change attempted
### Error observed
### Root cause
### Fix applied
### Verification command(s)
### Result
### Follow-up action

## Next Planned Update
- Integrate your partner's Hebrew troubleshooting log as a timeline section with exact command/output pairs.

## Cheat Sheet Growth Recommendation
As modules grow, keeping everything in one giant cheat sheet becomes hard to search and maintain.

Recommended structure:
1. Keep one short top-level index (quick start + links).
2. Split by controller/module:
  - `f1tenth_sim_cheat_sheet.md` (core simulator operations only)
  - `gap_follow/gap_follow_progress.md` and a short `gap_follow` runbook
  - `mppi_controller/mppi_progress.md` and `mppi_controller/mppi_controller_brief.md`
3. Add one shared troubleshooting file for cross-cutting issues (ROS env sourcing, rocker, Foxglove, shebang, permissions).
4. Keep each file to one purpose:
  - runbook (exact commands)
  - progress log (what happened, why, what next)
  - concept brief (theory + strengths/weaknesses)

This gives faster onboarding, less duplication, and cleaner history of decisions.
