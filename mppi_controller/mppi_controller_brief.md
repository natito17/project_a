# MPPI Controller Brief

## What MPPI Is
MPPI (Model Predictive Path Integral control) is a sampling-based model predictive controller.
At each control cycle it:
1. Samples many candidate control sequences.
2. Rolls them forward through a vehicle dynamics model.
3. Scores each trajectory with a cost function.
4. Chooses a weighted best action for immediate execution.

In this project, the state is `(x, y, yaw, v)` and the control is `(steering, acceleration)`.

## What This MPPI Node Currently Does
- Uses odometry + LiDAR to estimate current state and nearby obstacles.
- Selects a local heading target from the farthest LiDAR beam.
- Optimizes steering and acceleration with MPPI.
- Publishes Ackermann command on `/drive`.

## Advantages
1. Handles nonlinear dynamics naturally.
2. Works with non-smooth costs (obstacle penalties, piecewise penalties).
3. Can run on GPU (`torch` CUDA) for faster sampling.
4. Flexible: behavior can be reshaped by cost terms without redesigning planner logic.

## Disadvantages
1. Sensitive to cost tuning; wrong weights can cause unstable or unsafe behavior.
2. Computationally heavier than simple reactive methods.
3. Local objective can get trapped in poor decisions (e.g., wall attraction in open spaces).
4. Requires consistent environment/dependency setup (torch, pytorch_mppi).

## Strengths in Current Implementation
1. Clean and compact structure.
2. Correct base bicycle dynamics formulation for low-level control.
3. Includes both soft and hard collision penalties.
4. Already integrated with ROS2 topics used by simulator.

## Weaknesses in Current Implementation
1. Target selection is purely local (farthest LiDAR ray), no global route objective.
2. Cost and MPPI constants are hardcoded (not ROS params).
3. No explicit action-rate/smoothness penalty.
4. Control loop at 10 Hz may be low for aggressive maneuvers.
5. Dependency management is fragile (external venv/PYTHONPATH workaround).

## How to Improve Path Quality
1. Add waypoint/raceline progress term to cost (not only heading).
2. Increase predictive fidelity: include steering dynamics or tire/slip approximation.
3. Add action smoothing and steering-rate limits in cost.
4. Tune safety margins by speed (dynamic safety radius).
5. Raise controller frequency if compute budget allows.
6. Use adaptive sampling: more samples near difficult geometry.
7. Expose all key constants as ROS parameters and tune per map.
8. Add quantitative evaluation metrics (lap time, collision count, min obstacle distance, control smoothness).

## How to Improve Reliability
1. Install torch + pytorch_mppi in the same runtime environment as ROS node (`/sim_ws/.venv`).
2. Declare runtime dependencies in package metadata.
3. Add startup diagnostics in node logs (device, topic rates, import/version checks).
4. Add offline replay tests for known corner cases.

## Toward a "Best Possible" MPPI for This Project
A strong final architecture usually combines:
- Global planner: route/raceline target.
- Local MPPI: short-horizon dynamic optimization around that target.
- Safety layer: emergency braking / control barrier constraints.
- Continuous tuning loop with map-specific benchmarks.

This hybrid design tends to outperform a purely local LiDAR-target MPPI in both speed and robustness.
