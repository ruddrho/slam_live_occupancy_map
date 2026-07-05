# Project Report

## Title

**Autonomous Obstacle-Avoiding Differential Drive Robot Using LIDAR and Dynamic Window Approach (DWA)**

## Objective

The project develops a simulation-based mobile robot that autonomously reaches a user-selected destination while avoiding walls and obstacles. Navigation is purely local. No predefined path, global planner, waypoint list, spline, or line-following method is used.

## Differential-drive model

The commanded body velocities are linear velocity `v` and angular velocity `omega`.

The left and right wheel angular velocities are:

```text
omega_left  = (v - omega*L/2)/r
omega_right = (v + omega*L/2)/r
```

where `L` is wheel separation and `r` is wheel radius.

The pose is integrated exactly for constant commands, producing straight motion or circular arcs without side slip.

## LiDAR model

The sensor scans 360 degrees. Wall, rectangle, and polygon intersections use exact ray-to-segment calculations. Cylindrical obstacles use exact ray-to-circle calculations. Each scan returns ranges, endpoints, and hit points.

## Dynamic Window Approach

At every control cycle, DWA:

1. calculates dynamically reachable linear and angular velocities,
2. samples velocity candidates,
3. predicts the motion produced by each candidate,
4. rejects trajectories that violate geometric or stopping-distance safety,
5. scores valid trajectories,
6. applies the highest-scoring command.

The score contains:

- goal heading,
- direct progress,
- obstacle clearance,
- forward velocity,
- smooth command variation,
- steering moderation,
- braking safety,
- final goal proximity.

## Human-like avoidance

The robot detects obstacles early through LiDAR, reduces speed when stopping clearance becomes limited, and executes smooth circular turns. A short local recovery mode changes the DWA weights when distance-to-goal progress stalls. Recovery still uses only the clicked goal and current LiDAR scan.

## Visualization

The visualization displays:

- walls and obstacles,
- circular robot body,
- left and right wheels,
- heading arrow,
- LiDAR rays,
- LiDAR hit points,
- red-star goal,
- sparse historical position dots,
- speed, turn rate, wheel rates,
- simulation time,
- goal distance,
- predicted clearance,
- DWA candidate count.

No global route line is displayed.


## Hybrid navigation correction

The original goal-directed DWA could stop in local minima because it had
no knowledge of how to travel around large obstacle groups. The corrected
architecture uses A* on a safety-inflated occupancy grid to obtain global
connectivity. A monotonic look-ahead target provides continuous path
tracking, while DWA remains responsible for dynamic feasibility, smooth
steering, LiDAR clearance, and safe braking. Automatic replanning is
triggered by excessive cross-track error or insufficient path progress.


## Continuous projection path tracking

Robot position is projected onto the nearest admissible path segment and
stored as a monotonic arc-length coordinate. The reference target is then
interpolated at a speed-dependent forward distance. DWA trajectory scores
include a Gaussian centreline term, reducing corner cutting and lowering
cross-track error.

## Live SLAM occupancy mapping

The robot maintains an independent pose estimate using noisy wheel
odometry and local correlative scan matching. Each 360-degree LiDAR ray
updates a log-odds occupancy grid: traversed cells are marked free and
detected endpoints are marked occupied. The live map panel displays
unknown, free, and occupied regions as the robot explores.


## High-accuracy route tracking

The planned route is resampled at 0.08 m intervals. A bounded cross-track
correction modifies the look-ahead bearing, while the DWA cost evaluates
the RMS distance of the full predicted trajectory from a local route
window. Additional heading-alignment and local-progress terms reduce
corner cutting and improve convergence to the path centreline.
