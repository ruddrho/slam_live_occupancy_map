# Hybrid A* + LiDAR + Dynamic Window Approach Robot Navigation

This corrected project solves the local-minimum problem of pure DWA by
combining three layers:

1. **A* global path planning** on a safety-inflated occupancy grid.
2. **Continuous monotonic look-ahead path following**.
3. **Real-time LiDAR + DWA local motion control**.

## Why the previous version stopped

DWA is a local planner. In environments with long walls, U-shaped
obstacles, or goals hidden behind several barriers, direct goal scoring
can lead to a local minimum. The robot may safely stop because every
short prediction appears worse than remaining stationary.

## New behavior

- The clicked goal is converted into a collision-free A* route.
- The path is simplified and uniformly resampled.
- A look-ahead target continuously advances along the route.
- DWA plans smooth velocity commands toward the current look-ahead target.
- The robot slows before corners.
- LiDAR and DWA still reject unsafe local trajectories.
- Cross-track deviation or lack of progress triggers automatic replanning.
- A stationary-command fallback rotates the robot toward the route.
- Collision protection holds or turns the robot instead of ending the mission.
- Terminal docking completes the last part of the route quickly.

## Run

```matlab
clear functions
clear
clc
close all
main
```

Click a collision-free destination. The blue dashed line is the global
A* route, the magenta point is the current continuous look-ahead target,
and the black dots show actual robot history.

## New modular files

- `buildOccupancyGrid.m`
- `planPathAStar.m`
- `postProcessPath.m`
- `pathFollower.m`

## Output

Generated in `results/`:

- `hybrid_astar_dwa_navigation.avi`
- `final_hybrid_navigation.png`
- `hybrid_navigation_performance.png`
- `hybrid_navigation_metrics.csv`
- `hybrid_navigation_results.mat`


## Accurate path tracking update

The path follower now projects the robot onto continuous path segments
rather than selecting the nearest stored point. Progress is measured in
metres of path arc length, the look-ahead distance changes with robot
speed and tracking error, and DWA receives an additional centreline
tracking score. The main panel shows the planned route, actual travelled
line, projected path point, and moving look-ahead target.

## SLAM-live occupancy map

A second live panel builds an occupancy map directly from LiDAR scans.
The implementation includes:

- noisy differential-drive odometry prediction,
- lightweight correlative scan matching,
- log-odds free-space updates,
- log-odds occupied-cell updates,
- unknown/free/occupied visualization,
- estimated robot trajectory and pose,
- explored-area percentage.

The final map is exported as `slam_live_occupancy_map.png`.


## Ultra-accurate path-line tracking

The tracking layer now uses a denser path, a shorter speed-adaptive
look-ahead, bounded cross-track steering correction, and a DWA objective
that evaluates every point of each predicted trajectory against a local
window of the A* route. Candidate commands are also scored for path
heading alignment and progress toward the moving reference target.

In the main map:

- Blue dashed line: planned A* path
- Red solid line: actual robot trajectory
- Cyan point: projected centreline position
- Magenta point: corrected moving look-ahead target


## A* route-lock correction

The previous cross-track steering correction used the wrong sign. When
the robot moved to the right side of the directed A* route, it was
commanded farther right. The corrected controller subtracts the signed
cross-track correction, steering the robot back toward the route.

DWA now also applies a route-lock constraint. A predicted trajectory is
rejected when it leaves the configured corridor around the safety-inflated
A* route. If the robot is already outside the corridor, DWA accepts only
commands that maintain or reduce the final route deviation.


## Runtime route-lock fix

`routeLockSafe` is now evaluated only after the complete predicted
trajectory has been compared with the A* reference route. This removes
the undefined-variable error while preserving the route-lock constraint.


## A* route animation

After the destination is selected, the map now displays an A* animation
similar to the supplied reference video:

1. Blue search-tree edges and cyan expanded nodes appear progressively.
2. The final collision-free route is revealed in bright green.
3. Temporary search graphics are removed.
4. The existing DWA, route-lock, SLAM, and robot animation starts exactly
   as before.

The A* sequence is saved separately as:

```text
results/astar_route_animation.avi
```

This feature changes visualization only; path planning and navigation
results are unchanged.


## Faster A* animation

The A* visualization now runs approximately two to three times faster:
search nodes are grouped into larger frame updates, the final route uses
fewer reveal frames, playback is recorded at 30 FPS, and the final hold
is reduced to 0.25 seconds. Navigation and controller behavior are
unchanged.


## Combined video output

In addition to the separate videos, the project now creates one combined
split-screen video frame:

- Left panel: A* search and route animation
- Right panel: Hybrid A* + DWA navigation

The shorter panel automatically holds its last frame while the longer
panel continues, so the full navigation can be watched in one video.

Output file:

```text
results/combined_astar_navigation.avi
```


## SLAM endpoint alignment fix

The former magenta point was the drifted SLAM pose estimate, not a second
destination. The occupancy map is now anchored to the simulation world
frame, so the map, A* route, robot trajectory, and clicked goal use the
same coordinates.

The exported occupancy map now shows:

- one red-star goal only,
- the actual robot trajectory,
- no misleading magenta endpoint,
- a trajectory endpoint aligned with the goal after successful arrival.

The live SLAM panel uses a blue triangle for the current robot pose.


## Label visibility and fast post-goal output fix

All dashboard labels now use dedicated fixed panels:

- one heading panel above the maps,
- a SLAM map panel in the upper-right,
- a complete navigation and legend panel in the lower-right.

No status label is placed on top of the navigation map.

After the robot reaches the goal, the performance figure is created and
displayed immediately. High-resolution image saving, MAT export, SLAM
export, and combined-video creation run afterward. The combined-video
composer is also faster because it reads source videos sequentially and
renders its title band only once.
