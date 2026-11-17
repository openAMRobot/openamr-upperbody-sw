# Station approach: tolerances from the arm reach (Proposed, 17 Nov 2026)

Simulation-derived working values for the Build 1 work-shelf grasp. Not a site, robot or safety tolerance.

## Problem

Baseline 1 run 44076639872: Nav2 SUCCEEDED with the base 0.340 m short of `stations.work_shelf`. The global goal
checker of openamr-platform-sw accepts 0.35 m, and Smac2D with downsampling ends its path on a 0.10 m cell centre.
From there the item was out of reach (OMPL: no valid goal state).

## Reach

`tools/build1_diagnosis/reach_probe.py` on the expanded Build 1 URDF (gazebo profile, #70 at 77bbf18; the arm, lift
and base frames do not differ in the later heads): damped least-squares inverse kinematics of `openarm_left_joint1..7`,
joint limits minus 0.02 rad, 8 seeds, for the three hand poses of scn04 at the lift's L1000 stop: approach (0.10 m
back), grasp (TCP 0.03 m above the item centre) and lift (+0.08 m), with the side-grasp orientation and MoveIt's
tolerances (0.01 m and 0.3 rad; lift 0.015 m). Collisions are not checked. At the station the item is at
x 0.750, y 0.160, z 0.895 m in `base_footprint`.

Base pose error from the station, one axis at a time (all three poses reachable):

| Axis | Reachable |
|---|---|
| along the heading (+ is closer to the shelf) | -0.06 to +0.12 m |
| to the left | -0.20 to +0.15 m |
| yaw | -0.30 to +0.20 rad |

Combined (16 directions in the plane x yaw -a, 0, +a): 0.03 m / 0.05 rad 48 of 48, 0.03 m / 0.08 rad 48 of 48,
0.04 m / 0.05 rad 48 of 48, 0.04 m / 0.08 rad 48 of 48, 0.05 m / 0.10 rad 44 of 48, 0.06 m / 0.10 rad 44 of 48.
The binding side is a base too far from the shelf: the item is near the far edge of the arm's reach.

```
python3 tools/build1_diagnosis/reach_probe.py <expanded.urdf> axes
python3 tools/build1_diagnosis/reach_probe.py <expanded.urdf> circle 0.04 0.08 16
```

## Tolerances

- Reach budget (true base pose against the station): 0.04 m and 0.08 rad. The mission checks it from Gazebo ground
  truth (`station_reach_position_error`, `station_reach_yaw_error`).
- Station goal checker (openamr-platform-sw #164): 0.025 m and 0.05 rad. The difference, 0.015 m and 0.03 rad, is left
  to localization (AMCL map pose against ground truth), which the mission records as
  `nav_localization_minus_ground_truth` and `nav_localization_yaw_minus_ground_truth`.
- Global Nav2 tolerances are unchanged (0.35 m, 0.35 rad).

## Mechanism

openamr-platform-sw #164, inside Nav2: `NavigateToPose.behavior_tree` selects `navigate_to_station.xml`, which runs the
default navigation, then one `StationPlanner` (NavFn) path that ends at the goal pose and `StationApproach`
(nav2_graceful_controller) with `station_goal_checker`. No other velocity publisher.

## Localization uncertainty at the station (measured, 17 Nov 2026)

Configuration: openamr-platform-sw #161 with #164 (`station_goal_checker` 0.025 m / 0.05 rad, AMCL updating every
0.05 m / 0.05 rad), firmware_sil, hosted runners at a real-time factor of 0.13 to 0.25. Every value is taken after the
base has settled, from Gazebo ground truth. Statistics: `tools/build1_diagnosis/loc_stats.py` on the job logs; the 95 %
bound is a one-sided prediction bound for the next run, mean + t(0.95, n-1) x sd x sqrt(1 + 1/n).

Runs: 44105673151, 44107726177, 44110378874, 44110449067, 44114692549, 44114738411 (`a3eee67` to `d1c88be`) and
44130637433, 44130671011, 44130685870, 44130700211, 44130714431, 44130728617 (`fc1fe22`, which also records the AMCL
covariance and the beams with a return).

| Quantity | n | Mean | Max | 95 % bound |
|---|---|---|---|---|
| AMCL map pose (/tf) minus ground truth, position | 12 | 0.036 m | 0.058 m | 0.062 m |
| same, heading | 12 | 0.007 rad | 0.016 rad | 0.017 rad |
| last /amcl_pose minus ground truth, position | 6 | 0.029 m | 0.048 m | 0.053 m |
| last /amcl_pose minus ground truth, heading | 6 | 0.023 rad | 0.050 rad | 0.064 rad |
| AMCL 1-sigma x / y / yaw | 6 | 0.053 m / 0.114 m / 0.073 rad | 0.078 m / 0.128 m / 0.085 rad | |
| AMCL updates during the navigation | 6 | 68 | 77 | |
| odometry minus ground truth, position | 12 | 0.065 m | 0.079 m | 0.081 m |
| odometry minus ground truth, heading | 12 | 0.032 rad | 0.076 rad | 0.067 rad |
| LiDAR beams with a return at the station (of 360) | 6 | 86 | 91 | |
| where Nav2 believes the base is, against the goal | 12 | 0.026 m | 0.048 m | 0.046 m |
| **base against the station (ground truth), position** | 12 | **0.054 m** | **0.080 m** | **0.088 m** |
| **base against the station (ground truth), heading** | 12 | **0.023 rad** | **0.064 rad** | **0.052 rad** |

Against the reach budget (0.04 m, 0.08 rad):
- **Position: the bound exceeds the budget.** Localization alone has a 95 % bound of 0.062 m, four times the 0.015 m it
  was allotted, and AMCL's own 1-sigma (0.05 to 0.13 m) is larger than the budget. The base stood within 0.04 m of the
  station in 2 of 12 runs. The precise Nav2 controller does its part (Nav2's own estimate is within 0.025 m when it
  accepts); the error is in the map pose it is given. At the station 83 to 91 of the 360 LiDAR beams have a return (279 have none at the origin).
- **Heading: within the budget** (95 % bound 0.052 rad against 0.08 rad).
- This is the case for aligning the final pose to the shelf itself (Worker M's station marker / shelf-geometry
  alignment): AMCL in this scene cannot place the base within 0.04 m. AMCL was not tuned for this scene.
- The arm reached and grasped in runs with up to 0.070 m station error: the probe's budget is conservative (it allows
  no collision-free margin beyond IK). The budget is not widened here: that needs the reach probe with collisions or an
  owner decision.

## Not verified

- Collisions in the IK (the shelf board above, the tote wall): MoveIt checks them in the run.
- MoveIt's own IK solver may fail closer to the reach edge than this probe.
- Other stations, items or lift stops: the derivation is for this grasp only.
