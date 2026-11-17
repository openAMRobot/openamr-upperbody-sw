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

## Not verified

- Collisions in the IK (the shelf board above, the tote wall): MoveIt checks them in the run.
- MoveIt's own IK solver may fail closer to the reach edge than this probe.
- Other stations, items or lift stops: the derivation is for this grasp only.
