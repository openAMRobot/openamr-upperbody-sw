# Gazebo base and lift: ground truth now follows the commands, 10 Nov 2026

Simulation evidence only. Nothing here says anything about the physical robot. Same runner class and software versions as
`2026-11-09.md` (ubuntu-24.04, ROS 2 Jazzy, Gazebo Sim 8 Harmonic, gz-physics DART 6.13.2 from gz_dartsim_vendor, no GPU, headless).
Same workspace: this repository plus the pins of `config/upstream_pins.yaml`.

## The defect (9 Nov)

In DART, the default engine, the base and lift_joint did not move in the simulator's own poses. Meanwhile `/odom` reported about 1 m
and the lift's FollowJointTrajectory goal reported success. G01 and the integrated smoke checked odom and the action result only.

## Root causes

These are two independent causes. I bisected on a minimal harness: the same converted model in a plain `gz sim -s` world, driven over gz
transport, with ros2_control systems and joints removed one at a time.

1. **Base: the wheel and caster friction never reached the physics.** platform-sw's `openamrobot_description/urdf/gazebo_control.xacro`
   (pin `131e500`) writes the drive-wheel friction (mu 1.5, mu2 0.5, fdir1 `1 0 0`) and the frictionless casters (mu 0) as a raw
   `<surface>` inside `<gazebo reference="...">`. sdformat's URDF parser does not apply that `<surface>` to the link's collisions.
   After `gz sdf -p`, no wheel collision has any `<surface>`. Every contact was mu 1, and the four caster spheres were lumped into
   `base_footprint` by their fixed joints. They dragged at mu 1 as hard as the drive wheels pushed, so the wheels spun in place,
   and DiffDrive integrated the wheel speed into odom.
   - Evidence: with the arms' and lift's ros2_control removed, the bare model moved about 1.4 m in ground truth against 7.3 m of odom (slip).
     With mu 0 on the casters and platform-sw's wheel values, it moved 7.27 m against 7.18 m.
   - Interaction: in the full model, with all 18 joints servo-held, the base did not move at all. This needed both arms' joint1
     held; either arm alone was not enough. With the friction fixed, the arms make no difference.
2. **Lift: DART does not apply a servo command to a joint on a limit.** gz_ros2_control drives a position interface in DART by
   setting a velocity command on a SERVO actuator. DART 6.13 `constraint/JointConstraint::update()` checks, in order, the position
   limits, then the velocity limits, then the servo. It `continue`s out of the loop as soon as a limit constraint is active.
   lift_joint starts at 0.0, which is its lower limit. Gravity gives the carriage a small downward velocity there on every step, so a
   limit constraint was active on every step and the command never reached the joint. The controller saw a hold error under its
   goal tolerance, which is why the goal succeeded at 0.0.
   - Evidence: the lift stayed at 0.0 with the full model in DART (dantzig and pgs solvers, ODE, FCL and Bullet collision detectors),
     with the arm interfaces removed, and with the base friction fixed. With the lower limit at -0.001 m it moved to every stop.
     With the joint started at 0.002 m but the limits unchanged, it also stayed. So only being at the limit matters.

## Fix (this repository, gazebo profile only)

- `openamrobot_upperbody_description/urdf/base_contact_gazebo.xacro` restates platform-sw's own friction values as `<mu1>`,
  `<mu2>` and `<fdir1>` under `<gazebo reference>`, which the conversion applies to every collision of the link, the lumped caster
  collisions included. No new number. TODO(Gazebo digital twin owners, P-03 D-12): make the same change in platform-sw's `gazebo_control.xacro`. When the pin
  carries it, delete this file. Test: `test_base_contact_gazebo.py`.
- `combined_robot.xacro` adds `lift_sim_margin`, 0.001 m in the gazebo profile and 0 otherwise. lift_joint's simulated limits
  become -0.001 to 0.351 m. This is a numerical simulation margin, not a hardware value. mock and real keep 0 to 0.350 m exactly
  (`test_t04_optb_composition.py`).

## Measured after the fix (DART, `robot_sim.launch.py headless:=true`, `scripts/sim_smoke_measure.py` and `test_integrated_sim_smoke.py`)

| move | commanded | /odom | ground truth (gz dynamic_pose) |
|---|---|---|---|
| forward at 0.2 m/s, three runs | 1.0 m | 0.980 / 0.953 / 0.961 m | 0.980 / 0.953 / 0.961 m (odom - truth < 1e-8 m) |
| rotate at 0.4 rad/s | 90 deg | 86.2 deg | 88.4 deg |
| lift L1000 -> L1175 -> L1350 -> L1000 | goals succeed (14.5 / 14.5 / 28.4 s sim) | lift_joint error < 8e-5 m | lift_carriage_link rise 0.17498 / 0.17498 / -0.34996 m |
| both arms to `raised`, then `park` | success, 5.3 to 5.4 s each | final error < 1e-3 rad | not read |

Odom equals ground truth to 1e-8 m on a straight line. That is expected now that the wheels do not slip: DiffDrive's odom and the
chassis follow the same wheel motion. One measurement run gave 0.219 m in both odom and ground truth. A late `/cmd_vel` subscription
lost the start of the command, so the physics was not the cause. `sim_smoke_support._drive` now waits for the subscriber.

Real-time factor about 0.36 on this runner.

## Tests that now judge ground truth

- `openamrobot_upperbody_bringup/test/test_integrated_sim_smoke.py`: tests 6 and 7 were "the defect is present" and are now plain
  assertions. Test 6: the 1 m drive within 0.2 m in ground truth, and odom within 0.2 m of ground truth. Test 7: every lift stop
  within 0.01 m, both in `/joint_states` and in lift_carriage_link's simulator height. Run: 8/8 plus 2/2 shutdown checks pass.
- `openamrobot_upperbody_gazebo/test/test_g01_gazebo_smoke.py` test 4 now also reads the model's simulator pose: 0.588 m ground truth,
  0.588 m odom, for 0.2 m/s over 3 s. Pass.

## Not done

- Nav2 (`nav2:=true`) was not run on 9 Nov because the navigation2 packages were not installed on that runner. On 10 Nov, with
  `ros-jazzy-navigation2` and `ros-jazzy-nav2-bringup` installed from apt and platform-sw's `openamrobot_nav2` at the pin, it was only
  started, not driven. What was observed:
  - controller_server, planner_server, bt_navigator, collision_monitor and velocity_smoother came up, and lifecycle_manager_navigation
    activated controller_server.
  - It then stopped at "Activating planner_server" for more than 3 minutes. The global costmap's frame is `map`, and this launch
    starts no map source and no localization.
  - platform-sw's `navigation_status_node` died at import: `No module named 'openamr_nav_msgs'`. That package is not in the pinned
    workspace.
  - The local costmap warned that its inflation radius (0.15 m) is smaller than the footprint's inscribed radius (0.30 m).
  No navigation goal was sent. Nav2 in the integrated sim needs a map source (or SLAM) and the `openamr_nav_msgs` package. Both are
  platform-sw's.
- The Bullet-Featherstone engine was not re-measured after the fix. It is not the default, and the gripper tests (G02) use it.
- The stop-polygon acceptance and the Gate A replay are not in this repository. They should read ground truth, as these tests now do.
