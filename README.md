# openamr-upperbody-sw

Upper-body software for the OpenAMRobot mobile manipulator: the arm and lift mounted on the mobile base. Turns `openamr-platform` (the base) into a full mobile manipulator.

**Status:** planning / early development (v0.2 cycle, simulation first).

## What lives here
- **Combined description:** base + lift + arm as one URDF/xacro, arm attached through a standard mounting-plate frame.
- **Lift control:** ros2_control configuration and controllers for the vertical lift.
- **MoveIt:** planning groups for the arm and for arm + lift on the combined model.
- **Bringup:** launch files that compose base, lift, and an arm pulled from `openamrobot-manipulation`.

## What we are building (v0.2, simulation)
1. Combined URDF/xacro with a correct TF tree and collision meshes.
2. Lift and arm actuated in Gazebo through ros2_control.
3. MoveIt planning and execution on the combined model.
4. Whole-body state published for demonstration capture: arm state, lift height, base odometry.

Physical lift build and hardware bring-up follow next cycle. This cycle is simulation only.

## Depends on
- `openamr-platform-sw` (the mobile base).
- `openamrobot-manipulation` (the arm and manipulation server).
- `openamr-upperbody-hw` / `-fw` (lift mechanics and firmware, next cycle).

## Design rule
The lift is a first-class subsystem, not an accessory. Its height is recorded state and enters the URDF, simulation, MoveIt, and the demonstration schema from day one.

Part of the OpenAMRobot ecosystem: https://github.com/openAMRobot
