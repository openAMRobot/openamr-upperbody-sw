# openamr-upperbody-sw

Upper-body software for the OpenAMRobot mobile manipulator: the arm and lift mounted on the mobile base. Turns `openamr-platform` (the base) into a full mobile manipulator.

> **Status:** Planned, no code yet

The v0.2 cycle is simulation first. The ROS 2 packages here are scaffolding: manifests and empty directories, no source yet.

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

## Ownership, licensing, and contributions

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD** (Cyprus Company ID HE479056). Botshare LTD owns the transferable economic rights in original OpenAMRobot material created by or validly assigned to it. Third-party material remains subject to its respective ownership, licences, and notices.

Original OpenAMRobot software and firmware are licensed under MIT, documentation under CC BY 4.0, and hardware design source under CERN-OHL-P-2.0, as mapped in [`LICENSING.md`](LICENSING.md). Public distribution grants the permissions stated in the applicable licence; it does not transfer ownership of underlying copyright, trademarks, patents, or other intellectual property.

Accepted external contributions require DCO sign-off and an applicable Individual or Corporate Contributor Agreement. See the organization [IP Policy](https://github.com/openAMRobot/.github/blob/main/IP_POLICY.md), [Contribution Guide](https://github.com/openAMRobot/.github/blob/main/CONTRIBUTING.md), and [Contributor Agreement Process](https://github.com/openAMRobot/.github/blob/main/CLA.md).
