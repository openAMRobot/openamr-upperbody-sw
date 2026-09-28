# openamr-upperbody-sw

Upper-body software for OpenAMRobot 2.0: the combined mobile base, fixed mast and OpenArm 2.0 arms. An actuated lift is deferred to OpenAMRobot 3.0.

> **Status:** Development scaffold. The ROS 2 packages contain manifests and empty directories; the combined model and integration are not yet implemented.

## What lives here
- **Combined description:** base + custom fixed mast + arms in one URDF/Xacro, using the official OpenArm arm-mount frames.
- **MoveIt:** arm planning groups and collision configuration on the combined model. No actuated lift planning group in OpenAMRobot 2.0.
- **Bringup:** launch files composing the base, fixed mast and arm integration from `openamrobot-manipulation`.
- **Configuration metadata:** selected mast position, model/configuration identity and calibration references.
- **Lift control:** future OpenAMRobot 3.0 scope.

## Current development scope
1. Combined URDF/Xacro with a correct TF tree, collision geometry and documented mounting transforms.
2. Arm simulation through ros2_control, followed by physical integration under the approved work packages and acceptance gates.
3. MoveIt planning and execution on the combined model.
4. Demonstration capture of arm state, base odometry and versioned mast/configuration identity.

Simulation is an integration stage, not a blanket deferral of physical OpenAMRobot 2.0 work. Model, simulation and hardware acceptance must be recorded separately.

## Mast configuration
OpenAMRobot 2.0 uses a fixed mast with four indexed shoulder-axis positions, 1300, 1350, 1400 and 1450 mm above the floor. The release configuration is `mast_1350`, a **1350 mm shoulder-axis height**. The mast top is 1500 mm above the floor. **1700 mm is the assembled robot-height envelope**, including the head camera and other mounted equipment, not the shoulder-axis height. The other three positions are mechanical provision and engineering-analysis options, not release configurations. Decision of record: P-03 Decision Addendum revision 18.2, 28 September 2026.

The mast is one MISUMI HFS6-60120 aluminium profile on the platform centre bracket; the OpenArm supplier body is not installed. The combined model uses the approved mast geometry and inertials, preserving the official arm attachment frames and kinematics. A4 provides the reach, TCP orientation, stand-off, collision, payload and F2S evidence for the release configuration; the three other index positions are analysis options only.

## Depends on
- `openamr-platform-sw`: mobile-base model and software.
- `openamrobot-manipulation`: arm integration and manipulation server.
- `openamr-upperbody-hw`: custom mast geometry, mounting interfaces and as-built measurements.
- `openamr-upperbody-fw`: any separately agreed non-lift firmware integration; lift firmware is OpenAMRobot 3.0 scope.

## Design rule
OpenAMRobot 2.0 mast height is **versioned configuration metadata**, not runtime lift state. Keep the selected configuration consistent across URDF/Xacro, MoveIt, TF/camera calibration, the robot configuration hash and dataset metadata. Changing the index position is a configuration change that requires the affected calibration and readiness checks and a recorded decision. Represent the assembled mast with fixed joints; do not introduce a locked actuated lift as the current baseline.

Package descriptions and model semantics are tracked in [issue #7](https://github.com/openAMRobot/openamr-upperbody-sw/issues/7).

Part of the OpenAMRobot ecosystem: https://github.com/openAMRobot

## Ownership, licensing, and contributions

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD** (Cyprus Company ID HE479056). Botshare LTD owns the transferable economic rights in original OpenAMRobot material created by or validly assigned to it. Third-party material remains subject to its respective ownership, licences, and notices.

Original OpenAMRobot software and firmware are licensed under MIT, documentation under CC BY 4.0, and hardware design source under CERN-OHL-P-2.0, as mapped in [`LICENSING.md`](LICENSING.md). Public distribution grants the permissions stated in the applicable licence; it does not transfer ownership of underlying copyright, trademarks, patents, or other intellectual property.

Accepted external contributions require DCO sign-off and an applicable Individual or Corporate Contributor Agreement. See the organization [IP Policy](https://github.com/openAMRobot/.github/blob/main/IP_POLICY.md), [Contribution Guide](https://github.com/openAMRobot/.github/blob/main/CONTRIBUTING.md), and [Contributor Agreement Process](https://github.com/openAMRobot/.github/blob/main/CLA.md).
