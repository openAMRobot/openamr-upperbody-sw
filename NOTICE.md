# Notice

OpenAMRobot is a project initiated, operated, and controlled by **Botshare LTD**, Company ID HE479056, Chrysanthou Mylona 1, Panayides Building, Office 1, 3030 Limassol, Cyprus (info@botshare.ai; https://botshare.ai).

Copyright and transferable economic rights in original OpenAMRobot material are owned by Botshare LTD or used by it under applicable agreements. Accepted external contributions are governed by the OpenAMRobot Contributor Agreement.

Botshare LTD does not claim ownership of third-party material. Authentic upstream ownership, licence, patent, attribution, and modification notices remain effective.

## Third-party material

### Files derived from Enactic OpenArm (Apache-2.0)

These six files are derived from Enactic OpenArm repositories at the commits pinned in `ros2/src/openamrobot_upperbody_description/config/upstream_pins.yaml`. They are licensed under the Apache License, Version 2.0 ([apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0); full text in [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt), unchanged), not under the repository's MIT licence. Each file keeps its upstream copyright and licence notice and states what OpenAMRobot changed. Neither upstream repository has a NOTICE file at the pinned commit.

| File in this repository | Upstream repository and file | Copyright (as in the file) | Modification status |
|---|---|---|---|
| `ros2/src/openamrobot_upperbody_description/config/openarm_preset_left.yaml` | [enactic/openarm_description](https://github.com/enactic/openarm_description) `assets/robot/openarm_v2.0/config/robot_presets/left_arm_with_pinch_gripper.yaml` @ `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` | Copyright 2026 Enactic, Inc. | modified: `robot.root_frame` and `primary_arm.connection.parent` |
| `ros2/src/openamrobot_upperbody_description/config/openarm_preset_right.yaml` | [enactic/openarm_description](https://github.com/enactic/openarm_description) `assets/robot/openarm_v2.0/config/robot_presets/right_arm_with_pinch_gripper.yaml` @ `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` | Copyright 2026 Enactic, Inc. | modified: `robot.root_frame` and `primary_arm.connection.parent` |
| `ros2/src/openamrobot_upperbody_control/config/arm_controllers.yaml` | [enactic/openarm_ros2](https://github.com/enactic/openarm_ros2) `openarm_bimanual_moveit_config/config/openarm_v2.0/ros2_controllers.yaml` @ `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | Copyright 2025 Enactic, Inc. | modified: state interfaces, gripper controller parameters, `lift_controller` added (listed in the file header) |
| `ros2/src/openamrobot_upperbody_moveit/config/kinematics.yaml` | [enactic/openarm_ros2](https://github.com/enactic/openarm_ros2) `openarm_bimanual_moveit_config/config/openarm_v2.0/kinematics.yaml` @ `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | Copyright 2025 Enactic, Inc. | unmodified values |
| `ros2/src/openamrobot_upperbody_moveit/config/pilz_cartesian_limits.yaml` | [enactic/openarm_ros2](https://github.com/enactic/openarm_ros2) `openarm_bimanual_moveit_config/config/openarm_v2.0/pilz_cartesian_limits.yaml` @ `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | Copyright 2026 Enactic, Inc. | unmodified values |
| `ros2/src/openamrobot_upperbody_moveit/config/openamrobot.srdf` (section 1, "Imported from upstream") | [enactic/openarm_ros2](https://github.com/enactic/openarm_ros2) `openarm_bimanual_moveit_config/config/openarm_v2.0/openarm_bimanual.srdf` @ `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | Copyright 2026 Enactic, Inc. | modified: robot name, one `disable_collisions` pair dropped; sections 2 and 3 are OpenAMRobot's |

The `package.xml` of `openamrobot_upperbody_description`, `openamrobot_upperbody_control` and `openamrobot_upperbody_moveit` lists Apache-2.0 next to MIT for these files.
