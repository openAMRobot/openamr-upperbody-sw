# openamrobot_object_perception

Object pose estimation for OpenAMRobot, milestone 1 (**Proposed**; design in
[`docs/architecture/object_perception.md`](../../../docs/architecture/object_perception.md)). Simulation evidence only:
no estimate here is a measurement of a real camera, and nothing here is a safety function.

`object_pose_estimator.py` reads one depth image (default: the head camera of the Build 1 scene,
`/head_camera/rgbd/depth_image` and its camera info), takes the camera's pose in `base_footprint` from /tf, and publishes
the tote and the item as `geometry_msgs/PoseWithCovarianceStamped` in `base_footprint` on `/perception/objects/<name>/pose`
(Proposed topic), at most `rate_hz` (2 Hz). It reads no simulator ground truth and publishes no command.

The method is in `box_fit.py` (no ROS import): a RANSAC line through the camera-facing face of each box, seen from above,
and the known box sizes. `synthetic.py` is a ray-cast depth fixture for the tests, not a camera simulation.

| Parameter | Default | Meaning |
|---|---|---|
| `target_frame` | `base_footprint` | frame of the points and of the published poses |
| `depth_topic`, `camera_info_topic` | head camera | 32FC1 (metres) or 16UC1 (millimetres) depth |
| `camera_frame` | empty | REP-103 optical frame of the image; empty takes the image header's frame |
| `support_z` | 0.82 | height of the shelf level the tote stands on (site data, not an object pose) |
| `tote_size`, `tote_wall`, `item_size` | Build 1 scene values | known box sizes (the item catalogue) |
| `region` | `[0.45, 1.4, -0.9, 0.9]` | x min, x max, y min, y max in `target_frame` where objects are looked for |
| `max_jump_m`, `reacquire_count` | 0.05, 4 | a jump is held back until it repeats at one place |

Tests: `python3 -m pytest test/test_box_fit.py` (no ROS). The Build 1 integration is tested in
`openamrobot_build1_scene/test/test_object_perception.py` and by the launch test `test_scn04_grasp_estimate`.
