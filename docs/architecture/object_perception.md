# Object perception: object poses from the robot's own cameras

Status: **Proposed**. Owners: the perception work-package owner (method, estimator) and the software lead (topic,
message, `object_pose_source` values). Nothing here changes decisions.yaml. Scope: the Build 1 scene (`ros2/src/openamrobot_build1_scene`) and the
new package `ros2/src/openamrobot_object_perception`. Simulation evidence only: no estimate here is a measurement of a real
camera, and nothing here is a safety function.

## Starting point (integration/baseline1 at 024ccb8)

- Two declared sources. `planning_pose_source` (`map` default, `ground_truth` for the sim-oracle profile) places the robot's
  base in the planning scene. `object_pose_source` (Proposed, only value `ground_truth`) gives the tote, item and payload poses
  of the planning scene and the grasp targets, read through `planning_scene.object_pose_topic(source, name)`. The rig refuses
  a run without the declaration.
- Cameras in the model: the head camera (RGB-D stand-in for the ZED Mini on the lift carriage, pitched down 30 degrees,
  640 x 360, 90 degree field of view, 6 Hz), two wrist cameras (colour only, 320 x 240, placeholder pose datums) and the base
  camera (Gemini 336L at `base_camera_link`; per audits #89 the description's Gazebo sensor sits on the legacy `camera_link`;
  openamr-platform-sw #167 adds a simulation-only test camera at `base_camera_link` for its marker detector).
- Objects: a tote (0.40 x 0.30 x 0.06 m, 5 mm walls) on work shelf level 0 (0.82 m) and one item (0.035 x 0.035 x 0.14 m,
  0.15 kg) standing in it (`config/scene.yaml`; the grocery scene of PR #86 has more).

## What the grasp needs from an estimate

The numbers come from `arm_frames.py` and the scn04 diagnosis in `docs/build1_arm_diagnosis.md`.

| Axis | Budget | Where it comes from |
|---|---|---|
| Along the fingers (base x at face yaw 0) | -0.9 mm to +1 mm characterised | The item centre goes 9 mm behind the TCP (`ITEM_DEPTH_BEHIND_TCP_M`). The palm face meets the item at 0.138 - 0.1106 - 0.0175 = 9.9 mm, so an item 0.9 mm nearer to the robot than estimated touches the palm. Depths of 8 to 13 mm lifted 10 of 10; on the TCP (9 mm shallower) 3 of 6 failed. The constants give 9.9 mm, not 14 mm; this note uses the constants. |
| Across the fingers (base y) | not derived | The open finger gap is not stated in the files read here (scene.yaml only says the item is narrower). Recorded per run until a source gives the gap. |
| Height | about +-10 mm | Grasp height is the item centre + 0.03 m; the hand body then clears the 0.88 m tote rim by 14.5 mm and the fingers (half height 27.5 mm) stay on the item. |
| Yaw about z | 0.01 rad | The approach holds yaw to 0.02 rad (`APPROACH_YAW_TOLERANCE_RAD`); hands 0.025 to 0.075 rad off the face closed on the item's edges. The estimate takes at most half of the budget. |
| Reach | adds to the arm's own error | The executed hand leaves its plan by up to 9 mm (the fixture padding note in `planning_scene.py`). The estimate error adds to that; it does not replace it. |
| Latency | one estimate no older than 1 s at target read, object and base still | scn04 reads the target once, before the arm moves. The camera runs at 6 Hz, the estimator at 2 Hz, stamped with the image time. |

On the ideal simulated depth image the method below is within 1.5 mm and 0.01 rad (static tests). Real stereo depth at
0.5 m is noisier than the along-finger budget; the budget, not this simulation, decides what a real camera must do.

## Options

| Option | For | Against |
|---|---|---|
| (a) Fiducial tags on totes and items | Robust ID and 6D pose; an AprilTag 36h11 detector exists in openamr-platform-sw (docking, #167 station marker); a stepping stone to clutter. | Changes the world and the site process (tags on every tote). The item's 35 mm face carries at most a 30 mm tag: about 17 pixels from the head camera at 0.54 m, below what a 36h11 tag needs to decode. A detector dependency from platform-sw into this repository. |
| (b) Depth segmentation and box fitting of known objects | No scene change, no new dependency; uses the item catalogue sizes and the site's shelf heights; sub-millimetre on ideal depth. | Assumes upright boxes of known size; sensitive to depth noise and to clutter; one item per tote region in this milestone. |
| (c) Learned 6D pose estimator | General items, clutter, partial views. | Models or training data, a GPU (Jetson on the 2.0 robot), licence and provenance review of weights; far more than one item needs. |

**Recommendation and milestone 1: (b) for both the tote and the item, from the head camera.** Tags are rejected for the
first milestone because the item cannot carry a decodable tag at this camera resolution, so the item would need (b) anyway,
and (b) fits the tote to under a millimetre without a world change. Tags on the tote are milestone 2, as a cross-check and
for real totes. The wrist cameras refine the final approach in milestone 3, once they have depth or tags. (c) follows when
the item set outgrows boxes of known size.

Which camera: the head camera searches and gives the milestone 1 target. From the parked pose it sees the tote and the item
at about 0.54 m (1.7 mm per pixel) under shelf level 1. The wrist cameras are colour only with placeholder poses. The base
camera, 0.046 m above base_link and tilted up 10 degrees, sees shelf fronts, not into a tote at 0.82 m.

## Method (milestone 1, `openamrobot_object_perception/box_fit.py`)

1. Depth pixels become points in `base_footprint` through the camera intrinsics and the camera's pose from /tf (the robot's
   own joint states, lift included). No object pose and no ground truth is an input. Known: the box sizes and the height of
   the shelf level the task names.
2. Tote: the points from 12 mm above the level to just above the rim. The nearest strong RANSAC line (seen from above) is
   the face that looks at the camera; along it, the run of points as long as one tote side gives the centre along the face
   (posts on the same line are separate runs); the centre is half the other side behind the face; the pose is at the bottom,
   from the rim height.
3. Item: the points between the tote rim and 0.03 m above the item's expected top, grouped on a 1 cm grid. A group is the
   item when it stops below the band top (a post continues), its top is within 12 mm of the expected top (a lifted or fallen
   item is not taken), it fits inside the item's diagonal, and its camera-facing face, a RANSAC line, is one item side wide.
   A finger next to the item merges with it into a group too wide: nothing is reported, never the finger.
4. A detection more than 0.05 m from the last published pose is held back until it repeats four times at one place.

## Topic and frame contract (Proposed)

- `object_pose_source: estimate` (Proposed) next to `ground_truth` (default, unchanged).
- Topic `/perception/objects/<name>/pose`, `geometry_msgs/PoseWithCovarianceStamped`, frame `base_footprint`, stamp the
  depth image time, published only when the object is found, at most 2 Hz. Names in milestone 1: `tote`, `item`. The
  payload is not estimated; under `estimate` it stays at its scene.yaml pose and the declaration says so.
- Pose conventions as the simulator's models: the item at its centre, the tote at its bottom centre. Yaw about +z, folded by
  shape symmetry (square item: a quarter turn; tote: a half turn). Roll and pitch are assumed zero (upright boxes) and carry a
  variance of 1 rad^2, meaning not estimated.
- Covariance of x, y, z and yaw: measurement-resolution terms only (pixel size at range over sqrt(12) across the face,
  over sqrt(24) along it, line-fit spread for yaw). Uncalibrated; the validation's `nees_xy` calibrates it.
- Why `base_footprint`: the grasp targets are in `base_footprint`, and no localisation error enters (`map` was 0.023 to
  0.064 m off the ground truth frame in the missions of 17 Nov). The estimate is valid while the base stands still. Objects
  that must persist while the base moves need a `map`-frame variant (estimate composed with map -> base_footprint), not
  defined here.
- The loader places a `base_footprint` estimate as it is (no base pose); under `estimate` a tote or item not yet estimated is
  left out of the planning scene, not put at its scene.yaml pose.
- The topic name and message go into openamrobot-interfaces through a contract change request (software lead) before this
  leaves Proposed.

## Validation per run

- `scripts/object_pose_validation.py` (Build 1 scene only) takes each estimate, Gazebo's pose of that object
  (`planning_scene.object_truth_topic`) and of the base (`/build1/ground_truth/odom`), and records estimate minus truth in
  `base_footprint`: dx, dy, dz, position, yaw (folded) and `nees_xy`. The running record (count, mean and largest error,
  last error) is on `/build1/perception/validation` (latched JSON) and goes into every scenario JSON as `perception_validation`.
- scn04 also logs `<name>_estimate_error_*` for the tote and the item at the moment it reads the grasp target, recorded
  without a limit.
- Ground truth only measures. Under `estimate` no grasp target, planning scene object or command reads it; scn04's outcome
  checks (start height, push, lift, hold, release) read Gazebo's item pose through `object_truth_topic`, with unchanged limits.

## What counts as the perception profile

A run is the perception profile when all of these hold:

1. `object_pose_source: estimate`: no object pose or grasp target from ground truth.
2. `planning_pose_source: map`: no base pose from ground truth.
3. The robot reaches the station on its own localisation (Baseline 1 mission, workflow profile `perception`), with station
   alignment from its own camera where it is used.
4. Ground truth is read only by the validation and the outcome measurement, and run_info says so.
5. The scenario checks are unchanged.

`test_scn04_grasp_estimate` meets 1, 2 (spawn pose and odometry, no Nav2), 4 and 5, not 3: it is evidence for object
perception milestone 1, not the perception profile.

## What remains for a full perception profile

| Item | Owner |
|---|---|
| Navigation pose without ground truth: the Baseline 1 mission with `planning_pose_source: map` and `object_pose_source: estimate` (profile `perception` in `baseline1-acceptance.yml`), station alignment through openamr-platform-sw #167 | software lead; openamr-platform-sw for #167 |
| A verified base camera sensor at `base_camera_link` (audits #89, platform-sw #53 and #167) | perception owner |
| Detector robustness: a depth noise model of the ZED Mini, clutter and several items (grocery scene, PR #86), occlusion by the hand, knocked-over items, covariance calibration from the validation records | perception owner |
| Wrist camera depth or tags for the final approach | perception owner, with the device package owner |
| Contract change request for `/perception/objects/<name>/pose` and the `estimate` value in openamrobot-interfaces | software lead |
| A `map`-frame object estimate for objects that persist while the base moves | perception owner, software lead |
