<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Candidate simulation capture profile for Baseline 1 (Proposed, 19 Nov 2026)

Status: Proposed. `capture_profile_baseline1_gazebo_candidate.json` (next to this page) is the candidate written by
`tools/baseline1_capture_binding.py` from the recorded run named in its `provenance`. Registration is a reviewed code
change in openamrobot-embodied-ai (`APPROVED_CAPTURE_PROFILES` in `recording/profile.py`) by its owners; nothing is
registered or pushed there from this repository. Simulation evidence only: never real robot data, physical, safety
or release acceptance.

## What the converter expects (embodied-ai `9643da1`, read from `validate_profile` and `read_bag`)

- `capture_version: 1`; `profile_id`, `episode_id`, `dataset_id`, `dataset_revision`, `task`,
  `robot_configuration_id` non-empty; `origin` one of `synthetic`, `simulation`, `robot`.
- A non-synthetic profile only with `status: accepted`, an `approval_ref`, and an entry in `APPROVED_CAPTURE_PROFILES`
  whose `sha256` equals `capture_contract_digest(profile)` (fps, max_skew_ns, streams, components, channels,
  source_commits, component_contract, phase_evidence and the ids are hashed).
- Integer `fps` and `max_skew_ns`. One stream per topic, each with `id`, `component`, `topic`, `type`, `frame_id`,
  one `clock_domain` for all streams, and `timestamp_source` `header` or `bag` (headerless topics also
  `frame_id_source: configuration`). Stamps strictly increasing per stream.
- Exactly the four canonical camera features (`observation.images.head`, `.base`, `.camera_wrist_left`,
  `.camera_wrist_right`), each with a `calibration_id`. A camera frame is never reused on the export grid, so `fps`
  is at most the slowest camera rate.
- State and action channels each equal, in order, the C9 component contract (name, component, unit, range); with a
  lift, the contract file and its sha256 are bound and the contract must be `accepted` for non-synthetic data.
- Base actions are `semantics: requested_velocity`. Units convert only along the converter's table
  (rad, deg/degree, m, mm, m/s, rad/s): a lift command in stop indices is refused.
- Lift motion needs `phase_evidence` (reviewed stowed pose, tolerance, base odometry stream).

## What the opt-in AI capture records (baseline1_record.py, `BASELINE1_AI_CAPTURE=1`)

| Need | Recorded topic | Note |
|---|---|---|
| Head camera | `/head_camera/rgbd/image`, `/head_camera/rgbd/camera_info` | 640 x 360, 6 Hz |
| Base camera | `/base_camera/rgbd/image`, `/base_camera/rgbd/camera_info` | 640 x 480, 6 Hz (not the station camera) |
| Wrist cameras | `/left_wrist_camera/image`, `/right_wrist_camera/image`, their `camera_info` | 320 x 240, 5 Hz |
| Gripper command | `/build1/capture/gripper_command` (JointState, both fingers, rad, 20 Hz) | held GripperCommand goal; Proposed topic |
| Motor-side gripper trace | `/sim/<side>_arm_emulator/status`, key `mit_q` | last MIT position command over CAN; not a field path |
| Base state | `/odom` (base_state_adapter) | recorder gets Fast DDS peers 0 to 119 (PR #89 finding) |

Camera values are the provisional, UNVERIFIED sensor values of `build1_robot.urdf.xacro`. The cameras are recorded at
native rate (not throttled): throttling would force the export grid lower still.

## Result of the one recording run (19 Nov 2026)

Recording: [Baseline 1 acceptance 44448676604](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44448676604)
on `a207463`, dispatched with `ai_capture` 1, `station_alignment` 1, profile `sim-oracle`; 8 of 8 phases passed.
Binding: [Bag to LeRobot 44453343290](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44453343290).

| | Before (run 44309791416, default bag) | After (run 44448676604, AI capture) |
|---|---|---|
| Observation channels bound | 17 of 19 | 19 of 19 (`/odom` now recorded, 5513 messages at 50 Hz) |
| Action channels bound | 17 of 19 | 19 of 19 (grippers from `/build1/capture/gripper_command`) |
| C9 cameras bound | 0 of 4 | 4 of 4 (6, 6, 5, 5 Hz, each with camera_info) |
| Bag on disk | 185.6 MB, 102.5 MB per minute of Gazebo time | 1615.5 MB, 879.2 MB per minute (images 717.5 MB per minute) |
| Converter `validate_profile` | REFUSED: unapproved profile | REFUSED: unapproved profile |
| Converter `read_bag` | REFUSED: timestamps of `cmd_vel` | REFUSED: timestamps of `arm_joint_states` |

The run was slower: real-time factor 0.11 to 0.14 against 0.18 to 0.21, mission wall time about 756 s against 492 s
(one run each, on different runners; not a measurement of the capture's cost). `/build1/run_info` and
`/build1/rtf_stats` are missing from this bag: rosbag2 stopped topic discovery once it held 48 subscriptions for
48 named topics, counting regex matches, before those topics appeared. The head of this PR records the capture with one
anchored regex and no topic list, so discovery is not stopped (unit-tested, not yet run in the stack). For the same
reason the committed candidate has `pose_source` and `object_pose_source` null; the run log declares
`sim-oracle`, `pose_source: ground_truth` and `object_pose_source: ground_truth`, and the tool now fills them from the
log when the bag has no `/build1/run_info`. `component_contract.path` in the candidate is the CI runner's checkout
path and is re-bound by whoever registers the profile.

## Gaps the candidate cannot close (for the owners)

1. Registration and approval: embodied-AI owner and software lead (`approval_ref`, digest, `status: accepted`).
2. C9 component contract (openamrobot-interfaces #39) is `proposed`; non-synthetic export needs it `accepted`
   (software lead). It names `/arm/joint_states` as the lift state source; in Baseline 1 `lift_joint` is on
   `/joint_states` (binding.json flags it).
3. `/cmd_vel` receipt stamps repeat under simulation time: see `timestamp_rule_proposal.md`.
4. Lift action is a stop index on an event topic (`/sil/lift/command`): needs a metre target stream (held, like the
   gripper) or a converter hold rule, and a stop-index-to-metre mapping owner.
5. Base action stops when Nav2 is idle (`/cmd_vel` is not published during the arm phases).
6. Calibration ids are candidate identities of Gazebo's `camera_info`, not calibrations.
7. `fps` and `max_skew_ns` in the candidate are derived from camera rates, not approved values.
8. Units: radians (decision on #79) against the degree-based C9 v1 LeRobot profile; see
   `dataset_location_proposal.md`.
9. `robot_configuration_id` is not declared by `/build1/run_info`.
