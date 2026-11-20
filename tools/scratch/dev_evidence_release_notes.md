**Development evidence only. Simulation, not validated.** Gazebo Harmonic simulation, `sim-oracle` acceptance profile (`pose_source: ground_truth`, `object_pose_source: ground_truth`), simulation time. Not robot data, not hardware, perception, safety or release evidence, and not a training-ready dataset.

The conversion depends on Proposed conversion rules (`episode_origin`, `controller_reference`, `command_semantics`, `vector_segments`, `stop_index`, `range_tolerance`, `held_command`, `lift_positioning_from_command`, `lift_phase_report`, `reference_integrity`, plus `keep_last` for equal timestamps) and a Proposed capture profile. None of them is approved by the converter owner (SatyaA27) or the software lead (panthera-momagdii). The C9 component contract (openamrobot-interfaces #39) and the radians contract change (#41, draft #42) are Proposed. **C10: NOT_VERIFIED** (no approved C10 policy; exit 2, training_ready false). C9 sidecar: valid against the Proposed contract.

This release only preserves the files before their workflow artifacts expire (2026-12-19). Nothing was re-recorded or re-converted for it: every asset is a byte copy of an existing artifact, checked against the manifest hashes after upload.

### Source recording
- Run https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44448676604 (job 135306975778, Baseline 1 whole-robot mission), head `a20746338701e7961c28ba4650e11996a1a76919`, artifact `baseline1-acceptance-attempt1`, path `ws/build1_logs/bag/`.
- `bag_0.mcap` sha256 `4b1f23f220b1b8aaedeff6da34e76131aee62373399683bd083087418459bf50`
- `metadata.yaml` sha256 `cf2a9bb5fc3dcfc0476a6fb1372df863e2ae956f137e456c8cf327e829d27f47`
- Source commits: openamr-upperbody-sw `a20746338701`, openamr-platform-sw `931e631db661`, openarm_description `14ff67b638ff`, openarm_ros2 `4e837e1d0dae`, openarm_can `bfd4e52194ae`, openamrobot-interfaces `43ec67747b10`.
- Robot model sha256 `e1a3c33d59f6c5628f1329c42fe04b67e11d1720db4a2f65e1dfae211885a2e2`, taken from the recording job log (asset `recording-job-135306975778.log`); `/build1/run_info` is not in this bag.

### Reference dataset (primary)
- Converted twice, byte-identical, and verified with `oa-verify-reference` in https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44598573882 (job 135823460974, branch `scratch/ai-capture-e2e` at `64fd9c645745`), converter openamrobot-embodied-ai `cf7f2835e204107fbb461af8be2f9254a33b9f0b` (draft PR #8), C9 assets openamrobot-interfaces `87ab8a936c97`, component profile `43ec67747b10` (Proposed), LeRobot 0.6.1.
- 1 episode, 543 frames at 5 fps (grid 7.6 s to 116.0 s), 19 state and 19 action channels, 4 cameras. 1 pre-origin frame dropped.
- `meta/openamrobot/conversion.json` sha256 `2d0855f61daac7e8d65c6844cb3c94e87bd5e43daf823eeb14967106d445b1db`; it lists the sha256 of every other dataset file (`reference.output_sha256`) and of both bag files (`source_bag_sha256`).
- Equal-timestamp samples dropped and counted (kept + dropped = metadata.yaml count): `/arm/joint_states` 53087 + 122, `/left_arm_controller/controller_state` 53103 + 117, `/right_arm_controller/controller_state` 53116 + 94, `/cmd_vel` 1664 + 1288. All other streams kept in full.

### Earlier conversion (kept for the record)
- https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/44521207508 (branch `scratch/ai-capture-e2e` at `93273b5`), converter `8d012ea9feb9`, same recording, same 543 frames, before `reference_integrity`. Superseded by the reference dataset above.

### Assets
See `SHA256SUMS`. `baseline1-reference-dataset-cf7f283.tar` unpacks to the LeRobot dataset directory; `baseline1-reference-dataset-cf7f283.files.sha256` is the per-file list printed by run 44598573882. If the bag is published in parts, rebuild it with the command in `REASSEMBLE.txt` and check it with `sha256sum -c`.

Regenerate and verify, from an openamrobot-embodied-ai checkout at `cf7f283` with the bag directory `bag/` holding `bag_0.mcap` and `metadata.yaml`: see the `reference` job of `.github/workflows/scratch-ai-capture-e2e.yml` on `scratch/ai-capture-e2e` at `64fd9c645745`, and run `oa-verify-reference <dataset> --bag bag --run-log recording-job-135306975778.log --require-clean-converter`.
