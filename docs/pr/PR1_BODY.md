## Why A6 and A7 are one PR

The change from the fixed mast to the lift column straddles the description and the simulation/MoveIt packages: the description gives up the mast, the control, Gazebo and MoveIt packages follow the lift joint, and the numbers each of them records (collision matrix, planning poses, configuration IDs) change together. On current `main` the description is only canonical if it carries the lift, and the lift work lives in the same commits as the Gazebo and MoveIt work, so the two cannot be split by commit; a cherry-pick of the A6 part on its own does not build green against `main`. A file-level split (description files in one PR, everything else in a second) is possible on request, at the cost of a first PR whose tests need the second.

`feat/a6-canonical-description` @ `44d28ea` stays on the fork as the historical A6 delivery (the mast model); this PR supersedes it.

## What is in it

Five packages under `ros2/src/`, plus `docs/`, `evidence/` and the changelog:

- `openamrobot_upperbody_description` — the canonical combined description: `combined_robot.xacro` composes the platform base (unchanged, `base_footprint` root), option B's upper body on its lift column and the upstream OpenArm macros; datums with provenance, frame contract, per-expansion evidence checks (T01-T16, C01/C02, E01, W01/W02, V01, H01/H02) and the harness that runs and reads them.
- `openamrobot_upperbody_control` — `ros2_control` for the two OpenArm v2.0 arms, one file for the mock, gazebo and real profiles (joint state broadcaster, trajectory and gripper controllers); the lift is not in it, we have no authority to command it.
- `openamrobot_upperbody_gazebo` — Gazebo Harmonic simulation of the combined model: the base on its DiffDrive plugin, both arms on `gz_ros2_control`, `odom -> base_footprint`.
- `openamrobot_upperbody_moveit` — MoveIt 2 configuration: arm planning groups with the lift as a planned joint, a collision matrix derived (83 disabled pairs, 478 enabled) rather than sampled, named poses, regression checks M01-M07, `joint_state_clamp.py`.
- `openamrobot_upperbody_bringup` — launch files composing base, lift column and arms in simulation.

## Evidence

Class: `static+mock` for the A6 suite, `simulation` for A7. **No physical evidence.** Every check reports one of five values:

| value | meaning |
|---|---|
| `PASS` | the check ran and its assertion held |
| `WARN` | the check ran and recorded a finding; it is not a pass and every WARN has a check ID |
| `FAIL` | an assertion was violated, or a process guard saw a process fail to start, die or exit badly |
| `SKIP` | a stated, reasoned non-comparison (the one legitimate case: no git checkout to compare an upstream pin with) |
| `NOT_RUN` | a registered check left no result; never a `PASS`, and it fails the exit status |

- **A7 regression** (`run_a7_regression.sh`), on the code of this PR (`013b524`; the pushed head adds report copies, measurement data and documents, and one unit-test file): **no run is green, by design: M07 FAILs** (below). Run 2 and run 3: 13 PASS / 1 WARN (V01) / 2 FAIL (M07, and AUX with it, because the bringup `colcon test` step exits 1) / 0 NOT_RUN. Run 1 additionally failed **G01**: its `gz sim` server ignored SIGINT, the stop request and SIGTERM and was ended by the stop helper's SIGKILL at 9.2 s; nothing was left running, and the run says so (11 PASS / 1 WARN / 3 FAIL). Earlier, on `7daa2ab` (the shutdown-race fix, before the rebase and before option B: these prove that fix) five runs of 13 PASS / 1 WARN / 0 FAIL, plus three on `dada509` and three on `aa94fc3`.
- **One A7 run in six failed on this code earlier** (P16, `9bc327c`: 11 PASS / 1 WARN / 2 FAIL, M02 `CONTROL_FAILED`) under a simulator orphaned by G01's launch that nothing detected. Its cause was identified, it is now detected (and, since P17, stopped by the launch), and the report is kept (`evidence/reports/README.md`, P16). The number to quote is "one failure in six on that commit, cause identified", not "three for three"; and P17's run 1 above is a second, different one.
- **M07 currently FAILs by design**: no joint's rest position may be within `joint_rest_margin_rad` (a datum, 1e-3 rad, `placeholder`) of either limit. It FAILs on `openarm_right_finger_joint1` (rests at +2.6e-8 rad, past its open limit) and on `openarm_left_finger_joint1`, `openarm_left_joint4` and `openarm_right_joint4`. The zero and open poses are limits by construction, in the old body too (`evidence/m02_bisect/RESULTS.md`, last section), so the check is red until the margin, the rest pose, or the claim it makes is decided; that is a question for the reviewers, not something the PR hides.
- **A6 suite** (`verify_a6.sh`): **24 PASS / 4 WARN (T08, E01, W02, V01) / 0 FAIL / 0 NOT_RUN**, three runs on `013b524` (and on `1168f45` before). The first pass of this step, on `a713533`, failed 3 of 3 (AUX: `test_evidence_dir.py`, the stop helper's report was in the system temp directory); the reports are kept and the fix is `013b524`. `config_id`s changed again (the datums revision, for the new `joint_rest_margin_rad`).
- The reports are in `evidence/reports/`, with `README.md` listing commit and result for each, including the failed ones. `evidence/frozen_a6_993b000/` is the frozen expansion set of the earlier mast-model A6, with a verifier.
- **Orphaned simulators.** Two of the eight A7 runs of P15 left a `gz sim` server running after the launch under test was torn down, and one more did after G01 in P16 (the natural rate was about 1 in 13 launches). G01, G02, M02 and M07 now fail when a simulator of their own partition survives their launch, and `sim.launch.py` stops the server through a detached helper (SIGINT, then the service request, then SIGTERM and SIGKILL); a server that has to be sent SIGTERM or SIGKILL is itself a FAIL of the run that needed it. After that fix: **0 orphans seen by the detector in 12 runs each of G01, G02 and M02** (natural rate before: about 1 in 13). That is not a claim that it is fixed: 12 clean runs of a one-in-13 event are not unlikely, and the one live force-stop above shows the hang still occurs (one in 49 launches since, 36 of them in that count). The cause of the hang is inside gz-sim and was not found. The detector was proven through the launch-test path with an injected fault (`canary_gz_orphan.sh`) and on decoy processes.

## Dependencies

- **platform-sw `935efc7` (#55)**, on platform-sw `main` (`174731c` at the time of writing): `base_footprint` root, `base_link` 0.098 m above it, and the near-field sensor frames `us_*` (4 x MB7060) and `tof_*` (2 x TF-Luna) declared. This description requires it (its root and the 0.098 m are asserted against platform-sw's `base_footprint_joint`).
- The six near-field frames that earlier versions of our contract declared (`tof_front_left_link` ... `ultrasonic_rear_link`) are removed from our contract and are `consumed_from` platform-sw, not declared by us. T04 now reports which of them each expansion contains and passes **by deletion**: nothing in our contract can be absent any more.
- No upstream (`openarm_description`) file is modified; the pin is unchanged.

## Open questions for reviewers

- [ ] **Alex and Cuma: the two opt-in parameters on `openamrobot_upperbody.xacro`.** `emit_arm_mounts` (default `true`) and `column_inertial` (default `''`, none). Both default to current `main` behaviour. They exist because the upstream arm macro emits a link and a joint with the same names as option B's mount frames, so the combined robot has to turn those off, and because the combined robot supplies the lift's sourced catalogue mass. Two separate statements, kept apart because the first one is weaker than it sounds. (a) A test (`test_with_the_opt_in_parameters_at_their_defaults_the_macro_expands_as_it_does_on_main`) asserts that the mount frames and joints exist and that no inertial appears anywhere with the defaults; it does **not** compare against `main`. (b) Separately, compared byte for byte against `upstream/main` (`tools/compare_option_b_wrapper.sh`, run at `HEAD`), the standalone wrapper with the defaults expands identically apart from the empty `lift_column_base_link`, which is serialised as `<link name="lift_column_base_link"></link>` (an open/close pair) instead of a self-closing tag, and the path xacro writes in its header comment. That script is not part of the test suite. Needs acceptance, or a different composition.
- [ ] **Supplier drawing for the lift column.** The column has **no visual and no collision geometry**: it exists in the model as an inertia tensor only, so every render committed under `evidence/renders/` shows the arms with no column between them, and the arms can plan through the column body. It also carries one invented lateral dimension: `lift_column_section` y is set equal to the sourced 240 mm x side. Both are marked in `datums.yaml` (`placeholder`/`provisional`); nothing downstream treats them as measured.
- [ ] **V01's interference count was taken on collision geometry only.** The arms' visual meshes were not intersected separately, and the column has neither kind of geometry, so a zero there says nothing about the column.

## Known gaps

- `lift_mass_tl3_400` is `null` (no supplier mass for the fallback TiMOTION TL3 400 column). By design a null datum that is read is a hard-fail that names itself: the `tl3_400` configuration does not expand.
- V01 (manual visual check) is `WARN`. Its eight items were measured by script on the lift model (none reads outside its expected value; two sub-questions that need eyes are CANNOT-MEASURE; `evidence/v01/`, renders in `evidence/renders/`), and it stays `WARN` until `lift_mass_tl3_400` is sourced, however the eight land. No person has looked at the running simulator.
- M02 and the clamp: **the activation count is retired as a criterion.** It scatters 646 to 1054 inside one unchanged configuration (and 1795 / 957 / 1832 on three runs of a tests-only diff), so a rise from about 200 to about 1800 could not be read as a signal. M02 now gates on two numbers: readings beyond the epsilon (must be 0) and the largest overshoot magnitude (must be below the epsilon, 1e-5 rad); the counts are reported as `diagnostic_only`. In the three runs above: 0 excursions and a largest overshoot of 1.5e-6 rad (margin about 6.5x; P16's worst variant run read 4.2e-6, about 2.4x). The epsilon has not been widened. M02 also waits for the five controllers to be `active` before its first pose and FAILs, naming their states, if they never are. The right finger rests on its open limit (see M07); why it sits within 1e-5 rad of it in the new body and 2e-4 to 5e-4 inside in the left one was measured (finger effort, both trees, `evidence/m02_bisect/RESULTS.md`) and **not explained: cause unknown**. The M02 bisect is closed.
- No runtime producer of the configuration identity has an owner: `config_id` is computed offline from files; nothing in a running system publishes it.
- Arms are 62 mm apart at their bases: no contact at home, but the left gripper reaches right-arm links in about 3% of random left-arm states (sampled).
- A `gz sim` server that is asked to stop sometimes never begins its teardown and survives SIGINT, the stop request and SIGTERM (about 1 launch in 13 before the helper; 1 in 49 after). The launch now ends it by SIGKILL and the run FAILs naming it; the underlying hang is in gz-sim and is not fixed here.

## `config_id` inputs changed

`config_id` kept its format and changed its inputs: the canonical URDF, `column`, `base_plate`, `head_pitch`, `base_tilt`, `arm_package`, `hardware_profile`, datums revision and every upstream pin (before: `mast_id` and no column or base-plate inputs). It still validates against the C9 schema, so nothing fails to say that **an id recorded before this change is not comparable with one computed now**. The frozen A6 set keeps the old inputs and says so. Satya (C9 owner): please confirm that consumers do not compare ids across this change.

## AI assistance

This PR was developed with Claude Code (Anthropic) against a written design (architecture documents and prompts). The work was run and tested by the repository's evidence harness (`verify_a6.sh`, `run_a7_regression.sh`; the reports are in `evidence/reports/`, produced on the author's workstation, not by a hosted CI service) and reviewed by Tushar Goel. Prompts and design documents are available on request.
Per the OpenAMRobot AI Contribution Policy.

## Checklist

- [ ] DCO sign-off on every commit
- [ ] Contributor agreement accepted
- [ ] No upstream files modified
- [ ] No frame names outside design/FRAMES.md
