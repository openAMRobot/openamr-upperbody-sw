<!-- Body for openAMRobot/openamr-upperbody-sw PR #14 (A6 + A7), from branch followups/a6-a7-49d2368. Replaces docs/pr/PR1_BODY.md.
     Tushar Goel's description at 49d2368 is kept, unchanged apart from heading levels, under "Description at 49d2368" below. -->

## Work package

**A6 + A7**: the canonical combined OpenAMRobot 2.0 description (A6: platform base, option B lift column and upper body, both OpenArm v2.0 arms) and its simulation and MoveIt integration (A7: `ros2_control`, Gazebo Harmonic, MoveIt 2, bringup). Issue #10 tracks it. Author: Tushar Goel. Follow-up commits (`49d2368..8e1aba1`) were prepared for Tushar to adopt, at Alex's decision of 16 Oct. They are merged into `feat/a7-gazebo-moveit` (P19) as a merge commit with Alex's commits unchanged, together with Tushar's P18 commits (W04, evidence, the gz-sim issue draft). M07 is as Alex wrote it.

## What changed since `5139a49` (P29: test-only, and two false lines)

- **The AUX race is fixed in the test, not in the clamp.** `test_zz_a_reading_beyond_the_epsilon_is_not_hidden` compared the clamp's `activations` before and after switching the injected value; the publisher keeps sending in-epsilon noise until the switch lands and each such message is a legitimate clamp, so one in flight made the counts differ by one. The test now compares within the snapshot taken once the first beyond-epsilon reading has been counted, and checks within one snapshot that the largest clamped overshoot is still the noise. It is as strict as before and stricter for a reading hidden once: it fails on two deliberately broken clamps and passes on the real one in every run of a 30-run loop (`evidence/p29/`). No clamp, epsilon or margin changed.
- **A7 three more runs on `c84cfbd`: 14 / 2 / 0, each**; no check changed result against the clean P28 run. **Recorded (`process_exits` of the three reports, `evidence/reports/20261028T090816Z_a7.json` and the two after it): 0 of 12 launches of P29 had a `parameter_bridge` that ended in -11**; counting P24 to P29 (the runs whose exit codes are recorded), it is **1 of 60 launches**, the one of P28 run 2. The observed console figure (OBSERVED-NOT-RECORDED, up to P23) is a different basis and is not added to it: see "The `parameter_bridge` SIGSEGV" under Tests. A6 was not re-run: nothing it covers changed.
- **The one recorded `parameter_bridge` -11 is not in this branch's bridge.** It was P28 run 2, G02, `parameter_bridge-3`, which is `platform_bridge` from openamr-platform-sw, not `base_joint_states_bridge` from this branch.
- **The ros_gz draft** (`docs/issues/ros_gz-bridge_teardown_segfault.md`) now has the recorded exit codes of P24 to P28, kept apart from the older observed ones. The recorded -11 of P28 was in `platform_bridge`, the platform package's bridge, not in the bridge this branch adds. It is still **not filed** (no credential here), and still not a merge gate.
- **`docs/v01_visual_check.md`** has a sheet for a person to fill in against the six captures. It is blank on purpose: V01 stays WARN.
- **Two lines in the design folder** that called the column's lateral side invented now say it is sourced. Not in this repository.

## What changed since `403a529` (P28: the column geometry, 22 Oct answer from Alex)

Nothing outside this list changed. Tested code `a14a3ff`, platform-sw `dfd44a5`, datums revision `2026-10-28.a` (every `config_id` changes). Details: `CHANGELOG.md` (P28), `docs/v01_visual_check.md` (last section), `evidence/p28/`.

- **The column section is the supplier's: 240 x 60 mm** (`lift_column_section`, supplier, provisional; DOLD Hexalift datasheet page 4, "Outer profile 240x60"; matches option B drawing sheet 2 rev A). **The lateral side is no longer invented.**
- **The carriage is the Hexalift inner stage, 209.5 x 51 mm** (new `lift_inner_section`, from the same drawing: "Inner profile 209.5x51"). It was in the datasheet, as an embedded image the PDF's text layer does not contain, so no placeholder was created. Strictly smaller than the outer on both axes and nested in it. **It does not remove the overlap in volume** (the outer collision is a solid box; the housing wall is not in the datasheet): 0.003740 m3 at L1000, was 0.020160.
- **The adapter plate is its own link, 360 x 100 x 10 mm**, rear edge flush with the column's rear face, and the arm profile stands on it (its parent now), pose from the carriage unchanged.
- **Collision matrix: 99 disabled pairs (was 95), 567 enabled of 666 (was 535 of 630).** Four new pairs are the plate's; one pair relabelled `Adjacent` to `ConstantTransform`. The stage nesting moved no pair (a structural neighbour, never evaluated). M03 PASS in all three runs.
- **Density band of the lift mass re-fitted, 100 to 1600 kg/m3 (was 1200): a judgement call**, because the sourced section puts the catalogue 11.7 kg at 1533 kg/m3. Please look at it.
- **ADR-010 section 2 corrected** (design folder, not in this repository): it attributed Alex's Slack sentence of 7 Oct to the register; the register's ARM-MOUNT says the profile stands on a 10 mm adapter plate.
- **Evidence:** A6 x3 25 / 3 / 0; A7 x3 14 / 2 / 0, 12 / 2 / 2, 14 / 2 / 0. The 12 / 2 / 2 run failed G02 (`parameter_bridge-3` -11, the known shutdown crash) and AUX (a race in `test_zz_a_reading_beyond_the_epsilon_is_not_hidden`; found and fixed in P29, above). Six RViz captures: `evidence/p28/rviz/`.
- **Tracked upstream, not merge gates for this PR:** the shutdown crashes of `apriltag_node` and `move_group` are [openamr-platform-sw#84](https://github.com/openAMRobot/openamr-platform-sw/issues/84); the gz-sim server teardown and the `ros_gz` `parameter_bridge` SIGSEGV are drafted in `docs/issues/gz-sim_server_teardown_segfault_and_hang.md` and `docs/issues/ros_gz-bridge_teardown_segfault.md` and **are not filed: this workspace has no GitHub credential**, so their URLs are `<TO BE FILLED IN WHEN FILED>` (gazebosim/gz-sim and gazebosim/ros_gz). Neither is investigated or allow-listed here.
- **Still open:** V01 is WARN (`lift_mass_tl3_400` null; nobody has gone through the checklist on a screen); the stages overlap in volume; the inner stage's length, centring and orientation and the plate's rear-flush placement are assumptions; the column's parent frame; Alex's acceptance of the edits to his xacro; 5 STALE `NEAR-FIELD-FRAMES` citations in `check_decision_currency.py` (before this step; not touched).

## What changed since `e5cf613` (P22, P23 and P24)

- **The column statement in the reports.** Mohammad's review of `9406af6` on this PR (on GitHub, not in the repository) says: "The new reports still use platform 174731c and explicitly state that the lift column has no collision geometry. They do not close the current combined-model handoff after platform #70/#71 or the requested column collision placeholder." Two cases. The reports he reviewed (P18, code `bb39c83`, before `b45d8f6`) said it truthfully: there were no column boxes yet and the base was `174731c`; his objection was right for them. What he asked for then arrived: the column placeholder with Alex's follow-ups (`b45d8f6`, merged in P19) and the base with the pin `dfd44a5`. But the A6 and A7 generators kept writing the old sentence, so **24 reports say the column has no collision geometry when it has: every committed A6 and A7 report of P19 to P21, `evidence/reports/20261018T191032Z_dev.json` to `20261019T010818Z_a7.json`.** That sentence was a generator string, **not a finding**: nobody measured it. P23 removed it from `build_evidence.py` (the limitation and the V01 item 1 note) and `build_a7_evidence.py`. No committed report is edited; `evidence/reports/README.md`, section P23, says which are true (32, P15 to P18) and which are false (24). An earlier version of this section said the statement "did not describe the model"; for the reports Mohammad cited, it did.
- **`datums.yaml` re-cited, so the datums revision is `2026-10-20.a` and every `config_id` changed (P23).** `base_link_height_above_floor` cited FRAMES-REP105 at P-03 rev19.0; 0.098 m is stated in CAMERAS at P-03 rev19.4 (`decisions.yaml:600`), and FRAMES-REP105 says only "axle height". The value is unchanged. The `lift_column_section` note, which said the column has "no collision, no visual", was corrected in the same bump. Every id in the A6 reports (all 14) and the A7 reports (all 9) differs from the ids of P21 and before. The datums revision is a `config_id` input, so an id recorded under another revision is not comparable (description README: "Change a value, bump `revision`. The revision goes into every configuration id"), and the note under "`config_id` inputs changed" below (Satya, C9: an id recorded before the change is not comparable with one computed now) already says that; no input was added.
- **Decision currency: 0 STALE (P23).** `tools/check_decision_currency.py` read the register at `9109d3e` and found one stale citation (the `datums.yaml` one above) in P22; after the re-citation it finds none (`evidence/p23/`).
- **The placeholder boxes have one colour and the geometry has been measured (P25).** The two column stages and `arm_profile_link` had no material, so RViz drew them red and Gazebo black; they now use one named material (tan) in the URDF and in a `<gazebo>` block per link, and the two column visuals lost their `name` because sdformat ignores that block for a named visual. **This edits Alex's `openamrobot_upperbody.xacro` and needs his acceptance**, like P15's opt-in parameters. Measured, not changed, and waiting on Alex: the column's two stages interpenetrate (M03 cannot see it: the pair is `Adjacent` across `lift_joint`), the column's lateral side is invented, the arm profile neither overlaps the column nor stands on it, and `lift_column_base_joint`'s parent is `base_footprint`. A6 x3 25 / 3 / 0 and A7 x3 14 / 2 / 0 on `1165166`, nothing moved against P24; the record is the last section of `docs/v01_visual_check.md`, the numbers are in `evidence/p25/`.
- **The bridge crash is now recorded as data (P24).** The sidecar of each launch test (G01, G02, M01, M02, M07) records the name and exit code of every process of its launch, and the A7 report lists them per launch (`process_exits`). **Run-level rate, plainly: 3 of the 12 A7 runs of P19 to P23 on `dfd44a5` failed on this crash, 25 percent, across two different launches (G02 twice, M07 once).** The three runs of P24 (`15109f9`), the three of P25 (`1165166`) and the three of P27 (`26f539c`) did not fail; **the three of P28 (`a14a3ff`) had one failure on it (run 2, G02, `parameter_bridge-3` -11, recorded in the report's `process_exits`), and the three of P29 (`c84cfbd`) did not fail either, so counting all of them it is 4 of 27, 15 percent.** The cause of three of those four is the console's -11 (OBSERVED-NOT-RECORDED: it cannot be recovered); the cause of the fourth is the report's `process_exits` (RECORDED). The two bases are kept apart and the launch-level figures are not summed. See "The `parameter_bridge` SIGSEGV" under Tests. P23 text that follows is kept for the record: the bridge exit rate was labelled OBSERVED-NOT-RECORDED and there was a third occurrence. The JSON never recorded the -11, so the rate is as unbacked as the "1 of 7" that was retracted. I chose to label it everywhere it appears and to make `tools/check_evidence_backing.py` accept it only while the text says so, rather than change the stop-helper sidecar (which would not recover any past occurrence).
- **The manipulation commit is settled.** The P19 to P23 runs used `openamrobot-manipulation` `795143e`, not the `095e6c7` of Alex's block (see Evidence).
- **New evidence**: A6 three runs 25 PASS / 3 WARN / 0 FAIL and A7 three runs on `77374a0` (14 / 2 / 0 twice, 13 / 1 / 2 once), and the same again on `15109f9` (A6 25 / 3 / 0 three times, A7 14 / 2 / 0 three times), `evidence/reports/20261020T*`; the outputs of the two prototype checks, `evidence/p22_checks/`; documents corrected in P22 (CHANGELOG, `docs/verification.md`, the READMEs, and the design folder's DESIGN_LOG, VERIFICATION, DATUMS, FRAMES and ADR-010).

## Evidence

**Current (the P19 to P23 runs, 16 to 20 Oct).**

- Base SHA: `f57e54cfc9e7f9536e94c52749502cc983fb908e` (openAMRobot/openamr-upperbody-sw `main`, the base of this PR on GitHub).
- Head SHA: the tested code of the latest runs (P24) is `15109f9a501aea9f1d7d506870043029686d18bf` (`git_hash` of `evidence/reports/20261020T12*Z_a7.json` and `20261020T115*Z_dev.json`, clean tree); the P23 runs are on `77374a07e58013af4a4d512a6d444b4a5b4ccf1c`, the P19 to P21 runs are on `538a32a`, `24c66f5` and `fe2e821da6ef7d9cf11bf6e3e9f6c05b77b6f151`. The pushed head adds only reports and documents to the tested code. Before P22 and P23 the PR head was `e5cf6135b04b7c2febfc5dbe9fd4d6d8ef71cdd0`.
- Workspace: `openamr-platform-sw` `dfd44a55457bf5c2d4f81966b862a0865fd52aca` (#70 and #71, on platform-sw `main` since 16 Oct); `src/upstream/openarm_description` `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` and `openarm_ros2` `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` (the pins in `config/upstream_pins.yaml`; each report's `upstream` block). Simulator versions are in each A7 report's `simulator` block (Gazebo 8.15.0, `gz_ros2_control` 1.2.20, `ros_gz_sim` 1.0.24, MoveIt 2.12.4).
- The reports do not record the `openamrobot-manifest` or `openamrobot-manipulation` commits; the workspace does. Every run on this workstation, P15 to P23, used `openamrobot-manipulation` **`795143e55fb98faf60e259f4323c52d374514b38`** and manifest `8111c48e48f93c20f2142534ffc340857234bf7a`: both checkouts have a single reflog entry, the clone of 5 Oct, and have not moved (`evidence/p23/workspace_checkouts.txt`). The `095e6c7bbd270db6d21de9664a654bb37919a2eb` in Alex's block below is his container's checkout: it is two commits ahead of `795143e`, and those two change only `AGENTS.md`, `README.md`, `integration/README.md` and `integration/openarm_v2_combined_model_inventory.md` (GitHub compare, same file). Neither commit touches the baseline `.repos` that C01 reads.
- Machine: the author's WSL2 workstation (Ubuntu 24.04, ROS 2 Jazzy binaries). Nothing here ran on a hosted runner: PR #14's `quality/test` job is skipped by configuration (see "Not verified").
- Reports and per-run detail: `evidence/reports/README.md`, sections P19 to P21 and P23.

**Earlier (Alex's runs on `703fce9`, platform-sw `8531089`), kept unchanged apart from the notes in brackets.**

- Base SHA: `f57e54cfc9e7f9536e94c52749502cc983fb908e` (openAMRobot/openamr-upperbody-sw `main`)
- Head SHA: `703fce9f44c73a67dbfb8799e5dc89920d3bbb8c` (the tested code; the commit after it changes only this file)
- Workspace: `openamrobot-manifest` `8111c48e48f93c20f2142534ffc340857234bf7a` (`vcs import`). In that workspace:
  - this repository at the head above;
  - `openamr-platform-sw` `8531089e9eca1fdc56998853f0f67bf3f1e12434`: PR #71, branch `fix/chassis-collision-caster-overlap`, on `main` `d619012` (#70). #71 raises the chassis collision box bottom to 0.055 m. It was not merged at the time of writing [it merged on 16 Oct as platform-sw `dfd44a5`, which this branch now pins];
  - `openamrobot-manipulation` `095e6c7bbd270db6d21de9664a654bb37919a2eb`;
  - `src/upstream/openarm_description` `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` and `src/upstream/openarm_ros2` `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` (the pins in `config/upstream_pins.yaml`).
- Machine: Ubuntu 24.04 container, ROS 2 Jazzy binaries, `/usr/bin/python3` = 3.12, 4 cores, no GPU (ogre2 renders in software; the simulator runs at about 0.4x real time). controller_manager 4.48.1, MoveIt 2.12.4.
- Run sequentially with nothing else running; every launch under a timeout, and no simulator, bridge, `move_group` or `robot_state_publisher` left running after any step.

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
rm -rf build install log
colcon build --symlink-install --packages-skip openarm_hardware openarm
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
timeout 3600 colcon test --executor sequential --packages-select openamrobot_upperbody_description openamrobot_upperbody_control \
    openamrobot_upperbody_gazebo openamrobot_upperbody_moveit openamrobot_upperbody_bringup
colcon test-result --verbose --test-result-base build/<package>        # for each of the five packages
timeout 1800 bash src/openamr-upperbody-sw/ros2/src/openamrobot_upperbody_description/scripts/verify_a6.sh    # twice
timeout 3600 bash src/openamr-upperbody-sw/ros2/src/openamrobot_upperbody_bringup/scripts/run_a7_regression.sh  # twice
```

The evidence reports are `20261016T185914Z_dev.json` (A6), and `20261016T190708Z_a7.json` and `20261016T191444Z_a7.json` (A7), written by the harness from the clean tree at `703fce9`. The earlier runs are `20261016T150136Z_dev.json` and `20261016T151057Z_a7.json` (`ff8f9ad` on platform-sw `d619012`), and `20261016T140434Z_dev.json`, `20261016T140930Z_dev.json`, `20261016T142206Z_a7.json` and `20261016T143422Z_a7.json` (`8e1aba1` on `15d0eb5`). None of them is committed in this change.

## Tests

### This step: the merge head, platform-sw pinned at `dfd44a5` (#70 and #71)

**Pinned base: platform-sw `dfd44a55457bf5c2d4f81966b862a0865fd52aca`** (#71 on top of #70 `d619012`). Its `imu_link` is `(0.0544, 0, 0.1210)`, yaw -90 degrees (set by #70). Reports and per-run detail: `evidence/reports/README.md`, sections P19 and P20 (and P21 and P23 for the later runs).

| base | tested code | A6 (`verify_a6.sh`), 3 runs | A7 (`run_a7_regression.sh`) |
|---|---|---|---|
| `d619012` (#70 only) | `538a32a` | 25 PASS / 3 WARN / 0 FAIL, each | **12 PASS / 2 WARN / 2 FAIL (M03, AUX), 3 of 3 runs** |
| `dfd44a5` (#70 + #71) | `538a32a` | 25 / 3 / 0, each | 14 / 2 / 0, then **12 / 2 / 2 (G02, AUX) twice** |
| `dfd44a5` (the pin) | `24c66f5` | 25 / 3 / 0, each | **14 / 2 / 0, 3 of 3 runs** |
| `dfd44a5` (the pin) | `fe2e821` | 25 / 3 / 0, each | **14 / 2 / 0, 3 of 3 runs** |
| `dfd44a5` (the pin) | `77374a0` (P23: generators and `datums.yaml` fixed, datums revision `2026-10-20.a`) | 25 / 3 / 0, each | 14 / 2 / 0 twice, and **13 PASS / 1 WARN / 2 FAIL (M07, AUX) once** (the bridge SIGSEGV below) |
| `dfd44a5` (the pin) | `15109f9` (P24: the exit codes of every launch process are recorded) | 25 / 3 / 0, each | **14 / 2 / 0, 3 of 3 runs** |

WARNs are E01, W02, V01 (A6) and M07, V01 (A7). `WARN` is not `PASS`.

**Why two pins.** On `d619012` M03 FAILs in 3 of 3 runs and AUX with it: #70's chassis box (bottom 0.050 m) overlaps the four caster-wheel spheres (top 0.0524 m) by 2.4 mm, so `base_link` collides with each `{fl,fr,bl,br}_wheel_link` in every state, the derivation finds 91 of the 95 pairs of the contract, and the two ACM regeneration tests fail. platform-sw's own commit for #71 records the same overlap, the same 2.4 mm and the M03 failure. On `dfd44a5` #71 raises the box bottom to 0.055 m (2.6 mm clear) and M03 passes in all six runs. The matrix was neither regenerated nor changed for either base. The pin is `dfd44a5` by the architecture chat's decision, because the `d619012` failure is a defect already fixed upstream of it.

**A6: 24 PASS / 4 WARN became 25 / 3 because T08 cleared, and the cause is in platform-sw, not in this repository.** T08 ("collision mesh is the same file as the visual mesh") WARNed on six platform-sw links (`base_link`, the four caster links and `lidar_link`, each reusing its visual STL). platform-sw #70 (`d619012`) replaced those collision meshes with boxes and cylinders ("chassis, castors and LiDAR are boxes/cylinders instead of STL meshes"), so T08 finds none. No commit of this branch changed T08 or its inputs. (Alex's section below already says T08 is PASS after #70.)

**The `parameter_bridge` SIGSEGV at shutdown: A7 on the pinned base is not reliably green.** In 3 of the 12 A7 runs on `dfd44a5` of P19 to P23 (runs 2 and 3 of the P19 set, both G02; and run 1 of the P23 set, M07; none of the three P24 runs), a check FAILed, and AUX with it (all three **recorded**: `evidence/reports/20261018T200829Z_a7.json`, `20261018T201608Z_a7.json` and `20261020T105921Z_a7.json`), and the cause was a `parameter_bridge` process that died with exit code -11 while the launch was stopping. **For the two P19 failures that exit code is OBSERVED-NOT-RECORDED**: it was read from the console at the time and quoted in the issue draft, and no report, sidecar or log in this repository carries it. **For the P23 failure the console's last lines are kept** (`evidence/p23/a7_run1_console_tail.txt`: "Proc parameter_bridge-4 exited with code -11"), still not the sidecar, and the report records only that M07's `test_m07_processes_started_and_exited_cleanly` failed. In G02's two failures the physics had passed and its server stopped in 1.4 s; in the M07 failure M07's joint rows were as in the passing runs (no failing joint, the four WARNs) and the stop took 1.42 s. Facts:
- It is the stock `ros_gz_bridge` executable crashing in its own teardown. The instance that crashed in the two P19 failures is `base_joint_states_bridge`, the bridge this branch's `sim.launch.py` adds for the base's joint states (not platform-sw's `platform_bridge`), in the Bullet-Featherstone gripper launch, within 0.1 s of the SIGINT that stops the launch. The P23 console tail names the process `parameter_bridge-4` and not the instance, and the launch was M07's: the crash is not specific to the gripper test.
- **OBSERVED-NOT-RECORDED rates** (the exit codes were read from the console, so none of these derives from data): 3 of 48 launches on `dfd44a5` and 3 of 72 in all A7 runs since P18; none on `d619012` or in P18; the six runs of P20 and P21, on the same pin, were clean: 0 of 24 launches. What the reports do record is which checks failed in which runs (G02 and AUX in two P19 runs, M07 and AUX in one P23 run) and that they passed in the others; the number of launches is four per run. I did not investigate the cause and cannot say it is unrelated to this branch's launch (the P17 stop helper, the spawner gating): it is not attributable to a change here, but that is an absence of evidence, not a test.
- **It is not allow-listed.** The process guard allows SIGABRT (-6) for `parameter_bridge`, a known once-in-a-while heap-corruption abort, and does not allow SIGSEGV; nothing was widened. The test fails when it happens, which is the intended behaviour.
- **Up to P23 the only observed rate is 3 of 72 launches since P18, and it is OBSERVED-NOT-RECORDED** (it was 2 of 60 until the P23 run). **From P24 the exit codes are recorded**: in the three P24 runs, 0 of 12 launches with a `parameter_bridge` ended in -11 (derived from `process_exits`, `evidence/reports/20261020T120539Z_a7.json` and the two after it; BACKED). The observed and the recorded figures are not added together. An earlier figure, "parameter_bridge exits -11 at shutdown in about 1 of 7 G02 runs" (P16), appeared in a report to the architecture chat **with no evidence behind it, and it was never recorded**: I searched every P16 and P17 report and log in the repository (the twelve P16 A7 reports on `9bc327c` and `aa94fc3`, the 36-run orphan count, the shutdown timings, the bisect logs) and found no `parameter_bridge` exit -11. The only bridge crash recorded before P19 is one SIGABRT (-6, "corrupted double-linked list") in a P12A G01 log; the one G02 FAIL in P17's 36 runs was a JSON-parsing bug in the test (`gz_link_poses`). It is an unbacked claim we made, not a rate.
- Drafted, **not filed** (no credential): `docs/issues/ros_gz-bridge_teardown_segfault.md` for gazebosim/ros_gz. Issue URL: `<TO BE FILLED IN WHEN FILED>`.

**Alex's spawner change (`1637743`), and what the timing does and does not show.** Across the 48 simulator launches of the twelve A7 runs of P19 to P21 (G01, G02, M02, M07 in each): no force-stop (W04 PASS in all twelve), and **all 48 stopped on the first step, in 1.32 to 1.53 s**, with no SIGINT-to-SIGTERM escalation (the force-stop path took about 9 s when it fired). That is the timing to cite, and what it shows must be stated: **the same stop helper stopped 36 of 36 launches in 1.32 to 1.44 s before the spawner change** (P17), so these timings are the stop helper's, not an effect of the spawner change. Zero force-stops in 48 launches is also uninformative at the prior rate (2 in about 61: about 1.6 expected, so zero has about a 20% chance with nothing changed). What does bear on the spawner change: `test_sim_launch_spawners.py` fails 3 of 4 on the old `sim.launch.py`; and on `49d2368` the old launch left `joint_state_broadcaster` inactive in two A7 runs (G01 NOT_RUN, G02 and M02 FAIL), where none of the 48 launches failed to start its controllers.

**M02 and M07, twelve A7 runs.** M02: 0 excursions in all twelve; largest overshoot 1.28e-6 to 3.23e-6 rad against the 1e-5 epsilon (not widened). M07 is WARN in all twelve (the four upstream-limit joints, no joint missing from the coverage).

**RViz.** Six real RViz2 screenshots at L1000 / L1175 / L1350 x bp000 / bp050: `evidence/renders/rviz_*.png` (`evidence/renders/rviz/README.md`). **The two lift-column boxes render solid red.** The URDF gives them no material and I did not investigate why they are red (I did not check whether it is RViz's default for a visual without a material, or something else). V01 stays WARN: no person has looked at the running simulator.

### Earlier results (Alex's, on `703fce9` and before)

Summary:
- **On `703fce9` with platform-sw `8531089` (#71, on main with #70):**
  - `colcon test`, five packages: 826 tests, 805 passed, 2 failed, 19 skipped.
  - A6: 25 PASS / 3 WARN / 0 FAIL / 0 NOT_RUN.
  - A7, twice: 11 PASS / 2 WARN / 2 FAIL / 0 NOT_RUN. M03 and AUX fail on two negative tests whose example pair #71 made collision-free. The committed collision matrix is correct; see below.
- **On `ff8f9ad` with platform-sw `d619012` (main, with #70):**
  - A6: 25 PASS / 3 WARN / 0 FAIL / 0 NOT_RUN.
  - A7: 10 PASS / 2 WARN / 3 FAIL / 0 NOT_RUN. M03 and AUX fail on #70's chassis/caster-wheel collision overlap; M01 fails on a harness start-up race.
  - Test counts behind those two runs: 636 passed (description), 8 passed (control), 28 passed (gazebo), 78 passed / 5 failed / 3 skipped (moveit), 65 passed / 3 failed (bringup, A7 run).
- **On `8e1aba1` with platform-sw `15d0eb5`:**
  - `colcon test`: 825 tests, 806 passed, 0 failed, 19 skipped.
  - A6: 24 PASS / 4 WARN / 0 FAIL, twice.
  - A7: 13 PASS / 2 WARN / 0 FAIL, twice.

### On `703fce9`, platform-sw `8531089` (PR #71: the chassis box clears the caster wheels)

Run sequentially after a rebuild. Every launch ran under a timeout, and nothing was left running after any step.

| Run | Result |
|---|---|
| `check_urdf`, `base_tilt` bt05 / bt10 / bt15 | All pass |
| Chassis box vs caster spheres | The box spans 0.0550 to 0.2950 m above the floor; each caster-wheel sphere tops out at 0.0524 m. **Clearance +2.6 mm on all four**, at all three tilts. The caster boxes (top 0.0522 m) are clear too. |
| `colcon test`, five packages | **826 tests: 805 passed, 2 failed, 19 skipped**. description 636 / 0 / 0, control 8 / 0 / 0, gazebo 28 / 0 / 0, moveit 81 / 2 / 3, bringup 52 / 0 / 16 |
| `verify_a6.sh` | **25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |
| `run_a7_regression.sh`, run 1 | **11 PASS, 2 WARN (M07, V01), 2 FAIL (M03, AUX), 0 NOT_RUN** |
| `run_a7_regression.sh`, run 2 | **11 PASS, 2 WARN (M07, V01), 2 FAIL (M03, AUX), 0 NOT_RUN** |

- **M01 passes at L1000, L1175 and L1350 in both runs.** `703fce9` makes it wait for the controllers before its first pose.
- **G01, G02, M02 and M04 to M06 pass.** M07 is WARN on the four upstream-limit joints, with all 16 expected joints covered. Bringup in A7: 68 / 0 / 0.
- **M03 and AUX fail on two negative tests, not on the matrix.** `test_m03_3` and `test_m03_4` mutate the matrix around a fixed example pair, `CONSTANT_AND_COLLIDING = ("base_link", "bl_caster_link")`, "collides in the default state". #71 made that pair collision-free (2.8 mm), so the expected FAIL no longer comes.
  - The committed matrix is correct for this base: `regenerate_acm.sh` derives exactly the contract (95 pairs, fingerprint `sha256:50ba49b5…`, 0 differences). So `--write` changes nothing, and nothing is regenerated.
  - Proposed fix, not in this change: use `("base_link", "lidar_link")` as the example. It is Adjacent, has a fixed transform, and the lidar cylinder lies inside the chassis box. With only that constant changed, both tests pass (2 of 2, scratch copy).

### On `ff8f9ad`, platform-sw `d619012` (main, with #70)

Run once each, sequentially, after a rebuild.

| Run | Result |
|---|---|
| `verify_a6.sh` | **25 PASS, 3 WARN (E01, W02, V01), 0 FAIL, 0 NOT_RUN** (exit 0). T08 is PASS now: #70 removed the base's collision meshes. |
| `run_a7_regression.sh` | **10 PASS, 2 WARN (M07, V01), 3 FAIL (M01, M03, AUX), 0 NOT_RUN** (exit 1) |

Test counts behind those two runs (passed / failed / skipped):

| Package | Result |
|---|---|
| `openamrobot_upperbody_description` | 636 / 0 / 0 |
| `openamrobot_upperbody_control` | 8 / 0 / 0 |
| `openamrobot_upperbody_gazebo` | 28 / 0 / 0 |
| `openamrobot_upperbody_moveit` | 78 / 5 / 3 |
| `openamrobot_upperbody_bringup` | A6 run: 52 / 0 / 16 (A7-gated). A7 run: 65 / 3 / 0 |

**The three A7 FAILs.** Neither cause is in this branch's code:

- **M03 and AUX: platform-sw #70's collision geometry.** The chassis box of `base_link` (0.780 x 0.580 x 0.245 m, bottom at z 0.050 m above the floor) and the four caster-wheel spheres (r 0.026 m, centre z 0.0264 m, top 0.0524 m) overlap by 2.4 mm. That is a permanent collision of constant pairs, `base_link` with each `{fl,fr,bl,br}_wheel_link`.
  - `regenerate_acm.sh` refuses to disable such a pair ("a permanent collision of the model, not disabled, nothing written"): 91 derived pairs against 95 in the contract.
  - The contract still disables the four as `ConstantTransform`, a label M03 shows to be false.
  - Fixing it belongs in platform-sw: a caster-wheel sphere clear of the chassis box, or a box that ends above the wheels. It is not disabled here.
- **M01: a start-up race in the M01 harness.** It is not a model change.
  - At every lift stop the first `left_arm` goals ended `CONTROL_FAILED`: "Action client not connected to action server: left_arm_controller/follow_joint_trajectory". The controllers of the mock stack were still being spawned when M01 sent its first pose.
  - M01 does not wait for its controllers to be active; M02 and M07 do, through `a7_support.wait_for_controllers_active`.
  - On `8e1aba1` it passed in both A7 runs. The fix is for M01 to use the same wait. It is not part of this change.
- **M07 is WARN** (the four upstream-limit joints) and covers all 16 expected joints.

**The base description of #70, checked on Jazzy** (`robo_urdf.urdf.xacro`):
- `base_tilt` bt05, bt10 and bt15 expand and pass `check_urdf`; bt99 is rejected.
- 25 links and 24 joints: no duplicate names, one parent per link, root `base_footprint`.
- Every frame of #70's table matches, from `base_link`, to 0.05 mm, and the axes match:
  - `base_camera_link` (0.387, 0, 0.046), pitch 5/10/15° up;
  - `imu_link` (0.0544, 0, 0.1210), yaw -90° (sensor X to -y, Y to +x, Z to +z);
  - `lidar_link` (0.330, 0, 0.124), yaw pi;
  - `us_front_*` (0.3906, ±0.130, -0.0066) and `us_rear_*` (-0.3886, ±0.130, -0.0066), yaw pi;
  - `tof_front` and `tof_rear` (±0.399, 0, -0.0066), 20° down, the rear one yaw pi.
- In a 60 s headless `gz_simulator` run, all eight sensor topics publish with their own `frame_id`:
  - `/imu`, `/tof/front` and `/tof/rear` at 50.0 Hz sim time;
  - `/scan` and the four `/ultrasonic/*` at 10.0 Hz.

### On `8e1aba1`, platform-sw `15d0eb5` (before #70)

| Run | Result |
|---|---|
| `colcon test`, five packages | **825 tests: 806 passed, 0 failed, 19 skipped** |
| `verify_a6.sh`, run 1 | **24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |
| `verify_a6.sh`, run 2 | **24 PASS, 4 WARN (T08, E01, W02, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |
| `run_a7_regression.sh`, run 1 | **13 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |
| `run_a7_regression.sh`, run 2 | **13 PASS, 2 WARN (M07, V01), 0 FAIL, 0 NOT_RUN** (exit 0) |

`colcon test` per package (passed / failed / skipped):

| Package | Passed | Failed | Skipped |
|---|---|---|---|
| `openamrobot_upperbody_description` | 635 | 0 | 0 |
| `openamrobot_upperbody_control` | 8 | 0 | 0 |
| `openamrobot_upperbody_gazebo` | 28 | 0 | 0 |
| `openamrobot_upperbody_moveit` | 83 | 0 | 3 |
| `openamrobot_upperbody_bringup` | 52 | 0 | 16 |

The 19 skips are listed in #16, and each reason names it. `config_id`: 14 of the 15 recorded ids differ from `49d2368`, because of the column geometry in `b45d8f6`.

**Failing runs before the change, one per behaviour change:**

| Change | What failed before |
|---|---|
| `b45d8f6` (a) | The column boxes with the old contract: A6 FAIL 22 / 4 / 2 (T04: "contract says collision_free"). The old collision matrix with the boxes: A7 M03 FAIL (95 derived pairs vs 83 in the contract). |
| `b45d8f6` (b) | On `49d2368`, M07 was FAIL on the same four joints. In this run it is WARN. |
| `1637743` | `test_sim_launch_spawners.py` fails 3 of 4 on the old `sim.launch.py`. On this machine the old launch left `joint_state_broadcaster` inactive (G01 NOT_RUN, G02 and M02 FAIL in two A7 runs on `49d2368`). |
| `84d80a9` | `test_a7_controller_deadline.py` fails 3 of 4 on the old helper (all-active just after the deadline returned ok). `test_m07_coverage.py` fails 5 of 5: its helpers do not exist in the old code. |
| `20886a6` | T04's pinned list fails until the two frames are listed. |
| `ff8f9ad` | On platform-sw `d619012`, the old `test_t07_collision.py` fails 2 tests (seven consumed sensor frames with a visual and no collision, then KeyError). The new file passes 9 of 9. |

## Disagreements with the register and platform-sw (audit of 19 Oct)

From comparing `openAMRobot/.github` `decisions.yaml` (commit `9109d3e`, in force P-03 rev19.6), platform-sw `dfd44a5` and this repository. None is ours to settle and none is changed by this PR.

- **Near-field sensor x positions.** The register's NEAR-FIELD-FRAMES (P-03 rev19.0, 12 Oct) gives x ±0.390 for the four ultrasonics and the two ToF sensors (`decisions.yaml:374-376`). platform-sw's merged #70 has 0.3906 and −0.3886 for the ultrasonics and ±0.399 for the ToF sensors, status `measured` from CAD Rev B (`ros2/src/openamrobot_description/config/sensor_frames.yaml:43,61,68,75`). The register was updated for Alex's 16 Oct decisions (`imu_link`, the base camera, the LiDAR) and not for these. Which numbers are the decision is Alex's.
- **`base_link` at 0.098 m is stated in CAMERAS (P-03 rev19.4), not in FRAMES-REP105** (`decisions.yaml:600`, against `:339-343`). Our `datums.yaml` cited FRAMES-REP105 at rev19.0 for it; P23 re-cited it (see "P23" below). The value is unchanged.
- **The column's lateral side was unsourced everywhere until P28.** The register gives the 240 mm fore-aft side only (`decisions.yaml:1081`); `datums.yaml` set the lateral side equal to it, and since `b45d8f6` that invented number was collision geometry. It is now the DOLD datasheet's 60 mm (P28); the register still has no lateral side.
- **platform-sw's `sensor_frames.yaml` cites BOM Issue 7.7 and P-03 rev19.0** (`:6`); the register is at BOM Issue 7.10 and P-03 rev19.6 (`decisions.yaml:55-57`, `:153-159`).
- **The register has no entry for the rest-state rule (M07) or for the platform-sw pin.**

## Not verified

- No hardware.
- No supplier CAD for the column: the boxes carry the datasheet's sections (P28) but are boxes, the outer is solid, and the inner stage's length is the stroke, so collision checks against the column body are only as good as the box.
- No F2S check, no reach acceptance, and no physical lift safety (power loss, E-stop holding).
- `tl3_400` is not verified: it is refused until its supplier data exist.
- No human visual review in RViz or Gazebo: V01 is WARN; L1000/L1175/L1350 × bp000/bp050 are still to be looked at.
- Not run on a hosted CI runner or on a machine with a GPU. **The hosted job `quality/test` is skipped by configuration, not by a missing dependency**: `.github/workflows/repository-quality.yml` calls the reusable workflow without `verify: true`, whose default is false (`openAMRobot/.github` `repository-quality-reusable.yml:36-40`, `:244`), and the check run for `e5cf613` is `skipped` with the same start and end time (GitHub API, 19 Oct). Running A6 there needs more than flipping that input (the job would check out this repository alone); the proposal is in the P22 report, and no workflow file was changed.
- The gz-sim teardown hang (the server ignores the stop request) is not fixed; see "Findings that are not resolved" below. The detached stop helper forces such a server; since P18 that is a **WARN (W04)**, not a FAIL, and a server still running after the force-stop is still the launch test's FAIL.
- A7 on platform-sw `8531089` is not green: M03 and AUX fail on the outdated negative-test example pair (see Tests; one-line fix proposed). On `d619012` it also failed on #70's caster overlap and on the M01 race, both fixed since. #71 is not merged. (P19: #71 has since merged on platform-sw `main` as `dfd44a5`; this step's results on both pins are under "Tests".)
- On `d619012`, `colcon test` of the five packages was not run as a separate step; its counts come from the A6 and A7 runs. On `8531089` it was run.
- The #70 sensor values are checked against #70's own table, not against the CAD.
- `license/cla` on PR #14 is still pending.
- The six conditional skips that did not trigger here (C02, C01, datums revision, W02, SRDF import) are listed in #16 but not changed.
- V01's recorded measurement text in `build_evidence.py` said "0 of 478 enabled pairs" (P16); since P28 it points to the re-measurement: 0 of 567 enabled pairs colliding at the six configurations (`evidence/p28/v01_geometry.json`).
- The message of `ed8c56d` says the header comments change `config_id`. That is not so: the canonical URDF is `ET.canonicalize`d, which drops comments. The changed ids come from `b45d8f6`.

## Findings that are not resolved

Stated as they are. None of the three gates anything.

- **The rise of the M02 clamp activation count, from about 200 to about 1800 per run, is unexplained.** It gates nothing. The count scatters 646 to 1054 inside one unchanged configuration (1795 / 957 / 1832 on three runs of a tests-only diff), so it is retired as a criterion and reported as `diagnostic_only`. Why the new body's right finger rests within 1e-5 rad of its open limit while the left one rests 2e-4 to 5e-4 inside is not explained (cause unknown; the effort at rest was measured in both trees, `evidence/m02_bisect/RESULTS.md`). The M02 bisect is closed.
- **The rest-position hypothesis was tested and refuted.** The hypothesis was that the lift body puts the right finger on its open limit where the old mast body (`7daa2ab`) did not. The old body rests within 2.5e-5 rad of the same four limits (both `joint4`, both `finger_joint1`), and the finger effort at rest has the same sign in both bodies: a gripper's `open` and the arm's `home` are limits in upstream's joint definitions. The named poses that sit on a limit are listed for both bodies in `evidence/w03/` as evidence files; W03 is not a registered check. This does not settle M07: M07 stays as Alex decided it (WARN on those four joints), and removing it is his call.
- **The gz-sim issue is drafted and UNFILED.** A `gz sim` server that is asked to stop sometimes never begins its teardown and survives SIGINT, the stop request and SIGTERM (a segfault in a libgz-sim8 thread, and a hang). The draft, with the counts and a reproduction, is `docs/issues/gz-sim_server_teardown_segfault_and_hang.md`. **It has not been filed: the workspace has no GitHub credential. Issue URL (gazebosim/gz-sim): `<TO BE FILLED IN WHEN FILED>`.** A second draft, for the `parameter_bridge` SIGSEGV at shutdown (gazebosim/ros_gz), is `docs/issues/ros_gz-bridge_teardown_segfault.md`, **also not filed**. Issue URL (gazebosim/ros_gz): `<TO BE FILLED IN WHEN FILED>`. A bare server (empty world, no ROS) did not reproduce it: 30 of 30 clean in each of two stop modes.

**What M02 gates on, and what it read.** Two numbers: readings beyond the epsilon (must be 0) and the largest overshoot magnitude (must be below the epsilon, 1e-5 rad). In the three A7 runs of P18 (`bb39c83`): **0 excursions in all three; largest overshoot 1.42e-6 / 2.37e-6 / 1.76e-6 rad**, against the 1e-5 epsilon. The epsilon has not been widened. This step's nine runs are under "Tests".

## Review items taken up in P21

From Mohammad's (panthera-momagdii) and Satya's reviews of this PR.

- **W04 no longer passes on evidence that is not there** (Mohammad, inline, `build_a7_evidence.py` at `9406af6`). The stop helper's report can still be in state `started` when `launch_guard.read_gz_stop_report` gives up waiting, and `gz_server_shutdown_report` keeps that object in the sidecar; a truthy report that was not `force_stopped` returned PASS, as did a report with no state or an unrecognised one. W04 now reads a completed `stopped` as PASS and `force_stopped` as WARN, and everything else as **NOT_RUN**: `started`, no state, an unrecognised or non-string state, a report that is not an object, an absent report, and `no_server` (the helper found no server, so nothing shows a server stopping on request). A regression drives the timed-out path end to end, from a report left in `started` through `read_gz_stop_report` and `gz_server_shutdown_report` into W04 (the absent-helper case did not cover it); six of the new tests fail on the old code. This is the H01/H02 class of hole, a check passing when its evidence is missing, reintroduced in a check written by us. The survivor check is untouched.
- **M03 regression for platform-sw #71** (its reviewer's request): `test_m03_caster_clearance.py` reads the chassis box and the four caster-wheel spheres from the expanded model and evaluates them analytically. The old box (bottom 0.050 m) overlaps the spheres (top 0.052421 m) by 2.421 mm and `acm_derive.derive` reports four permanent collisions and disables none; the pinned box (bottom 0.055 m) clears them by 2.579 mm and the four pairs are certified `ConstantTransform`. On platform-sw `d619012` the new-box tests fail, on `dfd44a5` they pass. It runs without a ROS graph or a simulator.
- **M07 and W03.** **M07 is restored exactly as Alex wrote it** (WARN on the four upstream-limit joints, `joint_rest_margin_rad`, the datums revision). **W03 does not replace it and is not a check**: W03 is static, the named poses of the SRDF against the joint limits in the URDF, and Mohammad's point stands: "A static named-pose report does not establish dynamic sample coverage." M07 is dynamic, samples of the rest pose in Gazebo with every expected joint covered. W03 is evidence files (`evidence/w03/`) only. **One agreed rest-state rule is still pending with Alex and Om, including what happens to missing joint-state evidence; it is not settled, and nothing here claims it is.**
- **`config_id` carry-over** (Satya, C9/C10): the digest is of the compact sorted-key JSON payload that `print_config_id.py` hashes, not of the raw URDF/Xacro file. To use the digest with C10, **retain that exact payload**, not merely the raw file; the same bytes are described in the description package README. Satya also confirms that the lift stops are calibration IDs, not configuration axes, which matches ADR-010 as corrected by rev19.0.

## Dependencies

Tushar's statement about platform-sw is kept under "Description at 49d2368" below. Every `package.xml` change against `main` (`f57e54c`) is listed here: **83 dependencies added** across the five packages, and none removed. The duplicate `<test_depend>moveit_msgs</test_depend>` in `openamrobot_upperbody_moveit` is declared once. Licences are as declared by the installed package (ROS) or by the Ubuntu copyright file (apt).

- OpenAMRobot packages are MIT.
- `openarm_description` is built from source at the pin, Apache-2.0.
- No pip, no new apt source, and no vendor SDK.

| Package | Tag | Dependency | Licence | Source |
|---|---|---|---|---|
| `openamrobot_upperbody_bringup` | exec_depend | `launch` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch) |
| `openamrobot_upperbody_bringup` | exec_depend | `launch_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-ros) |
| `openamrobot_upperbody_bringup` | exec_depend | `xacro` | BSD | ROS 2 Jazzy binary (ros-jazzy-xacro) |
| `openamrobot_upperbody_bringup` | exec_depend | `robot_state_publisher` | BSD | ROS 2 Jazzy binary (ros-jazzy-robot-state-publisher) |
| `openamrobot_upperbody_bringup` | exec_depend | `controller_manager` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-controller-manager) |
| `openamrobot_upperbody_bringup` | exec_depend | `joint_state_publisher_gui` | BSD | ROS 2 Jazzy binary (ros-jazzy-joint-state-publisher-gui) |
| `openamrobot_upperbody_bringup` | exec_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_bringup` | exec_depend | `sensor_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-sensor-msgs) |
| `openamrobot_upperbody_bringup` | exec_depend | `rviz2` | BSD | ROS 2 Jazzy binary (ros-jazzy-rviz2) |
| `openamrobot_upperbody_bringup` | test_depend | `launch_testing` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing) |
| `openamrobot_upperbody_bringup` | test_depend | `launch_testing_ament_cmake` | Apache-2.0, BSD | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ament-cmake) |
| `openamrobot_upperbody_bringup` | test_depend | `launch_testing_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ros) |
| `openamrobot_upperbody_bringup` | test_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_bringup` | test_depend | `control_msgs` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-control-msgs) |
| `openamrobot_upperbody_bringup` | test_depend | `controller_manager_msgs` | BSD | ROS 2 Jazzy binary (ros-jazzy-controller-manager-msgs) |
| `openamrobot_upperbody_bringup` | test_depend | `sensor_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-sensor-msgs) |
| `openamrobot_upperbody_bringup` | test_depend | `trajectory_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-trajectory-msgs) |
| `openamrobot_upperbody_bringup` | test_depend | `builtin_interfaces` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-builtin-interfaces) |
| `openamrobot_upperbody_control` | exec_depend | `joint_state_broadcaster` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-joint-state-broadcaster) |
| `openamrobot_upperbody_control` | exec_depend | `joint_trajectory_controller` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-joint-trajectory-controller) |
| `openamrobot_upperbody_control` | exec_depend | `gripper_controllers` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-gripper-controllers) |
| `openamrobot_upperbody_control` | test_depend | `ament_cmake_pytest` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ament-cmake-pytest) |
| `openamrobot_upperbody_control` | test_depend | `openamrobot_upperbody_description` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_control` | test_depend | `python3-yaml` | MIT (PyYAML licence text) | Ubuntu 24.04 apt (python3-yaml) |
| `openamrobot_upperbody_description` | exec_depend | `openamrobot_description` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_description` | exec_depend | `openarm_description` | Apache-2.0 | enactic/openarm_description @ 14ff67b (source, pinned) |
| `openamrobot_upperbody_description` | test_depend | `gz_ros2_control` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-gz-ros2-control) |
| `openamrobot_upperbody_description` | test_depend | `python3-pytest` | MIT (Expat) | Ubuntu 24.04 apt (python3-pytest) |
| `openamrobot_upperbody_description` | test_depend | `python3-yaml` | MIT (PyYAML licence text) | Ubuntu 24.04 apt (python3-yaml) |
| `openamrobot_upperbody_description` | test_depend | `python3-jsonschema` | MIT | Ubuntu 24.04 apt (python3-jsonschema) |
| `openamrobot_upperbody_description` | test_depend | `python3-numpy` | BSD-3-Clause | Ubuntu 24.04 apt (python3-numpy) |
| `openamrobot_upperbody_description` | test_depend | `liburdfdom-tools` | BSD-3-Clause | Ubuntu 24.04 apt (liburdfdom-tools) |
| `openamrobot_upperbody_gazebo` | exec_depend | `ros_gz_bridge` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ros-gz-bridge) |
| `openamrobot_upperbody_gazebo` | exec_depend | `gz_ros2_control` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-gz-ros2-control) |
| `openamrobot_upperbody_gazebo` | exec_depend | `controller_manager` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-controller-manager) |
| `openamrobot_upperbody_gazebo` | exec_depend | `robot_state_publisher` | BSD | ROS 2 Jazzy binary (ros-jazzy-robot-state-publisher) |
| `openamrobot_upperbody_gazebo` | exec_depend | `joint_state_publisher` | BSD | ROS 2 Jazzy binary (ros-jazzy-joint-state-publisher) |
| `openamrobot_upperbody_gazebo` | exec_depend | `xacro` | BSD | ROS 2 Jazzy binary (ros-jazzy-xacro) |
| `openamrobot_upperbody_gazebo` | exec_depend | `launch` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch) |
| `openamrobot_upperbody_gazebo` | exec_depend | `launch_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-ros) |
| `openamrobot_upperbody_gazebo` | exec_depend | `rviz2` | BSD | ROS 2 Jazzy binary (ros-jazzy-rviz2) |
| `openamrobot_upperbody_gazebo` | exec_depend | `openamrobot_upperbody_control` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_gazebo` | exec_depend | `openamrobot_gazebo` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_gazebo` | test_depend | `launch_testing` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing) |
| `openamrobot_upperbody_gazebo` | test_depend | `ament_cmake_pytest` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ament-cmake-pytest) |
| `openamrobot_upperbody_gazebo` | test_depend | `launch_testing_ament_cmake` | Apache-2.0, BSD | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ament-cmake) |
| `openamrobot_upperbody_gazebo` | test_depend | `launch_testing_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ros) |
| `openamrobot_upperbody_gazebo` | test_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_gazebo` | test_depend | `control_msgs` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-control-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `controller_manager_msgs` | BSD | ROS 2 Jazzy binary (ros-jazzy-controller-manager-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `sensor_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-sensor-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `geometry_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-geometry-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `nav_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-nav-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `tf2_msgs` | BSD | ROS 2 Jazzy binary (ros-jazzy-tf2-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `trajectory_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-trajectory-msgs) |
| `openamrobot_upperbody_gazebo` | test_depend | `builtin_interfaces` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-builtin-interfaces) |
| `openamrobot_upperbody_moveit` | buildtool_depend | `ament_cmake_python` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ament-cmake-python) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_configs_utils` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-configs-utils) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_kinematics` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-kinematics) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_planners_ompl` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-planners-ompl) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_simple_controller_manager` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-simple-controller-manager) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_ros_visualization` | BSD-3-Clause | ROS 2 Jazzy binary (ros-jazzy-moveit-ros-visualization) |
| `openamrobot_upperbody_moveit` | exec_depend | `moveit_py` | BSD | ROS 2 Jazzy binary (ros-jazzy-moveit-py) |
| `openamrobot_upperbody_moveit` | exec_depend | `launch` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch) |
| `openamrobot_upperbody_moveit` | exec_depend | `launch_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-ros) |
| `openamrobot_upperbody_moveit` | exec_depend | `rviz2` | BSD | ROS 2 Jazzy binary (ros-jazzy-rviz2) |
| `openamrobot_upperbody_moveit` | exec_depend | `python3-yaml` | MIT (PyYAML licence text) | Ubuntu 24.04 apt (python3-yaml) |
| `openamrobot_upperbody_moveit` | exec_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_moveit` | exec_depend | `sensor_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-sensor-msgs) |
| `openamrobot_upperbody_moveit` | exec_depend | `std_srvs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-std-srvs) |
| `openamrobot_upperbody_moveit` | exec_depend | `openamrobot_upperbody_control` | MIT | OpenAMRobot workspace package |
| `openamrobot_upperbody_moveit` | test_depend | `ament_cmake_pytest` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-ament-cmake-pytest) |
| `openamrobot_upperbody_moveit` | test_depend | `python3-pytest` | MIT (Expat) | Ubuntu 24.04 apt (python3-pytest) |
| `openamrobot_upperbody_moveit` | test_depend | `launch_testing` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing) |
| `openamrobot_upperbody_moveit` | test_depend | `launch_testing_ament_cmake` | Apache-2.0, BSD | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ament-cmake) |
| `openamrobot_upperbody_moveit` | test_depend | `launch_testing_ros` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-launch-testing-ros) |
| `openamrobot_upperbody_moveit` | test_depend | `rclpy` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-rclpy) |
| `openamrobot_upperbody_moveit` | test_depend | `moveit_msgs` | BSD | ROS 2 Jazzy binary (ros-jazzy-moveit-msgs) |
| `openamrobot_upperbody_moveit` | test_depend | `python3-numpy` | BSD-3-Clause | Ubuntu 24.04 apt (python3-numpy) |
| `openamrobot_upperbody_moveit` | test_depend | `joint_state_publisher` | BSD | ROS 2 Jazzy binary (ros-jazzy-joint-state-publisher) |
| `openamrobot_upperbody_moveit` | test_depend | `shape_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-shape-msgs) |
| `openamrobot_upperbody_moveit` | test_depend | `geometry_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-geometry-msgs) |
| `openamrobot_upperbody_moveit` | test_depend | `std_msgs` | Apache-2.0 | ROS 2 Jazzy binary (ros-jazzy-std-msgs) |

## Integration Gate

- **Overlapping work:**
  - This change sits on PR #14 (Tushar's `feat/a7-gazebo-moveit` @ `49d2368`, draft).
  - platform-sw #70 (merged, main `d619012`) supersedes #68 and #69. It brings the 2.0 sensor frames, simplified collision and the Gazebo range and IMU sensors into the base this model includes. The contract lists its frames by name only, so the new `imu_link` pose needs no change here; T07 now records the consumed sensor frames (`ff8f9ad`).
  - platform-sw #71 (`8531089`, merged on 16 Oct as `dfd44a5`, which this branch now pins) raises the chassis collision box bottom to 0.055 m, clear of the caster-wheel spheres by 2.6 mm. The results on top of Tests are on it.
  - platform-sw #53 (draft, Gemini RGB-D), and the planned base-description PR (platform-sw #49).
- **Reused:**
  - The upstream OpenArm v2.0 description and `openarm_ros2` MoveIt config at the pins, with Apache-2.0 headers kept on the derived files.
  - platform-sw's description and bridge, by reference.
  - The repository's own harness: `verify_a6.sh`, `run_a7_regression.sh`, `regenerate_acm.sh`.
- **Rejected:**
  - Widening the joint-state clamp epsilon.
  - A sampled collision matrix: the matrix stays derived, and `--write` was used only after Alex accepted it.
  - Moving the arm and gripper spawners onto `--activate-as-group`: it changes the controller order; not needed.
  - Skipping or removing any test.
- **Review conflict, for the reviewers to settle.** Mohamed's review asks to resolve M07 "without … turning an unaddressed failure into a pass/warning". Alex accepted follow-up (b), which reports the four upstream-limit joints as WARN. This branch follows Alex's decision.
  - Any other joint within the margin, a listed joint further past its limit than the margin, and incomplete coverage all remain FAIL.
  - Mohamed's coverage finding is fixed in `84d80a9`.
- **STATE.md:** the repository has no STATE.md, so there is none to update.

## Safety

No safety path is touched. No file matches the `safety_paths` globs of `openAMRobot/.github` `maintainers.yaml` (E-stop, brake, contactor, watchdog, motor enable, charge inhibit, safety).

- The lift stays outside control authority: `lift_joint` keeps effort 0 and velocity 0, and no controller drives it.
- The spawner change affects simulation start-up only.
- Simulation and mock results are not safety evidence.

## AI disclosure

- **Tushar's commits** (`f57e54c..49d2368`): developed with Claude Code (Anthropic), as Tushar states below ("AI assistance"). Each commit carries `Assisted-by: Claude Code`.
- **Follow-up commits** (`49d2368..703fce9`) and this file: produced by Claude Code at Alex Reznichenko's request, and signed off by him (DCO). That covers the verification runs, the code, test and configuration changes, the regenerated collision matrix, the `decision-allow` markers, the SPDX headers, issue #16 and this description.
- **P18 and P19 commits** (`bb39c83`, `9406af6`, the merge and the commits after it): produced by Claude Code for Tushar Goel and signed off by him; each carries `Assisted-by: Claude Code`.
- **P22** (documentation corrections, the CHANGELOG, `tools/check_decision_currency.py`, `tools/check_evidence_backing.py` and their two lists): produced by Claude Code for Tushar Goel and signed off by him; each commit carries `Assisted-by: Claude Code`.
- No upstream file is modified.

## Follow-up commits on this branch

| Commit | What it does |
|---|---|
| `b45d8f6` | (a) 240 x 240 mm placeholder boxes, visual and collision, on both lift-column stages (opt-in `column_geometry`), the frame contract line, and the re-derived collision matrix: 95 disabled pairs, was 83; 535 of 630 enabled. (b) M07 WARN for the four upstream-limit joints. |
| `1637743` | `joint_state_broadcaster` spawner: `--switch-timeout 30`, `--service-call-timeout 45`. The arm and gripper spawners start only after it exits 0. |
| `84d80a9` | The controller wait honours one absolute deadline. M07 requires coverage of every expected joint (Mohamed's two findings). |
| `20886a6` | Frame contract: `imu_link` and `base_camera_link` consumed from platform-sw, names only. |
| `405d162` | `decision-allow` markers on the 48 historical or negative-test register lines; the stale `mast_id` launch argument in the clamp-loop test is removed. |
| `ed8c56d` | Duplicate `moveit_msgs` removed; the frame reference points at `frame_contract.yaml`; SPDX MIT headers on 116 source files without a header. |
| `8e1aba1` | Every skip names #16. |
| `ff8f9ad` | T07 records platform-sw's consumed sensor frames (#70) instead of failing them. |
| `703fce9` | M01 waits for the controllers before its first pose (it raced the mock controllers' start-up). |

## Description at 49d2368 (Tushar Goel, unchanged)

Where this text and the sections above disagree, the sections above describe the branch head. Specifically:
- M07 is now WARN, not FAIL;
- the column now has placeholder geometry;
- the collision matrix has 95 disabled pairs;
- a gz server the stop helper had to force is a WARN (W04), not a FAIL; a server still running after that is a FAIL, as before;
- A7 is 13 PASS / 2 WARN / 0 FAIL;
- the "cause unknown" sentence on the right finger under "Known gaps" is restated, with what was measured, under "Findings that are not resolved"; and this step's results (the merge head, platform-sw pinned) are under "Tests".

### Why A6 and A7 are one PR

The change from the fixed mast to the lift column straddles the description and the simulation/MoveIt packages: the description gives up the mast, the control, Gazebo and MoveIt packages follow the lift joint, and the numbers each of them records (collision matrix, planning poses, configuration IDs) change together. On current `main` the description is only canonical if it carries the lift, and the lift work lives in the same commits as the Gazebo and MoveIt work, so the two cannot be split by commit; a cherry-pick of the A6 part on its own does not build green against `main`. A file-level split (description files in one PR, everything else in a second) is possible on request, at the cost of a first PR whose tests need the second. <!-- decision-allow: LIFT historical: the change away from the superseded fixed mast -->

`feat/a6-canonical-description` @ `44d28ea` stays on the fork as the historical A6 delivery (the mast model); this PR supersedes it.

### What is in it

Five packages under `ros2/src/`, plus `docs/`, `evidence/` and the changelog:

- `openamrobot_upperbody_description` — the canonical combined description: `combined_robot.xacro` composes the platform base (unchanged, `base_footprint` root), option B's upper body on its lift column and the upstream OpenArm macros; datums with provenance, frame contract, per-expansion evidence checks (T01-T16, C01/C02, E01, W01/W02, V01, H01/H02) and the harness that runs and reads them.
- `openamrobot_upperbody_control` — `ros2_control` for the two OpenArm v2.0 arms, one file for the mock, gazebo and real profiles (joint state broadcaster, trajectory and gripper controllers); the lift is not in it, we have no authority to command it.
- `openamrobot_upperbody_gazebo` — Gazebo Harmonic simulation of the combined model: the base on its DiffDrive plugin, both arms on `gz_ros2_control`, `odom -> base_footprint`.
- `openamrobot_upperbody_moveit` — MoveIt 2 configuration: arm planning groups with the lift as a planned joint, a collision matrix derived (83 disabled pairs, 478 enabled) rather than sampled, named poses, regression checks M01-M07, `joint_state_clamp.py`.
- `openamrobot_upperbody_bringup` — launch files composing base, lift column and arms in simulation.

### Evidence recorded at 013b524 (Tushar)

Class: `static+mock` for the A6 suite, `simulation` for A7. **No physical evidence.** Every check reports one of five values:

| value | meaning |
|---|---|
| `PASS` | the check ran and its assertion held |
| `WARN` | the check ran and recorded a finding; it is not a pass and every WARN has a check ID |
| `FAIL` | an assertion was violated, or a process guard saw a process fail to start, die or exit badly |
| `SKIP` | a stated, reasoned non-comparison (the one legitimate case: no git checkout to compare an upstream pin with) |
| `NOT_RUN` | a registered check left no result; never a `PASS`, and it fails the exit status |

- **A7 regression** (`run_a7_regression.sh`), on the code of this PR (`013b524`; the pushed head adds report copies, measurement data and documents, and one unit-test file): **no run is green, by design: M07 FAILs** (below). Run 2 and run 3: 13 PASS / 1 WARN (V01) / 2 FAIL (M07, and AUX with it, because the bringup `colcon test` step exits 1) / 0 NOT_RUN [P22: the two reports, `20261015T121048Z_a7.json` and `20261015T121946Z_a7.json`, have 12 PASS; 15 checks]. Run 1 additionally failed **G01**: its `gz sim` server ignored SIGINT, the stop request and SIGTERM and was ended by the stop helper's SIGKILL at 9.2 s; nothing was left running, and the run says so (11 PASS / 1 WARN / 3 FAIL). Earlier, on `7daa2ab` (the shutdown-race fix, before the rebase and before option B: these prove that fix) five runs of 13 PASS / 1 WARN / 0 FAIL, plus three on `dada509` and three on `aa94fc3`.
- **One A7 run in six failed on this code earlier** (P16, `9bc327c`: 11 PASS / 1 WARN / 2 FAIL, M02 `CONTROL_FAILED`) under a simulator orphaned by G01's launch that nothing detected. Its cause was identified, it is now detected (and, since P17, stopped by the launch), and the report is kept (`evidence/reports/README.md`, P16). The number to quote is "one failure in six on that commit, cause identified", not "three for three"; and P17's run 1 above is a second, different one.
- **M07 currently FAILs by design**: no joint's rest position may be within `joint_rest_margin_rad` (a datum, 1e-3 rad, `placeholder`) of either limit. It FAILs on `openarm_right_finger_joint1` (rests at +2.6e-8 rad, past its open limit) and on `openarm_left_finger_joint1`, `openarm_left_joint4` and `openarm_right_joint4`. The zero and open poses are limits by construction, in the old body too (`evidence/m02_bisect/RESULTS.md`, last section), so the check is red until the margin, the rest pose, or the claim it makes is decided; that is a question for the reviewers, not something the PR hides.
- **A6 suite** (`verify_a6.sh`): **24 PASS / 4 WARN (T08, E01, W02, V01) / 0 FAIL / 0 NOT_RUN**, three runs on `013b524` (and on `1168f45` before). The first pass of this step, on `a713533`, failed 3 of 3 (AUX: `test_evidence_dir.py`, the stop helper's report was in the system temp directory); the reports are kept and the fix is `013b524`. `config_id`s changed again (the datums revision, for the new `joint_rest_margin_rad`).
- The reports are in `evidence/reports/`, with `README.md` listing commit and result for each, including the failed ones. `evidence/frozen_a6_993b000/` is the frozen expansion set of the earlier mast-model A6, with a verifier.
- **Orphaned simulators.** Two of the eight A7 runs of P15 left a `gz sim` server running after the launch under test was torn down, and one more did after G01 in P16 (the natural rate was about 1 in 13 launches). G01, G02, M02 and M07 now fail when a simulator of their own partition survives their launch, and `sim.launch.py` stops the server through a detached helper (SIGINT, then the service request, then SIGTERM and SIGKILL); a server that has to be sent SIGTERM or SIGKILL is itself a FAIL of the run that needed it. After that fix: **0 orphans seen by the detector in 12 runs each of G01, G02 and M02** (natural rate before: about 1 in 13). That is not a claim that it is fixed: 12 clean runs of a one-in-13 event are not unlikely, and the one live force-stop above shows the hang still occurs (one in 49 launches since, 36 of them in that count). The cause of the hang is inside gz-sim and was not found. The detector was proven through the launch-test path with an injected fault (`canary_gz_orphan.sh`) and on decoy processes.

### Dependencies on platform-sw (Tushar)

- **platform-sw `935efc7` (#55)**, on platform-sw `main` (`174731c` at the time of writing): `base_footprint` root, `base_link` 0.098 m above it, and the near-field sensor frames `us_*` (4 x MB7060) and `tof_*` (2 x TF-Luna) declared. This description requires it (its root and the 0.098 m are asserted against platform-sw's `base_footprint_joint`).
- The six near-field frames that earlier versions of our contract declared (`tof_front_left_link` ... `ultrasonic_rear_link`) are removed from our contract and are `consumed_from` platform-sw, not declared by us. T04 now reports which of them each expansion contains and passes **by deletion**: nothing in our contract can be absent any more.
- No upstream (`openarm_description`) file is modified; the pin is unchanged.

### Open questions for reviewers

- [ ] **Alex and Cuma: the two opt-in parameters on `openamrobot_upperbody.xacro`.** `emit_arm_mounts` (default `true`) and `column_inertial` (default `''`, none). Both default to current `main` behaviour. They exist because the upstream arm macro emits a link and a joint with the same names as option B's mount frames, so the combined robot has to turn those off, and because the combined robot supplies the lift's sourced catalogue mass. Two separate statements, kept apart because the first one is weaker than it sounds. (a) A test (`test_with_the_opt_in_parameters_at_their_defaults_the_macro_expands_as_it_does_on_main`) asserts that the mount frames and joints exist and that no inertial appears anywhere with the defaults; it does **not** compare against `main`. (b) Separately, compared byte for byte against `upstream/main` (`tools/compare_option_b_wrapper.sh`, run at `HEAD`), the standalone wrapper with the defaults expands identically apart from the empty `lift_column_base_link`, which is serialised as `<link name="lift_column_base_link"></link>` (an open/close pair) instead of a self-closing tag, and the path xacro writes in its header comment. That script is not part of the test suite. Needs acceptance, or a different composition.
- [ ] **Supplier drawing for the lift column.** The column has **no visual and no collision geometry**: it exists in the model as an inertia tensor only, so every render committed under `evidence/renders/` shows the arms with no column between them, and the arms can plan through the column body. It also carries one invented lateral dimension: `lift_column_section` y is set equal to the sourced 240 mm x side. Both are marked in `datums.yaml` (`placeholder`/`provisional`); nothing downstream treats them as measured.
- [ ] **V01's interference count was taken on collision geometry only.** The arms' visual meshes were not intersected separately, and the column has neither kind of geometry, so a zero there says nothing about the column.

### Known gaps

- `lift_mass_tl3_400` is `null` (no supplier mass for the fallback TiMOTION TL3 400 column). By design a null datum that is read is a hard-fail that names itself: the `tl3_400` configuration does not expand.
- V01 (manual visual check) is `WARN`. Its eight items were measured by script on the lift model (none reads outside its expected value; two sub-questions that need eyes are CANNOT-MEASURE; `evidence/v01/`, renders in `evidence/renders/`), and it stays `WARN` until `lift_mass_tl3_400` is sourced, however the eight land. No person has looked at the running simulator.
- M02 and the clamp: **the activation count is retired as a criterion.** It scatters 646 to 1054 inside one unchanged configuration (and 1795 / 957 / 1832 on three runs of a tests-only diff), so a rise from about 200 to about 1800 could not be read as a signal. M02 now gates on two numbers: readings beyond the epsilon (must be 0) and the largest overshoot magnitude (must be below the epsilon, 1e-5 rad); the counts are reported as `diagnostic_only`. In the three runs above: 0 excursions and a largest overshoot of 1.5e-6 rad (margin about 6.5x; P16's worst variant run read 4.2e-6, about 2.4x). The epsilon has not been widened. M02 also waits for the five controllers to be `active` before its first pose and FAILs, naming their states, if they never are. The right finger rests on its open limit (see M07); why it sits within 1e-5 rad of it in the new body and 2e-4 to 5e-4 inside in the left one was measured (finger effort, both trees, `evidence/m02_bisect/RESULTS.md`) and **not explained: cause unknown**. The M02 bisect is closed.
- No runtime producer of the configuration identity has an owner: `config_id` is computed offline from files; nothing in a running system publishes it.
- Arms are 62 mm apart at their bases: no contact at home, but the left gripper reaches right-arm links in about 3% of random left-arm states (sampled).
- A `gz sim` server that is asked to stop sometimes never begins its teardown and survives SIGINT, the stop request and SIGTERM (about 1 launch in 13 before the helper; 1 in 49 after). The launch now ends it by SIGKILL and the run FAILs naming it; the underlying hang is in gz-sim and is not fixed here.

### `config_id` inputs changed

`config_id` kept its format and changed its inputs: the canonical URDF, `column`, `base_plate`, `head_pitch`, `base_tilt`, `arm_package`, `hardware_profile`, datums revision and every upstream pin (before: `mast_id` and no column or base-plate inputs). It still validates against the C9 schema, so nothing fails to say that **an id recorded before this change is not comparable with one computed now**. The frozen A6 set keeps the old inputs and says so. Satya (C9 owner): please confirm that consumers do not compare ids across this change. <!-- decision-allow: CONFIG-IDS historical: the config_id inputs before this change -->

### AI assistance (Tushar)

This PR was developed with Claude Code (Anthropic) against a written design (architecture documents and prompts). The work was run and tested by the repository's evidence harness (`verify_a6.sh`, `run_a7_regression.sh`; the reports are in `evidence/reports/`, produced on the author's workstation, not by a hosted CI service) and reviewed by Tushar Goel. Prompts and design documents are available on request.
Per the OpenAMRobot AI Contribution Policy.

### Checklist

- [ ] DCO sign-off on every commit
- [ ] Contributor agreement accepted
- [ ] No upstream files modified
- [ ] No frame names outside ros2/src/openamrobot_upperbody_description/config/frame_contract.yaml
