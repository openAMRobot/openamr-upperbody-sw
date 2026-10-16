# Changelog

All notable changes to this repository. Evidence for the description is static and mock only, and the Gazebo evidence is class
`simulation`; nothing here is a hardware result.

## [Unreleased]

### Added / Changed / Fixed (P17)

- **The simulator is stopped once per launch, by a detached helper, and a server that has to be forced is a FAIL.** `sim.launch.py`'s `OnShutdown`
  fired twice on every shutdown (the shell's death is a second `Shutdown`) and re-executed the same `ExecuteProcess`: "executed more than once",
  which abandoned the rest of the shutdown; on the `ros2 launch` + SIGINT path the stop request was not started at all (launch refuses to start an
  action once `context.is_shutdown` is set). Measured before changing anything (`launch/sim.launch.py` header has the numbers): launch signals the
  `/bin/sh` around `gz sim`, never the server; the `/server_control` request went wrong 4 times in 18 launches (two server segfaults inside
  libgz-sim8 during teardown, two servers that accepted the request, never began teardown and ignored SIGTERM, one of which was orphaned by
  launch's SIGTERM of the shell at 5 s). SIGINT sent to the server itself, as Ctrl-C does, went wrong 0 times in 24 launches. The helper now does:
  SIGINT to the `gz sim` processes; the service request as well after 3 s if one is still there; SIGTERM then SIGKILL after 8 s; servers followed by
  pid (a first version walked the process tree and called a reparented server "gone"; found by the driver, not by a test). Its report is written
  before it sends anything harder than SIGINT and `launch_guard.gz_server_shutdown_report` reads it: a forced server is a survivor even though it
  is gone. **The ORPH detector is unchanged for a server that is still running and no allowance was added.** Not found: why a server hangs in its
  teardown after being asked to stop (inside gz-sim / gz_ros2_control). The force-stop path is proven on decoy processes only; it was never
  exercised live (36 of 36 runs stopped on the first step).
- **M07, the rest-position check** (`test_m07_rest_position_margin.py`, A7): no joint's rest position may be within `joint_rest_margin_rad` of
  either limit; FAIL, not WARN. The margin is a datum (`datums.yaml`, `placeholder`, 1e-3 rad, report-only; revision bumped, so every
  `config_id` changes), taken from the between-run scatter of a rest position (up to 2.8e-4 rad) and not tuned to a verdict. **It FAILs today** on
  `openarm_right_finger_joint1` (rests at +2.6e-8 rad, past its open limit), and on `openarm_left_finger_joint1` and both `joint4`; `lift_joint`
  is listed as not measured, with the reason.
- **M02 gates on exactly two numbers**: readings beyond epsilon (0) and the largest overshoot magnitude (below 1e-5). Activation and episode
  counts are reported under `diagnostic_only`, labelled, and gate nothing. The epsilon is not widened. **M02 waits for the five controllers to be
  `active`** (120 s, FAIL naming their states) before it sends the first pose.
- **G02's own helper no longer fails on two JSON messages** (`gz topic -n 1` printed two; `json.loads` raised "Extra data": 1 of 12 live G02 runs).
- **Finger effort at rest was read in both trees** (`evidence/m02_bisect/RESULTS.md`, last section): the sign did not change between the bodies,
  which refutes P16's hypothesis; the old body also rests within 2.5e-5 rad of the same four limits; why the new right finger sits at the limit
  and the left one does not is not answered.
- **Evidence pass on `013b524`** (`evidence/reports/README.md`, P17): A6 24 PASS / 4 WARN / 0 FAIL, three runs (the first pass, on `a713533`, failed
  3 of 3: the helper's report was in the system temp directory, which `test_evidence_dir.py` forbids; fixed). A7: M07 FAILs in all three runs by
  design (and AUX with it); run 1 also failed G01 because its server ignored SIGINT, the service request and SIGTERM and was SIGKILLed by the
  helper at 9.2 s: the force-stop path, seen live once in 49 launches, leaving nothing running. **0 orphans in 12 runs each of G01, G02 and M02
  after the fix (natural rate before: about 1 in 13)** is a count, not a claim that the hang is gone. `test_a7_controller_wait.py` pins M02's
  controller wait and its FAIL, against a fake controller manager.
- `tools/compare_option_b_wrapper.sh`: the byte-level comparison of option B's wrapper against `upstream/main` that the PR body cites.

### Added / Changed (P16)

- **G02, G01 and M02 fail when a `gz sim` of their own partition outlives their launch.** (G02 first; G01 and M02 after a natural orphan of G01's
  launch degraded the rest of an A7 run in the second evidence pass.) The ruby wrapper is what the launch owns and what the process guard reads;
  the server it forks was left running after about 2 of 8 A7 runs and nothing noticed. G02's post-shutdown phase now looks (within a 10 s grace
  period) for any `gz sim` whose environment carries the test's own `GZ_PARTITION`; a survivor is a FAIL (not a WARN), judged before it is
  signalled and only then stopped, and written into G02's sidecar (`details.gz_server_after_shutdown`). Chosen over a new G03 because the orphan
  is a property of G02's own teardown and a G03 would need a second simulator launch of its own to observe anything. Proven by a decoy-process
  unit test, a canary (`test/canary_gz_orphan.sh`: the real G02 against a launch without the `/server_control` stop handler, which must fail naming
  the survivor) and one natural orphan found in the first run of the session, before the check existed. **Not proven on a natural orphan with the
  check in place: 0 of 12 live runs produced one** (`evidence/reports/README.md`, P16). M02's launch leaves a simulator behind too after a failed
  run (the check now covers it).
- **The eight V01 items of the lift model are measured** (`docs/v01_visual_check.md`, `evidence/v01/`), none reads outside its expected value, two
  sub-questions that need eyes are CANNOT-MEASURE, and V01 stays WARN because `lift_mass_tl3_400` is null. Renders per lift stop and an isometric
  are in `evidence/renders/`.
- **M02's clamp rise was bisected and the cause was not found** (`evidence/m02_bisect/RESULTS.md`): the ACM, the mass, the arm separation and the
  open command are each ruled out, and so is platform-sw's new base (old body on the new base reads 188 and 216); the rise is
  `right_finger_joint1` resting on its open limit in the new body. The epsilon is untouched.

### Fixed (P15, task 0)

- **`joint_state_clamp.py` exits 0 on SIGINT and SIGTERM.** P14 ran A7 twice on an unchanged commit and got 2 FAIL, then 0 FAIL; the cause was
  this node, ours, exiting 1 at shutdown. Reproduced standalone against a flooded `/joint_states` before fixing: every SIGTERM exited 1 (30 of
  30, an uncaught `ExternalShutdownException`, which rclpy raises out of `spin` for every shutdown) and 1 of 30 SIGINTs exited 1 with the
  `take_message` `RuntimeError` the signal provokes when it lands between the wait set and the take. `run()` now treats `KeyboardInterrupt`,
  `ExternalShutdownException` and that `RuntimeError` **with a closed context** as a clean stop; the same error with a live context, and
  anything else, still raises. The node is destroyed before `rclpy.try_shutdown()`. Not allow-listed: `launch_guard.py` is unchanged.
  `test_joint_state_clamp_shutdown.py` sends real signals to a real node under a message flood (against the old script it fails for
  both signals: SIGTERM on the first attempt, SIGINT on the third) and unit-tests the decision with an injected `spin`.

### Changed (P15, rebase onto main and adopt option B)

- **Both branches are rebased onto `upstream/main`**, where Alex's option B (two commits, authorship intact) and the rev19.0 column position are
  merged. Nobody's commits were re-authored. Main won for everything describing the upper body or the baseline (README wording, package
  descriptions, his config, xacro, launch file and tests, all byte-identical to main afterwards); ours won for the arm composition, control
  profiles, Gazebo, MoveIt and the harness; manifests and CMake took both.
- **The mast is deleted and the arms are rooted on option B's mount frames.** `urdf/mast.xacro`, `config/mast_ids.yaml` and the datums
  `mast_base_pose`, `left_arm_mount_pose`, `right_arm_mount_pose`, `mast_section`, `mast_mass` and `head_camera_pose` are gone; `mast_id` is not
  an argument. `combined_robot.xacro` composes option B's macro on platform-sw's `base_footprint` and calls the upstream arm macros at his
  `openarm_{left,right}_base_link` pose (`geometry.arm_mount`: x +0.180, y +-0.031, z +0.166 from the carriage). ADR-008 is otherwise unchanged:
  two calls, derived single-arm presets, no OpenArm body. Two things had to give, both recorded in the package README: option B's macro
  gains two opt-in parameters (`emit_arm_mounts`, `column_inertial`, defaults = its own behaviour) because the upstream arm macro emits a link
  and a joint of the same names as his mount frames; and the arms are placed by the macro `openarm_v20_robot` itself calls, because that macro
  declares its `root_frame` as a link and `lift_carriage_link` is already his.
- **rev19.0 numbers**: two base-plate positions (`bp000`, `bp050`; ADR-010's five were rev18.7); the column axis at x +0.060 / +0.110 from
  `base_link`, not on the drive axle (`column_axis_x_bp000`); `base_link_height_above_floor` 0.098 (a placeholder since S3, now
  provisional with a source), asserted against platform-sw's `base_footprint_joint`; the lift is a DOLD Hexalift V4 350 of **11.7 kg**
  catalogue (`lift_mass_dold_v4_350`, provisional) on `lift_column_base_link`, replacing `mast_mass` (3.0 kg, placeholder).
- **Three things option B does better, adopted**: `effort="0" velocity="0"` on a joint we have no authority to command (kept, and
  asserted); a null datum is a hard-fail that names itself (`lift_mass_tl3_400`, as its `tl3_400` lengths; the schema allows `null`
  only with `source` and `status` `placeholder`); and **calibration IDs are not configuration IDs** (the lift stops `L1000`, `L1175`,
  `L1350` are joint positions; T05 and E01 are per stop; they are reported beside the config ids, not inside them).
- **`config_id` changed meaning and kept its format.** Before: canonical URDF, `mast_id`, `arm_package`, `hardware_profile`, datums
  revision, `openarm_description` commit. Now: canonical URDF, `column`, `base_plate`, `head_pitch`, `base_tilt`, `arm_package`,
  `hardware_profile`, datums revision, every upstream pin. It still validates against C9, so nothing fails to say that an id recorded
  before this is not comparable with one computed now. The frozen A6 set (`evidence/frozen_a6_993b000/`) keeps the old inputs.
- **The near-field frames leave our contract.** The six declared-but-absent frames (`tof_front_left_link` ... `ultrasonic_rear_link`, `req: W`,
  T04's WARN since S4) are removed, not renamed. platform-sw declares `us_front_left`, `us_front_right`, `us_rear_left`, `us_rear_right`
  (4 x MB7060) and `tof_front`, `tof_rear` (2 x TF-Luna) in 935efc7, merged on its main; the contract says they are `consumed_from` there and
  T04 reports which of them each expansion contains, without a WARN or a FAIL. T04 is PASS by deletion.
- **platform-sw 935efc7 (REP 105) is followed**: the root is `base_footprint`, `base_link` is 0.098 m above it; the planar joint's child, the
  SRDF virtual joint and G01's odom frame (`odom -> base_footprint`, read from the DiffDrive plugin) follow. W01 is PASS: the root carries no
  inertial, so KDL sees `base_link`'s 4.18 kg (was a WARN since S3).
- **The two rules V01 found are implemented**: a dimension list declares `axis_convention` and T12 asserts the assignment against an
  expansion (`lift_column_section`: x fore-aft, y lateral); a mass of ours declares a `density_band` and T10 reports its implied density,
  scoped to our own masses (upstream arm `link2` implies 297 kg/m3 over its bounding box, below the old mast's 337, so a global band cannot
  work). The one placeholder of the lift, the column's lateral side, is the one invented number, set equal to the sourced 240 mm side.
- **The collision matrix is re-derived: 83 pairs (20 upstream + 18 `Adjacent` + 45 `ConstantTransform`), was 113 (20 + 16 + 77).** The
  by-construction set shrinks because a prismatic joint now lies between the base and the arms; 478 pairs are enabled (448). M03 holds
  `lift_joint` at each stop and samples the rest: PASS at all three. M04's fixture is found again against `arm_profile_link`; the base body is
  still out of the arms' reach (0 of 6000). M01 runs at three lift stops (was four mast IDs), M02 at the baseline with the lift at `L1000`.
- **Moved numbers, each with its reason**: A6 22 PASS / 6 WARN becomes 24 / 4 (T04 by deletion, W01 by the root); modelled mass 21.96 kg
  becomes 30.66 kg (the lift's 11.7 kg replaces the mast's 3.0 kg); E01 heights 1.5 m becomes 1.223 / 1.398 / 1.573 m per stop, depth
  0.686 m becomes 0.780 m (platform-sw's `us_*`/`tof_*` frames sit on the chassis edge) and width 0.505 m becomes 0.482 m (the wrist cameras
  are 62 mm apart at their bases now, so the wheels set the width); datums 15 (8 placeholder) become 13 (4 placeholder, 8 provisional).
  `docs/verification.md` and the package READMEs carry the rest.
- **V01 stays WARN**, with every checklist item stating what to measure. P14's look at the mast is kept as history: V01-F1 was confirmed
  correct by the design owner and V01-F2 is closed by the 11.7 kg catalogue figure; none of the eight items of the lift model has an answer.
- `base_joint_states_stub.py` reports `lift_joint` at `lift_position` on mock (robot_state_publisher publishes nothing above a joint nobody
  reports); on Gazebo `joint_state_publisher` reports it at zero. H01 learns `ament_add_ros_isolated_pytest_test`, which option B uses.

### Changed (P14, publish and record)

- **V01 records what the first manual look found, and stays WARN.** A person looked at the mast (item 1) and found two defects no automated
  check was examining: the mast profile is probably rotated 90 degrees (`mast_section` `[0.060, 0.120]` is 60 mm fore-aft; option B orients the
  same HFS6-60120 cross-section 120 mm fore-aft; the value was right and its axis assignment was only a prose note no test asserted), and
  `mast_mass = 3.0` kg is implausible for 1.2365 m of that profile (about 337 kg/m3 over the bounding box, low by roughly 2.5 to 4 times; an
  estimate, the datasheet is the source). The dimensions themselves are a real HFS6-60120 profile 14 mm from Alex's 1250 mm. The reports of
  all three profiles carry the findings (`details.findings`, `answered_items`, `unanswered_items`) and a limitation line; V01 is not PASS (items
  2 to 8 have no recorded answer) and is not recorded as having found nothing. Neither defect is fixed: both go with `mast.xacro` when option B
  is adopted. Two rules are proposed to the design owners and implemented nowhere: a dimension-pair or -triple datum declares its axis
  convention with a test asserting it, and a mass is checked for plausible implied density, placeholders included. `docs/v01_visual_check.md`
  has the status, and item 1 no longer lists "a thin spike" as a wrong thing to see (a 60 x 120 x 1236.5 mm section reads as one from the side).
- **The A6 expansion set is published** under `evidence/frozen_a6_993b000/`: the 14 expansions of the contract matrix at `993b000`, each with
  its configuration id (plain text, 487 KB), an index with the commit, datums revision and pins, and `verify_config_ids.py`, which recomputes
  every id from the files with the standard library only. It is the **fixed-mast** configuration (P-03 rev18.2) that option B supersedes, and
  says so. `mast_1350/openarm/gazebo` is the only id that differs between the A6 and A7 heads (A7 adds the `gz_ros2_control` model plugin); its
  A7 expansion is kept apart in `a7_variant/`. No configuration id was changed by this step.

### Changed (P12A, the harness and the matrix)

- **The evidence harness can no longer lie by omission.** Failures planted in each of the five packages (`scripts/harness_canary.py`) showed that the control package's tests were run and never read
  (exit 3, 0 FAIL, ten failing canaries), that a result under a check ID no report lists was dropped, that `colcon test` exits 0 whatever its tests did, that a sidecar with no test behind it was a PASS, and that a process
  of a launch test that dies of SIGSEGV or cannot start leaves the test passing. `NOT_RUN` is a new result value (never PASS, fails the exit status); every registered test must leave a result file (H02); every launch
  test checks that its processes started, that none died while the tests ran, and how they exited (H01, `launch_guard.py`). The A6 and A7 numbers survived (22/6/0 and 13/1/0 with the two new checks); the guard found
  a `parameter_bridge` abort, a `joint_state_publisher` exit 1 and a left-over gz server at shutdown, and the A7-gated M01/M02 launch tests failed two of three runs in a sample of three (they raced their own empty launch) while the A6 report dropped those failures.
- **The collision matrix disables only what can be justified, and is derived, not sampled.** 113 pairs: 20 declared by upstream, 36 `Adjacent`, 77 `ConstantTransform`; 448 enabled. The 458 sampled `Never` pairs are
  gone (upstream declares none; every one was written by our own Setup Assistant run). M03 recomputes the reasons and has no WARN class; the sampled part is information about the enabled pairs. Regenerating gives the
  contract back every time. The named-pose plans take about 5 to 9 per cent longer offline and all still plan and execute.
- The top-level `warnings` list of the A7 report is gone (every WARN has a check ID); a reduced M01 is a WARN. No reason code the library never returns is quoted anywhere.

### Changed (P11, A7 close-out)

- `openamrobot_upperbody_moveit`: M03 splits the collision matrix by who owns the claim. A pair whose links are both upstream's is
  *inherited* and a collision there is a WARN (pair, seed, sample index); every other pair is *ours* and a collision is a FAIL. The split is
  derived from the frame contract's owner column and the `openarm_` prefix, and the kinematic tree: our pairs with no joint between their
  links (77) are certified by construction with one evaluation, the rest (267) are sampled, and the sampled result is worded "no counterexample
  in N samples at seed S", never "validated". The P10 mutation test never worked (it relabelled a pair that does not collide); the new
  mutations use pairs measured to collide.
- The collision matrix is a checked-in contract with a recorded fingerprint; `regenerate_acm.sh` proposes and prints the diff, `--write`
  replaces it. `collisions_updater` has no seed, so regenerating is not a verification step.
- `joint_state_clamp.py`: `move_group` plans from `/joint_states_clamped`, the simulated joint state with float noise past a limit (at most
  1e-5 rad; measured up to 9.9e-7) clamped to the limit, logged and counted. Nothing else reads it, no limit was widened and no pose moved. The
  start-state retry loop of M01/M02 is removed (it never fired, and it was keyed on a code MoveIt does not return here); M01/M02 and the report
  carry the clamp's counters.
- M04's wording follows VERIFICATION.md (nearest reachable volume), M05 fails the single-number version in both directions, and M06's
  conclusion states both halves (the stop condition is retired; the limits remain placeholders).

### Added

- `openamrobot_upperbody_description`: the canonical OpenAMRobot 2.0 robot description (A6). One xacro entry point,
  `combined_robot.xacro`, composes the platform base (unchanged), a fixed mast selected by `mast_id` (`mast_1300`,
  `mast_1350` release, `mast_1400`, `mast_1450`) and two upstream OpenArm v2.0 arms rooted on mast plate links, with
  `arm_package` (`openarm`, `so101` stub, `none`), `hardware_profile` (`mock`, `gazebo`, `real`: selects the arm `ros2_control`
  plugin, see below) and `enable_planar_base`
  (a `planar` joint `world -> base_link`, present and disabled by default, axis `0 0 1`).
- Head camera and wrist camera frames as plain frames with a visual marker (the ZED wrapper is deferred).
- `config/datums.yaml` (15 datums with provenance: 8 placeholder, 6 provisional, 1 verified), its JSON schema, and
  `config/frame_contract.yaml`, the machine-readable frame and joint contract that the tests enforce.
- Verification suite and `verify_a6.sh`, which writes an `openamr.evidence.v1` report: expansion matrix, `check_urdf`,
  single root, frame contract, shoulder height per mast ID, argument validation, collision and inertia checks, mass report,
  planar joint, datum rules, configuration ids (`sha256:` plus 64 hex characters, matching the C9 dataset schema),
  installed-versus-source check, and warnings for the envelope (E01), KDL root inertia (W01), the planar joint under KDL
  (W02) and the missing visual inspection (V01). T14-T16 and L01 cover `ros2_control` (next entries).
- `view.launch.py` (in `openamrobot_upperbody_bringup`): robot_state_publisher, joint_state_publisher_gui and RViz2 for any
  mast ID. It started in the description package and moved to bringup with the mock bringup, with no forwarder.
- Arm `ros2_control` profiles (P06): `urdf/ros2_control.xacro`, `openamr_arm_ros2_control(hardware_profile)`, for
  `arm_package:=openarm` only. `mock` (`mock_components/GenericSystem`) and `real` (`openarm_hardware/OpenArmHW`, CAN-FD) call
  the upstream bimanual macro as upstream's own top-level v2.0 file does; `gazebo` is our own block with
  `gz_ros2_control/GazeboSimSystem` for the same joints and interfaces. The mobile base has no `ros2_control` block in any
  profile. Checks T14, T15, T16 (expansion only; nothing loads the Gazebo or real plugin).
- `openamrobot_upperbody_control/config/arm_controllers.yaml`: controller manager at 100 Hz, `joint_state_broadcaster`, a
  `JointTrajectoryController` per arm and a gripper controller per arm, one file for all three profiles. Derived from
  `enactic/openarm_ros2` at `4e837e1` (Apache-2.0); the header lists the changes.
- `openamrobot_upperbody_bringup/launch/mock.launch.py`: robot_state_publisher, `ros2_control_node` and the five spawners on
  mock hardware (args `mast_id`, `rviz`). Check L01 (`launch_testing`): all five controllers `active` within 30 s and one
  trajectory on `left_arm_controller` followed within 0.02 rad. Mock hardware reflects commands as state, so this shows the
  wiring, not tracking performance.
- `verify_a6.sh` also runs the control and bringup tests, and the evidence report reads the bringup results for L01.
- A6 evidence has no SKIP: T14-T16 and L01 are results, under `mock_components/GenericSystem` (19 PASS, 6 WARN, 0 SKIP,
  0 FAIL at `2083c76`). That is weaker than hardware, and `evidence_class` stays `static+mock`: no controller has driven a
  motor.
- `config/upstream_pins.yaml`: the config id falls back to this pinned `openarm_description` commit when there is no git
  checkout (a checkout with local changes is still an error); a test keeps it equal to the checkout and the baseline pin.
- T02 also runs `check_urdf` on `gazebo` and `real` at `mast_1300` (labelled extras).
- `.gitattributes`: `*.sh text eol=lf`, so `verify_a6.sh` cannot pick up CRLF through the `/mnt/c` checkout.
- T10 reports, after the modelled total, the `gross_mass_limit` it compared against and the F2S vehicle estimate (93 kg), each
  with its source, and prints "per-link masses still placeholders" if and only if a reported group draws a mass from a datum
  whose status is `placeholder`. The clause is computed from `datums.yaml` on every run, so it disappears when real masses
  land. T10 is still report-only: it fails only when the modelled total exceeds `gross_mass_limit`.
- `f2s_vehicle_mass_estimate` datum (`report_only`, source `f2s`, status `provisional`): Alex's F2S design proposal of 28 Sep,
  about 93 kg gross against the 120 kg cap. Second-hand: the F2S document itself is not in this repository.
- Check C02: `config/upstream_pins.yaml` must equal the HEAD of a clean `src/upstream/openarm_description`. A difference is a
  FAIL naming both hashes. With no checkout it is a SKIP that says so, the only one this tree is allowed to have.
- `docs/verification.md`: how to run `verify_a6.sh`, the evidence report field by field, what `static+mock` permits, and the
  six WARNs with what would clear each.
- READMEs for the description package and, stating their scaffold status, for the control, bringup, Gazebo and MoveIt
  packages.
- Test dependencies `python3-numpy` and `liburdfdom-tools` declared in the description package, so
  `rosdep install --from-paths src` installs what T02 and T05 use.

- `openamrobot_upperbody_gazebo` (A7, part 1): `sim.launch.py` (args `mast_id`, `world`, `headless`, plus `physics_engine` and
  `rviz`) spawns the combined robot in Gazebo Harmonic with the base on platform-sw's DiffDrive plugin and both arms on
  `gz_ros2_control`; an empty world; platform-sw's `gz_bridge.yaml` used by reference, plus one bridge for the wheels' joint
  states; `joint_state_publisher` merging `/base/joint_states` and `/arm/joint_states` so `/joint_states` has one publisher;
  `odom -> base_link` from the DiffDrive plugin only. The `gazebo` profile loads `gz_ros2_control` into the model (model plugin,
  `gz_ros2_control-system`, with `arm_controllers.yaml` by installed path), names read from the installed library.
- Checks G01 (headless smoke test: five controllers `active`, an arm goal completes, platform-sw's topics unchanged by name and
  type, `/cmd_vel` and `/odom` work, one `odom -> base_link` publisher, `/joint_states` once each) and G02 (grippers close and
  `finger_joint2` follows in the physics). `verify_gazebo.sh` writes a `gazebo` evidence report, `evidence_class: "simulation"`.
- `docs/v01_visual_check.md`: the checklist for V01, to be answered by a person with the simulator and RViz open.
- Check T15 gains the finger-mimic multiplier test (a hard-coded multiplier, including one equal to the yaml's, fails), and the
  plugin tests. T10 gained a tripwire (a verified mass datum with no base/arm provenance entry fails), replaced in P10 by the provenance map below.

- `openamrobot_upperbody_moveit` (A7, part 2): `config/openamrobot.srdf` (the groups, `home` / `hands_up`, end effectors, passive joints and arm-internal
  pairs imported from `enactic/openarm_ros2` @ `4e837e1` with attribution; a fixed `world -> base_link` virtual joint; `park`, a placeholder
  copy of `home`; per-hand gripper `open` / `closed`; the self-collision matrix generated with the Setup Assistant's `collisions_updater` at
  `mast_1350`, 10000 samples, never/adjacent/default on, always off, reproducible with `scripts/regenerate_acm.sh`); `kinematics.yaml` (KDL),
  `pilz_cartesian_limits.yaml` (upstream's, inert), `ompl_planning.yaml` (MoveIt's defaults), `joint_limits.yaml` (our acceleration entries,
  tagged `engineering_default` / `placeholder`, naming manipulation#10 and the upstream file, ADR-009), `moveit_controllers.yaml` for our
  controller names (option 1 of the gripper fork, `GripperCommand`; the controller type is unchanged); `launch/move_group.launch.py`, built with
  `MoveItConfigsBuilder` over the combined xacro and its args.
- Checks M03 (the collision matrix validated by sampling, `scripts/validate_acm.py`), M04 (a goal inside a base-side volume is rejected by
  planning, not merely unsuccessful), and the proposed M05 (per-hand gripper values, the single-number version fails) and M06 (the plans need
  some acceleration limit and do not depend on its value).
- `openamrobot_upperbody_bringup`: `moveit.launch.py` (`mast_id`, `sim` mock|gazebo, `rviz`; `headless`, `physics_engine` for Gazebo), a stub for the
  wheel joints on mock, checks M01 (every named pose on mock at all four mast IDs) and M02 (the same in Gazebo at `mast_1350`, Bullet-Featherstone
  passed explicitly), `run_a7_regression.sh` and `build_a7_evidence.py` (`evidence/<ts>_a7.json`, `evidence_class: "simulation"`, a dirty tree
  writes nothing). M01 and M02 skip unless `OPENAMR_A7=1`, so `verify_a6.sh` does not start them.
- `docs/v01_visual_check.md` has an eighth item (MoveIt: a named pose, preview against execution); V01 stays `WARN`.

### Changed

- The evidence harness writes no report from a dirty tree: it prints the results, says `Evidence: NOT WRITTEN`, and exits 3 if
  nothing failed. Every file in `evidence/` is a clean-commit run.
- C02 checks every pin in `config/upstream_pins.yaml`, not only `openarm_description`, and the file is normalised to quoted
  40-hex hashes (`openarm_ros2` was a 7-character hash nothing compared). A pin with no checkout is listed as not compared; C01
  compares every pin with the manipulation baseline.
- In `ros2_control.xacro` the finger-mimic multiplier property moved from inside the mock/real branch to file scope so it can be
  read and tested; the expansion of every profile (mock, real; gazebo before the plugin was added) is byte-identical.
- Repository README: status updated, and a pointer to the description package. The mast configuration text is unchanged.
- `mast_section` datum: the invented 80 mm square placeholder is now the MISUMI HFS6-60120 profile, 60 x 120 mm (`source: plan`,
  `status: provisional`; which axis carries the 120 mm is unconfirmed). Datums revision `2026-10-08.d`, so every
  configuration id changed.
- `package.xml` descriptions of the bringup, control and MoveIt packages describe the 2.0 fixed-mast scope instead of a lift.
- The control README's note on the gripper controller's deprecation warning now states, in order: what is in use, that it is
  a warning and not a removal, that the replacement serves a different action type, that the warning misspells the
  replacement's name, and that the choice is manipulation#11, ADR-009 point 4 and P10's. The controller type is unchanged.
- Datums revision `2026-10-08.e` (the new F2S datum), so every configuration id changed. C01 no longer compares the pin file
  with a checkout (that is C02); it keeps the format and the baseline-pin comparison.

- `sim.launch.py`'s `physics_engine` defaults to DART again (`gz-physics-dartsim-plugin`), not Bullet-Featherstone: a
  gripper-mimic convenience is not grounds to re-baseline platform-sw's physics. G02 passes `gz-physics-bullet-featherstone-plugin`
  explicitly and its sidecar records which engine each Gazebo check ran on.

- The description no longer resolves `openamrobot_upperbody_control`'s path. The `gazebo` profile's `gz_ros2_control` `<parameters>`
  is the new `controllers_file` xacro arg (default empty, no `<parameters>` element then); `sim.launch.py`, which depends on both
  packages, passes `arm_controllers.yaml`'s installed path. This removes the `$(find)` that the description could neither declare
  (a colcon cycle) nor leave undeclared. The `gazebo` configuration id changes; the other 13 expansions of the matrix are
  byte-identical to before. T15 asserts the element appears with the arg and that the default expansion is valid URDF.

- `config/mass_provenance.yaml`, the mass-provenance map: `placeholder`, `external_nominal` or `verified` for each of the four mass
  groups (base and openarm are `external_nominal`, mast and sensors are `placeholder`, nothing is `verified`). T10 reads the
  caveat off it: `per-link masses still placeholders` when any group is `placeholder`, and a separate line naming the
  `external_nominal` groups. A group that draws its mass from a datum (the mast, `mast_mass`) must agree with the datum's status;
  `verified` needs a `document`. This replaces P09's tripwire, which only guarded the absence of the map. Because the sensors
  carry no inertial (zero mass is a figure chosen so the model expands), that group is a placeholder, so the caveat now needs the
  cameras weighed as well as the mast.

- Gazebo README: the DART finger finding (P10 task 0, a ten-minute check). The position command moves a finger on DART once
  `position_proportional_gain` is raised (0.1 and 1.0: no motion; 5.0: `finger_joint1` reaches the goal and oscillates), so the
  missing finger motion is a gain and not an engine fault; why the threshold is where it is was not isolated, the gain is
  plugin-wide and was not tried on the arms, and `finger_joint2` still does not follow on DART, so G02 stays on
  Bullet-Featherstone. A tuning note for platform-sw's owner; nothing is filed upstream.

### Removed

- `config/lift/`: an empty scaffold directory. An actuated lift is OpenAMRobot 3.0 scope.

### Known limitations

- **M03 FAILS** (A7): the collision matrix disables `openarm_left_ee_link1` / `openarm_left_link2` as `Never` and they collide at sample 1112 of 2000 at every mast ID.
  The pair is not re-enabled. A 20000-sample diagnostic found 14 of the 458 `Never` pairs colliding, all arm-against-arm. The matrix is not certified.
- MoveIt never sees `base_link`'s inertial (W01); `park` is a placeholder copy of `home`; the acceleration limits are engineering defaults, not vendor
  data; the gripper fork is open on manipulation#11 (option 1 is used; option 2 was driven on mock, not in Gazebo); MoveIt 2.12.4 refuses a start state
  1e-7 rad past a joint limit, so planning from `home` in Gazebo is intermittently refused (M01/M02 retry and count); `move_group` exits with SIGSEGV when
  stopped. Everything A7 shows is `simulation` class.

- Base inertials are platform-sw CAD placeholders; the mast mass, mast and plate transforms and all camera poses are
  placeholder datums. No physical evidence.
- `ros2_control` runs on `mock` (L01) and in Gazebo (G01, G02); the `real` block is checked by expansion only,
  `openarm_hardware` cannot be built in the reference workspace (it needs OpenArmCAN). In Gazebo the arms' position interface is
  a velocity servo, so tracking there says nothing about the real arm. `finger_joint2` is not declared under `ros2_control`:
  Gazebo's Bullet-Featherstone engine follows the URDF `<mimic>` on its own, its DART default does not move the fingers at all.
- The Gazebo launch's `physics_engine` defaults to DART, the engine platform-sw tuned the base on, and the grippers do not close
  there. G02 passes Bullet-Featherstone explicitly. The base's contact behaviour was compared on neither engine (G01 measures
  kinematics).
- The gripper controllers stay `position_controllers/GripperActionController` (deprecated in Jazzy) on purpose: the replacement,
  `parallel_gripper_action_controller`, serves `ParallelGripperCommand` instead of `GripperCommand`, so switching decides the
  gripper interface (manipulation#11, ADR-009 point 4), which is open.
- Issue #7: the description strings it names are fixed here; whether that closes it is for the code owners to judge.
